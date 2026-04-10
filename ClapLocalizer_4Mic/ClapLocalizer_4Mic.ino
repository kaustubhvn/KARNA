// =============================================================================
//  ClapLocalizer_4Mic.ino  —  Teensy 4.1  +  4× INMP441
//  14–15 cm square array  |  Direction + XY multilateration
// =============================================================================
//
//  Architecture follows the proven 3-mic reference script (GunshotLocalize.ino):
//    AudioRecordQueue  →  read_full_buffer()  →  DSP buffers
//  Upgraded to 4 mics with:
//    6-pair GCC-PHAT (4 FFTs + 6 IFFTs, each mic FFT computed once)
//    ES-SCFL zero-cyclic-sum validation (729 combos)
//    Weighted LS DoA — overdetermined 6-eq system → most accurate azimuth
//    SRP-PHAT fine refinement (±20° around WLS, fallback full scan)
//    Gauss-Newton XY multilateration → range + position
//    Baseline freeze to prevent double-triggering
//
// ── WIRING ────────────────────────────────────────────────────────────────────
//   ALL 4 MICS:  WS → Pin 20   BCLK → Pin 21   VDD → 3.3V   GND → GND
//   Mic 0  SD → Pin 8   L/R → GND   (I2S1 Left)
//   Mic 1  SD → Pin 8   L/R → 3.3V  (I2S1 Right)
//   Mic 2  SD → Pin 6   L/R → GND   (I2S2 Left)
//   Mic 3  SD → Pin 6   L/R → 3.3V  (I2S2 Right)
//
// ── ARRAY LAYOUT (top view) ──────────────────────────────────────────────────
//
//   M0 ─── M1        +Y = forward (North)
//   |       |        +X = right   (East)
//   |  [0,0]|        Array center = origin
//   M2 ─── M3
//
// ── SERIAL OUTPUT ─────────────────────────────────────────────────────────────
//   Debug:   [DET] ... [GCC] ... [WLS] ... [SRP] ... [GN] ...
//   Result:  EVT,<az_math°>,<x_m>,<y_m>,<range_m>,<conf>,<zcs_µs>,<time_ms>
//   Python radar reads EVT lines.  az_math: 0°=East CCW (atan2 convention).
//
// ── HOW TO TUNE ───────────────────────────────────────────────────────────────
//   1. Flash → open Serial Monitor 115200 baud
//   2. Quiet room: watch [DET] B0 B1 B2 B3 ≈ 1.0
//   3. Clap sharply 1–3 m away: note peak B values
//   4. Set DET_BAND_RATIO ≈ 50% of observed peak
//   5. Verify EVT line appears once per clap (no double-trigger)
// =============================================================================

#include <Arduino.h>
#include <Audio.h>
#include <arm_math.h>

// ── Array geometry ─────────────────────────────────────────────────────────────
// Measure center-to-center of capsules (not PCB edges).
// 14 cm array → 0.070   |   15 cm array → 0.075
#define HALF_M      0.070f

// Speed of sound (m/s) — adjust for room temperature
// 15°C→340.4  20°C→343.4  25°C→346.4  30°C→349.4
#define SOS_MS      343.0f

// ── Detection ─────────────────────────────────────────────────────────────────
// Primary gate:  DET_MIN_MICS_HIT mics must have band-energy ratio > DET_BAND_RATIO
// Secondary gate: mic-0 STE ratio must exceed DET_STE_RATIO
#define DET_BAND_RATIO    8.0f
#define DET_STE_RATIO     4.0f
#define DET_MIN_MICS_HIT  3

#define BAND_LO_HZ    300.0f
#define BAND_HI_HZ   4000.0f

#define DET_ADAPT_BETA  0.01f
#define DET_ALPHA_SLOW  0.995f

#define REARM_MS            450
#define BASELINE_FREEZE_MS  600

// ── GCC-PHAT ──────────────────────────────────────────────────────────────────
#define FFT_SIZE        1024
#define BLOCK_SAMPLES   128
#define BLOCKS_NEEDED   (FFT_SIZE / BLOCK_SAMPLES)
#define GCC_N_PEAKS     3
// Max TDOA: 14cm diagonal=0.198m → 0.198/343×44100≈25.5 → 27
#define TDOA_MAX_SAMP   27
#define ZCS_TOL         4.0e-4f
#define TDOA_SMOOTH     0.20f

// ── DoA and position ──────────────────────────────────────────────────────────
#define SRP_FINE_DEG    20.0f
#define SRP_FINE_STEP    0.5f
#define GN_MAX_ITER     20
#define GN_EPS          1e-5f
#define GN_MAX_STEP     3.0f
#define GN_MAX_RANGE   12.0f

// ── Output ────────────────────────────────────────────────────────────────────
#define SERIAL_BAUD     115200
#define DEBUG_PRINT     1   // 0 = CSV only (cleaner for Python visualizer)

// ── Constants ─────────────────────────────────────────────────────────────────
#define NUM_MICS    4
#define NUM_PAIRS   6
#define FS          ((float)AUDIO_SAMPLE_RATE_EXACT)

static const float MIC_POS[4][2] = {
    { -HALF_M,  HALF_M },   // M0 top-left
    {  HALF_M,  HALF_M },   // M1 top-right
    { -HALF_M, -HALF_M },   // M2 bottom-left
    {  HALF_M, -HALF_M }    // M3 bottom-right
};

static const int8_t PI_[6] = { 0, 0, 0, 1, 1, 2 };
static const int8_t PJ_[6] = { 1, 2, 3, 2, 3, 3 };
static const int8_t ZCS[3][3] = { {0,3,1}, {0,4,2}, {1,5,2} };

// ── Audio pipeline (reference script pattern) ──────────────────────────────────
AudioInputI2SQuad  i2s_quad;
AudioRecordQueue   rec[NUM_MICS];
AudioConnection    pc0(i2s_quad,0,rec[0],0);
AudioConnection    pc1(i2s_quad,1,rec[1],0);
AudioConnection    pc2(i2s_quad,2,rec[2],0);
AudioConnection    pc3(i2s_quad,3,rec[3],0);

// ── DSP buffers ────────────────────────────────────────────────────────────────
static float32_t mic_f[NUM_MICS][FFT_SIZE];  // filled by read_full_buffer
static float32_t hann [FFT_SIZE];
static arm_rfft_fast_instance_f32 fft_inst, det_fft_inst;

#define DET_FFT  256
static float32_t det_in[DET_FFT], det_out[DET_FFT], det_mag[DET_FFT/2];

static float32_t mic_fft[NUM_MICS][FFT_SIZE];  // per-mic FFT (computed once/event)
static float32_t gcc_G  [FFT_SIZE];
static float32_t gcc_rho[NUM_PAIRS][FFT_SIZE];
static float32_t tmp    [FFT_SIZE];

// ── State ──────────────────────────────────────────────────────────────────────
static float    baseline_band[NUM_MICS], baseline_ste;
static bool     baseline_frozen = false;
static uint32_t freeze_until_ms = 0, last_event_ms = 0, event_count = 0;

typedef struct { float tau; float score; } Peak;
static Peak  gcc_peaks [NUM_PAIRS][GCC_N_PEAKS];
static float best_tau  [NUM_PAIRS];
static float smooth_tau[NUM_PAIRS];

// ── Utility ────────────────────────────────────────────────────────────────────
static void build_hann() {
    for (int n=0;n<FFT_SIZE;n++)
        hann[n]=0.5f*(1.0f-cosf(2.0f*(float)M_PI*n/(FFT_SIZE-1)));
}

static const char* compass8(float az) {
    float c=fmodf(90.0f-az+360.0f,360.0f);
    static const char* D[]={"N","NE","E","SE","S","SW","W","NW"};
    return D[(int)((c+22.5f)/45.0f)%8];
}

// Reference script pattern: drain BLOCKS_NEEDED queue blocks into outBuf
static bool read_full_buffer(AudioRecordQueue& q, float32_t* buf) {
    if (q.available() < BLOCKS_NEEDED) return false;
    int off=0;
    for (int b=0;b<BLOCKS_NEEDED;b++) {
        int16_t* blk=(int16_t*)q.readBuffer();
        for (int i=0;i<BLOCK_SAMPLES;i++) buf[off+i]=blk[i]/32768.0f;
        off+=BLOCK_SAMPLES;
        q.freeBuffer();
    }
    return true;
}

// ── Step 1: Detection ──────────────────────────────────────────────────────────
static float det_band_energy(const float32_t* buf) {
    for (int i=0;i<DET_FFT;i++) {
        float w=0.5f*(1.0f-cosf(2.0f*(float)M_PI*i/(DET_FFT-1)));
        det_in[i]=buf[i]*w;
    }
    arm_rfft_fast_f32(&det_fft_inst,det_in,det_out,0);
    arm_cmplx_mag_f32(det_out,det_mag,DET_FFT/2);
    int k_lo=max(1,(int)(BAND_LO_HZ*DET_FFT/FS));
    int k_hi=min(DET_FFT/2-1,(int)(BAND_HI_HZ*DET_FFT/FS));
    float s=0; for(int k=k_lo;k<=k_hi;k++) s+=det_mag[k]; return s;
}

static float det_ste(const float32_t* buf, int n) {
    float s=0; for(int i=0;i<n;i++) s+=buf[i]*buf[i]; return sqrtf(s/n);
}

static bool run_detection() {
    float band_e[NUM_MICS],ste_e[NUM_MICS],ratio[NUM_MICS];
    int hits=0;

    for (int m=0;m<NUM_MICS;m++) {
        float dc=0;
        for(int s=0;s<DET_FFT;s++) dc+=mic_f[m][s];
        dc/=DET_FFT;
        static float32_t dbuf[DET_FFT];
        for(int s=0;s<DET_FFT;s++) dbuf[s]=mic_f[m][s]-dc;

        band_e[m]=det_band_energy(dbuf);
        ste_e[m]=det_ste(mic_f[m],FFT_SIZE);

        if(!baseline_frozen) {
            float r=band_e[m]/(baseline_band[m]+1e-12f);
            float beta=(r<2.0f)?DET_ADAPT_BETA:(1.0f-DET_ALPHA_SLOW);
            baseline_band[m]=(1.0f-beta)*baseline_band[m]+beta*band_e[m];
        }
        ratio[m]=band_e[m]/(baseline_band[m]+1e-12f);
        if(ratio[m]>DET_BAND_RATIO) hits++;
    }

    if(!baseline_frozen) {
        float r0=ste_e[0]/(baseline_ste+1e-12f);
        float beta=(r0<2.0f)?DET_ADAPT_BETA:(1.0f-DET_ALPHA_SLOW);
        baseline_ste=(1.0f-beta)*baseline_ste+beta*ste_e[0];
    }
    float ste_r=ste_e[0]/(baseline_ste+1e-12f);

    if(baseline_frozen && millis()>=freeze_until_ms) {
        baseline_frozen=false;
        for(int m=0;m<NUM_MICS;m++) baseline_band[m]=band_e[m];
        baseline_ste=ste_e[0];
    }

    #if DEBUG_PRINT
    Serial.print(F("[DET] B0=")); Serial.print(ratio[0],1);
    Serial.print(F(" B1="));     Serial.print(ratio[1],1);
    Serial.print(F(" B2="));     Serial.print(ratio[2],1);
    Serial.print(F(" B3="));     Serial.print(ratio[3],1);
    Serial.print(F("  STE="));   Serial.print(ste_r,1);
    Serial.print(F("  Hit="));   Serial.print(hits);
    bool w=!baseline_frozen&&(millis()-last_event_ms)>=REARM_MS
          &&hits>=DET_MIN_MICS_HIT&&ste_r>DET_STE_RATIO;
    if(w) Serial.print(F("  *** TRIGGER ***"));
    Serial.println();
    #endif

    bool trig=!baseline_frozen&&(millis()-last_event_ms)>=REARM_MS
             &&hits>=DET_MIN_MICS_HIT&&ste_r>DET_STE_RATIO;
    if(trig){last_event_ms=millis();baseline_frozen=true;freeze_until_ms=millis()+BASELINE_FREEZE_MS;}
    return trig;
}

// ── Step 2: GCC-PHAT ───────────────────────────────────────────────────────────
static void compute_mic_ffts() {
    int k_lo=max(1,(int)(BAND_LO_HZ*FFT_SIZE/FS));
    int k_hi=min(FFT_SIZE/2-1,(int)(BAND_HI_HZ*FFT_SIZE/FS));
    for(int m=0;m<NUM_MICS;m++) {
        float dc=0;
        for(int s=0;s<FFT_SIZE;s++) dc+=mic_f[m][s]; dc/=FFT_SIZE;
        for(int s=0;s<FFT_SIZE;s++) tmp[s]=(mic_f[m][s]-dc)*hann[s];
        arm_rfft_fast_f32(&fft_inst,tmp,mic_fft[m],0);
        for(int k=0;k<k_lo;k++){mic_fft[m][2*k]=0;mic_fft[m][2*k+1]=0;}
        for(int k=k_hi+1;k<FFT_SIZE/2;k++){mic_fft[m][2*k]=0;mic_fft[m][2*k+1]=0;}
    }
}

static void gcc_pair(int pi,int pj,int idx) {
    const float eps=1e-10f;
    gcc_G[0]=gcc_G[1]=0;
    for(int k=1;k<FFT_SIZE/2;k++) {
        float Ar=mic_fft[pi][2*k],Ai=mic_fft[pi][2*k+1];
        float Br=mic_fft[pj][2*k],Bi=mic_fft[pj][2*k+1];
        float Gr=Ar*Br+Ai*Bi, Gi=Ai*Br-Ar*Bi;
        float mg=sqrtf(Gr*Gr+Gi*Gi)+eps;
        gcc_G[2*k]=Gr/mg; gcc_G[2*k+1]=Gi/mg;
    }
    arm_rfft_fast_f32(&fft_inst,gcc_G,gcc_rho[idx],1);

    Peak* pk=gcc_peaks[idx];
    for(int p=0;p<GCC_N_PEAKS;p++){pk[p].tau=0;pk[p].score=-1e9f;}

    auto ins=[&](int li,float v){
        if(v<=pk[GCC_N_PEAKS-1].score) return;
        float y0=gcc_rho[idx][(li-1+FFT_SIZE)%FFT_SIZE];
        float y1=v,y2=gcc_rho[idx][(li+1)%FFT_SIZE];
        float den=y0-2.0f*y1+y2;
        float dlt=(fabsf(den)>1e-10f)?constrain(0.5f*(y0-y2)/den,-0.5f,0.5f):0.0f;
        int sl=(li>FFT_SIZE/2)?li-FFT_SIZE:li;
        pk[GCC_N_PEAKS-1]={((float)sl+dlt)/FS,v};
        for(int p=GCC_N_PEAKS-1;p>0&&pk[p].score>pk[p-1].score;p--){Peak t=pk[p];pk[p]=pk[p-1];pk[p-1]=t;}
    };

    for(int l=1;l<=TDOA_MAX_SAMP&&l<FFT_SIZE-1;l++){float v=gcc_rho[idx][l];if(v>gcc_rho[idx][l-1]&&v>gcc_rho[idx][l+1])ins(l,v);}
    for(int l=FFT_SIZE-TDOA_MAX_SAMP;l<FFT_SIZE-1;l++){if(l<1)continue;float v=gcc_rho[idx][l];if(v>gcc_rho[idx][l-1]&&v>gcc_rho[idx][l+1])ins(l,v);}
    ins(0,gcc_rho[idx][0]);
}

static void run_gcc_phat(){compute_mic_ffts();for(int p=0;p<NUM_PAIRS;p++)gcc_pair(PI_[p],PJ_[p],p);}

// ── Step 3: ES-SCFL ────────────────────────────────────────────────────────────
static bool run_escfl(float& zcs_us) {
    float best=1e9f; int bc[NUM_PAIRS]={},combo[NUM_PAIRS]={};
    for(int c=0;c<729;c++){
        float t[NUM_PAIRS],zcs=0,sc=0;
        for(int p=0;p<NUM_PAIRS;p++) t[p]=gcc_peaks[p][combo[p]].tau;
        for(int l=0;l<3;l++) zcs+=fabsf(t[ZCS[l][0]]+t[ZCS[l][1]]-t[ZCS[l][2]]);
        for(int p=0;p<NUM_PAIRS;p++) sc+=gcc_peaks[p][combo[p]].score;
        float s=zcs-3e-4f*sc;
        if(s<best){best=s;memcpy(bc,combo,sizeof(bc));}
        for(int p=NUM_PAIRS-1;p>=0;p--)if(++combo[p]<GCC_N_PEAKS)break;else combo[p]=0;
    }
    for(int p=0;p<NUM_PAIRS;p++) best_tau[p]=gcc_peaks[p][bc[p]].tau;
    for(int p=0;p<NUM_PAIRS;p++) smooth_tau[p]=TDOA_SMOOTH*best_tau[p]+(1.0f-TDOA_SMOOTH)*smooth_tau[p];
    float zcs=0;
    for(int l=0;l<3;l++) zcs+=fabsf(smooth_tau[ZCS[l][0]]+smooth_tau[ZCS[l][1]]-smooth_tau[ZCS[l][2]]);
    zcs_us=zcs*1e6f;
    bool v=(zcs<ZCS_TOL);
    #if DEBUG_PRINT
    Serial.print(F("[GCC] ZCS=")); Serial.print(zcs_us,1); Serial.print(F("µs  ")); Serial.println(v?"CONSISTENT":"UNCERTAIN");
    Serial.print(F("[GCC] TDOAs(µs):"));
    for(int p=0;p<NUM_PAIRS;p++){Serial.print(" ");Serial.print(smooth_tau[p]*1e6f,1);}
    Serial.println();
    #endif
    return v;
}

// ── Step 4: Weighted LS DoA (6-equation overdetermined system) ─────────────────
static float run_wls_doa(float& conf) {
    float A00=0,A01=0,A11=0,b0=0,b1=0;
    float tau_max=(float)TDOA_MAX_SAMP/FS;
    for(int p=0;p<NUM_PAIRS;p++){
        float dx=MIC_POS[PJ_[p]][0]-MIC_POS[PI_[p]][0];
        float dy=MIC_POS[PJ_[p]][1]-MIC_POS[PI_[p]][1];
        float bv=SOS_MS*smooth_tau[p];
        float w=max(0.05f,gcc_peaks[p][0].score)*constrain(fabsf(smooth_tau[p])/(tau_max*0.3f),0.1f,1.0f);
        A00+=w*dx*dx; A01+=w*dx*dy; A11+=w*dy*dy;
        b0 +=w*dx*bv; b1 +=w*dy*bv;
    }
    float det=A00*A11-A01*A01+1e-14f;
    float ux=(A11*b0-A01*b1)/det, uy=(-A01*b0+A00*b1)/det;
    float n=hypotf(ux,uy); if(n<1e-6f){conf=0;return 0;} ux/=n;uy/=n;
    float resid=0;
    for(int p=0;p<NUM_PAIRS;p++){
        float dx=MIC_POS[PJ_[p]][0]-MIC_POS[PI_[p]][0];
        float dy=MIC_POS[PJ_[p]][1]-MIC_POS[PI_[p]][1];
        resid+=fabsf((dx*ux+dy*uy)/SOS_MS-smooth_tau[p]);
    }
    conf=constrain(1.0f-resid/(NUM_PAIRS*tau_max*0.5f),0.0f,1.0f);
    float az=atan2f(uy,ux)*180.0f/(float)M_PI;
    #if DEBUG_PRINT
    Serial.print(F("[WLS] Az=")); Serial.print(az,1); Serial.print(F("° (")); Serial.print(compass8(az)); Serial.print(F(")  conf=")); Serial.println(conf,3);
    #endif
    return az;
}

// ── Step 5: SRP fine refinement ────────────────────────────────────────────────
static float srp_lookup(int p,float t){
    float lag=t*FS; if(fabsf(lag)>TDOA_MAX_SAMP+1)return 0.0f;
    float idf=(lag>=0)?lag:(lag+FFT_SIZE);
    int i0=(int)idf%FFT_SIZE,i1=(i0+1)%FFT_SIZE; float fr=idf-floorf(idf);
    return gcc_rho[p][i0]*(1.0f-fr)+gcc_rho[p][i1]*fr;
}

static float srp_eval(float az){
    float ar=az*(float)M_PI/180.0f,ux=cosf(ar),uy=sinf(ar),s=0;
    for(int p=0;p<NUM_PAIRS;p++){float dx=MIC_POS[PJ_[p]][0]-MIC_POS[PI_[p]][0],dy=MIC_POS[PJ_[p]][1]-MIC_POS[PI_[p]][1];s+=srp_lookup(p,(dx*ux+dy*uy)/SOS_MS);}
    return s;
}

static float run_srp(float wls_az,float wls_conf,float& peak){
    float best=-1e9f,baz=wls_az;
    if(wls_conf>=0.25f){for(float a=wls_az-SRP_FINE_DEG;a<=wls_az+SRP_FINE_DEG;a+=SRP_FINE_STEP){float s=srp_eval(a);if(s>best){best=s;baz=a;}}}
    if(best<0||wls_conf<0.25f){for(int a=0;a<360;a+=5){float s=srp_eval((float)a);if(s>best){best=s;baz=(float)a;}}}
    peak=best; baz=fmodf(baz+360.0f,360.0f);
    #if DEBUG_PRINT
    Serial.print(F("[SRP] Az=")); Serial.print(baz,1); Serial.print(F("° (")); Serial.print(compass8(baz)); Serial.print(F(")  peak=")); Serial.println(peak,4);
    #endif
    return baz;
}

// ── Step 6: Combine azimuth ────────────────────────────────────────────────────
static float combine_az(float waz,float wc,float saz,float sp){
    float sc=constrain(sp/0.25f,0.0f,1.0f)*0.7f;
    float tot=wc+sc+1e-9f;
    float ux=wc*cosf(waz*(float)M_PI/180.0f)+sc*cosf(saz*(float)M_PI/180.0f);
    float uy=wc*sinf(waz*(float)M_PI/180.0f)+sc*sinf(saz*(float)M_PI/180.0f);
    return fmodf(atan2f(uy/tot,ux/tot)*180.0f/(float)M_PI+360.0f,360.0f);
}

// ── Step 7: Gauss-Newton XY ────────────────────────────────────────────────────
static bool run_gn(float az_seed,float& ox,float& oy,float& orange,float& oc){
    float ar=az_seed*(float)M_PI/180.0f,x=2.0f*cosf(ar),y=2.0f*sinf(ar);
    for(int it=0;it<GN_MAX_ITER;it++){
        float J00=0,J01=0,J11=0,r0=0,r1=0;
        for(int p=0;p<NUM_PAIRS;p++){
            int mi=PI_[p],mj=PJ_[p];
            float dxi=x-MIC_POS[mi][0],dyi=y-MIC_POS[mi][1];
            float dxj=x-MIC_POS[mj][0],dyj=y-MIC_POS[mj][1];
            float di=sqrtf(dxi*dxi+dyi*dyi)+1e-9f,dj=sqrtf(dxj*dxj+dyj*dyj)+1e-9f;
            float rv=(di-dj)-SOS_MS*smooth_tau[p];
            float jx=dxi/di-dxj/dj,jy=dyi/di-dyj/dj;
            J00+=jx*jx;J01+=jx*jy;J11+=jy*jy;r0+=jx*rv;r1+=jy*rv;
        }
        float dt=J00*J11-J01*J01+1e-14f;
        float dx=-(J11*r0-J01*r1)/dt,dy=-(-J01*r0+J00*r1)/dt;
        float st=sqrtf(dx*dx+dy*dy);
        if(st>GN_MAX_STEP){dx*=GN_MAX_STEP/st;dy*=GN_MAX_STEP/st;}
        x+=dx;y+=dy; if(st<GN_EPS)break;
    }
    float rng=sqrtf(x*x+y*y); bool ok=(rng>0.05f)&&(rng<GN_MAX_RANGE);
    float fit=0;
    for(int p=0;p<NUM_PAIRS;p++){
        int mi=PI_[p],mj=PJ_[p];
        float di=sqrtf((x-MIC_POS[mi][0])*(x-MIC_POS[mi][0])+(y-MIC_POS[mi][1])*(y-MIC_POS[mi][1]))+1e-9f;
        float dj=sqrtf((x-MIC_POS[mj][0])*(x-MIC_POS[mj][0])+(y-MIC_POS[mj][1])*(y-MIC_POS[mj][1]))+1e-9f;
        float rv=(di-dj)-SOS_MS*smooth_tau[p]; fit+=rv*rv;
    }
    oc=ok?constrain(1.0f-sqrtf(fit)/0.08f,0.0f,1.0f):0.0f;
    ox=x;oy=y;orange=rng;
    #if DEBUG_PRINT
    Serial.print(F("[GN]  X=")); Serial.print(x,2); Serial.print(F("m  Y=")); Serial.print(y,2); Serial.print(F("m  R=")); Serial.print(rng,1); Serial.print(F("m  Conf=")); Serial.println(oc,2);
    #endif
    return ok;
}

// ── Output ─────────────────────────────────────────────────────────────────────
static void print_human(float az,float x,float y,float r,float c,float zus,bool zok){
    float comp=fmodf(90.0f-az+360.0f,360.0f);
    Serial.println(F("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"));
    Serial.print(F("  Azimuth: ")); Serial.print(comp,1); Serial.print(F("° (")); Serial.print(compass8(az)); Serial.println(F(")"));
    Serial.print(F("  X,Y    : ")); Serial.print(x,2); Serial.print(F("m, ")); Serial.print(y,2); Serial.println(F("m"));
    Serial.print(F("  Range  : ")); Serial.print(r,1); Serial.println(F("m"));
    Serial.print(F("  Conf   : ")); Serial.print(c*100.0f,0); Serial.println(F("%"));
    Serial.print(F("  ZCS    : ")); Serial.print(zus,1); Serial.println(zok?F("µs ✓"):F("µs (uncertain)"));
    Serial.println(F("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"));
}

static void emit_csv(float az,float x,float y,float r,float c,float zus){
    Serial.print(F("EVT,"));
    Serial.print(az,2);Serial.print(',');Serial.print(x,3);Serial.print(',');
    Serial.print(y,3);Serial.print(',');Serial.print(r,2);Serial.print(',');
    Serial.print(c,3);Serial.print(',');Serial.print(zus,1);Serial.print(',');
    Serial.println(millis());
}

// ── Setup ──────────────────────────────────────────────────────────────────────
void setup(){
    Serial.begin(SERIAL_BAUD);
    while(!Serial && millis()<5000); delay(500);

    Serial.println(F("\n╔══════════════════════════════════════════╗"));
    Serial.println(F(  "║  4-Mic Clap Localizer  —  Teensy 4.1   ║"));
    Serial.println(F(  "╚══════════════════════════════════════════╝"));
    Serial.print(F("[CFG] Array: ")); Serial.print(HALF_M*200.0f,0); Serial.println(F(" cm square"));
    Serial.print(F("[CFG] SOS  : ")); Serial.print(SOS_MS,1); Serial.println(F(" m/s"));
    Serial.print(F("[CFG] Band : ")); Serial.print(BAND_LO_HZ,0); Serial.print(F("-")); Serial.print(BAND_HI_HZ,0); Serial.println(F(" Hz"));
    Serial.print(F("[CFG] Det  : BandRatio>")); Serial.print(DET_BAND_RATIO,1); Serial.print(F(" STE>")); Serial.print(DET_STE_RATIO,1); Serial.print(F(" MinMics=")); Serial.println(DET_MIN_MICS_HIT);
    Serial.print(F("[CFG] Rearm: ")); Serial.print(REARM_MS); Serial.print(F("ms  Freeze:")); Serial.print(BASELINE_FREEZE_MS); Serial.println(F("ms"));
    Serial.println(F("[CFG] DoA  : WLS(6-eq) + SRP-PHAT fine + Gauss-Newton XY"));
    Serial.println();

    AudioMemory(40);
    arm_rfft_fast_init_f32(&fft_inst,FFT_SIZE);
    arm_rfft_fast_init_f32(&det_fft_inst,DET_FFT);
    build_hann();
    for(int m=0;m<NUM_MICS;m++) baseline_band[m]=1e-6f;
    baseline_ste=1e-6f;
    memset(smooth_tau,0,sizeof(smooth_tau));
    for(int m=0;m<NUM_MICS;m++) rec[m].begin();

    Serial.print(F("[SYS] Fs     = ")); Serial.print(FS,2); Serial.println(F(" Hz"));
    Serial.println(F("[SYS] CSV: EVT,az_math°,x_m,y_m,range_m,conf,zcs_µs,time_ms"));
    Serial.println(F("[SYS] Ready.\n"));
}

// ── Main loop (reference script pattern) ───────────────────────────────────────
void loop(){
    // Wait until all 4 queues have 8 full blocks (= FFT_SIZE samples each)
    for(int m=0;m<NUM_MICS;m++) if(rec[m].available()<BLOCKS_NEEDED){delay(1);return;}

    // Fill mic_f[][] — same pattern as reference script's read_full_buffer
    for(int m=0;m<NUM_MICS;m++) if(!read_full_buffer(rec[m],mic_f[m])) return;

    // DC removal (reference script pattern)
    for(int m=0;m<NUM_MICS;m++){
        float mean=0;
        for(int s=0;s<FFT_SIZE;s++) mean+=mic_f[m][s]; mean/=FFT_SIZE;
        for(int s=0;s<FFT_SIZE;s++) mic_f[m][s]-=mean;
    }

    if(!run_detection()) return;

    event_count++;
    Serial.print(F("\n[EVT] #")); Serial.println(event_count);

    run_gcc_phat();

    float zcs_us; bool zcs_ok=run_escfl(zcs_us);
    float wls_conf,wls_az=run_wls_doa(wls_conf);
    float srp_peak,srp_az=run_srp(wls_az,wls_conf,srp_peak);
    float az=combine_az(wls_az,wls_conf,srp_az,srp_peak);

    float x,y,range,ml_conf; bool ml_ok=run_gn(az,x,y,range,ml_conf);
    if(!ml_ok){x=0;y=0;range=0;ml_conf=0;}

    float conf=(wls_conf+constrain(srp_peak/0.25f,0.0f,1.0f)*0.7f+ml_conf)/3.0f;
    if(!zcs_ok) conf*=0.5f;

    print_human(az,x,y,range,conf,zcs_us,zcs_ok);
    emit_csv(az,x,y,range,conf,zcs_us);
}
