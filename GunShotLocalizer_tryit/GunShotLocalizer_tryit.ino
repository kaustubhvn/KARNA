// =============================================================================
//  GunShotLocalizer.ino   —   Teensy 4.1  +  4× INMP441  (square array)
//  Algorithm : GCC-PHAT  →  ES-SCFL  →  SRP-PHAT azimuth
// =============================================================================
//
// ── WIRING ────────────────────────────────────────────────────────────────────
//   ALL FOUR MICS share:
//     WS  (LRCLK) → Teensy Pin 20
//     SCK (BCLK)  → Teensy Pin 21
//     VDD → 3.3 V,  GND → GND
//
//   Mic 0 (FL)  SD → Pin 8   L/R → GND    (I2S1 Left)
//   Mic 1 (FR)  SD → Pin 8   L/R → 3.3V   (I2S1 Right)
//   Mic 2 (BL)  SD → Pin 6   L/R → GND    (I2S2 Left)
//   Mic 3 (BR)  SD → Pin 6   L/R → 3.3V   (I2S2 Right)
//
// ── ARRAY LAYOUT (top-view) ───────────────────────────────────────────────────
//
//        NORTH (forward)
//           ↑
//   M0(FL) ─── M1(FR)
//      |    |    |
//   M2(BL) ─── M3(BR)
//
//   In code coordinates:
//     +Y = NORTH   (M0/M1 side)
//     +X = EAST    (M1/M3 side)
//
// ── HOW AZIMUTH IS DEFINED ────────────────────────────────────────────────────
//   0°   = NORTH  (sound comes from in front of the M0/M1 edge)
//   90°  = EAST   (sound comes from the M1/M3 side)
//   180° = SOUTH
//   270° = WEST   (sound comes from the M0/M2 side)
//
//   SRP-PHAT sweeps a unit direction vector (ux, uy) across all azimuths.
//   For compass azimuth θ:
//     ux = sin(θ)   ← East component
//     uy = cos(θ)   ← North component
//   This is the standard compass→XY convention (NOT math convention).
//
// ── WHY EAST/WEST WAS SWAPPED IN THE PREVIOUS VERSION ────────────────────────
//   The old code used ux=cos(az), uy=sin(az) which is MATH convention
//   (0° = East, CCW positive). Combined with the physical array orientation
//   this caused a left-right mirror. Fixed below by using sin/cos swap
//   AND keeping +X = East in MIC_POS.
//
// ── UART OUTPUT (Serial3, Pin 14 TX) → ESP32-S3 Hub ──────────────────────────
//   "GS,<mic>,<angle_deg>,<direction_str>\n"
//
// ── CALIBRATION ───────────────────────────────────────────────────────────────
//   If results are still rotated after flashing, adjust ANGLE_OFFSET_DEG.
//   Shoot a known-direction sound (e.g. clap directly NORTH = in front of
//   M0/M1 edge). If display shows 30° instead of 0°, set ANGLE_OFFSET_DEG -30.
//   Range: -180 to +180.
// =============================================================================

#include <Arduino.h>
#include <Audio.h>
#include <arm_math.h>

// =============================================================================
// ── CONFIGURATION ─────────────────────────────────────────────────────────────
// =============================================================================

#define HALF_M        0.07f       // array half-side (m), 14 cm square

// ── Azimuth correction ────────────────────────────────────────────────────────
// Set to 0 first. If output is consistently off, adjust in 90° steps first,
// then fine-tune. Positive = rotate output clockwise.
#define ANGLE_OFFSET_DEG   0.0f

// ── Detection ─────────────────────────────────────────────────────────────────
#define DET_BAND_RATIO   6.0f    // raise if false triggers, lower if misses
#define DET_STE_RATIO    4.0f
#define DET_MIN_MICS_HIT 3
#define BAND_LO_HZ     200.0f   // gunshots have energy from ~200 Hz
#define BAND_HI_HZ    4000.0f
#define DET_ALPHA      0.993f
#define REARM_MS        600
#define BASELINE_FREEZE_MS 700

// ── GCC-PHAT ──────────────────────────────────────────────────────────────────
#define GCC_FFT_SIZE   1024
#define GCC_N_PEAKS    3
#define TDOA_MAX_SAMP  27
#define ZCS_TOL        4.0e-4f
#define TDOA_SMOOTH    0.15f

// ── UART ──────────────────────────────────────────────────────────────────────
#define HUB_BAUD       115200
#define SERIAL_BAUD    115200
#define DEBUG_DET      0         // set 1 to print [DET] lines for tuning

// =============================================================================
// ── FIXED CONSTANTS ───────────────────────────────────────────────────────────
// =============================================================================

#define SOS_MS        343.0f
#define NUM_MICS      4
#define NUM_PAIRS     6
#define BLOCK_SAMP    128
#define FS            ((float)AUDIO_SAMPLE_RATE_EXACT)

#define PRE_MS      200
#define PRE_SAMP    ((int)(FS * PRE_MS / 1000.0f))
#define RING_SAMP   (((PRE_SAMP + GCC_FFT_SIZE + BLOCK_SAMP - 1) \
                      / BLOCK_SAMP + 4) * BLOCK_SAMP)

// ── Mic positions ─────────────────────────────────────────────────────────────
// +X = East (FR/BR side),  +Y = North (FL/FR side)
// This MUST match the physical layout above.
static const float MIC_POS[4][2] = {
    { -HALF_M, +HALF_M },   // M0 FL  (West,  North)
    { +HALF_M, +HALF_M },   // M1 FR  (East,  North)
    { -HALF_M, -HALF_M },   // M2 BL  (West,  South)
    { +HALF_M, -HALF_M },   // M3 BR  (East,  South)
};
static const char* MIC_NAME[4] = {"FL","FR","BL","BR"};

static const int8_t PI_[NUM_PAIRS] = {0, 0, 0, 1, 1, 2};
static const int8_t PJ_[NUM_PAIRS] = {1, 2, 3, 2, 3, 3};
static const int8_t ZCS[3][3]      = { {0,3,1}, {0,4,2}, {1,5,2} };

// =============================================================================
// ── AUDIO PIPELINE ────────────────────────────────────────────────────────────
// =============================================================================

AudioInputI2SQuad  quad;
AudioRecordQueue   Q[NUM_MICS];
AudioConnection    C0(quad,0,Q[0],0);
AudioConnection    C1(quad,1,Q[1],0);
AudioConnection    C2(quad,2,Q[2],0);
AudioConnection    C3(quad,3,Q[3],0);

// =============================================================================
// ── BUFFERS & STATE ───────────────────────────────────────────────────────────
// =============================================================================

static int16_t  ring[NUM_MICS][RING_SAMP];
static uint32_t ring_head = 0;

static arm_rfft_fast_instance_f32 fft_inst;
static arm_rfft_fast_instance_f32 det_fft_inst;
static float32_t hann_win[GCC_FFT_SIZE];

#define DET_FFT 256
static float32_t det_in [DET_FFT];
static float32_t det_out[DET_FFT];
static float32_t det_mag[DET_FFT / 2];

static float32_t gcc_fA [GCC_FFT_SIZE];
static float32_t gcc_fB [GCC_FFT_SIZE];
static float32_t gcc_G  [GCC_FFT_SIZE];
static float32_t gcc_rho[NUM_PAIRS][GCC_FFT_SIZE];
static float32_t mic_win[NUM_MICS][GCC_FFT_SIZE];
static float32_t fwin   [GCC_FFT_SIZE];

static float    baseline_band[NUM_MICS];
static float    baseline_ste;
static bool     baseline_frozen = false;
static uint32_t freeze_until_ms = 0;
static uint32_t last_event_ms   = 0;
static uint32_t event_count     = 0;

typedef struct { float tau; float score; } Peak;
static Peak  gcc_peaks [NUM_PAIRS][GCC_N_PEAKS];
static float smooth_tau[NUM_PAIRS];
static float best_tau  [NUM_PAIRS];

// =============================================================================
// ── UTILITIES ─────────────────────────────────────────────────────────────────
// =============================================================================

static void build_hann() {
    for (int n = 0; n < GCC_FFT_SIZE; n++)
        hann_win[n] = 0.5f*(1.0f - cosf(2.0f*(float)M_PI*n/(GCC_FFT_SIZE-1)));
}

static const char* angle_to_compass(float az) {
    // az is compass degrees: 0=N, 90=E, 180=S, 270=W
    az = fmodf(az + 360.0f, 360.0f);
    if (az < 22.5f  || az >= 337.5f) return "NORTH";
    if (az < 67.5f)                   return "NORTH-EAST";
    if (az < 112.5f)                  return "EAST";
    if (az < 157.5f)                  return "SOUTH-EAST";
    if (az < 202.5f)                  return "SOUTH";
    if (az < 247.5f)                  return "SOUTH-WEST";
    if (az < 292.5f)                  return "WEST";
    return                                   "NORTH-WEST";
}

static int find_closest_mic() {
    float best = -1.0f; int winner = 0;
    for (int m = 0; m < NUM_MICS; m++) {
        int32_t st = ((int32_t)ring_head - DET_FFT
                      + (int32_t)RING_SAMP*4) % (int32_t)RING_SAMP;
        float pk = 0;
        for (int s = 0; s < DET_FFT; s++) {
            float v = fabsf(ring[m][(st+s) % RING_SAMP] / 32768.0f);
            if (v > pk) pk = v;
        }
        if (pk > best) { best = pk; winner = m; }
    }
    return winner;
}

static void ring_extract(int m, int offset, float32_t* dst, int n) {
    int32_t end   = (int32_t)ring_head - offset;
    int32_t start = end - n;
    for (int s = 0; s < n; s++) {
        int32_t idx = ((start+s) % (int32_t)RING_SAMP + RING_SAMP) % RING_SAMP;
        dst[s] = ring[m][idx] / 32768.0f;
    }
}

// =============================================================================
// ── STEP 1: CAPTURE ───────────────────────────────────────────────────────────
// =============================================================================

static bool capture_block() {
    for (int m = 0; m < NUM_MICS; m++)
        if (Q[m].available() < 1) return false;
    int16_t* blk[NUM_MICS];
    for (int m = 0; m < NUM_MICS; m++) blk[m] = (int16_t*)Q[m].readBuffer();
    for (int s = 0; s < BLOCK_SAMP; s++) {
        uint32_t wi = (ring_head + s) % RING_SAMP;
        for (int m = 0; m < NUM_MICS; m++) ring[m][wi] = blk[m][s];
    }
    for (int m = 0; m < NUM_MICS; m++) Q[m].freeBuffer();
    ring_head = (ring_head + BLOCK_SAMP) % RING_SAMP;
    return true;
}

// =============================================================================
// ── STEP 2: DETECTION ─────────────────────────────────────────────────────────
// =============================================================================

static float det_band_energy(const float32_t* buf) {
    for (int i = 0; i < DET_FFT; i++) {
        float w = 0.5f*(1.0f - cosf(2.0f*(float)M_PI*i/(DET_FFT-1)));
        det_in[i] = buf[i] * w;
    }
    arm_rfft_fast_f32(&det_fft_inst, det_in, det_out, 0);
    arm_cmplx_mag_f32(det_out, det_mag, DET_FFT/2);
    int k_lo = max(1,           (int)(BAND_LO_HZ * DET_FFT / FS));
    int k_hi = min(DET_FFT/2-1, (int)(BAND_HI_HZ * DET_FFT / FS));
    float s = 0;
    for (int k = k_lo; k <= k_hi; k++) s += det_mag[k];
    return s;
}

static float det_ste(const float32_t* buf, int n) {
    float s = 0;
    for (int i = 0; i < n; i++) s += buf[i]*buf[i];
    return sqrtf(s/n);
}

static bool run_detection() {
    static float32_t dbuf[DET_FFT];
    float band_e[NUM_MICS], ste_e[NUM_MICS], ratio[NUM_MICS];
    int hits = 0;

    for (int m = 0; m < NUM_MICS; m++) {
        int32_t start = ((int32_t)ring_head - DET_FFT
                         + (int32_t)RING_SAMP*4) % (int32_t)RING_SAMP;
        for (int s = 0; s < DET_FFT; s++)
            dbuf[s] = ring[m][(start+s) % RING_SAMP] / 32768.0f;

        float dc = 0;
        for (int s = 0; s < DET_FFT; s++) dc += dbuf[s];
        dc /= DET_FFT;
        for (int s = 0; s < DET_FFT; s++) dbuf[s] -= dc;

        band_e[m] = det_band_energy(dbuf);
        ste_e[m]  = det_ste(dbuf, DET_FFT);

        if (!baseline_frozen)
            baseline_band[m] = DET_ALPHA*baseline_band[m]
                              + (1.0f-DET_ALPHA)*band_e[m];

        ratio[m] = band_e[m] / (baseline_band[m] + 1e-12f);
        if (ratio[m] > DET_BAND_RATIO) hits++;
    }

    if (!baseline_frozen)
        baseline_ste = DET_ALPHA*baseline_ste + (1.0f-DET_ALPHA)*ste_e[0];
    float ste_ratio = ste_e[0] / (baseline_ste + 1e-12f);

    if (baseline_frozen && millis() >= freeze_until_ms) {
        baseline_frozen = false;
        for (int m = 0; m < NUM_MICS; m++) baseline_band[m] = band_e[m];
        baseline_ste = ste_e[0];
    }

    #if DEBUG_DET
    Serial.print("[DET]");
    for (int m = 0; m < NUM_MICS; m++) {
        Serial.print(" B"); Serial.print(m); Serial.print("=");
        Serial.print(ratio[m], 1);
    }
    Serial.print(" STE="); Serial.print(ste_ratio, 1);
    Serial.print(" Hit="); Serial.println(hits);
    #endif

    bool ok = !baseline_frozen
           && (millis() - last_event_ms) >= (uint32_t)REARM_MS
           && hits >= DET_MIN_MICS_HIT
           && ste_ratio > DET_STE_RATIO;

    if (ok) {
        last_event_ms   = millis();
        baseline_frozen = true;
        freeze_until_ms = millis() + BASELINE_FREEZE_MS;
    }
    return ok;
}

// =============================================================================
// ── STEP 3: GCC-PHAT ──────────────────────────────────────────────────────────
// =============================================================================

static void gcc_pair(int pi, int pj, int idx) {
    const float eps = 1e-10f;
    int k_lo = max(1,               (int)(BAND_LO_HZ * GCC_FFT_SIZE / FS));
    int k_hi = min(GCC_FFT_SIZE/2-1,(int)(BAND_HI_HZ * GCC_FFT_SIZE / FS));

    arm_rfft_fast_f32(&fft_inst, mic_win[pi], gcc_fA, 0);
    arm_rfft_fast_f32(&fft_inst, mic_win[pj], gcc_fB, 0);

    for (int k = 0; k < GCC_FFT_SIZE; k++) gcc_G[k] = 0.0f;
    for (int k = k_lo; k <= k_hi; k++) {
        float Ar=gcc_fA[2*k], Ai=gcc_fA[2*k+1];
        float Br=gcc_fB[2*k], Bi=gcc_fB[2*k+1];
        float Gr=Ar*Br+Ai*Bi, Gi=Ai*Br-Ar*Bi;
        float mg=sqrtf(Gr*Gr+Gi*Gi)+eps;
        gcc_G[2*k]=Gr/mg; gcc_G[2*k+1]=Gi/mg;
    }
    arm_rfft_fast_f32(&fft_inst, gcc_G, gcc_rho[idx], 1);

    Peak* pk = gcc_peaks[idx];
    for (int p = 0; p < GCC_N_PEAKS; p++) { pk[p].tau=0; pk[p].score=-1e9f; }

    auto ins = [&](int li, float v) {
        if (v <= pk[GCC_N_PEAKS-1].score) return;
        float y0=gcc_rho[idx][(li-1+GCC_FFT_SIZE)%GCC_FFT_SIZE];
        float y1=v, y2=gcc_rho[idx][(li+1)%GCC_FFT_SIZE];
        float den=y0-2.0f*y1+y2;
        float dlt=(fabsf(den)>1e-10f)?constrain(0.5f*(y0-y2)/den,-0.5f,0.5f):0.0f;
        int sl=(li>GCC_FFT_SIZE/2)?li-GCC_FFT_SIZE:li;
        pk[GCC_N_PEAKS-1]={(((float)sl+dlt)/FS), v};
        for (int p=GCC_N_PEAKS-1;p>0&&pk[p].score>pk[p-1].score;p--){
            Peak t=pk[p]; pk[p]=pk[p-1]; pk[p-1]=t;
        }
    };

    for (int l=1; l<=TDOA_MAX_SAMP && l<GCC_FFT_SIZE-1; l++) {
        float v=gcc_rho[idx][l];
        if(v>gcc_rho[idx][l-1]&&v>gcc_rho[idx][l+1]) ins(l,v);
    }
    for (int l=GCC_FFT_SIZE-TDOA_MAX_SAMP; l<GCC_FFT_SIZE-1; l++) {
        if(l<1) continue;
        float v=gcc_rho[idx][l];
        if(v>gcc_rho[idx][l-1]&&v>gcc_rho[idx][l+1]) ins(l,v);
    }
    ins(0, gcc_rho[idx][0]);
}

static void run_gcc_phat() {
    for (int m = 0; m < NUM_MICS; m++) {
        ring_extract(m, 0, fwin, GCC_FFT_SIZE);
        float dc=0;
        for (int s=0; s<GCC_FFT_SIZE; s++) dc+=fwin[s];
        dc/=GCC_FFT_SIZE;
        for (int s=0; s<GCC_FFT_SIZE; s++)
            mic_win[m][s]=(fwin[s]-dc)*hann_win[s];
    }
    for (int p=0; p<NUM_PAIRS; p++) gcc_pair(PI_[p],PJ_[p],p);
}

// =============================================================================
// ── STEP 4: ES-SCFL ───────────────────────────────────────────────────────────
// =============================================================================

static bool run_escfl(float& zcs_out) {
    float best=1e9f;
    int   bc[NUM_PAIRS]={}, combo[NUM_PAIRS]={};

    for (int c=0; c<729; c++) {
        float t[NUM_PAIRS];
        for (int p=0; p<NUM_PAIRS; p++) t[p]=gcc_peaks[p][combo[p]].tau;
        float zcs=0;
        for (int l=0; l<3; l++)
            zcs+=fabsf(t[ZCS[l][0]]+t[ZCS[l][1]]-t[ZCS[l][2]]);
        float sc=0;
        for (int p=0; p<NUM_PAIRS; p++) sc+=gcc_peaks[p][combo[p]].score;
        float comb=zcs-3e-4f*sc;
        if(comb<best){best=comb; memcpy(bc,combo,sizeof(bc));}
        for(int p=NUM_PAIRS-1;p>=0;p--){if(++combo[p]<GCC_N_PEAKS)break;combo[p]=0;}
    }

    for (int p=0; p<NUM_PAIRS; p++)
        best_tau[p]=gcc_peaks[p][bc[p]].tau;
    for (int p=0; p<NUM_PAIRS; p++)
        smooth_tau[p]=TDOA_SMOOTH*best_tau[p]+(1.0f-TDOA_SMOOTH)*smooth_tau[p];

    zcs_out = best;
    return (best < ZCS_TOL);
}

// =============================================================================
// ── STEP 5: SRP-PHAT AZIMUTH ──────────────────────────────────────────────────
//
//  KEY FIX: compass azimuth convention
//  For compass bearing θ (0=North, 90=East, clockwise):
//    East component  (X) = sin(θ)
//    North component (Y) = cos(θ)
//
//  The expected TDOA for a plane wave from direction θ arriving at pair (i,j):
//    τ_ij = (dx·sin(θ) + dy·cos(θ)) / SOS
//  where dx = mic_j.x - mic_i.x,  dy = mic_j.y - mic_i.y
//
//  This replaces the old ux=cos(az), uy=sin(az) (math convention, 0=East)
//  which mirrored East/West relative to compass North.
// =============================================================================

static float srp_lookup(int pair, float tau_sec) {
    float lag = tau_sec * FS;
    if (fabsf(lag) > TDOA_MAX_SAMP + 1) return 0.0f;
    float idf = (lag >= 0) ? lag : (lag + GCC_FFT_SIZE);
    int   i0  = (int)idf % GCC_FFT_SIZE;
    int   i1  = (i0 + 1) % GCC_FFT_SIZE;
    float fr  = idf - floorf(idf);
    return gcc_rho[pair][i0]*(1.0f-fr) + gcc_rho[pair][i1]*fr;
}

static float srp_eval(float az_deg) {
    // Convert compass azimuth to unit vector in array (X=East, Y=North) frame
    float az_rad = az_deg * (float)M_PI / 180.0f;
    float ux = sinf(az_rad);   // East component  ← KEY FIX (was cosf)
    float uy = cosf(az_rad);   // North component ← KEY FIX (was sinf)

    float s = 0;
    for (int p = 0; p < NUM_PAIRS; p++) {
        float dx = MIC_POS[PJ_[p]][0] - MIC_POS[PI_[p]][0];
        float dy = MIC_POS[PJ_[p]][1] - MIC_POS[PI_[p]][1];
        // Expected TDOA = (dx·ux + dy·uy) / SOS
        s += srp_lookup(p, (dx*ux + dy*uy) / SOS_MS);
    }
    return s;
}

static float run_srp_phat() {
    // Coarse sweep: 2° steps across full 360°
    float bst = -1e9f; int baz = 0;
    for (int a = 0; a < 360; a += 2) {
        float s = srp_eval((float)a);
        if (s > bst) { bst = s; baz = a; }
    }
    // Fine sweep: 0.25° steps in ±10° window around coarse peak
    float faz = (float)baz, fbst = -1e9f;
    for (float a = baz - 10.0f; a <= baz + 10.0f; a += 0.25f) {
        float s = srp_eval(a);
        if (s > fbst) { fbst = s; faz = a; }
    }

    // Apply orientation trim and wrap to [0, 360)
    faz = fmodf(faz + ANGLE_OFFSET_DEG + 360.0f, 360.0f);
    return faz;
}

// =============================================================================
// ── OUTPUT ────────────────────────────────────────────────────────────────────
// =============================================================================

static void emit_event(int closest_mic, float angle, bool zcs_ok, float zcs_us) {
    const char* mic_str = MIC_NAME[closest_mic];
    const char* dir_str = angle_to_compass(angle);

    // USB Serial — human-readable debug
    Serial.println(F("========================================"));
    Serial.print(F("  Event #    : ")); Serial.println(event_count);
    Serial.print(F("  Closest mic: ")); Serial.println(mic_str);
    Serial.print(F("  Angle      : ")); Serial.print(angle, 1); Serial.println(F(" deg (0=N 90=E 180=S 270=W)"));
    Serial.print(F("  Direction  : ")); Serial.println(dir_str);
    Serial.print(F("  ZCS        : ")); Serial.print(zcs_us, 1);
    Serial.println(zcs_ok ? F(" µs (consistent)") : F(" µs (uncertain — angle less reliable)"));
    Serial.println(F("========================================"));

    // Serial3 (pin 14) → ESP32-S3 hub
    // Format: GS,<mic>,<angle>,<direction>\n
    char buf[80];
    snprintf(buf, sizeof(buf), "GS,%s,%.1f,%s\n", mic_str, angle, dir_str);
    Serial3.print(buf);
}

// =============================================================================
// ── SETUP ─────────────────────────────────────────────────────────────────────
// =============================================================================

void setup() {
    Serial.begin(SERIAL_BAUD);
    Serial3.begin(HUB_BAUD);
    delay(1000);

    Serial.println(F("=== GUNSHOT LOCALIZER  GCC-PHAT + SRP-PHAT ==="));
    Serial.println(F("Azimuth convention: 0=NORTH  90=EAST  180=SOUTH  270=WEST"));
    Serial.print(F("Array: 14 cm square  Band: "));
    Serial.print(BAND_LO_HZ, 0); Serial.print(F("–"));
    Serial.print(BAND_HI_HZ, 0); Serial.println(F(" Hz"));
    Serial.println(F("NORTH = direction M0(FL)/M1(FR) mics are facing"));
    Serial.print(F("Angle offset: ")); Serial.print(ANGLE_OFFSET_DEG, 1); Serial.println(F(" deg"));
    Serial.println(F("Waiting for gunshot...\n"));

    AudioMemory(60);
    arm_rfft_fast_init_f32(&fft_inst,     GCC_FFT_SIZE);
    arm_rfft_fast_init_f32(&det_fft_inst, DET_FFT);
    build_hann();

    for (int m = 0; m < NUM_MICS; m++) baseline_band[m] = 1e-6f;
    baseline_ste = 1e-6f;
    memset(smooth_tau, 0, sizeof(smooth_tau));

    for (int m = 0; m < NUM_MICS; m++) Q[m].begin();
}

// =============================================================================
// ── MAIN LOOP ─────────────────────────────────────────────────────────────────
// =============================================================================

static uint32_t det_block_count = 0;

void loop() {
    while (capture_block()) det_block_count++;

    if (det_block_count < 2) { delayMicroseconds(200); return; }
    det_block_count = 0;

    if (!run_detection()) return;

    event_count++;
    Serial.print(F("\n[EVT] #")); Serial.println(event_count);

    run_gcc_phat();

    float zcs_raw;
    bool  zcs_ok = run_escfl(zcs_raw);
    float zcs_us = zcs_raw * 1e6f;

    float angle   = run_srp_phat();
    int   closest = find_closest_mic();

    emit_event(closest, angle, zcs_ok, zcs_us);
}
