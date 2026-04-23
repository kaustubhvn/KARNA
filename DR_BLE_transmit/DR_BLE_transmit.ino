// ============================================================
// PDR Localization System — EKF Enhanced Version
// ============================================================

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_Sensor.h>
#include <TinyGPSPlus.h>
#include <HardwareSerial.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <math.h>

// ================= PINS =================
#define I2C_SDA 8
#define I2C_SCL 9
#define GPS_RX_PIN 2
#define GPS_TX_PIN 1
#define GPS_BAUD 9600

#define IMU_SAMPLE_HZ 100
#define IMU_INTERVAL_MS (1000 / IMU_SAMPLE_HZ)

// ================= BLE =================
#define GPS_SERVICE_UUID  "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define GPS_CHAR_UUID     "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define DEVICE_NAME       "GPS_TRACKER"

BLEServer*         bleServer       = nullptr;
BLECharacteristic* gpsChar         = nullptr;
bool               clientConnected = false;

class ServerCB : public BLEServerCallbacks {
    void onConnect(BLEServer*) override {
        clientConnected = true;
        Serial.println("Map display connected");
    }
    void onDisconnect(BLEServer*) override {
        clientConnected = false;
        Serial.println("Map display disconnected — restarting advertising");
        BLEDevice::startAdvertising();
    }
};

// ================= SENSORS =================
Adafruit_MPU6050 mpu;
Adafruit_BMP280 bmp;
TinyGPSPlus gps;
HardwareSerial gpsSerial(1);

// ================= CONSTANTS =================
const float GRAVITY             = 9.81f;
const float LPF_ALPHA           = 0.15f;
const float STEP_UPPER_THRESHOLD = 10.5f;
const float STEP_LOWER_THRESHOLD = 9.5f;
const int   STEP_DEBOUNCE_MS    = 300;
const float WEINBERG_K          = 0.45f;

// ================= STATE =================
float accelBiasX=0, accelBiasY=0, accelBiasZ=0;
float gyroBiasZ=0;

float accelMagLPF = GRAVITY;

bool     stepPeakDetected=false, belowLower=true;
uint32_t lastStepTime=0, stepCount=0;
bool     stepThisFrame=false;

float accelMax=GRAVITY, accelMin=GRAVITY;
float heading_rad = 0;

// ================= EKF STATE =================
float ekf_x=0, ekf_y=0;
float P_x=1,   P_y=1;

// ================= GPS =================
double originLat=0, originLon=0;
bool   originSet=false;

double rawGpsLat=0, rawGpsLon=0;
double fusedLat=0,  fusedLon=0;

float gpsSpeed=0, gpsCourse=0;
int   gpsSats=0;
bool  gpsValid=false;

const float GPS_OUTLIER_METERS = 30.0f;

// ================= BMP =================
float altitude_m=0, pressure_hpa=0, altitudeBaseline=0;

// ================= TIMING =================
uint32_t lastImuTime=0, lastBleTime=0, lastBmpTime=0;

// ================= UTILS =================
const double EARTH_R = 6371000.0;

void xy_to_latlon(double ox, double oy, float x, float y, double &lat, double &lon){
    lat = ox + (y / EARTH_R) * (180.0 / M_PI);
    lon = oy + (x / (EARTH_R * cos(ox * M_PI / 180.0))) * (180.0 / M_PI);
}

void latlon_to_xy(double ox, double oy, double lat, double lon, float &x, float &y){
    y = (lat - ox) * M_PI / 180.0 * EARTH_R;
    x = (lon - oy) * M_PI / 180.0 * EARTH_R * cos(ox * M_PI / 180.0);
}

// ================= CALIBRATION =================
void calibrateIMU(){
    delay(2000);
    for(int i=0; i<500; i++){
        sensors_event_t a, g, t;
        mpu.getEvent(&a, &g, &t);
        accelBiasX += a.acceleration.x;
        accelBiasY += a.acceleration.y;
        accelBiasZ += a.acceleration.z;
        gyroBiasZ  += g.gyro.z;
        delay(5);
    }
    accelBiasX /= 500;
    accelBiasY /= 500;
    accelBiasZ  = (accelBiasZ / 500) - GRAVITY;
    gyroBiasZ  /= 500;
}

// ================= EKF =================
void ekf_predict(float stepLen){
    ekf_x += stepLen * cosf(heading_rad);
    ekf_y += stepLen * sinf(heading_rad);
    P_x   += 0.05f;
    P_y   += 0.05f;
}

void ekf_update(float gps_x, float gps_y){
    float dx   = gps_x - ekf_x;
    float dy   = gps_y - ekf_y;
    float dist = sqrtf(dx*dx + dy*dy);
    if(dist > GPS_OUTLIER_METERS) return;

    float K = 0.2f;
    if     (gpsSats >= 8) K = 0.6f;
    else if(gpsSats >= 5) K = 0.4f;

    ekf_x += K * dx;
    ekf_y += K * dy;
    P_x   *= (1.0f - K);
    P_y   *= (1.0f - K);
}

// ================= SETUP =================
void setup(){
    Serial.begin(115200);
    Wire.begin(I2C_SDA, I2C_SCL);

    mpu.begin();
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    bmp.begin(0x76);

    gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

    calibrateIMU();

    // ── BLE init ──────────────────────────────────────────────
    BLEDevice::init(DEVICE_NAME);
    BLEDevice::setMTU(64);

    bleServer = BLEDevice::createServer();
    bleServer->setCallbacks(new ServerCB());

    BLEService* svc = bleServer->createService(GPS_SERVICE_UUID);

    gpsChar = svc->createCharacteristic(
        GPS_CHAR_UUID,
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
    );
    gpsChar->addDescriptor(new BLE2902());  // required for notifications

    svc->start();

    BLEAdvertising* adv = BLEDevice::getAdvertising();
    adv->addServiceUUID(GPS_SERVICE_UUID);
    adv->setScanResponse(true);
    adv->setMinPreferred(0x06);
    BLEDevice::startAdvertising();

    Serial.println("BLE advertising as: " DEVICE_NAME);

    // ── BMP baseline ──────────────────────────────────────────
    for(int i=0; i<20; i++){
        altitudeBaseline += bmp.readAltitude(1013.25);
        delay(50);
    }
    altitudeBaseline /= 20;

    lastImuTime = millis();
}

// ================= IMU =================
void updateIMU(){
    sensors_event_t a, g, t;
    mpu.getEvent(&a, &g, &t);

    float ax = a.acceleration.x - accelBiasX;
    float ay = a.acceleration.y - accelBiasY;
    float az = a.acceleration.z - accelBiasZ;
    float gz = g.gyro.z - gyroBiasZ;

    float accelMag = sqrtf(ax*ax + ay*ay + az*az);
    accelMagLPF = LPF_ALPHA * accelMag + (1.0f - LPF_ALPHA) * accelMagLPF;

    if(accelMagLPF > accelMax) accelMax = accelMagLPF;
    if(accelMagLPF < accelMin) accelMin = accelMagLPF;

    uint32_t now = millis();
    stepThisFrame = false;

    if(!stepPeakDetected && belowLower && accelMagLPF > STEP_UPPER_THRESHOLD){
        stepPeakDetected = true;
        belowLower       = false;
    }

    if(stepPeakDetected && accelMagLPF < STEP_LOWER_THRESHOLD){
        belowLower = true;
        if(now - lastStepTime >= STEP_DEBOUNCE_MS){
            stepThisFrame = true;
            stepCount++;
            lastStepTime = now;

            float stepLen = WEINBERG_K * powf(accelMax - accelMin, 0.25f);
            stepLen = constrain(stepLen, 0.3f, 1.0f);

            ekf_predict(stepLen);

            accelMax = accelMagLPF;
            accelMin = accelMagLPF;
        }
        stepPeakDetected = false;
    }

    float dt = IMU_INTERVAL_MS / 1000.0f;
    heading_rad += gz * dt;
    while(heading_rad >  M_PI) heading_rad -= 2.0f * M_PI;
    while(heading_rad < -M_PI) heading_rad += 2.0f * M_PI;
}

// ================= GPS =================
void processGPS(){
    if(!gps.location.isUpdated()) return;

    rawGpsLat = gps.location.lat();
    rawGpsLon = gps.location.lng();
    gpsValid  = gps.location.isValid();
    gpsSats   = gps.satellites.value();
    gpsSpeed  = gps.speed.mps();
    gpsCourse = gps.course.deg();

    if(!gpsValid || gpsSats < 4) return;

    if(!originSet){
        originLat = rawGpsLat;
        originLon = rawGpsLon;
        originSet = true;
        return;
    }

    float gps_x, gps_y;
    latlon_to_xy(originLat, originLon, rawGpsLat, rawGpsLon, gps_x, gps_y);
    ekf_update(gps_x, gps_y);
    xy_to_latlon(originLat, originLon, ekf_x, ekf_y, fusedLat, fusedLon);
}

// ================= BLE TRANSMIT =================
// void transmitBLE(){
//     if(!clientConnected) return;

//     char buf[128];
//     int len = snprintf(buf, sizeof(buf),
//         "%lu,%d,%.2f,%.3f,%.3f,%.2f,%.2f,"
//         "%.7f,%.7f,%.7f,%.7f,"
//         "%.2f,%.1f,%d,%d",
//         stepCount,
//         (int)stepThisFrame,
//         heading_rad * 180.0f / M_PI,
//         ekf_x, ekf_y,
//         altitude_m, pressure_hpa,
//         rawGpsLat, rawGpsLon,
//         fusedLat,  fusedLon,
//         gpsSpeed, gpsCourse,
//         gpsSats, (int)gpsValid
//     );

//    ////Add the serial print lines here, to monitor the BLE transmit data.
//    Serial.print(millis());
//   Serial.print(" ms | BLE TX -> ");
//   Serial.println(buf);

//     gpsChar->setValue((uint8_t*)buf, len);
//     gpsChar->notify();
// }

// ================= BLE TRANSMIT =================
void transmitBLE(){
    if(!clientConnected) return;

    // Receiver expects exactly: "LAT,LON,HDG"
    // fusedLat/fusedLon = EKF output, heading_rad converted to 0–360°
    float heading_deg = heading_rad * 180.0f / M_PI;
    if(heading_deg < 0) heading_deg += 360.0f;

    char buf[48];
    int len = snprintf(buf, sizeof(buf),
        "%.6f,%.6f,%.1f",
        (float)rawGpsLat,
        (float)rawGpsLon,
        heading_deg
    );

    Serial.print(millis());
    Serial.print(" ms | BLE TX -> ");
    Serial.println(buf);

    gpsChar->setValue((uint8_t*)buf, len);
    gpsChar->notify();
}

// ================= LOOP =================
void loop(){
    while(gpsSerial.available()){
        if(gps.encode(gpsSerial.read())){
            processGPS();
        }
    }

    uint32_t now = millis();

    if(now - lastImuTime >= IMU_INTERVAL_MS){
        lastImuTime = now;
        updateIMU();
    }

    if(now - lastBmpTime >= 500){
        lastBmpTime  = now;
        pressure_hpa = bmp.readPressure() / 100.0f;
        altitude_m   = bmp.readAltitude(1013.25f) - altitudeBaseline;
    }

    if(now - lastBleTime >= 100){
        lastBleTime = now;
        transmitBLE();   // no BLE.poll() needed — BLEDevice stack is interrupt-driven
    }
}