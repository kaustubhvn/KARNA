// ============================================================
// PDR Localization System — ESP32-S3 SuperMini
// Sensors: MPU6050 + BMP280 (I2C) + BN220 GPS (UART)
// BLE GATT notification output at ~10Hz
// ============================================================
// ⚠ MOUNTING NOTE: Always mount at waist, chest, or secure
// pocket. Hand-held testing introduces severe arm-swing noise
// that corrupts step detection and heading. Results will be
// poor if you just hold the device while walking.
// ============================================================

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_Sensor.h>
#include <TinyGPSPlus.h>
#include <HardwareSerial.h>
#include <ArduinoBLE.h>
#include <math.h>

// ============================================================
// PIN CONFIGURATION
// ============================================================
#define I2C_SDA         8
#define I2C_SCL         9
#define GPS_RX_PIN      1       // ESP32 RX ← GPS TX
#define GPS_TX_PIN      3       // ESP32 TX → GPS RX (unused)
#define GPS_BAUD        9600
#define IMU_SAMPLE_HZ   100
#define IMU_INTERVAL_MS (1000 / IMU_SAMPLE_HZ)

// ============================================================
// BLE CONFIGURATION
// ============================================================
#define BLE_DEVICE_NAME     "PDR_ESP32"
#define BLE_SERVICE_UUID    "12345678-1234-1234-1234-1234567890AB"
#define BLE_CHAR_UUID       "12345678-1234-1234-1234-1234567890CD"

BLEService         pdrService(BLE_SERVICE_UUID);
BLECharacteristic  pdrChar(BLE_CHAR_UUID,
                           BLERead | BLENotify,
                           128);   // 128-byte payload

// ============================================================
// SENSOR OBJECTS
// ============================================================
Adafruit_MPU6050  mpu;
Adafruit_BMP280   bmp;
TinyGPSPlus       gps;
HardwareSerial    gpsSerial(1);   // UART1

// ============================================================
// CALIBRATION STATE
// ============================================================
float accelBiasX = 0, accelBiasY = 0, accelBiasZ = 0;
float gyroBiasX  = 0, gyroBiasY  = 0, gyroBiasZ  = 0;
const int   CALIB_SAMPLES   = 500;
const float GRAVITY         = 9.81f;

// ============================================================
// IMU FILTERING
// ============================================================
// α close to 1 = heavy smoothing (slower response)
// α close to 0 = less smoothing (faster response)
// 0.15 works well at 100Hz for walking (2 Hz gait)
const float LPF_ALPHA   = 0.15f;
float       accelMagLPF = GRAVITY;   // initialise to 1g

// ============================================================
// STEP DETECTION PARAMETERS
// ============================================================
// TUNING GUIDE:
//   STEP_UPPER_THRESHOLD: peak that confirms a step push-off.
//     Too low → false steps from swaying. Too high → missed steps.
//     Start at 10.5 m/s² and adjust ±0.5 until ~2 steps/sec walking
//     gives exactly 2 counts/sec.
//   STEP_LOWER_THRESHOLD: must fall below this after a peak.
//     Prevents double-counting the same step.
//   STEP_DEBOUNCE_MS: minimum time between steps.
//     Normal walking ~350–600ms between steps.
//     Set to 300ms to allow fast walking; raise to 400ms if
//     you see double-counting.
const float STEP_UPPER_THRESHOLD = 10.5f;   // m/s²
const float STEP_LOWER_THRESHOLD = 9.5f;    // m/s²
const int   STEP_DEBOUNCE_MS     = 300;     // ms

bool        stepPeakDetected  = false;
bool        belowLower        = true;
uint32_t    lastStepTime      = 0;
uint32_t    stepCount         = 0;
bool        stepThisFrame     = false;

// Peak tracker for Weinberg step length
float accelMax = GRAVITY;
float accelMin = GRAVITY;

// ============================================================
// STEP LENGTH ESTIMATION — WEINBERG METHOD
// ============================================================
// step_length = K * (amax - amin)^0.25
// K is the Weinberg constant. Calibrate experimentally:
//   Walk 20 steps on a measured 10m course.
//   Measure actual distance / step_count = actual_step_len
//   Compute estimated_step_len from formula.
//   K_new = K_old * (actual_step_len / estimated_step_len)
// Typical K range: 0.38–0.50 for waist/chest mount.
// Reduce K for chest mount, increase for loose pocket.
const float WEINBERG_K = 0.45f;   // meters

// ============================================================
// HEADING / YAW STATE
// ============================================================
// ⚠ CRITICAL LIMITATION: MPU6050 has NO magnetometer.
// Gyro-only yaw drifts at ~0.5–3°/sec depending on temperature
// and motion. Without mag or GPS correction, heading becomes
// unreliable after ~30–60 seconds of walking. This implementation
// corrects heading using GPS course when moving fast enough.
// This is NOT sufficient for indoor use or slow walking.
// Future upgrade path: add HMC5883L or use BNO055.
float heading_rad   = 0.0f;   // radians, 0 = east (GPS convention)
float gyroBiasZ_rt  = 0.0f;   // runtime drift estimate (future use)

// Complementary heading fusion weight
// β = 0 → pure gyro (drifts fast), β = 1 → pure GPS course (jumpy)
// 0.05–0.15 works well for outdoor walking
const float GPS_HEADING_WEIGHT = 0.08f;

// ============================================================
// PEDESTRIAN DEAD RECKONING STATE
// ============================================================
float pdr_x = 0.0f;   // meters east  from origin
float pdr_y = 0.0f;   // meters north from origin

// ============================================================
// GPS STATE AND REFERENCE ORIGIN
// ============================================================
double originLat = 0.0, originLon = 0.0;
bool   originSet = false;

double rawGpsLat = 0.0, rawGpsLon = 0.0;
double fusedLat  = 0.0, fusedLon  = 0.0;
float  gpsSpeed  = 0.0f;    // m/s
float  gpsCourse = 0.0f;    // degrees true north
int    gpsSats   = 0;
bool   gpsValid  = false;
bool   newGpsFix = false;

// GPS fusion parameters
// GPS_FUSION_ALPHA: weight given to GPS position vs PDR each fix.
//   0.3 = trusts GPS 30%, PDR 70%.
//   Increase if GPS quality is good (open sky, 8+ sats).
//   Decrease if GPS is noisy or indoors.
// GPS_OUTLIER_METERS: reject GPS fix if it jumps more than this
//   from current PDR position. 30m is conservative for 1Hz GPS.
const float GPS_FUSION_ALPHA   = 0.30f;
const float GPS_OUTLIER_METERS = 30.0f;
const float GPS_MIN_SPEED_MS   = 0.5f;    // m/s — below this ignore GPS course
const int   GPS_MIN_SATS       = 4;

// ============================================================
// BMP280 STATE
// ============================================================
float altitude_m = 0.0f;
float pressure_hpa = 0.0f;
float altitudeBaseline = 0.0f;
bool  altBaselineSet   = false;

// ============================================================
// TIMING
// ============================================================
uint32_t lastImuTime    = 0;
uint32_t lastBleTime    = 0;
uint32_t lastBmpTime    = 0;
const int BLE_TX_INTERVAL_MS = 100;   // 10Hz
const int BMP_INTERVAL_MS    = 500;   // 2Hz

// ============================================================
// COORDINATE CONVERSION UTILITIES
// ============================================================
// Earth radius in meters (WGS-84 approximation)
const double EARTH_R = 6371000.0;

// Convert local x (east), y (north) displacement in meters
// to absolute lat/lon given an origin lat/lon
void xy_to_latlon(double ox, double oy,         // origin lat/lon degrees
                  float x,  float y,            // local meters
                  double &lat, double &lon) {
    lat = ox + (y / EARTH_R) * (180.0 / M_PI);
    lon = oy + (x / (EARTH_R * cos(ox * M_PI / 180.0))) * (180.0 / M_PI);
}

// Convert absolute lat/lon to local x (east), y (north) meters
// relative to origin
void latlon_to_xy(double ox,  double oy,        // origin lat/lon degrees
                  double lat, double lon,        // point lat/lon degrees
                  float &x,   float &y) {
    y = (float)((lat - ox) * M_PI / 180.0 * EARTH_R);
    x = (float)((lon - oy) * M_PI / 180.0 * EARTH_R *
                cos(ox * M_PI / 180.0));
}

// ============================================================
// CALIBRATION
// ============================================================
void calibrateIMU() {
    Serial.println("[CAL] Keep device STILL for calibration...");
    delay(2000);

    double sumAx = 0, sumAy = 0, sumAz = 0;
    double sumGx = 0, sumGy = 0, sumGz = 0;

    for (int i = 0; i < CALIB_SAMPLES; i++) {
        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);

        sumAx += a.acceleration.x;
        sumAy += a.acceleration.y;
        sumAz += a.acceleration.z;
        sumGx += g.gyro.x;
        sumGy += g.gyro.y;
        sumGz += g.gyro.z;

        delay(5);   // ~200Hz during calibration
    }

    accelBiasX = sumAx / CALIB_SAMPLES;
    accelBiasY = sumAy / CALIB_SAMPLES;
    // Z bias: remove gravity component (assume device roughly flat)
    accelBiasZ = (sumAz / CALIB_SAMPLES) - GRAVITY;
    gyroBiasX  = sumGx / CALIB_SAMPLES;
    gyroBiasY  = sumGy / CALIB_SAMPLES;
    gyroBiasZ  = sumGz / CALIB_SAMPLES;

    Serial.printf("[CAL] AccelBias: %.4f %.4f %.4f\n",
                  accelBiasX, accelBiasY, accelBiasZ);
    Serial.printf("[CAL] GyroBias:  %.4f %.4f %.4f\n",
                  gyroBiasX,  gyroBiasY,  gyroBiasZ);
    Serial.println("[CAL] Calibration complete.");
}

// ============================================================
// SETUP
// ============================================================
void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("[BOOT] PDR Localization System");

    // I2C
    Wire.begin(I2C_SDA, I2C_SCL);

    // MPU6050
    if (!mpu.begin()) {
        Serial.println("[ERR] MPU6050 not found! Check wiring.");
        while (1) delay(500);
    }
    mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    // Built-in DLPF at 21Hz reduces high-freq vibration noise
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    Serial.println("[OK] MPU6050 ready");

    // BMP280
    if (!bmp.begin(0x76)) {
        if (!bmp.begin(0x77)) {
            Serial.println("[ERR] BMP280 not found! Check wiring.");
            while (1) delay(500);
        }
    }
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                    Adafruit_BMP280::SAMPLING_X2,
                    Adafruit_BMP280::SAMPLING_X16,
                    Adafruit_BMP280::FILTER_X16,
                    Adafruit_BMP280::STANDBY_MS_500);
    Serial.println("[OK] BMP280 ready");

    // GPS UART
    gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
    Serial.println("[OK] GPS UART ready");

    // IMU Calibration
    calibrateIMU();

    // BLE
    if (!BLE.begin()) {
        Serial.println("[ERR] BLE init failed!");
        while (1) delay(500);
    }
    BLE.setLocalName(BLE_DEVICE_NAME);
    BLE.setAdvertisedService(pdrService);
    pdrService.addCharacteristic(pdrChar);
    BLE.addService(pdrService);
    BLE.advertise();
    Serial.printf("[OK] BLE advertising as '%s'\n", BLE_DEVICE_NAME);

    // Baseline altitude (average 20 readings)
    float altSum = 0;
    for (int i = 0; i < 20; i++) {
        altSum += bmp.readAltitude(1013.25);
        delay(50);
    }
    altitudeBaseline = altSum / 20.0f;
    altBaselineSet = true;
    Serial.printf("[OK] Altitude baseline: %.2f m\n", altitudeBaseline);

    lastImuTime = millis();
    Serial.println("[READY] System running.");
}

// ============================================================
// IMU + PDR UPDATE (called at ~100Hz)
// ============================================================
void updateIMU() {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // Apply bias correction
    float ax = a.acceleration.x - accelBiasX;
    float ay = a.acceleration.y - accelBiasY;
    float az = a.acceleration.z - accelBiasZ;
    float gz = g.gyro.z        - gyroBiasZ;

    // Acceleration magnitude (should be ~9.81 at rest)
    float accelMag = sqrtf(ax*ax + ay*ay + az*az);

    // Low-pass filter — smooth out high-frequency vibrations
    accelMagLPF = LPF_ALPHA * accelMag + (1.0f - LPF_ALPHA) * accelMagLPF;

    // Track min/max for Weinberg within each step window
    if (accelMagLPF > accelMax) accelMax = accelMagLPF;
    if (accelMagLPF < accelMin) accelMin = accelMagLPF;

    // ---- STEP DETECTION (Schmidt trigger / peak detector) ----
    stepThisFrame = false;
    uint32_t now  = millis();

    if (!stepPeakDetected && belowLower &&
        accelMagLPF > STEP_UPPER_THRESHOLD) {
        // Rising edge — potential step
        stepPeakDetected = true;
        belowLower = false;
    }

    if (stepPeakDetected && accelMagLPF < STEP_LOWER_THRESHOLD) {
        // Falling edge — confirm step if debounce elapsed
        belowLower = true;
        if ((now - lastStepTime) >= (uint32_t)STEP_DEBOUNCE_MS) {
            // ---- CONFIRMED STEP ----
            stepThisFrame = true;
            stepCount++;
            lastStepTime = now;

            // Weinberg step length
            float stepLen = WEINBERG_K *
                            powf(accelMax - accelMin, 0.25f);
            // Clamp to sane range (0.3m–1.0m)
            stepLen = constrain(stepLen, 0.30f, 1.00f);

            // PDR displacement update
            pdr_x += stepLen * cosf(heading_rad);
            pdr_y += stepLen * sinf(heading_rad);

            // Reset peak tracker for next step
            accelMax = accelMagLPF;
            accelMin = accelMagLPF;
        }
        stepPeakDetected = false;
    }

    // ---- GYRO YAW INTEGRATION ----
    // dt is nominally 10ms (100Hz) but we use actual elapsed time
    float dt = IMU_INTERVAL_MS / 1000.0f;
    // gz in rad/s; integrate into heading
    // MPU6050 z-axis is vertical when flat. Adjust sign if
    // your mounting orientation is different.
    heading_rad += gz * dt;
    // Wrap to [-π, π]
    while (heading_rad >  M_PI) heading_rad -= 2.0f * M_PI;
    while (heading_rad < -M_PI) heading_rad += 2.0f * M_PI;
}

// ============================================================
// GPS PROCESSING (called when new NMEA data arrives)
// ============================================================
void processGPS() {
    if (!gps.location.isValid() || !gps.location.isUpdated()) return;

    rawGpsLat = gps.location.lat();
    rawGpsLon = gps.location.lng();
    gpsValid  = gps.location.isValid();
    gpsSats   = gps.satellites.isValid() ? gps.satellites.value() : 0;
    gpsSpeed  = gps.speed.isValid() ?
                (float)gps.speed.mps() : 0.0f;
    gpsCourse = gps.course.isValid() ?
                (float)gps.course.deg() : gpsCourse;

    newGpsFix = true;

    // Quality gate
    if (!gpsValid || gpsSats < GPS_MIN_SATS) return;

    // Set reference origin on first valid fix
    if (!originSet) {
        originLat = rawGpsLat;
        originLon = rawGpsLon;
        originSet = true;
        fusedLat  = rawGpsLat;
        fusedLon  = rawGpsLon;
        Serial.printf("[GPS] Origin set: %.7f, %.7f\n",
                      originLat, originLon);
        return;
    }

    // Convert GPS fix to local x, y
    float gps_x, gps_y;
    latlon_to_xy(originLat, originLon,
                 rawGpsLat, rawGpsLon,
                 gps_x, gps_y);

    // Outlier rejection: if GPS jumps too far from PDR, skip it
    float dist = sqrtf((gps_x - pdr_x)*(gps_x - pdr_x) +
                       (gps_y - pdr_y)*(gps_y - pdr_y));
    if (dist > GPS_OUTLIER_METERS) {
        Serial.printf("[GPS] Outlier rejected: %.1f m jump\n", dist);
        return;
    }

    // ---- COMPLEMENTARY POSITION FUSION ----
    pdr_x = GPS_FUSION_ALPHA * gps_x + (1.0f - GPS_FUSION_ALPHA) * pdr_x;
    pdr_y = GPS_FUSION_ALPHA * gps_y + (1.0f - GPS_FUSION_ALPHA) * pdr_y;

    // ---- HEADING CORRECTION FROM GPS COURSE ----
    // Only trust GPS course when moving at meaningful speed
    if (gpsSpeed > GPS_MIN_SPEED_MS && gps.course.isValid()) {
        // GPS course is clockwise from true north.
        // Convert to math convention: CCW from east.
        float gpsCourse_rad = (90.0f - gpsCourse) * M_PI / 180.0f;
        // Complementary blend
        // Blend shortest angular path to avoid wrap-around issues
        float diff = gpsCourse_rad - heading_rad;
        while (diff >  M_PI) diff -= 2.0f * M_PI;
        while (diff < -M_PI) diff += 2.0f * M_PI;
        heading_rad += GPS_HEADING_WEIGHT * diff;
    }

    // Reconstruct fused lat/lon from fused x, y
    xy_to_latlon(originLat, originLon, pdr_x, pdr_y, fusedLat, fusedLon);
}

// ============================================================
// BLE TRANSMISSION
// ============================================================
// Packet format (comma-separated ASCII for easy Python parsing):
// steps,stepFlag,heading_deg,x,y,alt,pressure,
// rawLat,rawLon,fusedLat,fusedLon,
// gpsSpeed,gpsCourse,sats,gpsValid
// ============================================================
void transmitBLE() {
    if (!BLE.connected()) return;

    char buf[128];
    float headingDeg = heading_rad * 180.0f / M_PI;

    int len = snprintf(buf, sizeof(buf),
        "%lu,%d,%.2f,%.3f,%.3f,%.2f,%.2f,"
        "%.7f,%.7f,%.7f,%.7f,"
        "%.2f,%.1f,%d,%d",
        stepCount,
        (int)stepThisFrame,
        headingDeg,
        pdr_x, pdr_y,
        altitude_m, pressure_hpa,
        rawGpsLat, rawGpsLon,
        fusedLat,  fusedLon,
        gpsSpeed, gpsCourse,
        gpsSats, (int)gpsValid
    );

    pdrChar.writeValue((uint8_t*)buf, len);
}

// ============================================================
// MAIN LOOP
// ============================================================
void loop() {
    // Feed GPS bytes to TinyGPSPlus
    while (gpsSerial.available() > 0) {
        char c = gpsSerial.read();
        if (gps.encode(c)) {
            processGPS();
        }
    }

    uint32_t now = millis();

    // IMU update at 100Hz
    if (now - lastImuTime >= IMU_INTERVAL_MS) {
        lastImuTime = now;
        updateIMU();
    }

    // BMP280 update at 2Hz
    if (now - lastBmpTime >= BMP_INTERVAL_MS) {
        lastBmpTime  = now;
        pressure_hpa = bmp.readPressure() / 100.0f;
        float rawAlt = bmp.readAltitude(1013.25f);
        altitude_m   = rawAlt - altitudeBaseline;   // relative altitude
    }

    // BLE transmit at 10Hz
    if (now - lastBleTime >= BLE_TX_INTERVAL_MS) {
        lastBleTime = now;
        BLE.poll();
        transmitBLE();

        // Debug to Serial
        Serial.printf("Steps:%lu H:%.1f° x:%.2f y:%.2f "
                      "Alt:%.1fm GPS:%s Sats:%d\n",
                      stepCount,
                      heading_rad * 180.0f / M_PI,
                      pdr_x, pdr_y,
                      altitude_m,
                      gpsValid ? "OK" : "NO",
                      gpsSats);
    }
}