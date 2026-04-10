#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SerialFlash.h>

// Quad I2S input (4 mics from SEN0526)
AudioInputI2SQuad      i2sQuad; 

// Analyze objects for each channel
AudioAnalyzePeak       peak1; // Mic 1
AudioAnalyzePeak       peak2; // Mic 2
AudioAnalyzePeak       peak3; // Mic 3
AudioAnalyzePeak       peak4; // Mic 4

// Audio connections
AudioConnection        patchCord1(i2sQuad, 0, peak1, 0); 
AudioConnection        patchCord2(i2sQuad, 1, peak2, 0); 
AudioConnection        patchCord3(i2sQuad, 2, peak3, 0); 
AudioConnection        patchCord4(i2sQuad, 3, peak4, 0); 

void setup() {
  Serial.begin(115200);
  AudioMemory(16);  
  delay(500);
  Serial.println("Mic1 Mic2 Mic3 Mic4");  // Label for Serial Plotter
}

void loop() {
  if (peak1.available() && peak2.available() && peak3.available() && peak4.available()) {

    float p1 = peak1.read();
    float p2 = peak2.read();
    float p3 = peak3.read();
    float p4 = peak4.read();

    // Print space-separated values for Serial Plotter
    Serial.print(p1, 4); Serial.print(" ");
    Serial.print(p2, 4); Serial.print(" ");
    Serial.print(p3, 4); Serial.print(" ");
    Serial.println(p4, 4);
  }

  delay(500); // smoother plotting
}
