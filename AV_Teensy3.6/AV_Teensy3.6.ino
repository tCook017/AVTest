/* Useful Resources
 * MPU6050 Datasheet - https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf
 */

#include <Adafruit_Sensor.h>     // sensor abstraction library
#include <Adafruit_BMP3XX.h>     // barometric pressure sensor
#include <SD.h>
#include <TinyMPU6050.h>
#include <Wire.h>

#define SEALEVELPRESSURE_HPA (1013.2)

/* TODOs:
 *  Create version without Serial prints before flight (keep Serial1)
 */

MPU6050 mpu(Wire);
Adafruit_BMP3XX bmp = Adafruit_BMP3XX();

char* telemFileName = "telem.csv";
char* lordFileName = "telamen.csv";
char loopIndex = 0;

void setup() {
  Serial.begin(115200);

  // Setup I2C bus (Used by the NXP 9-DOF & the BMP388 breakout boards)
  Wire.setSDA(18);
  Wire.setSCL(19);

  // Setup IMU (MPU6050) using TinyMPU6050 library.
  mpu.Initialize(); // Must do this after setting up Wire, since it internally calls 
  mpu.RegisterWrite(MPU6050_ACCEL_CONFIG, 0x08);  // By default, TinyMPU6050 uses +-2g, we want +-16g
  Serial.println("Starting MPU6050 Calibration...");
  mpu.Calibrate();
  Serial.println("MPU6050 Calibration complete.");

  // Wait for the hardware to initialize
  bool initialized = false;
  while (!initialized) {
    delay(1000);
    initialized = true;
    if (!bmp.begin_I2C(0x77)) {
      Serial.println("Error detecting BMP388");
      initialized = false;
    }
    if (!SD.begin(BUILTIN_SDCARD)) {
      Serial.println("Error detecting SD card");
      initialized = false;
    }
  }

  // Setup Barometer (BMP388)
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_2X); // needed for the bmp388
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_2X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);

  // Setup XBee UART
  Serial1.clear();
  Serial1.setRX(0);
  Serial1.setTX(1);
  Serial1.begin(115200);

  // Setup Lord IMU UART
  Serial2.clear();
  Serial2.setRX(9);
  Serial2.setTX(10);
  Serial2.begin(115200);

  // Write to telemetry file, can use as a spacer to know what data is new.
  writeToFile(telemFileName, "Test");

  delay(500); // Delay for smoothing initial BMP
  Serial.println("Setup complete.");
}

void loop() {
  mpu.Execute();  // Update values (which mpu.Get____() will read from) with new data.

  // Time
  unsigned long timestamp = millis();
  Serial.println(timestamp);

  // Gyro x,y,z [degrees/second]
  Serial.print("\t");
  Serial.print(mpu.GetGyroX());
  Serial.print(", ");
  Serial.print(mpu.GetGyroY());
  Serial.print(", ");
  Serial.println(mpu.GetGyroZ());

  // Acceleration x,y,z [m/s²]
  Serial.print("\t");
  Serial.print(mpu.GetAccX());
  Serial.print(", ");
  Serial.print(mpu.GetAccY());
  Serial.print(", ");
  Serial.println(mpu.GetAccZ());

  // Attitude (yaw, pitch, roll in [degrees])
  Serial.print("\t");
  Serial.print(mpu.GetAngX());
  Serial.print(", ");
  Serial.print(mpu.GetAngY());
  Serial.print(", ");
  Serial.println(mpu.GetAngZ());

  // Read bmp tempature [C], pressure [hPa], and altitude [m]
  Serial.print(bmp.temperature);  
  Serial.print(", ");
  Serial.print(bmp.pressure / 100);
  Serial.print(", ");
  Serial.println(bmp.readAltitude(SEALEVELPRESSURE_HPA));

  String newTelem = String(timestamp) + ',';
  newTelem += String(mpu.GetGyroX()) + ',';
  newTelem += String(mpu.GetGyroY()) + ',';
  newTelem += String(mpu.GetGyroZ());
  newTelem += ',';
  newTelem += String(mpu.GetAccX()) + ',';
  newTelem += String(mpu.GetAccY()) + ',';
  newTelem += String(mpu.GetAccZ());
  newTelem += ',';
  newTelem += String(bmp.temperature) + ',';
  newTelem += String(bmp.pressure) + ',';
  newTelem += String(bmp.readAltitude(SEALEVELPRESSURE_HPA));
  writeToFile(telemFileName, newTelem);

  // Send data over XBee once every 10 loops
  if (loopIndex >= 9) {
    Serial1.println(newTelem);
    loopIndex = -1; // Will be updated below to 0
  }
  ++loopIndex;

  unsigned short lordAvailable = Serial2.available();
  char lordTelem[lordAvailable];
  Serial2.readBytes(lordTelem, lordAvailable);
  writeToFile(lordFileName, ((String)lordTelem) + '\n');
}

void writeToFile(char* fileName, String telem) {
  File telemFile = SD.open(fileName, FILE_WRITE);
  if (telemFile) {
    telemFile.println(telem);
    telemFile.close();
    Serial.println("Wrote to " + (String)fileName + " on SD");
  }
  else {
    Serial.println("Error writing to " + (String)fileName + " on SD");
  }
}
