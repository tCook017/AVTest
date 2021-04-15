#include <Adafruit_Sensor.h>     // sensor abstraction library
//#include <Adafruit_FXAS21002C.h> // gyroscope
//#include <Adafruit_FXOS8700.h>   // accelerometer and magnetometer
//#include <Adafruit_MPU6050.h>   // Another IMU
#include <Adafruit_BMP3XX.h>     // barometric pressure sensor
#include <SD.h>
#include <TinyMPU6050.h>
#include <Wire.h>

#define SEALEVELPRESSURE_HPA (1013.2)

/* TODOs:
 *  Create version without Serial prints before flight (keep Serial1)
 *  Reduce/remove delays
 */

// The FXAS21002C and the FXOS8700 constitute the NXP 9-DOF board
//Adafruit_FXAS21002C gyro = Adafruit_FXAS21002C(0x0021002C);
//Adafruit_FXOS8700 accel = Adafruit_FXOS8700(0x8700A, 0x8700B);
//Adafruit_MPU6050 mpu;
MPU6050 mpu(Wire);
Adafruit_BMP3XX bmp = Adafruit_BMP3XX();

File telemFile;
char* telemFileName = "telem.csv";
char loopIndex = 0;

void setup() {
  Serial.begin(115200);

  // Setup I2C bus (Used by the NXP 9-DOF & the BMP388 breakout boards)
  Wire.setSDA(18);
  Wire.setSCL(19);

  mpu.Initialize(); // Must do this after setting up Wire, since it internally calls 
  Serial.println("Starting MPU6050 Calibration...");
  mpu.Calibrate();
  Serial.println("MPU6050 Calibration complete.");

  // Wait for the hardware to initialize
  bool initialized = false;
  while (!initialized) {
    delay(1000);
    initialized = true;
//    if (!gyro.begin()) {
//      Serial.println("Error detecting FXAS21002C");
//      initialized = false;
//    }
//    if (!accel.begin(ACCEL_RANGE_8G)) {
//      Serial.println("Error detecting FXOS8700");
//      initialized = false;
//    }
//    if (!mpu.begin()) {
//      Serial.println("Error detecting MPU6050");
//      initialized = false;
//    }
    if (!bmp.begin_I2C(0x77)) {
      Serial.println("Error detecting BMP388");
      initialized = false;
    }
    if (!SD.begin(BUILTIN_SDCARD)) {
      Serial.println("Error detecting SD card");
      initialized = false;
    }
  }

  // Setup IMU (MPU6050)
  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // Setup Barometer (BMP388)
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_2X); // needed for the bmp388
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_2X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);

  // Setup XBee
  Serial1.clear();
  Serial1.setRX(0);
  Serial1.setTX(1);
  Serial1.begin(9600);

  telemFile = SD.open(telemFileName, FILE_WRITE);
  if (telemFile) {
    telemFile.println("Test");
    telemFile.close();
  }
  else {
    Serial.println("Error writing to SD");
  }

  delay(500); // Delay for smoothing initial BMP
  Serial.println("Setup complete.");
}

void loop() {
//  sensors_event_t gyro_event, accel_event;  // For old IMU setup
//  gyro.getEvent(&gyro_event); // For FXAS21002C
//  accel.getEvent(&accel_event, NULL); // For FXOS8700
  sensors_event_t gyro_event, accel_event, temp_event;
//  mpu.getEvent(&gyro_event, &accel_event, &temp_event);
  mpu.Execute();
  
  String newTelem = "";

  // Read time
  unsigned long timestamp = millis();
  Serial.print(timestamp);
  Serial.print(" - ");
  
//  Serial1.print(timestamp);
//  Serial1.print(',');
//  newTelem += String(timestamp) + ',';

  // Read gyroscope
  Serial.print(gyro_event.gyro.x);
  Serial.print(", ");
  Serial.print(gyro_event.gyro.y);
  Serial.print(", ");
  Serial.println(gyro_event.gyro.z);

//  Serial1.print(gyro_event.gyro.x);
//  Serial1.print(',');
//  Serial1.print(gyro_event.gyro.y);
//  Serial1.print(',');
//  Serial1.print(gyro_event.gyro.z);
//  newTelem += String(gyro_event.gyro.x) + ',';
//  newTelem += String(gyro_event.gyro.y) + ',';
//  newTelem += String(gyro_event.gyro.z);
  
  // Read accelerometer [m/s^2]
  Serial.print(" - ");
  Serial.print(accel_event.acceleration.x);
  Serial.print(", ");
  Serial.print(accel_event.acceleration.y);
  Serial.print(", ");
  Serial.println(accel_event.acceleration.z);

//  Serial1.print(',');
//  Serial1.print(accel_event.acceleration.x);
//  Serial1.print(',');
//  Serial1.print(accel_event.acceleration.y);
//  Serial1.print(',');
//  Serial1.print(accel_event.acceleration.z);
//  newTelem += ',';
//  newTelem += String(accel_event.acceleration.x) + ',';
//  newTelem += String(accel_event.acceleration.y) + ',';
//  newTelem += String(accel_event.acceleration.z);

  // Read bmp tempature [C], pressure [hPa], and altitude [m]
  Serial.print(" - ");
  Serial.print(bmp.temperature);  
  Serial.print(", ");
  Serial.print(bmp.pressure / 100);
  Serial.print(", ");
  Serial.println(bmp.readAltitude(SEALEVELPRESSURE_HPA));  

//  Serial1.print(',');
//  Serial1.print(bmp.temperature);
//  Serial1.print(',');
//  Serial1.print(bmp.pressure); 
//  Serial1.print(',');
//  Serial1.println(bmp.readAltitude(SEALEVELPRESSURE_HPA));
//  newTelem += ',';
//  newTelem += String(bmp.temperature) + ',';
//  newTelem += String(bmp.pressure) + ',';
//  newTelem += String(bmp.readAltitude(SEALEVELPRESSURE_HPA));

  if (loopIndex >= 9) {
    newTelem += String(timestamp) + ',';
    newTelem += String(gyro_event.gyro.x) + ',';
    newTelem += String(gyro_event.gyro.y) + ',';
    newTelem += String(gyro_event.gyro.z);
    newTelem += ',';
    newTelem += String(accel_event.acceleration.x) + ',';
    newTelem += String(accel_event.acceleration.y) + ',';
    newTelem += String(accel_event.acceleration.z);
    newTelem += ',';
    newTelem += String(bmp.temperature) + ',';
    newTelem += String(bmp.pressure) + ',';
    newTelem += String(bmp.readAltitude(SEALEVELPRESSURE_HPA));
    Serial1.println(newTelem);
    
    loopIndex = -1; // Will be updated below to 0
  }

  telemFile = SD.open(telemFileName, FILE_WRITE);
  if (telemFile) {
    telemFile.println(newTelem);
    telemFile.close();
    Serial.println("Wrote to SD");
  }
  else {
    Serial.println("Error writing to SD");
  }

//  delay(1000);
  ++loopIndex;
}
