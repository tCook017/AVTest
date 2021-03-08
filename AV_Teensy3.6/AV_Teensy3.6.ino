#include <Adafruit_Sensor.h>     // sensor abstraction library
#include <Adafruit_FXAS21002C.h> // gyroscope
#include <Adafruit_FXOS8700.h>   // accelerometer and magnetometer
#include <Adafruit_BMP3XX.h>     // barometric pressure sensor
#include <SD.h>
#include <Wire.h>

#define SEALEVELPRESSURE_HPA (1013.2)

/* TODOs:
 *  Create version without Serial prints before flight (keep Serial1)
 *  Reduce/remove delays
 */

const int SD_SELECT = BUILTIN_SDCARD;

// The FXAS21002C and the FXOS8700 constitute the NXP 9-DOF board
Adafruit_FXAS21002C gyro = Adafruit_FXAS21002C(0x0021002C);
Adafruit_FXOS8700 accel = Adafruit_FXOS8700(0x8700A, 0x8700B);
Adafruit_BMP3XX bmp = Adafruit_BMP3XX();

File telemFile;

void setup() {
  Serial.begin(9600);

  // Setup I2C bus (Used by the NXP 9-DOF & the BMP388 breakout boards)
  Wire.setSDA(18);
  Wire.setSCL(19);

  // Wait for the hardware to initialize
  bool initialized = false;
  while (!initialized) {
    delay(1000);
    initialized = true;
    if (!gyro.begin()) {
      Serial.println("Error detecting FXAS21002C");
      initialized = false;
    }
    if (!accel.begin(ACCEL_RANGE_8G)) {
      Serial.println("Error detecting FXOS8700");
      initialized = false;
    }
    if (!bmp.begin_I2C(0x77)) {
      Serial.println("Error detecting BMP388");
      initialized = false;
    }
    if (!SD.begin(SD_SELECT)) {
      Serial.println("Error detectig SD card");
      initialized = false;
    }
  }

  // Setup Barometer (Bmp388)
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_2X); // needed for the bmp388
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_2X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);

  // Setup XBee
  Serial1.clear();
  Serial1.setRX(0);
  Serial1.setTX(1);
  Serial1.begin(9600);

  telemFile = SD.open("telemFile.csv", FILE_WRITE);
  telemFile.close();
}

void loop() {
  sensors_event_t gyro_event, accel_event;
  gyro.getEvent(&gyro_event);
  accel.getEvent(&accel_event, NULL);
  
  String newTelem = "";

  // Read time
  unsigned long timestamp = millis();
  Serial.print(timestamp);
  Serial.print(" - ");
  
//  Serial1.print(timestamp);
//  Serial1.print(',');
  newTelem += String(timestamp) + ',';

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
  newTelem += String(gyro_event.gyro.x) + ',';
  newTelem += String(gyro_event.gyro.y) + ',';
  newTelem += String(gyro_event.gyro.z);
  
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
  newTelem += ',';
  newTelem += String(accel_event.acceleration.x) + ',';
  newTelem += String(accel_event.acceleration.y) + ',';
  newTelem += String(accel_event.acceleration.z);

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
  newTelem += ',';
  newTelem += String(bmp.temperature) + ',';
  newTelem += String(bmp.pressure) + ',';
  newTelem += String(bmp.readAltitude(SEALEVELPRESSURE_HPA));

  telemFile = SD.open("telemFile.csv", FILE_WRITE);
  if (telemFile != NULL) {
    telemFile.println(newTelem);
    dataFile.close();
    Serial.println("Wrote to SD");
  }
  else {
    Serial.println("Error writing to SD");
  }

  delay(1000);
}
