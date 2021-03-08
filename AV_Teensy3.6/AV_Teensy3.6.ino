#include <Adafruit_Sensor.h>     // sensor abstraction library
#include <Adafruit_FXAS21002C.h> // gyroscope
#include <Adafruit_FXOS8700.h>   // accelerometer and magnetometer
#include <Adafruit_BMP3XX.h>     // barometric pressure sensor
#include <Wire.h>

#define SEALEVELPRESSURE_HPA (1013.2)

// The FXAS21002C and the FXOS8700 constitute the NXP 9-DOF board
Adafruit_FXAS21002C gyro = Adafruit_FXAS21002C(0x0021002C);
Adafruit_FXOS8700 accel = Adafruit_FXOS8700(0x8700A, 0x8700B);
Adafruit_BMP3XX bmp = Adafruit_BMP3XX();

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
}

void loop() {
  sensors_event_t gyro_event, accel_event;
  gyro.getEvent(&gyro_event);
  accel.getEvent(&accel_event, NULL);

  Serial.print(millis());
  Serial.print(" - ");
  Serial.print(gyro_event.gyro.x);
  Serial.print(", ");
  Serial.print(gyro_event.gyro.y);
  Serial.print(", ");
  Serial.println(gyro_event.gyro.z);

  Serial1.print(millis());
  Serial1.print(',');
  Serial1.print(gyro_event.gyro.x);
  Serial1.print(',');
  Serial1.print(gyro_event.gyro.y);
  Serial1.print(',');
  Serial1.print(gyro_event.gyro.z);
  Serial1.print("\n");

  Serial.print(" - ");  // read accel {m/s^2}
  Serial.print(accel_event.acceleration.x);
  Serial.print(", ");
  Serial.print(accel_event.acceleration.y);
  Serial.print(", ");
  Serial.println(accel_event.acceleration.z);

  Serial1.print(',');
  Serial1.print(accel_event.acceleration.x);
  Serial1.print(',');
  Serial1.print(accel_event.acceleration.y);
  Serial1.print(',');
  Serial1.print(accel_event.acceleration.z);

  Serial.print(" - ");  // read tempature [C]
  Serial.println(bmp.temperature);  

  Serial1.print(',');
  Serial1.print(bmp.temperature);

  Serial.print(" - ");  // read pressure [hPa]
  Serial.println(bmp.pressure / 100);  

  Serial1.print(',');
  Serial1.print(bmp.pressure);

  Serial.print(" - ");  // read altitude [m]
  Serial.println(bmp.readAltitude(SEALEVELPRESSURE_HPA));  

  Serial1.print(',');
  Serial1.print(bmp.readAltitude(SEALEVELPRESSURE_HPA));

  delay(1000);
}
