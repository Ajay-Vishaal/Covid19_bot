// Includes
//uncomment the lora library and functions while deploying lora feature
//#include "LoRaWAN.h"
#include <LSM6DSOSensor.h>
#include <LIS2DW12Sensor.h>
#include <LIS2MDLSensor.h>
#include <LPS22HHSensor.h>
#include <STTS751Sensor.h>
#include <HTS221Sensor.h>

//const uint8_t payload[]=""

//const char *devEui = "Fill_Yours";
//const char *appEui = "Fill_Yours";
//const char *appKey = "Fill_Yours";

#ifdef ARDUINO_SAM_DUE
#define DEV_I2C Wire1
#elif defined(ARDUINO_ARCH_STM32)
#define DEV_I2C Wire
#elif defined(ARDUINO_ARCH_AVR)
#define DEV_I2C Wire
#else
#define DEV_I2C Wire
#endif
#define Serial Serial

// Components
LSM6DSOSensor *AccGyr;
LIS2DW12Sensor *Acc2;
LIS2MDLSensor *Mag;
LPS22HHSensor *PressTemp;
HTS221Sensor *HumTemp;
STTS751Sensor *Temp3;

void setup(void) {
  // Led.
  pinMode(LED_BUILTIN, OUTPUT);

  // Initialize serial for output.
  Serial.begin(115200);

  // Initialize I2C bus.
  DEV_I2C.begin();

  while (!Serial) { }

  // Use your region's ISM band
  //  LoRaWAN.begin(IN865);
  // Helium SubBand
  //  LoRaWAN.setSubBand(2);
  // Disable Adaptive Data Rate
  //  LoRaWAN.setADR(false);
  //   Set Data Rate 1 - Max Payload 53 Bytes
  //  LoRaWAN.setDataRate(1);
  // Device IDs and Key
  //  LoRaWAN.joinOTAA(appEui, appKey, devEui);

  //  Serial.print("Connecting");


  AccGyr = new LSM6DSOSensor (&DEV_I2C);
  AccGyr->Enable_X();
  AccGyr->Enable_G();
  Acc2 = new LIS2DW12Sensor (&DEV_I2C);
  Acc2->Enable_X();
  Mag = new LIS2MDLSensor (&DEV_I2C);
  Mag->Enable();
  PressTemp = new LPS22HHSensor(&DEV_I2C);
  PressTemp->Enable();
  HumTemp = new HTS221Sensor (&DEV_I2C);
  HumTemp->Enable();
  Temp3 = new STTS751Sensor (&DEV_I2C);
  Temp3->Enable();
}

void loop(void) {
  // Led blinking.
  //  digitalWrite(LED_BUILTIN, HIGH);
  //  delay(2000);
  //  digitalWrite(LED_BUILTIN, LOW);
  //  delay(1000);
  // Read humidity and temperature.
  float humidity = 0, temperature = 0;
  HumTemp->GetHumidity(&humidity);
  HumTemp->GetTemperature(&temperature);

  // Read pressure and temperature.
  float pressure = 0, temperature2 = 0;
  PressTemp->GetPressure(&pressure);
  PressTemp->GetTemperature(&temperature2);

  //Read temperature
  float temperature3 = 0;
  Temp3->GetTemperature(&temperature3);

  // Read accelerometer and gyroscope.
  int32_t accelerometer[3];
  int32_t gyroscope[3];
  AccGyr->Get_X_Axes(accelerometer);
  AccGyr->Get_G_Axes(gyroscope);

  //Read accelerometer
  int32_t accelerometer2[3];
  Acc2->Get_X_Axes(accelerometer2);

  //Read magnetometer
  int32_t magnetometer[3];
  Mag->GetAxes(magnetometer);

  // Output data.

  //  if (LoRaWAN.joined() && !LoRaWAN.busy())
  //  {Serial.print("Joined");
  if (Serial.available() > 0) {
    char inByte = Serial.read();
    char str[2];
    str[1] = inByte;
    delay(250);
    if (strcmp(str, "humi") == 1) {
      Serial.print(" Hum[%]: ");
      Serial.print(humidity, 2);
      Serial.print("; Temp[C]: ");
      Serial.print(temperature, 2);
      //        LoRaWAN.sendPacket(1, data, sizeof(payload));
    }
    if (strcmp(str, "pres") == 1) {
      Serial.print(" Pres[hPa]: ");
      Serial.print(pressure, 2);
      Serial.print(" ;Temp2[C]: ");
      Serial.print(temperature2, 2);
      //        LoRaWAN.sendPacket(1, payload, sizeof(payload));
    }
    if (strcmp(str, "temp") == 1) {
      Serial.print(" Temp3[C]: ");
      Serial.print(temperature3, 2);
      //        LoRaWAN.sendPacket(1, payload, sizeof(payload));
    }
    if (strcmp(str, "gyro") == 1) {
      Serial.print(" Acc[mg]: ");
      Serial.print(accelerometer[0]);
      Serial.print(",");
      Serial.print(accelerometer[1]);
      Serial.print(",");
      Serial.print(accelerometer[2]);
      Serial.print(" Gyr[mdps]: ");
      Serial.print(gyroscope[0]);
      Serial.print(",");
      Serial.print(gyroscope[1]);
      Serial.print(",");
      Serial.print(gyroscope[2]);
      //        LoRaWAN.sendPacket(1, payload, sizeof(payload));
    }
    if (strcmp(str, "acce") == 1) {
      Serial.print(" Acc2[mg]: ");
      Serial.print(accelerometer2[0]);
      Serial.print(",");
      Serial.print(accelerometer2[1]);
      Serial.print(",");
      Serial.print(accelerometer2[2]);
      //        LoRaWAN.sendPacket(1, payload, sizeof(payload));
    }
    if (strcmp(str, "magn") == 1) {
      Serial.print(" Mag[mGauss]: ");
      Serial.print(magnetometer[0]);
      Serial.print(",");
      Serial.print(magnetometer[1]);
      Serial.print(",");
      Serial.print(magnetometer[2]);
      //        LoRaWAN.sendPacket(1, payload, sizeof(payload));
    }
  }
}
//}
