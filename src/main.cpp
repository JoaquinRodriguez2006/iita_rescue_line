#include <Wire.h>
#include <VL53L0X.h>

VL53L0X sensor;   // Sensor 1
VL53L0X sensor1;  // Sensor 2

void setup()
{
  Serial.begin(9600);
  Wire1.begin();   // Initialize the first I2C bus
  Wire2.begin();  // Initialize the second I2C bus

  sensor.setBus(&Wire1);   // Assign the first bus to Sensor 1
  sensor1.setBus(&Wire2); // Assign the second bus to Sensor 2

  sensor.setAddress(0x30); // Set unique address for Sensor 1
  sensor1.setAddress(0x30); // Set unique address for Sensor 2

// Continue with your setup and loop functions as before

  sensor.init();
  sensor.setTimeout(500);
  sensor.startContinuous();

  sensor1.init();
  sensor1.setTimeout(500);
  sensor1.startContinuous();
}

void loop()
{
  int distance = sensor.readRangeContinuousMillimeters();
  int distance1 = sensor1.readRangeContinuousMillimeters();

  Serial.print("Distance 1: ");
  Serial.print(distance);
  Serial.print("mm");

  if (sensor.timeoutOccurred()) { Serial.print(" TIMEOUT"); }

  Serial.print("   Distance 2: ");
  Serial.print(distance1 - 60);
  Serial.print("mm");

  if (sensor1.timeoutOccurred()) { Serial.print(" TIMEOUT"); }

  Serial.println();
  delay(100);
}
