/* This example shows how to use continuous mode to take
range measurements with the VL53L0X. It is based on
vl53l0x_ContinuousRanging_Example.c from the VL53L0X API.

The range readings are in units of mm. */

#include <Wire.h>
#include <VL53L0X.h>

VL53L0X sensor;

void setup()
{
  Serial.begin(9600);
  Wire.begin();

  sensor.setTimeout(500);

  //to check for measurments in a more efficient manner you can
  //connect the sensors GPIO0 pin to Arduino, and initialise
  //it with "sensor.init(myPinNumber)"
  if (!sensor.init())
  {
    Serial.println("Failed to detect and initialize sensor!");
    while (1) {}
  }

  // Start continuous back-to-back mode (take readings as
  // fast as possible).  To use continuous timed mode
  // instead, provide a desired inter-measurement period in
  // ms (e.g. sensor.startContinuous(100)).
  sensor.startContinuous();
}

void loop()
{
  if(sensor.available())
  {
       Serial.print(sensor.readRangeMillimeters());
       if (sensor.timeoutOccurred()) { Serial.print(" TIMEOUT"); }
  }

  Serial.println();
}
