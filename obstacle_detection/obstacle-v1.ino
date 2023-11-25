#include <Arduino.h>
#include <drivebase.h>
#include <PID.h>
#include <wire.h> 
#include <MPU6050.h>
#include <HCSR04.h>
#include <VL53L0X.h>

float frontUSReading;
elapsedMicros sinceFrontFire;
volatile unsigned long long frontVal;
volatile bool frontFireTime = true;

int reverse = 0;
int angle0;

void getFrontAnalog()
{
  frontVal = sinceFrontFire;
  frontFireTime = true;
}

void loop() {
    if (frontFireTime)
  {
    digitalWrite(TRIG_FRONT_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_FRONT_PIN, HIGH);
    delayMicroseconds(5);
    digitalWrite(TRIG_FRONT_PIN, LOW);
    sinceFrontFire = 0;
    frontFireTime = false;
    if (sinceFrontPrint > 50)
    {
      // Serial.print("f:\t");
      // Serial.println(((frontVal)*0.034) / 2.0);
      sinceFrontPrint = 0;
      frontUSReading = ((frontVal)*0.034) / 2.0;
    }
  }

    if (frontUSReading < 19) 
    {
        (40, FORWARD, -1, 1050);  // hard-coded turn
        runTime2(50, FORWARD, 0.4, 100);
        angle0 = correctedimu - 85; // initialise setpoint
        if (line_middle < 170){ // arc turn
            robot.steer(40, 0, 0.35);}
        else {
            runTime(50, FORWARD, 0.0, 300);
            // runTime(50, BACKWARD, 0.0, 500);
            int error = ((((int)correctedimu - angle0) + 180) % 360 - 180);
            if (error < -180)
                error += 360;

            if (error > 0)
                robot.steer(abs(error), 0, -1);
            else
                robot.steer(abs(error), 0, 1);

            if (abs(error) < 2)
            {
                taskDone = true; // action will be updated in the next loop
            }
            // runTime2(40, FORWARD, -1, 1050);
            // taskDone = true;           // next action = imu turn 90 deg back to line
            angle0 = correctedimu - 70; } 
    }

}
