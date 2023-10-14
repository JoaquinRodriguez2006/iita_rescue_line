#include <Arduino.h>
#include <drivebase.h>
#include <PID.h>
double steer, speed;
int dir, green_state, serial5state = 0

// https://www.arduino.cc/reference/en/
// https://www.pjrc.com/teensy/td_timing_IntervalTimer.html
// https://www.arduino.cc/en/Hacking/libraryTutorial

Moto bl(29, 28, 27);
Moto fl(7, 6, 5); // pwm dir enc
Moto br(36, 37, 38);
Moto fr(4, 3, 2);
DriveBase robot(&fl, &fr, &bl, &br);

void ISR1() { bl.updatePulse(); }
void ISR2() { fl.updatePulse(); }
void ISR3() { br.updatePulse(); }
void ISR4() { fr.updatePulse(); }

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(9600);
  attachInterrupt(digitalPinToInterrupt(27), ISR1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(5), ISR2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(38), ISR3, CHANGE);
  attachInterrupt(digitalPinToInterrupt(2), ISR4, CHANGE);
  
  // Serial Functions
  Serial.begin(115200);
  Serial5.begin(57600);
}

int counter = 0;
int incomingByte;
int reverse = 0;
int angle0;


void loop()
{
  // put your main code here, to run repeatedly:
  
  if (Serial5.available() > 0)
    {
      incomingByte = Serial5.read();
      Serial.print(char(incomingByte));

      // Data Reading
      if (incomingByte == 97)
        serial5state = 0; // There are no obstacels on the way, so the robot can keep going.
      else if (incomingByte == 98)
        serial5state = 3; // There are two green squares, and the robot has to go back
      
      // Data Processing
      else if (serial5state == 0)
      {
        robot.steer(100, HIGH, 0);
        delay(2000)
        Serial.println('AVANZAPA')
      }
      
      else if (serial5state == 3)
      {
          robot.steer(100, LOW, 0);
          delay(2000)
          Serial.println('RETROCEDEPA')
      }
    }
}
