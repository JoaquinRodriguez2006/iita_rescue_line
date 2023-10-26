#include <Arduino.h>
#include <drivebase.h>
#include <PID.h>

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
}

int counter = 0;
void loop()
{
  // put your main code here, to run repeatedly:
  robot.steer(20, LOW, 0);
  delay(2000);
  robot.steer(20, HIGH, 0);
  delay(2000);
}