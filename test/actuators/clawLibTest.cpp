#include <Arduino.h>
#include <Servo.h>
#include <claw.h>

DFServo sort(22, 540, 2390, 274);
DFServo left(14, 540, 2390, 274);
DFServo right(15, 540, 2390, 274);
DFServo lift(12, 540, 2390, 274);
DFServo deposit(23, 540, 2390, 274);
Claw claw(&lift, &left, &right, &sort, &deposit);

void setup()
{
    // put your setup code here, to run once:
    Serial.begin(115200);
}

void loop()
{
    claw.collect();
}