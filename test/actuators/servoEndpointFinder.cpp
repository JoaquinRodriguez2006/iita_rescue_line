#include <Arduino.h>
#include <Servo.h>

Servo myservo;

int counter = 0;
// min endpoint is 540
// max endpoint is 2390
// increasing number makes it go anti-clockwise
// servo range is 275deg

void setup()
{
    // put your setup code here, to run once:
    Serial.begin(9600);
    myservo.attach(14);
}

void loop()
{
    myservo.writeMicroseconds(2450);
    delay(5000);
    for (counter = 2450; counter >= 550; counter -= 50)
    {
        myservo.writeMicroseconds(counter);
        Serial.println(counter);
        delay(100);
    }
    delay(5000);
}