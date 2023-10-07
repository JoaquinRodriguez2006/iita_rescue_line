#include <Arduino.h>

void setup()
{
  // put your setup code here, to run once:
  pinMode(16, OUTPUT);
  pinMode(17, INPUT_PULLUP);
  pinMode(13, OUTPUT);
  digitalWrite(16, LOW);
}

void loop()
{
  digitalWrite(13, digitalRead(17));
}