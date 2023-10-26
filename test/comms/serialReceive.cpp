#include <Arduino.h>

void setup()
{
    Serial.begin(115200);
    Serial2.begin(57600);
}

void loop()
{
    int incomingByte;

    if (Serial2.available() > 0)
    {
        incomingByte = Serial2.read();
        Serial.print(char(incomingByte));
    }
}