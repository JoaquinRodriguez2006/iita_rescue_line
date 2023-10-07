#include <Arduino.h>

void setup()
{
    Serial.begin(115200);
    Serial6.begin(57600);
}

void loop()
{
    int incomingByte;

    if (Serial6.available() > 0)
    {
        incomingByte = Serial6.read();
        Serial.print(char(incomingByte));
    }
}