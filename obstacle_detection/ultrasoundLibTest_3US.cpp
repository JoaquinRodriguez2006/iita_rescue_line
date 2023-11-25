#include <Arduino.h>
#include <NewPing.h>

#define SONAR_NUM 3
#define MAX_DISTANCE 400
#define PING_INTERVAL 33

NewPing sonar[SONAR_NUM] = {
    NewPing(3, 2, MAX_DISTANCE),   // right
    NewPing(9, 8, MAX_DISTANCE), // left
    NewPing(11, 10, MAX_DISTANCE), // front
};                                 // trigger, echo, max distance in cm

unsigned long pingTimer[SONAR_NUM]; // Holds the next ping time.
unsigned int cm[SONAR_NUM];
uint8_t currentSonar = 0;

void ISR5()
{
    if (sonar[currentSonar].check_timer())
        cm[currentSonar] = sonar[currentSonar].ping_result / US_ROUNDTRIP_CM;
    if (cm[0] < 0 or cm[1] < 0 or cm[2] < 0) {
      for (uint8_t i = 0; i < SONAR_NUM; i++)
    {
        if (millis() >= pingTimer[i])
        {
            pingTimer[i] += PING_INTERVAL * SONAR_NUM;
            if (i == 0 && currentSonar == SONAR_NUM - 1) {
            oneSensorCycle(); 
            if (cm[0] < 5 and cm[2] < 3) {
                digitalWrite(LED_BUILTIN, HIGH);
                delay(1000);
                digitalWrite(LED_BUILTIN, LOW);
                delay(1000);
            }
        }
            sonar[currentSonar].timer_stop();
            currentSonar = i;
            cm[currentSonar] = 0;
            sonar[currentSonar].ping_timer(ISR5);
        }
    }
    }
}

void oneSensorCycle() {
    for (uint8_t i = 0; i < SONAR_NUM; i++)
    {
        Serial.print(i);
        Serial.print("=");
        Serial.print(cm[i]);
        Serial.print("cm ");
    }
    Serial.println();
}

void setup()
{
    // Blink
    pinMode(LED_BUILTIN, OUTPUT);

    // Distance Sensor
    Serial.begin(115200);
    pingTimer[0] = millis() + 75;
    for (uint8_t i = 1; i < SONAR_NUM; i++)
        pingTimer[i] = pingTimer[i - 1] + PING_INTERVAL;
}

void loop()
{
    
}
