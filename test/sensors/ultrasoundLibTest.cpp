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

void ISR5()///Esta función se ejecutará cada vez que el sensor ultrasónico detecte un eco.///
{ // If ping echo, set distance to array.
    if (sonar[currentSonar].check_timer())
        cm[currentSonar] = sonar[currentSonar].ping_result / US_ROUNDTRIP_CM;
}

void oneSensorCycle()///muestra la distancia medida por cada sensor y lo muestra en el monitor///
{ // Do something with the results.
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
    Serial.begin(115200); // Open serial monitor at 115200 baud to see ping results.
    pingTimer[0] = millis() + 75;///el primer sensor hace su primera medicion desp de 75 milisegundos
    ///las mediciones de todos los sensores se distribuyan a lo largo del tiempo y no se realicen al mismo tiempo.//
    for (uint8_t i = 1; i < SONAR_NUM; i++)
        pingTimer[i] = pingTimer[i - 1] + PING_INTERVAL;
        ///utiliza el pingtimer para la funcion de arriba///
}

void loop()
{
    for (uint8_t i = 0; i < SONAR_NUM; i++)///se verifican los sensores uno por uno
    {
        if (millis() >= pingTimer[i])///Acá se decide si hay  que medir la distancia del sensor
        {
            pingTimer[i] += PING_INTERVAL * SONAR_NUM;///Se configura el temporizador para los sensores
            if (i == 0 && currentSonar == SONAR_NUM - 1)
                oneSensorCycle(); // Do something with results.
            sonar[currentSonar].timer_stop();
            currentSonar = i;
            cm[currentSonar] = 0;
            sonar[currentSonar].ping_timer(ISR5);
            ///Se repite todo para seguir tomando lectura de distancias y decisiones basada en la lectura///
        }
    }
}
