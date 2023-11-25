#include <Arduino.h>
#include <drivebase.h>
#include <PID.h>
#include <wire.h> 
#include <MPU6050.h>
#include <NewPing.h>
#define SONAR_NUM 3
#define MAX_DISTANCE 400
#define PING_INTERVAL 33

MPU6050 mpu;
double steer, speed;
int dir, green_state, serial5state = 0; // 0 is motor angle, 1 is motor speed
// Temporizadores
unsigned long timer = 0;
float timeStep = 0.01;

// Pitch, Roll and Yaw valores
float pitch = 0;
float roll = 0;
float yaw = 0;
//the end 

Moto bl(29, 28, 27);
Moto fl(7, 6, 5); // pwm dir enc
Moto br(36, 37, 38);
Moto fr(4, 3, 2);
DriveBase robot(&fl, &fr, &bl, &br);

void ISR1() { bl.updatePulse(); }
void ISR2() { fl.updatePulse(); }
void ISR3() { br.updatePulse(); }
void ISR4() { fr.updatePulse(); }


int reverse = 0;
int angle0;

void serialEvent5()
{
    int data = Serial5.read();
    Serial.println("reading serial 5");
    if (data == 255)
        serial5state = 0;
    else if (data == 254)
        serial5state = 1;//izquierda
    else if (data == 253)
        serial5state = 2;//derecha
    else if (data == 252)
        serial5state = 3;//180°   
    else if (serial5state == 0)
    {
        speed = 20;          // 20 set as min 
    }
    else if (serial5state == 1)
    {
        steer = ((double)data - 90) / 90;
    }
    else if (serial5state == 2)
    {
        green_state = data;

        if (green_state == 3 && !reverse)
        {
            reverse = 1;
            angle0 = yaw;
        }
    }
}

//////ultrasound////////
NewPing sonar[SONAR_NUM] = {NewPing(39, 33, MAX_DISTANCE)};

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
    //SENSOR//
 Serial.begin(115200); // Open serial monitor at 115200 baud to see ping results.
    pingTimer[0] = millis() + 75;///el primer sensor hace su primera medicion desp de 75 milisegundos
    ///las mediciones de todos los sensores se distribuyan a lo largo del tiempo y no se realicen al mismo tiempo.//
    for (uint8_t i = 1; i < SONAR_NUM; i++)
        pingTimer[i] = pingTimer[i - 1] + PING_INTERVAL;
//END//


    Serial.begin(115200);
    // put your setup code here, to run once:
    attachInterrupt(digitalPinToInterrupt(27), ISR1, CHANGE);
    attachInterrupt(digitalPinToInterrupt(5), ISR2, CHANGE);
    attachInterrupt(digitalPinToInterrupt(38), ISR3, CHANGE);
    attachInterrupt(digitalPinToInterrupt(2), ISR4, CHANGE);
    pinMode(16, OUTPUT);
    pinMode(1, INPUT_PULLUP);
    pinMode(13, OUTPUT);
    Serial1.begin(57600);
    Serial5.begin(57600);
    //mpu


  // Initialize MPU6050
  while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }
  
  // Calibrate gyroscope. The calibration must be at rest.
  // If you don't want calibrate, comment this line.
  mpu.calibrateGyro();

  // Set threshold sensivty. Default 3.
  // If you don't want use threshold, comment this line or set 0.
  mpu.setThreshold(3);
//mpu
}
   

void loop()
{
   
   //mpu
   timer = millis();

  // Read normalized values
  Vector norm = mpu.readNormalizeGyro();

  delay((timeStep*1000) - (millis() - timer));///destacado delay////

  // Calculate Pitch, Roll and Yaw
  pitch = pitch + norm.YAxis * timeStep;
  roll = roll + norm.XAxis * timeStep;
  yaw = yaw + norm.ZAxis * timeStep;

  // Output raw
  Serial.print(" Pitch = ");
  Serial.print(pitch);
  Serial.print(" Roll = ");
  Serial.print(roll);  
  Serial.print(" Yaw = ");
  Serial.println(yaw);

  // Wait to full timeStep period   
   
    if (digitalRead(1) == 1)
    {
        robot.steer(0, dir, 0);
        digitalWrite(13, HIGH);
        dir = !dir;
    }
    else
    {
        if (reverse)
        {
            int error = abs((int)yaw - angle0);

            if (error < 175)
            { // turn 180 degrees
                // Serial.println(abs((int)imu[0] - angle0));
                robot.steer(40, 1, 1);
            }

            else
            {
                reverse = 0;
            }
        }
        else
        {
            robot.steer(speed, 0, steer);
            digitalWrite(13, HIGH);
        }
    }
    Serial.print(speed);
    Serial.print(" ");
    Serial.println(steer);
////sensores/////
 if (cm[0] <= 30) { // Considera el primer sensor de ultrasonido
         // Llama a la función de detención si la distancia es menor o igual a 30 cm
    } else {
          robot.steer(0, 0, 0); 
        // ... (resto del código) ...
    }
}
