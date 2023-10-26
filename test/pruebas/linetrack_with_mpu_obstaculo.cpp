#include <Arduino.h>
#include <drivebase.h>
#include <NewPing.h>
#include <PID.h>
#include <wire.h> 
#include <MPU6050.h>

// CONSTANTS //
#define FORWARD 0
#define BACKWARD 1
#define TRIG_FRONT_PIN 39
#define ECHO_FRONT_PIN 33

// STATE VARIABLES & FLAGS //
int serial6state = 0; // serial code e.g. 255

double speed;          // speed (0 to 100)
double steer;          // angle (0 to 180 deg, will -90 later)
int incoming_task = 0; // 0 = no green squares, 1 = left, 2 = right, 3 = double, 4 = pick up cube, 10 = deposit
int line_middle = 0;   // if there is a line to reacquire after obstacle

int action;            // action to take (part of a task)
bool taskDone = false; // if true, update current_task

int evacAngle0;
int angle0;             // initial IMU reading
int depositCounter = 0; // number of loops where robot is moving parallel to deposit zone
unsigned long long depositTime = 0;
bool startUp = false;

double wallTrackTarget = 100;
bool seenEvac = false;
bool deposited = false;
float frontUSReading;

int a = -112 + 180;
int b = -75 + 180;
int c = -23 + 180;
int d = 36 + 180;

float yaw = 0;
MPU6050 mpu;

int dir, green_state, serial5state = 0; // 0 is motor angle, 1 is motor speed
int imuLookup[360];
uint8_t currentSonar = 0;
elapsedMicros sinceFrontFire;
volatile unsigned long long frontVal;
volatile bool frontFireTime = true;
elapsedMillis sinceFrontPrint;

Moto bl(29, 28, 27);
Moto fl(7, 6, 5); // pwm dir enc
Moto br(36, 37, 38);
Moto fr(4, 3, 2);
DriveBase robot(&fl, &fr, &bl, &br);

void ISR1() { bl.updatePulse(); }
void ISR2() { fl.updatePulse(); }
void ISR3() { br.updatePulse(); }
void ISR4() { fr.updatePulse(); }

void getFrontAnalog()
{
  frontVal = sinceFrontFire;
  frontFireTime = true;
}

//////////////// HELPER FUNCTIONS ////////////////
void runTime(int speed, int dir, double steer, unsigned long long time)
{
  unsigned long long startTime = millis();
  // elapsedMillis startTime;
  while ((millis() - startTime) < time)
  {
    robot.steer(speed, dir, steer);
    digitalWrite(13, HIGH);
    if (digitalRead(17) == 1) // switch is off
    {
      Serial6.write(255);
    }
  }

  digitalWrite(13, LOW);

  // while ((millis() - startTime) < time + 500) {
  // robot.steer(0, FORWARD, 0);
  // }
}

void runTime2(int speed, int dir, double steer, unsigned long long time)
{
  unsigned long long startTime = millis();
  // elapsedMillis startTime;
  while ((millis() - startTime) < time)
  {
    robot.steer(speed, dir, steer);
    digitalWrite(13, HIGH);
    Serial6.write(255);
  }

  digitalWrite(13, LOW);
}

//////////////// ULTRASONIC SENSOR FUNCTIONS ////////////////

int imuIndex0, imuIndex1, imuIndex2, imuIndex3, imuIndex;
int imu[3];
char imuBuffer[30] = "                             ";
String imuString = String(imuBuffer);

int reverse = 0;


void serialEvent5()
{
    int data = Serial5.read();
    if (data == 255)
        serial5state = 0;
    else if (data == 254)
        serial5state = 1;
    else if (data == 253)
        serial5state = 2;
    else if (data == 252)
        serial5state = 3;
    else if (serial5state == 0)
    {
        speed = (double)data /20 * 20; // 20 set as max speed
        speed = max(15, speed);           // 15 set as min speed
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

// Timers
unsigned long timer = 0;
float timeStep = 0.01;

// Pitch, Roll and Yaw values
float pitch = 0;
float roll = 0;


void setup()
{
    ///////////////////// MPU6050 /////////////////////       
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

    Serial.begin(115200);
    // put your setup code here, to run once:
    attachInterrupt(digitalPinToInterrupt(27), ISR1, CHANGE);
    attachInterrupt(digitalPinToInterrupt(5), ISR2, CHANGE);
    attachInterrupt(digitalPinToInterrupt(38), ISR3, CHANGE);
    attachInterrupt(digitalPinToInterrupt(2), ISR4, CHANGE);
    pinMode(16, OUTPUT);
    pinMode(1, INPUT_PULLUP);
    pinMode(13, OUTPUT);
    // Serial.begin(115200);
    Serial1.begin(57600);
    Serial5.begin(57600);

    for (int i = 0; i < 360; i++)
  {
    if ((i - a + 360) % 360 < (b - a + 360) % 360)
    { // q1
      imuLookup[i] = (int)(((float)((i - a + 360) % 360) / (float)((b - a + 360) % 360)) * 90);
    }
    else if ((i - a + 360) % 360 < (c - a + 360) % 360)
    { // q2
      imuLookup[i] = (int)(((float)((i - b + 360) % 360) / (float)((c - b + 360) % 360)) * 90) + 90;
    }
    else if ((i - a + 360) % 360 < (d - a + 720) % 360)
    { // q3
      imuLookup[i] = (int)(((float)((i - c + 360) % 360) / (float)((d - c + 360) % 360)) * 90) + 180;
    }
    else
    {
      imuLookup[i] = (int)(((float)((i - d + 360) % 360) / (float)((a - d + 360) % 360)) * 90) + 270;
    }
  }
}

void loop()
{
    ///////////////////// ULTRASONIC SENSOR /////////////////////    
    int rawimu = imu[0];                              // ranges -180 to 180
    int correctedimu = imuLookup[rawimu + 180] - 180; // lookup table ranges from 0-360, but final result ranges from 0-180
    Serial.print("Raw IMU: ");
    Serial.print(rawimu);
    Serial.println(" ");
   ///////////////// MPU6050 /////////////////
   timer = millis();

  // Read normalized values
  Vector norm = mpu.readNormalizeGyro();

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
  delay((timeStep*1000) - (millis() - timer));
  //mpu
   
   ///////////////// LINETRACK /////////////////
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
            digitalWrite(13, LOW);
        }
    }
    Serial.print(speed);
    Serial.print(" ");
    Serial.println(steer);

    // Ultrasonic Sensor
    if (frontFireTime)
  {
    digitalWrite(TRIG_FRONT_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_FRONT_PIN, HIGH);
    delayMicroseconds(5);
    digitalWrite(TRIG_FRONT_PIN, LOW);
    sinceFrontFire = 0;
    frontFireTime = false;
    if (sinceFrontPrint > 50)
    {
      // Serial.print("f:\t");
      // Serial.println(((frontVal)*0.034) / 2.0);
      sinceFrontPrint = 0;
      frontUSReading = ((frontVal)*0.034) / 2.0;
    }
  }
    if (frontUSReading < 19 && digitalRead(17) == 0 && !(seenEvac)) // obstacle detected
      {
        angle0 = correctedimu - 85; // initialise setpoint
        // Turns 90°
        runTime2(40, FORWARD, -1, 1050);  // hard-coded turn
        runTime2(50, FORWARD, 0.4, 100);
        // arc turn with line_middle
        if (line_middle < 170) // arc turn
        {
        robot.steer(40, 0, 0.35);
        }
        else
        {
        runTime(50, FORWARD, 0.0, 300);
        // runTime(50, BACKWARD, 0.0, 500);
        // imu turn 90 deg back to line
        angle0 = correctedimu - 70;
        }
        int error = ((((int)correctedimu - angle0) + 180) % 360 - 180);
      if (error < -180)
        error += 360;

      if (error > 0)
        robot.steer(abs(error), 0, -1);
      else
        robot.steer(abs(error), 0, 1);

      if (abs(error) < 2)
      {
        taskDone = true; // action will be updated in the next loop
      }
      // runTime2(40, FORWARD, -1, 1050);
      // taskDone = true;
      //break;
      }
}