#include <Arduino.h>
#include <drivebase.h>
#include <PID.h>
#include <wire.h> 
#include <NewPing.h>
#include <VL53L0X.h>
MPU6050 mpu;
double steer, speed;
int dir, green_state, serial5state = 0; // 0 is motor angle, 1 is motor speed

Moto bl(29, 28, 27);
Moto fl(7, 6, 5); // pwm dir enc
Moto br(36, 37, 38);
Moto fr(4, 3, 2);
DriveBase robot(&fl, &fr, &bl, &br)

void ISR1() { bl.updatePulse(); }
void ISR2() { fl.updatePulse(); }
void ISR3() { br.updatePulse(); }
void ISR4() { fr.updatePulse(); }

int imuIndex0, imuIndex1, imuIndex2, imuIndex3, imuIndex;
int imu[3];
char imuBuffer[30] = "                             ";
String imuString = String(imuBuffer);



int reverse = 0;
int angle0;

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
        speed = (double)data / 100 * 100; // 100 set as max speed
        speed = max(20, speed);           // 20 set as min speed
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
            angle0 = imu[0];
        }
    }
}
//mpu
// Timers
unsigned long timer = 0;
float timeStep = 0;

// Pitch, Roll and Yaw values
float pitch = 0;
float roll = 0;
float yaw = 0;
//the end

/// OLTRASOUND SENSORS///
#define SONAR_NUM 3
#define MAX_DISTANCE 400
#define PING_INTERVAL 33

NewPing sonar[SONAR_NUM] = {
    NewPing(8, 9, MAX_DISTANCE),   // right
    NewPing(11, 10, MAX_DISTANCE), // left
    NewPing(39, 33, MAX_DISTANCE), // front
};                                 // trigger, echo, max distance in cm

unsigned long pingTimer[SONAR_NUM]; // Holds the next ping time.
unsigned int cm[SONAR_NUM];
uint8_t currentSonar = 0;

void ISR5()
{ // If ping echo, set distance to array.
    if (sonar[currentSonar].check_timer())
        cm[currentSonar] = sonar[currentSonar].ping_result / US_ROUNDTRIP_CM;
}

void oneSensorCycle()
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
/// END ULTRASOUND ///
///TOF/// 
VL53L0X sensor;   // Sensor 1
VL53L0X sensor1;  // Sensor 2
///TOF///
void setup()
{
    Serial.begin(115200);
    // put your setup code here, to run once:
    attachInterrupt(digitalPinToInterrupt(27), ISR1, CHANGE);
    attachInterrupt(digitalPinToInterrupt(5), ISR2, CHANGE);
    attachInterrupt(digitalPinToInterrupt(38), ISR3, CHANGE);
    attachInterrupt(digitalPinToInterrupt(2), ISR4, CHANGE);
    pinMode(16, OUTPUT);
    pinMode(1, INPUT_PULLUP);
    pinMode(13, OUTPUT);
    Serial.begin(115200);
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

                ///ULTRASOUND///
                 Serial.begin(115200); // Open serial monitor at 115200 baud to see ping results.
    pingTimer[0] = millis() + 75;
    for (uint8_t i = 1; i < SONAR_NUM; i++)
        pingTimer[i] = pingTimer[i - 1] + PING_INTERVAL;
        ///END///
        ///TOF///
         Serial.begin(9600);
  Wire.begin();   // Initialize the first I2C bus
  Wire1.begin();  // Initialize the second I2C bus

  sensor.setBus(&Wire);   // Assign the first bus to Sensor 1
  sensor1.setBus(&Wire1); // Assign the second bus to Sensor 2

  sensor.setAddress(0x30); // Set unique address for Sensor 1
  sensor1.setAddress(0x30); // Set unique address for Sensor 2

// Continue with your setup and loop functions as before

  sensor.init();
  sensor.setTimeout(500);
  sensor.startContinuous();

  sensor1.init();
  sensor1.setTimeout(500);
  sensor1.startContinuous();
  ///END///
}



void loop()
{
   
   //mpu
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
   
   
   
   
    if (digitalRead(17) == 1)
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

        ///ULTRASOUND///
        for (uint8_t i = 0; i < SONAR_NUM; i++)
    {
        if (millis() >= pingTimer[i])
        {
            pingTimer[i] += PING_INTERVAL * SONAR_NUM;
            if (i == 0 && currentSonar == SONAR_NUM - 1)
                oneSensorCycle(); // Do something with results.
            sonar[currentSonar].timer_stop();
            currentSonar = i;
            cm[currentSonar] = 0;
            sonar[currentSonar].ping_timer(ISR5);
        }
    }
    ///END///
    ///TOF///
     int distance = sensor.readRangeContinuousMillimeters();
  int distance1 = sensor1.readRangeContinuousMillimeters();

  Serial.print("Distance 1: ");
  Serial.print(distance);
  Serial.print("mm");

  if (sensor.timeoutOccurred()) { Serial.print(" TIMEOUT"); }

  Serial.print("   Distance 2: ");
  Serial.print(distance1 - 60);
  Serial.print("mm");

  if (sensor1.timeoutOccurred()) { Serial.print(" TIMEOUT"); }

  Serial.println();
  delay(100);
  ///END///


}