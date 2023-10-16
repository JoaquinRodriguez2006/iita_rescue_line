#include <Arduino.h>
#include <drivebase.h>
#include <PID.h>
#include <wire.h> 
#include <MPU6050.h>
#include <SoftwareSerial.h>

SoftwareSerial BTSerial(34, 35); // RX, TX pins del módulo Bluetooth
float yaw = 0;
MPU6050 mpu;
double steer, speed;
int dir, green_state, serial5state = 0; // 0 is motor angle, 1 is motor speed


Moto bl(29, 28, 27);
Moto fl(7, 6, 5); // pwm dir enc
Moto br(36, 37, 38);
Moto fr(4, 3, 2);
DriveBase robot(&fl, &fr, &bl, &br);

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
//mpu
// Timers
unsigned long timer = 0;
float timeStep = 0.01;

// Pitch, Roll and Yaw values
float pitch = 0;
float roll = 0;

//the end


void setup()
{
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
    Serial.begin(115200);
    Serial1.begin(57600);
    Serial5.begin(57600);

    /////////////// HC06 ///////////////
    Serial.begin(9600); // Inicializa la comunicación serie a través del USB
    BTSerial.begin(9600); // Inicializa la comunicación serie a través del Bluetooth
}

// MPU6050

void loop()
{
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

    /////////////// HC06 ///////////////
    if (BTSerial.available() > 0) {
        char messageRaspberry = yaw;
        BTSerial.write(messageRaspberry);
        BTSerial.print(messageRaspberry);
  }
}