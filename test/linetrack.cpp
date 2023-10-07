#include <Arduino.h>
#include <drivebase.h>
#include <PID.h>
double steer, speed;
int dir, green_state, serial6state = 0; // 0 is motor angle, 1 is motor speed

Moto bl(4, 3, 2);
Moto fl(35, 34, 36); // pwm dir enc
Moto br(7, 6, 5);
Moto fr(30, 28, 29);
DriveBase robot(&fl, &fr, &bl, &br);

void ISR1() { bl.updatePulse(); }
void ISR2() { fl.updatePulse(); }
void ISR3() { br.updatePulse(); }
void ISR4() { fr.updatePulse(); }

int imuIndex0, imuIndex1, imuIndex2, imuIndex3, imuIndex;
int imu[3];
char imuBuffer[30] = "                             ";
String imuString = String(imuBuffer);

void serialEvent1()
{
    char data = char(Serial1.read());
    if (data == '#')
    {
        imuIndex = 0;
        imuString = String(imuBuffer);
        if (imuString.startsWith("#"))
        {
            imuIndex0 = imuString.indexOf("=");
            imuIndex1 = imuString.indexOf(",");
            imuIndex2 = imuString.lastIndexOf(",");
            imuIndex3 = imuString.lastIndexOf(" ");
            imu[0] = imuString.substring(imuIndex0 + 1, imuIndex1).toInt();
            imu[1] = imuString.substring(imuIndex1 + 1, imuIndex2).toInt();
            imu[2] = imuString.substring(imuIndex2 + 1, imuIndex3).toInt();
        }
        // Serial.print(imu[0]);
        // Serial.print(" ");
        // Serial.print(imu[1]);
        // Serial.print(" ");
        // Serial.println(imu[2]);
    }
    else
        imuIndex++;
    imuBuffer[imuIndex] = data;
}

int reverse = 0;
int angle0;

void serialEvent6()
{
    int data = Serial6.read();
    if (data == 255)
        serial6state = 0;
    else if (data == 254)
        serial6state = 1;
    else if (data == 253)
        serial6state = 2;
    else if (data == 252)
        serial6state = 3;
    else if (serial6state == 0)
    {
        speed = (double)data / 100 * 100; // 100 set as max speed
        speed = max(20, speed);           // 20 set as min speed
    }
    else if (serial6state == 1)
    {
        steer = ((double)data - 90) / 90;
    }
    else if (serial6state == 2)
    {
        green_state = data;

        if (green_state == 3 && !reverse)
        {
            reverse = 1;
            angle0 = imu[0];
        }
    }
}

void setup()
{
    // put your setup code here, to run once:
    attachInterrupt(digitalPinToInterrupt(2), ISR1, CHANGE);
    attachInterrupt(digitalPinToInterrupt(36), ISR2, CHANGE);
    attachInterrupt(digitalPinToInterrupt(5), ISR3, CHANGE);
    attachInterrupt(digitalPinToInterrupt(29), ISR4, CHANGE);
    pinMode(16, OUTPUT);
    pinMode(17, INPUT_PULLUP);
    pinMode(13, OUTPUT);
    Serial.begin(115200);
    Serial1.begin(57600);
    Serial6.begin(57600);
}

void loop()
{
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
            int error = abs((int)imu[0] - angle0);

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
}