#include <Arduino.h>

int imuIndex;
float imu[3];
// char imuBuffer[30] = "                             ";
// String imuString = String(imuBuffer);

String imuString = "";

void serialEvent1()
{
  char data = char(Serial1.read());
  if (data == '#')
  {
    imuIndex = 0;

    // imuString = String(imuBuffer);
    if (imuString.startsWith("#"))
    {
      int imuIndex0 = imuString.indexOf("=");
      int imuIndex1 = imuString.indexOf(",");
      int imuIndex2 = imuString.lastIndexOf(",");
      int imuIndex3 = imuString.lastIndexOf(" ");
      imu[0] = imuString.substring(imuIndex0 + 1, imuIndex1).toFloat();
      imu[1] = imuString.substring(imuIndex1 + 1, imuIndex2).toFloat();
      imu[2] = imuString.substring(imuIndex2 + 1, imuIndex3).toFloat();
    }
    Serial.print(imu[0]);
    Serial.print(" ");
    Serial.print(imu[1]);
    Serial.print(" ");
    Serial.println(imu[2]);
    // imuBuffer = "                             ";
    imuString = "";
  }
  else
    imuIndex++;
  // imuBuffer[imuIndex] = data;
  imuString += data;
}

void setup()
{
  Serial.begin(115200);
  Serial1.begin(57600);
}

void loop()
{
}
