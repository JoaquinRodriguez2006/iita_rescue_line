#ifndef lidar_h
#define lidar_h

#include <Arduino.h>
#include <Wire.h>
#include "VL53L0X.h"

class Lidar
{
public:
    Lidar(TwoWire *bus);
    int getDist();

private:
    int _distance;
    TwoWire *_bus;
    VL53L0X _lidar;
};

#endif