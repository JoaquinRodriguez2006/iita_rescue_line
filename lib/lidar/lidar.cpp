#include <Arduino.h>
#include <Wire.h>
#include "VL53L0X.h"
#include "lidar.h"

Lidar::Lidar(TwoWire *bus)
{
    this->_bus = bus;
    _bus->begin();
    _bus->setClock(400000); // use 400 kHz I2C
    _lidar.setBus(_bus);
    _lidar.setTimeout(500);
    _lidar.init();
    _lidar.setMeasurementTimingBudget(33000);
    _lidar.startContinuous(33);
}

int Lidar::getDist()
{
    _distance = _lidar.readReg(false);
    return _distance;
}