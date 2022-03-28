#ifndef DRONE_AERO_NAV_H_
#define DRONE_AERO_NAV_H_
#include "BMP280.h"
#include "I2Cdev.h"

#define DMP_REFRESH_PERIOD_MS 100

struct AeroNavRawData
{
    double temperature;
    double pressure;
};

class AeroNav
{
public:
    AeroNav();
    void initialize(I2Cdev *i2cBusInterface);
    void update();
    AeroNavRawData *getRawDataPitot();
    AeroNavRawData *getRawDataStatic();

private:
    I2Cdev *i2cBus;
    BMP280 bmpPitot;
    BMP280 bmpStatic;
    AeroNavRawData rawDataPitot;
    AeroNavRawData rawDataStatic;
};

extern "C" void vTaskDroneAeroNav(void *pvParameters);

#endif