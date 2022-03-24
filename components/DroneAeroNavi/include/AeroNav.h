#include "BMP280.h"
#include "I2Cdev.h"

#define DMP_REFRESH_PERIOD_MS 100

class AeroNav
{
public:
    AeroNav();
    void initialize(I2Cdev* i2cBusInterface);
    void update();

private:
    I2Cdev* i2cBus;
    BMP280 bmp1;
    BMP280 bmp2;
};

extern "C" void vTaskDroneAeroNav(void* pvParameters);
