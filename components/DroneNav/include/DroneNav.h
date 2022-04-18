#ifndef DRONE_NAV_H_
#define DRONE_NAV_H_

#include "AeroNav.h"
#include "InertialNav.h"
#include "AltitudeKalman.h"
#include "esp_dsp.h"

#define CONFIG_AIR_TEMPERATURE_GAIN 0.05
#define CONFIG_AIR_SPEED_GAIN 0.10
#define CONFIG_AIR_GAS_CONST 287.058



class DroneNav
{
public:
    DroneNav();
    void initialize(AeroNav *aeroNavModule, InertialNav *inertialNavModule);
    void update();

    double getAltitude();
    double getVerticalSpeed();
    double getAirSpeed();

private:
    AeroNav *aeroNav;
    InertialNav *inertialNav;

    AltitudeKalman altitudeKalman;

    double altitude = 0;
    double verticalSpeed = 0;
    double airSpeed = 0.0;
    double airTemperature = 0;
};

extern "C" void vTaskDroneNav(void *pvParameters);

#endif
