#ifndef DRONE_NAV_H_
#define DRONE_NAV_H_

#include "AeroNav.h"
#include "InertialNav.h"
#include "AltitudeKalman.h"
#include "esp_dsp.h"

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

    double altitude;
    double verticalSpeed;
    double airSpeed;
};

extern "C" void vTaskDroneNav(void *pvParameters);

#endif
