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

private:
    AeroNav *aeroNav;
    InertialNav *inertialNav;

    AltitudeKalman altitudeKalman;
};

extern "C" void vTaskDroneNav(void *pvParameters);

#endif
