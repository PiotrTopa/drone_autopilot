#ifndef DRONE_NAV_H_
#define DRONE_NAV_H_

#include "AeroNav.h"
#include "InertialNav.h"
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

    dspm::Mat kX = dspm::Mat(3, 1);     // X
    dspm::Mat kXprev = dspm::Mat(3, 1); // X_(-1)
    dspm::Mat kP = dspm::Mat(3, 3);     // X
    dspm::Mat kPprev = dspm::Mat(3, 3); // X_(-1)

    dspm::Mat kV = dspm::Mat(2, 1); // X_(-1)
    dspm::Mat kQn = dspm::Mat(3, 3);



    dspm::Mat kH = dspm::Mat(2, 3); // observation matrix H
    dspm::Mat kR = dspm::Mat(2, 2); // observation matrix H
};

extern "C" void vTaskDroneNav(void *pvParameters);

#endif
