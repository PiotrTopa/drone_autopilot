#include <stdlib.h>
#include <sys/time.h>
#include "esp_log.h"

#include "DroneNav.h"

static const char *TAG = "DroneNav";
double seaLevelPressure = 100800.0;

/**
 * Default contructor.
 */
DroneNav::DroneNav()
{
}

void DroneNav::initialize(AeroNav *aeroNavModule, InertialNav *inertialNavModule)
{
    aeroNav = aeroNavModule;
    inertialNav = inertialNavModule;
    altitudeKalman->setInitialState(200);
}

void DroneNav::update()
{
    double dt = 0.1;
    dspm::Mat F(3, 3);
    F(0, 0) = 1;
    F(0, 1) = dt;
    F(0, 2) = 0.5 * dt * dt;
    F(1, 0) = 0;
    F(1, 1) = 1;
    F(1, 2) = dt;
    F(2, 0) = 0;
    F(2, 1) = 0;
    F(2, 2) = 1;

    dspm::Mat kQ = F * kQn * F.t();

    // ---
    // prediction update
    // ---

    // extrapolate new state
    kXprev = kX;
    kX = F * kXprev;

    // extrapolate new uncertainty
    kPprev = kP;
    kP = F * kPprev * F.t() + kQ;

    // ---
    // state update
    // ---

    // measurmnet vector
    AeroNavRawData* barometricData = aeroNav->getRawDataStatic();
    VectorFloat* ifrAcceleration = inertialNav->getIfrAcceleration();

    dspm::Mat kZ = dspm::Mat(2, 1);
    kZ(0, 0) = 44330 * (1.0 - pow(barometricData->pressure / seaLevelPressure, 0.1903));
    kZ(1, 0) = ifrAcceleration->z;

    // compute k gain
    dspm::Mat kHt = kH.t();
    dspm::Mat kK = kP * kHt * (kH * kP * kHt + kR).inverse();

    // update state estimate
    kX = kX + kK * (kZ - kH * kX);

    dspm::Mat kI = dspm::Mat::eye(3);
    // update uncertainty estimate
    kP = (kI - kK * kH)*kP*(kI - kK * kH).t() + (kK * kR * kK.t());

    // std::cout << "X:" << std::endl
    //           << kX << std::endl;
    // std::cout << "z:" << std::endl
    //           << kZ << std::endl;
    // std::cout << "P:" << std::endl
    //           << kP << std::endl;
    //printf("ifrRawAcceleration: %df %df %df\n", inertialNav->localRawAcceleration.x, inertialNav->localRawAcceleration.y, inertialNav->localRawAcceleration.z);
    //printf("ifrRawAcceleration: %df %df %df\n", inertialNav->localRawAcceleration.x, inertialNav->localRawAcceleration.y, inertialNav->localRawAcceleration.z);

    printf("%f %f %f ## %f %f\n", kX(0, 0), kX(1, 0), kX(2, 0), kZ(0, 0), kZ(0, 1));    
}

extern "C" void vTaskDroneNav(void *pvParameters)
{
    DroneNav *droneNav = (DroneNav *)pvParameters;
    while (true)
    {
        droneNav->update();
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}
