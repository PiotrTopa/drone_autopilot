#include <stdlib.h>
#include <sys/time.h>
#include "esp_log.h"

#include "DroneNav.h"
#include "AltitudeKalman.h"

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

    dspm::Mat z = dspm::Mat(3, 1);
    z(0, 0) = 200;
    altitudeKalman.setInitialState(z);
}

void DroneNav::update()
{
    // measurmnet vector
    AeroNavRawData *barometricData = aeroNav->getRawDataStatic();
    VectorFloat *ifrAcceleration  = inertialNav->getIfrAcceleration();

    dspm::Mat Z = dspm::Mat(2, 1);
    Z(0, 0) = 44330 * (1.0 - pow(barometricData->pressure / seaLevelPressure, 0.1903));
    Z(1, 0) = ifrAcceleration->z;
    altitudeKalman.updateMeasurement(Z);

    dspm::Mat X = altitudeKalman.getExtrapolatedState();
    // std::cout << "P:" << std::endl
    //           << kP << std::endl;

    printf("%f %f %f ## %f %f\n", X(0, 0), X(1, 0), X(2, 0), Z(0, 0), Z(0, 1));
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
