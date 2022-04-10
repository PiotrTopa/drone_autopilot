#include <stdlib.h>
#include <sys/time.h>
#include "esp_log.h"

#include "DroneNav.h"
#include "AltitudeKalman.h"

//static const char *TAG = "DroneNav";
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

double DroneNav::getAltitude() {
    return altitude;
}

double DroneNav::getVerticalSpeed() {
    return verticalSpeed;
}

double DroneNav::getAirSpeed() {
    return airSpeed;
}

void DroneNav::update()
{
    // measurmnet vector
    AeroNavRawData *barometricDataStatic = aeroNav->getRawDataStatic();
    //AeroNavRawData *barometricDataDynamic = aeroNav->getRawDataDynamic();
    VectorFloat *ifrAcceleration  = inertialNav->getIfrAcceleration();

    dspm::Mat Z = dspm::Mat(2, 1);
    Z(0, 0) = 44330 * (1.0 - pow(barometricDataStatic->pressure / seaLevelPressure, 0.1903));
    Z(1, 0) = ifrAcceleration->z;
    altitudeKalman.updateMeasurement(Z);

    dspm::Mat X = altitudeKalman.getState();
    altitude = X(0, 0);
    verticalSpeed = X(1, 0);
    
    // std::cout << "P:" << std::endl
    //           << kP << std::endl;

    printf("%f %f\n", altitude, verticalSpeed);
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
