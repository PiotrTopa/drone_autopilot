#include <stdlib.h>
#include <sys/time.h>
#include "esp_log.h"

#include "DroneNav.h"
#include "AltitudeKalman.h"

//static const char *TAG = "DroneNav";
double seaLevelPressure = 102100.0;

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
    AeroNavRawData *barometricDataDynamic = aeroNav->getRawDataDynamic();
    VectorFloat *ifrAcceleration  = inertialNav->getIfrAcceleration();

    /* Update Kalman filter for altitude measurement */
    dspm::Mat Z = dspm::Mat(2, 1);
    Z(0, 0) = 44330 * (1.0 - pow(barometricDataStatic->pressure / seaLevelPressure, 0.1903));
    Z(1, 0) = ifrAcceleration->z;
    altitudeKalman.updateMeasurement(Z);

    /* Read altitude and vertical speed from Kalman filter */
    dspm::Mat X = altitudeKalman.getMeasuredState();
    altitude = X(0, 0);
    verticalSpeed = X(1, 0);

    /* Air temperature reading / filter update */
    double measuredAirTemperature = barometricDataStatic->temperature;
    if(!isnan(measuredAirTemperature)) {
        airTemperature += (CONFIG_AIR_TEMPERATURE_GAIN) * (measuredAirTemperature - airTemperature);
    }

    /* Air speed reading / calculation / filter update */
    double airDensity = barometricDataStatic->pressure / (CONFIG_AIR_GAS_CONST * (airTemperature + 273.15));
    double measuredAirSpeed = sqrt(2 * (barometricDataDynamic->pressure - barometricDataStatic->pressure) / airDensity);
    if(!isnan(measuredAirSpeed)) {
        airSpeed += (CONFIG_AIR_SPEED_GAIN) * (measuredAirSpeed - airSpeed);
    }
    
    printf("alti: %f vrtSpd: %f tmp: %f airSpd: %f %f\n", altitude, verticalSpeed, airTemperature, measuredAirSpeed, airSpeed);
    //dspm::Mat P = altitudeKalman.getP();
    //printf("%.15e %.15e %.15e %.15e %.15e %.15e %.15e %.15e %.15e\n", P(0, 0), P(0, 1), P(0, 2), P(1, 0), P(1, 1), P(1, 2), P(2, 0), P(2, 1), P(2, 2));
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
