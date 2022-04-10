#include <stdlib.h>
#include <sys/time.h>
#include "esp_log.h"

#include "AeroNav.h"

static const char *TAG = "AeroNav";
// double seaLevelPressure = 103400.0;

/**
 * Default contructor.
 */
AeroNav::AeroNav()
{
}

/**
 * Initializes both sensors
 */
void AeroNav::initialize(I2Cdev *i2cBusInterface)
{
    i2cBus = i2cBusInterface;

    ESP_LOGI(TAG, "Initialization");

    // configure sensor in Dynamic tube
    bmpDynamic.initialize(i2cBus, BMP280_I2C_ADDR_PRIM);
    bmpDynamic.configuration.osTemperature = BMP280_OS_16X;
    bmpDynamic.configuration.osPressure = BMP280_OS_16X;
    bmpDynamic.configuration.outputDataRata = BMP280_ODR_0_5_MS;
    bmpDynamic.configuration.filter = BMP280_FILTER_COEFF_16;
    bmpDynamic.powerMode = BMP280_NORMAL_MODE;
    bmpDynamic.writeConfiguration();

    // configure static sensor
    bmpStatic.initialize(i2cBus, BMP280_I2C_ADDR_SEC);
    bmpStatic.configuration.osTemperature = BMP280_OS_16X;
    bmpStatic.configuration.osPressure = BMP280_OS_16X;
    bmpStatic.configuration.outputDataRata = BMP280_ODR_0_5_MS;
    // bmpStatic.configuration.filter = BMP280_FILTER_COEFF_16;
    bmpStatic.powerMode = BMP280_NORMAL_MODE;
    bmpStatic.writeConfiguration();

    ESP_LOGI(TAG, "Initialization completed");
}

/**
 * Update sensors values
 */
void AeroNav::update()
{
    bmpDynamic.readRawData();
    bmpStatic.readRawData();
    rawDataDynamic.temperature = bmpDynamic.getTemperatureDouble();
    rawDataDynamic.pressure = bmpDynamic.getPressureDouble();
    rawDataStatic.temperature = bmpStatic.getTemperatureDouble();
    rawDataStatic.pressure = bmpStatic.getPressureDouble();
}

/**
 * Return raw data from sensor in Dynamic tube
 */
AeroNavRawData *AeroNav::getRawDataDynamic()
{
    return &rawDataDynamic;
}

/**
 * Return raw data from static pressure sensor
 */
AeroNavRawData *AeroNav::getRawDataStatic()
{
    return &rawDataStatic;
}

extern "C" void vTaskDroneAeroNav(void *pvParameters)
{
    AeroNav *nav = (AeroNav *)pvParameters;
    while (true)
    {
        nav->update();
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}
