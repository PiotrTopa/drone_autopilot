#include <stdlib.h>
#include <sys/time.h>
#include "esp_log.h"

#include "AeroNav.h"

static const char *TAG = "AeroNav";
double seaLevelPressure = 103400.0;

/**
 * Default contructor.
 */
AeroNav::AeroNav()
{

}

/**
 * Initializes both sensors
 */
void AeroNav::initialize(I2Cdev* i2cBusInterface)
{
    i2cBus = i2cBusInterface;

    ESP_LOGI(TAG, "Initialization");
    
    // configure sensor
    bmp1.initialize(i2cBus, BMP280_I2C_ADDR_PRIM);
    bmp1.configuration.osTemperature = BMP280_OS_16X;
	bmp1.configuration.osPressure = BMP280_OS_16X;
	bmp1.configuration.outputDataRata = BMP280_ODR_0_5_MS;
	bmp1.configuration.filter = BMP280_FILTER_COEFF_16;
	bmp1.powerMode = BMP280_NORMAL_MODE;
	bmp1.writeConfiguration();

    // configure sensor
    bmp2.initialize(i2cBus, BMP280_I2C_ADDR_SEC);
    bmp2.configuration.osTemperature = BMP280_OS_16X;
	bmp2.configuration.osPressure = BMP280_OS_16X;
	bmp2.configuration.outputDataRata = BMP280_ODR_0_5_MS;
	//bmp2.configuration.filter = BMP280_FILTER_COEFF_16;
	bmp2.powerMode = BMP280_NORMAL_MODE;
	bmp2.writeConfiguration();

    ESP_LOGI(TAG, "Initialization completed");
}

/**
 * Update sensors values
 */
void AeroNav::update()
{
    double temp1, temp2, press1, press2, alti1, alti2;
    bmp1.readRawData();
    bmp2.readRawData();
    temp1 = bmp1.getTemperatureDouble();
    temp2 = bmp2.getTemperatureDouble();
    press1 = bmp1.getPressureDouble();
    press2 = bmp2.getPressureDouble();
    alti1 = 44330 * (1.0 - pow(press1 / seaLevelPressure, 0.1903));
    alti2 = 44330 * (1.0 - pow(press2 / seaLevelPressure, 0.1903));

    printf("T1: %3.2f T2: %3.2f dT: %3.2f\n", temp1, temp2, (temp2 - temp1));
    printf("P1: %3.2f P2: %3.2f dP: %3.2f\n", press1, press2, (press2 - press1));
    printf("A1: %3.2f A2: %3.2f dA: %3.2f\n", alti1, alti2, (alti2 - alti1));
		// temperature = bmp.getTemperatureDouble();
		// pressure = bmp.getPressureDouble();
		// altitude = 44330 * (1.0 - pow(pressure / seaLevelPressure, 0.1903));
		// printf("Temp: %3.2f C; Press: %3.2f Pa; Alti: %3.2f m\n", temperature, pressure, altitude);
		// vTaskDelay(250/ portTICK_PERIOD_MS);
    vTaskDelay(250 / portTICK_PERIOD_MS);
}

extern "C" void vTaskDroneAeroNav(void *pvParameters)
{
    AeroNav *nav = (AeroNav *)pvParameters;
    while (true)
    {
        nav->update();
    }
    vTaskDelete(NULL);
}

