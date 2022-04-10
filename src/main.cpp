#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <driver/i2c.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_vfs.h"
#include "esp_spiffs.h"
#include "esp_dsp.h"

#include "I2Cdev.h"
#include "InertialNav.h"
#include "AeroNav.h"
#include "DroneNav.h"

#define EIGEN_MPL2_ONLY

#define PIN_I2C_SDA GPIO_NUM_32
#define PIN_I2C_SCL GPIO_NUM_33

#define CONFIG_WIDTH 135
#define CONFIG_HEIGHT 240
#define CONFIG_OFFSETX 53
#define CONFIG_OFFSETY 40
#define CONFIG_MOSI_GPIO 19
#define CONFIG_SCLK_GPIO 18
#define CONFIG_CS_GPIO 5
#define CONFIG_DC_GPIO 16
#define CONFIG_RESET_GPIO 23
#define CONFIG_BL_GPIO 4

I2Cdev i2c = I2Cdev(I2C_NUM_0);
InertialNav inertialNav = InertialNav();
AeroNav aeroNav = AeroNav();
DroneNav droneNav = DroneNav();

double pressure, altitude, temperature;

static const char *TAG = "DRONE";

void start(void)
{
	xTaskCreate(update_display, "update_display", 4096, NULL, 2, NULL);

	i2c.initialize(PIN_I2C_SDA, PIN_I2C_SCL, 400000);

	inertialNav.initialize(&i2c);
	xTaskCreate(vTaskDroneInertialNav, "InertialNav", 2048, (void *)&inertialNav, 2, NULL);

	aeroNav.initialize(&i2c);
	xTaskCreate(vTaskDroneAeroNav, "AeroNav", 2048, (void *)&aeroNav, 2, NULL);

	droneNav.initialize(&aeroNav, &inertialNav);
	xTaskCreate(vTaskDroneNav, "DroneNav", 2 * 4096, (void *)&droneNav, 2, NULL);
}

extern "C" void app_main(void)
{
	start();
}