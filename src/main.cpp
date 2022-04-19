#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <driver/i2c.h>
#include <driver/uart.h>

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
#include "GPSM6V2.h"

#define EIGEN_MPL2_ONLY

#define PIN_I2C_SDA GPIO_NUM_32
#define PIN_I2C_SCL GPIO_NUM_33

I2Cdev i2c = I2Cdev(I2C_NUM_0);
InertialNav inertialNav = InertialNav();
AeroNav aeroNav = AeroNav();
DroneNav droneNav = DroneNav();

double pressure, altitude, temperature;

//static const char *TAG = "DRONE";

void start(void)
{
	// i2c.initialize(PIN_I2C_SDA, PIN_I2C_SCL, 400000);

	// inertialNav.initialize(&i2c);
	// xTaskCreate(vTaskDroneInertialNav, "InertialNav", 2048, (void *)&inertialNav, 2, NULL);

	// aeroNav.initialize(&i2c);
	// xTaskCreate(vTaskDroneAeroNav, "AeroNav", 2048, (void *)&aeroNav, 2, NULL);

	// droneNav.initialize(&aeroNav, &inertialNav);
	// xTaskCreate(vTaskDroneNav, "DroneNav", 2 * 4096, (void *)&droneNav, 2, NULL);

	printf("Starting GPS");
	GPSM6V2* gps = new GPSM6V2();
	gps->initialize(UART_NUM_2);
	while(true) {
		gps->update();
		vTaskDelay(100 / portTICK_PERIOD_MS);
	}
	
}

extern "C" void app_main(void)
{
	start();
}