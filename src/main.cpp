#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <driver/i2c.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_vfs.h"
#include "esp_spiffs.h"

#include "inertialNavi.h"
#include "BMP280.h"
#include "I2Cdev.h"

extern "C"
{
#include "st7789.h"
#include "fontx.h"
}

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
BMP280 bmp;
InertialNavi inertialNavi = InertialNavi();
double pressure, altitude, temperature;
double seaLevelPressure = 103400.0;

static const char *TAG = "ST7789";

void HorizontalTest(TFT_t *dev, FontxFile *fx, int width, int height)
{
	// get font width & height
	uint8_t buffer[FontxGlyphBufSize];
	uint8_t fontWidth;
	uint8_t fontHeight;
	GetFontx(fx, 0, buffer, &fontWidth, &fontHeight);

	uint16_t color;
	lcdFillScreen(dev, BLACK);
	uint8_t ascii[60];

	lcdSetFontDirection(dev, 0);

	color = WHITE;
	sprintf((char *)ascii, "Acc");
	lcdDrawString(dev, fx, 0, fontHeight * 1 - 1, ascii, color);
	sprintf((char *)ascii, "X:%3.2f", inertialNavi.ifrAcceleration.x);
	lcdDrawString(dev, fx, 0, fontHeight * 2 - 1, ascii, color);
	sprintf((char *)ascii, "Y:%3.2f", inertialNavi.ifrAcceleration.y);
	lcdDrawString(dev, fx, 0, fontHeight * 3 - 1, ascii, color);
	sprintf((char *)ascii, "Z:%3.2f", inertialNavi.ifrAcceleration.z);
	lcdDrawString(dev, fx, 0, fontHeight * 4 - 1, ascii, color);

	color = YELLOW;
	sprintf((char *)ascii, "Rot");
	lcdDrawString(dev, fx, 60, fontHeight * 1 - 1, ascii, color);
	sprintf((char *)ascii, "Y:%3.2f", inertialNavi.rotation.yaw * 180/M_PI);
	lcdDrawString(dev, fx, 60, fontHeight * 2 - 1, ascii, color);
	sprintf((char *)ascii, "P:%3.2f", inertialNavi.rotation.pitch * 180/M_PI);
	lcdDrawString(dev, fx, 60, fontHeight * 3 - 1, ascii, color);
	sprintf((char *)ascii, "R:%3.2f", inertialNavi.rotation.roll * 180/M_PI);
	lcdDrawString(dev, fx, 60, fontHeight * 4 - 1, ascii, color);

/**
	color = RED;
	sprintf((char *)ascii, "Velocity");
	lcdDrawString(dev, fx, 0, fontHeight * 6 - 1, ascii, color);
	sprintf((char *)ascii, "X: %3.2f", inertialNavi.ifrVelocity.x);
	lcdDrawString(dev, fx, 0, fontHeight * 7 - 1, ascii, color);
	sprintf((char *)ascii, "Y: %3.2f", inertialNavi.ifrVelocity.y);
	lcdDrawString(dev, fx, 0, fontHeight * 8 - 1, ascii, color);
	sprintf((char *)ascii, "Z: %3.2f", inertialNavi.ifrVelocity.z);
	lcdDrawString(dev, fx, 0, fontHeight * 9 - 1, ascii, color);

	color = GREEN;
	sprintf((char *)ascii, "Position");
	lcdDrawString(dev, fx, 0, fontHeight * 11 - 1, ascii, color);
	sprintf((char *)ascii, "X: %3.1f", inertialNavi.ifrPosition.x);
	lcdDrawString(dev, fx, 0, fontHeight * 12 - 1, ascii, color);
	sprintf((char *)ascii, "Y: %3.1f", inertialNavi.ifrPosition.y);
	lcdDrawString(dev, fx, 0, fontHeight * 13 - 1, ascii, color);
	sprintf((char *)ascii, "Z: %3.1f", inertialNavi.ifrPosition.z);
	lcdDrawString(dev, fx, 0, fontHeight * 14 - 1, ascii, color);
**/

	color = GREEN;
	sprintf((char *)ascii, "Aero");
	lcdDrawString(dev, fx, 0, fontHeight * 11 - 1, ascii, color);
	sprintf((char *)ascii, "Temp: %3.2f C", temperature);
	lcdDrawString(dev, fx, 0, fontHeight * 12 - 1, ascii, color);
	sprintf((char *)ascii, "Pres: %3.2f Pa", pressure);
	lcdDrawString(dev, fx, 0, fontHeight * 13 - 1, ascii, color);
	sprintf((char *)ascii, "Alti: %3.2f m", altitude);
	lcdDrawString(dev, fx, 0, fontHeight * 14 - 1, ascii, color);
}

extern "C" void update_display(void *pvParameters)
{
	TFT_t dev;
	FontxFile fx16G[2];

	esp_vfs_spiffs_conf_t conf = {
		.base_path = "/spiffs",
		.partition_label = NULL,
		.max_files = 10,
		.format_if_mount_failed = true};

	// Use settings defined above toinitialize and mount SPIFFS filesystem.
	// Note: esp_vfs_spiffs_register is anall-in-one convenience function.
	esp_err_t ret = esp_vfs_spiffs_register(&conf);

	if (ret != ESP_OK)
	{
		if (ret == ESP_FAIL)
		{
			ESP_LOGE(TAG, "Failed to mount or format filesystem");
		}
		else if (ret == ESP_ERR_NOT_FOUND)
		{
			ESP_LOGE(TAG, "Failed to find SPIFFS partition");
		}
		else
		{
			ESP_LOGE(TAG, "Failed to initialize SPIFFS (%s)", esp_err_to_name(ret));
		}
		return;
	}

	size_t total = 0, used = 0;
	ret = esp_spiffs_info(NULL, &total, &used);
	if (ret != ESP_OK)
	{
		ESP_LOGE(TAG, "Failed to get SPIFFS partition information (%s)", esp_err_to_name(ret));
	}
	else
	{
		ESP_LOGI(TAG, "Partition size: total: %d, used: %d", total, used);
	}

	// set font file
	InitFontx(fx16G, "/spiffs/ILGH16XB.FNT", ""); // 8x16Dot Gothic

	spi_master_init(&dev, CONFIG_MOSI_GPIO, CONFIG_SCLK_GPIO, CONFIG_CS_GPIO, CONFIG_DC_GPIO, CONFIG_RESET_GPIO, CONFIG_BL_GPIO);
	lcdInit(&dev, CONFIG_WIDTH, CONFIG_HEIGHT, CONFIG_OFFSETX, CONFIG_OFFSETY);

	while (true)
	{
		HorizontalTest(&dev, fx16G, CONFIG_WIDTH, CONFIG_HEIGHT);
		vTaskDelay(50);
	}
}

extern "C" void update_bmp(void *pvParameters) {
	while (true)
	{
		bmp.readRawData();
		temperature = bmp.getTemperatureDouble();
		pressure = bmp.getPressureDouble();
		altitude = 44330 * (1.0 - pow(pressure / seaLevelPressure, 0.1903));
		printf("READING %3.2f %3.2f %3.2f\n", temperature, pressure, altitude);
		vTaskDelay(250/ portTICK_PERIOD_MS);
	}
}


void start(void)
{
	xTaskCreate(update_display, "update_display", 4096, NULL, 2, NULL);

	i2c.initialize(PIN_I2C_SDA, PIN_I2C_SCL, 400000);
	inertialNavi.initialize(&i2c);

	xTaskCreate(vTaskDroneInertialNavi, "InertialNavi", 2048, (void *) &inertialNavi, 2, NULL);

	bmp.initialize(&i2c, 0x77);
	
	bmp.configuration.osTemperature = BMP280_OS_16X;
	bmp.configuration.osPressure = BMP280_OS_16X;
	bmp.configuration.outputDataRata = BMP280_ODR_0_5_MS;
	bmp.configuration.filter = BMP280_FILTER_COEFF_16;
	bmp.powerMode = BMP280_NORMAL_MODE;
	bmp.writeConfiguration();

	xTaskCreate(update_bmp, "update_bmp", 2048, NULL, 2, NULL);
}

extern "C" void app_main(void)
{
	start();
}