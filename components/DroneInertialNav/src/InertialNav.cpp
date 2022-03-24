#include <stdlib.h>
#include <sys/time.h>
#include "esp_log.h"

#include "InertialNav.h"
#include "MPU6050_6Axis_MotionApps20.h"


static const char *TAG = "InertialNav";

/**
 * Default contructor.
 */
InertialNav::InertialNav()
{

}

/**
 * Initializes MPU6050
 */
void InertialNav::initialize(I2Cdev* i2cBusInterface)
{
    i2cBus = i2cBusInterface;

    ESP_LOGI(TAG, "Initialization");
    mpu.initialize(i2cBus, MPU6050_DEFAULT_ADDRESS);

    uint8_t devId = mpu.getDeviceID();
    ESP_LOGI(TAG, "DevID: %x", devId);

    mpu.CalibrateGyro();
    mpu.CalibrateAccel();
    mpu.dmpInitialize();

    //mpu.setXGyroOffset(CONFIG_X_GYRO_OFFSET);
    //mpu.setYGyroOffset(CONFIG_Y_GYRO_OFFSET);
    //mpu.setZGyroOffset(CONFIG_Z_GYRO_OFFSET);
    //mpu.setZAccelOffset(CONFIG_Z_ACCEL_OFFSET);
    mpu.setDMPEnabled(true);
    ESP_LOGI(TAG, "Initialization completed");
}

void InertialNav::updateRotation()
{
    float ypr[3];
    mpu.dmpGetYawPitchRoll(ypr, &quaternion, &gravity);
    rotation.yaw = ypr[0];
    rotation.pitch = ypr[1];
    rotation.roll = ypr[2];
}

void InertialNav::updateIfrVelocityPosition()
{
    struct timeval tv_now;
    gettimeofday(&tv_now, NULL);
    if ((int64_t)tv_now.tv_sec % 30 == 0)
    {
        ifrVelocity = VectorFloat();
        ifrPosition = VectorFloat();
    }

    float delta = (float) DMP_REFRESH_PERIOD_MS / 1000.;
    ifrVelocity.x += delta * (float)ifrAcceleration.x;
    ifrVelocity.y += delta * (float)ifrAcceleration.y;
    ifrVelocity.z += delta * (float)ifrAcceleration.z;

    ifrPosition.x += delta * (float)ifrVelocity.x;
    ifrPosition.y += delta * (float)ifrVelocity.y;
    ifrPosition.z += delta * (float)ifrVelocity.z;
}

/**
 * Reads FIFO data and updates local variables to let them be used
 * by other components.
 */
void InertialNav::update()
{
    mpuIntStatus = mpu.getIntStatus();
    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    if ((mpuIntStatus & 0x10) || fifoCount == 1024)
    {
        // reset so we can continue cleanly
        mpu.resetFIFO();
    }
    else if (mpuIntStatus & 0x02)
    {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize)
        {
            fifoCount = mpu.getFIFOCount();
        }

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);

        // read and calculate data
        mpu.dmpGetQuaternion(&quaternion, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &quaternion);

        // read acceleration, compute to m/s
        VectorInt16 localRawAcceleration, linearRawAcceleration, ifrRawAcceleration;
        mpu.dmpGetAccel(&localRawAcceleration, fifoBuffer);
        mpu.dmpGetLinearAccel(&linearRawAcceleration, &localRawAcceleration, &gravity);
        mpu.dmpGetLinearAccelInWorld(&ifrRawAcceleration, &linearRawAcceleration, &quaternion);

        ifrAcceleration = VectorFloat(
            (float)ifrRawAcceleration.x / 835.f,
            (float)ifrRawAcceleration.y / 835.f,
            (float)ifrRawAcceleration.z / 835.f
        );

        updateRotation();
        updateIfrVelocityPosition();
    }

    // Best result is to match with DMP refresh rate
    //  Its last value in components/MPU6050/MPU6050_6Axis_MotionApps20.h file line 310
    //  Now its 0x13, which means DMP is refreshed with 10Hz rate
    vTaskDelay(DMP_REFRESH_PERIOD_MS / portTICK_PERIOD_MS);
}

extern "C" void vTaskDroneInertialNav(void *pvParameters)
{
    InertialNav *navi = (InertialNav *)pvParameters;
    while (true)
    {
        navi->update();
    }
    vTaskDelete(NULL);
}

/**
 * Default constructor
 */
RotationVectorFloat::RotationVectorFloat()
{
    yaw = 0.;
    pitch = 0.;
    roll = 0.;
}

/**
 * Parametrized constructor
 */
RotationVectorFloat::RotationVectorFloat(float y, float p, float r)
{
    yaw = y;
    pitch = p;
    roll = r;
}
