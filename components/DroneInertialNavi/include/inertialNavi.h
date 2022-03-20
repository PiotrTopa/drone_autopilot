#include "MPU6050.h"
//#include "MPU6050_6Axis_MotionApps20.h"

#define DMP_REFRESH_PERIOD_MS 100

class RotationVectorFloat
{
public:
    float yaw;
    float pitch;
    float roll;

    RotationVectorFloat();
    RotationVectorFloat(float yaw, float pitch, float roll);
};

class InertialNavi
{
public:
    Quaternion quaternion; // [w, x, y, z]         quaternion container
    VectorFloat gravity;   // [x, y, z]            gravity vector
    VectorFloat ifrAcceleration;
    VectorFloat ifrVelocity;
    VectorFloat ifrPosition;
    RotationVectorFloat rotation;

    InertialNavi();
    void initialize();
    void update();

private:
    MPU6050 mpu;
    uint16_t packetSize = 42; // expected DMP packet size (default is 42 bytes)
    uint16_t fifoCount;       // count of all bytes currently in FIFO
    uint8_t fifoBuffer[64];   // FIFO storage buffer
    uint8_t mpuIntStatus;     // holds actual interrupt status byte from MPU
    void updateRotation();
    void updateIfrVelocityPosition();
};

extern "C" void vTaskDroneInertialNavi(void* pvParameters);

