set(COMPONENT_SRCS "src/inertialNavi.cpp")
set(COMPONENT_ADD_INCLUDEDIRS "include")
set(COMPONENT_REQUIRES
    MPU6050
    I2Cdev
)

register_component()
