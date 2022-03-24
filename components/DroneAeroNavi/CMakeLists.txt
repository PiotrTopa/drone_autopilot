set(COMPONENT_SRCS "src/AeroNav.cpp")
set(COMPONENT_ADD_INCLUDEDIRS "include")
set(COMPONENT_REQUIRES
    BMP280
    I2Cdev
)

register_component()
