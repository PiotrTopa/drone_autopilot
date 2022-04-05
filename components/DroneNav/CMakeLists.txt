set(COMPONENT_SRCS "src/DroneNav.cpp" "src/AltitudeKalman.cpp" "src/KalmanFilter.cpp")
set(COMPONENT_ADD_INCLUDEDIRS "include")
set(COMPONENT_REQUIRES
    DroneAeroNav
    DroneInertialNav
    esp-dsp
)

register_component()
