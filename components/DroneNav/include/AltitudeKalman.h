#ifndef DRONE_NAV_ALTITUDE_KALMAN_H_
#define DRONE_NAV_ALTITUDE_KALMAN_H_

class AltitudeKalman
{
public:
    AltitudeKalman();
    void initialize(AeroNav *aeroNavModule, InertialNav *inertialNavModule);
    
private:
    AeroNav *aeroNav;
    InertialNav *inertialNav;

    dspm::Mat kX = dspm::Mat(3, 1);     // X
    dspm::Mat kP = dspm::Mat(3, 3);     // X
    dspm::Mat kQn = dspm::Mat(3, 3);
    dspm::Mat kH = dspm::Mat(2, 3); // observation matrix H
    dspm::Mat kHt = dspm::Mat(3, 2); // observation matrix transposed H
    dspm::Mat kR = dspm::Mat(2, 2); // observation matrix H    
}

#endif