#ifndef DRONE_NAV_ALTITUDE_KALMAN_H_
#define DRONE_NAV_ALTITUDE_KALMAN_H_

class AltitudeKalman
{
public:
    AltitudeKalman();
    void initialize(AeroNav *aeroNavModule, InertialNav *inertialNavModule);
    
private:
    void updateLastMeasurementTime();
    int32_t getTimeDelta();
    dspm::Mat getF(int32_t timeDelta);
    void update(dspm::Mat z);


    AeroNav *aeroNav;
    InertialNav *inertialNav;
    struct timeval lastMeasurementTime;

    dspm::Mat X = dspm::Mat(3, 1);     // X
    dspm::Mat P = dspm::Mat(3, 3);     // X
    dspm::Mat Qn = dspm::Mat(3, 3);
    dspm::Mat H = dspm::Mat(2, 3); // observation matrix H
    dspm::Mat Ht = dspm::Mat(3, 2); // observation matrix transposed H
    dspm::Mat R = dspm::Mat(2, 2); // observation matrix H
    dspm::Mat I = dspm::Mat::eye(3);
}

#endif
