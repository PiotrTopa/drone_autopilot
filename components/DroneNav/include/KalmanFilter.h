#ifndef DRONE_NAV_ALTITUDE_KALMAN_H_
#define DRONE_NAV_ALTITUDE_KALMAN_H_

#include "esp_dsp.h"

class KalmanFilter
{
public:
    KalmanFilter();
    void initialize();
    dspm::Mat getExtrapolatedState();
    dspm::Mat getState();
    void updateMeasurement(dspm::Mat z);
    void setInitialState(dspm::Mat y);

protected:
    void updateLastMeasurementTime();
    int32_t getTimeDelta();
    virtual dspm::Mat getF(int32_t timeDelta);
    struct timeval lastMeasurementTime;

    dspm::Mat X;
    dspm::Mat P;
    dspm::Mat Qn;
    dspm::Mat H;  // observation matrix H
    dspm::Mat Ht; // observation matrix transposed H
    dspm::Mat R;  // observation matrix H
    dspm::Mat I = dspm::Mat::eye(3);
};

#endif
