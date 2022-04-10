#include "KalmanFilter.h"

KalmanFilter::KalmanFilter() {

}

void KalmanFilter::setInitialState(dspm::Mat initialX)
{
    X = initialX;
}

void KalmanFilter::updateLastMeasurementTime() {
    gettimeofday(&lastMeasurementTime, NULL);
}

int32_t KalmanFilter::getTimeDelta() {
    struct timeval nowTime;
    gettimeofday(&nowTime, NULL);
    return (int32_t)(nowTime.tv_sec - lastMeasurementTime.tv_sec) * 1000000L + (int32_t)(nowTime.tv_usec - lastMeasurementTime.tv_usec);
}

/**
 * Return process model.
 */
dspm::Mat KalmanFilter::getF(int32_t timeDelta) {
    return dspm::Mat::ones(3);
}

/** 
 * Return current extrapolated state
 */ 
dspm::Mat KalmanFilter::getExtrapolatedState() {
    int32_t timeDelta = getTimeDelta();
    return getF(timeDelta) * X;
}

/** 
 * Return last calculated state
 */ 
dspm::Mat KalmanFilter::getState() {
    return X;
}

/**
 * Update measurement
 */
void KalmanFilter::updateMeasurement(dspm::Mat z) {
    dspm::Mat F = getF(getTimeDelta());

    // calculate process uncertainty
    dspm::Mat Q = F * Qn * F.t();

    // calculate state extrapolation
    X = F * X;

    // calculate covariance extrapolation
    P = F * P * F.t() + Q;

    // compute Kalman gain
    dspm::Mat K = P * Ht * (H * P * Ht + R).inverse();

    // update state estimate
    X = X + K * (z - H * X);

    // update uncertainty estimate
    P = (I - K * H) * P * (I - K * H).t() + (K * R * K.t());

    updateLastMeasurementTime();
}