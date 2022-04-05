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
    double dt = (double) timeDelta / 1000000.0;
    dspm::Mat F(3, 3);
    F(0, 0) = 1;
    F(0, 1) = dt;
    F(0, 2) = 0.5 * dt * dt;
    F(1, 0) = 0;
    F(1, 1) = 1;
    F(1, 2) = dt;
    F(2, 0) = 0;
    F(2, 1) = 0;
    F(2, 2) = 1;
    return F;
}

/** 
 * Return current extrapolated state
 */ 
dspm::Mat KalmanFilter::getExtrapolatedState() {
    int32_t timeDelta = getTimeDelta();
    return getF(timeDelta) * X;
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