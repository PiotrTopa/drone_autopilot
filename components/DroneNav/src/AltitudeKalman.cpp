#include "AltitudeKalman.h"

void AltitudeKalman::setInitialState(dspm::Mat initialX)
{
    // Set the initial state
    X = initialX;
    
    // Set initial uncertainty
    // this also keeps the state, and those values has impact on the time
    // required to stabilize the filter
    P(0, 0) = 0.0218685;
    P(0, 1) = 0.00620678;
    P(0, 2) = 0.00173698;
    P(1, 0) = 0.00620679;
    P(1, 1) = 0.00176162;
    P(1, 2) = 0.000492993;
    P(2, 0) = 0.00173698;
    P(2, 1) = 0.000492993;
    P(2, 2) = 0.000137965;

    // Set measurement matrix
    H(0, 0) = 1;
    H(0, 1) = 0;
    H(0, 2) = 0;
    H(1, 0) = 0;
    H(1, 1) = 0;
    H(1, 2) = 1;
    Ht = H.t();

    // Set measurement noise
    dspm::Mat V = dspm::Mat(2, 1); // X_(-1)
    V(0, 0) = 0.63;
    V(1, 0) = 0.05;
    R = V * V.t();

    // Set process noise
    Qn.clear();
    Qn(2, 2) = 0.1;
    Qn(3, 3) = 0.1;
}

void AltitudeKalman::updateLastMeasurementTime() {
    gettimeofday(&lastMeasurementTime, NULL);
}

int32_t AltitudeKalman::getTimeDelta() {
    struct timeval nowTime;
    gettimeofday(&nowTime, NULL);
    return (int32_t)(nowTime.tv_sec - lastMeasurementTime.tv_sec) * 1000000L + (int32_t)(nowTime.tv_usec - lastMeasurementTime.tv_usec);
}

/**
 * Return process model.
 */
dspm::Mat AltitudeKalman::getF(int32_t timeDelta) {
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
dspm::Mat AltitudeKalman::getExtrapolatedState() {
    int32_t timeDelta = getTimeDelta();
    return getF(timeDelta) * X;
}

/**
 * Update measurement
 */
void AltitudeKalman::updateMeasurement(dspm::Mat z) {
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
}