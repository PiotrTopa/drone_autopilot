#include "AltitudeKalman.h"

void AltitudeKalman::setInitialState(float altitude)
{
    // Set the initial state
    X.clear();
    X(0, 0) = altitude;

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
 * Return current extrapolated state
 */ 
void AltitudeKalman::getState() {
    int32_t timeDelta = getTimeDelta();
}
