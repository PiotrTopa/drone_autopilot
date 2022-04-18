#include "AltitudeKalman.h"

AltitudeKalman::AltitudeKalman() {
    // Set initial uncertainty
    // this also keeps the state, and those values has impact on the time
    // required to stabilize the filter
    P = dspm::Mat(3, 3); 
    P(0, 0) = 2.387716248631477e-02;
    P(0, 1) = 6.778599228709936e-03;
    P(0, 2) = 1.895012916065753e-03;
    
    P(1, 0) = 6.778594572097063e-03;
    P(1, 1) = 1.924409065395594e-03;
    P(1, 2) = 5.379837239161134e-04;
    
    P(2, 0) = 1.895012916065753e-03;
    P(2, 1) = 5.379840731620789e-04;
    P(2, 2) = 1.503979583503678e-04;

    // Set measurement matrix
    H = dspm::Mat(2, 3);
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
    Qn = dspm::Mat(3, 3);
    Qn.clear();
    Qn(2, 2) = 0.01;
    Qn(3, 3) = 0.1;
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