#include "AltitudeKalman.h"

void AltitudeKalman::setInitialState(float altitude)
{
    kX.clear();
    kX(0, 0) = altitude;

    kP(0, 0) = 0.0218685;
    kP(0, 1) = 0.00620678;
    kP(0, 2) = 0.00173698;
    kP(1, 0) = 0.00620679;
    kP(1, 1) = 0.00176162;
    kP(1, 2) = 0.000492993;
    kP(2, 0) = 0.00173698;
    kP(2, 1) = 0.000492993;
    kP(2, 2) = 0.000137965;

    dspm::Mat kV = dspm::Mat(2, 1); // X_(-1)
    kV(0, 0) = 0.63;
    kV(1, 0) = 0.05;

    kR = kV * kV.t();

    kH(0, 0) = 1;
    kH(0, 1) = 0;
    kH(0, 2) = 0;
    kH(1, 0) = 0;
    kH(1, 1) = 0;
    kH(1, 2) = 1;

    kQn.clear();
    kQn(2, 2) = 0.1;
    kQn(3, 3) = 0.1;
}

void AltitudeKalman::setInitialState() {

}
