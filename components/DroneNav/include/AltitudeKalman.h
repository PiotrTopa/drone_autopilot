#ifndef DRONE_NAV_KALMAN_FILTER_H_
#define DRONE_NAV_KALMAN_FILTER_H_
#include "KalmanFilter.h"

#include "esp_dsp.h"

class AltitudeKalman : public KalmanFilter
{
public:
    AltitudeKalman();
protected:
    dspm::Mat getF(int32_t timeDelta);
};

#endif
