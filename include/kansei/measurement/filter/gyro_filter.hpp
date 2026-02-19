#pragma once
#include <cmath>
#include "keisan/matrix/vector.hpp"

class GyroFilter
{
public:
    GyroFilter();

    void update(const keisan::Vector<3>& gyro_raw);
    keisan::Vector<3> get_filtered_gyro() const;
    bool isCalibrated() const;

private:
    static constexpr int CALIBRATION_SAMPLES = 500;
    int calibration_counter;
    bool calibrated;
    keisan::Vector<3> gyro_bias;
    keisan::Vector<3> gyro_filtered;

    double lpf_alpha;
    double deadband;
    double bias_adapt_alpha;
    double stationary_threshold;
};

