#include "kansei/measurement/filter/gyro_filter.hpp"

GyroFilter::GyroFilter()
{
    calibration_counter = 0;
    calibrated = false;
    gyro_bias = keisan::Vector<3>::zero();
    gyro_filtered = keisan::Vector<3>::zero();

    lpf_alpha = 0.7;
    deadband = 0.02;
    bias_adapt_alpha = 0.001;
    stationary_threshold = 0.05;
}

bool GyroFilter::isCalibrated() const
{
    return calibrated;
}

void GyroFilter::update(const keisan::Vector<3>& gyro_raw)
{
    if (!calibrated)
    {
        gyro_bias += gyro_raw;
        calibration_counter++;

        if (calibration_counter >= CALIBRATION_SAMPLES)
        {
            gyro_bias /= CALIBRATION_SAMPLES;
            calibrated = true;
        }

        return;
    }

    keisan::Vector<3> gyro_unbiased = gyro_raw - gyro_bias;

    double norm =
        std::sqrt(
            gyro_unbiased[0]*gyro_unbiased[0] +
            gyro_unbiased[1]*gyro_unbiased[1] +
            gyro_unbiased[2]*gyro_unbiased[2]
        );

    if (norm < stationary_threshold)
    {
        gyro_bias =
            (1.0 - bias_adapt_alpha) * gyro_bias +
            bias_adapt_alpha * gyro_raw;
    }

    for (int i=0; i<3; i++)
    {
        gyro_filtered[i] =
            lpf_alpha * gyro_filtered[i] +
            (1.0 - lpf_alpha) * gyro_unbiased[i];
    }
 
    for (int i=0; i<3; i++)
    {
        if (std::fabs(gyro_filtered[i]) < deadband)
            gyro_filtered[i] = 0.0;
    }
}

keisan::Vector<3> GyroFilter::get_filtered_gyro() const
{
    return gyro_filtered;
}