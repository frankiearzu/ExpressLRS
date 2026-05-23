/**
 * Adaptation From Fusion Bias Filter
 * by Seb Madgwick
 * @brief Run-time estimation and compensation of gyroscope offset.
 */

#include <Arduino.h>
#include "mpu_biasFilter.h"
#include "logging.h"

/**
 * @brief High-pass filter cutoff frequency in Hz.
 */
#define CUTOFF_FREQUENCY (0.02f)

typedef struct {
    float sampleRate; // Hz
    float stationaryThreshold; // rad per second
    float stationaryPeriod; // seconds
} BiasSettings;

/**
 * @brief Bias structure. All members are private.
 */
typedef struct {
    BiasSettings settings;
    float filterCoefficient;
    unsigned int timeout;
    unsigned int timer;
    VectorFloat offset;
} BiasData;

//------------------------------------------------------------------------------
// Variables

static BiasSettings gyroBiasDefaultSettings = {
    .sampleRate = 100.0f,
    .stationaryThreshold = 3.0f,  // 3-Degress 
    .stationaryPeriod = 3.0f, // 3 cycles of stationary
};

static BiasData bias;

//------------------------------------------------------------------------------
// Functions

/**
 * @brief Initialises the bias structure.
 * @param bias Bias structure.
 */
void gyroBiasInitialise(const float sampleRate) {
    gyroBiasDefaultSettings.sampleRate = sampleRate;
    bias.settings = gyroBiasDefaultSettings;
    bias.filterCoefficient = 2.0f * (float) M_PI  * CUTOFF_FREQUENCY * (1.0f / bias.settings.sampleRate);
    bias.timeout = (unsigned int) (bias.settings.stationaryPeriod * bias.settings.sampleRate);

    bias.timer = 0;
    bias.offset.x = 0;
    bias.offset.y = 0;
    bias.offset.z = 0;

    #if defined(DEBUG_LOG)
    DBGLN("Gyro Bias SampleRate=%f Bias Timeout = %d",sampleRate, bias.timeout);
    char a[15]; sprintf(a, "%1.6f", bias.filterCoefficient);
    char b[15]; sprintf(b, "%1.6f", bias.settings.stationaryThreshold);
    DBGLN("Bias filterCoeficient=%s, Threshold=%s ",a,b);
    #endif
}

/**
 * @brief Updates the bias algorithm and returns the offset-corrected
 * gyroscope. This function must be called for every gyroscope sample at the
 * configured sample rate.
 * @param gyroscope Gyroscope in radians per second.
 * @return Offset-corrected gyroscope in radians per second.
 */
void gyroBiasUpdate(VectorFloat gyroscope) {
    // Apply gyroscope offset
    gyroscope.x -= bias.offset.x;
    gyroscope.y -= bias.offset.y;
    gyroscope.z -= bias.offset.z;

    // Reset timer if gyroscope not stationary
    if ((fabsf(gyroscope.x) > bias.settings.stationaryThreshold) ||
        (fabsf(gyroscope.y) > bias.settings.stationaryThreshold) ||
        (fabsf(gyroscope.z) > bias.settings.stationaryThreshold)) {
        bias.timer = 0;
        return; // Not Stationary
    }

    // Increment timer while gyroscope stationary
    if (bias.timer < bias.timeout) {
        bias.timer++;
        return;
    }

    // Update high-pass filter while timer has elapsed
    bias.offset.x += gyroscope.x * bias.filterCoefficient;
    bias.offset.y += gyroscope.y * bias.filterCoefficient;
    bias.offset.z += gyroscope.z * bias.filterCoefficient;
}

VectorFloat getBiasOffsets() {
    return bias.offset;
}
