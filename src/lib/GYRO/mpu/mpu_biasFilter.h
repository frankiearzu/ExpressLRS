#pragma once
#include "gyro_types.h"

void gyroBiasInitialise(const float sampleRate);
void gyroBiasUpdate(VectorFloat gyroscope);
VectorFloat getBiasOffsets();
