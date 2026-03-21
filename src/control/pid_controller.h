#pragma once

#include "system/shared_types.h"

void initPIDController(PIDController& pid,
                       float kp,
                       float ki,
                       float kd,
                       float outputMin,
                       float outputMax);

void resetPIDController(PIDController& pid);

float computePID(PIDController& pid,
                 float targetFrequencyHz,
                 const PitchResult& pitch);

MotorCommand buildMotorCommand(float pidOutput);
