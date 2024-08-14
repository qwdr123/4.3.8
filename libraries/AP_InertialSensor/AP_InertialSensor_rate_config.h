#pragma once

#include <AP_Vehicle/AP_Vehicle_Type.h>
#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_InertialSensor/AP_InertialSensor_config.h>

#ifndef AP_INERTIALSENSOR_FAST_SAMPLE_WINDOW_ENABLED
#define AP_INERTIALSENSOR_FAST_SAMPLE_WINDOW_ENABLED (AP_INERTIALSENSOR_ENABLED && HAL_INS_RATE_LOOP && APM_BUILD_TYPE(APM_BUILD_ArduCopter))
#endif
