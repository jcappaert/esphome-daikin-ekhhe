#pragma once

#include "esphome/core/log.h"

#define DAIKIN_DBG(TAG, ...) ESP_LOGD(TAG, __VA_ARGS__)
#define DAIKIN_WARN(TAG, ...) ESP_LOGW(TAG, __VA_ARGS__)
#define DAIKIN_ERROR(TAG, ...) ESP_LOGE(TAG, __VA_ARGS__)
