#pragma once

#include "esphome/core/log.h"

#ifndef DAIKIN_EKHHE_DEBUG
#define DAIKIN_EKHHE_DEBUG 0
#endif

#if DAIKIN_EKHHE_DEBUG
#define DAIKIN_HEALTH(TAG, ...) ESP_LOGI(TAG, __VA_ARGS__)
#define DAIKIN_DBG(TAG, ...) ESP_LOGD(TAG, __VA_ARGS__)
#else
#define DAIKIN_HEALTH(TAG, ...) ESP_LOGI(TAG, __VA_ARGS__)
#define DAIKIN_DBG(TAG, ...) do { } while (0)
#endif

#define DAIKIN_WARN(TAG, ...) ESP_LOGW(TAG, __VA_ARGS__)
#define DAIKIN_ERROR(TAG, ...) ESP_LOGE(TAG, __VA_ARGS__)
