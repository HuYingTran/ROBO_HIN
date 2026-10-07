#pragma once

// Logging nhẹ: HIN_LOG_LEVEL 0 = tắt, 1 = lỗi, 2 = cảnh báo, 3 = thông tin, 4 = debug.
// Không gọi trong ISR.
#ifndef HIN_LOG_LEVEL
#define HIN_LOG_LEVEL 2
#endif

#if defined(ARDUINO)
#include <Arduino.h>
#define HIN_LOG_IMPL(tag, fmt, ...) Serial.printf("[" tag "] " fmt "\n", ##__VA_ARGS__)
#else
#include <cstdio>
#define HIN_LOG_IMPL(tag, fmt, ...) std::printf("[" tag "] " fmt "\n", ##__VA_ARGS__)
#endif

#if HIN_LOG_LEVEL >= 1
#define HIN_LOGE(fmt, ...) HIN_LOG_IMPL("E", fmt, ##__VA_ARGS__)
#else
#define HIN_LOGE(...)
#endif
#if HIN_LOG_LEVEL >= 2
#define HIN_LOGW(fmt, ...) HIN_LOG_IMPL("W", fmt, ##__VA_ARGS__)
#else
#define HIN_LOGW(...)
#endif
#if HIN_LOG_LEVEL >= 3
#define HIN_LOGI(fmt, ...) HIN_LOG_IMPL("I", fmt, ##__VA_ARGS__)
#else
#define HIN_LOGI(...)
#endif
#if HIN_LOG_LEVEL >= 4
#define HIN_LOGD(fmt, ...) HIN_LOG_IMPL("D", fmt, ##__VA_ARGS__)
#else
#define HIN_LOGD(...)
#endif
