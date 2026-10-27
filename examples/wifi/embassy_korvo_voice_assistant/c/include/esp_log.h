// ESP-IDF's log macros, printed with picolibc's `printf`.
#pragma once

#include <stdio.h>

#define ESP_LOGE(tag, format, ...) printf("E %s: " format "\n", tag, ##__VA_ARGS__)
#define ESP_LOGW(tag, format, ...) printf("W %s: " format "\n", tag, ##__VA_ARGS__)
#define ESP_LOGI(tag, format, ...) printf("I %s: " format "\n", tag, ##__VA_ARGS__)
