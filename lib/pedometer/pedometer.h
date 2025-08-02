#include <stdint.h>

#pragma once

/* PEDOMETER CONSTANTS */
#define _1_SECOND 50 // ODR = 50Hz, 50 samples = 1 second
#define REGULATION_OFF_TIME (_1_SECOND << 1) // 2s
#define RAW_SIZE 4
#define THRESHOLD_SIZE 4
#define WINDOW_SIZE ((RAW_SIZE << 2) + 1)
#define SENSITIVITY 400 // 400 = 0.1g, for 2g range
#define INIT_OFFSET_VALUE 4000 // 1g, for 2g range
#define INIT_VALUE_MIN 0 // 0g

typedef struct {
    int32_t raw[RAW_SIZE];                    // 4 elements
    int32_t threshold[THRESHOLD_SIZE];        // 4 elements
    int32_t window[WINDOW_SIZE];              // 17 elements ((4 << 2) + 1 = 17)
    int8_t idx_window_min, idx_window_max, idx_threshold, idx_buffer, idx_average;
    int8_t step_samples, max_min_samples, possible_steps;
    uint8_t flag_max, flag_threshold, flag_threshold_counter, step_counting_mode;
    int32_t count_steps;
    uint32_t last_max, last_min, filter_mean;
    uint32_t buffer_dynamic_threshold, new_threshold, old_threshold;
} pedometer_t;
// Current size of pedometer_t is
// vars: 7 uint32_t (28 bytes) + 4 int8_t (4 bytes) + 5 uint8_t (5 bytes) = 37 bytes
// array: 4 uint32_t (16 bytes) + 4 int32_t (16 bytes) + 17 int32_t (68 bytes) = 100 bytes
// Total: 37 + 100 = 137 bytes

void pedometer_init(pedometer_t *p);
int32_t pedometer_step(pedometer_t *p, int16_t x, int16_t y, int16_t z);
