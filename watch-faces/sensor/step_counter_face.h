/*
 * MIT License
 *
 * Copyright (c) 2025 Ruben Nic
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#pragma once

/*
 * STEP COUNTING AND LOGGING
 *
 * This watch face works with Movement's built-in tracking of accelerometer state to log activity over time.
 * The watch face shows the number of active minutes counted for each of the last 14 days. Layout:
 *
 *  - Top left is display title (ACT or AC for Activity)
 *  - Top right is the day of the month corresponding to the data point shown on screen.
 *  - Bottom row is the number of active minutes counted on the given day.
 *  - If the display is showing today's active minutes, the SIGNAL indicator is also energized, to remind you
 *    that the accelerometer sensor is sensing, and the watch face is still counting today's active minutes.
 *
 * A short press of the Alarm button moves backwards in the data log, showing yesterday's active minutes,
 * then the day before, etc. going back 14 days.
 *
 */

#include "movement.h"
#include "watch.h"
#include "pedometer.h"

#define ACTIVITY_LOGGING_NUM_DAYS (14)

typedef struct {
    uint32_t activity_log[ACTIVITY_LOGGING_NUM_DAYS];   // the activity log
    uint16_t data_points;                               // the number of days logged
    uint8_t display_index;                              // the index we are displaying on screen
    pedometer_t *pedometer; // Pedometer instance to track steps
} step_counter_state_t;

void step_counter_face_setup(uint8_t watch_face_index, void ** context_ptr);
void step_counter_face_activate(void *context);
bool step_counter_face_loop(movement_event_t event, void *context);
void step_counter_face_resign(void *context);
movement_watch_face_advisory_t step_counter_face_advise(void *context);

#define step_counter_face ((const watch_face_t){ \
    step_counter_face_setup, \
    step_counter_face_activate, \
    step_counter_face_loop, \
    step_counter_face_resign, \
    step_counter_face_advise, \
})
