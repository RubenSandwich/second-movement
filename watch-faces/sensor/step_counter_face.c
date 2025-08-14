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

#include <stdlib.h>
#include <string.h>
#include "step_counter_face.h"
#include "filesystem.h"
#include "watch.h"
#include "watch_utility.h"

static void _step_counter_face_update_display(step_counter_state_t *state) {
    char buf[8];
    watch_date_time_t timestamp = movement_get_local_date_time();

    watch_display_text_with_fallback(WATCH_POSITION_TOP_LEFT, "STP", "SP");

    if (state->display_index == 0) {
        // if we are at today, just show the count so far
        snprintf(buf, 8, "%2d", timestamp.unit.day);
        watch_display_text(WATCH_POSITION_TOP_RIGHT, buf);
        snprintf(buf, 8, "%4lu  ", state->pedometer->counted_steps);
        watch_display_text(WATCH_POSITION_BOTTOM, buf);

        // also indicate that this is the active day — we are still sensing active minutes!
        watch_set_indicator(WATCH_INDICATOR_SIGNAL);
    } else {
        // otherwise we need to go into the log.
        watch_clear_indicator(WATCH_INDICATOR_SIGNAL);
        int32_t pos = ((int16_t)state->data_points - (int32_t)state->display_index) % ACTIVITY_LOGGING_NUM_DAYS;
        // get day of month for today - display_index
        uint32_t unixtime = watch_utility_date_time_to_unix_time(timestamp, movement_get_current_timezone_offset());
        unixtime -= 86400 * state->display_index;
        timestamp = watch_utility_date_time_from_unix_time(unixtime, movement_get_current_timezone_offset());

        // display date
        snprintf(buf, 8, "%2d", timestamp.unit.day);
        watch_display_text(WATCH_POSITION_TOP_RIGHT, buf);

        if (pos < 0) {
            // no data at this index
            watch_display_text(WATCH_POSITION_BOTTOM, "no dat");
        } else {
            // we are displaying the step for that day
            snprintf(buf, 8, "%4lu  ", state->activity_log[pos]);
            watch_display_text(WATCH_POSITION_BOTTOM, buf);
        }
    }
}

void step_counter_face_setup(uint8_t watch_face_index, void ** context_ptr) {
    (void) watch_face_index;
    if (*context_ptr == NULL) {
        *context_ptr = malloc(sizeof(step_counter_state_t));
        memset(*context_ptr, 0, sizeof(step_counter_state_t));
        // At first run, tell Movement to run the accelerometer in the background. It will now run at this rate forever.
        movement_set_accelerometer_background_rate(LIS2DW_DATA_RATE_25_HZ);
        lis2dw_enable_fifo();
        lis2dw_clear_fifo();

        // Initialize the pedometer instance
        step_counter_state_t *state = (step_counter_state_t *)*context_ptr;
        state->pedometer = malloc(sizeof(pedometer_t));
        pedometer_init(state->pedometer);
    }
}

void step_counter_face_activate(void *context) {
    step_counter_state_t *state = (step_counter_state_t *)context;
    state->display_index = 0;
}

bool step_counter_face_loop(movement_event_t event, void *context) {
    step_counter_state_t *state = (step_counter_state_t *)context;
    switch (event.event_type) {
        case EVENT_ALARM_BUTTON_DOWN:
            state->display_index = (state->display_index + 1) % ACTIVITY_LOGGING_NUM_DAYS;
            // fall through
        case EVENT_ACTIVATE:
            if (watch_sleep_animation_is_running()) {
                watch_stop_sleep_animation();
            }
            _step_counter_face_update_display(state);
            break;
        case EVENT_TICK:
            {
                if (
                    (movement_get_local_date_time().unit.second == 0 || movement_get_local_date_time().unit.second == 30 ) && state->display_index == 0
                ) {
                    printf("updating display\n\r");
                    _step_counter_face_update_display(state);
                }
            }
            break;
        case EVENT_BACKGROUND_TASK:
            {
                size_t pos = state->data_points % ACTIVITY_LOGGING_NUM_DAYS;
                state->activity_log[pos] = state->pedometer->counted_steps;
                state->data_points++;

                // reset the pedometer for the next day
                pedometer_init(state->pedometer);
            }
            break;
        case EVENT_LOW_ENERGY_UPDATE:
            // start tick animation if necessary
            if (!watch_sleep_animation_is_running()) watch_start_sleep_animation(1000);
            // update the display as usual
            _step_counter_face_update_display(state);
            break;
        case EVENT_TIMEOUT:
            // snap back to today on timeout
            state->display_index = 0;
            _step_counter_face_update_display(state);
            break;
        default:
            movement_default_loop_handler(event);
            break;
    }

    return true;
}

void step_counter_face_resign(void *context) {
    (void) context;
}

movement_watch_face_advisory_t step_counter_face_advise(void *context) {
    step_counter_state_t *state = (step_counter_state_t *)context;
    movement_watch_face_advisory_t retval = { 0 };
    lis2dw_fifo_t fifo;

    lis2dw_read_fifo(&fifo);
    if (fifo.count > 0) {
        printf("Activity logging: %d readings in FIFO\n\r", fifo.count);

        uint32_t prev_steps = state->pedometer->counted_steps;

        // we have a reading, so we are active
        for (uint8_t i = 0; i < fifo.count; i++) {
            pedometer_step(
                state->pedometer,
                fifo.readings[i].x,
                fifo.readings[i].y,
                fifo.readings[i].z
            );
        }

        uint32_t new_steps = state->pedometer->counted_steps;
        printf(
            "Activity logging: %lu new steps detected\n\r",
            new_steps - prev_steps
        );
    }

    lis2dw_clear_fifo();

    // if (!HAL_GPIO_A4_read()) {

    //     // watch_enable_i2c();
    //     // only count this as an active minute if the previous minute was also active.
    //     // otherwise, set the flag and we'll count the next minute if the wearer is still active.
    //     printf("Activity logging: active minute detected\n\r");
    //     lis2dw_fifo_t fifo;
    //     lis2dw_read_fifo(&fifo);
    //     if (fifo.count > 0) {
    //         // we have a reading, so we are active
    //         printf("Activity logging: %d readings in FIFO\n\r", fifo.count);
    //         for (uint8_t i = 0; i < fifo.count; i++) {
    //             printf("FIFO reading %d: x=%d, y=%d, z=%d\n\r",
    //                 i,
    //                 fifo.readings[i].x,
    //                 fifo.readings[i].y,
    //                 fifo.readings[i].z
    //             );
    //         }
    //     } else {
    //         // no readings, so we are not active
    //         printf("Activity logging: no readings in FIFO\n\r");
    //     }
    //     if (state->previous_minute_was_active) {
    //         state->active_minutes_today++;
    //     }
    //     else state->previous_minute_was_active = true;

    //     // watch_disable_i2c();
    // } else {
    //     printf("Activity logging: inactive\n\r");
    //     state->previous_minute_was_active = false;
    // }

    watch_date_time_t datetime = movement_get_local_date_time();
    // request a background task at midnight to shuffle the data into the log
    if (datetime.unit.hour == 0 && datetime.unit.minute == 0) {
        retval.wants_background_task = true;
    }

    return retval;
}
