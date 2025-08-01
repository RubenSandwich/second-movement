#include <stdio.h>
#include <stddef.h>
#include <stdlib.h>
#include <inttypes.h>
#include "pedometer.h"

/* PEDOMETER CONSTANTS */

#define _1_SECOND 50 // ODR = 50Hz, 50 samples = 1 second
#define REGULATION_OFF_TIME (_1_SECOND << 1) // 2s
#define THRESHOLD_ORDER 4
#define FILTER_ORDER 4
#define WINDOW_SIZE ((FILTER_ORDER << 2) + 1)
#define SENSITIVITY 400 // 400 = 0.1g, for 2g range
#define INIT_OFFSET_VALUE 4000 // 1g, for 2g range
#define INIT_VALUE_MIN 0 // 0g

/* PEDOMETER VARIABLES */
typedef struct {
    int32_t raw[FILTER_ORDER];
    int32_t filtered[WINDOW_SIZE];
    int32_t threshold[THRESHOLD_ORDER];
    int32_t count_steps;
    int8_t idx_window_min, idx_window_max, idx_threshold, idx_buffer, idx_average;
    int8_t step_samples, regulation_mode, max_min_samples, possible_steps;
    uint8_t flag_max, flag_threshold, flag_threshold_counter;
    uint32_t last_max, last_min, filter_mean;
    uint32_t buffer_dynamic_threshold, new_threshold, old_threshold;
} pedometer_t;

void pedometer_init(pedometer_t *p) {
    memset(p, 0, sizeof(pedometer_t));
    p->old_threshold = INIT_OFFSET_VALUE;
    p->buffer_dynamic_threshold = INIT_OFFSET_VALUE * THRESHOLD_ORDER;

    for (int i = 0; i < THRESHOLD_ORDER; i++)
      p->threshold[i] = INIT_OFFSET_VALUE;
}

int32_t pedometer_step(pedometer_t *p, int16_t x, int16_t y, int16_t z) {
  // Invalid pedometer instance
  if (p == NULL) {
    return 0;
  }

  int32_t module_data = abs(x) + abs(y) + abs(z);

  // Eliminate the last value from the mean and add the new value
  p->filter_mean = p->filter_mean - p->raw[p->idx_average] + module_data;
  int32_t filter_module = p->filter_mean / FILTER_ORDER;

  p->raw[p->idx_average] = module_data;
  p->filtered[p->idx_buffer] = filter_module;

  /* MAX AND MIN DETECTION  */
  p->last_max = p->filtered[0];
  p->idx_window_max = 0;
  for (int8_t i = 0; i < WINDOW_SIZE; i++)
  {

    if (p->filtered[i] > p->last_max)
    {
      p->last_max = p->filtered[i];
      p->idx_window_max = i;
    }
  }

  p->last_min = p->filtered[0];
  p->idx_window_min = 0;
  for (int8_t i = 0; i < WINDOW_SIZE; i++)
  {

    if (p->filtered[i] < p->last_min)
    {
      p->last_min = p->filtered[i];
      p->idx_window_min = i;
    }
  }

  /* MAX SEARCHING */
  if (p->flag_max == 0)
  {
    // Is the Window max in the midle of the window? If so mark it as a max
    if (p->idx_window_max == ((p->idx_buffer + (WINDOW_SIZE >> 1)) % WINDOW_SIZE))
    {
      p->flag_max = 1;
      p->last_max = p->last_max;
      p->max_min_samples = 0;
    }
  }
  else
  {
    // Is the Window min in the midle of the window? If so mark it as a min
    if (p->idx_window_min == ((p->idx_buffer + (WINDOW_SIZE >> 1)) % WINDOW_SIZE))
    {
      p->last_min = p->last_min;
      int32_t difference = p->last_max - p->last_min;
      p->flag_max = 0;
      p->max_min_samples = 0;

      // Detect if the SENSITIVITYs are in of the Threshold level
      if ((p->last_max > (p->old_threshold + (SENSITIVITY >> 1))) && (p->last_min < (p->old_threshold - (SENSITIVITY >> 1))))
      {
        // Possible step detected, will be analyze later
        p->flag_threshold = 1;
        p->flag_threshold_counter = 0;
      }
      else
      {
        p->flag_threshold_counter++;
      }

      /* THRESHOLD LEVEL UPDATE */
      if (difference > SENSITIVITY)
      {
        // Threshold level is calculated with the 4 previous good differences
        // The same method as the Filtering method
        p->new_threshold = (p->last_max + p->last_min) >> 1;
        p->buffer_dynamic_threshold = p->buffer_dynamic_threshold - p->threshold[p->idx_threshold] + p->new_threshold;
        p->old_threshold = p->buffer_dynamic_threshold / THRESHOLD_ORDER;
        p->threshold[p->idx_threshold] = p->new_threshold;
        p->idx_threshold++;
        if (p->idx_threshold > THRESHOLD_ORDER - 1)
        {
          p->idx_threshold = 0;
        }

        /* STEP CERTIFICATION */
        if (p->flag_threshold)
        {
          // DETECTED STEP
          p->flag_threshold = 0;
          p->step_samples = 0;
          p->max_min_samples = 0;

          if (p->regulation_mode)
          {
            // the step is counted
            p->count_steps++;
          }
          else
          {
            p->possible_steps++;

            // conclude that the person is walking
            if (p->possible_steps == 8)
            {
              p->count_steps = p->count_steps + p->possible_steps;
              p->possible_steps = 0;
              p->regulation_mode = 1;
            }
          }
        }
      }
      p->last_min = INIT_VALUE_MIN;

      // If the SENSITIVITYs are out of the Threshold Level 2 times consecutively with a good difference,
      // Is very probably that the acceleration profile isn't a step sequence
      // (Reset the possible steps to discard false steps)
      if (p->flag_threshold_counter > 1)
      {
        p->flag_threshold_counter = 0;
        p->max_min_samples = 0;
        p->last_max = 0;
        p->last_min = 0;
        p->regulation_mode = 0;
        p->possible_steps = 0;
        p->flag_threshold_counter = 0;
      }
    }
    else
    {
      // Reset step detection if searching for minimum takes too long (>25 samples)
      p->max_min_samples++;
      if (p->max_min_samples == _1_SECOND)
      {
        p->max_min_samples = 0;
        p->last_max = 0;
        p->last_min = 0;
        p->flag_max = 0;
        p->possible_steps = 0;
      }
    }
  }


  p->idx_buffer++;
  if (p->idx_buffer > WINDOW_SIZE - 1)
  {
    p->idx_buffer = 0;
  }

  p->idx_average++;
  if (p->idx_average > FILTER_ORDER - 1)
  {
    p->idx_average = 0;
  }

  p->step_samples++;
  if (p->step_samples >= REGULATION_OFF_TIME)
  {
    // If the pedometer takes 2 seconds without counting a step it resets all parameters
    p->step_samples = 0;
    p->possible_steps = 0;
    p->regulation_mode = 0;
    p->max_min_samples = 0;
    if (p->regulation_mode == 1)
    {
      p->regulation_mode = 0;
      p->old_threshold = INIT_OFFSET_VALUE;
      p->buffer_dynamic_threshold = INIT_OFFSET_VALUE * THRESHOLD_ORDER;
      p->idx_threshold = 0;
      p->flag_threshold_counter = 0;
      for (int8_t i = 0; i < THRESHOLD_ORDER; i++)
      {
        p->threshold[i] = INIT_OFFSET_VALUE;
      }
    }
  }

  return p->count_steps;
}
