/* main.c

   Implementation of a high level interface for object detection.

   Copyright 2020 Owen Niles <oniles@college.harvard.edu>

   This file is part of R3D2.

   R3D2 is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
   
   R3D2 is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program. If not, see <https://www.gnu.org/licenses/>. */

#include <string.h>

#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "r3d2.h"

int64_t echo[2][2];

/* Save the start and end times of the pulse that we receive from the distance
   sensor so that we can calculate the width of the pulse. */
void
distance_isr (int64_t *pulse)
{
#define __START 0
#define __END 1
  int edge = pulse[__START] > pulse[__END];
  
  pulse[edge] = esp_timer_get_time ();
#undef __START
#undef __END
}

int64_t
get_distance (unsigned sensor)
{
  int dist;
  
  if (sensor > 1 || echo[sensor][0] == 0 || echo[sensor][1] == 0)
    return -1;

  dist = (echo[sensor][1] - echo[sensor][0]) / 58;
  memset (echo[sensor], 0, sizeof echo[sensor]);
  
  return dist;
}

void
measure (void)
{
  /* Reset the variable just in case it contains garbage data. */
  memset (echo, 0, sizeof echo);

  /* Send pulses to activate both distance sensors. */
  ESP_ERROR_CHECK (gpio_set_level (DIST0_TRIG_PIN, 1));
  ESP_ERROR_CHECK (gpio_set_level (DIST1_TRIG_PIN, 1));
  
  vTaskDelay (1);
  
  ESP_ERROR_CHECK (gpio_set_level (DIST0_TRIG_PIN, 0));
  ESP_ERROR_CHECK (gpio_set_level (DIST1_TRIG_PIN, 0));
}
