/* main.c

   Control motors, servo, distance sensors, and LEDs inside an R2D2 robot.

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

#include "driver/mcpwm.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "defs.h"

static void
r3d2_pwm_init (void)
{
  mcpwm_config_t servo;

  servo.frequency = 50;
  servo.cmpr_a = 0;
  servo.cmpr_b = 0;
  servo.duty_mode = MCPWM_DUTY_MODE_0;
  servo.counter_mode = MCPWM_UP_COUNTER;

  ESP_ERROR_CHECK (mcpwm_gpio_init (MCPWM_UNIT_0, MCPWM1A, SERVO_CTL_PIN));
  ESP_ERROR_CHECK (mcpwm_init (MCPWM_UNIT_0, MCPWM_TIMER_1, &servo));
}

void
app_main (void)
{
  r3d2_pwm_init ();
}
