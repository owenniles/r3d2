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

#include <limits.h>

#include "driver/gpio.h"
#include "driver/mcpwm.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "defs.h"

enum measure_flag {
		   M_DIST0 = 0b1, /* Measure distance using sensor 0 */
		   M_DIST1 = 0b10 /* Measure distance using sensor 1 */
};

enum turn_flag { T_NOHEAD = 0b1 };

/* The direction in which R3D2's head is turned. */
static unsigned head_pos;

static int dist[7];
static char stale = CHAR_MAX;

/* Causes R3D2 to place his head in one of four positions. */
static void
look (unsigned pos)
{
  static const int step = (SERVO_DUTY_MAX - SERVO_DUTY_MIN) / 3;
  int duty = SERVO_DUTY_MIN + step * (pos > 3 ? 3 : pos);

  ESP_ERROR_CHECK (mcpwm_set_duty_in_us (MCPWM_UNIT_1, MCPWM_TIMER_0,
					 MCPWM_GEN_A, duty));
  vTaskDelay (500 / portTICK_PERIOD_MS);
  head_pos = pos;
}

/* Measures the distance to the closest object. */
static int
measure (int flags)
{
  return 0;
}

static void
move (int dist)
{
}

/* Causes R3D2 to turn his entire body either left or right */
static void
turn (int direction, int flags)
{
}

static void
pwm_init (void)
{
  mcpwm_config_t pulse;

  pulse.frequency = 50;
  pulse.cmpr_a = 0;
  pulse.cmpr_b = 0;
  pulse.duty_mode = MCPWM_DUTY_MODE_0;
  pulse.counter_mode = MCPWM_UP_COUNTER;

  ESP_ERROR_CHECK (mcpwm_init (MCPWM_UNIT_0, MCPWM_TIMER_0, &pulse));
  ESP_ERROR_CHECK (mcpwm_init (MCPWM_UNIT_1, MCPWM_TIMER_0, &pulse));

  look (0);
}

static void
io_init (void)
{
  gpio_config_t in, out;

  in.pin_bit_mask = DIST0_ECHO_MASK | DIST1_ECHO_MASK;
  in.mode = GPIO_MODE_INPUT;
  in.pull_up_en = GPIO_PULLUP_DISABLE;
  in.pull_down_en = GPIO_PULLDOWN_ENABLE;
  in.intr_type = GPIO_INTR_ANYEDGE;

  out.pin_bit_mask = (DRIVER_VCC_MASK | DRIVER_A1_MASK | DRIVER_A2_MASK
		      | DRIVER_B1_MASK | DRIVER_B2_MASK);
  out.mode = GPIO_MODE_OUTPUT;
  out.pull_up_en = GPIO_PULLUP_DISABLE;
  out.pull_down_en = GPIO_PULLDOWN_ENABLE;
  out.intr_type = GPIO_INTR_DISABLE;

  ESP_ERROR_CHECK (gpio_config (&in));
  ESP_ERROR_CHECK (gpio_config (&out));
  
  ESP_ERROR_CHECK (mcpwm_gpio_init (MCPWM_UNIT_0, MCPWM0A, DRIVER_PWMA_PIN));
  ESP_ERROR_CHECK (mcpwm_gpio_init (MCPWM_UNIT_0, MCPWM0B, DRIVER_PWMB_PIN));
  ESP_ERROR_CHECK (mcpwm_gpio_init (MCPWM_UNIT_1, MCPWM0A, SERVO_PULSE_PIN));

  pwm_init();
}

void
app_main (void)
{
  io_init ();
}
