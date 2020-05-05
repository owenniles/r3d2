/* main.c

   Implementation of a high-level interface for controlling R3D2's movement.

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

#include "driver/gpio.h"
#include "driver/mcpwm.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "r3d2.h"

/* The relative direction in which R3D2's head is facing. */
static unsigned head_pos;
static int moving;

static void start (void);

/* Causes R3D2 to place his head in one of four positions. */
void
look (unsigned pos)
{
  static const int step = (SERVO_DUTY_MAX - SERVO_DUTY_MIN) / 3;
  int duty = SERVO_DUTY_MIN + step * (pos > 3 ? 3 : pos);

  ESP_ERROR_CHECK (mcpwm_set_duty_in_us (MCPWM_UNIT_1, MCPWM_TIMER_0,
					 MCPWM_GEN_A, duty));
  vTaskDelay (500 / portTICK_PERIOD_MS);
  head_pos = pos;
}

/* Makes the move the specified number, which can be negative, of centimeters
   in the direction in which it is facing. NOTE: I have not connected the Hall
   sensors to the microcontroller yet, so this function is not implemented as
   specified, yet. */
void
move (int direction)
{
  int in1, in2;
  
  if (moving)
    return;

  in1 = direction > 0;
  in2 = direction < 0;
  
  ESP_ERROR_CHECK (gpio_set_level (DRIVER_A1_PIN, in1));
  ESP_ERROR_CHECK (gpio_set_level (DRIVER_A2_PIN, in2));
  ESP_ERROR_CHECK (gpio_set_level (DRIVER_B1_PIN, in1));
  ESP_ERROR_CHECK (gpio_set_level (DRIVER_B2_PIN, in2));
  
  start ();
}

/* Gently accelerate to normal movement speed. */
static void
start (void)
{
  moving = 1;

  for (int i = 20; i <= 50; i += 3)
    {
      ESP_ERROR_CHECK (mcpwm_set_duty (MCPWM_UNIT_0, MCPWM_TIMER_0,
				       MCPWM_GEN_A, i));
      ESP_ERROR_CHECK (mcpwm_set_duty (MCPWM_UNIT_0, MCPWM_TIMER_0,
				       MCPWM_GEN_B, i));
      vTaskDelay (100 / portTICK_PERIOD_MS);
    }
}

/* Gently stop moving. */
void
stop (void)
{
  float duty_a, duty_b, step_a, step_b;

  if (!moving)
    return;

  duty_a = mcpwm_get_duty (MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_A);
  duty_b = mcpwm_get_duty (MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_B);
  step_a = (duty_a - 20) / 10;
  step_b = (duty_b - 20) / 10;

  for (int i = 1; i <= 10; ++i)
    {
      ESP_ERROR_CHECK (mcpwm_set_duty (MCPWM_UNIT_0, MCPWM_TIMER_0,
				       MCPWM_GEN_A, duty_a - step_a * i));
      ESP_ERROR_CHECK (mcpwm_set_duty (MCPWM_UNIT_0, MCPWM_TIMER_0,
				       MCPWM_GEN_B, duty_b - step_b * i));
      vTaskDelay (100 / portTICK_PERIOD_MS);
    }

  ESP_ERROR_CHECK (mcpwm_set_duty (MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_A,
				   0));
  ESP_ERROR_CHECK (mcpwm_set_duty (MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_B,
				   0));

  ESP_ERROR_CHECK (gpio_set_level (DRIVER_A1_PIN, 0));
  ESP_ERROR_CHECK (gpio_set_level (DRIVER_A2_PIN, 0));
  ESP_ERROR_CHECK (gpio_set_level (DRIVER_B1_PIN, 0));
  ESP_ERROR_CHECK (gpio_set_level (DRIVER_B2_PIN, 0));

  vTaskDelay (500 / portTICK_PERIOD_MS);

  moving = 0;
}

/* Causes R3D2 to turn his entire body either left or right */
void
turn (int direction, uint8_t flags)
{
  int in1 = direction > 0;
  int in2 = direction < 0;

  ESP_ERROR_CHECK (gpio_set_level (DRIVER_A1_PIN, in1));
  ESP_ERROR_CHECK (gpio_set_level (DRIVER_A2_PIN, in2));
  ESP_ERROR_CHECK (gpio_set_level (DRIVER_B1_PIN, in2));
  ESP_ERROR_CHECK (gpio_set_level (DRIVER_B2_PIN, in1));

  start ();
  vTaskDelay (100 / portTICK_PERIOD_MS);
  stop ();
}
