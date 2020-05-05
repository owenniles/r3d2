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

static void start (void);
static void stop (void);

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
move (int dist)
{
  int l1 = dist > 0;
  int l2 = dist < 0;
  
  ESP_ERROR_CHECK (gpio_set_level (DRIVER_A1_PIN, l1));
  ESP_ERROR_CHECK (gpio_set_level (DRIVER_A2_PIN, l2));
  ESP_ERROR_CHECK (gpio_set_level (DRIVER_B1_PIN, l1));
  ESP_ERROR_CHECK (gpio_set_level (DRIVER_B2_PIN, l2));
  
  start ();
  vTaskDelay (2000 / portTICK_PERIOD_MS);
  stop ();
}

/* Gently accelerate to normal movement speed. */
static void
start (void)
{
  for (int i = 20; i <= 50; i += 3)
    {
      ESP_ERROR_CHECK (mcpwm_set_duty (MCPWM_UNIT_0, MCPWM_TIMER_0,
				       MCPWM_GEN_A, i));
      ESP_ERROR_CHECK (mcpwm_set_duty (MCPWM_UNIT_0, MCPWM_TIMER_0,
				       MCPWM_GEN_B, i));
      vTaskDelay (100 / portTICK_PERIOD_MS);
    }
  printf ("Started\n");
}

/* Gently stop moving. */
static void
stop (void)
{
  float duty_a = mcpwm_get_duty (MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_A);
  float duty_b = mcpwm_get_duty (MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_B);
  float step_a = (duty_a - 20) / 10;
  float step_b = (duty_b - 20) / 10;

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
}

/* Causes R3D2 to turn his entire body either left or right */
void
turn (int direction, uint8_t flags)
{
}
