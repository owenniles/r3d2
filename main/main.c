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

#include "driver/gpio.h"
#include "driver/mcpwm.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "r3d2.h"

static int waiting;

static void pwm_init (void);

static void
io_init (void)
{
  gpio_config_t in, out;
  
  in.pin_bit_mask = DIST0_ECHO_MASK | DIST1_ECHO_MASK;
  in.mode = GPIO_MODE_INPUT;
  in.pull_up_en = GPIO_PULLUP_DISABLE;
  in.pull_down_en = GPIO_PULLDOWN_ENABLE;
  in.intr_type = GPIO_INTR_ANYEDGE;

  out.pin_bit_mask = (DIST0_TRIG_MASK | DIST1_TRIG_MASK | DRIVER_VCC_MASK
		      | DRIVER_A1_MASK | DRIVER_A2_MASK | DRIVER_B1_MASK
		      | DRIVER_B2_MASK);
  out.mode = GPIO_MODE_OUTPUT;
  out.pull_up_en = GPIO_PULLUP_DISABLE;
  out.pull_down_en = GPIO_PULLDOWN_ENABLE;
  out.intr_type = GPIO_INTR_DISABLE;

  /* Configure input and output pins. */
  ESP_ERROR_CHECK (gpio_config (&in));
  ESP_ERROR_CHECK (gpio_config (&out));

  /* Add interrupt handlers for all input pins. */
  ESP_ERROR_CHECK (gpio_install_isr_service (ESP_INTR_FLAG_SHARED));
  ESP_ERROR_CHECK (gpio_isr_handler_add (DIST0_ECHO_PIN, distance_isr,
					 echo[0]));
  ESP_ERROR_CHECK (gpio_isr_handler_add (DIST1_ECHO_PIN, distance_isr,
					 echo[1]));

  /* Select PWM pins. */
  ESP_ERROR_CHECK (mcpwm_gpio_init (MCPWM_UNIT_0, MCPWM0A, DRIVER_PWMA_PIN));
  ESP_ERROR_CHECK (mcpwm_gpio_init (MCPWM_UNIT_0, MCPWM0B, DRIVER_PWMB_PIN));
  ESP_ERROR_CHECK (mcpwm_gpio_init (MCPWM_UNIT_1, MCPWM0A, SERVO_PULSE_PIN));

  pwm_init();
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

  /* Wake the motor driver from standby mode. */
  ESP_ERROR_CHECK (gpio_set_level (DRIVER_VCC_PIN, 1));
  
  look (0);
}

static void
warmup (void)
{
  for (int i = 0; i < 10; ++i)
    {
      measure ();
      vTaskDelay (10 / portTICK_PERIOD_MS);
    }
}

void
app_main (void)
{
  io_init ();
  warmup ();
}
