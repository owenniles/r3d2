/* defs.h

   Useful, program-wide declarations.

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

#ifndef _DEFS_H_
#define _DEFS_H_

#include "driver/gpio.h"

#define SERVO_PULSE_PIN GPIO_NUM_23
#define SERVO_DUTY_MIN 960
#define SERVO_DUTY_MAX 2140

#define DIST0_TRIG_PIN GPIO_NUM_19
#define DIST0_TRIG_MASK GPIO_SEL_19
#define DIST0_ECHO_PIN GPIO_NUM_36
#define DIST0_ECHO_MASK GPIO_SEL_36
#define DIST1_TRIG_PIN GPIO_NUM_18
#define DIST1_TRIG_MASK GPIO_SEL_18
#define DIST1_ECHO_PIN GPIO_NUM_39
#define DIST1_ECHO_MASK GPIO_SEL_39

#define DRIVER_VCC_PIN GPIO_NUM_13
#define DRIVER_VCC_MASK GPIO_SEL_13
#define DRIVER_A1_PIN GPIO_NUM_5
#define DRIVER_A1_MASK GPIO_SEL_5
#define DRIVER_A2_PIN GPIO_NUM_17
#define DRIVER_A2_MASK GPIO_SEL_17
#define DRIVER_PWMA_PIN GPIO_NUM_22
#define DRIVER_PWMA_MASK GPIO_SEL_22
#define DRIVER_B1_PIN GPIO_NUM_16
#define DRIVER_B1_MASK GPIO_SEL_16
#define DRIVER_B2_PIN GPIO_NUM_4
#define DRIVER_B2_MASK GPIO_SEL_4
#define DRIVER_PWMB_PIN GPIO_NUM_21
#define DRIVER_PWMB_MASK GPIO_SEL_21

#endif /* not _DEFS_H_ */
