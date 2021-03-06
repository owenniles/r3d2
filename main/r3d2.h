/* r3d2.h

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

#ifndef _R3D2_H_
#define _R3D2_H_

#include <stdint.h>

#include "driver/gpio.h"

enum direction { FORWARD = 1, BACKWARD = -1, LEFT = 1, RIGHT = -1 };
enum turn_flags { LOCK_HEAD = 0b1 };

#define straighten(pos) (0 + (pos > 1) * 3)

extern int64_t echo[2][2];
extern unsigned long long hall[2];
extern int head_pos;

/* This is a joke. */
#define PP3V3_V0_PIN GPIO_NUM_25
#define PP3V3_V0_MASK GPIO_SEL_25
#define PP3V3_V1_PIN GPIO_NUM_26
#define PP3V3_V1_MASK GPIO_SEL_26

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

#define HALL0_PIN GPIO_NUM_34
#define HALL0_MASK GPIO_SEL_34
#define HALL1_PIN GPIO_NUM_35
#define HALL1_MASK GPIO_SEL_35

/* Object detection interface. */
void distance_isr (void *);
int64_t getdist (unsigned);
void measure (void);

/* Movement interface. */
void hall_isr (void *);
void look (int);
void move (int);
void stop (void);
void turn (int, uint8_t);

#endif /* not _DEFS_H_ */
