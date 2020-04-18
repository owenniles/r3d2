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

#define SERVO_CTL_PIN GPIO_NUM_23
#define SERVO_DUTY_MIN 960
#define SERVO_DUTY_MAX 2140

#endif /* not _DEFS_H_ */
