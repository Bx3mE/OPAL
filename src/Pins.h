/*
  Pins.h - driver code to handle Pin definitions on PJRC Teensy 4.x board

  Part of OpenGalvo - OPAL Firmware

  Copyright (c) 2020-2021 Daniel Olsson

  OPAL Firmware is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  OPAL Firmware is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with OPAL Firmware.  If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#ifndef PINS_H
  #define PINS_H
  /*
  The following pins are not configurable as DIRECT PORT Writes are used within the code
    XY2_100 - clock = 22;
    XY2_100 - sync  = 17;  
    XY2_100 - dataX = 19;
    XY2_100 - dataY = 14;
  */

 /*
  Pin for SSR Relay control of stepper PSU
 */
  #define STEPPER_SSR_OUT_PIN 20
 /*
  Pin for SSR Relay control of galvo PSU
 */
  #define GALVO_SSR_OUT_PIN 11

   /*
  Pin for SSR Relay control of laser PSU
 */
  #define LASER_SSR_OUT_PIN 10

  /*
    PWM pin to control laser power/intensity (0-4095)
  */
  #define LASER_PWM_OUT_PIN 6

#endif
