/*
 * openscan firmware
 * Copyright (C) Christian Völlinger  2019
 *
 * Based on orginal openscan firmware
 * Copyright (C) 2019 Thomas Megel
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#ifndef CONFIG_H_
#define CONFIG_H_

/**
 * define board by commenting following lines
 */

#define OPENSCAN
//#define ZUMSCAN



#ifdef OPENSCAN
#ifdef ZUMSCAM
#error only one board define allowed
#endif
#include <menuIO/keyIn.h>

#define BUTTON_UP_PIN 11 //pin for button up
#define BUTTON_DOWN_PIN 10 //pin for button down
#define BUTTON_OK_PIN 9 //pin for button ok

#define PHOTO1_PIN 3 //CameraPin

#define STEPPERTABLE_DIR_PIN 4 //StepperTable direction pin
#define STEPPERTABLE_STEP_PIN 5 //StepperTable step pin

#define STEPPERROTOR_DIR_PIN 6 //StepperRotor direction pin
#define STEPPERROTOR_STEP_PIN 7 //StepperRotor step pin
#endif

#ifdef ZUMSCAN
#ifdef OPENSCAN
#error only one board define allowed
#endif
#include <TimerOne.h>
#include <ClickEncoder.h>
#include <menuIO/clickEncoderIn.h>

#define BUTTON_UP_PIN A0 //pin for button up
#define BUTTON_DOWN_PIN A1 //pin for button down
#define BUTTON_OK_PIN 4 //pin for button ok

#define PHOTO1_PIN 3 //CameraPin

#define STEPPERTABLE_DIR_PIN 13 //StepperTable direction pin
#define STEPPERTABLE_STEP_PIN 12 //StepperTable step pin
#define STEPPERTABLE_ENABLE_PIN 9

#define STEPPERROTOR_DIR_PIN 8 //StepperRotor direction pin
#define STEPPERROTOR_STEP_PIN 7 //StepperRotor step pin
#define STEPPERROTOR_ENABLE_PIN 6
#endif

#endif /* CONFIG_H_ */