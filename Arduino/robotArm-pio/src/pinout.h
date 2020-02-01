#ifndef PINOUT_H_
#define PINOUT_H_


/*
 * pinout of RAMPS 1.4
 *
 * source: http://reprap.org/wiki/RAMPS_1.4
 */

//RAMPS 1.4 PINS
#define X_STEP_PIN         54
#define X_DIR_PIN          55
#define X_ENABLE_PIN       38
#define X_MIN_PIN           3
#define X_MAX_PIN           2

#define Y_STEP_PIN         60
#define Y_DIR_PIN          61
#define Y_ENABLE_PIN       56
#define Y_MIN_PIN          14
#define Y_MAX_PIN          15

#define Z_STEP_PIN         46
#define Z_DIR_PIN          48
#define Z_ENABLE_PIN       62
#define Z_MIN_PIN          18
#define Z_MAX_PIN          19

#define E_STEP_PIN         31
#define E_DIR_PIN          33
#define E_ENABLE_PIN       35

#define Q_STEP_PIN         37
#define Q_DIR_PIN          39
#define Q_ENABLE_PIN       41

#define SDPOWER            -1
#define SDSS               53
#define LED_PIN            13

#define FAN_PIN             9

#define PS_ON_PIN          12
#define KILL_PIN           -1

#define HEATER_0_PIN       10
#define HEATER_1_PIN        8
#define TEMP_0_PIN         13   // ANALOG NUMBERING
#define TEMP_1_PIN         14   // ANALOG NUMBERING

//RAMPS AUX-1
#define STEPPER_GRIPPER_PIN_0 28
#define STEPPER_GRIPPER_PIN_1 30
#define STEPPER_GRIPPER_PIN_2 32
#define STEPPER_GRIPPER_PIN_3 34

#define ROTATE_ENDSTOP_PIN 38
#define LOWER_ENDSTOP_PIN  40
#define HIGHER_ENDSTOP_PIN 42



#endif
