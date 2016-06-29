#ifndef _CONFIGURATION_HEADS_H_
#define _CONFIGURATION_HEADS_H_

#include "stepper.h"

/**
 * Definitions for FABtotum Heads.
 */

// 0 .. 99 are reserved
#define FAB_HEADS_default_ID     0
#define FAB_HEADS_default_DRIVE  0
#define FAB_HEADS_default_HEATER 0
#define FAB_HEADS_default_SMART  1

#define FAB_HEADS_hybrid_ID     1

#define FAB_HEADS_print_v2_ID   2

#define FAB_HEADS_mill_v2_ID    3
#define FAB_HEADS_mill_v2_SMART 1

#define FAB_HEADS_laser_ID      4

#define FAB_HEADS_5th_axis_ID    5
#define FAB_HEADS_5th_axis_DRIVE 1
#define FAB_HEADS_5th_axis_SMART 0

#define FAB_HEADS_direct_ID     6
#define FAB_HEADS_direct_DRIVE  2
#define FAB_HEADS_direct_HEATER 0
#define FAB_HEADS_direct_SMART  0

// 100 ... are free for public use

#endif // _CONFIGURATION_HEADS_H_


/*struct Fab_Head {
   uint8_t slot;
   uint8_t type0:7;  // Bit field: 6-3: reserved, 2: uart, 1: twi, 0: passive/active
   bool purpose:1;  // 0: single, 1: multi
   uint8_t type1;  // Bit field: 7-3: reserved, 2: laser, 1: milling, 0: additive
   uint8_t wattage;  // 0-255
   uint8_t axis;
   uint8_t servo;
   uint8_t heater;
};

// Preferred slot
#define FAB_TOOL_0 0
#define FAB_TOOL_1 1
#define FAB_TOOL_2 2

// Type 0
#define FAB_HEAD_PASSIVE 0b0000000
#define FAB_HEAD_ACTIVE  0b0000001
#define FAB_HEAD_TWI     0b0000010
#define FAB_HEAD_UART    0b0000100

// Type 1
#define FAB_HEAD_ADDITIVE    0b00000001
#define FAB_HEAD_MILLING     0b00000010
#define FAB_HEAD_SUBTRACTIVE 0b00000010
#define FAB_HEAD_LASER       0b00000100

// Multi-purposedness
#define FAB_HEAD_SINGLE_PURPOSE false
#define FAB_HEAD_MULTIPURPOSE true

// Axis
#define FAB_HEAD_NO_AXIS 0b00000000
#define FAB_HEAD_E0_AXIS 0b00000001
#define FAB_HEAD_E1_AXIS 0b00000010
#define FAB_HEAD_E2_AXIS 0b00000100

// Servo
#define FAB_HEAD_NO_SERVO 0b00000000
#define FAB_HEAD_SERVO_0  0b00000001
#define FAB_HEAD_SERVO_1  0b00000010

// Heater
#define FAB_HEAD_NO_HEATER 0b00000000
#define FAB_HEAD_HEATER_0  0b00000001
#define FAB_HEAD_HEATER_1  0b00000010
#define FAB_HEAD_HEATER_2  0b00000100
#define FAB_HEAD_HEATER_B  0b00001000

Fab_Head tooldefs[] = {
   {  // Default
      FAB_TOOL_0,
      FAB_HEAD_ACTIVE | FAB_HEAD_TWI,
      FAB_HEAD_SINGLE_PURPOSE,
      FAB_HEAD_ADDITIVE,
      0,
      FAB_HEAD_E0_AXIS,
      FAB_HEAD_NO_SERVO,
      FAB_HEAD_NO_HEATER
   }, {  // Hybrid
      FAB_TOOL_0,
      FAB_HEAD_ACTIVE | FAB_HEAD_TWI,
      FAB_HEAD_MULTIPURPOSE,
      FAB_HEAD_ADDITIVE | FAB_HEAD_SUBTRACTIVE,
      0,
      FAB_HEAD_E0_AXIS,
      FAB_HEAD_SERVO_1,
      FAB_HEAD_HEATER_0
   }, {  // Print v2
      FAB_TOOL_0,
      FAB_HEAD_ACTIVE | FAB_HEAD_TWI,
      FAB_HEAD_SINGLE_PURPOSE,
      FAB_HEAD_ADDITIVE,
      0,
      FAB_HEAD_E0_AXIS,
      FAB_HEAD_NO_SERVO,
      FAB_HEAD_HEATER_0
   }, {  // Mill v2
      FAB_TOOL_0,
      FAB_HEAD_ACTIVE | FAB_HEAD_TWI,
      FAB_HEAD_SINGLE_PURPOSE,
      FAB_HEAD_ADDITIVE,
      0,
      FAB_HEAD_E0_AXIS,
      FAB_HEAD_NO_SERVO,
      FAB_HEAD_NO_HEATER
   }, {  // Laser
      FAB_TOOL_0,
      FAB_HEAD_ACTIVE | FAB_HEAD_TWI,
      FAB_HEAD_SINGLE_PURPOSE,
      FAB_HEAD_ADDITIVE,
      0,
      FAB_HEAD_E0_AXIS,
      FAB_HEAD_NO_SERVO,
      FAB_HEAD_NO_HEATER
   }, {  // 5th axis
      FAB_TOOL_1,
      FAB_HEAD_PASSIVE,
      FAB_HEAD_SINGLE_PURPOSE,
      FAB_HEAD_ADDITIVE,
      0,
      FAB_HEAD_E1_AXIS,
      FAB_HEAD_NO_SERVO,
      FAB_HEAD_NO_HEATER
   }, {  // Direct-drive
      FAB_TOOL_2,
      FAB_HEAD_ACTIVE,
      FAB_HEAD_SINGLE_PURPOSE,
      FAB_HEAD_ADDITIVE,
      200,
      FAB_HEAD_E2_AXIS,
      FAB_HEAD_NO_SERVO,
      FAB_HEAD_HEATER_0
   }
};
*/
