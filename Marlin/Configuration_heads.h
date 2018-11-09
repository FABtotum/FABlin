#ifndef _CONFIGURATION_HEADS_H_
#define _CONFIGURATION_HEADS_H_

/**
 * Definitions for FABtotum Heads.
 */

// 0 (for default) + the effective number of heads
//#define TOOLS_FACTORY_SIZE 9

// 0 .. 99 are reserved
#define FAB_HEADS_default_ID     0
#define FAB_HEADS_default_DRIVE  0
#define FAB_HEADS_default_HEATER 0
#define FAB_HEADS_default_SMART  0

#define FAB_HEADS_hybrid_ID     1
#define FAB_HEADS_hybrid_SMART  1

#define FAB_HEADS_print_v2_ID   2

#define FAB_HEADS_mill_v2_ID    3
#define FAB_HEADS_mill_v2_SMART 1

#define FAB_HEADS_laser_ID      4

#define FAB_HEADS_5th_axis_ID    5
#define FAB_HEADS_5th_axis_DRIVE 1
#define FAB_HEADS_5th_axis_SMART 0

#define FAB_HEADS_direct_ID     6
#define FAB_HEADS_direct_DRIVE  2
#define FAB_HEADS_direct_HEATER 1
#define FAB_HEADS_direct_SMART  0

#define FAB_HEADS_laser_pro_ID  7

#define FAB_HEADS_digitizer_ID  8

// 100 ... are free for public use
#define FAB_HEADS_thirdparty_ID 100

#endif // _CONFIGURATION_HEADS_H_
