/**
 tools.cpp

 Tools configuration handler
*/

#include <stdint.h>
#include <Wire.h>

#if defined(SMART_COMM)
#include <SmartComm.h>
#endif

#include "Marlin.h"
#include "Configuration_heads.h"
#include "tools.h"

// Dev jots: DO NOT take into account
/* Struct of a tool / head descriptor

   //preferred tool slot: 0, ... (default:0)
   drive number: 0-2  (default:0)
   heater: (n/a)
   smart: 0/1
   serial port: 1 (should not be changed)
   serial baud rate: 0, 300, ..., 115200 (default: 0 - autodetect)
   twi enabled: 0/1

*/
/*
struct Fab_Head {
   //uint8_t slot;
   uint8_t type0: 1;  // passive/active
   uint16_t type1: 15;  // Bit field: 7-3: reserved, 2: laser, 1: milling, 0: additive
   //bool purpose: 1;  // 0: single, 1: multi
   uint8_t type2;  // Serial comm Baud:4 ; mode:4
   uint8_t wattage;  // 0-255
   uint8_t axis; -> drive(s)
   uint8_t servo; -> ?
   uint8_t heater;
};
*/
/*
       0000  000 1      /2
probe+ 0000  101 0 20/67
   300 0001  100 0 20/11
   600 0010  111 0 21/67
  1200 0011  110 0 21/11
  2400 0100  001 0 67/11
  4800 0101  001 1 67/11/2
  9600 0110  010 0 11/67
 19200 0111  010 1 11/67/2
 28800 1000
 31250 1001
 38400 1010
 57600 1011
115200 1100
       1101
       1110
probe- 1111
*/

/**
 * tools.define
 * 
 * Define a logical tool that can be selected through `T` address
 * 
 * A tool can be a physical addon (head) to be harnessed, or a different
 * configuration of equipped hardware.
 * 
 * Max tools number is statically set to 3 at present.
 * 
 */
// TODO: maybe *inline* this, or does the compiler know better?
void tools_s::define(uint8_t tool, unsigned int drive, unsigned int heater, bool twi)
{
   tool_extruder_mapping[tool] = drive;
   tool_heater_mapping[tool] = heater;
   tool_twi_support[tool] = twi;
}

/**
 * tools.load
 * 
 * Load the referenced tool and activate all of its defined hardware and
 * facilities.
 * 
 */
uint8_t tools_s::load (uint8_t tool)
{
   active_tool = tool;
   active_extruder = tool_extruder_mapping[tool];
   head_is_dummy = !tool_twi_support[tool];

   if (installed_head_id <= FAB_HEADS_laser_ID)
   {
      // For legacy heads we do not rely on SmartComm module
      if (head_is_dummy) {
         TWCR &= ~MASK(TWEN);
      } else {
         Wire.begin();
      }
   }
   else
   {
      if (head_is_dummy) {
         SmartHead.end();
      } else {
         SmartHead.begin();
         // SmartComm configuration should have already been done

         // WARNING: possibly breaking change for custom heads with id > 99
         // dependant on legacy behaviour. That is, FABlin comm support shall
         // be explicitely configured for them to continue to work.
      }
   }

   return active_extruder;
}

tools_s tools;
