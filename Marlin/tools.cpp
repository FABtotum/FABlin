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
#include "temperature.h"

int8_t tool_heater_mapping[TOOLS_MAGAZINE_SIZE]/*   = { 0, -1, 0, ... }*/;  // Tool to heater mapping

/**
 * tools.load
 *
 * Load a factory tool configuration into one of the user definable slots.
 */
void tools_s::load (uint8_t tool, uint8_t id)
{
   if (/*id == 0 ||*/ id >= TOOLS_FACTORY_SIZE) return;
   if (tool >= TOOLS_MAGAZINE_SIZE) return;
   memcpy(&(magazine[tool]), &(factory[id]), sizeof factory[id]);
}

/**
 * tools.define
 *
 * Define a logical tool that can be selected through `T` address
 *
 * A tool can be a physical addon (head) to be harnessed, or a different
 * configuration of equipped hardware.
 *
 */
void tools_s::define(uint8_t tool, int8_t drive, int8_t heater, uint8_t serial, bool multi)
{
   magazine[tool].serial = serial;

   if (multi)
   {
      magazine[tool].extruders = drive;
      magazine[tool].heaters = heater;
   }
   else
   {
      magazine[tool].extruders = (drive >= 0)? (1 << drive) : 0;
      magazine[tool].heaters = (heater >= 0)? (1 << heater) : 0;
   }
}

/**
 * tools.change
 *
 * Loads the referenced tool and activate all of its defined hardware and
 * facilities.
 *
 */
uint8_t tools_s::change (uint8_t tool)
{
   // Load serial communication setting
   tool_twi_support[tool] = magazine[tool].serial != 0;

   // Load first selected extruder into map
   tool_extruder_mapping[tool] = -1;
   for (uint8_t e = 0; e < EXTRUDERS; e++) {
      if (magazine[tool].extruders & (1 << e)) {
         tool_extruder_mapping[tool] = e;
         break;
      }
   }

   // Load last selected heater into map, starting from higher ordered
   // heaters to lower ordered.
   // While we're at it, we enable and disable the heaters and temp sensors
   tool_heater_mapping[tool] = -1;
   for (int8_t h = HEATERS-1; h >= 0; h--)
   {
      if (magazine[tool].heaters & (TP_HEATER_0 << h)) {
         tp_enable_heater(TP_HEATER_0 << h);
         tool_heater_mapping[tool] = h;
      } else {
         // If sensor is specified only disable heater
         if (magazine[tool].heaters & (TP_SENSOR_0 << h)) {
            tp_disable_heater(TP_HEATER_0 << h);
         } else {
            tp_disable_sensor(TP_SENSOR_0 << h);
         }
      }
   }
   // Possibly enable sensor with no heater enabled
   for (int8_t h = HEATERS-1; h >= 0; h--)
   {
      if (magazine[tool].heaters & (TP_SENSOR_0 << h)) {
         tp_enable_sensor(TP_SENSOR_0 << h);
         tool_heater_mapping[tool] = h;
      }
   }

   if (tool_extruder_mapping[tool] >= 0 && tool_extruder_mapping[tool] < EXTRUDERS)
   {
      if (tool_heater_mapping[tool] >= 0 && tool_heater_mapping[tool] < HEATERS) {
         extruder_heater_mapping[tool_extruder_mapping[tool]] = tool_heater_mapping[tool];
      } else {
         extruder_heater_mapping[tool_extruder_mapping[tool]] = -1;
      }
   }

   // Bed must be initialized separately because Marlin
   if (magazine[tool].heaters & TP_HEATER_BED) {
      tp_enable_heater(TP_HEATER_BED);
   } else {
      tp_disable_sensor(TP_SENSOR_BED);
   }
   if (magazine[tool].heaters & TP_SENSOR_BED) {
      tp_enable_sensor(TP_SENSOR_BED);
   }

   // Set globals
   //head_is_dummy = !tool_twi_support[tool];
   active_extruder = tool_extruder_mapping[tool];

   // Copy, so we can customize the installed tool without changing tools in the magazine
   memcpy(&installed_head, &(magazine[tool]), sizeof magazine[tool]);

#ifdef SMART_COMM
   if (installed_head_id <= FAB_HEADS_laser_ID)
   {
#endif
      // For legacy heads we do not rely on SmartComm module
      if (!tool_twi_support[tool]) {
         TWCR = 0;
      } else {
         Wire.begin();
      }
#ifdef SMART_COMM
   }
   else
   {
      if (!tool_twi_support[tool]) {
         SmartHead.end();
      } else {
         SmartHead.begin();
         // SmartComm configuration should have already been done

         // WARNING: possibly breaking change for custom heads with id > 99
         // dependant on legacy behaviour. That is, FABlin comm support shall
         // be explicitely configured for them to continue to work.
      }
   }
#endif

   active_tool = tool;
   return active_extruder;
}

tools_s tools;
