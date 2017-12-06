#ifndef _TOOLS_H_
#define _TOOLS_H_

#include <stdint.h>
#include "Configuration.h"

#define TOOL_SERIAL_NONE 0
#define TOOL_SERIAL_SER  1
#define TOOL_SERIAL_TWI  2

#ifndef TOOLS_MAGAZINE_SIZE
   #define TOOLS_MAGAZINE_SIZE EXTRUDERS
#endif

typedef struct tool_s {
   uint8_t mode = 0;
   uint8_t serial  = 0;
   uint8_t extruders:4;
   uint8_t heaters = 0;
   uint8_t thtable = THERMISTOR_HOTSWAP_DEFAULT_INDEX;
   int16_t mintemp = HEATER_0_MINTEMP;  // °C
   int16_t maxtemp = HEATER_0_MAXTEMP;  // °C
   const char *mods = NULL;
} tool_t;

extern struct tools_s
{
   void    define (uint8_t, int8_t=0, int8_t=0, uint8_t=0, bool=false);
   uint8_t change (uint8_t);
   void    load   (uint8_t, uint8_t);

   // Factory defined tools
   // TODO: make local and/or const
   tool_t factory[TOOLS_FACTORY_SIZE];

   // Magazine of user definable tools
   tool_t magazine[TOOLS_MAGAZINE_SIZE];

} tools;

extern int8_t tool_heater_mapping[TOOLS_MAGAZINE_SIZE];

#endif
