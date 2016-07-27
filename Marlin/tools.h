#ifndef _TOOLS_H_
#define _TOOLS_H_

#include <stdint.h>

extern struct tools_s
{
   void    define (uint8_t, unsigned int=0, unsigned int=0, bool=true);
   uint8_t load   (uint8_t);

} tools;

#endif
