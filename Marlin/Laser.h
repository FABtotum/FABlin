#ifndef _LASER_H_
#define _LASER_H_

#include <stdint.h>
#include "Marlin.h"

namespace Laser
{
	extern bool    enabled;
	extern uint8_t power;
	extern bool    synchronized;
	extern unsigned long max_inactive_time;

	void enable  (void);
	void disable (void);

	inline bool isEnabled (void)
	{
#if defined(MOTHERBOARD) && (MOTHERBOARD == 25)
		enabled = READ(HEATER_0_PIN);
#endif
		return enabled;
	}

	void setPower (uint16_t);
}

#endif  // _LASER_H_
