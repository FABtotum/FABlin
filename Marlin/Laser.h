#ifndef _LASER_H_
#define _LASER_H_

#include <stdint.h>

namespace Laser
{
	extern bool    enabled;
	extern uint8_t power;
	extern bool    synchronized;
	extern unsigned long max_inactive_time;

	void enable  (void);
	void disable (void);

	bool isEnabled (void);

	void setPower (uint16_t);
}

#endif  // _LASER_H_
