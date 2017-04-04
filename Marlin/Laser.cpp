#include "Laser.h"
#include "Marlin.h"
#include "temperature.h"

namespace Laser
{
	bool enabled = false;
	uint8_t power = 0;
	bool    synchronized = false;
	unsigned long max_inactive_time = 0;

	bool isEnabled ()
	{
	  // Test power for Laser head
	  if (installed_head_id == FAB_HEADS_laser_ID) {
		 enabled = READ(HEATER_0_PIN);
	  }

	  return enabled;
	}

	void enable ()
	{
	  // Laser tools use servo 0 pwm, in the future this may be configurable
	  SERVO1_ON();
	  ::servo_detach(0);

	  // Enable supplementary +24v power for fabtotum laser head
	  if (installed_head_id == FAB_HEADS_laser_ID) {
		 disable_heater();
		 WRITE(HEATER_0_PIN, 1);
	  }
	}

	void disable ()
	{
	  power = 0;

	  // Disable supplementary +24v power for fabtotum laser head
	  if (installed_head_id == FAB_HEADS_laser_ID) {
		 WRITE(HEATER_0_PIN, 0);
	  }
	}

	void setPower (uint16_t _power)
	{
	  if (_power > MAX_PWM) {
		 power = MAX_PWM;
	  } else if (_power < 0) {
		 power = 0;
	  } else {
		 power = _power;
	  }

	  if (power && !fanSpeed) {
		 fanSpeed = EXTRUDER_AUTO_FAN_SPEED;
	  }
	}

}
