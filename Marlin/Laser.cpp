#include "Laser.h"
#include "Marlin.h"
#include "temperature.h"

namespace Laser
{
	bool enabled = false;
	uint8_t power = 0;
	bool    synchronized = false;
	unsigned long max_inactive_time = 0;

	/*bool isEnabled ()
	{
	  // Test power for Laser head
	  //if (installed_head_id == FAB_HEADS_laser_ID
	  //|| installed_head_id == FAB_HEADS_laser_pro_ID) {
		 enabled = READ(HEATER_0_PIN);
	  //}

	  return enabled;
	}*/

	void enable ()
	{
	  // Laser tools use servo 0 lines, in the future this may be configurable
	  // This should be more or less shared code, apart from the ugly servo pin names
	  ::servo_detach(0);
	  SERVO1_OFF();
	  SET_OUTPUT(SERVO0_PIN);
	  SET_OUTPUT(NOT_SERVO1_ON_PIN);

#if MOTHERBOARD == 25  // FABtotum's TOTUMduino
		// Proprietary laser extensions (and quirks)
	  if (installed_head_id == FAB_HEADS_laser_ID || installed_head_id == FAB_HEADS_laser_pro_ID) {
		 tp_disable_heater();
		 SET_OUTPUT(HEATER_0_PIN);
		 WRITE(HEATER_0_PIN, 1);
		 tp_enable_sensor(TP_SENSOR_0);
	  }
#endif
	}

	void disable ()
	{
	  power = 0;
	  ::servo_attach(0, SERVO0_PIN);

	  // Disable supplementary +24v power for fabtotum laser head
	  //if (installed_head_id == FAB_HEADS_laser_ID) {
		 WRITE(HEATER_0_PIN, 0);
		 //tp_enable_heater();
	  //}
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

		// QUIRK: auto turn on blower when activating laser head
	  if (power && !fanSpeed && installed_head_id == FAB_HEADS_laser_ID) {
		 fanSpeed = EXTRUDER_AUTO_FAN_SPEED;
	  }
	}

}
