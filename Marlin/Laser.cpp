#include "Laser.h"
#include "Marlin.h"
#include "temperature.h"

namespace Laser
{
  bool enabled = false;
  uint8_t power = 0;
  bool    synchronized = false;
  unsigned long max_inactive_time = 0;

  void enable ()
  {
    // Heaters disabled
    tp_disable_heater();
#if MOTHERBOARD == 25  // FABtotum's TOTUMduino
    // Proprietary laser extensions (and quirks)
    if (installed_head_id == FAB_HEADS_laser_ID || installed_head_id == FAB_HEADS_laser_pro_ID) {
      SET_OUTPUT(HEATER_0_PIN);
      WRITE(HEATER_0_PIN, 1);
    }
#endif

    // Laser tools use servo 0 lines, in the future this may be configurable
    // This should be more or less shared code, apart from the ugly servo pin names
    ::servo_detach(0);
    SERVO1_OFF();
    SET_OUTPUT(SERVO0_PIN);
    SET_OUTPUT(NOT_SERVO1_ON_PIN);

    enabled = true;
  }

  void disable ()
  {
    enabled = false;
    power = 0;
    WRITE(HEATER_0_PIN, 0);
    //::servo_attach(0, SERVO0_PIN);
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

#if defined(MOTHERBOARD) && (MOTHERBOARD == 25)
    // QUIRK: auto turn on blower when activating laser head
    if (power && !fanSpeed && installed_head_id == FAB_HEADS_laser_ID) {
      fanSpeed = EXTRUDER_AUTO_FAN_SPEED;
    }
#endif
  }
}
