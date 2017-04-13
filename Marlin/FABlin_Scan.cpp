#include "FABlin_Scan.h"
#include "temperature.h"

namespace Scan
{
	void enable ()
	{
		tp_disable_sensor(TP_SENSOR_BED);
		tp_disable_heater(TP_HEATER_BED);

		SET_OUTPUT(SCAN_STEP_PIN);
		SET_OUTPUT(SCAN_ENABLE_PIN);
		SET_OUTPUT(SCAN_BED_ON_PIN);

		WRITE(SCAN_BED_ON_PIN, SCAN_BED_ON);
		delay(1000);
	}

	void disable ()
	{
		WRITE(SCAN_BED_ON_PIN, !SCAN_BED_ON);
	}
}
