#include "Configuration.h"
#include "pins.h"
#include "ExternalProbe.h"

namespace ExternalProbe
{

static uint8_t external_zprobe_id = 0;

bool readState(void)
{
	switch(external_zprobe_id)
	{
		case 1:
			return !READ(NOT_SECURE_SW_PIN);
		case 2:
			return READ(I2C_SCL);
		default:
			return false;
	}
}

bool isEnabled(void)
{
	return external_zprobe_id != 0;
}

void disable(void)
{
	setSource(0);
}

/**
 * 
 * 0 - disabled
 * 1 - SECURE_SW_PIN
 * 2 - I2C_SCL
 */
bool setSource(uint8_t id)
{
	// restore pin status if needed
	/*switch(external_zprobe_id)
	{
		case 1:
			//NOT_SECURE_SW_PIN;
			break;
		case 2:
			//I2C_SCL;
			break;
		default:;
	}*/

	external_zprobe_id = id;
	switch(id)
	{
		case 0:
			break;
		case 1:
			pinMode(NOT_SECURE_SW_PIN,INPUT);
			break;
		case 2:
			pinMode(I2C_SCL,INPUT);
			break;
		default:
			return false;
	}
	
	return true;
}

uint8_t getSource(void)
{
	return external_zprobe_id;
}

} /* namespace ExternalProbe */
