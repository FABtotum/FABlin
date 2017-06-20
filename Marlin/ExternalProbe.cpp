#include "Configuration.h"
#include "pins.h"
#include "ExternalProbe.h"

namespace ExternalProbe
{

static volatile uint8_t external_zprobe_id = 0;
// Digitizer probe is sensitive to virations and during fast homing moves
// it is triggered. With the trigger flags the individual reactions can
// be disable when not needed.
static volatile uint8_t trigger_flags = 0;

#define TRIGGER_ON_X_MOVE	0x01
#define TRIGGER_ON_Y_MOVE	0x02
#define TRIGGER_ON_Z_MOVE	0x04

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

void enable(uint8_t axis)
{
	trigger_flags |= 1 << axis;
}

void disable(uint8_t axis)
{
	trigger_flags &= ~(1 << axis);
}

bool isEnabled(uint8_t axis)
{
	if(external_zprobe_id)
	{
		return trigger_flags & (1 << axis);
	}
	return false;
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
	trigger_flags = 0;
	
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
