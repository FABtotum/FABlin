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

static volatile bool external_zprobe_inverting = true;

#define TRIGGER_ON_X_MOVE	0x01
#define TRIGGER_ON_Y_MOVE	0x02
#define TRIGGER_ON_Z_MOVE	0x04
#define TRIGGER_ON_ANY_MOVE	(TRIGGER_ON_X_MOVE | TRIGGER_ON_Y_MOVE | TRIGGER_ON_Z_MOVE)

bool readState(void)
{
	switch(external_zprobe_id)
	{
		case 1:
			return READ(NOT_SECURE_SW_PIN) ^ external_zprobe_inverting;
		case 2:
			return READ(I2C_SCL) ^ external_zprobe_inverting;
		default:
			return false;
	}
}

void enable()
{
	trigger_flags = TRIGGER_ON_ANY_MOVE;
}

void disable()
{
	trigger_flags = 0;
}

bool isEnabled()
{
	if( isActive() )
		return trigger_flags & TRIGGER_ON_ANY_MOVE;
	return false;
}

bool isActive()
{
	return external_zprobe_id;
}

/**
 * 
 * 0 - disabled
 * 1 - SECURE_SW_PIN
 * 2 - I2C_SCL
 */
bool setSource(uint8_t id)
{
	trigger_flags = 0;
	
	external_zprobe_id = id;
	switch(id)
	{
		case 0:
			break;
		case 1:
			SET_INPUT(NOT_SECURE_SW_PIN);
			external_zprobe_inverting = true;
			break;
		case 2:
			TWCR &= ~(1 << TWEN);
			SET_INPUT(I2C_SCL);
			external_zprobe_inverting = false;
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

void setInverted(bool inverted)
{
	external_zprobe_inverting = inverted;
}

bool getInverted()
{
	return external_zprobe_inverting;
}

} /* namespace ExternalProbe */
