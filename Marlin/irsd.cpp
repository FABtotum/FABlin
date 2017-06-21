#include "irsd.h"
#include "Configuration.h"

#if defined(IRSD)

void irsd_init (void)
{
	pinMode(IRSD_PIN, INPUT);
	pinMode(IRSD_ENABLE_PIN, OUTPUT);

#if defined(IRSD_MODE_DIGITAL) && (IRSD_MODE_DIGITAL > 0)
	digitalWrite(IRSD_PIN, HIGH);
#endif
}

void irsd_enable ()
{
#if defined(IRSD_MODE_DIGITAL) && (IRSD_MODE_DIGITAL > 0)
	digitalWrite(IRSD_PIN, HIGH);
#endif

	digitalWrite(IRSD_ENABLE_PIN, !IRSD_ENABLE_INVERTING);
}

void irsd_disable ()
{
	digitalWrite(IRSD_ENABLE_PIN, IRSD_ENABLE_INVERTING);
}

#endif
