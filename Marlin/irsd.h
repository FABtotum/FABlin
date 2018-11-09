#ifndef _IRSD_H_
#define _IRSD_H_

/**
 * Configuration parameters
 * 
 * IRSD_PIN              6  Pin for readings
 * IRSD_PIN_INVERTING    0  Reading logic level
 * IRSD_ENABLE_PIN      25  Pin for enabling
 * IRSD_ENABLE_INVERTING 1  Enable pin logic level
 * IRSD_MODE_DIGITAL     1  Whether to activate digital mode
 */

#if defined(IRSD)

void irsd_init (void);
void irsd_enable (void);
void irsd_disable (void);

#endif

#endif
