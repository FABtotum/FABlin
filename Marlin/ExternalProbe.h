#ifndef _EXTERNAL_PROBE_H_
#define _EXTERNAL_PROBE_H_

namespace ExternalProbe
{

	/**
	 * Returns the state of probe
	 * true - TRIGGERED
	 * false - open
	 */
	bool    readState(void);
	
	/**
	 * Set probe endstop source
	 * 
	 * @param id 0 - disabled, 1 - secure_sw, 2 - digitizer probe
	 * 
	 */
	bool    setSource(uint8_t id);
	
	/**
	 * Inverte probe state reading
	 */
	void 	setInverted(bool inverted);
	
	/**
	 * Returns probe state inversion configuration
	 */
	bool    getInverted();
	
	/**
	 * Returns configured probe endstop source
	 */
	uint8_t getSource(void);
	
	/**
	 * Returns true if source is not 0 and probe is enabled
	 */
	bool    isEnabled();
	
	/**
	 * Returns true if source is not 0
	 */
	bool    isActive();
	
	/**
	 * Enable probe usage 
	 */
	void    enable();
	
	/**
	 * Disable probe usage but keep the source configured
	 */
	void   disable();
}

#endif /* _EXTERNAL_PROBE_H_ */
