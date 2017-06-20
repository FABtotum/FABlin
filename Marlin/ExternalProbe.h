#ifndef _EXTERNAL_PROBE_H_
#define _EXTERNAL_PROBE_H_

namespace ExternalProbe
{

	bool    readState(void);
	bool    setSource(uint8_t id);
	uint8_t getSource(void);
	bool    isEnabled(uint8_t axis);
	void	enable(uint8_t axis);
	void	disable(uint8_t axis);
}

#endif /* _EXTERNAL_PROBE_H_ */
