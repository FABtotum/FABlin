#ifndef _EXTERNAL_PROBE_H_
#define _EXTERNAL_PROBE_H_

namespace ExternalProbe
{

	bool    readState(void);
	bool    setSource(uint8_t id);
	uint8_t getSource(void);
	bool    isEnabled(void);
	void	disable(void);
}

#endif /* _EXTERNAL_PROBE_H_ */
