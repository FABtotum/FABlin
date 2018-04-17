from boiler import fabui
import re

# Given no particular head is installed
fabui.assertOk('M793S0')
fabui.send('M999')

# Given the probe sensor is functional
m750 = fabui.send('M750')
assert m750[-1] == 'ok', "M750 not ok"
match = re.compile('^Pressure:\s*(\d+\.\d+)$').search(m750[-2])
assert match, "No Pressure measured"
assert float(match.group(1)) > 100, "Measured pressure too low: {}".format(match.group(1))
assert float(match.group(1)) < 900, "Measured pressure too high: {}".format(match.group(1))
