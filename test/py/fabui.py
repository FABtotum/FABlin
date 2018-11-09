from xmlrpclib import ServerProxy
import re


class Fabui:

    errors = {
        "ERROR_MIN_TEMP": 103,
        "ERROR_MAX_TEMP": 104,
        "ERROR_HEAD_ABSENT": 125
    }

    def __init__ (self, rpc_host, rpc_port):
        self.proxy = ServerProxy('http://{}:{}/FABUI'.format(rpc_host, rpc_port))

    def send (self, cmd):
        return self.proxy.send(cmd)

    def do_reset (self):
        return self.proxy.do_reset()

    def assertOk (self, cmd):
        ok = self.send(cmd)
        error = re.compile('^E:\s+(.+)$')
        for line in range(0, len(ok)):
            err = error.search(ok[line])
            assert err is None, "Got error: {}".format(err.group(1))
        assert re.compile('^ok(\s*$|\s+.+$)').match(ok[-1]), "Command returned {}".format(ok[-1])

    def assertErrorCode (self, code):
        m730 = self.send('M730')
        assert int(m730[0]) == code, "Error code {} is different from {}".format(m730[0], code)

    def assertHeadSoftId (self, value):
        m793 = self.send('M793')
        head_id = int(m793[0])
        assert head_id == int(value), "Installed head {} is different from {}".format(head_id, value)

    def assertWorkingMode (self, value):
        m450 = self.send('M450')
        wm = re.compile('^Working Mode:\s*(.+)\s*$').search(m450[0])
        assert wm is not None, "Cannot retrieve current working mode"
        assert wm.group(1) == value, "Current working mode {} is different than {}".format(wm.group(1), value)

    def assertExtTemp (self):
        m105 = self.send('M105')
        assert re.compile('^ok\s+T\:\s*[1-9]+\.\d/0\.0\s+').match(m105[-1]) is not None, "Temperature is {}".format(m105[-1])
