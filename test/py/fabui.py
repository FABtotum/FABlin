from xmlrpclib import ServerProxy
import re


class Fabui:

    def __init__ (self, rpc_host, rpc_port):
        self.proxy = ServerProxy('http://{}:{}/FABUI'.format(rpc_host, rpc_port))

    def send (self, cmd):
        return self.proxy.send(cmd)

    def do_reset (self):
        return self.proxy.do_reset()

    def assertOk (self, cmd):
        ok = self.send(cmd)
        assert re.compile('^ok(\s*$|\s+.+$)').match(ok[-1]), "Command returned {}".format(ok[-1])

    def assertErrorCode (self, code):
        m730 = self.send('M730')
        assert int(m730[0]) == code, "Error code {} is different from {}".format(m730[0], code)

    def assertExtTemp (self):
        m105 = self.send('M105')
        assert re.compile('^ok\s+T\:\s*[1-9]+\.\d/0\.0\s+').match(m105[-1]) is not None, "Temperature is {}".format(m105[-1])
