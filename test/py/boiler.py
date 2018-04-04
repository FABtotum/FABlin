# Script: boiler.py
#
# Boilerplate for a Python test script
#
# Description:
# This makes use of fabui service's XML-RPC server to run g-code on a local or
# remote machine. You can either copy and expand this script or import it and
# ake use of the `fabui` proxy object.
#

from os import environ
from fabui import Fabui

#
# Variable: RPC_HOST
#
# The test execution host name
#
# Default:
# 127.0.0.1
#
try:
    rpc_host = environ['RPC_HOST']
except:
    rpc_host = '127.0.0.1'

#
# Variable: RPC_PORT
#
# The test execution host port
#
# Default:
# 8000
try:
    rpc_port = environ['RPC_PORT']
except:
    rpc_port = 8000

fabui = Fabui(rpc_host, rpc_port)

### Insert test here ###
