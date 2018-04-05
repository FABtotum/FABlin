from boiler import fabui

fabui.send('G27')
fabui.send('G90')
fabui.send('G0 X100 Y1 F5000')
fabui.send('M400')
