from boiler import fabui
import sys

mod_names = [
  'Void',
  'Hybrid Head',
  'Printing Head',
  "Milling Head",
  "Laser Head",
  "Reserved",
  "Printing Head PRO",
  "Laser Head PRO",
  "Prism"
]

head_id = int(sys.argv[1])
working_mode = sys.argv[2]

fabui.send('M793 S0')

print "The following tests need {} to be installed.".format(mod_names[head_id])
print "Ensure the module is installed by the UI and press ENTER to continue."
print "Otherwise input the phrase 'skip' and press ENTER to skip this batch."
reply = raw_input("> ")

if reply == 'skip':
    sys.exit(1)
else:
    fabui.do_reset()

    fabui.assertWorkingMode(working_mode)

    fabui.assertHeadSoftId(head_id)
