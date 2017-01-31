==========================
FABlin Firmware
==========================
FABtotum Marlin Derivate Firmware.

This is the FABlin, a Marlin Derivate Firmware. Marlin was originally
created by Erik Zalm (https://github.com/ErikZalm/Marlin) The changes
made are focused on multipurpose personal fabrication and hardware
characteristics of the FABtotum Personal Fabricator such as 3D/4 Axis
milling, 3D/4 axis scanning, Touch-probing and the Totumduino Board
I/Os capabilities.


Compiling
---------

### with Arduino IDE

To compile you need the contents of this repository inside your Arduino
IDE sketchbook. The firmware is contained inside the `Marlin` sketch.

For further info refer to https://www.arduino.cc/en/Guide/Environment#toc7.


### with PlatformIO

A default `platformio.ini` file with correct directories definitions is
now included in the repository. To compile the firmware simply run:

  platformio run

To directly flash the firmware onto TOTUMduino, run:

  platformio run -t program

`platformio.ini` has a default definition for using an AVRISP-mk2 usb
programmer. To flash or upload the firmware through other means refer
to [platformio
documentation](http://docs.platformio.org/en/stable/userguide/cmd_run.html).


Latest Changes
--------------

### Version 1.0.0097.1
- Min power level for laser
- Auto fan on when laser active
- Better diagnostic messages for laser head

### Version: 1.0.0097
- Final laser support: moved dedicated commands from M6/7 to M60/1/2
- Broadened number of cases in which head misplacement is detected
- Workaround for false z-min endstop triggerings

For the complete changelog, see [ChangeLog](ChangeLog.txt).
