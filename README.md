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

To correctly compile the firmware with Arduino you must set the folder
where this repository is checked out as Arduino's *sketchbook folder*
inside Arduino's preferences. Alternatively you can manually copy the
[FABlin/libraries/SmartComm](libraries/SmartComm) folder into your
current sketchbook's *libraries* folder.

For further info refer to
https://www.arduino.cc/en/Guide/Environment#toc7.


### with PlatformIO

A default `platformio.ini` file with correct directories definitions is
now included in the repository. To compile the firmware simply run:

  platformio run

inside this repository's root folder. To upload the firmware on the board
or flash it directly into it refer to [platformio documentation](http://docs.platformio.org/en/stable/userguide/cmd_run.html).
`platformio.ini` has a default definition for using an AVRISP-mk2 usb
programmer. To directly flash the firmware onto TOTUMduino using that
defintion, run:

  platformio run -t program


Latest Changes
--------------

### Version 1.0.0098

- Improved homing procedure for partial homing
- Better laser timeout management: M60 has a longer, configurable, timeout; M61 retains a short immutable timeout
- Laser is turned down when carriage hits X or Y endstops
- Added support for head-embedded feeders
- Improved support for custom heads
- Support for setting min temp with M801's R parameter
- Improved G30 positioning
- M150 command can set rgb led glowing speed
- Modified M300 command to play sequences of beeps
- Firmware startup messages suppressed
- Various small fixes and improvements

For the complete changelog, see [ChangeLog](ChangeLog.txt).
