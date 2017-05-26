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
or flash it directly into it refer to
[platformio documentation](http://docs.platformio.org/en/stable/userguide/cmd_run.html).
`platformio.ini` has a default definition for using an AVRISP-mk2 usb
programmer. To directly flash the firmware onto TOTUMduino using that
defintion, run:

  platformio run -t program


Latest Changes
--------------

### Version 1.1.0

This is a breaking release geared toward use in the new
[FAB-UI/Colibri](https://github.com/FABtotum/fabui-colibri) system.

- Corrected and normalized command output format:
  - 'echo:' prefixes removed
  - All errors are prefixed with 'E:' or 'A:'
  - Changed command output format of: G30,G38,M105,M109,M190,M303
  - Various fixes to commands output
- Added auto temp reporting function, managed through M155
- Added support for wire end check. M805 enable/disable check
- Added M564 command: set axes limits and restrict movements
- Fixed laser head initialization
- Small changes in documentation


For the complete changelog, see [ChangeLog](ChangeLog.txt).
