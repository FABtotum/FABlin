==========================
FABlin Firmware
==========================
FABtotum Marlin Derivate Firmware.

This is the FABlin, a Marlin Derivate Firmware. Marlin was originally created by Erik Zalm (https://github.com/ErikZalm/Marlin)
The changes made are focused on multipurpose personal fabrication and hardware characteristics of the FABtotum Personal Fabricator such as 3D/4 Axis milling, 3D/4 axis scanning, Touch-probing and the Totumduino Board I/Os capabilities.

Supported G-codes: http://forum.fabtotum.cc/showthread.php?1364-Supported-Gcodes


Compiling
---------

### with Arduino IDE

To compile you need the contents of this repository inside your Arduino IDE sketchbook. The firmware is contained inside the `Marlin` sketch.

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


Changelog
---------

### Version: 1.0.0096 (release candidate 2)
- Fixed G27/G28 behaviour near axis limits and silenced verbose error messages
- Fixed G29 hitting Y endstop
- Changed M503 and other diagnostic outputs
- Fixed THERMISTOR_INPUT_HOTSWAP feature

For complete changelog, see [ChangeLog](ChangeLog.txt).
