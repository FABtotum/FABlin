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

Now FABlin requires the custom _SmartComm_ module, stored inside 
libraries. To correctly compile the firmware with Arduino you must set 
the folder where this repository is checked out as Arduino's sketchbook 
folder inside Arduino's preferences. Alternatively you can manually 
copy the [FABlin/libraries/SmartComm](libraries/SmartComm) folder into 
your current sketchbook's libraries folder.

For further info refer to 
https://www.arduino.cc/en/Guide/Environment#toc7.


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

### Version: 1.0.0096 (release candidate 1)
* Fixed bed hitting the bottom during G27 / G28
