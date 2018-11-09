
                        FABlin Firmware
                        ===============
for **FABtotum Personal Fabricator** and **FABtotum Core** platforms

This is FABlin, a Marlin derivative firmware. Marlin was originally
created by Erik Zalm (https://github.com/ErikZalm/Marlin). The changes
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

#### Version 1.1.1.3

- Fix power supply issues in Laser Heads
- Fix `M999` behavior with pending temperature errors
- Fix `M42` command output

#### Version 1.1.1.2

- More stable temperature readings on edge and error conditions


#### Version 1.1.1.1

- Fix heaters not working inside PID tuning procedure (M303 command)
- Fix missing bed temperature readings with some toolheads
- Better stability when reconfiguring tools and installing new heads
- Fix M563 output when the bed's thermistor is active without the bed heater being so


### Version 1.1.1

- Add support functions for Fabtotum Laser Head PRO
- Added support for head-installed _external probe_, `M746 S2` selects it
- Expanded `G38` for external-probing with movement on all the three axes
- Better emergency stop in case of head faults
- `M793 S0` can be issued to shut-down all head lines and functions
- Changes in tool configuration
  - `M563` accepts multiple values per parameter, e.g. `M563 P0 D0 H0:1`
  - Revised heaters numbering:
    - 0: bed heater
    - 1: head heater
    - 4: bed temp sensor with disabled heater
    - 5: head temp sensor with disabled heater
  - Revised `M563` command output
- Added support for a motor driver (drive n.3) through bed pins; no DIR pin available

For the complete changelog, see [ChangeLog](ChangeLog.txt).


Contributors
------------

FABlin is possible thanks to the contributions by:

- Simone Cociancich
- Marco Rizzuto
- Daniel Kesler
- imarin2
- Krios Mane
- Enrico Ambrosini
- Wolfgang Meyerle
