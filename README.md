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
Now FABlin requires the custom _SmartComm_ module, stored inside libraries. To correctly compile the firmware with Arduino you must set the folder where this repository is checked out as Arduino's sketchbook folder inside Arudino's preferences. Alternatively you can manually copy the [FABlin/libraries/SmartComm](libraries/SmartComm) folder into your current sketchbook's libraries folder.

For further info refer to https://www.arduino.cc/en/Guide/Environment#toc7.


Changelog
---------

### Version: 1.0.0096 (development)
* Support for configurable communication with mounted heads
* New g-codes:
  - `M563` Configure tools
  - `M575` Set communication interface parameters
  - `M790` Send one or multiple commands to the head

### Version: 1.0.0095.1 - 2016-05-02
* Fixed a bug with timing when using M3,M4,M5
* Subtractive gcodes will not require a G4 before M5 or after M3 and M4 like before..
  M3,M4 and M5 will in fact start spinning after 1.5 seconds the last movement operation is completed.
  This allows to use some standard Gcode Posts processors instead of manually editing the code with pauses

### Version: 1.0.0095 - 2016-04-22

* Default Baud rate changed to 250000 bps.
* General Error management & Endstop management revamp.
* Changed sound notifications
* Moved G28 homing position to the effective center of the build area point of rotation when setting the screws.The point is in the center of the print area in the V2 heated bed.
* Fixed G28 inability to probe the center of the work-area when z_max_endstop is triggered
* Sounds alerts revamp to be more clear, silent mode can be triggered on some commands that usually emit sound.
*  M735 S1-0 enable /disable silent mode (less beeps all around)
* If door safety is enabled, opening the door will trigger an instant emergency stop of all movement and routines, reverting the unit to a safe situation. Door safety can still be enabled or disabled from the FABUI’s settings>hardware menu.
* Inactivity timer changed to 10 minutes for longer heating times.
* Fan management for V1 and V2 heads (SELECTABLE_AUTO_FAN_ON_TEMP_CHANGE behaviour modified: V2 heads won't have mandatory auto-cooling unless wanted, while V1 will have auto-cooling unless configured as disabled.) This reduces the heating time with V2 heads while avoiding clogging due to prolonged overheating in V1 heads.
* Added optional external power supply shutdown procedure (M786).
* Added new error codes. Min_temp extrusion now is triggered on the FABUI as well:
   * ERROR_AMBIENT_TEMP 122 (when ambient temp is < extruder_0_mintemp
   * ERROR_EXTRUDE_MINTEMP 123 (when cold extrusion is triggered, M302 S0
   * to disable)
   * ERROR_LONG_EXTRUSION 124 (when extruding too much!)
* Added command M793 to set the installed head version (dev only).

   to set it:    
    M793 S1  (V1 print+mill)
    M793 S2  (V2 print only)
    M793 S3  (V2 milling only)
    M793 S4-99 (reserved)


   * All codes related to the head have been discontinued in favour of a single ID.
The process of setting an installed head and all the operating params is manual.
The FABUI will load the defaults for the requested ID. (e.g for the milling head V2)


This also helps with "dumb" heads where no IC is required on board for identification, like the Head Development Kit).
the following codes are therefore discontinued as a method to recognize the head.


M780, - was: read Head Product Name
M781 - was: read Head Vendor Name
M782 - was: read Head product ID
M783 - was: read Head vendor ID
M784 - was: read Head Serial ID
M785 - was: read Head firmware version
M786 - was: read needed firmware version of FABtotum Personal Fabricator Main Controller
M787 - was: read Head capability: type0 (passive, active)
M788 - was: read Head capability: type1 (additive, milling, syringe, laser etc..)
M789 - was: read Head capability: purpose (single purpose, multipurpose)
M790 - was: read Head capability: wattage (0-200W)
M791 - was: read Head capability: axis (number of axis)
M792 - was: read Head capability: servo (number of axis)


While they still may be present in the code, less and less use of this functionalities will be implemented in favour of a single on-board database of functionalities.
Please consider ID between 1 and 99 as reserved for official heads to avoid a configuration conflict between heads.
Custom heads with IC on board, can have any ID from 99. Check the FAB-ESC firmware for Head recognition implementation: https://github.com/FABtotum/FAB-ESC
