/* -*- c++ -*- */

/*
    Reprap firmware based on Sprinter and grbl.
 Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm

 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 This firmware is a mashup between Sprinter and grbl.
  (https://github.com/kliment/Sprinter)
  (https://github.com/simen/grbl/tree)

 It has preliminary support for Matthew Roberts advance algorithm
    http://reprap.org/pipermail/reprap-dev/2011-May/003323.html
 */

#include "Marlin.h"

#ifdef ENABLE_AUTO_BED_LEVELING
#include "vector_3.h"
  #ifdef AUTO_BED_LEVELING_GRID
    #include "qr_solve.h"
  #endif
#endif // ENABLE_AUTO_BED_LEVELING

#include "ultralcd.h"
#include "planner.h"
#include "stepper.h"
#include "temperature.h"
#include "motion_control.h"
#include "cardreader.h"
#include "watchdog.h"
#include "ConfigurationStore.h"
#include "language.h"
#include "pins_arduino.h"
#include "math.h"
#include "Wire.h"

#ifdef BLINKM
#include "BlinkM.h"
#include "Wire.h"
#endif

#if NUM_SERVOS > 0
#include "Servo.h"
#endif

#if defined(DIGIPOTSS_PIN) && DIGIPOTSS_PIN > -1
#include <SPI.h>
#endif


// look here for descriptions of G-codes: http://linuxcnc.org/handbook/gcode/g-code.html
// http://objects.reprap.org/wiki/Mendel_User_Manual:_RepRapGCodes

//Implemented Codes
//-------------------
// G0  -> G1
// G1  - Coordinated Movement X Y Z E
// G2  - CW ARC
// G3  - CCW ARC
// G4  - Dwell S<seconds> or P<milliseconds>
// G10 - retract filament according to settings of M207
// G11 - retract recover filament according to settings of M208
// G28 - Home all Axis
// G29 - Detailed Z-Probe, probes the bed at 3 or more points.  Will fail if you haven't homed yet.
// G30 - Single Z Probe, probes bed at current XY location S<mm> searching Z length
// G90 - Use Absolute Coordinates
// G91 - Use Relative Coordinates
// G92 - Set current position to coordinates given

// M Codes
// M0   - Unconditional stop - Wait for user to press a button on the LCD (Only if ULTRA_LCD is enabled)
// M1   - Same as M0
// M17  - Enable/Power all stepper motors
// M18  - Disable all stepper motors; same as M84
// M20  - List SD card
// M21  - Init SD card
// M22  - Release SD card
// M23  - Select SD file (M23 filename.g)
// M24  - Start/resume SD print
// M25  - Pause SD print
// M26  - Set SD position in bytes (M26 S12345)
// M27  - Report SD print status
// M28  - Start SD write (M28 filename.g)
// M29  - Stop SD write
// M30  - Delete file from SD (M30 filename.g)
// M31  - Output time since last M109 or SD card start to serial
// M32  - Select file and start SD print (Can be used _while_ printing from SD card files):
//        syntax "M32 /path/filename#", or "M32 S<startpos bytes> !filename#"
//        Call gcode file : "M32 P !filename#" and return to caller file after finishing (similar to #include).
//        The '#' is necessary when calling from within sd files, as it stops buffer prereading
// M42  - Change pin status via gcode Use M42 Px Sy to set pin x to value y, when omitting Px the onboard led will be used.
// M80  - Turn on Power Supply
// M81  - Turn off Power Supply
// M82  - Set E codes absolute (default)
// M83  - Set E codes relative while in Absolute Coordinates (G90) mode
// M84  - Disable steppers until next move,
//        or use S<seconds> to specify an inactivity timeout, after which the steppers will be disabled.  S0 to disable the timeout.
// M85  - Set inactivity shutdown timer with parameter S<seconds>. To disable set zero (default)
// M92  - Set axis_steps_per_unit - same syntax as G92
// M104 - Set extruder target temp
// M105 - Read current temp
// M106 - Fan on
// M107 - Fan off
// M109 - Sxxx Wait for extruder current temp to reach target temp. Waits only when heating
//        Rxxx Wait for extruder current temp to reach target temp. Waits when heating and cooling
//        IF AUTOTEMP is enabled, S<mintemp> B<maxtemp> F<factor>. Exit autotemp by any M109 without F
// M114 - Output current position to serial port
// M115 - Capabilities string
// M117 - display message
// M119 - Output Endstop status to serial port
// M126 - Solenoid Air Valve Open (BariCUDA support by jmil)
// M127 - Solenoid Air Valve Closed (BariCUDA vent to atmospheric pressure by jmil)
// M128 - EtoP Open (BariCUDA EtoP = electricity to air pressure transducer by jmil)
// M129 - EtoP Closed (BariCUDA EtoP = electricity to air pressure transducer by jmil)
// M140 - Set bed target temp
// M150 - Set BlinkM Color Output R: Red<0-255> U(!): Green<0-255> B: Blue<0-255> over i2c, G for green does not work.
// M190 - Sxxx Wait for bed current temp to reach target temp. Waits only when heating
//        Rxxx Wait for bed current temp to reach target temp. Waits when heating and cooling
// M200 D<millimeters>- set filament diameter and set E axis units to cubic millimeters (use S0 to set back to millimeters).
// M201 - Set max acceleration in units/s^2 for print moves (M201 X1000 Y1000)
// M202 - Set max acceleration in units/s^2 for travel moves (M202 X1000 Y1000) Unused in Marlin!!
// M203 - Set maximum feedrate that your machine can sustain (M203 X200 Y200 Z300 E10000) in mm/sec
// M204 - Set default acceleration: S normal moves T filament only moves (M204 S3000 T7000) in mm/sec^2  also sets minimum segment time in ms (B20000) to prevent buffer under-runs and M20 minimum feedrate
// M205 -  advanced settings:  minimum travel speed S=while printing T=travel only,  B=minimum segment time X= maximum xy jerk, Z=maximum Z jerk, E=maximum E jerk
// M206 - set additional homing offset
// M207 - set retract length S[positive mm] F[feedrate mm/min] Z[additional zlift/hop], stays in mm regardless of M200 setting
// M208 - set recover=unretract length S[positive mm surplus to the M207 S*] F[feedrate mm/sec]
// M209 - S<1=true/0=false> enable automatic retract detect if the slicer did not support G10/11: every normal extrude-only move will be classified as retract depending on the direction.
// M218 - set hotend offset (in mm): T<extruder_number> X<offset_on_X> Y<offset_on_Y>
// M220 S<factor in percent>- set speed factor override percentage
// M221 S<factor in percent>- set extrude factor override percentage
// M226 P<pin number> S<pin state>- Wait until the specified pin reaches the state required
// M240 - Trigger a camera to take a photograph
// M250 - Set LCD contrast C<contrast value> (value 0..63)
// M280 - set servo position absolute. P: servo index, S: angle or microseconds
// M300 - Play beep sound S<frequency Hz> P<duration ms>
// M301 - Set PID parameters P I and D
// M302 - Allow cold extrudes, or set the minimum extrude S<temperature>.
// M303 - PID relay autotune S<temperature> sets the target temperature. (default target temperature = 150C)
// M304 - Set bed PID parameters P I and D
// M350 - Set microstepping mode.
// M351 - Toggle MS1 MS2 pins directly.
// M400 - Finish all moves
// M401 - Lower z-probe if present
// M402 - Raise z-probe if present
// M500 - stores parameters in EEPROM
// M501 - reads parameters from EEPROM (if you need reset them after you changed them temporarily).
// M502 - reverts to the default "factory settings".  You still need to store them in EEPROM afterwards if you want to.
// M503 - print the current settings (from memory not from EEPROM)
// M540 - Use S[0|1] to enable or disable the stop SD card print on endstop hit (requires ABORT_ON_ENDSTOP_HIT_FEATURE_ENABLED)
// M600 - Pause for filament change X[pos] Y[pos] Z[relative lift] E[initial retract] L[later retract distance for removal]
// M605 - Set dual x-carriage movement mode: S<mode> [ X<duplication x-offset> R<duplication temp offset> ]
// M665 - set delta configurations
// M666 - set delta endstop adjustment
// M907 - Set digital trimpot motor current using axis codes.
// M908 - Control digital trimpot directly.
// M928 - Start SD logging (M928 filename.g) - ended by M29
// M999 - Restart after being stopped by error

//FABtotum custom M codes
//-----------------------
// M3 S[RPM] SPINDLE ON - Clockwise
// M4 S[RPM] SPINDLE ON - CounterClockwise
// M5        SPINDLE OFF

// M563 [Pn [D<0-2>] [S<0,1>]] - Edit tool definition or query defined tools

// M700 S<0-255> - Laser Power Control
// M701 S<0-255> - Ambient Light, Set Red
// M702 S<0-255> - Ambient Light, Set Green
// M703 S<0-255> - Ambient Light, Set Blue
// M704 - Signalling Light ON (same colors of Ambient Light)
// M705 - Signalling Light OFF
// M706 S<0-255> - Head Light

// M710 - write and store in eeprom calibrated z_probe offset length
// M711 - write and store in eeprom calibrated zprobe extended angle
// M712 - write and store in eeprom calibrated zprobe retacted angle
// M713 - autocalibration of z-probe length and store in eeprom
// M714 - alternate the X axis endstop (M714 S0 use standard X axis endstop, M714 S1 use X axis max endstop)

// M720 - 24VDC head power ON
// M721 - 24VDC head power OFF
// M722 - 5VDC SERVO_1 power ON
// M723 - 5VDC SERVO_1 power OFF
// M724 - 5VDC SERVO_2 power ON
// M725 - 5VDC SERVO_2 power OFF
// M726 - 5VDC RASPBERRY PI power ON
// M727 - 5VDC RASPBERRY PI power OFF
// M728	- RASPBERRY Alive Command
// M729 - RASPBERRY Sleep                    //wait for the complete shutdown of raspberryPI
// M730 - Read last error code
// M731 - Disable kill on Door Open
// M732 - Enable or disable the permanent door security switch (M732 S0 -> disable (unsafe), M732 S1 -> enable (safe))

// M734   Enable /disable Endstop warnings
// M735   Enable /disable silent mode (sounds except for power-on)
// M740 - read WIRE_END sensor
// M741 - read DOOR_OPEN sensor
// M742 - read REEL_LENS_OPEN sensor
// M743 - read SECURE_SWITCH sensor
// M744 - read HOT_BED placed in place
// M745 - read Head placed in place

// M750 - read PRESSURE sensor (ANALOG 0-1023)
// M751 - read voltage monitor 24VDC input supply (ANALOG V)
// M752 - read voltage monitor 5VDC input supply (ANALOG V)
// M753 - read current monitor input supply (ANALOG A)
// M754 - read tempearture raw values (10bit ADC output)

// M760 - read FABtotum Personal Fabricator Main Controller serial ID
// M761 - read FABtotum Personal Fabricator Main Controller control code of serial ID
// M762 - read FABtotum Personal Fabricator Main Controller board version number
// M763 - read FABtotum Personal Fabricator Main Controller production batch number
// M764 - read FABtotum Personal Fabricator Main Controller control code of production batch number
// M765 - read FABtotum Personal Fabricator Firmware Version
// M766 - read FABtotum Personal Fabricator Firmware Build Date and Time
// M767 - read FABtotum Personal Fabricator Firmware Update Author


// M779 - force Head product ID reading (for testing purpose only)
// [unimplemented] M780 - read Head Product Name
// [unimplemented] M781 - read Head Vendor Name
// M782 - read Head product ID
// [unimplemented] M783 - read Head vendor ID
// M784 - read Head Serial ID
// M785 - Turn Prism UV module On/off M785 S[0-1] // [overrides] M785 - read Head firmware version
// M786 - External Power OFF // [overrides] M786 - read needed firmware version of FABtotum Personal Fabricator Main Controller
// M787 - external power on/off pin control // [overrides] M787 - read Head capability: type0 (passive, active)
// [unimplemented] M788 - read Head capability: type1 (additive, milling, syringe, laser etc..)
// [unimplemented] M789 - read Head capability: purpose (single purpose, multipurpose)
// [unimplemented] M790 - read Head capability: wattage (0-200W)
// [unimplemented] M791 - read Head capability: axis (number of axis)
// [unimplemented] M792 - read Head capability: servo (number of axis)
// M793 - set/read installed head soft ID

// M800 - changes/reads the thermistor of extruder0 type index
// M801 - changes/reads the current extruder0 max temp
// M802 - returns supported thermistor types by index
// M803 - changes/reads the current extruder0 thermistor input
// M804 - changes/reads the current automatic fan on temp change configuration.

// M998 - Restart after being killed
// M999 - Restart after being stopped

//Stepper Movement Variables

//===========================================================================
//=============================imported variables============================
//===========================================================================


//===========================================================================
//=============================public variables=============================
//===========================================================================
#ifdef SDSUPPORT
CardReader card;
#endif
bool PRISM;
bool red_led=false;
float homing_feedrate[] = HOMING_FEEDRATE;
bool axis_relative_modes[] = AXIS_RELATIVE_MODES;
int feedmultiply=100; //100->1 200->2
int saved_feedmultiply;
int extrudemultiply=100; //100->1 200->2
int extruder_multiply[EXTRUDERS] = {100
  #if EXTRUDERS > 1
    , 100
    #if EXTRUDERS > 2
      , 100
    #endif
  #endif
};
float volumetric_multiplier[EXTRUDERS] = {1.0
  #if EXTRUDERS > 1
    , 1.0
    #if EXTRUDERS > 2
      , 1.0
    #endif
  #endif
};
float current_position[NUM_AXIS] = { 0.0, 0.0, 0.0, 0.0 };
float add_homeing[3]={0,0,0};
#ifdef DELTA
float endstop_adj[3]={0,0,0};
#endif
float min_pos[3] = { X_MIN_POS, Y_MIN_POS, Z_MIN_POS };
float max_pos[3] = { X_MAX_POS, Y_MAX_POS, Z_MAX_POS };
bool axis_known_position[3] = {false, false, false};
float zprobe_zoffset;
bool inactivity = true;
bool silent=false;

//endstop configs
bool X_MIN_ENDSTOP_INVERTING = false; // set to true to invert the logic of the endstop.
bool Y_MIN_ENDSTOP_INVERTING = true; // set to true to invert the logic of the endstop.
bool Z_MIN_ENDSTOP_INVERTING = true; // set to true to invert the logic of the endstop.
bool X_MAX_ENDSTOP_INVERTING = false; // set to true to invert the logic of the endstop.
bool Y_MAX_ENDSTOP_INVERTING = true; // set to true to invert the logic of the endstop.
bool Z_MAX_ENDSTOP_INVERTING = true; // set to true to invert the logic of the endstop.

// Extruder offset
#if EXTRUDERS > 1
#ifndef DUAL_X_CARRIAGE
  #define NUM_EXTRUDER_OFFSETS 2 // only in XY plane
#else
  #define NUM_EXTRUDER_OFFSETS 3 // supports offsets in XYZ plane
#endif
float extruder_offset[NUM_EXTRUDER_OFFSETS][EXTRUDERS] = {
#if defined(EXTRUDER_OFFSET_X) && defined(EXTRUDER_OFFSET_Y)
  EXTRUDER_OFFSET_X, EXTRUDER_OFFSET_Y
#endif
};
#endif
uint8_t active_tool = 0;     // Active logical tool
uint8_t active_extruder = 0; // Active actual extruder
bool head_is_dummy = false;  // Head reaquires TWI silencing
uint8_t tool_extruder_mapping[] = { 0, 1, 2 };  // Tool to drive mapping
uint8_t tool_heater_mapping[]   = { 0, -1, 0 };  // Tool to heater mapping
bool    tool_twi_support[]      = { true, false, false };  // Tool TWI support
int fanSpeed=0;
#ifdef SERVO_ENDSTOPS
  int servo_endstops[] = SERVO_ENDSTOPS;
  int servo_endstop_angles[] = SERVO_ENDSTOP_ANGLES;
#endif
#ifdef BARICUDA
int ValvePressure=0;
int EtoPPressure=0;
#endif

#ifdef FWRETRACT
  bool autoretract_enabled=false;
  bool retracted=false;
  float retract_length = RETRACT_LENGTH;
  float retract_feedrate = RETRACT_FEEDRATE;
  float retract_zlift = RETRACT_ZLIFT;
  float retract_recover_length = RETRACT_RECOVER_LENGTH;
  float retract_recover_feedrate = RETRACT_RECOVER_FEEDRATE;
#endif

#ifdef ULTIPANEL
  #ifdef PS_DEFAULT_OFF
    bool powersupply = false;
  #else
	  bool powersupply = true;
  #endif
#endif

#ifdef DELTA
  float delta[3] = {0.0, 0.0, 0.0};
  #define SIN_60 0.8660254037844386
  #define COS_60 0.5
  // these are the default values, can be overriden with M665
  float delta_radius= DELTA_RADIUS;
  float delta_tower1_x= -SIN_60*delta_radius; // front left tower
  float delta_tower1_y= -COS_60*delta_radius;
  float delta_tower2_x=  SIN_60*delta_radius; // front right tower
  float delta_tower2_y= -COS_60*delta_radius;
  float delta_tower3_x= 0.0;                  // back middle tower
  float delta_tower3_y= delta_radius;
  float delta_diagonal_rod= DELTA_DIAGONAL_ROD;
  float delta_diagonal_rod_2= sq(delta_diagonal_rod);
  float delta_segments_per_second= DELTA_SEGMENTS_PER_SECOND;
#endif


int servo_extended_angle=servo_endstop_angles[4];
int servo_retracted_angle=servo_endstop_angles[5];

unsigned long fab_serial_code=0;
unsigned long fab_control_serial_code=0;
unsigned int fab_board_version=0;
unsigned int fab_batch_number=0;
unsigned long fab_control_batch_number=0;
unsigned int led_board_version=0;
unsigned int flex_board_version=0;
unsigned int plateconn_board_version=0;
unsigned int hotplate_board_version=0;
unsigned int general_assembly_version=0;
unsigned int installed_head_id=0;

bool head_placed = false;

//===========================================================================
//=============================Private Variables=============================
//===========================================================================
const char axis_codes[NUM_AXIS] = {'X', 'Y', 'Z', 'E'};
static float destination[NUM_AXIS] = {  0.0, 0.0, 0.0, 0.0};
static float offset[3] = {0.0, 0.0, 0.0};
static bool home_all_axis = true;
static float feedrate = 1500.0, next_feedrate, saved_feedrate;
static long gcode_N, gcode_LastN, Stopped_gcode_LastN = 0;

static bool relative_mode = false;  //Determines Absolute or Relative Coordinates

static char cmdbuffer[BUFSIZE][MAX_CMD_SIZE];
static bool fromsd[BUFSIZE];
static int bufindr = 0;
static int bufindw = 0;
static int buflen = 0;
//static int i = 0;
static char serial_char;
static int serial_count = 0;
static boolean comment_mode = false;
static char *strchr_pointer; // just a pointer to find chars in the command string like X, Y, Z, E, etc

const int sensitive_pins[] = SENSITIVE_PINS; // Sensitive pin list for M42

//static float tt = 0;
//static float bt = 0;

//Inactivity shutdown variables
static unsigned long previous_millis_cmd = 0;
static unsigned long max_inactive_time = DEFAULT_DEACTIVE_TIME*1000l;
static unsigned long max_steppers_inactive_time = DEFAULT_STEPPERS_DEACTIVE_TIME*1000l;

unsigned long starttime=0;
unsigned long stoptime=0;

static uint8_t tmp_extruder;


bool Stopped=false;

#if NUM_SERVOS > 0
  Servo servos[NUM_SERVOS];
#endif

bool CooldownNoWait = true;
bool target_direction;

//Insert variables if CHDK is defined
#ifdef CHDK
unsigned long chdkHigh = 0;
boolean chdkActive = false;
#endif

unsigned int LaserSoftPwm;
unsigned int FabSoftPwm_TMR;
unsigned int HeadLightSoftPwm;
unsigned int FabSoftPwm_LMT;
unsigned int RedSoftPwm;
unsigned int GreenSoftPwm;
unsigned int BlueSoftPwm;
unsigned int RedSoftPwm_old;
unsigned int GreenSoftPwm_old;
unsigned int BlueSoftPwm_old;

bool triggered_kill=false;
bool enable_door_kill=true;
bool enable_permanent_door_kill=true;
bool rpi_recovery_flag=false;

#ifdef EXTERNAL_ENDSTOP_Z_PROBING
bool enable_secure_switch_zprobe=false;
#endif

float rpm = 0;

unsigned int fading_speed=100;
unsigned int led_update_cycles=0;
bool red_fading=false;
bool green_fading=false;
bool blue_fading=false;
bool slope=true;
bool fading_started=false;

bool z_probe_activation=true;

bool home_Z_reverse=false;

bool x_axis_endstop_sel=false;
bool monitor_secure_endstop=false; //default is off

bool min_x_endstop_triggered=false;
bool max_x_endstop_triggered=false;
bool min_y_endstop_triggered=false;
bool max_y_endstop_triggered=false;

byte SERIAL_HEAD_0=0;
byte SERIAL_HEAD_1=0;
byte SERIAL_HEAD_2=0;
byte SERIAL_HEAD_3=0;
byte SERIAL_HEAD_4=0;
byte SERIAL_HEAD_5=0;
byte SERIAL_HEAD_6=0;
byte SERIAL_HEAD_7=0;


unsigned long i2c_pre_millis=0;
bool i2c_timeout=false;

bool zeroed_far_from_home_x=true;
bool zeroed_far_from_home_y=true;

float safe_probing_offset=1;        //it will probe until the (probe length - safe_probing_offset) is reached

#ifdef THERMISTOR_HOTSWAP
uint8_t extruder_0_thermistor_index = THERMISTOR_HOTSWAP_DEFAULT_INDEX;
#endif

#ifdef SELECTABLE_AUTO_FAN_ON_TEMP_CHANGE
bool auto_fan_on_temp_change = true;
#endif

//===========================================================================
//=============================Routines======================================
//===========================================================================

void get_arc_coordinates();
bool setTargetedHotend(int code);

void serial_echopair_P(const char *s_P, float v)
    { serialprintPGM(s_P); SERIAL_ECHO(v); }
void serial_echopair_P(const char *s_P, double v)
    { serialprintPGM(s_P); SERIAL_ECHO(v); }
void serial_echopair_P(const char *s_P, unsigned long v)
    { serialprintPGM(s_P); SERIAL_ECHO(v); }

extern "C"{
  extern unsigned int __bss_end;
  extern unsigned int __heap_start;
  extern void *__brkval;

  int freeMemory() {
    int free_memory;

    if((int)__brkval == 0)
      free_memory = ((int)&free_memory) - ((int)&__bss_end);
    else
      free_memory = ((int)&free_memory) - ((int)__brkval);

    return free_memory;
  }
}

//adds an command to the main command buffer
//thats really done in a non-safe way.
//needs overworking someday
void enquecommand(const char *cmd)
{
  if(buflen < BUFSIZE)
  {
    //this is dangerous if a mixing of serial and this happens
    strcpy(&(cmdbuffer[bufindw][0]),cmd);
    SERIAL_ECHO_START;
    SERIAL_ECHOPGM("enqueing \"");
    SERIAL_ECHO(cmdbuffer[bufindw]);
    SERIAL_ECHOLNPGM("\"");
    bufindw= (bufindw + 1)%BUFSIZE;
    buflen += 1;
  }
}

void enquecommand_P(const char *cmd)
{
  if(buflen < BUFSIZE)
  {
    //this is dangerous if a mixing of serial and this happens
    strcpy_P(&(cmdbuffer[bufindw][0]),cmd);
    SERIAL_ECHO_START;
    SERIAL_ECHOPGM("enqueing \"");
    SERIAL_ECHO(cmdbuffer[bufindw]);
    SERIAL_ECHOLNPGM("\"");
    bufindw= (bufindw + 1)%BUFSIZE;
    buflen += 1;
  }
}

void setup_killpin()
{
  #if defined(KILL_PIN) && KILL_PIN > -1
    pinMode(KILL_PIN,INPUT);
    WRITE(KILL_PIN,HIGH);
  #endif
}

void setup_photpin()
{
  #if defined(PHOTOGRAPH_PIN) && PHOTOGRAPH_PIN > -1
    SET_OUTPUT(PHOTOGRAPH_PIN);
    WRITE(PHOTOGRAPH_PIN, LOW);
  #endif
}



void setup_powerhold()
{
  #if defined(SUICIDE_PIN) && SUICIDE_PIN > -1
    SET_OUTPUT(SUICIDE_PIN);
    WRITE(SUICIDE_PIN, HIGH);
  #endif
  #if defined(PS_ON_PIN) && PS_ON_PIN > -1
    SET_OUTPUT(PS_ON_PIN);
	#if defined(PS_DEFAULT_OFF)
	  WRITE(PS_ON_PIN, PS_ON_ASLEEP);
    #else
	  WRITE(PS_ON_PIN, PS_ON_AWAKE);
	#endif
  #endif
}

void suicide()
{
  #if defined(SUICIDE_PIN) && SUICIDE_PIN > -1
    SET_OUTPUT(SUICIDE_PIN);
    WRITE(SUICIDE_PIN, LOW);
  #endif
}

void servo_init()
{
  #if (NUM_SERVOS >= 1) && defined(SERVO0_PIN) && (SERVO0_PIN > -1)
    servos[0].attach(SERVO0_PIN);
  #endif
  #if (NUM_SERVOS >= 2) && defined(SERVO1_PIN) && (SERVO1_PIN > -1)
    servos[1].attach(SERVO1_PIN);
  #endif
  #if (NUM_SERVOS >= 3) && defined(SERVO2_PIN) && (SERVO2_PIN > -1)
    servos[2].attach(SERVO2_PIN);
  #endif
  #if (NUM_SERVOS >= 4) && defined(SERVO3_PIN) && (SERVO3_PIN > -1)
    servos[3].attach(SERVO3_PIN);
  #endif
  #if (NUM_SERVOS >= 5)
    #error "TODO: enter initalisation code for more servos"
  #endif

  // Set position of Servo Endstops that are defined
  #ifdef SERVO_ENDSTOPS
  for(int8_t i = 0; i < 3; i++)
  {
    if(servo_endstops[i] > -1) {
      servos[servo_endstops[i]].write(servo_endstop_angles[i * 2 + 1]);
    }
  }
  #endif

  #if defined (ENABLE_AUTO_BED_LEVELING) && (PROBE_SERVO_DEACTIVATION_DELAY > 0)
  delay(PROBE_SERVO_DEACTIVATION_DELAY);
  servos[servo_endstops[Z_AXIS]].detach();
  #endif
}

void defineTool(uint8_t tool, unsigned int drive=0, unsigned int heater=0, bool twi=true)
{
   tool_extruder_mapping[tool] = drive;
   tool_heater_mapping[tool] = heater;
   tool_twi_support[tool] = twi;
}

uint8_t loadTool (uint8_t tool)
{
   active_tool = tool;
   active_extruder = tool_extruder_mapping[tool];
   head_is_dummy = !tool_twi_support[tool];

   if (head_is_dummy) {
      TWCR &= ~MASK(TWEN);
   } else {
      Wire.begin();
   }

   return active_extruder;
}

void FabtotumIO_init()
{
   BEEP_ON()

   pinMode(RED_PIN,OUTPUT);
   pinMode(GREEN_PIN,OUTPUT);
   SET_OUTPUT(BLUE_PIN);
   pinMode(HOT_LED_PIN,OUTPUT);
   pinMode(DOOR_OPEN_PIN,INPUT);
   pinMode(HEAD_LIGHT_PIN,OUTPUT);
   pinMode(LASER_GATE_PIN,OUTPUT);
   pinMode(MILL_MOTOR_ON_PIN,OUTPUT);
   pinMode(NOT_SERVO1_ON_PIN,OUTPUT);
   pinMode(NOT_SERVO2_ON_PIN,OUTPUT);

   //setting analog as input
   pinMode(MAIN_CURRENT_SENSE_PIN,INPUT);
   pinMode(MON_5V_PIN,INPUT);
   pinMode(MON_24V_PIN,INPUT);
   pinMode(PRESSURE_ANALOG_PIN,INPUT);

   pinMode(NOT_REEL_LENS_OPEN_PIN,OUTPUT);

   //fastio init
   // SET_INPUT(IO)  ; SET_OUTPUT(IO);
   SET_INPUT(WIRE_END_PIN);
   SET_OUTPUT(NOT_RASPI_PWR_ON_PIN);
   SET_INPUT(NOT_SECURE_SW_PIN);
   SET_OUTPUT(NOT_REEL_LENS_OPEN_PIN);
   SET_OUTPUT(LIGHT_SIGN_ON_PIN);
   SET_OUTPUT(RPI_RECOVERY_PIN);

   //set output
   RED_OFF();
   GREEN_OFF();
   BLUE_OFF();
   analogWrite(BLUE_PIN,255);

   //analogWrite(HEATER_0_PIN,0);

   HOT_LED_OFF();

   HEAD_LIGHT_OFF();
   LASER_GATE_OFF();

   MILL_MOTOR_OFF();

   SERVO1_OFF();
   SERVO2_OFF();

   RASPI_PWR_ON();

   LIGHT_SIGN_ON();

   //TCCR1A= _BV(COM1A1) | _BV(COM1B1) | _BV(WGM10);
   //TCCR1B= _BV(WGM12) | _BV(CS11) | _BV(CS10);

   FabSoftPwm_TMR=0;
   FabSoftPwm_LMT=MAX_PWM;
   HeadLightSoftPwm=0;
   LaserSoftPwm=0;
   RedSoftPwm=0;
   GreenSoftPwm=0;
   BlueSoftPwm=0;

   servos[0].write(SERVO_SPINDLE_ZERO);       //set Zero POS for SERVO1  (MILL MOTOR input: 1060 us equal to Full CCW, 1460us equal to zero, 1860us equal to Full CW)
   servos[1].write(950);        //set Zero POS for SERVO2  (servo probe)

   triggered_kill=false;
   enable_door_kill=true;
   rpm = 0;

   switch (installed_head_id)
   {
      case 4:  // Direct-drive?
         defineTool(2, 0, 0, true);
         defineTool(0, 2, 0, false);
         break;
      case 3:  // Milling V2
         defineTool(0, 0, 0, false);
         break;
      default:
         defineTool(0, 0, 0, true);
   }
   // Starting tool is #0 (T0)
   loadTool(0);

   /*fading_speed=200;
   fading_started=false;
   led_update_cycles=0;
   red_fading=false;
   green_fading=false;
   blue_fading=false;
   slope=true;*/
   set_amb_color(0,0,0);
   set_amb_color_fading(true,true,false,fading_speed);

   _delay_ms(50);
   BEEP_OFF()
   _delay_ms(30);
   BEEP_ON()
   _delay_ms(50);
   BEEP_OFF()
}

void setup()
{
  setup_killpin();
  setup_powerhold();
  MYSERIAL.begin(BAUDRATE);
  SERIAL_PROTOCOLLNPGM("start");
  SERIAL_ECHO_START;

  // Check startup - does nothing if bootloader sets MCUSR to 0
  byte mcu = MCUSR;
  if(mcu & 1) SERIAL_ECHOLNPGM(MSG_POWERUP);
  if(mcu & 2) SERIAL_ECHOLNPGM(MSG_EXTERNAL_RESET);
  if(mcu & 4) SERIAL_ECHOLNPGM(MSG_BROWNOUT_RESET);
  if(mcu & 8) SERIAL_ECHOLNPGM(MSG_WATCHDOG_RESET);
  if(mcu & 32) SERIAL_ECHOLNPGM(MSG_SOFTWARE_RESET);
  MCUSR=0;

  SERIAL_ECHOPGM(MSG_MARLIN_FABTOTUM);
  SERIAL_ECHOLNPGM(STRING_BUILD_VERSION);
  #ifdef STRING_BUILD_DATE
    #ifdef STRING_CONFIG_H_AUTHOR
      SERIAL_ECHO_START;
      SERIAL_ECHOPGM(MSG_CONFIGURATION_VER);
      SERIAL_ECHOPGM(STRING_BUILD_DATE);
      SERIAL_ECHOPGM(MSG_AUTHOR);
      SERIAL_ECHOLNPGM(STRING_CONFIG_H_AUTHOR);
    #endif
  #endif
  SERIAL_ECHO_START;
  SERIAL_ECHOPGM(MSG_FREE_MEMORY);
  SERIAL_ECHO(freeMemory());
  SERIAL_ECHOPGM(MSG_PLANNER_BUFFER_BYTES);
  SERIAL_ECHOLN((int)sizeof(block_t)*BLOCK_BUFFER_SIZE);
  for(int8_t i = 0; i < BUFSIZE; i++)
  {
    fromsd[i] = false;
  }

  // loads data from EEPROM if available else uses defaults (and resets step acceleration rate)
  Config_RetrieveSettings();

  //test
  //Config_StoreSettings();

  tp_init();    // Initialize temperature loop
  plan_init();  // Initialize planner;
  watchdog_init();
  st_init();    // Initialize stepper, this enables interrupts!
  setup_photpin();
  servo_init();

  FabtotumIO_init();

  lcd_init();
  _delay_ms(1000);	// wait 1sec to display the splash screen

  #if defined(CONTROLLERFAN_PIN) && CONTROLLERFAN_PIN > -1
    SET_OUTPUT(CONTROLLERFAN_PIN); //Set pin used for driver cooling fan
  #endif

  #ifdef DIGIPOT_I2C
    digipot_i2c_init();
  #endif
}


void loop()
{
  if(buflen < (BUFSIZE-1))
    get_command();
  #ifdef SDSUPPORT
  card.checkautostart(false);
  #endif
  if(buflen)
  {
    #ifdef SDSUPPORT
      if(card.saving)
      {
        if(strstr_P(cmdbuffer[bufindr], PSTR("M29")) == NULL)
        {
          card.write_command(cmdbuffer[bufindr]);
          if(card.logging)
          {
            process_commands();
          }
          else
          {
            SERIAL_PROTOCOLLNPGM(MSG_OK);
          }
        }
        else
        {
          card.closefile();
          SERIAL_PROTOCOLLNPGM(MSG_FILE_SAVED);
        }
      }
      else
      {
        process_commands();
      }
    #else
      process_commands();
    #endif //SDSUPPORT
    buflen = (buflen-1);
    bufindr = (bufindr + 1)%BUFSIZE;
  }
  //check heater every n milliseconds
  manage_heater();
  manage_inactivity();
  checkHitEndstops();
  lcd_update();
}

void get_command()
{
  while( MYSERIAL.available() > 0  && buflen < BUFSIZE) {
    serial_char = MYSERIAL.read();
    if(serial_char == '\n' ||
       serial_char == '\r' ||
       (serial_char == ':' && comment_mode == false) ||
       serial_count >= (MAX_CMD_SIZE - 1) )
    {
      if(!serial_count) { //if empty line
        comment_mode = false; //for new command
        return;
      }
      cmdbuffer[bufindw][serial_count] = 0; //terminate string
      if(!comment_mode){
        comment_mode = false; //for new command
        fromsd[bufindw] = false;
        if(strchr(cmdbuffer[bufindw], 'N') != NULL)
        {
          strchr_pointer = strchr(cmdbuffer[bufindw], 'N');
          gcode_N = (strtol(&cmdbuffer[bufindw][strchr_pointer - cmdbuffer[bufindw] + 1], NULL, 10));
          if(gcode_N != gcode_LastN+1 && (strstr_P(cmdbuffer[bufindw], PSTR("M110")) == NULL) ) {
            SERIAL_ERROR_START;
            SERIAL_ERRORPGM(MSG_ERR_LINE_NO);
            SERIAL_ERRORLN(gcode_LastN);
            //Serial.println(gcode_N);
            FlushSerialRequestResend();
            serial_count = 0;
            return;
          }

          if(strchr(cmdbuffer[bufindw], '*') != NULL)
          {
            byte checksum = 0;
            byte count = 0;
            while(cmdbuffer[bufindw][count] != '*') checksum = checksum^cmdbuffer[bufindw][count++];
            strchr_pointer = strchr(cmdbuffer[bufindw], '*');

            if( (int)(strtod(&cmdbuffer[bufindw][strchr_pointer - cmdbuffer[bufindw] + 1], NULL)) != checksum) {
              SERIAL_ERROR_START;
              SERIAL_ERRORPGM(MSG_ERR_CHECKSUM_MISMATCH);
              SERIAL_ERRORLN(gcode_LastN);
              FlushSerialRequestResend();
              serial_count = 0;
              return;
            }
            //if no errors, continue parsing
          }
          else
          {
            SERIAL_ERROR_START;
            SERIAL_ERRORPGM(MSG_ERR_NO_CHECKSUM);
            SERIAL_ERRORLN(gcode_LastN);
            FlushSerialRequestResend();
            serial_count = 0;
            return;
          }

          gcode_LastN = gcode_N;
          //if no errors, continue parsing
        }
        else  // if we don't receive 'N' but still see '*'
        {
          if((strchr(cmdbuffer[bufindw], '*') != NULL))
          {
            SERIAL_ERROR_START;
            SERIAL_ERRORPGM(MSG_ERR_NO_LINENUMBER_WITH_CHECKSUM);
            SERIAL_ERRORLN(gcode_LastN);
            serial_count = 0;
            return;
          }
        }
        if((strchr(cmdbuffer[bufindw], 'G') != NULL)){
          strchr_pointer = strchr(cmdbuffer[bufindw], 'G');
          switch((int)((strtod(&cmdbuffer[bufindw][strchr_pointer - cmdbuffer[bufindw] + 1], NULL)))){
          case 0:
          case 1:
          case 2:
          case 3:
            if(Stopped == false) { // If printer is stopped by an error the G[0-3] codes are ignored.
          #ifdef SDSUPPORT
              if(card.saving)
                break;
          #endif //SDSUPPORT
              SERIAL_PROTOCOLLNPGM(MSG_OK);
            }
            else {
              SERIAL_ERRORLNPGM(MSG_ERR_STOPPED);
              LCD_MESSAGEPGM(MSG_STOPPED);
            }
            break;
          default:
            break;
          }

        }
        bufindw = (bufindw + 1)%BUFSIZE;
        buflen += 1;
      }
      serial_count = 0; //clear buffer
    }
    else
    {
      if(serial_char == ';') comment_mode = true;
      if(!comment_mode) cmdbuffer[bufindw][serial_count++] = serial_char;
    }
  }
  #ifdef SDSUPPORT
  if(!card.sdprinting || serial_count!=0){
    return;
  }

  //'#' stops reading from SD to the buffer prematurely, so procedural macro calls are possible
  // if it occurs, stop_buffering is triggered and the buffer is ran dry.
  // this character _can_ occur in serial com, due to checksums. however, no checksums are used in SD printing

  static bool stop_buffering=false;
  if(buflen==0) stop_buffering=false;

  while( !card.eof()  && buflen < BUFSIZE && !stop_buffering) {
    int16_t n=card.get();
    serial_char = (char)n;
    if(serial_char == '\n' ||
       serial_char == '\r' ||
       (serial_char == '#' && comment_mode == false) ||
       (serial_char == ':' && comment_mode == false) ||
       serial_count >= (MAX_CMD_SIZE - 1)||n==-1)
    {
      if(card.eof()){
        SERIAL_PROTOCOLLNPGM(MSG_FILE_PRINTED);
        stoptime=millis();
        char time[30];
        unsigned long t=(stoptime-starttime)/1000;
        int hours, minutes;
        minutes=(t/60)%60;
        hours=t/60/60;
        sprintf_P(time, PSTR("%i hours %i minutes"),hours, minutes);
        SERIAL_ECHO_START;
        SERIAL_ECHOLN(time);
        lcd_setstatus(time);
        card.printingHasFinished();
        card.checkautostart(true);

      }
      if(serial_char=='#')
        stop_buffering=true;

      if(!serial_count)
      {
        comment_mode = false; //for new command
        return; //if empty line
      }
      cmdbuffer[bufindw][serial_count] = 0; //terminate string
//      if(!comment_mode){
        fromsd[bufindw] = true;
        buflen += 1;
        bufindw = (bufindw + 1)%BUFSIZE;
//      }
      comment_mode = false; //for new command
      serial_count = 0; //clear buffer
    }
    else
    {
      if(serial_char == ';') comment_mode = true;
      if(!comment_mode) cmdbuffer[bufindw][serial_count++] = serial_char;
    }
  }

  #endif //SDSUPPORT

}


float code_value()
{
  return (strtod(&cmdbuffer[bufindr][strchr_pointer - cmdbuffer[bufindr] + 1], NULL));
}

long code_value_long()
{
  return (strtol(&cmdbuffer[bufindr][strchr_pointer - cmdbuffer[bufindr] + 1], NULL, 10));
}

bool code_seen(char code)
{
  strchr_pointer = strchr(cmdbuffer[bufindr], code);
  return (strchr_pointer != NULL);  //Return True if a character was found
}

#define DEFINE_PGM_READ_ANY(type, reader)       \
    static inline type pgm_read_any(const type *p)  \
    { return pgm_read_##reader##_near(p); }

DEFINE_PGM_READ_ANY(float,       float);
DEFINE_PGM_READ_ANY(signed char, byte);

#define XYZ_CONSTS_FROM_CONFIG(type, array, CONFIG) \
static const PROGMEM type array##_P[3] =        \
    { X_##CONFIG, Y_##CONFIG, Z_##CONFIG };     \
static inline type array(int axis)          \
    { return pgm_read_any(&array##_P[axis]); }

XYZ_CONSTS_FROM_CONFIG(float, base_min_pos,    MIN_POS);
XYZ_CONSTS_FROM_CONFIG(float, base_max_pos,    MAX_POS);
XYZ_CONSTS_FROM_CONFIG(float, base_home_pos,   HOME_POS);
XYZ_CONSTS_FROM_CONFIG(float, max_length,      MAX_LENGTH);
XYZ_CONSTS_FROM_CONFIG(float, home_retract_mm, HOME_RETRACT_MM);
XYZ_CONSTS_FROM_CONFIG(signed char, home_dir,  HOME_DIR);

#ifdef DUAL_X_CARRIAGE
  #if EXTRUDERS == 1 || defined(COREXY) \
      || !defined(X2_ENABLE_PIN) || !defined(X2_STEP_PIN) || !defined(X2_DIR_PIN) \
      || !defined(X2_HOME_POS) || !defined(X2_MIN_POS) || !defined(X2_MAX_POS) \
      || !defined(X_MAX_PIN) || X_MAX_PIN < 0
    #error "Missing or invalid definitions for DUAL_X_CARRIAGE mode."
  #endif
  #if X_HOME_DIR != -1 || X2_HOME_DIR != 1
    #error "Please use canonical x-carriage assignment" // the x-carriages are defined by their homing directions
  #endif

#define DXC_FULL_CONTROL_MODE 0
#define DXC_AUTO_PARK_MODE    1
#define DXC_DUPLICATION_MODE  2
static int dual_x_carriage_mode = DEFAULT_DUAL_X_CARRIAGE_MODE;

static float x_home_pos(int extruder) {
  if (extruder == 0)
    return base_home_pos(X_AXIS) + add_homeing[X_AXIS];
  else
    // In dual carriage mode the extruder offset provides an override of the
    // second X-carriage offset when homed - otherwise X2_HOME_POS is used.
    // This allow soft recalibration of the second extruder offset position without firmware reflash
    // (through the M218 command).
    return (extruder_offset[X_AXIS][1] > 0) ? extruder_offset[X_AXIS][1] : X2_HOME_POS;
}

static int x_home_dir(int extruder) {
  return (extruder == 0) ? X_HOME_DIR : X2_HOME_DIR;
}

static float inactive_extruder_x_pos = X2_MAX_POS; // used in mode 0 & 1
static bool active_extruder_parked = false; // used in mode 1 & 2
static float raised_parked_position[NUM_AXIS]; // used in mode 1
static unsigned long delayed_move_time = 0; // used in mode 1
static float duplicate_extruder_x_offset = DEFAULT_DUPLICATION_X_OFFSET; // used in mode 2
static float duplicate_extruder_temp_offset = 0; // used in mode 2
bool extruder_duplication_enabled = false; // used in mode 2
#endif //DUAL_X_CARRIAGE

static void axis_is_at_home(int axis) {
#ifdef DUAL_X_CARRIAGE
  if (axis == X_AXIS) {
    if (active_extruder != 0) {
      current_position[X_AXIS] = x_home_pos(active_extruder);
      min_pos[X_AXIS] =          X2_MIN_POS;
      max_pos[X_AXIS] =          max(extruder_offset[X_AXIS][1], X2_MAX_POS);
      return;
    }
    else if (dual_x_carriage_mode == DXC_DUPLICATION_MODE && active_extruder == 0) {
      current_position[X_AXIS] = base_home_pos(X_AXIS) + add_homeing[X_AXIS];
      min_pos[X_AXIS] =          base_min_pos(X_AXIS) + add_homeing[X_AXIS];
      max_pos[X_AXIS] =          min(base_max_pos(X_AXIS) + add_homeing[X_AXIS],
                                  max(extruder_offset[X_AXIS][1], X2_MAX_POS) - duplicate_extruder_x_offset);
      return;
    }
  }
#endif
  current_position[axis] = base_home_pos(axis) + add_homeing[axis];
  min_pos[axis] =          base_min_pos(axis) + add_homeing[axis];
  max_pos[axis] =          base_max_pos(axis) + add_homeing[axis];

  if(axis==Z_AXIS && home_Z_reverse){current_position[axis] = (Z_MAX_POS+Z_PROBE_OFFSET_FROM_EXTRUDER);}
  if(axis==X_AXIS && x_axis_endstop_sel){current_position[axis] = (X_MAX_POS);}
}

#ifdef ENABLE_AUTO_BED_LEVELING
#ifdef AUTO_BED_LEVELING_GRID
static void set_bed_level_equation_lsq(double *plane_equation_coefficients)
{
    vector_3 planeNormal = vector_3(-plane_equation_coefficients[0], -plane_equation_coefficients[1], 1);
    planeNormal.debug("planeNormal");
    plan_bed_level_matrix = matrix_3x3::create_look_at(planeNormal);
    //bedLevel.debug("bedLevel");

    //plan_bed_level_matrix.debug("bed level before");
    //vector_3 uncorrected_position = plan_get_position_mm();
    //uncorrected_position.debug("position before");

    vector_3 corrected_position = plan_get_position();
//    corrected_position.debug("position after");
    current_position[X_AXIS] = corrected_position.x;
    current_position[Y_AXIS] = corrected_position.y;
    current_position[Z_AXIS] = corrected_position.z;

    // but the bed at 0 so we don't go below it.
    current_position[Z_AXIS] = zprobe_zoffset; // in the lsq we reach here after raising the extruder due to the loop structure

    plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
}

#else // not AUTO_BED_LEVELING_GRID

static void set_bed_level_equation_3pts(float z_at_pt_1, float z_at_pt_2, float z_at_pt_3) {

    plan_bed_level_matrix.set_to_identity();

    vector_3 pt1 = vector_3(ABL_PROBE_PT_1_X, ABL_PROBE_PT_1_Y, z_at_pt_1);
    vector_3 pt2 = vector_3(ABL_PROBE_PT_2_X, ABL_PROBE_PT_2_Y, z_at_pt_2);
    vector_3 pt3 = vector_3(ABL_PROBE_PT_3_X, ABL_PROBE_PT_3_Y, z_at_pt_3);

    vector_3 from_2_to_1 = (pt1 - pt2).get_normal();
    vector_3 from_2_to_3 = (pt3 - pt2).get_normal();
    vector_3 planeNormal = vector_3::cross(from_2_to_1, from_2_to_3).get_normal();
    //planeNormal = vector_3(planeNormal.x, planeNormal.y, abs(planeNormal.z));
    planeNormal = vector_3(planeNormal.x, planeNormal.y, abs(planeNormal.z));

    plan_bed_level_matrix = matrix_3x3::create_look_at(planeNormal);

    vector_3 corrected_position = plan_get_position();
    current_position[X_AXIS] = corrected_position.x;
    current_position[Y_AXIS] = corrected_position.y;
    current_position[Z_AXIS] = corrected_position.z;

    // put the bed at 0 so we don't go below it.
    current_position[Z_AXIS] = zprobe_zoffset;

    plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);

}

#endif // AUTO_BED_LEVELING_GRID

static void run_z_probe() {
    plan_bed_level_matrix.set_to_identity();
    feedrate = homing_feedrate[Z_AXIS];

    // move down until you find the bed
    float zPosition = -10;
    plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], zPosition, current_position[E_AXIS], feedrate/60, active_extruder);
    st_synchronize();

        // we have to let the planner know where we are right now as it is not where we said to go.
    zPosition = st_get_position_mm(Z_AXIS);
    plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], zPosition, current_position[E_AXIS]);

    // move up the retract distance
    zPosition += home_retract_mm(Z_AXIS);
    plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], zPosition, current_position[E_AXIS], feedrate/60, active_extruder);
    st_synchronize();

    // move back down slowly to find bed
    feedrate = homing_feedrate[Z_AXIS]/4;
    zPosition -= home_retract_mm(Z_AXIS) * 2;
    plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], zPosition, current_position[E_AXIS], feedrate/60, active_extruder);
    st_synchronize();

    current_position[Z_AXIS] = st_get_position_mm(Z_AXIS);
    // make sure the planner knows where we are as it may be a bit different than we last said to move to
    plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
}

static void run_fast_z_probe(float feedrateProbing) {
    plan_bed_level_matrix.set_to_identity();
    feedrate = feedrateProbing; //homing_feedrate[Z_AXIS];

    // move down until you find the bed
    //float zPosition = -10;
    float zPosition = zprobe_zoffset-safe_probing_offset;
    plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], zPosition, current_position[E_AXIS], feedrate/60, active_extruder);
    st_synchronize();

    // we have to let the planner know where we are right now as it is not where we said to go.
    zPosition = st_get_position_mm(Z_AXIS);
    plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], zPosition, current_position[E_AXIS]);

}

#ifdef EXTERNAL_ENDSTOP_Z_PROBING
static void run_fast_external_z_endstop() {
    plan_bed_level_matrix.set_to_identity();

    // This function expects the feedrate to have been set externally.
    //feedrate = homing_feedrate[Z_AXIS];

    // move down until you find the bed
    float zPosition = -10;
    plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], zPosition, current_position[E_AXIS], feedrate/60, active_extruder);
    st_synchronize();

    // we have to let the planner know where we are right now as it is not where we said to go.
    zPosition = st_get_position_mm(Z_AXIS);
    plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], zPosition, current_position[E_AXIS]);

}
#endif


static void do_blocking_move_to(float x, float y, float z) {
    float oldFeedRate = feedrate;

    feedrate = XY_TRAVEL_SPEED;

    current_position[X_AXIS] = x;
    current_position[Y_AXIS] = y;
    current_position[Z_AXIS] = z;
    plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], feedrate/60, active_extruder);
    st_synchronize();

    feedrate = oldFeedRate;
}

static void do_blocking_move_relative(float offset_x, float offset_y, float offset_z) {
    do_blocking_move_to(current_position[X_AXIS] + offset_x, current_position[Y_AXIS] + offset_y, current_position[Z_AXIS] + offset_z);
}

static void setup_for_endstop_move() {
    saved_feedrate = feedrate;
    saved_feedmultiply = feedmultiply;
    feedmultiply = 100;
    previous_millis_cmd = millis();

    enable_endstops(true);
}

#ifdef EXTERNAL_ENDSTOP_Z_PROBING
static void setup_for_external_z_endstop_move() {
    saved_feedrate = feedrate;
    saved_feedmultiply = feedmultiply;
    feedmultiply = 100;
    previous_millis_cmd = millis();

    // enabling endstops acts as a safety in case the a z probe was incorrectly engaged or z_min is actually reached
    enable_endstops(true);
    enable_external_z_endstop(true);
}
#endif

static void clean_up_after_endstop_move() {
#ifdef ENDSTOPS_ONLY_FOR_HOMING
    enable_endstops(false);
#endif

#ifdef EXTERNAL_ENDSTOP_Z_PROBING
    enable_external_z_endstop(false);
#endif

    feedrate = saved_feedrate;
    feedmultiply = saved_feedmultiply;
    previous_millis_cmd = millis();
}

static void engage_z_probe() {
    // Engage Z Servo endstop if enabled
    #ifdef SERVO_ENDSTOPS
    if (servo_endstops[Z_AXIS] > -1) {
      SERVO2_ON();
    //_delay_ms(500);
#if defined (ENABLE_AUTO_BED_LEVELING) && (PROBE_SERVO_DEACTIVATION_DELAY > 0)
        servos[servo_endstops[Z_AXIS]].attach(0);
#endif
        servos[servo_endstops[Z_AXIS]].write(servo_endstop_angles[Z_AXIS * 2]);
#if defined (ENABLE_AUTO_BED_LEVELING) && (PROBE_SERVO_DEACTIVATION_DELAY > 0)
        delay(PROBE_SERVO_DEACTIVATION_DELAY);
        servos[servo_endstops[Z_AXIS]].detach();
#endif
    _delay_ms(500);
    SERVO2_OFF();
    }
    #endif
}

static void retract_z_probe() {
    // Retract Z Servo endstop if enabled
    #ifdef SERVO_ENDSTOPS
    if (servo_endstops[Z_AXIS] > -1) {
    SERVO2_ON();
    //_delay_ms(500);
#if defined (ENABLE_AUTO_BED_LEVELING) && (PROBE_SERVO_DEACTIVATION_DELAY > 0)
        servos[servo_endstops[Z_AXIS]].attach(0);
#endif
        servos[servo_endstops[Z_AXIS]].write(servo_endstop_angles[Z_AXIS * 2 + 1]);
#if defined (ENABLE_AUTO_BED_LEVELING) && (PROBE_SERVO_DEACTIVATION_DELAY > 0)
        delay(PROBE_SERVO_DEACTIVATION_DELAY);
        servos[servo_endstops[Z_AXIS]].detach();
#endif
     _delay_ms(500);
     SERVO2_OFF();
    }
    #endif
}

/// Probe bed height at position (x,y), returns the measured z value
static float probe_pt(float x, float y, float z_before, bool engage_z_flag) {
  // move to right place
  do_blocking_move_to(current_position[X_AXIS], current_position[Y_AXIS], z_before);
  //do_blocking_move_to(x - X_PROBE_OFFSET_FROM_EXTRUDER, y - Y_PROBE_OFFSET_FROM_EXTRUDER, current_position[Z_AXIS]);
  do_blocking_move_to(x + X_PROBE_OFFSET_FROM_EXTRUDER, y + Y_PROBE_OFFSET_FROM_EXTRUDER, current_position[Z_AXIS]);

  if(engage_z_flag)
    {
      engage_z_probe();   // Engage Z Servo endstop if available
    }
  run_z_probe();
  float measured_z = current_position[Z_AXIS];
  //retract_z_probe();

  SERIAL_PROTOCOLPGM(MSG_BED);
  SERIAL_PROTOCOLPGM(" x: ");
  SERIAL_PROTOCOL(x);
  SERIAL_PROTOCOLPGM(" y: ");
  SERIAL_PROTOCOL(y);
  SERIAL_PROTOCOLPGM(" z: ");
  SERIAL_PROTOCOL(measured_z);
  SERIAL_PROTOCOLPGM("\n");
  return measured_z;
}

/// Probe bed height at position (x,y), returns the measured z value
static float probe_pt_no_engz(float x, float y, float z_before, bool engage_z_flag) {
  // move to right place
  do_blocking_move_to(current_position[X_AXIS], current_position[Y_AXIS], z_before);
  //do_blocking_move_to(x - X_PROBE_OFFSET_FROM_EXTRUDER, y - Y_PROBE_OFFSET_FROM_EXTRUDER, current_position[Z_AXIS]);
  do_blocking_move_to(x + X_PROBE_OFFSET_FROM_EXTRUDER, y + Y_PROBE_OFFSET_FROM_EXTRUDER, current_position[Z_AXIS]);

  if(engage_z_flag)
    {
      engage_z_probe();   // Engage Z Servo endstop if available
    }

  run_z_probe();
  float measured_z = current_position[Z_AXIS];
  //retract_z_probe();

  SERIAL_PROTOCOLPGM(MSG_BED);
  SERIAL_PROTOCOLPGM(" x: ");
  SERIAL_PROTOCOL(x);
  SERIAL_PROTOCOLPGM(" y: ");
  SERIAL_PROTOCOL(y);
  SERIAL_PROTOCOLPGM(" z: ");
  SERIAL_PROTOCOL(measured_z);
  SERIAL_PROTOCOLPGM("\n");
  return measured_z;
}
#endif // #ifdef ENABLE_AUTO_BED_LEVELING

static void homeaxis(int axis) {
#define HOMEAXIS_DO(LETTER) \
  ((LETTER##_MIN_PIN > -1 && LETTER##_HOME_DIR==-1) || (LETTER##_MAX_PIN > -1 && LETTER##_HOME_DIR==1))

  if (axis==X_AXIS ? HOMEAXIS_DO(X) :
      axis==Y_AXIS ? HOMEAXIS_DO(Y) :
      axis==Z_AXIS ? HOMEAXIS_DO(Z) :
      0) {
    int axis_home_dir = home_dir(axis);

    if(home_Z_reverse && axis==Z_AXIS)
    {axis_home_dir =axis_home_dir *-1;}

    if(x_axis_endstop_sel && axis==X_AXIS)
    {axis_home_dir =axis_home_dir *-1;}

#ifdef DUAL_X_CARRIAGE
    if (axis == X_AXIS)
      axis_home_dir = x_home_dir(active_extruder);
#endif

    current_position[axis] = 0;
    plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);


    // Engage Servo endstop if enabled
    #ifdef SERVO_ENDSTOPS
      #if defined (ENABLE_AUTO_BED_LEVELING) && (PROBE_SERVO_DEACTIVATION_DELAY > 0)
        if (axis==Z_AXIS & z_probe_activation) {
          engage_z_probe();
        }
	    else
      #endif
      if (servo_endstops[axis] > -1) {
        servos[servo_endstops[axis]].write(servo_endstop_angles[axis * 2]);
      }
    #endif

    destination[axis] = 1.5 * max_length(axis) * axis_home_dir;
    feedrate = homing_feedrate[axis];

    if(home_Z_reverse && axis==Z_AXIS)              //speedup G27 (reversed Z homing)
    { // Movement of Z downwards to endstops...
	feedrate = homing_feedrate[axis]*6;
    }

    plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate/60, active_extruder);
    st_synchronize();
    if (!home_Z_reverse && axis==Z_AXIS) set_amb_color_fading(false,true,false,fading_speed);

    current_position[axis] = 0;
    plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
    destination[axis] = -home_retract_mm(axis) * axis_home_dir;
    plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate/60, active_extruder);
    st_synchronize();
    if (!home_Z_reverse && axis==Z_AXIS) {
      set_amb_color_fading(false,true,false,fading_speed);
      retract_z_probe();
      engage_z_probe();
    }

    destination[axis] = 2*home_retract_mm(axis) * axis_home_dir;
#ifdef DELTA
    feedrate = homing_feedrate[axis]/5;
#else
    feedrate = homing_feedrate[axis]/2 ;

    if (!home_Z_reverse && axis==Z_AXIS) {
    feedrate = homing_feedrate[axis]/10 ;
    }

#endif
    plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate/60, active_extruder);
    st_synchronize();
#ifdef DELTA
    // retrace by the amount specified in endstop_adj
    if (endstop_adj[axis] * axis_home_dir < 0) {
      plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
      destination[axis] = endstop_adj[axis];
      plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate/60, active_extruder);
      st_synchronize();
    }
#endif
    axis_is_at_home(axis);
    destination[axis] = current_position[axis];
    feedrate = 0.0;
    endstops_hit_on_purpose();
    axis_known_position[axis] = true;
    if(x_axis_endstop_sel && axis==X_AXIS)
      {
        plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
        enable_endstops(false);
        do_blocking_move_relative(-1,0,0);
        enable_endstops(true);
      }

    // Retract Servo endstop if enabled
    #ifdef SERVO_ENDSTOPS
      if (servo_endstops[axis] > -1) {
        servos[servo_endstops[axis]].write(servo_endstop_angles[axis * 2 + 1]);
      }
    #endif
#if defined (ENABLE_AUTO_BED_LEVELING) && (PROBE_SERVO_DEACTIVATION_DELAY > 0)
    if (axis==Z_AXIS & z_probe_activation){
      clean_up_after_endstop_move();
      plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS]+2, current_position[E_AXIS], 5, active_extruder);
      st_synchronize();
      retract_z_probe();
      //_delay_ms(100);
      plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS]-4, current_position[E_AXIS], 5, active_extruder);
      st_synchronize();
    }
#endif

  }
}
#define HOMEAXIS(LETTER) homeaxis(LETTER##_AXIS)
void refresh_cmd_timeout(void)
{
  previous_millis_cmd = millis();
}

#ifdef FWRETRACT
  void retract(bool retracting) {
    if(retracting && !retracted) {
      destination[X_AXIS]=current_position[X_AXIS];
      destination[Y_AXIS]=current_position[Y_AXIS];
      destination[Z_AXIS]=current_position[Z_AXIS];
      destination[E_AXIS]=current_position[E_AXIS];
      current_position[E_AXIS]+=retract_length/volumetric_multiplier[active_extruder];
      plan_set_e_position(current_position[E_AXIS]);
      float oldFeedrate = feedrate;
      feedrate=retract_feedrate*60;
      retracted=true;
      prepare_move();
      current_position[Z_AXIS]-=retract_zlift;
      plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
      prepare_move();
      feedrate = oldFeedrate;
    } else if(!retracting && retracted) {
      destination[X_AXIS]=current_position[X_AXIS];
      destination[Y_AXIS]=current_position[Y_AXIS];
      destination[Z_AXIS]=current_position[Z_AXIS];
      destination[E_AXIS]=current_position[E_AXIS];
      current_position[Z_AXIS]+=retract_zlift;
      plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
      //prepare_move();
      current_position[E_AXIS]-=(retract_length+retract_recover_length)/volumetric_multiplier[active_extruder];
      plan_set_e_position(current_position[E_AXIS]);
      float oldFeedrate = feedrate;
      feedrate=retract_recover_feedrate*60;
      retracted=false;
      prepare_move();
      feedrate = oldFeedrate;
    }
  } //retract
#endif //FWRETRACT

void process_commands()
{
  unsigned long codenum; //throw away variable
  char *starpos = NULL;
#ifdef ENABLE_AUTO_BED_LEVELING
  float x_tmp, y_tmp, z_tmp, real_z;
#endif
  if(code_seen('G'))
  {
    switch((int)code_value())
    {
    case 0: // G0 -> G1
    case 1: // G1
      //inactivity=false; //re-enable warning for inactivy.
      if(Stopped == false) {
        get_coordinates(); // For X Y Z E F
          #ifdef FWRETRACT
            if(autoretract_enabled)
            if( !(code_seen('X') || code_seen('Y') || code_seen('Z')) && code_seen('E')) {
              float echange=destination[E_AXIS]-current_position[E_AXIS];
              if((echange<-MIN_RETRACT && !retracted) || (echange>MIN_RETRACT && retracted)) { //move appears to be an attempt to retract or recover
                  current_position[E_AXIS] = destination[E_AXIS]; //hide the slicer-generated retract/recover from calculations
                  plan_set_e_position(current_position[E_AXIS]); //AND from the planner
                  retract(!retracted);
                  return;
              }
            }
          #endif //FWRETRACT
        prepare_move();
        //ClearToSend();
        return;
      }
      //break;
    case 2: // G2  - CW ARC
      if(Stopped == false) {
        get_arc_coordinates();
        prepare_arc_move(true);
        return;
      }
    case 3: // G3  - CCW ARC
      if(Stopped == false) {
        get_arc_coordinates();
        prepare_arc_move(false);
        return;
      }
    case 4: // G4 dwell
      LCD_MESSAGEPGM(MSG_DWELL);
      codenum = 0;
      if(code_seen('P')) codenum = code_value(); // milliseconds to wait
      if(code_seen('S')) codenum = code_value() * 1000; // seconds to wait

      st_synchronize();
      codenum += millis();  // keep track of when we started waiting
      previous_millis_cmd = millis();
      while(millis()  < codenum ){
        manage_heater();
        manage_inactivity();
        lcd_update();
      }
      break;
      #ifdef FWRETRACT
      case 10: // G10 retract
        retract(true);
      break;
      case 11: // G11 retract_recover
        retract(false);
      break;
      #endif //FWRETRACT

    case 27: //G27 Home all Axis one at a time explicit without Zprobe
      z_probe_activation=false;
      home_Z_reverse= true;
    case 28: //G28 Home all Axis one at a time
    if(!Stopped){
        retract_z_probe(); //Safety first.
  #ifdef ENABLE_AUTO_BED_LEVELING
        plan_bed_level_matrix.set_to_identity();  //Reset the plane ("erase" all leveling data)
  #endif //ENABLE_AUTO_BED_LEVELING

        store_last_amb_color();
        set_amb_color(0,255,0);

        saved_feedrate = feedrate;
        saved_feedmultiply = feedmultiply;
        feedmultiply = 100;
        previous_millis_cmd = millis();

        enable_endstops(true);
        
        /*if((READ(Z_MAX_PIN)^Z_MAX_ENDSTOP_INVERTING)&&(!home_Z_reverse)){
         //Z movement move to 50 if g27 just happened.
         destination[Z_AXIS] = 20;    // move up
         feedrate = max_feedrate[Z_AXIS];
         plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate, active_extruder);
         st_synchronize();
         }
         */

        for(int8_t i=0; i < NUM_AXIS; i++) {
          destination[i] = current_position[i];
        }
        feedrate = 0.0;

        home_all_axis = !((code_seen(axis_codes[X_AXIS])) || (code_seen(axis_codes[Y_AXIS])) || (code_seen(axis_codes[Z_AXIS])));

        //#if Z_HOME_DIR > 0                      // If homing away from BED do Z first
        if (Z_HOME_DIR > 0 || home_Z_reverse)
        if((home_all_axis) || (code_seen(axis_codes[Z_AXIS]))) {
          HOMEAXIS(Z);
        }
        //#endif
        if(Stopped){break;}

        #ifdef QUICK_HOME
        if((home_all_axis)||( code_seen(axis_codes[X_AXIS]) && code_seen(axis_codes[Y_AXIS])) )  //first diagonal move
        {
          zeroed_far_from_home_x=false;
          zeroed_far_from_home_y=false;
          current_position[X_AXIS] = 0;current_position[Y_AXIS] = 0;

         #ifndef DUAL_X_CARRIAGE
          int x_axis_home_dir = home_dir(X_AXIS);
         #else
          int x_axis_home_dir = x_home_dir(active_extruder);
          extruder_duplication_enabled = false;
         #endif

          plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
          destination[X_AXIS] = 1.5 * max_length(X_AXIS) * x_axis_home_dir;destination[Y_AXIS] = 1.5 * max_length(Y_AXIS) * home_dir(Y_AXIS);
          feedrate = homing_feedrate[X_AXIS];
          if(homing_feedrate[Y_AXIS]<feedrate)
            feedrate = homing_feedrate[Y_AXIS];
          if (max_length(X_AXIS) > max_length(Y_AXIS)) {
            feedrate *= sqrt(pow(max_length(Y_AXIS) / max_length(X_AXIS), 2) + 1);
          } else {
            feedrate *= sqrt(pow(max_length(X_AXIS) / max_length(Y_AXIS), 2) + 1);
          }
          plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate/60, active_extruder);
          st_synchronize();

          axis_is_at_home(X_AXIS);
          axis_is_at_home(Y_AXIS);
          plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
          destination[X_AXIS] = current_position[X_AXIS];
          destination[Y_AXIS] = current_position[Y_AXIS];
          plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate/60, active_extruder);
          feedrate = 0.0;
          st_synchronize();
          endstops_hit_on_purpose();

          current_position[X_AXIS] = destination[X_AXIS];
          current_position[Y_AXIS] = destination[Y_AXIS];
          current_position[Z_AXIS] = destination[Z_AXIS];
        }
        #endif

        if(Stopped){break;}

        if((home_all_axis) || (code_seen(axis_codes[X_AXIS])))
        {
          zeroed_far_from_home_x=false;
        #ifdef DUAL_X_CARRIAGE
          int tmp_extruder = active_extruder;
          extruder_duplication_enabled = false;
          active_extruder = !active_extruder;
          HOMEAXIS(X);
          inactive_extruder_x_pos = current_position[X_AXIS];
          active_extruder = tmp_extruder;
          HOMEAXIS(X);
          // reset state used by the different modes
          memcpy(raised_parked_position, current_position, sizeof(raised_parked_position));
          delayed_move_time = 0;
          active_extruder_parked = true;
        #else
          HOMEAXIS(X);
        #endif
        }

        if(Stopped){break;}

        if((home_all_axis) || (code_seen(axis_codes[Y_AXIS]))) {
          zeroed_far_from_home_y=false;
          HOMEAXIS(Y);
        }

        if(code_seen(axis_codes[X_AXIS]))
        {
          if(code_value_long() != 0) {
            current_position[X_AXIS]=code_value()+add_homeing[0];
          }
        }

        if(code_seen(axis_codes[Y_AXIS])) {
          if(code_value_long() != 0) {
            current_position[Y_AXIS]=code_value()+add_homeing[1];
          }
        }

        if(Stopped){break;}

        if(!home_Z_reverse){
        #if Z_HOME_DIR < 0                      // If homing towards BED do Z last
          #ifndef Z_SAFE_HOMING
            if((home_all_axis) || (code_seen(axis_codes[Z_AXIS]))) {
 
              #if defined (Z_RAISE_BEFORE_HOMING) && (Z_RAISE_BEFORE_HOMING > 0)
                destination[Z_AXIS] = Z_RAISE_BEFORE_HOMING * home_dir(Z_AXIS) * (-1);    // Set destination away from bed
                feedrate = max_feedrate[Z_AXIS];
                plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate, active_extruder);
                st_synchronize();
              #endif
              HOMEAXIS(Z);
            }
          #else                      // Z Safe mode activated.
            if(home_all_axis) {
              enable_endstops(false);
              destination[X_AXIS] = round(Z_SAFE_HOMING_X_POINT);
              destination[Y_AXIS] = round(Z_SAFE_HOMING_Y_POINT);
              destination[Z_AXIS] = Z_RAISE_BEFORE_HOMING * home_dir(Z_AXIS) * (-1);    // Set destination away from bed
              feedrate = XY_TRAVEL_SPEED;
              current_position[Z_AXIS] = 0;

              plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
              plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate, active_extruder);
              st_synchronize();
              current_position[X_AXIS] = destination[X_AXIS];
              current_position[Y_AXIS] = destination[Y_AXIS];
              enable_endstops(true);
              HOMEAXIS(Z);
            }
          }
                    // Let's see if X and Y are homed and probe is inside bed area.
            if(code_seen(axis_codes[Z_AXIS])) {
              if ( (axis_known_position[X_AXIS]) && (axis_known_position[Y_AXIS]) \
                && (current_position[X_AXIS]+X_PROBE_OFFSET_FROM_EXTRUDER >= X_MIN_POS) \
                && (current_position[X_AXIS]+X_PROBE_OFFSET_FROM_EXTRUDER <= X_MAX_POS) \
                && (current_position[Y_AXIS]+Y_PROBE_OFFSET_FROM_EXTRUDER >= Y_MIN_POS) \
                && (current_position[Y_AXIS]+Y_PROBE_OFFSET_FROM_EXTRUDER <= Y_MAX_POS)) {

                current_position[Z_AXIS] = 0;
                plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
                destination[Z_AXIS] = Z_RAISE_BEFORE_HOMING * home_dir(Z_AXIS) * (-1);    // Set destination away from bed
                feedrate = max_feedrate[Z_AXIS];
                plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate, active_extruder);
                st_synchronize();

                HOMEAXIS(Z);
              } else if (!((axis_known_position[X_AXIS]) && (axis_known_position[Y_AXIS]))) {
                  LCD_MESSAGEPGM(MSG_POSITION_UNKNOWN);
                  SERIAL_ECHO_START;
                  SERIAL_ECHOLNPGM(MSG_POSITION_UNKNOWN);
              } else {
                  LCD_MESSAGEPGM(MSG_ZPROBE_OUT);
                  SERIAL_ECHO_START;
                  SERIAL_ECHOLNPGM(MSG_ZPROBE_OUT);
              }
            }
          #endif
        #endif



        if(code_seen(axis_codes[Z_AXIS])) {
          if(code_value_long() != 0) {
            current_position[Z_AXIS]=code_value()+add_homeing[2];
          }
        }
        #ifdef ENABLE_AUTO_BED_LEVELING
          if((home_all_axis) || (code_seen(axis_codes[Z_AXIS]))) {
            current_position[Z_AXIS] += zprobe_zoffset;  //Add Z_Probe offset (the distance is negative)
          }
        #endif
        plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);


        #ifdef ENDSTOPS_ONLY_FOR_HOMING
          enable_endstops(false);
        #endif

        feedrate = saved_feedrate;
        feedmultiply = saved_feedmultiply;
        previous_millis_cmd = millis();
        endstops_hit_on_purpose();
        }
      z_probe_activation=true;
      home_Z_reverse=false;

      restore_last_amb_color();
      
      
       enable_endstops(false);
       //Z movement move to 50 if g27 just happened.
       destination[X_AXIS] = current_position[X_AXIS]+2; 
       destination[Y_AXIS] = current_position[Y_AXIS]+2; 
       destination[Z_AXIS] = current_position[Z_AXIS]; 
       destination[E_AXIS] = current_position[E_AXIS];    
       feedrate = max_feedrate[Z_AXIS];
       plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate, active_extruder);

      current_position[X_AXIS] = destination[X_AXIS];
      current_position[Y_AXIS] = destination[Y_AXIS];
      current_position[Z_AXIS] = destination[Z_AXIS];
      plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);

       st_synchronize();
       enable_endstops(true);
       

      break;

    
#ifdef ENABLE_AUTO_BED_LEVELING
    case 29: // G29 Detailed Z-Probe, probes the bed at 3 or more points.
        {
          if(!Stopped){
            #if Z_MIN_PIN == -1
            #error "You must have a Z_MIN endstop in order to enable Auto Bed Leveling feature!!! Z_MIN_PIN must point to a valid hardware pin."
            #endif

            store_last_amb_color();
            set_amb_color(0,255,0);


            // Prevent user from running a G29 without first homing in X and Y
            if (! (axis_known_position[X_AXIS] && axis_known_position[Y_AXIS]) )
            {
                LCD_MESSAGEPGM(MSG_POSITION_UNKNOWN);
                SERIAL_ECHO_START;
                SERIAL_ECHOLNPGM(MSG_POSITION_UNKNOWN);
                break; // abort G29, since we don't know where we are
            }

            st_synchronize();
            // make sure the bed_level_rotation_matrix is identity or the planner will get it incorectly
            //vector_3 corrected_position = plan_get_position_mm();
            //corrected_position.debug("position before G29");
            plan_bed_level_matrix.set_to_identity();
            vector_3 uncorrected_position = plan_get_position();
            //uncorrected_position.debug("position durring G29");
            current_position[X_AXIS] = uncorrected_position.x;
            current_position[Y_AXIS] = uncorrected_position.y;
            current_position[Z_AXIS] = uncorrected_position.z;
            plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
            setup_for_endstop_move();

            feedrate = homing_feedrate[Z_AXIS];
#ifdef AUTO_BED_LEVELING_GRID
            // probe at the points of a lattice grid

            int xGridSpacing = (RIGHT_PROBE_BED_POSITION - LEFT_PROBE_BED_POSITION) / (AUTO_BED_LEVELING_GRID_POINTS-1);
            int yGridSpacing = (BACK_PROBE_BED_POSITION - FRONT_PROBE_BED_POSITION) / (AUTO_BED_LEVELING_GRID_POINTS-1);


            // solve the plane equation ax + by + d = z
            // A is the matrix with rows [x y 1] for all the probed points
            // B is the vector of the Z positions
            // the normal vector to the plane is formed by the coefficients of the plane equation in the standard form, which is Vx*x+Vy*y+Vz*z+d = 0
            // so Vx = -a Vy = -b Vz = 1 (we want the vector facing towards positive Z

            // "A" matrix of the linear system of equations
            double eqnAMatrix[AUTO_BED_LEVELING_GRID_POINTS*AUTO_BED_LEVELING_GRID_POINTS*3];
            // "B" vector of Z points
            double eqnBVector[AUTO_BED_LEVELING_GRID_POINTS*AUTO_BED_LEVELING_GRID_POINTS];


            int probePointCounter = 0;
            bool zig = true;
            bool engage_z_flag=true;

            for (int yProbe=FRONT_PROBE_BED_POSITION; yProbe <= BACK_PROBE_BED_POSITION; yProbe += yGridSpacing)
            {
              int xProbe, xInc;
              if (zig)
              {
                xProbe = LEFT_PROBE_BED_POSITION;
                //xEnd = RIGHT_PROBE_BED_POSITION;
                xInc = xGridSpacing;
                zig = false;
              } else // zag
              {
                xProbe = RIGHT_PROBE_BED_POSITION;
                //xEnd = LEFT_PROBE_BED_POSITION;
                xInc = -xGridSpacing;
                zig = true;
              }

              for (int xCount=0; xCount < AUTO_BED_LEVELING_GRID_POINTS; xCount++)
              {
                float z_before;
                if (probePointCounter == 0)
                {
                  // raise before probing
                  z_before = Z_RAISE_BEFORE_PROBING;

                } else
                {
                  // raise extruder
                  z_before = current_position[Z_AXIS] + Z_RAISE_BETWEEN_PROBINGS;
                }

                if(Stopped)
                    {
                      break;
                    }
                float measured_z=0;

                for(int avg_measured_z=0; avg_measured_z<AVG_MEASURED_Z_MAX;avg_measured_z++)
                    {
                    measured_z = probe_pt_no_engz(xProbe, yProbe, z_before,engage_z_flag)+measured_z;
                    engage_z_flag=false;
                    }


                eqnBVector[probePointCounter] = measured_z/AVG_MEASURED_Z_MAX;

                eqnAMatrix[probePointCounter + 0*AUTO_BED_LEVELING_GRID_POINTS*AUTO_BED_LEVELING_GRID_POINTS] = xProbe;
                eqnAMatrix[probePointCounter + 1*AUTO_BED_LEVELING_GRID_POINTS*AUTO_BED_LEVELING_GRID_POINTS] = yProbe;
                eqnAMatrix[probePointCounter + 2*AUTO_BED_LEVELING_GRID_POINTS*AUTO_BED_LEVELING_GRID_POINTS] = 1;
                probePointCounter++;
                xProbe += xInc;
              }
            }
            clean_up_after_endstop_move();
            plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS]+2, current_position[E_AXIS], 5, active_extruder);
            st_synchronize();
            retract_z_probe();
            plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], 5, active_extruder);
            st_synchronize();



            // solve lsq problem
            double *plane_equation_coefficients = qr_solve(AUTO_BED_LEVELING_GRID_POINTS*AUTO_BED_LEVELING_GRID_POINTS, 3, eqnAMatrix, eqnBVector);

            SERIAL_PROTOCOLPGM("Eqn coefficients: a: ");
            SERIAL_PROTOCOL_F(plane_equation_coefficients[0],6);
            SERIAL_PROTOCOLPGM(" b: ");
            SERIAL_PROTOCOL_F(plane_equation_coefficients[1],6);
            SERIAL_PROTOCOLPGM(" d: ");
            SERIAL_PROTOCOL_F(plane_equation_coefficients[2],6);
            SERIAL_PROTOCOLLN("");


            set_bed_level_equation_lsq(plane_equation_coefficients);

            free(plane_equation_coefficients);

#else // AUTO_BED_LEVELING_GRID not defined

            // Probe at 3 arbitrary points
            // probe 1
            bool engage_z_flag=true;
            float z_at_pt_1 = probe_pt(ABL_PROBE_PT_1_X, ABL_PROBE_PT_1_Y, Z_RAISE_BEFORE_PROBING,engage_z_flag);
            engage_z_flag=false;
            // probe 2
            float z_at_pt_2 = probe_pt(ABL_PROBE_PT_2_X, ABL_PROBE_PT_2_Y, current_position[Z_AXIS] + Z_RAISE_BETWEEN_PROBINGS,engage_z_flag);

            // probe 3
            float z_at_pt_3 = probe_pt(ABL_PROBE_PT_3_X, ABL_PROBE_PT_3_Y, current_position[Z_AXIS] + Z_RAISE_BETWEEN_PROBINGS,engage_z_flag);

            clean_up_after_endstop_move();
            plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS]+2, current_position[E_AXIS], 5, active_extruder);
            st_synchronize();
            retract_z_probe();
            plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], 5, active_extruder);
            st_synchronize();

            set_bed_level_equation_3pts(z_at_pt_1, z_at_pt_2, z_at_pt_3);


#endif // AUTO_BED_LEVELING_GRID
            st_synchronize();

            // The following code correct the Z height difference from z-probe position and hotend tip position.
            // The Z height on homing is measured by Z-Probe, but the probe is quite far from the hotend.
            // When the bed is uneven, this height must be corrected.
            real_z = float(st_get_position(Z_AXIS))/axis_steps_per_unit[Z_AXIS];  //get the real Z (since the auto bed leveling is already correcting the plane)
            x_tmp = current_position[X_AXIS] + X_PROBE_OFFSET_FROM_EXTRUDER;
            y_tmp = current_position[Y_AXIS] + Y_PROBE_OFFSET_FROM_EXTRUDER;
            z_tmp = current_position[Z_AXIS];

            apply_rotation_xyz(plan_bed_level_matrix, x_tmp, y_tmp, z_tmp);         //Apply the correction sending the probe offset
            current_position[Z_AXIS] = z_tmp - real_z + current_position[Z_AXIS];   //The difference is added to current position and sent to planner.
            plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
          }
        }

        restore_last_amb_color();
        if(Stopped)
                    {
                      plan_bed_level_matrix.set_to_identity();  //Reset the plane ("erase" all leveling data)
                    }
        break;

    case 30: // G30 Single Z Probe
        {
          if(!Stopped){

            //engage_z_probe(); // Engage Z Servo endstop if available


            float feedRateUp = homing_feedrate[Z_AXIS];
            float feedRateDown = homing_feedrate[Z_AXIS];

            if (code_seen('U')) {
                // UP Value Feed Rate
                feedRateUp = code_value();
            }

            if (code_seen('D')) {
              // Down Value (Bed Retract) Feed Rate
              feedRateDown = code_value();
            }



            st_synchronize();
            // TODO: make sure the bed_level_rotation_matrix is identity or the planner will get set incorectly
            setup_for_endstop_move();

            feedrate = feedRateUp; //homing_feedrate[Z_AXIS];

            SERIAL_PROTOCOLPGM(" Feedrate: ");
            SERIAL_PROTOCOL(feedrate);
            SERIAL_PROTOCOLPGM(" ");

            run_fast_z_probe(feedrate);
            SERIAL_PROTOCOLPGM(MSG_BED);
            SERIAL_PROTOCOLPGM(" X: ");
            SERIAL_PROTOCOL(current_position[X_AXIS]);
            SERIAL_PROTOCOLPGM(" Y: ");
            SERIAL_PROTOCOL(current_position[Y_AXIS]);
            SERIAL_PROTOCOLPGM(" Z: ");
            SERIAL_PROTOCOL(current_position[Z_AXIS]);
            SERIAL_PROTOCOLPGM("\n");

            clean_up_after_endstop_move();

            feedrate = homing_feedrate[Z_AXIS];
            //retract_z_probe(); // Retract Z Servo endstop if available
          }
        }
        break;
#endif // ENABLE_AUTO_BED_LEVELING
#ifdef EXTERNAL_ENDSTOP_Z_PROBING
    case 38: // G38 endstop based z probe
	     // Same behaviour as G30 but with an endstop external z probe connected as described in M746
	     // It does nothing unless the probe is enabled first with M746 S1
        {
          if(!Stopped && enable_secure_switch_zprobe){

            st_synchronize();

            setup_for_external_z_endstop_move();

	    if (code_seen('S')) {
	      feedrate = code_value();

	      if(feedrate > 200 ) // ignore the user, greater than 200 for probing is considered to be dangerous (the tool will crash badly)
		feedrate = 200;
	    }
	    else
	      feedrate = homing_feedrate[Z_AXIS];

            run_fast_external_z_endstop();
            SERIAL_PROTOCOLPGM(MSG_BED);
            SERIAL_PROTOCOLPGM(" X: ");
            SERIAL_PROTOCOL(current_position[X_AXIS]);
            SERIAL_PROTOCOLPGM(" Y: ");
            SERIAL_PROTOCOL(current_position[Y_AXIS]);
            SERIAL_PROTOCOLPGM(" Z: ");
            SERIAL_PROTOCOL(current_position[Z_AXIS]);
            SERIAL_PROTOCOLPGM("\n");

            clean_up_after_endstop_move();
          }
        }
        break;
#endif //ENDSTOP_Z_PROBING
    case 90: // G90
      relative_mode = false;
      break;
    case 91: // G91
      relative_mode = true;
      break;
    case 92: // G92
      if(code_seen(axis_codes[X_AXIS]))
        {zeroed_far_from_home_x=true;}
      if(code_seen(axis_codes[Y_AXIS]))
         {zeroed_far_from_home_y=true;}
      if(!code_seen(axis_codes[E_AXIS]))
        st_synchronize();
      for(int8_t i=0; i < NUM_AXIS; i++) {
        if(code_seen(axis_codes[i])) {
           if(i == E_AXIS) {
             current_position[i] = code_value();
             plan_set_e_position(current_position[E_AXIS]);
           }
           else {
             current_position[i] = code_value()+add_homeing[i];
             plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
           }
        }
      }
      break;
    }
  }

  else if(code_seen('M'))
  {
    switch( (int)code_value() )
    {
#ifdef ULTIPANEL
    case 0: // M0 - Unconditional stop - Wait for user button press on LCD
    case 1: // M1 - Conditional stop - Wait for user button press on LCD
    {
      LCD_MESSAGEPGM(MSG_USERWAIT);
      codenum = 0;
      if(code_seen('P')) codenum = code_value(); // milliseconds to wait
      if(code_seen('S')) codenum = code_value() * 1000; // seconds to wait

      st_synchronize();
      previous_millis_cmd = millis();
      if (codenum > 0){
        codenum += millis();  // keep track of when we started waiting
        while(millis()  < codenum && !lcd_clicked()){
          manage_heater();
          manage_inactivity();
          lcd_update();
        }
      }else{
        while(!lcd_clicked()){
          manage_heater();
          manage_inactivity();
          lcd_update();
        }
      }
      LCD_MESSAGEPGM(MSG_RESUMING);
    }
    break;
#endif
    case 17:
        LCD_MESSAGEPGM(MSG_NO_MOVE);
        enable_x();
        enable_y();
        enable_z();
        enable_e0();
        enable_e1();
        enable_e2();
      break;

#ifdef SDSUPPORT
    case 20: // M20 - list SD card
      SERIAL_PROTOCOLLNPGM(MSG_BEGIN_FILE_LIST);
      card.ls();
      SERIAL_PROTOCOLLNPGM(MSG_END_FILE_LIST);
      break;
    case 21: // M21 - init SD card

      card.initsd();

      break;
    case 22: //M22 - release SD card
      card.release();

      break;
    case 23: //M23 - Select file
      starpos = (strchr(strchr_pointer + 4,'*'));
      if(starpos!=NULL)
        *(starpos-1)='\0';
      card.openFile(strchr_pointer + 4,true);
      break;
    case 24: //M24 - Start SD print
      card.startFileprint();
      starttime=millis();
      break;
    case 25: //M25 - Pause SD print
      card.pauseSDPrint();
      break;
    case 26: //M26 - Set SD index
      if(card.cardOK && code_seen('S')) {
        card.setIndex(code_value_long());
      }
      break;
    case 27: //M27 - Get SD status
      card.getStatus();
      break;
    case 28: //M28 - Start SD write
      starpos = (strchr(strchr_pointer + 4,'*'));
      if(starpos != NULL){
        char* npos = strchr(cmdbuffer[bufindr], 'N');
        strchr_pointer = strchr(npos,' ') + 1;
        *(starpos-1) = '\0';
      }
      card.openFile(strchr_pointer+4,false);
      break;
    case 29: //M29 - Stop SD write
      //processed in write to file routine above
      //card,saving = false;
      break;
    case 30: //M30 <filename> Delete File
      if (card.cardOK){
        card.closefile();
        starpos = (strchr(strchr_pointer + 4,'*'));
        if(starpos != NULL){
          char* npos = strchr(cmdbuffer[bufindr], 'N');
          strchr_pointer = strchr(npos,' ') + 1;
          *(starpos-1) = '\0';
        }
        card.removeFile(strchr_pointer + 4);
      }
      break;
    case 32: //M32 - Select file and start SD print
    {
      if(card.sdprinting) {
        st_synchronize();

      }
      starpos = (strchr(strchr_pointer + 4,'*'));

      char* namestartpos = (strchr(strchr_pointer + 4,'!'));   //find ! to indicate filename string start.
      if(namestartpos==NULL)
      {
        namestartpos=strchr_pointer + 4; //default name position, 4 letters after the M
      }
      else
        namestartpos++; //to skip the '!'

      if(starpos!=NULL)
        *(starpos-1)='\0';

      bool call_procedure=(code_seen('P'));

      if(strchr_pointer>namestartpos)
        call_procedure=false;  //false alert, 'P' found within filename

      if( card.cardOK )
      {
        card.openFile(namestartpos,true,!call_procedure);
        if(code_seen('S'))
          if(strchr_pointer<namestartpos) //only if "S" is occuring _before_ the filename
            card.setIndex(code_value_long());
        card.startFileprint();
        if(!call_procedure)
          starttime=millis(); //procedure calls count as normal print time.
      }
    } break;
    case 928: //M928 - Start SD write
      starpos = (strchr(strchr_pointer + 5,'*'));
      if(starpos != NULL){
        char* npos = strchr(cmdbuffer[bufindr], 'N');
        strchr_pointer = strchr(npos,' ') + 1;
        *(starpos-1) = '\0';
      }
      card.openLogFile(strchr_pointer+5);
      break;

#endif //SDSUPPORT

    case 31: //M31 take time since the start of the SD print or an M109 command
      {
      stoptime=millis();
      char time[30];
      unsigned long t=(stoptime-starttime)/1000;
      int sec,min;
      min=t/60;
      sec=t%60;
      sprintf_P(time, PSTR("%i min, %i sec"), min, sec);
      SERIAL_ECHO_START;
      SERIAL_ECHOLN(time);
      lcd_setstatus(time);
      autotempShutdown();
      }
      break;
    case 42: //M42 -Change pin status via gcode
      if (code_seen('S'))
      {
        int pin_status = code_value();
        int pin_number = LED_PIN;
        if (code_seen('P') && pin_status >= 0 && pin_status <= 255)
          pin_number = code_value();
        for(int8_t i = 0; i < (int8_t)sizeof(sensitive_pins); i++)
        {
          if (sensitive_pins[i] == pin_number)
          {
            pin_number = -1;
            break;
          }
        }
      #if defined(FAN_PIN) && FAN_PIN > -1
        if (pin_number == FAN_PIN)
          fanSpeed = pin_status;
      #endif
        if (pin_number > -1)
        {
          pinMode(pin_number, OUTPUT);
          digitalWrite(pin_number, pin_status);
          analogWrite(pin_number, pin_status);
        }
      }
     break;
    case 104: // M104
      if(setTargetedHotend(104)){
        break;
      }
      if(installed_head_id<2)
        #ifdef SELECTABLE_AUTO_FAN_ON_TEMP_CHANGE
          if(auto_fan_on_temp_change)
	    fanSpeed=255; //set fan on by default
        #else
            fanSpeed=255; //set fan on by default on V1 heads
        #endif

      inactivity=false;
      if (code_seen('S')) setTargetHotend(code_value(), tmp_extruder);
#ifdef DUAL_X_CARRIAGE
      if (dual_x_carriage_mode == DXC_DUPLICATION_MODE && tmp_extruder == 0)
        setTargetHotend1(code_value() == 0.0 ? 0.0 : code_value() + duplicate_extruder_temp_offset);
#endif
      setWatch();
      break;
    case 140: // M140 set bed temp
      inactivity=false;
      if (code_seen('S')) setTargetBed(code_value());
      break;
    case 105 : // M105
      if(setTargetedHotend(105)){
        break;
        }
      #if defined(TEMP_0_PIN) && TEMP_0_PIN > -1
        SERIAL_PROTOCOLPGM("ok T:");
        SERIAL_PROTOCOL_F(degHotend(tmp_extruder),1);
        SERIAL_PROTOCOLPGM(" /");
        SERIAL_PROTOCOL_F(degTargetHotend(tmp_extruder),1);
        #if defined(TEMP_BED_PIN) && TEMP_BED_PIN > -1
          SERIAL_PROTOCOLPGM(" B:");
          SERIAL_PROTOCOL_F(degBed(),1);
          SERIAL_PROTOCOLPGM(" /");
          SERIAL_PROTOCOL_F(degTargetBed(),1);
        #endif //TEMP_BED_PIN
        for (int8_t cur_extruder = 0; cur_extruder < EXTRUDERS; ++cur_extruder) {
          SERIAL_PROTOCOLPGM(" T");
          SERIAL_PROTOCOL(cur_extruder);
          SERIAL_PROTOCOLPGM(":");
          SERIAL_PROTOCOL_F(degHotend(cur_extruder),1);
          SERIAL_PROTOCOLPGM(" /");
          SERIAL_PROTOCOL_F(degTargetHotend(cur_extruder),1);
        }
      #else
        SERIAL_ERROR_START;
        SERIAL_ERRORLNPGM(MSG_ERR_NO_THERMISTORS);
      #endif

        SERIAL_PROTOCOLPGM(" @:");
      #ifdef EXTRUDER_WATTS
        SERIAL_PROTOCOL((EXTRUDER_WATTS * getHeaterPower(tmp_extruder))/127);
        SERIAL_PROTOCOLPGM("W");
      #else
        SERIAL_PROTOCOL(getHeaterPower(tmp_extruder));
      #endif

        SERIAL_PROTOCOLPGM(" B@:");
      #ifdef BED_WATTS
        SERIAL_PROTOCOL((BED_WATTS * getHeaterPower(-1))/127);
        SERIAL_PROTOCOLPGM("W");
      #else
        SERIAL_PROTOCOL(getHeaterPower(-1));
      #endif

        /*#ifdef SHOW_TEMP_ADC_VALUES
          #if defined(TEMP_BED_PIN) && TEMP_BED_PIN > -1
            SERIAL_PROTOCOLPGM("    ADC B:");
            SERIAL_PROTOCOL_F(degBed(),1);
            SERIAL_PROTOCOLPGM("C->");
            SERIAL_PROTOCOL_F(rawBedTemp()/OVERSAMPLENR,0);
          #endif
          for (int8_t cur_extruder = 0; cur_extruder < EXTRUDERS; ++cur_extruder) {
            SERIAL_PROTOCOLPGM("  T");
            SERIAL_PROTOCOL(cur_extruder);
            SERIAL_PROTOCOLPGM(":");
            SERIAL_PROTOCOL_F(degHotend(cur_extruder),1);
            SERIAL_PROTOCOLPGM("C->");
            SERIAL_PROTOCOL_F(rawHotendTemp(cur_extruder)/OVERSAMPLENR,0);
          }
        #endif*/

        SERIAL_PROTOCOLLN("");
      return;
      break;
    case 109:
    {// M109 - Wait for extruder heater to reach target.
      if(setTargetedHotend(109)){
        break;
      }
      //LCD_MESSAGEPGM(MSG_HEATING);
        if(installed_head_id<2)
        #ifdef SELECTABLE_AUTO_FAN_ON_TEMP_CHANGE
          if(auto_fan_on_temp_change)
	    fanSpeed=255; //set fan on by default
        #else
            fanSpeed=255; //set fan on by default on V1 heads
        #endif

      inactivity=false;

      #ifdef AUTOTEMP
        autotemp_enabled=false;
      #endif
      if (code_seen('S')) {
        setTargetHotend(code_value(), tmp_extruder);
#ifdef DUAL_X_CARRIAGE
        if (dual_x_carriage_mode == DXC_DUPLICATION_MODE && tmp_extruder == 0)
          setTargetHotend1(code_value() == 0.0 ? 0.0 : code_value() + duplicate_extruder_temp_offset);
#endif
        CooldownNoWait = true;
      } else if (code_seen('R')) {
        setTargetHotend(code_value(), tmp_extruder);
#ifdef DUAL_X_CARRIAGE
        if (dual_x_carriage_mode == DXC_DUPLICATION_MODE && tmp_extruder == 0)
          setTargetHotend1(code_value() == 0.0 ? 0.0 : code_value() + duplicate_extruder_temp_offset);
#endif
        CooldownNoWait = false;
      }
      #ifdef AUTOTEMP
        if (code_seen('S')) autotemp_min=code_value();
        if (code_seen('B')) autotemp_max=code_value();
        if (code_seen('F'))
        {
          autotemp_factor=code_value();
          autotemp_enabled=true;
        }
      #endif

      setWatch();
      codenum = millis();

      /* See if we are heating up or cooling down */
      target_direction = isHeatingHotend(tmp_extruder); // true if heating, false if cooling

      #ifdef TEMP_RESIDENCY_TIME
        long residencyStart;
        residencyStart = -1;
        /* continue to loop until we have reached the target temp
          _and_ until TEMP_RESIDENCY_TIME hasn't passed since we reached it */
        while(((residencyStart == -1) ||
              (residencyStart >= 0 && (((unsigned int) (millis() - residencyStart)) < (TEMP_RESIDENCY_TIME * 1000UL))))&&(!Stopped)) {
      #else
        while ( target_direction ? (isHeatingHotend(tmp_extruder)) : (isCoolingHotend(tmp_extruder)&&(CooldownNoWait==false)) ) {
      #endif //TEMP_RESIDENCY_TIME
          if( (millis() - codenum) > 1000UL )
          { //Print Temp Reading and remaining time every 1 second while heating up/cooling down
            SERIAL_PROTOCOLPGM("T:");
            SERIAL_PROTOCOL_F(degHotend(tmp_extruder),1);
            SERIAL_PROTOCOLPGM(" E:");
            SERIAL_PROTOCOL((int)tmp_extruder);
            #ifdef TEMP_RESIDENCY_TIME
              SERIAL_PROTOCOLPGM(" W:");
              if(residencyStart > -1)
              {
                 codenum = ((TEMP_RESIDENCY_TIME * 1000UL) - (millis() - residencyStart)) / 1000UL;
                 SERIAL_PROTOCOLLN( codenum );
              }
              else
              {
                 SERIAL_PROTOCOLLN( "?" );
              }
            #else
              SERIAL_PROTOCOLLN("");
            #endif
            codenum = millis();
          }
          manage_heater();
          manage_inactivity();
          lcd_update();
        #ifdef TEMP_RESIDENCY_TIME
            /* start/restart the TEMP_RESIDENCY_TIME timer whenever we reach target temp for the first time
              or when current temp falls outside the hysteresis after target temp was reached */
          if ((residencyStart == -1 &&  target_direction && (degHotend(tmp_extruder) >= (degTargetHotend(tmp_extruder)-TEMP_WINDOW))) ||
              (residencyStart == -1 && !target_direction && (degHotend(tmp_extruder) <= (degTargetHotend(tmp_extruder)+TEMP_WINDOW))) ||
              (residencyStart > -1 && labs(degHotend(tmp_extruder) - degTargetHotend(tmp_extruder)) > TEMP_HYSTERESIS) )
          {
            residencyStart = millis();
          }
        #endif //TEMP_RESIDENCY_TIME
        }
	if(!Stopped)
        {LCD_MESSAGEPGM(MSG_HEATING_COMPLETE);}
        starttime=millis();
        previous_millis_cmd = millis();
      }
      break;
    case 190: // M190 - Wait for bed heater to reach target.
    inactivity=false;
    #if defined(TEMP_BED_PIN) && TEMP_BED_PIN > -1
        LCD_MESSAGEPGM(MSG_BED_HEATING);
        if (code_seen('S')) {
          setTargetBed(code_value());
          CooldownNoWait = true;
        } else if (code_seen('R')) {
          setTargetBed(code_value());
          CooldownNoWait = false;
        }
        codenum = millis();

        target_direction = isHeatingBed(); // true if heating, false if cooling

        while (( target_direction ? (isHeatingBed()) : (isCoolingBed()&&(CooldownNoWait==false)) )&&(!Stopped))
        {
          if(( millis() - codenum) > 1000 ) //Print Temp Reading every 1 second while heating up.
          {
            float tt=degHotend(active_extruder);
            SERIAL_PROTOCOLPGM("T:");
            SERIAL_PROTOCOL(tt);
            SERIAL_PROTOCOLPGM(" E:");
            SERIAL_PROTOCOL((int)active_extruder);
            SERIAL_PROTOCOLPGM(" B:");
            SERIAL_PROTOCOL_F(degBed(),1);
            SERIAL_PROTOCOLLN("");
            codenum = millis();
          }
          manage_heater();
          manage_inactivity();
          lcd_update();
        }
        if(!Stopped)
	{LCD_MESSAGEPGM(MSG_BED_DONE);}
        previous_millis_cmd = millis();
    #endif
        break;

    #if defined(FAN_PIN) && FAN_PIN > -1
      case 106: //M106 Fan On
        if (code_seen('S')){
           fanSpeed=constrain(code_value(),0,255);
        }
        else {
          fanSpeed=255;
        }
        break;
      case 107: //M107 Fan Off
        fanSpeed = 0;
        break;
    #endif //FAN_PIN
    #ifdef BARICUDA
      // PWM for HEATER_1_PIN
      #if defined(HEATER_1_PIN) && HEATER_1_PIN > -1
        case 126: //M126 valve open
          if (code_seen('S')){
             ValvePressure=constrain(code_value(),0,255);
          }
          else {
            ValvePressure=255;
          }
          break;
        case 127: //M127 valve closed
          ValvePressure = 0;
          break;
      #endif //HEATER_1_PIN

      // PWM for HEATER_2_PIN
      #if defined(HEATER_2_PIN) && HEATER_2_PIN > -1
        case 128: //M128 valve open
          if (code_seen('S')){
             EtoPPressure=constrain(code_value(),0,255);
          }
          else {
            EtoPPressure=255;
          }
          break;
        case 129: //M129 valve closed
          EtoPPressure = 0;
          break;
      #endif //HEATER_2_PIN
    #endif

    #if defined(PS_ON_PIN) && PS_ON_PIN > -1
      case 80: // M80 - Turn on Power Supply
        SET_OUTPUT(PS_ON_PIN); //GND
        WRITE(PS_ON_PIN, PS_ON_AWAKE);

        // If you have a switch on suicide pin, this is useful
        // if you want to start another print with suicide feature after
        // a print without suicide...
        #if defined SUICIDE_PIN && SUICIDE_PIN > -1
            SET_OUTPUT(SUICIDE_PIN);
            WRITE(SUICIDE_PIN, HIGH);
        #endif

        #ifdef ULTIPANEL
          powersupply = true;
          LCD_MESSAGEPGM(WELCOME_MSG);
          lcd_update();
        #endif
        break;
      #endif

      case 81: // M81 - Turn off Power Supply
        disable_heater();
        st_synchronize();
        disable_e0();
        disable_e1();
        disable_e2();
        finishAndDisableSteppers();
        fanSpeed = 0;
        delay(1000); // Wait a little before to switch off
      #if defined(SUICIDE_PIN) && SUICIDE_PIN > -1
        st_synchronize();
        suicide();
      #elif defined(PS_ON_PIN) && PS_ON_PIN > -1
        SET_OUTPUT(PS_ON_PIN);
        WRITE(PS_ON_PIN, PS_ON_ASLEEP);
      #endif
      #ifdef ULTIPANEL
        powersupply = false;
        LCD_MESSAGEPGM(MACHINE_NAME" "MSG_OFF".");
        lcd_update();
      #endif
	  break;

    case 82:
      axis_relative_modes[3] = false;
      break;
    case 83:
      axis_relative_modes[3] = true;
      break;
    case 18: //compatibility
    case 84: // M84
      if(code_seen('S')){
        max_inactive_time = code_value() * 1000;
      }
      else
      {
        bool all_axis = !((code_seen(axis_codes[X_AXIS])) || (code_seen(axis_codes[Y_AXIS])) || (code_seen(axis_codes[Z_AXIS]))|| (code_seen(axis_codes[E_AXIS])));
        if(all_axis)
        {
          st_synchronize();
          disable_e0();
          disable_e1();
          disable_e2();
          finishAndDisableSteppers();
        }
        else
        {
          st_synchronize();
          if(code_seen('X')) disable_x();
          if(code_seen('Y')) disable_y();
          if(code_seen('Z')) disable_z();
          #if ((E0_ENABLE_PIN != X_ENABLE_PIN) && (E1_ENABLE_PIN != Y_ENABLE_PIN)) // Only enable on boards that have seperate ENABLE_PINS
            if(code_seen('E')) {
              disable_e0();
              disable_e1();
              disable_e2();
            }
          #endif
        }
      }
      break;
    case 85: // M85
      code_seen('S');
      max_inactive_time = code_value() * 1000;
      break;
    case 92: // M92
      for(int8_t i=0; i < NUM_AXIS; i++)
      {
        if(code_seen(axis_codes[i]))
        {
          if(i == 3) { // E
            float value = code_value();
            if(value < 20.0) {
              float factor = axis_steps_per_unit[i] / value; // increase e constants if M92 E14 is given for netfab.
              max_e_jerk *= factor;
              max_feedrate[i] *= factor;
              axis_steps_per_sqr_second[i] *= factor;
            }
            axis_steps_per_unit[i] = value;
          }
          else {
            axis_steps_per_unit[i] = code_value();
          }
        }
      }
      break;
    case 115: // M115
      SERIAL_PROTOCOLPGM(MSG_M115_REPORT);
      break;
    case 117: // M117 display message
      starpos = (strchr(strchr_pointer + 5,'*'));
      if(starpos!=NULL)
        *(starpos-1)='\0';
      lcd_setstatus(strchr_pointer + 5);
      break;
    case 114: // M114
      SERIAL_PROTOCOLPGM("X:");
      SERIAL_PROTOCOL(current_position[X_AXIS]);
      SERIAL_PROTOCOLPGM(" Y:");
      SERIAL_PROTOCOL(current_position[Y_AXIS]);
      SERIAL_PROTOCOLPGM(" Z:");
      SERIAL_PROTOCOL(current_position[Z_AXIS]);
      SERIAL_PROTOCOLPGM(" E:");
      SERIAL_PROTOCOL(current_position[E_AXIS]);

      SERIAL_PROTOCOLPGM(MSG_COUNT_X);
      SERIAL_PROTOCOL(float(st_get_position(X_AXIS))/axis_steps_per_unit[X_AXIS]);
      SERIAL_PROTOCOLPGM(" Y:");
      SERIAL_PROTOCOL(float(st_get_position(Y_AXIS))/axis_steps_per_unit[Y_AXIS]);
      SERIAL_PROTOCOLPGM(" Z:");
      SERIAL_PROTOCOL(float(st_get_position(Z_AXIS))/axis_steps_per_unit[Z_AXIS]);

      SERIAL_PROTOCOLLN("");
      break;
    case 120: // M120
      enable_endstops(false) ;
      break;
    case 121: // M121
      enable_endstops(true) ;
      break;
    case 119: // M119
    SERIAL_PROTOCOLLN(MSG_M119_REPORT);
      #if defined(X_MIN_PIN) && X_MIN_PIN > -1
        SERIAL_PROTOCOLPGM(MSG_X_MIN);
        SERIAL_PROTOCOLLN(((READ(X_MIN_PIN)^X_MIN_ENDSTOP_INVERTING)?MSG_ENDSTOP_HIT:MSG_ENDSTOP_OPEN));
      #endif
      #if defined(X_MAX_PIN) && X_MAX_PIN > -1
        SERIAL_PROTOCOLPGM(MSG_X_MAX);
        SERIAL_PROTOCOLLN(((READ(X_MAX_PIN)^X_MAX_ENDSTOP_INVERTING)?MSG_ENDSTOP_HIT:MSG_ENDSTOP_OPEN));
      #endif
      #if defined(Y_MIN_PIN) && Y_MIN_PIN > -1
        SERIAL_PROTOCOLPGM(MSG_Y_MIN);
        SERIAL_PROTOCOLLN(((READ(Y_MIN_PIN)^Y_MIN_ENDSTOP_INVERTING)?MSG_ENDSTOP_HIT:MSG_ENDSTOP_OPEN));
      #endif
      #if defined(Y_MAX_PIN) && Y_MAX_PIN > -1
        SERIAL_PROTOCOLPGM(MSG_Y_MAX);
        SERIAL_PROTOCOLLN(((READ(Y_MAX_PIN)^Y_MAX_ENDSTOP_INVERTING)?MSG_ENDSTOP_HIT:MSG_ENDSTOP_OPEN));
      #endif
      #if defined(Z_MIN_PIN) && Z_MIN_PIN > -1
        SERIAL_PROTOCOLPGM(MSG_Z_MIN);
        SERIAL_PROTOCOLLN(((READ(Z_MIN_PIN)^Z_MIN_ENDSTOP_INVERTING)?MSG_ENDSTOP_HIT:MSG_ENDSTOP_OPEN));
      #endif
      #if defined(Z_MAX_PIN) && Z_MAX_PIN > -1
        SERIAL_PROTOCOLPGM(MSG_Z_MAX);
        SERIAL_PROTOCOLLN(((READ(Z_MAX_PIN)^Z_MAX_ENDSTOP_INVERTING)?MSG_ENDSTOP_HIT:MSG_ENDSTOP_OPEN));
      #endif
      break;
      //TODO: update for all axis, use for loop
    #ifdef BLINKM
    case 150: // M150
      {
        byte red;
        byte grn;
        byte blu;

        if(code_seen('R')) red = code_value();
        if(code_seen('U')) grn = code_value();
        if(code_seen('B')) blu = code_value();

        SendColors(red,grn,blu);
      }
      break;
    #endif //BLINKM
    case 200: // M200 D<millimeters> set filament diameter and set E axis units to cubic millimeters (use S0 to set back to millimeters).
      {
        float area = .0;
        float radius = .0;
        if(code_seen('D')) {
          radius = (float)code_value() * .5;
          if(radius == 0) {
            area = 1;
          } else {
            area = M_PI * pow(radius, 2);
          }
        } else {
          //reserved for setting filament diameter via UFID or filament measuring device
          break;
        }
        tmp_extruder = active_extruder;
        if(code_seen('T')) {
          tmp_extruder = code_value();
          tmp_extruder = tool_extruder_mapping[tmp_extruder];
          if(tmp_extruder >= EXTRUDERS) {
            SERIAL_ECHO_START;
            SERIAL_ECHO(MSG_M200_INVALID_EXTRUDER);
          }
        }
        volumetric_multiplier[tmp_extruder] = 1 / area;
      }
      break;
    case 201: // M201
      for(int8_t i=0; i < NUM_AXIS; i++)
      {
        if(code_seen(axis_codes[i]))
        {
          max_acceleration_units_per_sq_second[i] = code_value();
        }
      }
      // steps per sq second need to be updated to agree with the units per sq second (as they are what is used in the planner)
      reset_acceleration_rates();
      break;
    #if 0 // Not used for Sprinter/grbl gen6
    case 202: // M202
      for(int8_t i=0; i < NUM_AXIS; i++) {
        if(code_seen(axis_codes[i])) axis_travel_steps_per_sqr_second[i] = code_value() * axis_steps_per_unit[i];
      }
      break;
    #endif
    case 203: // M203 max feedrate mm/sec
      for(int8_t i=0; i < NUM_AXIS; i++) {
        if(code_seen(axis_codes[i])) max_feedrate[i] = code_value();
      }
      break;
    case 204: // M204 acclereration S normal moves T filmanent only moves
      {
        if(code_seen('S')) acceleration = code_value() ;
        if(code_seen('T')) retract_acceleration = code_value() ;
      }
      break;
    case 205: //M205 advanced settings:  minimum travel speed S=while printing T=travel only,  B=minimum segment time X= maximum xy jerk, Z=maximum Z jerk
    {
      if(code_seen('S')) minimumfeedrate = code_value();
      if(code_seen('T')) mintravelfeedrate = code_value();
      if(code_seen('B')) minsegmenttime = code_value() ;
      if(code_seen('X')) max_xy_jerk = code_value() ;
      if(code_seen('Z')) max_z_jerk = code_value() ;
      if(code_seen('E')) max_e_jerk = code_value() ;
    }
    break;
    case 206: // M206 additional homeing offset
      for(int8_t i=0; i < 3; i++)
      {
        if(code_seen(axis_codes[i])) add_homeing[i] = code_value();
      }
      break;
    #ifdef DELTA
	case 665: // M665 set delta configurations L<diagonal_rod> R<delta_radius> S<segments_per_sec>
		if(code_seen('L')) {
			delta_diagonal_rod= code_value();
		}
		if(code_seen('R')) {
			delta_radius= code_value();
		}
		if(code_seen('S')) {
			delta_segments_per_second= code_value();
		}

		recalc_delta_settings(delta_radius, delta_diagonal_rod);
		break;
    case 666: // M666 set delta endstop adjustemnt
      for(int8_t i=0; i < 3; i++)
      {
        if(code_seen(axis_codes[i])) endstop_adj[i] = code_value();
      }
      break;
    #endif
    #ifdef FWRETRACT
    case 207: //M207 - set retract length S[positive mm] F[feedrate mm/min] Z[additional zlift/hop]
    {
      if(code_seen('S'))
      {
        retract_length = code_value() ;
      }
      if(code_seen('F'))
      {
        retract_feedrate = code_value()/60 ;
      }
      if(code_seen('Z'))
      {
        retract_zlift = code_value() ;
      }
    }break;
    case 208: // M208 - set retract recover length S[positive mm surplus to the M207 S*] F[feedrate mm/min]
    {
      if(code_seen('S'))
      {
        retract_recover_length = code_value() ;
      }
      if(code_seen('F'))
      {
        retract_recover_feedrate = code_value()/60 ;
      }
    }break;
    case 209: // M209 - S<1=true/0=false> enable automatic retract detect if the slicer did not support G10/11: every normal extrude-only move will be classified as retract depending on the direction.
    {
      if(code_seen('S'))
      {
        int t= code_value() ;
        switch(t)
        {
          case 0: autoretract_enabled=false;retracted=false;break;
          case 1: autoretract_enabled=true;retracted=false;break;
          default:
            SERIAL_ECHO_START;
            SERIAL_ECHOPGM(MSG_UNKNOWN_COMMAND);
            SERIAL_ECHO(cmdbuffer[bufindr]);
            SERIAL_ECHOLNPGM("\"");
        }
      }

    }break;
    #endif // FWRETRACT
    #if EXTRUDERS > 1
    case 218: // M218 - set hotend offset (in mm), T<extruder_number> X<offset_on_X> Y<offset_on_Y>
    {
      if(setTargetedHotend(218)){
        break;
      }
      if(code_seen('X'))
      {
        extruder_offset[X_AXIS][tmp_extruder] = code_value();
      }
      if(code_seen('Y'))
      {
        extruder_offset[Y_AXIS][tmp_extruder] = code_value();
      }
      #ifdef DUAL_X_CARRIAGE
      if(code_seen('Z'))
      {
        extruder_offset[Z_AXIS][tmp_extruder] = code_value();
      }
      #endif
      SERIAL_ECHO_START;
      SERIAL_ECHOPGM(MSG_HOTEND_OFFSET);
      for(tmp_extruder = 0; tmp_extruder < EXTRUDERS; tmp_extruder++)
      {
         SERIAL_ECHO(" ");
         SERIAL_ECHO(extruder_offset[X_AXIS][tmp_extruder]);
         SERIAL_ECHO(",");
         SERIAL_ECHO(extruder_offset[Y_AXIS][tmp_extruder]);
      #ifdef DUAL_X_CARRIAGE
         SERIAL_ECHO(",");
         SERIAL_ECHO(extruder_offset[Z_AXIS][tmp_extruder]);
      #endif
      }
      SERIAL_ECHOLN("");
    }break;
    #endif
    case 220: // M220 S<factor in percent>- set speed factor override percentage
    {
      if(code_seen('S'))
      {
        feedmultiply = code_value() ;
      }
    }
    break;
    case 221: // M221 S<factor in percent>- set extrude factor override percentage
    {
      if(code_seen('S'))
      {
        int tmp_code = code_value();
        if (code_seen('T'))
        {
          if(setTargetedHotend(221)){
            break;
          }
          extruder_multiply[tmp_extruder] = tmp_code;
        }
        else
        {
          extrudemultiply = tmp_code ;
        }
      }
    }
    break;

	case 226: // M226 P<pin number> S<pin state>- Wait until the specified pin reaches the state required
	{
      if(code_seen('P')){
        int pin_number = code_value(); // pin number
        int pin_state = -1; // required pin state - default is inverted

        if(code_seen('S')) pin_state = code_value(); // required pin state

        if(pin_state >= -1 && pin_state <= 1){

          for(int8_t i = 0; i < (int8_t)sizeof(sensitive_pins); i++)
          {
            if (sensitive_pins[i] == pin_number)
            {
              pin_number = -1;
              break;
            }
          }

          if (pin_number > -1)
          {
            st_synchronize();

            pinMode(pin_number, INPUT);

            int target;
            switch(pin_state){
            case 1:
              target = HIGH;
              break;

            case 0:
              target = LOW;
              break;

            case -1:
              target = !digitalRead(pin_number);
              break;
            }

            while(digitalRead(pin_number) != target){
              manage_heater();
              manage_inactivity();
              lcd_update();
            }
          }
        }
      }
    }
    break;

    #if NUM_SERVOS > 0
    case 280: // M280 - set servo position absolute. P: servo index, S: angle or microseconds
      {
        int servo_index = -1;
        int servo_position = 0;
        if (code_seen('P'))
          servo_index = code_value();
        if (code_seen('S')) {
          servo_position = code_value();
          if ((servo_index >= 0) && (servo_index < NUM_SERVOS)) {
#if defined (ENABLE_AUTO_BED_LEVELING) && (PROBE_SERVO_DEACTIVATION_DELAY > 0)
		      servos[servo_index].attach(0);
#endif
            servos[servo_index].write(servo_position);
#if defined (ENABLE_AUTO_BED_LEVELING) && (PROBE_SERVO_DEACTIVATION_DELAY > 0)
              delay(PROBE_SERVO_DEACTIVATION_DELAY);
              servos[servo_index].detach();
#endif
          }
          else {
            SERIAL_ECHO_START;
            SERIAL_ECHO("Servo ");
            SERIAL_ECHO(servo_index);
            SERIAL_ECHOLN(" out of range");
          }
        }
        else if (servo_index >= 0) {
          SERIAL_PROTOCOL(MSG_OK);
          SERIAL_PROTOCOL(" Servo ");
          SERIAL_PROTOCOL(servo_index);
          SERIAL_PROTOCOL(": ");
          SERIAL_PROTOCOL(servos[servo_index].read());
          SERIAL_PROTOCOLLN("");
        }
      }
      break;
    #endif // NUM_SERVOS > 0

    #if (LARGE_FLASH == true && ( BEEPER > 0 || defined(ULTRALCD) || defined(LCD_USE_I2C_BUZZER)))
    case 300: // M300
    {
      
      if (!silent){
      BEEP_ON()
      _delay_ms(50);
      BEEP_OFF()
      _delay_ms(50);
      BEEP_ON()
      _delay_ms(50);
      BEEP_OFF()
      }

    }
    break;
    #endif // M300

    #ifdef PIDTEMP
    case 301: // M301
      {
        if(code_seen('P')) Kp = code_value();
        if(code_seen('I')) Ki = scalePID_i(code_value());
        if(code_seen('D')) Kd = scalePID_d(code_value());

        #ifdef PID_ADD_EXTRUSION_RATE
        if(code_seen('C')) Kc = code_value();
        #endif

        updatePID();
        SERIAL_PROTOCOL(MSG_OK);
        SERIAL_PROTOCOL(" p:");
        SERIAL_PROTOCOL(Kp);
        SERIAL_PROTOCOL(" i:");
        SERIAL_PROTOCOL(unscalePID_i(Ki));
        SERIAL_PROTOCOL(" d:");
        SERIAL_PROTOCOL(unscalePID_d(Kd));
        #ifdef PID_ADD_EXTRUSION_RATE
        SERIAL_PROTOCOL(" c:");
        //Kc does not have scaling applied above, or in resetting defaults
        SERIAL_PROTOCOL(Kc);
        #endif
        SERIAL_PROTOCOLLN("");
      }
      break;
    #endif //PIDTEMP
    #ifdef PIDTEMPBED
    case 304: // M304
      {
        if(code_seen('P')) bedKp = code_value();
        if(code_seen('I')) bedKi = scalePID_i(code_value());
        if(code_seen('D')) bedKd = scalePID_d(code_value());

        updatePID();
        SERIAL_PROTOCOL(MSG_OK);
        SERIAL_PROTOCOL(" p:");
        SERIAL_PROTOCOL(bedKp);
        SERIAL_PROTOCOL(" i:");
        SERIAL_PROTOCOL(unscalePID_i(bedKi));
        SERIAL_PROTOCOL(" d:");
        SERIAL_PROTOCOL(unscalePID_d(bedKd));
        SERIAL_PROTOCOLLN("");
      }
      break;
    #endif //PIDTEMP
    case 240: // M240  Triggers a camera by emulating a Canon RC-1 : http://www.doc-diy.net/photo/rc-1_hacked/
     {
     	#ifdef CHDK

         SET_OUTPUT(CHDK);
         WRITE(CHDK, HIGH);
         chdkHigh = millis();
         chdkActive = true;

       #else

      	#if defined(PHOTOGRAPH_PIN) && PHOTOGRAPH_PIN > -1
	const uint8_t NUM_PULSES=16;
	const float PULSE_LENGTH=0.01524;
	for(int i=0; i < NUM_PULSES; i++) {
        WRITE(PHOTOGRAPH_PIN, HIGH);
        _delay_ms(PULSE_LENGTH);
        WRITE(PHOTOGRAPH_PIN, LOW);
        _delay_ms(PULSE_LENGTH);
        }
        delay(7.33);
        for(int i=0; i < NUM_PULSES; i++) {
        WRITE(PHOTOGRAPH_PIN, HIGH);
        _delay_ms(PULSE_LENGTH);
        WRITE(PHOTOGRAPH_PIN, LOW);
        _delay_ms(PULSE_LENGTH);
        }
      	#endif
      #endif //chdk end if
     }
    break;
#ifdef DOGLCD
    case 250: // M250  Set LCD contrast value: C<value> (value 0..63)
     {
	  if (code_seen('C')) {
	   lcd_setcontrast( ((int)code_value())&63 );
          }
          SERIAL_PROTOCOLPGM("lcd contrast value: ");
          SERIAL_PROTOCOL(lcd_contrast);
          SERIAL_PROTOCOLLN("");
     }
    break;
#endif
    #ifdef PREVENT_DANGEROUS_EXTRUDE
    case 302: // allow cold extrudes, or set the minimum extrude temperature
    {
	  float temp = .0;
	  if (code_seen('S')) temp=code_value();
      set_extrude_min_temp(temp);
    }
    break;
	#endif
    case 303: // M303 PID autotune
    {
      float temp = 150.0;
      int e=0;
      int c=5;
      if (code_seen('E')) e=code_value();
        if (e<0)
          temp=70;
      if (code_seen('S')) temp=code_value();
      if (code_seen('C')) c=code_value();
      PID_autotune(temp, e, c);
    }
    break;
    case 400: // M400 finish all moves
    {
      st_synchronize();
    }
    break;
#if defined(ENABLE_AUTO_BED_LEVELING) && defined(SERVO_ENDSTOPS)
    case 401:
    {
        engage_z_probe();    // Engage Z Servo endstop if available
    }
    break;

    case 402:
    {
        retract_z_probe();    // Retract Z Servo endstop if enabled
    }
    break;
#endif
    case 500: // M500 Store settings in EEPROM
    {
        Config_StoreSettings();
    }
    break;
    case 501: // M501 Read settings from EEPROM
    {
        Config_RetrieveSettings();
    }
    break;
    case 502: // M502 Revert to default settings
    {
        Config_ResetDefault();
    }
    break;
    case 503: // M503 print settings currently in memory
    {
        Config_PrintSettings();
    }
    break;
    #ifdef ABORT_ON_ENDSTOP_HIT_FEATURE_ENABLED
    case 540:
    {
        if(code_seen('S')) abort_on_endstop_hit = code_value() > 0;
    }
    break;
    #endif


    #ifdef FILAMENTCHANGEENABLE
    case 600: //Pause for filament change X[pos] Y[pos] Z[relative lift] E[initial retract] L[later retract distance for removal]
    {
        float target[4];
        float lastpos[4];
        target[X_AXIS]=current_position[X_AXIS];
        target[Y_AXIS]=current_position[Y_AXIS];
        target[Z_AXIS]=current_position[Z_AXIS];
        target[E_AXIS]=current_position[E_AXIS];
        lastpos[X_AXIS]=current_position[X_AXIS];
        lastpos[Y_AXIS]=current_position[Y_AXIS];
        lastpos[Z_AXIS]=current_position[Z_AXIS];
        lastpos[E_AXIS]=current_position[E_AXIS];
        //retract by E
        if(code_seen('E'))
        {
          target[E_AXIS]+= code_value();
        }
        else
        {
          #ifdef FILAMENTCHANGE_FIRSTRETRACT
            target[E_AXIS]+= FILAMENTCHANGE_FIRSTRETRACT ;
          #endif
        }
        plan_buffer_line(target[X_AXIS], target[Y_AXIS], target[Z_AXIS], target[E_AXIS], feedrate/60, active_extruder);

        //lift Z
        if(code_seen('Z'))
        {
          target[Z_AXIS]+= code_value();
        }
        else
        {
          #ifdef FILAMENTCHANGE_ZADD
            target[Z_AXIS]+= FILAMENTCHANGE_ZADD ;
          #endif
        }
        plan_buffer_line(target[X_AXIS], target[Y_AXIS], target[Z_AXIS], target[E_AXIS], feedrate/60, active_extruder);

        //move xy
        if(code_seen('X'))
        {
          target[X_AXIS]+= code_value();
        }
        else
        {
          #ifdef FILAMENTCHANGE_XPOS
            target[X_AXIS]= FILAMENTCHANGE_XPOS ;
          #endif
        }
        if(code_seen('Y'))
        {
          target[Y_AXIS]= code_value();
        }
        else
        {
          #ifdef FILAMENTCHANGE_YPOS
            target[Y_AXIS]= FILAMENTCHANGE_YPOS ;
          #endif
        }

        plan_buffer_line(target[X_AXIS], target[Y_AXIS], target[Z_AXIS], target[E_AXIS], feedrate/60, active_extruder);

        if(code_seen('L'))
        {
          target[E_AXIS]+= code_value();
        }
        else
        {
          #ifdef FILAMENTCHANGE_FINALRETRACT
            target[E_AXIS]+= FILAMENTCHANGE_FINALRETRACT ;
          #endif
        }

        plan_buffer_line(target[X_AXIS], target[Y_AXIS], target[Z_AXIS], target[E_AXIS], feedrate/60, active_extruder);

        //finish moves
        st_synchronize();
        //disable extruder steppers so filament can be removed
        disable_e0();
        disable_e1();
        disable_e2();
        delay(100);
        LCD_ALERTMESSAGEPGM(MSG_FILAMENTCHANGE);
        uint8_t cnt=0;
        while(!lcd_clicked()){
          cnt++;
          manage_heater();
          manage_inactivity();
          lcd_update();
          if(cnt==0)
          {
          #if BEEPER > 0
            SET_OUTPUT(BEEPER);
            WRITE(BEEPER,HIGH);
            delay(3);
            WRITE(BEEPER,LOW);
            delay(3);
          #else
			#if !defined(LCD_FEEDBACK_FREQUENCY_HZ) || !defined(LCD_FEEDBACK_FREQUENCY_DURATION_MS)
              lcd_buzz(1000/6,100);
			#else
			  lcd_buzz(LCD_FEEDBACK_FREQUENCY_DURATION_MS,LCD_FEEDBACK_FREQUENCY_HZ);
			#endif
          #endif
          }
        }

        //return to normal
        if(code_seen('L'))
        {
          target[E_AXIS]+= -code_value();
        }
        else
        {
          #ifdef FILAMENTCHANGE_FINALRETRACT
            target[E_AXIS]+=(-1)*FILAMENTCHANGE_FINALRETRACT ;
          #endif
        }
        current_position[E_AXIS]=target[E_AXIS]; //the long retract of L is compensated by manual filament feeding
        plan_set_e_position(current_position[E_AXIS]);
        plan_buffer_line(target[X_AXIS], target[Y_AXIS], target[Z_AXIS], target[E_AXIS], feedrate/60, active_extruder); //should do nothing
        plan_buffer_line(lastpos[X_AXIS], lastpos[Y_AXIS], target[Z_AXIS], target[E_AXIS], feedrate/60, active_extruder); //move xy back
        plan_buffer_line(lastpos[X_AXIS], lastpos[Y_AXIS], lastpos[Z_AXIS], target[E_AXIS], feedrate/60, active_extruder); //move z back
        plan_buffer_line(lastpos[X_AXIS], lastpos[Y_AXIS], lastpos[Z_AXIS], lastpos[E_AXIS], feedrate/60, active_extruder); //final untretract
    }
    break;
    #endif //FILAMENTCHANGEENABLE
    #ifdef DUAL_X_CARRIAGE
    case 605: // Set dual x-carriage movement mode:
              //    M605 S0: Full control mode. The slicer has full control over x-carriage movement
              //    M605 S1: Auto-park mode. The inactive head will auto park/unpark without slicer involvement
              //    M605 S2 [Xnnn] [Rmmm]: Duplication mode. The second extruder will duplicate the first with nnn
              //                         millimeters x-offset and an optional differential hotend temperature of
              //                         mmm degrees. E.g., with "M605 S2 X100 R2" the second extruder will duplicate
              //                         the first with a spacing of 100mm in the x direction and 2 degrees hotter.
              //
              //    Note: the X axis should be homed after changing dual x-carriage mode.
    {
        st_synchronize();

        if (code_seen('S'))
          dual_x_carriage_mode = code_value();

        if (dual_x_carriage_mode == DXC_DUPLICATION_MODE)
        {
          if (code_seen('X'))
            duplicate_extruder_x_offset = max(code_value(),X2_MIN_POS - x_home_pos(0));

          if (code_seen('R'))
            duplicate_extruder_temp_offset = code_value();

          SERIAL_ECHO_START;
          SERIAL_ECHOPGM(MSG_HOTEND_OFFSET);
          SERIAL_ECHO(" ");
          SERIAL_ECHO(extruder_offset[X_AXIS][0]);
          SERIAL_ECHO(",");
          SERIAL_ECHO(extruder_offset[Y_AXIS][0]);
          SERIAL_ECHO(" ");
          SERIAL_ECHO(duplicate_extruder_x_offset);
          SERIAL_ECHO(",");
          SERIAL_ECHOLN(extruder_offset[Y_AXIS][1]);
        }
        else if (dual_x_carriage_mode != DXC_FULL_CONTROL_MODE && dual_x_carriage_mode != DXC_AUTO_PARK_MODE)
        {
          dual_x_carriage_mode = DEFAULT_DUAL_X_CARRIAGE_MODE;
        }

        active_extruder_parked = false;
        extruder_duplication_enabled = false;
        delayed_move_time = 0;
    }
    break;
    #endif //DUAL_X_CARRIAGE

    case 907: // M907 Set digital trimpot motor current using axis codes.
    {
      #if defined(DIGIPOTSS_PIN) && DIGIPOTSS_PIN > -1
        for(int i=0;i<NUM_AXIS;i++) if(code_seen(axis_codes[i])) digipot_current(i,code_value());
        if(code_seen('B')) digipot_current(4,code_value());
        if(code_seen('S')) for(int i=0;i<=4;i++) digipot_current(i,code_value());
      #endif
      #ifdef MOTOR_CURRENT_PWM_XY_PIN
        if(code_seen('X')) digipot_current(0, code_value());
      #endif
      #ifdef MOTOR_CURRENT_PWM_Z_PIN
        if(code_seen('Z')) digipot_current(1, code_value());
      #endif
      #ifdef MOTOR_CURRENT_PWM_E_PIN
        if(code_seen('E')) digipot_current(2, code_value());
      #endif
      #ifdef DIGIPOT_I2C
        // this one uses actual amps in floating point
        for(int i=0;i<NUM_AXIS;i++) if(code_seen(axis_codes[i])) digipot_i2c_set_current(i, code_value());
        // for each additional extruder (named B,C,D,E..., channels 4,5,6,7...)
        for(int i=NUM_AXIS;i<DIGIPOT_I2C_NUM_CHANNELS;i++) if(code_seen('B'+i-NUM_AXIS)) digipot_i2c_set_current(i, code_value());
      #endif
    }
    break;
    case 908: // M908 Control digital trimpot directly.
    {
      #if defined(DIGIPOTSS_PIN) && DIGIPOTSS_PIN > -1
        uint8_t channel,current;
        if(code_seen('P')) channel=code_value();
        if(code_seen('S')) current=code_value();
        digitalPotWrite(channel, current);
      #endif
    }
    break;
    case 350: // M350 Set microstepping mode. Warning: Steps per unit remains unchanged. S code sets stepping mode for all drivers.
    {
      #if defined(X_MS1_PIN) && X_MS1_PIN > -1
        if(code_seen('S')) for(int i=0;i<=4;i++) microstep_mode(i,code_value());
        for(int i=0;i<NUM_AXIS;i++) if(code_seen(axis_codes[i])) microstep_mode(i,(uint8_t)code_value());
        if(code_seen('B')) microstep_mode(4,code_value());
        microstep_readings();
      #endif
    }
    break;
    case 351: // M351 Toggle MS1 MS2 pins directly, S# determines MS1 or MS2, X# sets the pin high/low.
    {
      #if defined(X_MS1_PIN) && X_MS1_PIN > -1
      if(code_seen('S')) switch((int)code_value())
      {
        case 1:
          for(int i=0;i<NUM_AXIS;i++) if(code_seen(axis_codes[i])) microstep_ms(i,code_value(),-1);
          if(code_seen('B')) microstep_ms(4,code_value(),-1);
          break;
        case 2:
          for(int i=0;i<NUM_AXIS;i++) if(code_seen(axis_codes[i])) microstep_ms(i,-1,code_value());
          if(code_seen('B')) microstep_ms(4,-1,code_value());
          break;
      }
      microstep_readings();
      #endif
    }
    break;
    case 700: // // M700 S<0-255> - Laser Power Control
    {
      if(code_seen('S'))
      {
        if (code_value()<=0) LaserSoftPwm=0;
        else if (code_value()>=0xFF) LaserSoftPwm=MAX_PWM;
        else {LaserSoftPwm=(unsigned int)(code_value()/2);}

        //write_LaserSoftPwm_t(LaserSoftPwm);
      }
    }
    break;
    case 701: // M701 S<0-255> - Ambient Light, Set Red
    {
      if(red_fading || green_fading || blue_fading)
      {stop_fading();}
      if(code_seen('S'))
      {
        if (code_value()<=0 || code_value()==0) RedSoftPwm=0;
        else if (code_value()>=255) RedSoftPwm=MAX_PWM;
        else {RedSoftPwm=(unsigned int)(code_value()/2);}

        //write_RedSoftPwm_t(RedSoftPwm);
      }
    }
    break;
    case 702: // M702 S<0-255> - Ambient Light, Set Green
    {
      if(red_fading || green_fading || blue_fading)
      {stop_fading();}
      if(code_seen('S'))
      {
        if (code_value()<=0) GreenSoftPwm=0;
        else if (code_value()>=255) GreenSoftPwm=MAX_PWM;
        else {GreenSoftPwm=(unsigned int)(code_value()/2);}

        //write_GreenSoftPwm_t(GreenSoftPwm);
      }
    }
    break;
    case 703: // M703 S<0-255> - Ambient Light, Set Blue
    {
      if(red_fading || green_fading || blue_fading)
      {stop_fading();}
      if(code_seen('S'))
      {
        if (code_value()<=0) BlueSoftPwm=0;
        else if (code_value()>=255) BlueSoftPwm=MAX_PWM;
        else {BlueSoftPwm=(unsigned int)(code_value()/2);}

        //write_BlueSoftPwm_t(BlueSoftPwm);
      }
    }
    break;
    case 704:// M704 - Signalling Light ON (same colors of Ambient Light)
    {
      LIGHT_SIGN_ON();
    }
    break;
    case 705:// M705 - Signalling Light OFF
    {
     LIGHT_SIGN_OFF();
    }
    break;
    case 706:// M706 S<0-255> - Head Light
    {
      if(code_seen('S'))
      {
        if (code_value()<=0) HeadLightSoftPwm=0;
        else if (code_value()>=0xFF) HeadLightSoftPwm=MAX_PWM;
        else {HeadLightSoftPwm=(unsigned int)(code_value()/2);}

        //write_HeadLightSoftPwm_t(HeadLightSoftPwm);
        /*
        String STR_TCCR5A = String(TCCR5A);
        SERIAL_ECHO(STR_TCCR5A);
        SERIAL_ECHO(",");
        */
      }
    }
    break;

    case 710: //M710 write and store in eeprom calibrated z_probe offset length
    {
      float value;
      if (code_seen('S'))
      {
        value = code_value();
        if ((Z_PROBE_OFFSET_RANGE_MIN <= value) && (value <= Z_PROBE_OFFSET_RANGE_MAX))
        {
          zprobe_zoffset = value; // compare w/ line 278 of ConfigurationStore.cpp
          SERIAL_PROTOCOL(MSG_ZPROBE_ZOFFSET " : ");
          SERIAL_PROTOCOLLN(zprobe_zoffset);
          Config_StoreSettings();
        }
        else
        {
          SERIAL_ECHO_START;
          SERIAL_ECHOPGM(MSG_ZPROBE_ZOFFSET);
          SERIAL_ECHOPGM(MSG_Z_MIN);
          SERIAL_ECHO(Z_PROBE_OFFSET_RANGE_MIN);
          SERIAL_ECHOPGM(MSG_Z_MAX);
          SERIAL_ECHO(Z_PROBE_OFFSET_RANGE_MAX);
          SERIAL_PROTOCOLLN("");
        }
      }
      else
      {
          SERIAL_PROTOCOL(MSG_ZPROBE_ZOFFSET " : ");
          SERIAL_PROTOCOLLN(zprobe_zoffset);
      }

      break;
    }

    case 711: // M711 - write and store in eeprom calibrated zprobe extended angle
    {
      int value;
      if (code_seen('S'))
      {
        value = code_value();
        servo_extended_angle = value;
        SERIAL_PROTOCOL("Servo Extended Angle: ");
        SERIAL_PROTOCOLLN(servo_extended_angle);
      }
      else
      {
          servo_extended_angle = servo_extended_angle+1;
          engage_z_probe();
          SERIAL_PROTOCOL("Servo Extended Angle: ");
          SERIAL_PROTOCOLLN(servo_extended_angle);
      }
      servo_endstop_angles[4] = servo_extended_angle;
      Config_StoreSettings();
      break;
    }

    case 712: // M712 - write and store in eeprom calibrated zprobe retacted angle
    {
      int value;
      if (code_seen('S'))
      {
        value = code_value();
        servo_retracted_angle = value;
        SERIAL_PROTOCOL("Servo Retracted Angle: ");
        SERIAL_PROTOCOLLN(servo_retracted_angle);
      }
      else
      {
          servo_retracted_angle = servo_retracted_angle-1;
          retract_z_probe();
          SERIAL_PROTOCOL("Servo Retracted Angle: ");
          SERIAL_PROTOCOLLN(servo_retracted_angle);
      }
      servo_endstop_angles[5] = servo_retracted_angle;
      Config_StoreSettings();
      break;
    }

    case 714: // M714 - alternate the X axis endstop (M714 S0 use standard X axis endstop, M714 S1 use X axis max endstop)
    {
      int value;
      if (code_seen('S'))
      {
        value = code_value();
        if(value==0)
        {
          x_axis_endstop_sel = false;
        }
        else
        {
          x_axis_endstop_sel = true;
        }
      }
      else
      {
          SERIAL_PROTOCOL("X axis now use: ");
          if(x_axis_endstop_sel)
          {
            SERIAL_PROTOCOLLN("Max endstop");
          }
          else
          {
            SERIAL_PROTOCOLLN("Min endstop");
          }
      }
      break;
    }

    case 720:// M720 - 24VDC head power ON
    {
      int servo_index = 0;
      int servo_position = SERVO_SPINDLE_ZERO;
        if ((servo_index >= 0) && (servo_index < NUM_SERVOS)) {
	      servos[servo_index].attach(0);
            servos[servo_index].write(servo_position);
        }
      MILL_MOTOR_ON();
      SERVO1_ON();
    }
    break;
    case 721:// M721 - 24VDC head power OFF
    {
      MILL_MOTOR_OFF();
      SERVO1_OFF();
    }
    break;


    case 722:// M722 - 5VDC SERVO_1 power ON
    {
      SERVO1_ON();
    }
    break;
    case 723:// M723 - 5VDC SERVO_1 power OFF
    {
      SERVO1_OFF();
    }
    break;


    case 724:// M724 - 5VDC SERVO_2 power ON
    {
      SERVO2_ON();
    }
    break;
    case 725:// M725 - 5VDC SERVO_2 power OFF
    {
      SERVO2_OFF();
    }
    break;


    case 726:// M726 - 5VDC RASPBERRY PI power ON
    {
      RASPI_PWR_ON();
    }
    break;
    case 727:// M727 - 5VDC RASPBERRY PI power OFF
    {
      RASPI_PWR_OFF();
    }
    break;

    case 728:// M728 - RASPI ALIVE
    {
      //Stopped = false;
      set_amb_color(255,255,255);
      store_last_amb_color();
      stop_fading();
      restore_last_amb_color();
    
      if (!silent){      
        
        //play intro tune
        analogWrite(BEEPER, 180);
  
        delay(50);      
        BEEP_OFF();
        delay(50);
  
        analogWrite(BEEPER, 185);
  
        delay(50);
        BEEP_OFF();
        delay(50);
  
        analogWrite(BEEPER, 190);
  
        delay(50);
        BEEP_OFF();
        delay(80);
        
        analogWrite(BEEPER, 115);
        
        delay(100);
        BEEP_OFF();
        delay(100);
        
      }
      

    }
    break;

    case 729: // M729 - RASPBERRY SLEEP
    {
      Stopped = true;
      MILL_MOTOR_OFF();
      SERVO1_OFF();                   //disable milling motor
      rpm=0;
      disable_heater();

      disable_x();
      disable_y();
      disable_z();
      disable_e0();
      disable_e1();
      disable_e2();
      
      //outro tune
      if (!silent){
        analogWrite(BEEPER, 115);
  
        delay(500);      
        BEEP_OFF();
        delay(150);
  
        analogWrite(BEEPER, 130);
  
        delay(50);
        BEEP_OFF();
        delay(80);
  
        analogWrite(BEEPER, 150);
  
        delay(50);
        BEEP_OFF();
        delay(60);
        
        analogWrite(BEEPER, 180);
        
        delay(100);
        BEEP_OFF();
        delay(100);
      }
      
      set_amb_color(0,0,0);
      store_last_amb_color();
      set_amb_color_fading(true,true,true,fading_speed);
      //_delay_ms(45000);
      //restore_last_amb_color();
       //while(1){}
    }
    break;

    case 730:   // M730 - READ LAST ERROR CODE
    {
      RPI_ERROR_ACK_OFF();
      SERIAL_PROTOCOL("ERROR ");
      SERIAL_PROTOCOL(": ");
      SERIAL_PROTOCOLLN(ERROR_CODE);
    }
    break;

    case 731:   // M731 - Disable kill on Door Open
    {
      enable_door_kill=false;
      triggered_kill=false;
      Stopped = false;
      FlushSerialRequestResend();
      restore_last_amb_color();
    }
    break;

    case 732:   // M732 - setting of permanent door kill
    {
      int value;
      if (code_seen('S'))
      {
        value = code_value();
        if(value>=1)
        {
          enable_permanent_door_kill=true;
        }
        else
        {
          enable_permanent_door_kill=false;
        }
      }
      else
      {
          SERIAL_PROTOCOL("DOOR KILL ENABLED: ");
          if(enable_permanent_door_kill)
          {
            SERIAL_PROTOCOLLN("TRUE");
          }
          else
          {
            SERIAL_PROTOCOLLN("FALSE");
          }
      }
      triggered_kill=false;
      Stopped = false;
    }
    break;              
    
    case 734: // monitor warning settings (endstops)
    {
      //1 enable
      //0 disable
      int value;
      if (code_seen('S')){
        value = code_value();
        if (value==1)
        {
          monitor_secure_endstop=true; 
          min_x_endstop_triggered=false;
          max_x_endstop_triggered=false;
          min_y_endstop_triggered=false;
          max_y_endstop_triggered=false;
        }
        
        if (value==0){
        monitor_secure_endstop=false; 
        
        }
      }else{
        SERIAL_PROTOCOLLN(monitor_secure_endstop);
        
      }
    }
    break;
    
    case 735:  //M735 S1-0 enable /disable silent mode (sounds except for power-on)
    {
    int value;
      if (code_seen('S'))
      {
        value = code_value();
        if(value==1)
        {
          silent=true; 
        }else{
          silent=false; 
        }
      }
    }
    break;

    case 3: // M3 S[RPM] SPINDLE ON - Clockwise  (MILL MOTOR input: 1060 us equal to Full CCW, 1460us equal to zero, 1860us equal to Full CW)
      {
        inactivity=false;
        int servo_index = 0;
        int servo_position = SERVO_SPINDLE_ZERO;
        float rpm_1=0;
        enable_endstops(false);
               
        
        if(!MILL_MOTOR_STATUS())
            {
              
        //wait
        codenum=1500;       
        st_synchronize();
        codenum += millis();  // keep track of when we started waiting
        previous_millis_cmd = millis();
        while(millis()  < codenum ){
          manage_heater();
          manage_inactivity();
          lcd_update();
        }
              
            if ((servo_index >= 0) && (servo_index < NUM_SERVOS))
              {
	      servos[servo_index].attach(0);
              servos[servo_index].write(servo_position);
              }
            MILL_MOTOR_ON();
            SERVO1_ON();
            fanSpeed=255;
            _delay_ms(2000);
            servos[servo_index].write(SERVO_SPINDLE_ARM);
            _delay_ms(1000);
            servos[servo_index].write(servo_position);
            _delay_ms(500);
           }

        if (code_seen('S')) {
          rpm = code_value();
          if(rpm<=RPM_SPINDLE_MIN)
            {rpm=RPM_SPINDLE_MIN;}
          if(rpm>=RPM_SPINDLE_MAX)
            {rpm=RPM_SPINDLE_MAX;}
            rpm_1=(((float)SERVO_SPINDLE_MAX-(float)SERVO_SPINDLE_ZERO)/(float)RPM_SPINDLE_MAX);
          servo_position = (((int)(rpm_1*rpm))+SERVO_SPINDLE_ZERO);
          //servo_position=(int)(round(0.02857*rpm+SERVO_SPINDLE_ZERO));
          if ((servo_index >= 0) && (servo_index < NUM_SERVOS)) {
	      servos[servo_index].attach(0);
              servos[servo_index].write(servo_position);
              _delay_ms(2000);
              servos[servo_index].detach();
          }
          else {
            SERIAL_ECHO_START;
            SERIAL_ECHO("Servo ");
            SERIAL_ECHO(servo_index);
            SERIAL_ECHOLN(" out of range");
          }
        }
        else if (servo_index >= 0) {
          SERIAL_PROTOCOL(MSG_OK);
          SERIAL_PROTOCOL(" Servo ");
          SERIAL_PROTOCOL(servo_index);
          SERIAL_PROTOCOL(": ");
          SERIAL_PROTOCOL((int)rpm);
          SERIAL_PROTOCOLLN("");
        }
      }
      break;

   case 4: // M4 S[RPM] SPINDLE ON - CounterClockwise  (MILL MOTOR input: 1060 us equal to Full CCW, 1460us equal to zero, 1860us equal to Full CW)
      {
        
        
        //wait
        codenum=1500;       
        st_synchronize();
        codenum += millis();  // keep track of when we started waiting
        previous_millis_cmd = millis();
        while(millis()  < codenum ){
          manage_heater();
          manage_inactivity();
          lcd_update();
        }


        inactivity=false;
        int servo_index = 0;
        int servo_position = SERVO_SPINDLE_ZERO;
        float rpm_1=0;
        enable_endstops(false);

        if(!MILL_MOTOR_STATUS())
            {
            if ((servo_index >= 0) && (servo_index < NUM_SERVOS))
              {
	      servos[servo_index].attach(0);
              servos[servo_index].write(servo_position);
              }
            MILL_MOTOR_ON();
            SERVO1_ON();
            fanSpeed=255;
            _delay_ms(2000);
            servos[servo_index].write(SERVO_SPINDLE_ARM);
            _delay_ms(1000);
            servos[servo_index].write(servo_position);
            _delay_ms(500);
           }

        if (code_seen('S')) {
          rpm = code_value();
          if(rpm<=RPM_SPINDLE_MIN)
            {rpm=RPM_SPINDLE_MIN;}
          if(rpm>=RPM_SPINDLE_MAX)
            {rpm=RPM_SPINDLE_MAX;}
            rpm_1=(((float)SERVO_SPINDLE_ZERO-(float)SERVO_SPINDLE_MIN)/(float)RPM_SPINDLE_MAX);
            servo_position = (SERVO_SPINDLE_ZERO-(int)(rpm_1*rpm));
            //servo_position=(int)(round(SERVO_SPINDLE_ZERO-0.02857*rpm));
          if ((servo_index >= 0) && (servo_index < NUM_SERVOS)) {
	      servos[servo_index].attach(0);
              servos[servo_index].write(servo_position);
              _delay_ms(2000);
              servos[servo_index].detach();
          }
          else {
            SERIAL_ECHO_START;
            SERIAL_ECHO("Servo ");
            SERIAL_ECHO(servo_index);
            SERIAL_ECHOLN(" out of range");
          }
        }
        else if (servo_index >= 0) {
          SERIAL_PROTOCOL(MSG_OK);
          SERIAL_PROTOCOL(" Servo ");
          SERIAL_PROTOCOL(servo_index);
          SERIAL_PROTOCOL(": ");
          SERIAL_PROTOCOL((int)rpm);
          SERIAL_PROTOCOLLN("");
        }
      }
      break;

   case 5: // M5 SPINDLE OFF
      {
        //wait
        codenum=1500;       
        st_synchronize();
        codenum += millis();  // keep track of when we started waiting
        previous_millis_cmd = millis();
        while(millis()  < codenum ){
          manage_heater();
          manage_inactivity();
          lcd_update();
        }
        
        int servo_index = 0;
        int servo_position = SERVO_SPINDLE_ZERO;
        enable_endstops(true);

        if(MILL_MOTOR_STATUS())
            {
            MILL_MOTOR_OFF();
            SERVO1_OFF();
            fanSpeed=0;
           }
         servos[servo_index].detach();
      }
      break;

    case 6: // M6 S[PWM] LASER ON (provisional code)
      {
        inactivity=false;
        int servo_index = 0;
        int servo_position = SERVO_SPINDLE_ZERO;
        float pwm_1=0;
        enable_endstops(false);

        if(!MILL_MOTOR_STATUS())
            {
            if ((servo_index >= 0) && (servo_index < NUM_SERVOS))
              {
			   servos[servo_index].attach(0);
               servos[servo_index].write(servo_position);
              }
		  MILL_MOTOR_ON();
          SERVO1_ON();
          fanSpeed=255;

		  //_delay_ms(2000);
          //servos[servo_index].write(SERVO_SPINDLE_ARM);
          //_delay_ms(1000);
          //servos[servo_index].write(servo_position);
          //_delay_ms(500);

		  }

        if (code_seen('S')) {
          int pwm = code_value();
          if(pwm<=0)
            {pwm=0;}
          if(pwm>=255)
            {pwm=255;}
        }

         SERIAL_PROTOCOL(MSG_OK);
         //SERIAL_PROTOCOL("Laser On!");
      }
      break;

   case 7: // M7 LASER OFF
      {
        int servo_index = 0;
        int servo_position = SERVO_SPINDLE_ZERO;
        enable_endstops(true);

        if(MILL_MOTOR_STATUS())
            {
            MILL_MOTOR_OFF();
            SERVO1_OFF();
            fanSpeed=0;
           }
         servos[servo_index].detach();
      }
      break;

   case 740: // M740 - read WIRE_END sensor
      {
        //SERIAL_PROTOCOLPGM(MSG_WIRE_END);
        SERIAL_PROTOCOLLN((WIRE_END_STATUS()?MSG_ENDSTOP_HIT:MSG_ENDSTOP_OPEN));
      }
      break;

   case 741: // M741 - read DOOR_OPEN sensor
      {
        //SERIAL_PROTOCOLPGM(MSG_DOOR_OPEN);
        SERIAL_PROTOCOLLN((!DOOR_OPEN_STATUS()?MSG_ENDSTOP_HIT:MSG_ENDSTOP_OPEN));
      }
      break;

   case 742: // M742 - read REEL_LENS_OPEN sensor
      {
        //SERIAL_PROTOCOLPGM(MSG_REEL_LENS_OPEN);
        SERIAL_PROTOCOLLN((REEL_LENS_OPEN_STATUS()?MSG_ENDSTOP_HIT:MSG_ENDSTOP_OPEN));
      }
      break;

   case 743: // M743 - read SECURE_SWITCH sensor
      {
        //SERIAL_PROTOCOLPGM(MSG_SECURE_SWITCH);
        SERIAL_PROTOCOLLN((SECURE_SW_STATUS()?MSG_ENDSTOP_HIT:MSG_ENDSTOP_OPEN));
      }
      break;

    case 744: // M744 - read HOT_BED placed in place
      {
        //SERIAL_PROTOCOLPGM(MSG_HOT_BED_PLACED);
        if(degBed()>0)
        {SERIAL_PROTOCOLLN(MSG_ENDSTOP_HIT);}
        else
        {SERIAL_PROTOCOLLN(MSG_ENDSTOP_OPEN);}
      }
      break;

    case 745:  // M745 - read Head placed in place
      {

        if(head_placed)
        {SERIAL_PROTOCOLLN(MSG_ENDSTOP_HIT);}
        else
        {SERIAL_PROTOCOLLN(MSG_ENDSTOP_OPEN);}
      }
      break;

#ifdef EXTERNAL_ENDSTOP_Z_PROBING
    case 746:   // M746 - setting of an endstop sensor, to be used as an external endstop zprobe.
		//
		// The actual pin is provided by EXTERNAL_ENDSTOP_Z_PROBING_PIN macro, which defaults to pin 71
		// (in totumduino's silk screen "Secure_sw", see also M743).
		//
		// M746 S1 to enable this functionality.
		// M746 S0 to disable this functionality.
		// M746 to see the status of this functionality.
		//
		// If enabled you need to keep an external zprobe connected to this totumduino connector that makes the endstop read "open" in normal state or
		// you will have undesirable behaviour (only during the time a Z probe is done, so during homing, G30 and G38).
		//
		// A possible probe is an electrical continuity probe between a copper cad (for making PCBs) and the mill (that in the hybrid head is to GND), like this:
		//
		// |  | (secure_sw)
		// |  \------------------------------+----- attach this to copper clad (via a metal washer to which the wire is soldered?)
		// \-----------------------/\/\/\/---|
		//                         1 kOhm
    {
      int value;
      if (code_seen('S'))
      {
        value = code_value();
        if(value>=1)
        {
          enable_secure_switch_zprobe=true;
        }
        else
        {
          enable_secure_switch_zprobe=false;
        }
      }
      else
      {
          SERIAL_PROTOCOL("SECURE_SW Z PROBE ENABLED: ");
          if(enable_secure_switch_zprobe)
          {
            SERIAL_PROTOCOLLN("TRUE");
          }
          else
          {
            SERIAL_PROTOCOLLN("FALSE");
          }
      }
    }
    break;
#endif

    case 747:   // M747 - Set logic. e.g. M747 X1
    {
      int value;
      if (code_seen('X'))
      {
        value = code_value();
        if(value==1)
        {
          X_MIN_ENDSTOP_INVERTING=true;
          X_MAX_ENDSTOP_INVERTING=true;
        }
	else if(value==2)
	{
          X_MIN_ENDSTOP_INVERTING=true;
          X_MAX_ENDSTOP_INVERTING=false;
	}
	else if(value==3)
	{
          X_MIN_ENDSTOP_INVERTING=false;
          X_MAX_ENDSTOP_INVERTING=true;
	}
        else
        {
          X_MIN_ENDSTOP_INVERTING=false;
	  X_MAX_ENDSTOP_INVERTING=false;
        }
      }

      if (code_seen('Y'))
      {
        value = code_value();
        if(value==1)
        {
          Y_MIN_ENDSTOP_INVERTING=true;
          Y_MAX_ENDSTOP_INVERTING=true;
        }
	else if(value==2)
	{
          Y_MIN_ENDSTOP_INVERTING=true;
          Y_MAX_ENDSTOP_INVERTING=false;
	}
	else if(value==3)
	{
          Y_MIN_ENDSTOP_INVERTING=false;
          Y_MAX_ENDSTOP_INVERTING=true;
	}
        else
        {
	  Y_MIN_ENDSTOP_INVERTING=false;
          Y_MAX_ENDSTOP_INVERTING=false;
        }
      }

      if (code_seen('Z'))
      {
        value = code_value();
        if(value==1)
        {
	  Z_MIN_ENDSTOP_INVERTING=true;
          Z_MAX_ENDSTOP_INVERTING=true;
        }
	else if(value==2)
	{
          Z_MIN_ENDSTOP_INVERTING=true;
          Z_MAX_ENDSTOP_INVERTING=false;
	}
	else if(value==3)
	{
          Z_MIN_ENDSTOP_INVERTING=false;
          Z_MAX_ENDSTOP_INVERTING=true;
	}
        else
        {
	  Z_MIN_ENDSTOP_INVERTING=false;
          Z_MAX_ENDSTOP_INVERTING=false;
        }
      }

    }
    break;

    case 750: // M750 - read PRESSURE sensor (ANALOG 0-1023)
      {
        SERIAL_PROTOCOLPGM("Pressure:");
        SERIAL_PROTOCOL_F(Pressure(),1);
        //SERIAL_PROTOCOL_F(Pressure(),1);
        SERIAL_PROTOCOLLN("");
      }
      break;

    case 751: // M751 - read voltage monitor 24VDC input supply (ANALOG V)
      {
        SERIAL_PROTOCOLPGM("V_24V:");
        SERIAL_PROTOCOL_F(Mon24V(),3);
        SERIAL_PROTOCOLLN(" V");
      }
      break;

    case 752: // M752 - read voltage monitor 5VDC input supply (ANALOG V)
      {
        SERIAL_PROTOCOLPGM("V_5V:");
        SERIAL_PROTOCOL_F(Mon5V(),3);
        SERIAL_PROTOCOLLN(" V");
      }
      break;

    case 753: // M753 - read current monitor input supply (ANALOG A)
      {
        SERIAL_PROTOCOLPGM("Isinked:");
        SERIAL_PROTOCOL_F(MainCurrent(),3);
        SERIAL_PROTOCOLLN(" A");
      }
      break;

    case 754 : // M754
      if(setTargetedHotend(105)){
        break;
        }
          #if defined(TEMP_BED_PIN) && TEMP_BED_PIN > -1
            SERIAL_PROTOCOLPGM("    ADC B:");
            SERIAL_PROTOCOL_F(degBed(),1);
            SERIAL_PROTOCOLPGM("C->");
            SERIAL_PROTOCOL_F(rawBedTemp()/OVERSAMPLENR,0);
          #endif
          for (int8_t cur_extruder = 0; cur_extruder < EXTRUDERS; ++cur_extruder) {
            SERIAL_PROTOCOLPGM("  T");
            SERIAL_PROTOCOL(cur_extruder);
            SERIAL_PROTOCOLPGM(":");
            SERIAL_PROTOCOL_F(degHotend(cur_extruder),1);
            SERIAL_PROTOCOLPGM("C->");
            SERIAL_PROTOCOL_F(rawHotendTemp(cur_extruder)/OVERSAMPLENR,0);
          }

        SERIAL_PROTOCOLLN("");
      return;
      break;


    case 756 :  // M756 - ERROR GENERATOR
      {
       
      ERROR_CODE=0;
      int value;
      if (code_seen('E'))
      {
        value = code_value();
        ERROR_CODE=int(value);
      }
      
      RPI_ERROR_ACK_ON(); 

      }
      break;


    case 760 :  // M760 - read FABtotum Personal Fabricator Main Controller serial ID
      {
        SERIAL_PROTOCOLLN(fab_serial_code);
      }
      break;

    case 761 :  // M761 - read FABtotum Personal Fabricator Main Controller control code of serial ID
      {
        SERIAL_PROTOCOLLN(fab_control_serial_code);
      }
      break;

    case 762 :  // M762 - read FABtotum Personal Fabricator Main Controller board version number
      {
        float fab_board_version_dotted;
        fab_board_version_dotted=fab_board_version;
        fab_board_version_dotted=fab_board_version_dotted/100;
        SERIAL_PROTOCOL("V ");
        SERIAL_PROTOCOL_F(fab_board_version_dotted,2);
        SERIAL_PROTOCOLLN("");
      }
      break;

    case 763 :  // M763 - read FABtotum Personal Fabricator Main Controller production batch number
      {
        if (code_seen('S')) {
          fab_batch_number = code_value_long();
        }
        SERIAL_PROTOCOLLN(fab_batch_number);
      }
      break;

    case 764 :  // M764 - read FABtotum Personal Fabricator Main Controller control code of production batch number
      {
        SERIAL_PROTOCOLLN(fab_control_batch_number);
      }
      break;

    case 765 :  // M765 - read FABtotum Personal Fabricator Firmware Version
      {
         SERIAL_PROTOCOLLN(STRING_BUILD_VERSION);
      }
      break;

    case 766 :  // M766 - read FABtotum Personal Fabricator Firmware Build Date and Time
      {
         SERIAL_PROTOCOLLN(STRING_BUILD_DATE);
      }
      break;

    case 767 :  // M767 - read FABtotum Personal Fabricator Firmware Update Author
      {
         SERIAL_PROTOCOLLN(STRING_CONFIG_H_AUTHOR);
      }
      break;

    case 779 :  // M779 - force head ID reading after reset /for testing purpose only
      {
         Read_Head_Info(true);

        _delay_ms(50);
        BEEP_OFF()
        _delay_ms(30);
        BEEP_ON()
        _delay_ms(50);
        BEEP_OFF()
      }
      break;

    case 782 :  // M782 - read Head product ID
      {
         SERIAL_PROTOCOL("Head Product ID: ");
         String String_Head = String(SERIAL_HEAD_0, HEX);
         SERIAL_PROTOCOLLN(String_Head);
      }
      break;


    case 784 :  // M784 - read Head Serial ID
      {
        String String_Head = String(SERIAL_HEAD_6,HEX);

         SERIAL_PROTOCOL("Head Serial ID: ");
         SERIAL_PROTOCOL(String_Head);

         String_Head = String(SERIAL_HEAD_5,HEX);
         SERIAL_PROTOCOL(String_Head);

         String_Head = String(SERIAL_HEAD_4,HEX);
         SERIAL_PROTOCOL(String_Head);

         String_Head = String(SERIAL_HEAD_3,HEX);
         SERIAL_PROTOCOL(String_Head);

         String_Head = String(SERIAL_HEAD_2,HEX);
         SERIAL_PROTOCOL(String_Head);

         String_Head = String(SERIAL_HEAD_1,HEX);
         SERIAL_PROTOCOL(String_Head);

         String_Head = String(SERIAL_HEAD_7,HEX);

         SERIAL_PROTOCOL(" CRC: ");
         SERIAL_PROTOCOLLN(String_Head);
      }
      break;

    case 785: {
      //PRISM UV MODULE ON/OFF
      if (code_seen('S')) {
          int PRISM_UV=code_value();
          if (PRISM_UV==1){
              PRISM = true;
              SERIAL_PROTOCOL("PRISM UV ON!");
          }
          if (PRISM_UV==0){
              PRISM = false;
              SERIAL_PROTOCOL("PRISM UV OFF!");
          }
      }else{
              SERIAL_PROTOCOL("Usage: M785 S[0-1]");
      }
    }
    break;
    
    
    /*
#define X_STEP_PIN         54
#define X_DIR_PIN          55
#define X_ENABLE_PIN       38

#define Y_STEP_PIN         60
#define Y_DIR_PIN          61
#define Y_ENABLE_PIN       56

#define Z_STEP_PIN         46
#define Z_DIR_PIN          48
#define Z_ENABLE_PIN       62

#define E0_STEP_PIN        26
#define E0_DIR_PIN         28
#define E0_ENABLE_PIN      24

#define E1_STEP_PIN        36
#define E1_DIR_PIN         34
#define E1_ENABLE_PIN      30
    */
        
    case 786: // M786 - external power on/off pin control
      {
        //kill external power supply.
        SERIAL_PROTOCOL("SHUTDOWN!");
        digitalWrite(51, LOW);
      }
      break;

   case 792: {
      //SWITCH XY/AB axis (5th axis interpolation mode) 
      //with this you can pilot A and B axis with X/Y G0 comands in full interpolation (marlin does not allow this natively)
      if (code_seen('S')) {
          int enable=code_value();
          if (enable==1){
             //enable 4/5th axis mode
             /*
             Original E0/E1 values:
              //Steps/unit X72.58 Y72.58 Z2133.33 E177.78
              #define E0_STEP_PIN        26
              #define E0_DIR_PIN         28
              #define E0_ENABLE_PIN      24
              
              #define E1_STEP_PIN        36
              #define E1_DIR_PIN         34
              #define E1_ENABLE_PIN      30
             */
             
            #define X_STEP_PIN         26
            #define X_DIR_PIN          28
            #define X_ENABLE_PIN       24

            #define Y_STEP_PIN         36
            #define Y_DIR_PIN          34
            #define Y_ENABLE_PIN       30
            
            
            axis_steps_per_unit[0]=177.78; 
            axis_steps_per_unit[1]=8.888;
            
          }
              //disable 5th axis
              #define X_STEP_PIN         54
              #define X_DIR_PIN          55
              #define X_ENABLE_PIN       38
              
              #define Y_STEP_PIN         60
              #define Y_DIR_PIN          61
              #define Y_ENABLE_PIN       56
              
              axis_steps_per_unit[0]=72.58;
              axis_steps_per_unit[1]=72.58;

          }else{
              SERIAL_PROTOCOL("Usage: M792 S[0-1]");
      }
    }
    break;
      
    case 793: // M793 - Set/read installed head soft ID
      {
        if (code_seen('S')) {
          installed_head_id = code_value_long();
        }
        SERIAL_PROTOCOLLN(installed_head_id);
      }
      break;

    /*case 793: // M794 P1|P0 - Enable disable
      {
        if (code_seen('S')) {
          installed_head_id = code_value_long();
        }
        SERIAL_PROTOCOLLN(installed_head_id);
      }
      break;*/

    case 563: // M563 [Px] [Dy] - Define tool or swap definitions
      {
         // Select target (logical) tool
         unsigned long target_tool;
        if (code_seen('P')) {
          target_tool = code_value_long();
       } else {
          target_tool = active_tool;

         // Output tool definitions
         SERIAL_ECHOLNPGM("");
         for (target_tool = 0; target_tool < EXTRUDERS; target_tool++)
         {
            if (target_tool == active_tool) {
               SERIAL_ECHOPGM(" * ");
            } else {
              SERIAL_ECHOPGM("   ");
           }
            SERIAL_ECHOPAIR("T", (unsigned long)target_tool);
            SERIAL_ECHOPAIR(": drive:", (unsigned long)tool_extruder_mapping[target_tool]);
            if (tool_twi_support[target_tool]) {
              SERIAL_ECHOPGM(" / twi:on");
           } else {
              SERIAL_ECHOPGM(" / twi:off");
           }
            SERIAL_ECHOLNPGM("");
         }
         return;
       }

       // Assign drive
       uint8_t drive;
        if (code_seen('D')) {
          drive = code_value_long();
           //tool_extruder_mapping[target_tool] = tmp_extruder;
        } else /*if (target_tool != active_tool)*/ {
           drive = active_extruder;
           /*tool_extruder_mapping[active_tool] = tool_extruder_mapping[target_tool];
           tool_extruder_mapping[target_tool] = active_extruder;*/
        }

        bool twi;
        if (code_seen('S')) {
           twi = code_value_long()==1 ? true : false;
        } else {
           twi = !head_is_dummy;
        }

        defineTool(target_tool, drive, 0, twi);

        // Reselect activ tool to reload definition
        loadTool(active_tool);
        //active_extruder = tool_extruder_mapping[active_tool];

      }
      break;

#ifdef THERMISTOR_HOTSWAP
    case 800:   // M800 - changes/reads the thermistor of extruder0 type index
		//
		// M800 S0 changes the extruder0 to the thermistor in FABtotum Head v1
		// M800 S1 changes the extruder0 to Marlin thermistor type 11 (the same as type 60, 100k NTC beta=3950)
		// M800 returns the current extruder0 type index (0=type 169, 1=type 11,...)
    {
      int value;

      if (code_seen('S'))
      {
        value = code_value();
        if(value>=0 && value<THERMISTOR_HOTSWAP_SUPPORTED_TYPES_LEN)
        {
          extruder_0_thermistor_index=value;
	  CRITICAL_SECTION_START
	  heater_ttbl_map[0] = thermistors_map[extruder_0_thermistor_index];
	  heater_ttbllen_map[0] = thermistors_map_len[extruder_0_thermistor_index];
	  CRITICAL_SECTION_END
        }
      }
      else
      {
	SERIAL_PROTOCOLLN_F(extruder_0_thermistor_index,DEC);
      }
    }
    break;
    case 801:   // M801 - changes/reads the current extruder0 max temp
		//
		// M801 S260 changes the extruder0 max temp to 260 Celsius
		// M801 returns the current max temp of extruder0
    {
      int value;
      if (code_seen('S'))
      {
        value = code_value();
        if(value>=0)
        {
	  CRITICAL_SECTION_START
	  maxttemp[0] = value;
	  CRITICAL_SECTION_END
        }
      }
      else
      {
          SERIAL_PROTOCOLLN(maxttemp[0]);
      }
    }
    break;
    case 802:   // M802 - returns supported thermistor types by index
    {
      int value;
          SERIAL_PROTOCOLLN(THERMISTOR_HOTSWAP_SUPPORTED_TYPES_AS_STRING);
    }
    break;

#endif

#ifdef THERMISTOR_INPUT_HOTSWAP
    case 803:   // M803 - changes/reads the current extruder0 thermistor input
		//
		// M803 S1 changes input of current extruder0 to thermistor1
		// M803 S0 changes input of current extruder0 to thermistor0
		// M803 returns the current input of extruder0
    {
      int value;

      if (code_seen('S'))
      {
        value = code_value();
        if(value == 0 || value == 1)
        {
          extruder_0_thermistor_input_index=value;
        }
      }
      else
      {
	SERIAL_PROTOCOLLN_F(extruder_0_thermistor_input_index,DEC);
      }
    }
    break;
#endif

#ifdef SELECTABLE_AUTO_FAN_ON_TEMP_CHANGE
    case 804:   // M804 - changes/reads the current automatic fan on temp change configuration.
		//
		// M804 S1 enables this automatic fan (default is enabled)
		// M804 S0 disables this automatic fan
		// M804 returns the current setting 1/0 (enabled/disabled)
    {
      int value;

      if (code_seen('S'))
      {
        value = code_value();

        auto_fan_on_temp_change=(value==0?false:true);
      }
      else
      {
	SERIAL_PROTOCOLLN_F((auto_fan_on_temp_change==false?0:1),DEC);
      }
    }
    break;
#endif



   case 998: // M998: Restart after being killed
      {
      triggered_kill=false;
      Stopped = false;
      FlushSerialRequestResend();
      restore_last_amb_color();
      RPI_ERROR_ACK_OFF();
      }
    break;

    case 999: // M999: Restart after being stopped
      {
      Stopped = false;
      lcd_reset_alert_level();
      gcode_LastN = Stopped_gcode_LastN;
      FlushSerialRequestResend();
      restore_last_amb_color();
      RPI_ERROR_ACK_OFF();
      }
    break;
    }
  }

  else if(code_seen('T'))
  {
    tmp_extruder = code_value();
    tmp_extruder = loadTool(tmp_extruder);
    if(tmp_extruder >= EXTRUDERS) {
      SERIAL_ECHO_START;
      SERIAL_ECHO("T");
      SERIAL_ECHO(tmp_extruder);
      SERIAL_ECHOLN(MSG_INVALID_EXTRUDER);
    }
    else {
      boolean make_move = false;
      if(code_seen('F')) {
        make_move = true;
        next_feedrate = code_value();
        if(next_feedrate > 0.0) {
          feedrate = next_feedrate;
        }
      }
      #if EXTRUDERS > 1
      if(tmp_extruder != active_extruder) {
        // Save current position to return to after applying extruder offset
        memcpy(destination, current_position, sizeof(destination));
      #ifdef DUAL_X_CARRIAGE
        if (dual_x_carriage_mode == DXC_AUTO_PARK_MODE && Stopped == false &&
            (delayed_move_time != 0 || current_position[X_AXIS] != x_home_pos(active_extruder)))
        {
          // Park old head: 1) raise 2) move to park position 3) lower
          plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS] + TOOLCHANGE_PARK_ZLIFT,
                current_position[E_AXIS], max_feedrate[Z_AXIS], active_extruder);
          plan_buffer_line(x_home_pos(active_extruder), current_position[Y_AXIS], current_position[Z_AXIS] + TOOLCHANGE_PARK_ZLIFT,
                current_position[E_AXIS], max_feedrate[X_AXIS], active_extruder);
          plan_buffer_line(x_home_pos(active_extruder), current_position[Y_AXIS], current_position[Z_AXIS],
                current_position[E_AXIS], max_feedrate[Z_AXIS], active_extruder);
          st_synchronize();
        }

        // apply Y & Z extruder offset (x offset is already used in determining home pos)
        current_position[Y_AXIS] = current_position[Y_AXIS] -
                     extruder_offset[Y_AXIS][active_extruder] +
                     extruder_offset[Y_AXIS][tmp_extruder];
        current_position[Z_AXIS] = current_position[Z_AXIS] -
                     extruder_offset[Z_AXIS][active_extruder] +
                     extruder_offset[Z_AXIS][tmp_extruder];

        active_extruder = tmp_extruder;

        // This function resets the max/min values - the current position may be overwritten below.
        axis_is_at_home(X_AXIS);

        if (dual_x_carriage_mode == DXC_FULL_CONTROL_MODE)
        {
          current_position[X_AXIS] = inactive_extruder_x_pos;
          inactive_extruder_x_pos = destination[X_AXIS];
        }
        else if (dual_x_carriage_mode == DXC_DUPLICATION_MODE)
        {
          active_extruder_parked = (active_extruder == 0); // this triggers the second extruder to move into the duplication position
          if (active_extruder == 0 || active_extruder_parked)
            current_position[X_AXIS] = inactive_extruder_x_pos;
          else
            current_position[X_AXIS] = destination[X_AXIS] + duplicate_extruder_x_offset;
          inactive_extruder_x_pos = destination[X_AXIS];
          extruder_duplication_enabled = false;
        }
        else
        {
          // record raised toolhead position for use by unpark
          memcpy(raised_parked_position, current_position, sizeof(raised_parked_position));
          raised_parked_position[Z_AXIS] += TOOLCHANGE_UNPARK_ZLIFT;
          active_extruder_parked = true;
          delayed_move_time = 0;
        }
      #else
        // Offset extruder (only by XY)
        int i;
        for(i = 0; i < 2; i++) {
           current_position[i] = current_position[i] -
                                 extruder_offset[i][active_extruder] +
                                 extruder_offset[i][tmp_extruder];
        }
        // Set the new active extruder and position
        active_extruder = tmp_extruder;
      #endif //else DUAL_X_CARRIAGE
#ifdef DELTA

  calculate_delta(current_position); // change cartesian kinematic  to  delta kinematic;
   //sent position to plan_set_position();
  plan_set_position(delta[X_AXIS], delta[Y_AXIS], delta[Z_AXIS],current_position[E_AXIS]);

#else
        plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);

#endif
        // Move to the old position if 'F' was in the parameters
        if(make_move && Stopped == false) {
           prepare_move();
        }
      }
      #endif
      SERIAL_ECHO_START;
      SERIAL_ECHO(MSG_ACTIVE_EXTRUDER);
      SERIAL_PROTOCOLLN((int)active_extruder);
    }
  }

  else
  {
    SERIAL_ECHO_START;
    SERIAL_ECHOPGM(MSG_UNKNOWN_COMMAND);
    SERIAL_ECHO(cmdbuffer[bufindr]);
    SERIAL_ECHOLNPGM("\"");
  }

  ClearToSend();
}

void FlushSerialRequestResend()
{
  //char cmdbuffer[bufindr][100]="Resend:";
  MYSERIAL.flush();
  SERIAL_PROTOCOLPGM(MSG_RESEND);
  SERIAL_PROTOCOLLN(gcode_LastN + 1);
  ClearToSend();
}

void ClearToSend()
{
  previous_millis_cmd = millis();
  #ifdef SDSUPPORT
  if(fromsd[bufindr])
    return;
  #endif //SDSUPPORT
  SERIAL_PROTOCOLLNPGM(MSG_OK);
}

void get_coordinates()
{
  bool seen[4]={false,false,false,false};
  for(int8_t i=0; i < NUM_AXIS; i++) {
    if(code_seen(axis_codes[i]))
    {
      destination[i] = (float)code_value() + (axis_relative_modes[i] || relative_mode)*current_position[i];
      seen[i]=true;
    }
    else destination[i] = current_position[i]; //Are these else lines really needed?
  }
  if(code_seen('F')) {
    next_feedrate = code_value();
    if(next_feedrate > 0.0) feedrate = next_feedrate;
  }
}

void get_arc_coordinates()
{
#ifdef SF_ARC_FIX
   bool relative_mode_backup = relative_mode;
   relative_mode = true;
#endif
   get_coordinates();
#ifdef SF_ARC_FIX
   relative_mode=relative_mode_backup;
#endif

   if(code_seen('I')) {
     offset[0] = code_value();
   }
   else {
     offset[0] = 0.0;
   }
   if(code_seen('J')) {
     offset[1] = code_value();
   }
   else {
     offset[1] = 0.0;
   }
}

void clamp_to_software_endstops(float target[3])
{
   if (min_software_endstops) {
    if (target[X_AXIS] < min_pos[X_AXIS]) target[X_AXIS] = min_pos[X_AXIS];
    if (target[Y_AXIS] < min_pos[Y_AXIS]) target[Y_AXIS] = min_pos[Y_AXIS];
    if (target[Z_AXIS] < min_pos[Z_AXIS]) target[Z_AXIS] = min_pos[Z_AXIS];
  }

  if (max_software_endstops) {
    if (target[X_AXIS] > max_pos[X_AXIS]) target[X_AXIS] = max_pos[X_AXIS];
    if (target[Y_AXIS] > max_pos[Y_AXIS]) target[Y_AXIS] = max_pos[Y_AXIS];
    if (target[Z_AXIS] > max_pos[Z_AXIS]) target[Z_AXIS] = max_pos[Z_AXIS];
  }
}

#ifdef DELTA
void recalc_delta_settings(float radius, float diagonal_rod)
{
	 delta_tower1_x= -SIN_60*radius; // front left tower
	 delta_tower1_y= -COS_60*radius;
	 delta_tower2_x=  SIN_60*radius; // front right tower
	 delta_tower2_y= -COS_60*radius;
	 delta_tower3_x= 0.0;                  // back middle tower
	 delta_tower3_y= radius;
	 delta_diagonal_rod_2= sq(diagonal_rod);
}

void calculate_delta(float cartesian[3])
{
  delta[X_AXIS] = sqrt(delta_diagonal_rod_2
                       - sq(delta_tower1_x-cartesian[X_AXIS])
                       - sq(delta_tower1_y-cartesian[Y_AXIS])
                       ) + cartesian[Z_AXIS];
  delta[Y_AXIS] = sqrt(delta_diagonal_rod_2
                       - sq(delta_tower2_x-cartesian[X_AXIS])
                       - sq(delta_tower2_y-cartesian[Y_AXIS])
                       ) + cartesian[Z_AXIS];
  delta[Z_AXIS] = sqrt(delta_diagonal_rod_2
                       - sq(delta_tower3_x-cartesian[X_AXIS])
                       - sq(delta_tower3_y-cartesian[Y_AXIS])
                       ) + cartesian[Z_AXIS];
  /*
  SERIAL_ECHOPGM("cartesian x="); SERIAL_ECHO(cartesian[X_AXIS]);
  SERIAL_ECHOPGM(" y="); SERIAL_ECHO(cartesian[Y_AXIS]);
  SERIAL_ECHOPGM(" z="); SERIAL_ECHOLN(cartesian[Z_AXIS]);

  SERIAL_ECHOPGM("delta x="); SERIAL_ECHO(delta[X_AXIS]);
  SERIAL_ECHOPGM(" y="); SERIAL_ECHO(delta[Y_AXIS]);
  SERIAL_ECHOPGM(" z="); SERIAL_ECHOLN(delta[Z_AXIS]);
  */
}
#endif

void prepare_move()
{
  clamp_to_software_endstops(destination);

  previous_millis_cmd = millis();
#ifdef DELTA
  float difference[NUM_AXIS];
  for (int8_t i=0; i < NUM_AXIS; i++) {
    difference[i] = destination[i] - current_position[i];
  }
  float cartesian_mm = sqrt(sq(difference[X_AXIS]) +
                            sq(difference[Y_AXIS]) +
                            sq(difference[Z_AXIS]));
  if (cartesian_mm < 0.000001) { cartesian_mm = abs(difference[E_AXIS]); }
  if (cartesian_mm < 0.000001) { return; }
  float seconds = 6000 * cartesian_mm / feedrate / feedmultiply;
  int steps = max(1, int(delta_segments_per_second * seconds));
  // SERIAL_ECHOPGM("mm="); SERIAL_ECHO(cartesian_mm);
  // SERIAL_ECHOPGM(" seconds="); SERIAL_ECHO(seconds);
  // SERIAL_ECHOPGM(" steps="); SERIAL_ECHOLN(steps);
  for (int s = 1; s <= steps; s++) {
    float fraction = float(s) / float(steps);
    for(int8_t i=0; i < NUM_AXIS; i++) {
      destination[i] = current_position[i] + difference[i] * fraction;
    }
    calculate_delta(destination);
    plan_buffer_line(delta[X_AXIS], delta[Y_AXIS], delta[Z_AXIS],
                     destination[E_AXIS], feedrate*feedmultiply/60/100.0,
                     active_extruder);
  }
#else

#ifdef DUAL_X_CARRIAGE
  if (active_extruder_parked)
  {
    if (dual_x_carriage_mode == DXC_DUPLICATION_MODE && active_extruder == 0)
    {
      // move duplicate extruder into correct duplication position.
      plan_set_position(inactive_extruder_x_pos, current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
      plan_buffer_line(current_position[X_AXIS] + duplicate_extruder_x_offset, current_position[Y_AXIS], current_position[Z_AXIS],
          current_position[E_AXIS], max_feedrate[X_AXIS], 1);
      plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
      st_synchronize();
      extruder_duplication_enabled = true;
      active_extruder_parked = false;
    }
    else if (dual_x_carriage_mode == DXC_AUTO_PARK_MODE) // handle unparking of head
    {
      if (current_position[E_AXIS] == destination[E_AXIS])
      {
        // this is a travel move - skit it but keep track of current position (so that it can later
        // be used as start of first non-travel move)
        if (delayed_move_time != 0xFFFFFFFFUL)
        {
          memcpy(current_position, destination, sizeof(current_position));
          if (destination[Z_AXIS] > raised_parked_position[Z_AXIS])
            raised_parked_position[Z_AXIS] = destination[Z_AXIS];
          delayed_move_time = millis();
          return;
        }
      }
      delayed_move_time = 0;
      // unpark extruder: 1) raise, 2) move into starting XY position, 3) lower
      plan_buffer_line(raised_parked_position[X_AXIS], raised_parked_position[Y_AXIS], raised_parked_position[Z_AXIS],    current_position[E_AXIS], max_feedrate[Z_AXIS], active_extruder);
      plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], raised_parked_position[Z_AXIS],
          current_position[E_AXIS], min(max_feedrate[X_AXIS],max_feedrate[Y_AXIS]), active_extruder);
      plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS],
          current_position[E_AXIS], max_feedrate[Z_AXIS], active_extruder);
      active_extruder_parked = false;
    }
  }
#endif //DUAL_X_CARRIAGE

  // Do not use feedmultiply for E or Z only moves
  if( (current_position[X_AXIS] == destination [X_AXIS]) && (current_position[Y_AXIS] == destination [Y_AXIS])) {
      plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate/60, active_extruder);
  }
  else {
    plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate*feedmultiply/60/100.0, active_extruder);
  }
#endif //else DELTA
  for(int8_t i=0; i < NUM_AXIS; i++) {
    current_position[i] = destination[i];
  }
}

void prepare_arc_move(char isclockwise) {
  float r = hypot(offset[X_AXIS], offset[Y_AXIS]); // Compute arc radius for mc_arc

  // Trace the arc
  mc_arc(current_position, destination, offset, X_AXIS, Y_AXIS, Z_AXIS, feedrate*feedmultiply/60/100.0, r, isclockwise, active_extruder);

  // As far as the parser is concerned, the position is now == target. In reality the
  // motion control system might still be processing the action and the real tool position
  // in any intermediate location.
  for(int8_t i=0; i < NUM_AXIS; i++) {
    current_position[i] = destination[i];
  }
  previous_millis_cmd = millis();
}

#if defined(CONTROLLERFAN_PIN) && CONTROLLERFAN_PIN > -1

#if defined(FAN_PIN)
  #if CONTROLLERFAN_PIN == FAN_PIN
    #error "You cannot set CONTROLLERFAN_PIN equal to FAN_PIN"
  #endif
#endif

unsigned long lastMotor = 0; //Save the time for when a motor was turned on last
unsigned long lastMotorCheck = 0;

void controllerFan()
{
  if ((millis() - lastMotorCheck) >= 2500) //Not a time critical function, so we only check every 2500ms
  {
    lastMotorCheck = millis();

    if(!READ(X_ENABLE_PIN) || !READ(Y_ENABLE_PIN) || !READ(Z_ENABLE_PIN) || (soft_pwm_bed > 0)
    #if EXTRUDERS > 2
       || !READ(E2_ENABLE_PIN)
    #endif
    #if EXTRUDER > 1
      #if defined(X2_ENABLE_PIN) && X2_ENABLE_PIN > -1
       || !READ(X2_ENABLE_PIN)
      #endif
       || !READ(E1_ENABLE_PIN)
    #endif
       || !READ(E0_ENABLE_PIN)) //If any of the drivers are enabled...
    {
      lastMotor = millis(); //... set time to NOW so the fan will turn on
    }

    if ((millis() - lastMotor) >= (CONTROLLERFAN_SECS*1000UL) || lastMotor == 0) //If the last time any driver was enabled, is longer since than CONTROLLERSEC...
    {
        digitalWrite(CONTROLLERFAN_PIN, 0);
        analogWrite(CONTROLLERFAN_PIN, 0);
    }
    else
    {
        // allows digital or PWM fan output to be used (see M42 handling)
        digitalWrite(CONTROLLERFAN_PIN, CONTROLLERFAN_SPEED);
        analogWrite(CONTROLLERFAN_PIN, CONTROLLERFAN_SPEED);
    }
  }
}
#endif

#ifdef TEMP_STAT_LEDS
static bool blue_led = false;
static uint32_t stat_update = 0;

void handle_status_leds(void) {
  float max_temp = 0.0;
  if(millis() > stat_update) {
     stat_update += 500; // Update every 0.5s
      for (int8_t cur_extruder = 0; cur_extruder < EXTRUDERS; ++cur_extruder) {
         //max_temp = max(max_temp, degHotend(cur_extruder));            //<------------------
         //max_temp = max(max_temp, degTargetHotend(cur_extruder));      //<-------------------
      }
      #if defined(TEMP_BED_PIN) && TEMP_BED_PIN > -1
        max_temp = max(max_temp, degTargetBed());
        max_temp = max(max_temp, degBed());
      #endif
      if((max_temp > 55.0) && (red_led == false)) {
        digitalWrite(STAT_LED_RED, 0);
        //digitalWrite(STAT_LED_BLUE, 0);
        red_led = true;
        //blue_led = false;
      }
      //if((max_temp < 54.0) && (blue_led == false))
      if((max_temp < 54.0)) {
        digitalWrite(STAT_LED_RED, 1);
        //digitalWrite(STAT_LED_BLUE, 1);
        red_led = false;
        //blue_led = true;
      }
  }
}
#endif

void manage_inactivity()
{

  if (max_steppers_inactive_time){
    if( (millis() - previous_millis_cmd) >  max_steppers_inactive_time){
        if(blocks_queued() == false) {
            //steppers disabled
            disable_x();
            disable_y();
            disable_z();
            disable_e0();
            disable_e1();
            disable_e2();
        }
    }
  }

  if(max_inactive_time)  {
    if( (millis() - previous_millis_cmd) >  max_inactive_time )
    {
      if(blocks_queued() == false) {
        if (inactivity == false){

         //steppers disabled
          disable_x();
          disable_y();
          disable_z();
          disable_e0();
          disable_e1();
          disable_e2();

          // disable heating & motors
          disable_heater();
          MILL_MOTOR_OFF();
          SERVO1_OFF();
          rpm=0;

          // warning
          RPI_ERROR_ACK_ON();
          ERROR_CODE=ERROR_IDLE_SAFETY;
          inactivity=true;
        }
      }
    }
  }

  #ifdef CHDK //Check if pin should be set to LOW after M240 set it to HIGH
    if (chdkActive)
    {
      chdkActive = false;
      if (millis()-chdkHigh < CHDK_DELAY) return;
      WRITE(CHDK, LOW);
    }
  #endif

  #if defined(KILL_PIN) && KILL_PIN > -1
    if( 0 == READ(KILL_PIN) )
      kill();
  #endif
  #if defined(CONTROLLERFAN_PIN) && CONTROLLERFAN_PIN > -1
    /controllerFan(); //Check if fan should be turned on to cool stepper drivers down
  #endif
  #ifdef EXTRUDER_RUNOUT_PREVENT
    if( (millis() - previous_millis_cmd) >  EXTRUDER_RUNOUT_SECONDS*1000 )
    if(degHotend(active_extruder)>EXTRUDER_RUNOUT_MINTEMP)
    {
     bool oldstatus=READ(E0_ENABLE_PIN);
     enable_e0();
     float oldepos=current_position[E_AXIS];
     float oldedes=destination[E_AXIS];
     plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS],
                      destination[E_AXIS]+EXTRUDER_RUNOUT_EXTRUDE*EXTRUDER_RUNOUT_ESTEPS/axis_steps_per_unit[E_AXIS],
                      EXTRUDER_RUNOUT_SPEED/60.*EXTRUDER_RUNOUT_ESTEPS/axis_steps_per_unit[E_AXIS], active_extruder);
     current_position[E_AXIS]=oldepos;
     destination[E_AXIS]=oldedes;
     plan_set_e_position(oldepos);
     previous_millis_cmd=millis();
     st_synchronize();
     WRITE(E0_ENABLE_PIN,oldstatus);
    }
  #endif
  #if defined(DUAL_X_CARRIAGE)
    // handle delayed move timeout
    if (delayed_move_time != 0 && (millis() - delayed_move_time) > 1000 && Stopped == false)
    {
      // travel moves have been received so enact them
      delayed_move_time = 0xFFFFFFFFUL; // force moves to be done
      memcpy(destination,current_position,sizeof(destination));
      prepare_move();
    }
  #endif
  #ifdef TEMP_STAT_LEDS
      if (PRISM==false){
      handle_status_leds();
      }else{
     digitalWrite(STAT_LED_RED, 0);
      }
  #endif
  check_axes_activity();

 if (((READ(DOOR_OPEN_PIN) && (!READ(X_ENABLE_PIN) || !READ(Y_ENABLE_PIN) || !READ(Z_ENABLE_PIN) || !READ(E0_ENABLE_PIN) || (READ(MILL_MOTOR_ON_PIN) && rpm>0))) && enable_door_kill) && enable_permanent_door_kill)
    {
     kill_by_door();                    // if the FABtotum is working and the user opens the front door the FABtotum will be disabled   
    }

 //if ((READ(X_MAX_PIN)^X_MAX_ENDSTOP_INVERTING) && (READ(X_MIN_PIN)^X_MIN_ENDSTOP_INVERTING))
 //   {
 //   rpi_recovery_flag=true;
 //   RPI_RECOVERY_ON();          //check if user is going to recover Raspberry OS
 //   stop_fading();
 //   set_amb_color(0,0,255);
 //    }
 //else
 //   {
 //      if(rpi_recovery_flag)
 //      {
 //      RPI_RECOVERY_OFF();
 //      rpi_recovery_flag=false;
 //      }
 //    }

 if ((READ(Y_MAX_PIN)^Y_MAX_ENDSTOP_INVERTING) && (READ(Y_MIN_PIN)^Y_MIN_ENDSTOP_INVERTING))
    { // the user pressed both Y-Endstops at the same time (or a hardware switch that enables them at the same time)
      RPI_ERROR_ACK_ON();
      ERROR_CODE=ERROR_Y_BOTH_TRIGGERED;
    }

 if ((READ(Z_MAX_PIN)^Z_MAX_ENDSTOP_INVERTING) && (READ(Z_MIN_PIN)^Z_MIN_ENDSTOP_INVERTING))
    { // the user pressed both Z-Endstops at the same time (or a hardware switch that enables them at the same time)
      RPI_ERROR_ACK_ON();
      ERROR_CODE=ERROR_Z_BOTH_TRIGGERED;
    }


 if (!READ(DOOR_OPEN_PIN) && !enable_door_kill)
    {
     enable_door_kill=true;                    // REARM the killing process if the user closes the front door
    }


 manage_secure_endstop();
 manage_fab_soft_pwm();                        // manage light
 manage_amb_color_fading();                    // manage ligth fading

}


void manage_secure_endstop()
{
  if(monitor_secure_endstop){
    
    if((READ(X_MIN_PIN)^X_MIN_ENDSTOP_INVERTING)&& !READ(X_ENABLE_PIN) && !min_x_endstop_triggered && !(RPI_ERROR_STATUS()))
    {
           
      min_x_endstop_triggered=true; //trigger
      max_x_endstop_triggered=false;
      min_y_endstop_triggered=false;
      max_y_endstop_triggered=false;
      
      RPI_ERROR_ACK_ON();
      ERROR_CODE=ERROR_X_MIN_ENDSTOP;
    }
        
    if((READ(X_MAX_PIN)^X_MAX_ENDSTOP_INVERTING)&&!READ(X_ENABLE_PIN) && !max_x_endstop_triggered  && !(RPI_ERROR_STATUS()))
    {
      //&& !x_axis_endstop_sel has been temporarily disabled.
            
      min_x_endstop_triggered=false; 
      max_x_endstop_triggered=true;  //trigger
      min_y_endstop_triggered=false;
      max_y_endstop_triggered=false;
      RPI_ERROR_ACK_ON();
      ERROR_CODE=ERROR_X_MAX_ENDSTOP;
    }
  
    if((READ(Y_MIN_PIN)^Y_MIN_ENDSTOP_INVERTING)&&!READ(Y_ENABLE_PIN) && !min_y_endstop_triggered && !(RPI_ERROR_STATUS()))
    {
      
      min_x_endstop_triggered=false; 
      max_x_endstop_triggered=false; 
      min_y_endstop_triggered=true; //trigger
      max_y_endstop_triggered=false;
      
      RPI_ERROR_ACK_ON();
      ERROR_CODE=ERROR_Y_MAX_ENDSTOP;
    }
    
    if((READ(Y_MAX_PIN)^Y_MAX_ENDSTOP_INVERTING)&& !READ(Y_ENABLE_PIN) && !max_y_endstop_triggered && !(RPI_ERROR_STATUS()))      
    {
      
      min_x_endstop_triggered=false; 
      max_x_endstop_triggered=false; 
      min_y_endstop_triggered=false;
      max_y_endstop_triggered=true;//trigger
      
      RPI_ERROR_ACK_ON();
      ERROR_CODE=ERROR_Y_MIN_ENDSTOP;
    }
  }


}
void manage_fab_soft_pwm()
{
  FabSoftPwm_TMR=FabSoftPwm_TMR+1;
  if(FabSoftPwm_TMR>LaserSoftPwm && LaserSoftPwm<MAX_PWM) LASER_GATE_OFF();
  if(FabSoftPwm_TMR>HeadLightSoftPwm && HeadLightSoftPwm<MAX_PWM) HEAD_LIGHT_OFF();
  if(FabSoftPwm_TMR>RedSoftPwm && RedSoftPwm<MAX_PWM) RED_OFF();
  if(FabSoftPwm_TMR>GreenSoftPwm && GreenSoftPwm<MAX_PWM) GREEN_OFF();
  if(FabSoftPwm_TMR>BlueSoftPwm && BlueSoftPwm<MAX_PWM) BLUE_OFF();
  if(FabSoftPwm_TMR>FabSoftPwm_LMT)
    {
      if(LaserSoftPwm>0)LASER_GATE_ON();
      if(HeadLightSoftPwm>0)HEAD_LIGHT_ON();
      if(RedSoftPwm>0)RED_ON();
      if(GreenSoftPwm>0)GREEN_ON();
      if(BlueSoftPwm>0)BLUE_ON();
      FabSoftPwm_TMR=0;
    }

}
void set_amb_color(unsigned int red,unsigned int green,unsigned int blue)
{
      RedSoftPwm=red;
      GreenSoftPwm=green;
      BlueSoftPwm=blue;

      /*write_RedSoftPwm_t(RedSoftPwm);
      write_GreenSoftPwm_t(GreenSoftPwm);
      write_BlueSoftPwm_t(BlueSoftPwm);*/
}

void set_amb_color_fading(bool red_bool,bool green_bool,bool blue_bool,unsigned int fading_speed_in)
{
      set_amb_color(0,0,0);

      red_fading=red_bool;
      green_fading=green_bool;
      blue_fading=blue_bool;

      /*write_red_fading(red_bool);
      write_green_fading(green_bool);
      write_blue_fading(blue_bool);*/

      fading_speed=fading_speed_in;
      //write_fading_speed(fading_speed_in);
}

void store_last_amb_color()
{
      RedSoftPwm_old=RedSoftPwm;
      GreenSoftPwm_old=GreenSoftPwm;
      BlueSoftPwm_old=BlueSoftPwm;
}

void restore_last_amb_color()
{
      RedSoftPwm=RedSoftPwm_old;
      GreenSoftPwm=GreenSoftPwm_old;
      BlueSoftPwm=BlueSoftPwm_old;

      /*write_RedSoftPwm_t(RedSoftPwm);
      write_GreenSoftPwm_t(GreenSoftPwm);
      write_BlueSoftPwm_t(BlueSoftPwm);*/
}

void stop_fading()
{
      set_amb_color_fading(false,false,false,fading_speed);
}

void manage_amb_color_fading()
{
  if(red_fading || green_fading || blue_fading)
  {
    fading_started=true;
  if(led_update_cycles>fading_speed)
      {
      if(slope)
        {
          if(red_fading) {RedSoftPwm=RedSoftPwm+1;}
          if(green_fading){GreenSoftPwm=GreenSoftPwm+1;}
          if(blue_fading){BlueSoftPwm=BlueSoftPwm+1;}
        }
      else
        {
          if(red_fading){RedSoftPwm=RedSoftPwm-1;}
          if(green_fading){GreenSoftPwm=GreenSoftPwm-1;}
          if(blue_fading){BlueSoftPwm=BlueSoftPwm-1;}
        }
      if(((RedSoftPwm==MAX_PWM || RedSoftPwm==0) && red_fading) || ((GreenSoftPwm==MAX_PWM || GreenSoftPwm==0) && green_fading) || ((BlueSoftPwm==MAX_PWM || BlueSoftPwm==0) && blue_fading))
        {slope=!slope;}

      led_update_cycles=0;
      }
   led_update_cycles=led_update_cycles+1;
  }
  if(!red_fading && !green_fading && !blue_fading && fading_started)
    {
      led_update_cycles=0;
      fading_started=false;
      slope=true;
    }
}


void Read_Head_Info(bool force=false)
{
   // Dummy heads can't be read...
   if (head_is_dummy)
   {
      // ...unless you force it
      if (force) {
         Wire.begin();
      } else {
         return;
      }
   }

   SERIAL_HEAD_0=I2C_read(SERIAL_N_FAM_DEV_CODE);
   SERIAL_HEAD_1=I2C_read(SERIAL_N_0);
   SERIAL_HEAD_2=I2C_read(SERIAL_N_1);
   SERIAL_HEAD_3=I2C_read(SERIAL_N_2);
   SERIAL_HEAD_4=I2C_read(SERIAL_N_3);
   SERIAL_HEAD_5=I2C_read(SERIAL_N_4);
   SERIAL_HEAD_6=I2C_read(SERIAL_N_5);
   SERIAL_HEAD_7=I2C_read(SERIAL_N_CRC);

   i2c_timeout=false;

   if(SERIAL_HEAD_0==63 && SERIAL_HEAD_1==63 && SERIAL_HEAD_2==63 && SERIAL_HEAD_3==63 && SERIAL_HEAD_4==63 && SERIAL_HEAD_5==63 && SERIAL_HEAD_6==63 && SERIAL_HEAD_7==63)
   {
      head_placed=false;
   }
   else
   {
      head_placed=true;
   }
}

char I2C_read(byte i2c_register)
{
  char byte_read;
  Wire.beginTransmission(SERIAL_ID_ADDR);      //starts communication
  Wire.write(i2c_register);                         //Sends the register we wish to read
  Wire.endTransmission();

  Wire.requestFrom(SERIAL_ID_ADDR, 1);    // request 1 bytes from slave device SERIAL_ID_ADDR
  i2c_pre_millis=millis();
  while(Wire.available()==0 && !(i2c_timeout))
    {
         if((millis()-i2c_pre_millis)>I2C_MAX_TIMEOUT)
           {
             i2c_timeout=true;
           }
    }   // slave may send less than requested

    if(i2c_timeout)
      {
        return('?');      // return a byte as character
      }
    else
      {
        byte_read=Wire.read();
        return(byte_read);      // return a byte as character
      }


}

void kill_by_door()
{ 

  store_last_amb_color();
  MILL_MOTOR_OFF();
  SERVO1_OFF();                   //disable milling motor
  rpm=0;

  //cli(); // Stop interrupts
  disable_heater();

  disable_x();
  disable_y();
  disable_z();
  disable_e0();
  disable_e1();
  disable_e2();

  set_amb_color(MAX_PWM,0,0);

  triggered_kill=true;
  Stopped = true;

#if defined(PS_ON_PIN) && PS_ON_PIN > -1
  pinMode(PS_ON_PIN,INPUT);
#endif
  SERIAL_ERROR_START;
  SERIAL_ERRORLNPGM(MSG_ERR_KILLED);
  LCD_ALERTMESSAGEPGM(MSG_KILLED);
  suicide();
  //while(1) { /* Intentionally left empty */ } // Wait for reset

  RPI_ERROR_ACK_ON();
  ERROR_CODE=ERROR_DOOR_OPEN;
  
  /*
  if (!silent){
  BEEP_ON();
  delay(500);
  BEEP_OFF();
  delay(100);
  BEEP_ON();
  delay(500);
  BEEP_OFF();
  delay(100);
  BEEP_ON();
  delay(1500);
  BEEP_OFF();
  }
  */
}

void kill()
{
  store_last_amb_color();
  MILL_MOTOR_OFF();
  SERVO1_OFF();                   //disable milling motor
  rpm=0;

  //cli(); // Stop interrupts
  disable_heater();

  disable_x();
  disable_y();
  disable_z();
  disable_e0();
  disable_e1();
  disable_e2();

  set_amb_color(MAX_PWM,0,0);

  triggered_kill=true;
  Stopped = true;
  quickStop();

#if defined(PS_ON_PIN) && PS_ON_PIN > -1
  pinMode(PS_ON_PIN,INPUT);
#endif

  SERIAL_ERROR_START;
  SERIAL_ERRORLNPGM(MSG_ERR_KILLED);
  LCD_ALERTMESSAGEPGM(MSG_KILLED);
  suicide();
  //while(1) { /* Intentionally left empty */ } // Wait for reset

  RPI_ERROR_ACK_ON();
  ERROR_CODE=ERROR_KILLED;
  
}

void Stop()
{
  store_last_amb_color();
  disable_heater();
  set_amb_color(MAX_PWM,MAX_PWM,0);
  if(Stopped == false) {
    Stopped = true;
    Stopped_gcode_LastN = gcode_LastN; // Save last g_code for restart
    SERIAL_ERROR_START;
    SERIAL_ERRORLNPGM(MSG_ERR_STOPPED);
    LCD_MESSAGEPGM(MSG_STOPPED);
  }

  RPI_ERROR_ACK_ON();
  ERROR_CODE=ERROR_STOPPED;
}

bool IsStopped() { return Stopped; };

#ifdef FAST_PWM_FAN
void setPwmFrequency(uint8_t pin, int val)
{
  val &= 0x07;
  switch(digitalPinToTimer(pin))
  {

    #if defined(TCCR0A)
    case TIMER0A:
    case TIMER0B:
//         TCCR0B &= ~(_BV(CS00) | _BV(CS01) | _BV(CS02));
//         TCCR0B |= val;
         break;
    #endif

    #if defined(TCCR1A)
    case TIMER1A:
    case TIMER1B:
//         TCCR1B &= ~(_BV(CS10) | _BV(CS11) | _BV(CS12));
//         TCCR1B |= val;
         break;
    #endif

    #if defined(TCCR2)
    case TIMER2:
    case TIMER2:
         TCCR2 &= ~(_BV(CS10) | _BV(CS11) | _BV(CS12));
         TCCR2 |= val;
         break;
    #endif

    #if defined(TCCR2A)
    case TIMER2A:
    case TIMER2B:
         TCCR2B &= ~(_BV(CS20) | _BV(CS21) | _BV(CS22));
         TCCR2B |= val;
         break;
    #endif

    #if defined(TCCR3A)
    case TIMER3A:
    case TIMER3B:
    case TIMER3C:
         TCCR3B &= ~(_BV(CS30) | _BV(CS31) | _BV(CS32));
         TCCR3B |= val;
         break;
    #endif

    #if defined(TCCR4A)
    case TIMER4A:
    case TIMER4B:
    case TIMER4C:
         TCCR4B &= ~(_BV(CS40) | _BV(CS41) | _BV(CS42));
         TCCR4B |= val;
         break;
   #endif

    #if defined(TCCR5A)
    case TIMER5A:
    case TIMER5B:
    case TIMER5C:
         TCCR5B &= ~(_BV(CS50) | _BV(CS51) | _BV(CS52));
         TCCR5B |= val;
         break;
   #endif

  }
}
#endif //FAST_PWM_FAN

bool setTargetedHotend(int code){
  tmp_extruder = active_extruder;
  if(code_seen('T')) {
    tmp_extruder = code_value();
    tmp_extruder = tool_extruder_mapping[tmp_extruder];
    if(tmp_extruder >= EXTRUDERS) {
      SERIAL_ECHO_START;
      switch(code){
        case 104:
          SERIAL_ECHO(MSG_M104_INVALID_EXTRUDER);
          break;
        case 105:
          SERIAL_ECHO(MSG_M105_INVALID_EXTRUDER);
          break;
        case 109:
          SERIAL_ECHO(MSG_M109_INVALID_EXTRUDER);
          break;
        case 218:
          SERIAL_ECHO(MSG_M218_INVALID_EXTRUDER);
          break;
        case 221:
          SERIAL_ECHO(MSG_M221_INVALID_EXTRUDER);
          break;
      }
      SERIAL_ECHOLN(tmp_extruder);
      return true;
    }
  }
  return false;
}
