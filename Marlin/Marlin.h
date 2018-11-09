// Tonokip RepRap firmware rewrite based off of Hydra-mmm firmware.
// License: GPL

#ifndef MARLIN_H
#define MARLIN_H

#define  FORCE_INLINE __attribute__((always_inline)) inline

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>

#include <util/delay.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>

#include "fastio.h"
#include "Configuration.h"
#include "Configuration_heads.h"
#include "pins.h"

#ifndef AT90USB
#define  HardwareSerial_h // trick to disable the standard HWserial
#endif

#if (ARDUINO >= 100)
# include "Arduino.h"
#else
# include "WProgram.h"
  //Arduino < 1.0.0 does not define this, so we need to do it ourselves
# define analogInputToDigitalPin(p) ((p) + A0)
#endif

#ifdef AT90USB
#include "HardwareSerial.h"
#endif

#include "MarlinSerial.h"

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

#include "WString.h"

#ifdef IRSD
#include "irsd.h"
#endif

#ifdef AT90USB
   #ifdef BTENABLED
         #define MYSERIAL bt
   #else
         #define MYSERIAL Serial
   #endif // BTENABLED
#else
  #define MYSERIAL MSerial
#endif

#include "tools.h"

#ifdef SMART_COMM
  #include <SmartComm.h>
#endif

#ifndef CRITICAL_SECTION_START
  #define CRITICAL_SECTION_START  unsigned char _sreg = SREG; cli();
  #define CRITICAL_SECTION_END    SREG = _sreg;
#endif //CRITICAL_SECTION_START

enum tp_report_t:uint8_t { TP_REPORT_NONE=0, TP_REPORT_AUTO=1, TP_REPORT_FULL=2 };

#define SERIAL_PROTOCOL(x) (MYSERIAL.print(x))
#define SERIAL_PROTOCOL_F(x,y) (MYSERIAL.print(x,y))
#define SERIAL_PROTOCOL_P(x) do { serialprintPGM(x); } while (0)
#define SERIAL_PROTOCOLPGM(x) (serialprintPGM(PSTR(x)))
#define SERIAL_PROTOCOLLN(x) (MYSERIAL.print(x),MYSERIAL.write('\n'))
#define SERIAL_PROTOCOLLN_F(x,y) (MYSERIAL.print(x,y),MYSERIAL.write('\n'))
#define SERIAL_PROTOCOLLN_P(x) do { serialprintPGM(x); MYSERIAL.write('\n'); } while (0)
#define SERIAL_PROTOCOLLNPGM(x) (serialprintPGM(PSTR(x)),MYSERIAL.write('\n'))


const char errormagic[]   PROGMEM = "E: ";
const char asyncmagic[]   PROGMEM = "A: ";
const char echomagic[]    PROGMEM = "";
const char commentmagic[] PROGMEM = "// ";

#define SERIAL_ERROR_START  (serialprintPGM(errormagic))
#define SERIAL_ERROR(x)      SERIAL_PROTOCOL(x)
#define SERIAL_ERRORPGM(x)   SERIAL_PROTOCOLPGM(x)
#define SERIAL_ERRORLN(x)    SERIAL_PROTOCOLLN(x)
#define SERIAL_ERRORLN_P(x)  SERIAL_PROTOCOLLN_P(x)
#define SERIAL_ERRORLNPGM(x) SERIAL_PROTOCOLLNPGM(x)

#define _ERROR_STOPPED     101

#define SERIAL_ASYNC_START  (serialprintPGM(asyncmagic))
#define SERIAL_ASYNCLN       SERIAL_PROTOCOLLN(x)
#define SERIAL_ASYNCLN_P     SERIAL_PROTOCOLLN_P(x)
#define SERIAL_ASYNCLNPGM(x) SERIAL_PROTOCOLLNPGM(x)

#define SERIAL_COMMENT_START  (serialprintPGM(commentmagic))

#define SERIAL_ECHO_START //(serialprintPGM(echomagic))
#define SERIAL_ECHO(x) SERIAL_PROTOCOL(x)
#define SERIAL_ECHOPGM(x) SERIAL_PROTOCOLPGM(x)
#define SERIAL_ECHOLN(x) SERIAL_PROTOCOLLN(x)
#define SERIAL_ECHOLNPGM(x) SERIAL_PROTOCOLLNPGM(x)

#define SERIAL_ECHOPAIR(name,value) (serial_echopair_P(PSTR(name),(value)))
#define SERIAL_ECHOPAIR_P(name,value) (serial_echopair_P(name,(value)))

#ifdef DEBUG
  #define LOG_DEBUG(x)      SERIAL_PROTOCOLLN(x)
  #define LOG_DEBUG_F(x,y)  SERIAL_PROTOCOLLN_F(x,y)
  #define LOG_DEBUGPGM(msg) SERIAL_PROTOCOLLNPGM(msg)
  #define SERIAL_DEBUG(var)  SERIAL_PROTOCOLPGM(#var " = "); SERIAL_PROTOCOLLN(var)
  #define LOG_DEBUG_R(reg)  do {                      \
    SERIAL_PROTOCOLPGM(#reg "=");                     \
    for (int i=7; i >= 0; i--) {                      \
      SERIAL_PROTOCOL((unsigned long)((reg>>i) & 1)); \
    }                                                 \
    SERIAL_PROTOCOLLNPGM("");                         \
  } while(0)
#else
  #define LOG_DEBUG(x)
  #define LOG_DEBUG_F(x,y)
  #define LOG_DEBUGPGM(msg)
  #define SERIAL_DEBUG(var)
  #define LOG_DEBUG_R(reg)
#endif

void serial_echopair_P(const char *s_P, float v);
void serial_echopair_P(const char *s_P, double v);
void serial_echopair_P(const char *s_P, unsigned long v);


//Things to write to serial from Program memory. Saves 400 to 2k of RAM.
FORCE_INLINE void serialprintPGM(const char *str)
{
  char ch=pgm_read_byte(str);
  while(ch)
  {
    MYSERIAL.write(ch);
    ch=pgm_read_byte(++str);
  }
}


void get_command();
void process_commands();

void manage_inactivity();

#if defined(DUAL_X_CARRIAGE) && defined(X_ENABLE_PIN) && X_ENABLE_PIN > -1 \
    && defined(X2_ENABLE_PIN) && X2_ENABLE_PIN > -1
  #define  enable_x() do { WRITE(X_ENABLE_PIN, X_ENABLE_ON); WRITE(X2_ENABLE_PIN, X_ENABLE_ON); } while (0)
  #define disable_x() do { WRITE(X_ENABLE_PIN,!X_ENABLE_ON); WRITE(X2_ENABLE_PIN,!X_ENABLE_ON); axis_known_position[X_AXIS] = false; } while (0)
#elif defined(X_ENABLE_PIN) && X_ENABLE_PIN > -1
  #define  enable_x() WRITE(X_ENABLE_PIN, X_ENABLE_ON)
  #define disable_x() { WRITE(X_ENABLE_PIN,!X_ENABLE_ON); axis_known_position[X_AXIS] = false; }
#else
  #define enable_x() ;
  #define disable_x() ;
#endif

#if defined(Y_ENABLE_PIN) && Y_ENABLE_PIN > -1
  #ifdef Y_DUAL_STEPPER_DRIVERS
    #define  enable_y() { WRITE(Y_ENABLE_PIN, Y_ENABLE_ON); WRITE(Y2_ENABLE_PIN,  Y_ENABLE_ON); }
    #define disable_y() { WRITE(Y_ENABLE_PIN,!Y_ENABLE_ON); WRITE(Y2_ENABLE_PIN, !Y_ENABLE_ON); axis_known_position[Y_AXIS] = false; }
  #else
    #define  enable_y() WRITE(Y_ENABLE_PIN, Y_ENABLE_ON)
    #define disable_y() { WRITE(Y_ENABLE_PIN,!Y_ENABLE_ON); axis_known_position[Y_AXIS] = false; }
  #endif
#else
  #define enable_y() ;
  #define disable_y() ;
#endif

#if defined(Z_ENABLE_PIN) && Z_ENABLE_PIN > -1
  #ifdef Z_DUAL_STEPPER_DRIVERS
    #define  enable_z() { WRITE(Z_ENABLE_PIN, Z_ENABLE_ON); WRITE(Z2_ENABLE_PIN, Z_ENABLE_ON); }
    #define disable_z() { WRITE(Z_ENABLE_PIN,!Z_ENABLE_ON); WRITE(Z2_ENABLE_PIN,!Z_ENABLE_ON); axis_known_position[Z_AXIS] = false; }
  #else
    #define  enable_z() WRITE(Z_ENABLE_PIN, Z_ENABLE_ON)
    #define disable_z() { WRITE(Z_ENABLE_PIN,!Z_ENABLE_ON); axis_known_position[Z_AXIS] = false; }
  #endif
#else
  #define enable_z() ;
  #define disable_z() ;
#endif

#if defined(E0_ENABLE_PIN) && (E0_ENABLE_PIN > -1)
  #define enable_e0() WRITE(E0_ENABLE_PIN, E0_ENABLE_ON)
  #define disable_e0() WRITE(E0_ENABLE_PIN,!E0_ENABLE_ON)
#else
  #define enable_e0()  /* nothing */
  #define disable_e0() /* nothing */
#endif

#if (EXTRUDERS > 1) && defined(E1_ENABLE_PIN) && (E1_ENABLE_PIN > -1)
  #define enable_e1() WRITE(E1_ENABLE_PIN, E1_ENABLE_ON)
  #define disable_e1() WRITE(E1_ENABLE_PIN,!E1_ENABLE_ON)
#else
  #define enable_e1()  /* nothing */
  #define disable_e1() /* nothing */
#endif

#if (EXTRUDERS > 2) && defined(E2_ENABLE_PIN) && (E2_ENABLE_PIN > -1)
  #if defined(HEAD_DRIVE_INDEX) && HEAD_DRIVE_INDEX > -1
	#define enable_e2()  do { if (active_extruder==HEAD_DRIVE_INDEX) WRITE(E2_ENABLE_PIN, E2_ENABLE_ON); } while (0)
	#define disable_e2() do { if (active_extruder==HEAD_DRIVE_INDEX) WRITE(E2_ENABLE_PIN,!E2_ENABLE_ON); } while (0)
  #else
    #define enable_e2()  WRITE(E2_ENABLE_PIN, E2_ENABLE_ON)
	#define disable_e2() WRITE(E2_ENABLE_PIN,!E2_ENABLE_ON)
  #endif
#else
  #define enable_e2()  /* nothing */
  #define disable_e2() /* nothing */
#endif

#if (EXTRUDERS > 3) && defined(E3_ENABLE_PIN) && (E3_ENABLE_PIN > -1)
  #if defined(BED_DRIVE_INDEX) && BED_DRIVE_INDEX > -1
    #define enable_e3()  do { if (active_extruder==BED_DRIVE_INDEX) WRITE(E3_ENABLE_PIN, E3_ENABLE_ON); } while (0)
    #define disable_e3() do { if (active_extruder==BED_DRIVE_INDEX) WRITE(E3_ENABLE_PIN,!E3_ENABLE_ON); } while (0)
  #else
    #define enable_e3()  WRITE(E3_ENABLE_PIN, E3_ENABLE_ON)
	#define disable_e3() WRITE(E3_ENABLE_PIN,!E3_ENABLE_ON)
  #endif
#else
  #define enable_e3()  /* nothing */
  #define disable_e3() /* nothing */
#endif

#define disable_e_steppers() do { disable_e0(); disable_e1(); disable_e2(); disable_e3(); } while (0)

#define INVALID_EXTRUDER 252
#define INVALID_EXTRUDER_1 (INVALID_EXTRUDER | 1)
#define INVALID_EXTRUDER_2 (INVALID_EXTRUDER | 2)
#define INVALID_EXTRUDER_3 (INVALID_EXTRUDER | 3)

// Macros for driving various external probes
#ifdef EXTERNAL_ENDSTOP_Z_PROBING

   #define ENABLE_SECURE_SWITCH_ZPROBE() do { enable_secure_switch_zprobe=true; } while (0)
   #define DISABLE_SECURE_SWITCH_ZPROBE() do { enable_secure_switch_zprobe=false; } while (0)

   #if defined(IRSD)
      #undef ENABLE_SECURE_SWITCH_ZPROBE
      #define ENABLE_SECURE_SWITCH_ZPROBE() do { \
         irsd_enable(); \
         enable_secure_switch_zprobe=true; \
      } while (0)

      #undef DISABLE_SECURE_SWITCH_ZPROBE
      #define DISABLE_SECURE_SWITCH_ZPROBE() do { \
         enable_secure_switch_zprobe=false; \
         irsd_disable(); \
      } while (0)
   #endif

#endif

enum AxisEnum {X_AXIS=0, Y_AXIS=1, Z_AXIS=2, E_AXIS=3};


void FlushSerialRequestResend();
void ClearToSend();

void get_coordinates();
#ifdef DELTA
void calculate_delta(float cartesian[3]);
extern float delta[3];
#endif
void prepare_move();
void kill();
void Stop(uint8_t=_ERROR_STOPPED);

void kill_by_door();

void set_amb_color(unsigned int red,unsigned int green,unsigned int blue);
void store_last_amb_color();
void restore_last_amb_color();
void manage_amb_color_fading();
void set_amb_color_fading(bool red,bool green,bool blue,unsigned int fading_speed);
void stop_fading();

void manage_secure_endstop();

bool IsStopped();

void enquecommand(const char *cmd); //put an ASCII command at the end of the current buffer.
void enquecommand_P(const char *cmd); //put an ASCII command at the end of the current buffer, read from flash
void prepare_arc_move(char isclockwise);
void clamp_to_software_endstops(float target[3]);

void refresh_cmd_timeout(void);

void manage_fab_soft_pwm(void);

void Read_Head_Info(bool=false);
char I2C_read(byte i2c_register);

void servo_attach (uint8_t, uint8_t=0);
void servo_detach (uint8_t);

#ifdef FAST_PWM_FAN
void setPwmFrequency(uint8_t pin, int val);
#endif

void print_heaterstates (tp_report_t format=TP_REPORT_FULL);

extern bool min_software_endstops;
extern bool max_software_endstops;

extern float homing_feedrate[];
extern bool axis_relative_modes[];
extern int feedmultiply;
extern int extrudemultiply; // Sets extrude multiply factor (in percent) for all extruders
extern int extruder_multiply[EXTRUDERS]; // sets extrude multiply factor (in percent) for each extruder individually
extern float volumetric_multiplier[EXTRUDERS]; // reciprocal of cross-sectional area of filament (in square millimeters), stored this way to reduce computational burden in planner
extern float current_position[NUM_AXIS] ;
extern float add_homeing[3];
#ifdef DELTA
extern float endstop_adj[3];
extern float delta_radius;
extern float delta_diagonal_rod;
extern float delta_segments_per_second;
void recalc_delta_settings(float radius, float diagonal_rod);
#endif
extern float min_pos[3];
extern float max_pos[3];
extern bool axis_known_position[3];
extern float zprobe_zoffset;
extern int fanSpeed;

#ifdef SERVO_ENDSTOPS
extern const int servo_endstops[];
extern int servo_endstop_angles[];
#endif

#ifdef BARICUDA
extern int ValvePressure;
extern int EtoPPressure;
#endif

#ifdef FAN_SOFT_PWM
extern unsigned char fanSpeedSoftPwm;
#endif

#ifdef FWRETRACT
extern bool autoretract_enabled;
extern bool retracted;
extern float retract_length, retract_feedrate, retract_zlift;
extern float retract_recover_length, retract_recover_feedrate;
#endif

extern unsigned long starttime;
extern unsigned long stoptime;

extern unsigned int ERROR_CODE;

extern bool head_placed;

#ifdef EXTERNAL_ENDSTOP_Z_PROBING
extern volatile bool enable_secure_switch_zprobe;
#endif

#ifdef THERMISTOR_HOTSWAP
namespace ThermistorHotswap
{
  void setTable (const unsigned short);
}

//extern int maxttemp[HEATERS];
#ifdef TEMP_SENSOR_1_AS_REDUNDANT
extern void *heater_ttbl_map[2];
extern uint8_t heater_ttbllen_map[2];
#else
extern void *heater_ttbl_map[HEATERS];
extern uint8_t heater_ttbllen_map[HEATERS];
#endif
extern void *thermistors_map[THERMISTOR_HOTSWAP_SUPPORTED_TYPES_LEN];
extern uint8_t thermistors_map_len[THERMISTOR_HOTSWAP_SUPPORTED_TYPES_LEN];
#endif

#ifdef THERMISTOR_INPUT_HOTSWAP
extern uint8_t extruder_0_thermistor_input_index;
#endif

// Handling multiple extruders pins
extern uint8_t active_tool;     // Active logical tool
extern int8_t active_extruder;
extern int8_t extruder_heater_mapping[EXTRUDERS];

void StopTool();
//extern bool head_is_dummy;
extern int8_t tool_extruder_mapping[TOOLS_MAGAZINE_SIZE];
extern bool    tool_twi_support[TOOLS_MAGAZINE_SIZE];

#ifdef DIGIPOT_I2C
extern void digipot_i2c_set_current( int channel, float current );
extern void digipot_i2c_init();
#endif


extern int servo_extended_angle;
extern int servo_retracted_angle;

extern unsigned long fab_serial_code;
extern unsigned long fab_control_serial_code;
extern unsigned int fab_board_version;
extern unsigned int fab_batch_number;
extern unsigned long fab_control_batch_number;
extern unsigned int led_board_version;
extern unsigned int flex_board_version;
extern unsigned int plateconn_board_version;
extern unsigned int hotplate_board_version;
extern unsigned int general_assembly_version;
extern unsigned int installed_head_id;

extern tool_t installed_head;

#ifdef SMART_COMM

  // Smart Heads
  extern SmartComm SmartHead;

#endif

extern uint8_t working_mode;

//FABtotum IO definition
#define RED_ON()	WRITE(RED_PIN,LOW)
#define RED_OFF()	WRITE(RED_PIN,HIGH)
#define GREEN_ON()	WRITE(GREEN_PIN,LOW)
#define GREEN_OFF()	WRITE(GREEN_PIN,HIGH)
#define BLUE_ON()	WRITE(BLUE_PIN,LOW)
#define BLUE_OFF()	WRITE(BLUE_PIN,HIGH)

#define MAX_PWM         127
#define PWM_SCALE (256/(MAX_PWM+1))

#define HOT_LED_ON()	WRITE(HOT_LED_PIN,HIGH)
#define HOT_LED_OFF()	WRITE(HOT_LED_PIN,LOW)

#define HEAD_LIGHT_ON()	WRITE(HEAD_LIGHT_PIN,HIGH)
#define HEAD_LIGHT_OFF()	WRITE(HEAD_LIGHT_PIN,LOW)
#define LASER_GATE_ON()	WRITE(LASER_GATE_PIN,LOW)
#define LASER_GATE_OFF()	WRITE(LASER_GATE_PIN,HIGH)

#define MILL_MOTOR_ON()	 WRITE(MILL_MOTOR_ON_PIN,HIGH)
#define MILL_MOTOR_OFF() if (active_extruder!=2 || IsStopped()) WRITE(MILL_MOTOR_ON_PIN,LOW)

#define MILL_MOTOR_STATUS()  READ(MILL_MOTOR_ON_PIN)

#define SERVO1_ON()	if (active_extruder!=2 || IsStopped()) WRITE(NOT_SERVO1_ON_PIN,LOW)
#define SERVO1_OFF()	if (active_extruder!=2) WRITE(NOT_SERVO1_ON_PIN,HIGH)
#define SERVO1_STATUS()   !READ(NOT_SERVO1_ON_PIN)

#define SERVO2_ON()	WRITE(NOT_SERVO2_ON_PIN,LOW)
#define SERVO2_OFF()	WRITE(NOT_SERVO2_ON_PIN,HIGH)
#define SERVO2_STATUS()  !READ(NOT_SERVO2_ON_PIN)

//FABtotum fastio definition
// READ(IO); WRITE(IO, v);  TOGGLE(IO) ;  SET_INPUT(IO)  ; SET_OUTPUT(IO);  GET_INPUT(IO) ; GET_OUTPUT(IO) ; GET_TIMER(IO)
#define RASPI_PWR_ON()	        WRITE(NOT_RASPI_PWR_ON_PIN,0)
#define RASPI_PWR_OFF()	        WRITE(NOT_RASPI_PWR_ON_PIN,1)
#define RASPI_PWR_STATUS()      !READ(NOT_RASPI_PWR_ON_PIN)

#define LIGHT_SIGN_ON()	        WRITE(LIGHT_SIGN_ON_PIN,1)
#define LIGHT_SIGN_OFF()	WRITE(LIGHT_SIGN_ON_PIN,0)

#define RPI_RECOVERY_ON()	WRITE(RPI_RECOVERY_PIN,1)
#define RPI_RECOVERY_OFF()	WRITE(RPI_RECOVERY_PIN,0)
#define RPI_RECOVERY_STATUS()   READ(RPI_RECOVERY_PIN)

#define RPI_ERROR_ACK_ON()	WRITE(RPI_RECOVERY_PIN,1)
#define RPI_ERROR_ACK_OFF()	WRITE(RPI_RECOVERY_PIN,0)
#define RPI_ERROR_STATUS()       READ(RPI_RECOVERY_PIN)
#define RASPI_MAX_TURN_OFF_DELAY  3000  //30 seconds, value in tens of ms

#define BEEP_ON()  TCCR0B = (TCCR0B & 0b11111000) | 0x02; analogWrite(BEEPER, 127);
#define BEEP_OFF() TCCR0B = (TCCR0B & 0b11111000) | 0x03; analogWrite(BEEPER, 255);

#define SERVO_SPINDLE_MAX  1832    //(MILL MOTOR input: 1060 us equal to Full CCW, 1460us equal to zero, 1860us equal to Full CW)
#define SERVO_SPINDLE_MIN  1148
#define SERVO_SPINDLE_ZERO  1488
#define SERVO_SPINDLE_ARM  1550

#define RPM_SPINDLE_MAX  14684    // spindle max rpm
#define RPM_SPINDLE_MIN  0     // spindle min rpm

#define WIRE_END_STATUS()      READ(WIRE_END_PIN)

#define	SECURE_SW_STATUS()     !READ(NOT_SECURE_SW_PIN)
#define	REEL_LENS_OPEN_STATUS()     !READ(NOT_REEL_LENS_OPEN_PIN)
#define	DOOR_OPEN_STATUS()     READ(DOOR_OPEN_PIN)

//messages
#define MSG_WIRE_END      "filament end: "
#define MSG_DOOR_OPEN      "door open: "
#define MSG_REEL_LENS_OPEN      "reel lens open: "
#define MSG_SECURE_SWITCH      "secure switch: "
#define MSG_HOT_BED_PLACED      "hotbed placed: "


//error codes
#define ERROR_KILLED      100
#define ERROR_STOPPED    _ERROR_STOPPED  // 101
#define ERROR_DOOR_OPEN   102
#define ERROR_MIN_TEMP    103
#define ERROR_MAX_TEMP    104
#define ERROR_MAX_BED_TEMP  105
#define ERROR_X_MAX_ENDSTOP  106
#define ERROR_X_MIN_ENDSTOP  107
#define ERROR_Y_MAX_ENDSTOP  108
#define ERROR_Y_MIN_ENDSTOP  109
#define ERROR_IDLE_SAFETY    110
#define ERROR_WIRE_END       111

//error codes for FABUI configurable functionalities
#define ERROR_Y_BOTH_TRIGGERED   120
#define ERROR_Z_BOTH_TRIGGERED   121

//temperature related messages
#define ERROR_AMBIENT_TEMP   122
#define ERROR_EXTRUDE_MINTEMP   123
#define ERROR_LONG_EXTRUSION   124
#define ERROR_HEAD_ABSENT    125

#define ERROR_INVALID_EXTRUDER 252
#if (EXTRUDERS > 1)
  #define ERROR_INVALID_EXTRUDER_1 (ERROR_INVALID_EXTRUDER | 1)
#endif
#if (EXTRUDERS > 2)
  #define ERROR_INVALID_EXTRUDER_2 (ERROR_INVALID_EXTRUDER | 2)
#endif

//POWER SHUTDOWN REQUEST:
#define ERROR_PWR_OFF    999

//Head Serial ID
#define SERIAL_ID_ADDR          80//(0x50)
#define SERIAL_N_FAM_DEV_CODE  0
#define SERIAL_N_0             1
#define SERIAL_N_1             2
#define SERIAL_N_2             3
#define SERIAL_N_3             4
#define SERIAL_N_4             5
#define SERIAL_N_5             6
#define SERIAL_N_CRC           7

#define I2C_MAX_TIMEOUT 1000

#define AVG_MEASURED_Z_MAX     1

#define WORKING_MODE_HYBRID 0
#define WORKING_MODE_FFF    1
#define WORKING_MODE_LASER  2
#define WORKING_MODE_CNC    3
#define MACHINE_MODE_SCAN   4
#define MACHINE_MODE_SLA    5

#define COMMENT_SEPARATOR ';'
#define VALUE_LIST_SEPARATOR ':'  // Separator for lists of values as parameter arguments

#endif
