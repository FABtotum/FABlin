/*
  temperature.c - temperature control
  Part of Marlin

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
#include "ultralcd.h"
#include "temperature.h"
#include "watchdog.h"
#include "Configuration_heads.h"
#include "language.h"

//===========================================================================
//=============================public variables============================
//===========================================================================
int target_temperature[HEATERS] = { 0 };
int target_temperature_bed = 0;
int current_temperature_raw[HEATERS] = { 0 };
float current_temperature[HEATERS] = { 0.0 };
int current_temperature_bed_raw = 0;
float current_temperature_bed = 0.0;
int current_pressure_raw_value=0;
int current_mon_main_supply_raw_value=0;
int current_mon_sec_supply_raw_value=0;
int current_main_current_raw_value=0;

float current_pressure_value=0.0;
float current_mon_24v_value=0.0;
float current_mon_5v_value=0.0;
float current_main_current_value=0.0;

#ifdef TEMP_SENSOR_1_AS_REDUNDANT
  int redundant_temperature_raw = 0;
  float redundant_temperature = 0.0;
#endif
#ifdef PIDTEMP
  float Kp=DEFAULT_Kp;
  float Ki=(DEFAULT_Ki*PID_dT);
  float Kd=(DEFAULT_Kd/PID_dT);
  #ifdef PID_ADD_EXTRUSION_RATE
    float Kc=DEFAULT_Kc;
  #endif
#endif //PIDTEMP

#ifdef PIDTEMPBED
  float bedKp=DEFAULT_bedKp;
  float bedKi=(DEFAULT_bedKi*PID_dT);
  float bedKd=(DEFAULT_bedKd/PID_dT);
#endif //PIDTEMPBED

#ifdef FAN_SOFT_PWM
  unsigned char fanSpeedSoftPwm;
#endif

unsigned char soft_pwm_bed;

unsigned int ERROR_CODE;

#ifdef BABYSTEPPING
  volatile int babystepsTodo[3]={0,0,0};
#endif

//===========================================================================
//=============================private variables============================
//===========================================================================
volatile uint8_t enabled_features = 0xff;

static volatile bool temp_meas_ready = false;

#ifdef PIDTEMP
  //static cannot be external:
  static float temp_iState[HEATERS] = { 0 };
  static float temp_dState[HEATERS] = { 0 };
  static float pTerm[HEATERS];
  static float iTerm[HEATERS];
  static float dTerm[HEATERS];
  //int output;
  static float pid_error[HEATERS];
  static float temp_iState_min[HEATERS];
  static float temp_iState_max[HEATERS];
  // static float pid_input[EXTRUDERS];
  // static float pid_output[EXTRUDERS];
  static bool pid_reset[HEATERS];
#endif //PIDTEMP
#ifdef PIDTEMPBED
  //static cannot be external:
  static float temp_iState_bed = { 0 };
  static float temp_dState_bed = { 0 };
  static float pTerm_bed;
  static float iTerm_bed;
  static float dTerm_bed;
  //int output;
  static float pid_error_bed;
  static float temp_iState_min_bed;
  static float temp_iState_max_bed;
#else //PIDTEMPBED
	static unsigned long  previous_millis_bed_heater;
#endif //PIDTEMPBED
  static unsigned char soft_pwm[HEATERS];

#ifdef FAN_SOFT_PWM
  static unsigned char soft_pwm_fan;
#endif
#if (defined(EXTRUDER_0_AUTO_FAN_PIN) && EXTRUDER_0_AUTO_FAN_PIN > -1) || \
    (defined(EXTRUDER_1_AUTO_FAN_PIN) && EXTRUDER_1_AUTO_FAN_PIN > -1) || \
    (defined(EXTRUDER_2_AUTO_FAN_PIN) && EXTRUDER_2_AUTO_FAN_PIN > -1)
  static unsigned long extruder_autofan_last_check;
#endif

#if HEATERS > 3
  # error Unsupported number of heaters
#elif HEATERS > 2
  # define ARRAY_BY_EXTRUDERS(v1, v2, v3) { v1, v2, v3 }
#elif HEATERS > 1
  # define ARRAY_BY_EXTRUDERS(v1, v2, v3) { v1, v2 }
#else
  # define ARRAY_BY_EXTRUDERS(v1, v2, v3) { v1 }
#endif

// Init min and max temp with extreme values to prevent false errors during startup
static int minttemp_raw[HEATERS] = ARRAY_BY_EXTRUDERS( HEATER_0_RAW_LO_TEMP , HEATER_1_RAW_LO_TEMP , HEATER_2_RAW_LO_TEMP );
static int maxttemp_raw[HEATERS] = ARRAY_BY_EXTRUDERS( HEATER_0_RAW_HI_TEMP , HEATER_1_RAW_HI_TEMP , HEATER_2_RAW_HI_TEMP );

#ifdef THERMISTOR_HOTSWAP
int minttemp[HEATERS] = ARRAY_BY_EXTRUDERS( 0, 0, 0 );
int maxttemp[HEATERS] = ARRAY_BY_EXTRUDERS( 16383, 16383, 16383 );
#else
static int minttemp[HEATERS] = ARRAY_BY_EXTRUDERS( 0, 0, 0 );
static int maxttemp[HEATERS] = ARRAY_BY_EXTRUDERS( 16383, 16383, 16383 );
#endif

//static int bed_minttemp_raw = HEATER_BED_RAW_LO_TEMP; /* No bed mintemp error implemented?!? */
#ifdef BED_MAXTEMP
static int bed_maxttemp_raw = HEATER_BED_RAW_HI_TEMP;
#endif

#ifdef TEMP_SENSOR_1_AS_REDUNDANT
  #ifdef THERMISTOR_HOTSWAP
  void *heater_ttbl_map[2] = {(void *)HEATER_0_TEMPTABLE, (void *)HEATER_1_TEMPTABLE };
  uint8_t heater_ttbllen_map[2] = { HEATER_0_TEMPTABLE_LEN, HEATER_1_TEMPTABLE_LEN };
  #else
  static void *heater_ttbl_map[2] = {(void *)HEATER_0_TEMPTABLE, (void *)HEATER_1_TEMPTABLE };
  static uint8_t heater_ttbllen_map[2] = { HEATER_0_TEMPTABLE_LEN, HEATER_1_TEMPTABLE_LEN };
  #endif
#else
  #ifdef THERMISTOR_HOTSWAP
  void *heater_ttbl_map[HEATERS] = ARRAY_BY_EXTRUDERS( (void *)HEATER_0_TEMPTABLE, (void *)HEATER_1_TEMPTABLE, (void *)HEATER_2_TEMPTABLE );
  uint8_t heater_ttbllen_map[HEATERS] = ARRAY_BY_EXTRUDERS( HEATER_0_TEMPTABLE_LEN, HEATER_1_TEMPTABLE_LEN, HEATER_2_TEMPTABLE_LEN );
  #else
  static void *heater_ttbl_map[HEATERS] = ARRAY_BY_EXTRUDERS( (void *)HEATER_0_TEMPTABLE, (void *)HEATER_1_TEMPTABLE, (void *)HEATER_2_TEMPTABLE );
  static uint8_t heater_ttbllen_map[HEATERS] = ARRAY_BY_EXTRUDERS( HEATER_0_TEMPTABLE_LEN, HEATER_1_TEMPTABLE_LEN, HEATER_2_TEMPTABLE_LEN );
  #endif
#endif

#ifdef THERMISTOR_HOTSWAP
  void *thermistors_map[THERMISTOR_HOTSWAP_SUPPORTED_TYPES_LEN] = GHEATER_TEMPTABLE(THERMISTOR_HOTSWAP_SUPPORTED_TYPES_LEN);
  uint8_t thermistors_map_len[THERMISTOR_HOTSWAP_SUPPORTED_TYPES_LEN] = GHEATER_TEMPTABLE_LEN(THERMISTOR_HOTSWAP_SUPPORTED_TYPES_LEN);
#endif

#ifdef THERMISTOR_INPUT_HOTSWAP
  uint8_t extruder_0_thermistor_input_index = THERMISTOR_HOTSWAP_INPUT_DEFAULT_INDEX;
#endif


static float analog2temp(int raw, uint8_t e);
static float analog2tempBed(int raw);
static void updateTemperaturesFromRawValues();

#ifdef WATCH_TEMP_PERIOD
int watch_start_temp[HEATERS] = ARRAY_BY_EXTRUDERS(0,0,0);
unsigned long watchmillis[HEATERS] = ARRAY_BY_EXTRUDERS(0,0,0);
#endif //WATCH_TEMP_PERIOD

#ifndef SOFT_PWM_SCALE
#define SOFT_PWM_SCALE 0
#endif

// We store last temperature error code here, because `M999` (by a legacy
// implementation upon which FABuiOS relies) must be able to reset and ignore
// errors even when the error conditions are still present.
// When this is already set (!= 0) the same error code is not raised again. The
// only way to reset this is to reset the firmware!
static uint8_t error_code = 0;

//===========================================================================
//=============================   functions      ============================
//===========================================================================

bool PID_autotune(float temp, int extruder, int ncycles)
{
  float input = 0.0;
  int cycles=0;
  bool heating = true;

  unsigned long temp_millis = millis();
  unsigned long t1=temp_millis;
  unsigned long t2=temp_millis;
  long t_high = 0;
  long t_low = 0;

  long bias, d;
  float Ku, Tu;
  float Kp, Ki, Kd;
  float max = 0, min = 10000;

  if ((extruder > HEATERS)
  #if (TEMP_BED_PIN <= -1)
       ||(extruder < 0)
  #endif
       ){
         SERIAL_ERROR_START;
         SERIAL_ERRORLNPGM("PID Autotune failed. Bad extruder number.");
          return false;
        }

  /*SERIAL_COMMENT_START;
  SERIAL_ECHOLNPGM("PID Autotune start");*/

  disable_heater(); // switch off all heaters.

  if (extruder<0)
  {
     soft_pwm_bed = (MAX_BED_POWER)/2;
     bias = d = (MAX_BED_POWER)/2;
   }
   else
   {
     soft_pwm[extruder] = (PID_MAX)/2;
     bias = d = (PID_MAX)/2;
  }

 for(;;) {

    while (!temp_meas_ready);
    if(temp_meas_ready == true) { // temp sample ready
      updateTemperaturesFromRawValues();

      input = (extruder<0)?current_temperature_bed:current_temperature[extruder];

      max=max(max,input);
      min=min(min,input);
      if(heating == true && input > temp) {
        if(millis() - t2 > 5000) {
          heating=false;
          if (extruder<0)
            soft_pwm_bed = (bias - d) >> 1;
          else
            soft_pwm[extruder] = (bias - d) >> 1;
          t1=millis();
          t_high=t1 - t2;
          max=temp;
        }
      }
      if(heating == false && input < temp) {
        if(millis() - t1 > 5000) {
          heating=true;
          t2=millis();
          t_low=t2 - t1;
          if(cycles > 0) {
            bias += (d*(t_high - t_low))/(t_low + t_high);
            bias = constrain(bias, 20 ,(extruder<0?(MAX_BED_POWER):(PID_MAX))-20);
            if(bias > (extruder<0?(MAX_BED_POWER):(PID_MAX))/2) d = (extruder<0?(MAX_BED_POWER):(PID_MAX)) - 1 - bias;
            else d = bias;

            SERIAL_COMMENT_START;
            SERIAL_PROTOCOLPGM(" bias: "); SERIAL_PROTOCOL(bias);
            SERIAL_PROTOCOLPGM(" d: "); SERIAL_PROTOCOL(d);
            SERIAL_PROTOCOLPGM(" min: "); SERIAL_PROTOCOL(min);
            SERIAL_PROTOCOLPGM(" max: "); SERIAL_PROTOCOLLN(max);
            if(cycles > 2) {
              Ku = (4.0*d)/(3.14159*(max-min)/2.0);
              Tu = ((float)(t_low + t_high)/1000.0);
              SERIAL_COMMENT_START;
              SERIAL_PROTOCOLPGM(" Ku: "); SERIAL_PROTOCOL(Ku);
              SERIAL_PROTOCOLPGM(" Tu: "); SERIAL_PROTOCOLLN(Tu);
              Kp = 0.6*Ku;
              Ki = 2*Kp/Tu;
              Kd = Kp*Tu/8;
              //SERIAL_COMMENT_START; SERIAL_PROTOCOLLNPGM(" Classic PID ");
              SERIAL_COMMENT_START; SERIAL_PROTOCOLPGM(" Kp: "); SERIAL_PROTOCOLLN(Kp);
              SERIAL_COMMENT_START; SERIAL_PROTOCOLPGM(" Ki: "); SERIAL_PROTOCOLLN(Ki);
              SERIAL_COMMENT_START; SERIAL_PROTOCOLPGM(" Kd: "); SERIAL_PROTOCOLLN(Kd);
            }
          }
          if (extruder<0)
            soft_pwm_bed = (bias + d) >> 1;
          else
            soft_pwm[extruder] = (bias + d) >> 1;
          cycles++;
          min=temp;
        }
      }
    }
    if(input > (temp + 20)) {
      SERIAL_ERROR_START;
      SERIAL_PROTOCOLLNPGM("PID Autotune failed! Temperature too high");
      return false;
    }
    if(millis() - temp_millis > 2000) {
      //int p;
      print_heaterstates(TP_REPORT_AUTO);

      temp_millis = millis();
    }
    if(((millis() - t1) + (millis() - t2)) > (10L*60L*1000L*2L)) {
      SERIAL_ERROR_START;
      SERIAL_PROTOCOLLNPGM("PID Autotune failed! timeout");
      return false;
    }
    if (cycles > ncycles) {
      //SERIAL_PROTOCOLLNPGM("PID Autotune finished! Put the last Kp, Ki and Kd constants from above into Configuration.h");
      // Why not just print them:
      SERIAL_PROTOCOLPGM(MSG_OK);
      SERIAL_PROTOCOLPGM(" Kp: "); SERIAL_PROTOCOL(Kp);
      SERIAL_PROTOCOLPGM(" Ki: "); SERIAL_PROTOCOL(Ki);
      SERIAL_PROTOCOLPGM(" Kd: "); SERIAL_PROTOCOLLN(Kd);
      return true;
    }
    lcd_update();
  }

  return false;
}

void updatePID()
{
#ifdef PIDTEMP
  for(int e = 0; e < HEATERS; e++) {
     temp_iState_max[e] = PID_INTEGRAL_DRIVE_MAX / Ki;
  }
#endif
#ifdef PIDTEMPBED
  temp_iState_max_bed = PID_INTEGRAL_DRIVE_MAX / bedKi;
#endif
}

int getHeaterPower(int heater) {
	if (heater<0)
		return soft_pwm_bed;
  return soft_pwm[heater];
}

#if (defined(EXTRUDER_0_AUTO_FAN_PIN) && EXTRUDER_0_AUTO_FAN_PIN > -1) || \
    (defined(EXTRUDER_1_AUTO_FAN_PIN) && EXTRUDER_1_AUTO_FAN_PIN > -1) || \
    (defined(EXTRUDER_2_AUTO_FAN_PIN) && EXTRUDER_2_AUTO_FAN_PIN > -1)

  #if defined(FAN_PIN) && FAN_PIN > -1
    #if EXTRUDER_0_AUTO_FAN_PIN == FAN_PIN
       #error "You cannot set EXTRUDER_0_AUTO_FAN_PIN equal to FAN_PIN"
    #endif
    #if EXTRUDER_1_AUTO_FAN_PIN == FAN_PIN
       #error "You cannot set EXTRUDER_1_AUTO_FAN_PIN equal to FAN_PIN"
    #endif
    #if EXTRUDER_2_AUTO_FAN_PIN == FAN_PIN
       #error "You cannot set EXTRUDER_2_AUTO_FAN_PIN equal to FAN_PIN"
    #endif
  #endif

void setExtruderAutoFanState(int pin, bool state)
{
  unsigned char newFanSpeed = (state != 0) ? EXTRUDER_AUTO_FAN_SPEED : 0;
  // this idiom allows both digital and PWM fan outputs (see M42 handling).
  pinMode(pin, OUTPUT);
  digitalWrite(pin, newFanSpeed);
  analogWrite(pin, newFanSpeed);
}

void checkExtruderAutoFans()
{
  uint8_t fanState = 0;

  // which fan pins need to be turned on?
  #if defined(EXTRUDER_0_AUTO_FAN_PIN) && EXTRUDER_0_AUTO_FAN_PIN > -1
    if (current_temperature[0] > EXTRUDER_AUTO_FAN_TEMPERATURE)
      fanState |= 1;
  #endif
  #if defined(EXTRUDER_1_AUTO_FAN_PIN) && EXTRUDER_1_AUTO_FAN_PIN > -1
    if (current_temperature[1] > EXTRUDER_AUTO_FAN_TEMPERATURE)
    {
      if (EXTRUDER_1_AUTO_FAN_PIN == EXTRUDER_0_AUTO_FAN_PIN)
        fanState |= 1;
      else
        fanState |= 2;
    }
  #endif
  #if defined(EXTRUDER_2_AUTO_FAN_PIN) && EXTRUDER_2_AUTO_FAN_PIN > -1
    if (current_temperature[2] > EXTRUDER_AUTO_FAN_TEMPERATURE)
    {
      if (EXTRUDER_2_AUTO_FAN_PIN == EXTRUDER_0_AUTO_FAN_PIN)
        fanState |= 1;
      else if (EXTRUDER_2_AUTO_FAN_PIN == EXTRUDER_1_AUTO_FAN_PIN)
        fanState |= 2;
      else
        fanState |= 4;
    }
  #endif

  // update extruder auto fan states
  #if defined(EXTRUDER_0_AUTO_FAN_PIN) && EXTRUDER_0_AUTO_FAN_PIN > -1
    setExtruderAutoFanState(EXTRUDER_0_AUTO_FAN_PIN, (fanState & 1) != 0);
  #endif
  #if defined(EXTRUDER_1_AUTO_FAN_PIN) && EXTRUDER_1_AUTO_FAN_PIN > -1
    if (EXTRUDER_1_AUTO_FAN_PIN != EXTRUDER_0_AUTO_FAN_PIN)
      setExtruderAutoFanState(EXTRUDER_1_AUTO_FAN_PIN, (fanState & 2) != 0);
  #endif
  #if defined(EXTRUDER_2_AUTO_FAN_PIN) && EXTRUDER_2_AUTO_FAN_PIN > -1
    if (EXTRUDER_2_AUTO_FAN_PIN != EXTRUDER_0_AUTO_FAN_PIN
        && EXTRUDER_2_AUTO_FAN_PIN != EXTRUDER_1_AUTO_FAN_PIN)
      setExtruderAutoFanState(EXTRUDER_2_AUTO_FAN_PIN, (fanState & 4) != 0);
  #endif
}

#endif // any extruder auto fan pins set

void manage_heater()
{
  float pid_input;
  float pid_output;

  if(temp_meas_ready != true)   //better readability
    return;

  updateTemperaturesFromRawValues();

  for(int e = 0; e < HEATERS; e++)
  {

  #ifdef PIDTEMP
    pid_input = current_temperature[e];

    #ifndef PID_OPENLOOP
        pid_error[e] = target_temperature[e] - pid_input;
        if(pid_error[e] > PID_FUNCTIONAL_RANGE) {
          pid_output = BANG_MAX;
          pid_reset[e] = true;
        }
        else if(pid_error[e] < -PID_FUNCTIONAL_RANGE || target_temperature[e] == 0) {
          pid_output = 0;
          pid_reset[e] = true;
        }
        else {
          if(pid_reset[e] == true) {
            temp_iState[e] = 0.0;
            pid_reset[e] = false;
          }
          pTerm[e] = Kp * pid_error[e];
          temp_iState[e] += pid_error[e];
          temp_iState[e] = constrain(temp_iState[e], temp_iState_min[e], temp_iState_max[e]);
          iTerm[e] = Ki * temp_iState[e];

          //K1 defined in Configuration.h in the PID settings
          #define K2 (1.0-K1)
          dTerm[e] = (Kd * (pid_input - temp_dState[e]))*K2 + (K1 * dTerm[e]);
          pid_output = constrain(pTerm[e] + iTerm[e] - dTerm[e], 0, PID_MAX);
        }
        temp_dState[e] = pid_input;
    #else
          pid_output = constrain(target_temperature[e], 0, PID_MAX);
    #endif //PID_OPENLOOP
    #ifdef PID_DEBUG
    SERIAL_ECHO_START;
    SERIAL_ECHO(" PID_DEBUG ");
    SERIAL_ECHO(e);
    SERIAL_ECHO(": Input ");
    SERIAL_ECHO(pid_input);
    SERIAL_ECHO(" Output ");
    SERIAL_ECHO(pid_output);
    SERIAL_ECHO(" pTerm ");
    SERIAL_ECHO(pTerm[e]);
    SERIAL_ECHO(" iTerm ");
    SERIAL_ECHO(iTerm[e]);
    SERIAL_ECHO(" dTerm ");
    SERIAL_ECHOLN(dTerm[e]);
    #endif //PID_DEBUG
  #else /* PID off */
    pid_output = 0;
    if(current_temperature[e] < target_temperature[e]) {
      pid_output = PID_MAX;
    }
  #endif

    // Check if temperature is within the correct range
    if((current_temperature[e] > minttemp[e]) && (current_temperature[e] < maxttemp[e]))
    {
      soft_pwm[e] = (int)pid_output >> 1;
    }
    else {
      soft_pwm[e] = 0;
    }

    #ifdef WATCH_TEMP_PERIOD
    if(watchmillis[e] && millis() - watchmillis[e] > WATCH_TEMP_PERIOD)
    {
        if(degHotend(e) < watch_start_temp[e] + WATCH_TEMP_INCREASE)
        {
            setTargetHotend(0, e);
            LCD_MESSAGEPGM("Heating failed");
            SERIAL_ECHO_START;
            SERIAL_ECHOLN("Heating failed");
        }else{
            watchmillis[e] = 0;
        }
    }
    #endif
    #ifdef TEMP_SENSOR_1_AS_REDUNDANT
      if(fabs(current_temperature[0] - redundant_temperature) > MAX_REDUNDANT_TEMP_SENSOR_DIFF) {
        disable_heater();
        if(IsStopped() == false) {
          SERIAL_ERROR_START;
          SERIAL_ERRORLNPGM("Extruder switched off. Temperature difference between temp sensors is too high !");
          LCD_ALERTMESSAGEPGM("Err: REDUNDANT TEMP ERROR");
        }
        #ifndef BOGUS_TEMPERATURE_FAILSAFE_OVERRIDE
          Stop();
        #endif
      }
    #endif
  } // End extruder for loop

  #if (defined(EXTRUDER_0_AUTO_FAN_PIN) && EXTRUDER_0_AUTO_FAN_PIN > -1) || \
      (defined(EXTRUDER_1_AUTO_FAN_PIN) && EXTRUDER_1_AUTO_FAN_PIN > -1) || \
      (defined(EXTRUDER_2_AUTO_FAN_PIN) && EXTRUDER_2_AUTO_FAN_PIN > -1)
  if(millis() - extruder_autofan_last_check > 2500)  // only need to check fan state very infrequently
  {
    checkExtruderAutoFans();
    extruder_autofan_last_check = millis();
  }
  #endif

  #ifndef PIDTEMPBED
  if(millis() - previous_millis_bed_heater < BED_CHECK_INTERVAL)
    return;
  previous_millis_bed_heater = millis();
  #endif

  #if TEMP_SENSOR_BED != 0

    #ifdef PIDTEMPBED
    pid_input = current_temperature_bed;

      #ifndef PID_OPENLOOP
		  pid_error_bed = target_temperature_bed - pid_input;
		  pTerm_bed = bedKp * pid_error_bed;
		  temp_iState_bed += pid_error_bed;
		  temp_iState_bed = constrain(temp_iState_bed, temp_iState_min_bed, temp_iState_max_bed);
		  iTerm_bed = bedKi * temp_iState_bed;

		  //K1 defined in Configuration.h in the PID settings
        #define K2 (1.0-K1)
		  dTerm_bed= (bedKd * (pid_input - temp_dState_bed))*K2 + (K1 * dTerm_bed);
		  temp_dState_bed = pid_input;

		  pid_output = constrain(pTerm_bed + iTerm_bed - dTerm_bed, 0, MAX_BED_POWER);

      #else
      pid_output = constrain(target_temperature_bed, 0, MAX_BED_POWER);
      #endif //PID_OPENLOOP

	  if((current_temperature_bed > BED_MINTEMP) && (current_temperature_bed < BED_MAXTEMP))
	  {
	    soft_pwm_bed = (int)pid_output >> 1;
	  }
	  else {
	    soft_pwm_bed = 0;
	  }

    #elif !defined(BED_LIMIT_SWITCHING)
      if (enabled_features & TP_SENSOR_BED)
      {
        // Check if temperature is within the correct range
        if((current_temperature_bed > BED_MINTEMP) && (current_temperature_bed < BED_MAXTEMP))
        {
          if(current_temperature_bed >= target_temperature_bed)
          {
            soft_pwm_bed = 0;
          }
          else
          {
            soft_pwm_bed = MAX_BED_POWER>>1;
          }
        }
        else
        {
          soft_pwm_bed = 0;
          WRITE(HEATER_BED_PIN,LOW);
        }
      }
    #else //#ifdef BED_LIMIT_SWITCHING

      if (enabled_features & TP_SENSOR_BED)
      {
        // Check if temperature is within the correct band
        if((current_temperature_bed > BED_MINTEMP) && (current_temperature_bed < BED_MAXTEMP))
        {
          if(current_temperature_bed > target_temperature_bed + BED_HYSTERESIS)
          {
            soft_pwm_bed = 0;
          }
          else if(current_temperature_bed <= target_temperature_bed - BED_HYSTERESIS)
          {
            soft_pwm_bed = MAX_BED_POWER>>1;
          }
        }
        else
        {
          soft_pwm_bed = 0;
          WRITE(HEATER_BED_PIN,LOW);
        }
      }
    #endif
  #endif
}

#define PGM_RD_W(x)   (short)pgm_read_word(&x)
// Derived from RepRap FiveD extruder::getTemperature()
// For hot end temperature measurement.
static float analog2temp(int raw, uint8_t e) {
#ifdef TEMP_SENSOR_1_AS_REDUNDANT
  if(e > HEATERS)
#else
  if(e >= HEATERS)
#endif
  {
      SERIAL_ERROR_START;
      SERIAL_ERROR((int)e);
      SERIAL_ERRORLNPGM(" - Invalid extruder number !");
      kill();
  }
  #ifdef HEATER_0_USES_MAX6675
    if (e == 0)
    {
      return 0.25 * raw;
    }
  #endif

  if(heater_ttbl_map[e] != NULL)
  {
    float celsius = 0;
    uint8_t i;
    short (*tt)[][2] = (short (*)[][2])(heater_ttbl_map[e]);

    for (i=1; i<heater_ttbllen_map[e]; i++)
    {
      if (PGM_RD_W((*tt)[i][0]) > raw)
      {
        celsius = PGM_RD_W((*tt)[i-1][1]) +
          (raw - PGM_RD_W((*tt)[i-1][0])) *
          (float)(PGM_RD_W((*tt)[i][1]) - PGM_RD_W((*tt)[i-1][1])) /
          (float)(PGM_RD_W((*tt)[i][0]) - PGM_RD_W((*tt)[i-1][0]));
        break;
      }
    }

    // Overflow: Set to last value in the table
    if (i == heater_ttbllen_map[e]) celsius = PGM_RD_W((*tt)[i-1][1]);

    return celsius;
  }
  return ((raw * ((5.0 * 100.0) / 1024.0) / OVERSAMPLENR) * TEMP_SENSOR_AD595_GAIN) + TEMP_SENSOR_AD595_OFFSET;
}

// Derived from RepRap FiveD extruder::getTemperature()
// For bed temperature measurement.
static float analog2tempBed(int raw) {
  #ifdef BED_USES_THERMISTOR
    float celsius = 0;
    byte i;

    for (i=1; i<BEDTEMPTABLE_LEN; i++)
    {
      if (PGM_RD_W(BEDTEMPTABLE[i][0]) > raw)
      {
        celsius  = PGM_RD_W(BEDTEMPTABLE[i-1][1]) +
          (raw - PGM_RD_W(BEDTEMPTABLE[i-1][0])) *
          (float)(PGM_RD_W(BEDTEMPTABLE[i][1]) - PGM_RD_W(BEDTEMPTABLE[i-1][1])) /
          (float)(PGM_RD_W(BEDTEMPTABLE[i][0]) - PGM_RD_W(BEDTEMPTABLE[i-1][0]));
        break;
      }
    }

    // Overflow: Set to last value in the table
    if (i == BEDTEMPTABLE_LEN) celsius = PGM_RD_W(BEDTEMPTABLE[i-1][1]);

    return celsius;
  #elif defined BED_USES_AD595
    return ((raw * ((5.0 * 100.0) / 1024.0) / OVERSAMPLENR) * TEMP_SENSOR_AD595_GAIN) + TEMP_SENSOR_AD595_OFFSET;
  #else
    return 0;
  #endif
}

/* Called to get the raw values into the the actual temperatures. The raw values are created in interrupt context,
    and this function is called from normal context as it is too slow to run in interrupts and will block the stepper routine otherwise */
static void updateTemperaturesFromRawValues()
{
    for (uint8_t e=0; e < HEATERS; e++)
    {
      if (enabled_features & (TP_SENSOR_0<<e)) {
        current_temperature[e] = analog2temp(current_temperature_raw[e], e);
      } else {
        current_temperature[e] = NOT_A_TEMPERATURE;
      }
    }

    if (enabled_features & TP_SENSOR_BED) {
      current_temperature_bed = analog2tempBed(current_temperature_bed_raw);
    } else {
      current_temperature_bed = NOT_A_TEMPERATURE;
    }

#ifdef TEMP_SENSOR_1_AS_REDUNDANT
    if (enabled_features & TEMP_SENSOR_1) {
      redundant_temperature = analog2temp(redundant_temperature_raw, 1);
    }
#endif

	current_pressure_value=((float)current_pressure_raw_value/OVERSAMPLENR);
	current_mon_24v_value=((float)current_mon_main_supply_raw_value/OVERSAMPLENR)*LSB_VALUE*DIV_24V;
	current_mon_5v_value=((float)current_mon_sec_supply_raw_value/OVERSAMPLENR)*LSB_VALUE*DIV_5V;
	current_main_current_value=((float)current_main_current_raw_value/OVERSAMPLENR)*LSB_VALUE*DIV_CURR;

    //Reset the watchdog after we know we have a temperature measurement.
    watchdog_reset();

    CRITICAL_SECTION_START;
    temp_meas_ready = false;
    CRITICAL_SECTION_END;
}

// TODO: make this dynamic by actually probing the configured ttable for the selected sensor
bool ttable_sorting (tp_features sensor)
{
  // Treat heaters as their numerically matching sensors
  // e.g.: TP_HEATER_0 (0x01) >> TP_SENSOR_0 (0x10)
  if (sensor < TP_SENSORS) sensor <<= 4;

  switch (sensor)
  {
    case TP_SENSOR_BED:
#if HEATER_BED_RAW_LO_TEMP < HEATER_BED_RAW_HI_TEMP
      return true;
#else
      return false;
#endif
      break;

    case TP_SENSOR_2:
#if HEATER_2_RAW_LO_TEMP < HEATER_2_RAW_HI_TEMP
      return true;
#else
      return false;
#endif
      break;

    case TP_SENSOR_1:
#if HEATER_1_RAW_LO_TEMP < HEATER_2_RAW_HI_TEMP
      return true;
#else
      return false;
#endif
      break;

    default:
    case TP_SENSOR_0:
#if HEATER_0_RAW_LO_TEMP < HEATER_0_RAW_HI_TEMP
      return true;
#else
      return false;
#endif
  }
}

/*
 * Function: tp_init_mintemp
 *
 * Set min temp limit for the given heater(s). Only available for head
 * heaters; not for the bed heater.
 *
 * Parameters:
 *
 *  value - min temp value in °C to set for the given heater(s). Valid values go from -127 to +127
 *  heater - the heater for which to set min temp, can be a bitmask of tp_features
 *
 */
void tp_init_mintemp (int8_t value, tp_features heater)
{
  for (unsigned int h = 0; h < HEATERS; h++)
  {
    if ((heater & (1<<h)) == 0) continue;

LOG_DEBUGPGM("tp_init_mintemp");
    minttemp[h] = value;

SERIAL_DEBUG(heater);
SERIAL_DEBUG(value);
SERIAL_DEBUG(HEATER_0_RAW_LO_TEMP);
SERIAL_DEBUG(HEATER_0_RAW_HI_TEMP);
    bool direct = ttable_sorting(heater);
    if (direct) {
      minttemp_raw[h] = 0;
    } else {
      minttemp_raw[h] = 1024 *OVERSAMPLENR;
    }
SERIAL_DEBUG(minttemp_raw[h]);
    while (analog2temp(minttemp_raw[h], h) < value) {
SERIAL_DEBUG(minttemp_raw[h]);
      if (direct) {
        minttemp_raw[h] += OVERSAMPLENR;
      } else {
        minttemp_raw[h] -= OVERSAMPLENR;
      }
    }
  }
}

/**
 * Function: tp_init_maxtemp
 *
 * Set max temp for heater(s)
 *
 * Parameters:
 *  value - Max temp in °C (from 0 to 65535)
 *  heater - Heater to for which to change max temp value
 *
 */
void tp_init_maxtemp (int16_t value, tp_features heater)
{
  for (unsigned int h = 0; h < HEATERS; h++)
  {
    if ((heater & (1<<h)) == 0) continue;

    maxttemp[h] = value;
    bool direct = ttable_sorting(heater);

    if (direct) {
      maxttemp_raw[0] = 1024 *OVERSAMPLENR;
    } else {
      maxttemp_raw[0] = 0;
    }
    while(analog2temp(maxttemp_raw[0], 0) > value) {
      if (direct) {
        maxttemp_raw[0] -= OVERSAMPLENR;
      } else {
        maxttemp_raw[0] += OVERSAMPLENR;
      }
    }
  }
}

inline void tp_enable_fan ()
{
  #if defined(FAN_PIN) && (FAN_PIN > -1)
    SET_OUTPUT(FAN_PIN);
    #ifdef FAST_PWM_FAN
    setPwmFrequency(FAN_PIN, 1); // No prescaling. Pwm frequency = F_CPU/256/8
    #endif
    #ifdef FAN_SOFT_PWM
    soft_pwm_fan = fanSpeedSoftPwm / 2;
    #endif
  #endif
}

void inline start_isr (bool force=false)
{
  // Use timer0 for temperature measurement
  // Interleave temperature interrupt with millies interrupt
  OCR0B = 128;

  if ((TIMSK0 & (1<<OCIE0B)) == 0 || force) {
    temp_meas_ready = false;
    TIMSK0 |= (1<<OCIE0B);
    // Wait for temperature measurement to settle
    delay(250);
  }
}

/*void inline stop_isr ()
{
  TIMSK0 &= ~(1<<OCIE0B);
}*/

void tp_init (uint8_t features)
{
  //stop_isr();
  enabled_features = features;
  tp_init();
}

void tp_init()
{
  //stop_isr();

#if (MOTHERBOARD == 80) && ((TEMP_SENSOR_0==-1)||(TEMP_SENSOR_1==-1)||(TEMP_SENSOR_2==-1)||(TEMP_SENSOR_BED==-1))
  //disable RUMBA JTAG in case the thermocouple extension is plugged on top of JTAG connector
  MCUCR=(1<<JTD);
  MCUCR=(1<<JTD);
#endif

  // Finish init of mult extruder arrays
  for(int e = 0; e < HEATERS; e++) {
    // populate with the first value
    maxttemp[e] = maxttemp[0];
#ifdef PIDTEMP
    temp_iState_min[e] = 0.0;
    temp_iState_max[e] = PID_INTEGRAL_DRIVE_MAX / Ki;
#endif //PIDTEMP
#ifdef PIDTEMPBED
    temp_iState_min_bed = 0.0;
    temp_iState_max_bed = PID_INTEGRAL_DRIVE_MAX / bedKi;
#endif //PIDTEMPBED
  }

  tp_enable_fan();

  tp_enable_heater(enabled_features & TP_HEATERS);

  //tp_enable_sensor(enabled_features & TP_SENSORS);

  /*if (enabled_features) {
    // Use timer0 for temperature measurement
    // Interleave temperature interrupt with millies interrupt
    OCR0B = 128;
    TIMSK0 |= (1<<OCIE0B);
    // Wait for temperature measurement to settle
    delay(250);
  }*/

#ifdef HEATER_0_MINTEMP
  tp_init_mintemp(HEATER_0_MINTEMP, TP_HEATER_0);
#endif //MINTEMP
#ifdef HEATER_0_MAXTEMP
  tp_init_maxtemp(HEATER_0_MAXTEMP, TP_HEATER_0);
  /*maxttemp[0] = HEATER_0_MAXTEMP;
  while(analog2temp(maxttemp_raw[0], 0) > HEATER_0_MAXTEMP) {
#if HEATER_0_RAW_LO_TEMP < HEATER_0_RAW_HI_TEMP
    maxttemp_raw[0] -= OVERSAMPLENR;
#else
    maxttemp_raw[0] += OVERSAMPLENR;
#endif
  }*/
#endif //MAXTEMP

#if (HEATERS > 1) && defined(HEATER_1_MINTEMP)
  tp_init_mintemp(HEATER_1_MINTEMP, TP_HEATER_1);
#endif // MINTEMP 1
#if (HEATERS > 1) && defined(HEATER_1_MAXTEMP)
  tp_init_maxtemp(HEATER_1_MAXTEMP, TP_HEATER_1);
  /*maxttemp[1] = HEATER_1_MAXTEMP;
  while(analog2temp(maxttemp_raw[1], 1) > HEATER_1_MAXTEMP) {
#if HEATER_1_RAW_LO_TEMP < HEATER_1_RAW_HI_TEMP
    maxttemp_raw[1] -= OVERSAMPLENR;
#else
    maxttemp_raw[1] += OVERSAMPLENR;
#endif
  }*/
#endif //MAXTEMP 1

#if (HEATERS > 2) && defined(HEATER_2_MINTEMP)
  tp_init_mintemp(HEATER_2_MINTEMP, TP_HEATER_2);
#endif //MINTEMP 2
#if (HEATERS > 2) && defined(HEATER_2_MAXTEMP)
  tp_init_maxtemp(HEATER_2_MAXTEMP, TP_HEATER_2);
  /*maxttemp[2] = HEATER_2_MAXTEMP;
  while(analog2temp(maxttemp_raw[2], 2) > HEATER_2_MAXTEMP) {
#if HEATER_2_RAW_LO_TEMP < HEATER_2_RAW_HI_TEMP
    maxttemp_raw[2] -= OVERSAMPLENR;
#else
    maxttemp_raw[2] += OVERSAMPLENR;
#endif
  }*/
#endif //MAXTEMP 2

#ifdef BED_MINTEMP
  /* No bed MINTEMP error implemented?!? */ /*
  while(analog2tempBed(bed_minttemp_raw) < BED_MINTEMP) {
#if HEATER_BED_RAW_LO_TEMP < HEATER_BED_RAW_HI_TEMP
    bed_minttemp_raw += OVERSAMPLENR;
#else
    bed_minttemp_raw -= OVERSAMPLENR;
#endif
  }
  */
#endif //BED_MINTEMP
#ifdef BED_MAXTEMP
  while(analog2tempBed(bed_maxttemp_raw) > BED_MAXTEMP) {
#if HEATER_BED_RAW_LO_TEMP < HEATER_BED_RAW_HI_TEMP
    bed_maxttemp_raw -= OVERSAMPLENR;
#else
    bed_maxttemp_raw += OVERSAMPLENR;
#endif
  }
#endif //BED_MAXTEMP
}

void setWatch()
{
#ifdef WATCH_TEMP_PERIOD
  for (int e = 0; e < HEATERS; e++)
  {
    if(degHotend(e) < degTargetHotend(e) - (WATCH_TEMP_INCREASE * 2))
    {
      watch_start_temp[e] = degHotend(e);
      watchmillis[e] = millis();
    }
  }
#endif
}

void tp_enable_heater (uint8_t heaters)
{
  //stop_isr();

  // Sensors must also be enabled, otherwise we are screwed
  tp_enable_sensor(heaters << 4);

  if (heaters & TP_HEATER_0) {
#if defined(HEATER_0_PIN) && (HEATER_0_PIN > -1)
    SET_OUTPUT(HEATER_0_PIN);
#endif

#ifdef HEATER_0_USES_MAX6675
  #ifndef SDSUPPORT
      SET_OUTPUT(MAX_SCK_PIN);
      WRITE(MAX_SCK_PIN,0);

      SET_OUTPUT(MAX_MOSI_PIN);
      WRITE(MAX_MOSI_PIN,1);

      SET_INPUT(MAX_MISO_PIN);
      WRITE(MAX_MISO_PIN,1);
  #endif
    SET_OUTPUT(MAX6675_SS);
    WRITE(MAX6675_SS,1);
#endif
  }

  if (heaters & TP_HEATER_1) {
#if defined(HEATER_1_PIN) && (HEATER_1_PIN > -1)
    SET_OUTPUT(HEATER_1_PIN);
#endif
  }

  if (heaters & TP_HEATER_2) {
#if defined(HEATER_2_PIN) && (HEATER_2_PIN > -1)
    SET_OUTPUT(HEATER_2_PIN);
#endif
  }

  if (heaters & TP_HEATER_BED) {
#if defined(HEATER_BED_PIN) && (HEATER_BED_PIN > -1)
    SET_OUTPUT(HEATER_BED_PIN);
#endif
  }

  enabled_features |= heaters;
  start_isr();
}

void tp_disable_heater (uint8_t heaters)
{
  //stop_isr();
  enabled_features &= ~heaters;

#if defined(TEMP_0_PIN) && TEMP_0_PIN > -1
  if (enabled_features & (heaters & TP_HEATER_0)) {
    setTargetHotend(0,0);
    target_temperature[0]=0;
    soft_pwm[0]=0;
  #if defined(HEATER_0_PIN) && HEATER_0_PIN > -1
    WRITE(HEATER_0_PIN,LOW);
  #endif
  }
#endif

#if defined(TEMP_1_PIN) && TEMP_1_PIN > -1
  if (enabled_features & (heaters & TP_HEATER_1)) {
    setTargetHotend(0,1);
    target_temperature[1]=0;
    soft_pwm[1]=0;
  #if defined(HEATER_1_PIN) && HEATER_1_PIN > -1
    WRITE(HEATER_1_PIN,LOW);
  #endif
  }
#endif

#if defined(TEMP_2_PIN) && TEMP_2_PIN > -1
  if (enabled_features & (heaters & TP_HEATER_2)) {
    setTargetHotend(0,2);
    target_temperature[2]=0;
    soft_pwm[2]=0;
  #if defined(HEATER_2_PIN) && HEATER_2_PIN > -1
    WRITE(HEATER_2_PIN,LOW);
  #endif
  }
#endif

#if defined(TEMP_BED_PIN) && TEMP_BED_PIN > -1
  if (heaters & TP_HEATER_BED) {
    setTargetBed(0);
    target_temperature_bed=0;
    soft_pwm_bed=0;
  #if defined(HEATER_BED_PIN) && HEATER_BED_PIN > -1
    WRITE(HEATER_BED_PIN,LOW);
  #endif
  }
#endif

  //start_isr();
}

void disable_heater()
{
  for(int i=0;i<HEATERS;i++)
    setTargetHotend(0,i);
  setTargetBed(0);
  #if defined(TEMP_0_PIN) && TEMP_0_PIN > -1
    target_temperature[0]=0;
    soft_pwm[0]=0;
    #if defined(HEATER_0_PIN) && HEATER_0_PIN > -1
    WRITE(HEATER_0_PIN,LOW);
   #endif
  #endif

  #if defined(TEMP_1_PIN) && TEMP_1_PIN > -1
    #if (HEATERS > 1)
      target_temperature[1]=0;
      soft_pwm[1]=0;
    #endif
    #if defined(HEATER_1_PIN) && HEATER_1_PIN > -1
      WRITE(HEATER_1_PIN,LOW);
    #endif
  #endif

  #if defined(TEMP_2_PIN) && TEMP_2_PIN > -1
     target_temperature[2]=0;
     soft_pwm[2]=0;
    #if defined(HEATER_2_PIN) && HEATER_2_PIN > -1
      WRITE(HEATER_2_PIN,LOW);
    #endif
  #endif

  #if defined(TEMP_BED_PIN) && TEMP_BED_PIN > -1
    target_temperature_bed=0;
    soft_pwm_bed=0;
    #if defined(HEATER_BED_PIN) && HEATER_BED_PIN > -1
      WRITE(HEATER_BED_PIN,LOW);
    #endif
  #endif
}

void tp_enable_sensor (uint8_t sensors)
{
  //stop_isr();

  if (sensors) {
    // Set analog inputs
    ADCSRA = 1<<ADEN | 1<<ADSC | 1<<ADIF | 0x07;
    DIDR0 = 0;
    #ifdef DIDR2
      DIDR2 = 0;
    #endif
  }

  if (sensors & TP_SENSOR_0) {
#if defined(TEMP_0_PIN) && (TEMP_0_PIN > -1)
    SET_ANALOG(TEMP_0_PIN);
#endif
  }

  if (sensors & TP_SENSOR_1) {
#if defined(TEMP_1_PIN) && (TEMP_1_PIN > -1)
    SET_ANALOG(TEMP_1_PIN);
#endif
  }

  if (sensors & TP_SENSOR_2) {
#if defined(TEMP_2_PIN) && (TEMP_2_PIN > -1)
    SET_ANALOG(TEMP_2_PIN);
#endif
  }

  if (sensors & TP_SENSOR_BED) {
#if defined(TEMP_BED_PIN) && (TEMP_BED_PIN > -1)
    SET_ANALOG(TEMP_BED_PIN);
#endif
  }
  start_isr(/*true*/);

  enabled_features |= sensors;
}

void tp_disable_sensor (uint8_t sensors)
{
  //stop_isr();
  enabled_features &= ~sensors;

  // When sensors are disabled heaters cannot work as well
  tp_disable_heater(sensors >> 4);

  if (sensors & TP_SENSOR_0) {
#if defined(TEMP_0_PIN) && (TEMP_0_PIN > -1)
  #if TEMP_0_PIN < 8
    DIDR0 &= ~(1 << TEMP_0_PIN);
  #else
    DIDR2 &= ~(1<<(TEMP_0_PIN - 8));
  #endif
#endif
  }

  if (sensors & TP_SENSOR_1) {
#if defined(TEMP_1_PIN) && (TEMP_1_PIN > -1)
  #if TEMP_1_PIN < 8
    DIDR0 &= ~(1<<TEMP_1_PIN);
  #else
    DIDR2 &= ~(1<<(TEMP_1_PIN - 8));
  #endif
#endif
  }

  if (sensors & TP_SENSOR_2) {
#if defined(TEMP_2_PIN) && (TEMP_2_PIN > -1)
  #if TEMP_2_PIN < 8
    DIDR0 &= ~(1 << TEMP_2_PIN);
  #else
    DIDR2 &= ~(1<<(TEMP_2_PIN - 8));
  #endif
#endif
  }

  if (sensors & TP_SENSOR_BED) {
#if defined(TEMP_BED_PIN) && (TEMP_BED_PIN > -1)
  #if TEMP_BED_PIN < 8
    DIDR0 &= ~(1<<TEMP_BED_PIN);
  #else
    DIDR2 &= ~(1<<(TEMP_BED_PIN - 8));
  #endif
#endif
  }

  //start_isr();
}

void inline max_temp_error (uint8_t e)
{
  // Only honor error if the relevant TP_SENSOR_e was enabled
  if (!(enabled_features & TP_SENSOR_0<<e)) return;

  if (enabled_features & TP_HEATER_0<<e) disable_heater();

  if (IsStopped() == false)
  {
    RPI_ERROR_ACK_ON();
    ERROR_CODE=ERROR_MAX_TEMP;
    SERIAL_ASYNC_START;
    SERIAL_ERRORLN((int)e);
    SERIAL_ERRORLNPGM(": Extruder switched off. MAXTEMP triggered !");
    LCD_ALERTMESSAGEPGM("Err: MAXTEMP");
#ifndef BOGUS_TEMPERATURE_FAILSAFE_OVERRIDE
  Stop(ERROR_MAX_TEMP);
#endif
  }
}

void min_temp_error (uint8_t e)
{
  // Silently skip this error if it was already raised since last firmware boot
  if (error_code == ERROR_MIN_TEMP) return;

  // Only process error if the relevant TP_SENSOR_e was enabled
  if (!(enabled_features & TP_SENSOR_0<<e)) {
    return;
  }

  head_placed = false;

  if (enabled_features & TP_HEATER_0<<e) {
    disable_heater();
  }

  if (IsStopped() == false)
  {
    SERIAL_ASYNC_START;
    SERIAL_ERRORLN((int)e);
    SERIAL_ERRORLNPGM(": Extruder switched off. MINTEMP triggered !");
    LCD_ALERTMESSAGEPGM("Err: MINTEMP");
  }
#ifndef BOGUS_TEMPERATURE_FAILSAFE_OVERRIDE
  Stop();
#endif

  RPI_ERROR_ACK_ON();
  error_code = ERROR_CODE = ERROR_MIN_TEMP;
}

void bed_max_temp_error (void)
{
  // Only disable heater if TP_HEATER_BED was enabled
  if (!(enabled_features & TP_SENSOR_BED)) return;

  if (enabled_features & TP_HEATER_BED) disable_heater();

  if (IsStopped() == false)
  {
    RPI_ERROR_ACK_ON();
    ERROR_CODE=ERROR_MAX_BED_TEMP;
    SERIAL_ASYNC_START;
    SERIAL_ERRORLNPGM("Temperature heated bed switched off. MAXTEMP triggered !!");
    LCD_ALERTMESSAGEPGM("Err: MAXTEMP BED");
#ifndef BOGUS_TEMPERATURE_FAILSAFE_OVERRIDE
    Stop();
#endif
  }
}

#ifdef HEATER_0_USES_MAX6675
#define MAX6675_HEAT_INTERVAL 250
long max6675_previous_millis = -HEAT_INTERVAL;
int max6675_temp = 2000;

int read_max6675()
{
  if (millis() - max6675_previous_millis < MAX6675_HEAT_INTERVAL)
    return max6675_temp;

  max6675_previous_millis = millis();
  max6675_temp = 0;

  #ifdef	PRR
    PRR &= ~(1<<PRSPI);
  #elif defined PRR0
    PRR0 &= ~(1<<PRSPI);
  #endif

  SPCR = (1<<MSTR) | (1<<SPE) | (1<<SPR0);

  // enable TT_MAX6675
  WRITE(MAX6675_SS, 0);

  // ensure 100ns delay - a bit extra is fine
  asm("nop");//50ns on 20Mhz, 62.5ns on 16Mhz
  asm("nop");//50ns on 20Mhz, 62.5ns on 16Mhz

  // read MSB
  SPDR = 0;
  for (;(SPSR & (1<<SPIF)) == 0;);
  max6675_temp = SPDR;
  max6675_temp <<= 8;

  // read LSB
  SPDR = 0;
  for (;(SPSR & (1<<SPIF)) == 0;);
  max6675_temp |= SPDR;

  // disable TT_MAX6675
  WRITE(MAX6675_SS, 1);

  if (max6675_temp & 4)
  {
    // thermocouple open
    max6675_temp = 2000;
  }
  else
  {
    max6675_temp = max6675_temp >> 3;
  }

  return max6675_temp;
}
#endif

// Timer 0 is shared with millies
ISR(TIMER0_COMPB_vect)
{
  //these variables are only accesible from the ISR, but static, so they don't lose their value
  static unsigned char temp_count = 0;
  static unsigned long raw_temp_0_value = 0;
#if defined(TEMP_1_PIN) && (TEMP_1_PIN > -1)
  static unsigned long raw_temp_1_value = 0;
#endif
#if defined(TEMP_2_PIN) && (TEMP_2_PIN > -1)
  static unsigned long raw_temp_2_value = 0;
#endif
  static unsigned long raw_temp_bed_value = 0;
  static unsigned long pressure_raw_value = 0;
  static unsigned long mon_24V_raw_value = 0;
  static unsigned long mon_5V_raw_value = 0;
  static unsigned long main_curr_raw_value = 0;

  static unsigned char temp_state = 16;
  static unsigned char pwm_count = (1 << SOFT_PWM_SCALE);
  static unsigned char soft_pwm_0;
  #if (HEATERS > 1) || defined(HEATERS_PARALLEL)
  static unsigned char soft_pwm_1;
  #endif
  #if (HEATERS > 2)
  static unsigned char soft_pwm_2;
  #endif
  #if HEATER_BED_PIN > -1
  static unsigned char soft_pwm_b;
  #endif

#if defined(DEBUG) && defined(TP_ISR_PROFILE_PIN)
  WRITE(TP_ISR_PROFILE_PIN,1);
#endif

  if(pwm_count == 0){
    soft_pwm_0 = soft_pwm[0];
    if(soft_pwm_0 > 0) {
      WRITE(HEATER_0_PIN,1);
      #ifdef HEATERS_PARALLEL
      WRITE(HEATER_1_PIN,1);
      #endif
    } else {
      if (enabled_features & TP_HEATER_0) WRITE(HEATER_0_PIN,0);
    }

    #if HEATERS > 1
    soft_pwm_1 = soft_pwm[1];
    if(soft_pwm_1 > 0) WRITE(HEATER_1_PIN,1); else WRITE(HEATER_1_PIN,0);
    #endif
    #if HEATERS > 2
    soft_pwm_2 = soft_pwm[2];
    if(soft_pwm_2 > 0) WRITE(HEATER_2_PIN,1); else WRITE(HEATER_2_PIN,0);
    #endif
    #if defined(HEATER_BED_PIN) && HEATER_BED_PIN > -1
    soft_pwm_b = soft_pwm_bed;
    if(soft_pwm_b > 0) {
      WRITE(HEATER_BED_PIN,1);
    } else {
      if (enabled_features & TP_HEATER_BED)
        WRITE(HEATER_BED_PIN,0);
    }
    #endif
    #ifdef FAN_SOFT_PWM
    soft_pwm_fan = fanSpeedSoftPwm / 2;
    if(soft_pwm_fan > 0) WRITE(FAN_PIN,1); else WRITE(FAN_PIN,0);
    #endif
  }
  if(soft_pwm_0 < pwm_count) {
      if (enabled_features & TP_HEATER_0) WRITE(HEATER_0_PIN,0);
      #ifdef HEATERS_PARALLEL
      WRITE(HEATER_1_PIN,0);
      #endif
    }
  #if HEATERS > 1
  if(soft_pwm_1 < pwm_count) WRITE(HEATER_1_PIN,0);
  #endif
  #if HEATERS > 2
  if(soft_pwm_2 < pwm_count) WRITE(HEATER_2_PIN,0);
  #endif
  #if defined(HEATER_BED_PIN) && HEATER_BED_PIN > -1
  if (soft_pwm_b < pwm_count)
  if (enabled_features & TP_HEATER_BED)
    WRITE(HEATER_BED_PIN,0);
  #endif
  #ifdef FAN_SOFT_PWM
  if(soft_pwm_fan < pwm_count) WRITE(FAN_PIN,0);
  #endif

  pwm_count += (1 << SOFT_PWM_SCALE);
  pwm_count &= 0x7f;

  switch(temp_state) {
    case 0: // Prepare TEMP_0
      #if defined(TEMP_0_PIN) && (TEMP_0_PIN > -1)
        #if TEMP_0_PIN > 7
          ADCSRB = 1<<MUX5;
        #else
          ADCSRB = 0;
        #endif
        ADMUX = ((1 << REFS0) | (TEMP_0_PIN & 0x07));
        ADCSRA |= 1<<ADSC; // Start conversion
      #endif
      lcd_buttons_update();
      temp_state = 1;
      break;
    case 1: // Measure TEMP_0
      #if defined(TEMP_0_PIN) && (TEMP_0_PIN > -1)
	raw_temp_0_value += ADC;
      #endif
      #ifdef HEATER_0_USES_MAX6675 // TODO remove the blocking
        raw_temp_0_value = read_max6675();
      #endif
      temp_state = 2;
      break;
    case 2: // Prepare TEMP_BED
      #if defined(TEMP_BED_PIN) && (TEMP_BED_PIN > -1)
        #if TEMP_BED_PIN > 7
          ADCSRB = 1<<MUX5;
        #else
          ADCSRB = 0;
        #endif
        ADMUX = ((1 << REFS0) | (TEMP_BED_PIN & 0x07));
        ADCSRA |= 1<<ADSC; // Start conversion
      #endif
      lcd_buttons_update();
      temp_state = 3;
      break;
    case 3: // Measure TEMP_BED
      #if defined(TEMP_BED_PIN) && (TEMP_BED_PIN > -1)
        raw_temp_bed_value += ADC;
      #endif
      temp_state = 4;
      break;
    case 4: // Prepare TEMP_1
      #if defined(TEMP_1_PIN) && (TEMP_1_PIN > -1)
        #if TEMP_1_PIN > 7
          ADCSRB = 1<<MUX5;
        #else
          ADCSRB = 0;
        #endif
        ADMUX = ((1 << REFS0) | (TEMP_1_PIN & 0x07));
        ADCSRA |= 1<<ADSC; // Start conversion
      #endif
      lcd_buttons_update();
      temp_state = 5;
      break;
    case 5: // Measure TEMP_1
      #if defined(TEMP_1_PIN) && (TEMP_1_PIN > -1)
	raw_temp_1_value += ADC;
      #endif
      temp_state = 6;
      break;
    case 6: // Prepare TEMP_2
      #if defined(TEMP_2_PIN) && (TEMP_2_PIN > -1)
        #if TEMP_2_PIN > 7
          ADCSRB = 1<<MUX5;
        #else
          ADCSRB = 0;
        #endif
        ADMUX = ((1 << REFS0) | (TEMP_2_PIN & 0x07));
        ADCSRA |= 1<<ADSC; // Start conversion
      #endif
      lcd_buttons_update();
      temp_state = 7;
      break;
    case 7: // Measure TEMP_2
      #if defined(TEMP_2_PIN) && (TEMP_2_PIN > -1)
        raw_temp_2_value += ADC;
      #endif
      temp_state = 8;
      temp_count++;
      break;
    case 8: // Prepare Pressure force sensor
      #if defined(PRESSURE_ANALOG_PIN) && (PRESSURE_ANALOG_PIN > -1)
        #if PRESSURE_ANALOG_PIN > 7
          ADCSRB = 1<<MUX5;
        #else
          ADCSRB = 0;
        #endif
        ADMUX = ((1 << REFS0) | (PRESSURE_ANALOG_PIN & 0x07));
        ADCSRA |= 1<<ADSC; // Start conversion
      #endif
      temp_state = 9;
      break;
    case 9: // Measure Pressure force sensor
      #if defined(PRESSURE_ANALOG_PIN) && (PRESSURE_ANALOG_PIN > -1)
        pressure_raw_value += ADC;
      #endif
      temp_state = 10;
      break;
    case 10: // Prepare 24V monitor
      #if defined(MON_24V_PIN) && (MON_24V_PIN > -1)
        #if MON_24V_PIN > 7
          ADCSRB = 1<<MUX5;
        #else
          ADCSRB = 0;
        #endif
        ADMUX = ((1 << REFS0) | (MON_24V_PIN & 0x07));
        ADCSRA |= 1<<ADSC; // Start conversion
      #endif
      temp_state = 11;
      break;
    case 11: // Measure 24V monitor
      #if defined(MON_24V_PIN) && (MON_24V_PIN > -1)
        mon_24V_raw_value += ADC;
      #endif
      temp_state = 12;
      break;
    case 12: // Prepare 5V monitor
      #if defined(MON_5V_PIN) && (MON_5V_PIN > -1)
        #if MON_5V_PIN > 7
          ADCSRB = 1<<MUX5;
        #else
          ADCSRB = 0;
        #endif
        ADMUX = ((1 << REFS0) | (MON_5V_PIN & 0x07));
        ADCSRA |= 1<<ADSC; // Start conversion
      #endif
      temp_state = 13;
      break;
    case 13: // Measure 5V monitor
      #if defined(MON_5V_PIN) && (MON_5V_PIN > -1)
        mon_5V_raw_value += ADC;
      #endif
      temp_state = 14;
      break;
    case 14: // Prepare Current monitor
      #if defined(MAIN_CURRENT_SENSE_PIN) && (MAIN_CURRENT_SENSE_PIN > -1)
        #if MAIN_CURRENT_SENSE_PIN > 7
          ADCSRB = 1<<MUX5;
        #else
          ADCSRB = 0;
        #endif
        ADMUX = ((1 << REFS0) | (MAIN_CURRENT_SENSE_PIN & 0x07));
        ADCSRA |= 1<<ADSC; // Start conversion
      #endif
      temp_state = 15;
      break;
    case 15: // Measure Current monitor
      #if defined(MAIN_CURRENT_SENSE_PIN) && (MAIN_CURRENT_SENSE_PIN > -1)
        main_curr_raw_value += ADC;
      #endif
      temp_state = 0;
      break;
    case 16: //Startup, delay initial temp reading a tiny bit so the hardware can settle.
      temp_state = 0;
      break;
//    default:
//      SERIAL_ERROR_START;
//      SERIAL_ERRORLNPGM("Temp measurement error!");
//      break;
  }

  if(temp_count >= OVERSAMPLENR) // 8 * 16 * 1/(16000000/64/256)  = 131ms.
  {
    if (enabled_features & TP_SENSORS)
    {
      if (!temp_meas_ready) //Only update the raw values if they have been read. Else we could be updating them during reading.
      {

        #ifdef THERMISTOR_INPUT_HOTSWAP
           #if (TEMP_1_PIN < 0)
           #error "THERMISTOR_INPUT_HOTSWAP needs both TEMP_0 and TEMP_1 to be defined"
           #endif
        current_temperature_raw[0] = ((extruder_0_thermistor_input_index == 1)?raw_temp_1_value:raw_temp_0_value);
        #else
        current_temperature_raw[0] = raw_temp_0_value;
        #endif

  #if HEATERS > 1
        current_temperature_raw[1] = raw_temp_1_value;
  #endif
  #ifdef TEMP_SENSOR_1_AS_REDUNDANT
        redundant_temperature_raw = raw_temp_1_value;
  #endif
  #if HEATERS > 2
        current_temperature_raw[2] = raw_temp_2_value;
  #endif
        current_temperature_bed_raw = raw_temp_bed_value;

        current_pressure_raw_value = pressure_raw_value;
        current_mon_main_supply_raw_value = mon_24V_raw_value;
        current_mon_sec_supply_raw_value = mon_5V_raw_value;
        current_main_current_raw_value = main_curr_raw_value;

      }

      if (enabled_features & TP_SENSOR_0)
      {
  #if HEATER_0_RAW_LO_TEMP > HEATER_0_RAW_HI_TEMP
        if(current_temperature_raw[0] <= maxttemp_raw[0])
  #else
        if(current_temperature_raw[0] >= maxttemp_raw[0])
  #endif
            max_temp_error(0);

  #if HEATER_0_RAW_LO_TEMP > HEATER_0_RAW_HI_TEMP
        if(current_temperature_raw[0] >= minttemp_raw[0])
  #else
        if(current_temperature_raw[0] <= minttemp_raw[0])
  #endif
          min_temp_error(0);
      }

  #if HEATERS > 1
      if (enabled_features & TP_SENSOR_1)
      {
  #if HEATER_1_RAW_LO_TEMP > HEATER_1_RAW_HI_TEMP
        if(current_temperature_raw[1] <= maxttemp_raw[1])
  #else
        if(current_temperature_raw[1] >= maxttemp_raw[1])
  #endif
        max_temp_error(1);

  #if HEATER_1_RAW_LO_TEMP > HEATER_1_RAW_HI_TEMP
        if(current_temperature_raw[1] >= minttemp_raw[1])
  #else
        if(current_temperature_raw[1] <= minttemp_raw[1])
  #endif
        min_temp_error(1);
      }
  #endif

  #if HEATERS > 2
      if (enabled_features & TP_SENSOR_2)
      {
  #if HEATER_2_RAW_LO_TEMP > HEATER_2_RAW_HI_TEMP
        if(current_temperature_raw[2] <= maxttemp_raw[2])
  #else
        if(current_temperature_raw[2] >= maxttemp_raw[2])
  #endif
          max_temp_error(2);

  #if HEATER_2_RAW_LO_TEMP > HEATER_2_RAW_HI_TEMP
        if(current_temperature_raw[2] >= minttemp_raw[2])
  #else
        if(current_temperature_raw[2] <= minttemp_raw[2])
  #endif
        min_temp_error(2);
      }
  #endif

      if (enabled_features & TP_SENSOR_BED)
      {
    /* No bed MINTEMP error? */
  #if defined(BED_MAXTEMP) && (TEMP_SENSOR_BED != 0)
    #if HEATER_BED_RAW_LO_TEMP > HEATER_BED_RAW_HI_TEMP
        if(current_temperature_bed_raw <= bed_maxttemp_raw)
    #else
        if(current_temperature_bed_raw >= bed_maxttemp_raw)
    #endif
        {
           //target_temperature_bed = 0;
           bed_max_temp_error();
        }
  #endif
      }
    }

    temp_meas_ready = true;
    temp_count = 0;
    raw_temp_0_value = 0;
  #if defined(TEMP_1_PIN) && (TEMP_1_PIN > -1)
    raw_temp_1_value = 0;
  #endif
  #if defined(TEMP_2_PIN) && (TEMP_2_PIN > -1)
    raw_temp_2_value = 0;
  #endif
    raw_temp_bed_value = 0;
    pressure_raw_value = 0;
    mon_24V_raw_value = 0;
    mon_5V_raw_value = 0;
    main_curr_raw_value = 0;
  }

#ifdef BABYSTEPPING
  for(uint8_t axis=0;axis<3;axis++)
  {
    int curTodo=babystepsTodo[axis]; //get rid of volatile for performance

    if(curTodo>0)
    {
      babystep(axis,/*fwd*/true);
      babystepsTodo[axis]--; //less to do next time
    }
    else
    if(curTodo<0)
    {
      babystep(axis,/*fwd*/false);
      babystepsTodo[axis]++; //less to do next time
    }
  }
#endif //BABYSTEPPING

#if defined(DEBUG) && defined(TP_ISR_PROFILE_PIN)
  WRITE(TP_ISR_PROFILE_PIN,0);
#endif
}

#ifdef PIDTEMP
// Apply the scale factors to the PID values
float scalePID_i(float i)
{
	return i*PID_dT;
}

float unscalePID_i(float i)
{
	return i/PID_dT;
}

float scalePID_d(float d)
{
    return d/PID_dT;
}

float unscalePID_d(float d)
{
	return d*PID_dT;
}

#endif //PIDTEMP
