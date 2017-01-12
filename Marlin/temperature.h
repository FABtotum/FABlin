/*
  temperature.h - temperature controller
  Part of Marlin

  Copyright (c) 2011 Erik van der Zalm

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef temperature_h
#define temperature_h 

#include "Marlin.h"
#include "planner.h"
#ifdef PID_ADD_EXTRUSION_RATE
  #include "stepper.h"
#endif

// public functions
void tp_init();  //initialize the heating
void manage_heater(); //it is critical that this is called periodically.

/*void write_LaserSoftPwm_t(unsigned int LaserSoftPwm_value);
void write_HeadLightSoftPwm_t(unsigned int HeadLightSoftPwm_value);
void write_RedSoftPwm_t(unsigned int RedSoftPwm_value);
void write_GreenSoftPwm_t(unsigned int GreenSoftPwm_value);
void write_BlueSoftPwm_t(unsigned int BlueSoftPwm_value);
void write_red_fading(bool red_fading_value);
void write_green_fading(bool green_fading_value);
void write_blue_fading(bool blue_fading_value);

void write_fading_speed(unsigned int fading_speed_value);

bool red_fading_read();
bool green_fading_read();
bool blue_fading_read();*/



// low level conversion routines
// do not use these routines and variables outside of temperature.cpp
extern int target_temperature[EXTRUDERS];  
extern float current_temperature[EXTRUDERS];
#ifdef SHOW_TEMP_ADC_VALUES
  extern int current_temperature_raw[EXTRUDERS];
  extern int current_temperature_bed_raw;
#endif
extern int current_pressure_raw_value;
extern int current_mon_main_supply_raw_value;
extern int current_mon_sec_supply_raw_value;
extern int current_main_current_raw_value;

extern float current_pressure_value;
extern float current_mon_24v_value;
extern float current_mon_5v_value;
extern float current_main_current_value;

extern int target_temperature_bed;
extern float current_temperature_bed;
#ifdef TEMP_SENSOR_1_AS_REDUNDANT
  extern float redundant_temperature;
#endif

#if defined(CONTROLLERFAN_PIN) && CONTROLLERFAN_PIN > -1
  extern unsigned char soft_pwm_bed;
#endif

#ifdef PIDTEMP
  extern float Kp,Ki,Kd,Kc;
  float scalePID_i(float i);
  float scalePID_d(float d);
  float unscalePID_i(float i);
  float unscalePID_d(float d);

#endif
#ifdef PIDTEMPBED
  extern float bedKp,bedKi,bedKd;
#endif
  
  
#ifdef BABYSTEPPING
  extern volatile int babystepsTodo[3];
#endif
  
//high level conversion routines, for use outside of temperature.cpp
//inline so that there is no performance decrease.
//deg=degreeCelsius

FORCE_INLINE float degHotend(uint8_t extruder) {  
  return current_temperature[extruder];
};

#ifdef SHOW_TEMP_ADC_VALUES
  FORCE_INLINE float rawHotendTemp(uint8_t extruder) {  
    return current_temperature_raw[extruder];
  };

  FORCE_INLINE float rawBedTemp() {  
    return current_temperature_bed_raw;
  };
#endif

FORCE_INLINE float degBed() {
  return current_temperature_bed;
};

FORCE_INLINE float degTargetHotend(uint8_t extruder) {  
  return target_temperature[extruder];
};

FORCE_INLINE float degTargetBed() {   
  return target_temperature_bed;
};

FORCE_INLINE void setTargetHotend(const float &celsius, uint8_t extruder) {  
  target_temperature[extruder] = celsius;
};

FORCE_INLINE void setTargetBed(const float &celsius) {  
  target_temperature_bed = celsius;
};

FORCE_INLINE bool isHeatingHotend(uint8_t extruder){  
  return target_temperature[extruder] > current_temperature[extruder];
};

FORCE_INLINE bool isHeatingBed() {
  return target_temperature_bed > current_temperature_bed;
};

FORCE_INLINE bool isCoolingHotend(uint8_t extruder) {  
  return target_temperature[extruder] < current_temperature[extruder];
};

FORCE_INLINE bool isCoolingBed() {
  return target_temperature_bed < current_temperature_bed;
};

FORCE_INLINE float rawPressure() {
  return current_pressure_raw_value;
};

FORCE_INLINE float rawMon24V() {
  return current_mon_main_supply_raw_value;
};

FORCE_INLINE float rawMon5V() {
  return current_mon_sec_supply_raw_value;
};

FORCE_INLINE float rawMainCurrent() {
  return current_main_current_raw_value;
};

FORCE_INLINE float Pressure() {
  return current_pressure_value;
};

FORCE_INLINE float Mon24V() {
  return current_mon_24v_value;
};

FORCE_INLINE float Mon5V() {
  return current_mon_5v_value;
};

FORCE_INLINE float MainCurrent() {
  return current_main_current_value;
};

#define degHotend0() degHotend(0)
#define degTargetHotend0() degTargetHotend(0)
#define setTargetHotend0(_celsius) setTargetHotend((_celsius), 0)
#define isHeatingHotend0() isHeatingHotend(0)
#define isCoolingHotend0() isCoolingHotend(0)
#if EXTRUDERS > 1
#define degHotend1() degHotend(1)
#define degTargetHotend1() degTargetHotend(1)
#define setTargetHotend1(_celsius) setTargetHotend((_celsius), 1)
#define isHeatingHotend1() isHeatingHotend(1)
#define isCoolingHotend1() isCoolingHotend(1)
#else
#define setTargetHotend1(_celsius) do{}while(0)
#endif
#if EXTRUDERS > 2
#define degHotend2() degHotend(2)
#define degTargetHotend2() degTargetHotend(2)
#define setTargetHotend2(_celsius) setTargetHotend((_celsius), 2)
#define isHeatingHotend2() isHeatingHotend(2)
#define isCoolingHotend2() isCoolingHotend(2)
#else
#define setTargetHotend2(_celsius) do{}while(0)
#endif
#if EXTRUDERS > 3
#error Invalid number of extruders
#endif

#define LSB_VALUE  0.00488
#define DIV_24V    11
#define DIV_5V     2
#define DIV_CURR   3.106

//#define MAX_PWM    127

int getHeaterPower(int heater);
void disable_heater();
void setWatch();
void updatePID();

FORCE_INLINE void autotempShutdown(){
 #ifdef AUTOTEMP
 if(autotemp_enabled)
 {
  autotemp_enabled=false;
  if(degTargetHotend(active_extruder)>autotemp_min)
    setTargetHotend(0,active_extruder);
 }
 #endif
}

void PID_autotune(float temp, int extruder, int ncycles);

#endif

