#ifndef _SMARTCOMM_H_
#define _SMARTCOMM_H_

/**
 * SmartComm
 *
 * Manage dynamically reconfigurable communications over serial and twi bus
 * to the heads.
 */

#include <Arduino.h>
#include <SoftwareSerial.h>

#include "modes.h"

typedef struct Smart_s
{
   #if defined(MOTHERBOARD) && (MOTHERBOARD == 25)
   //#if defined(__AVR_ATmega1280__) || defined(ARDUINO_AVR_MEGA)
      // SmartComm inside TOTUMduino can be instantiated with default values
      // and reconfigured later
      Smart_s (uint8_t=11, uint8_t=SDA);
   #else
      // SmartComm on other hardwares must be instantiated once for all
      Smart_s (uint8_t, uint8_t);
   #endif

   SoftwareSerial Serial;

   // Set serial mode
   void serial (bool=true);
   void serial (uint8_t, uint8_t, uint32_t=0xFF);

   // Set two-wire mode
   /*void wire (boolean=true);
   void wire (uint8_t);*/

   //void begin (uint8_t=0xFF);
   void end   ();

   private:
      uint8_t _rx, _tx;  // Serial pins used

      uint32_t probe (const char* = "\x05", const char* = "\x06", bool = false);

} SmartComm;

#endif  // _SMARTCOMM_H_
