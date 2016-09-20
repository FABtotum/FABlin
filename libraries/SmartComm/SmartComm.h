#ifndef _SMART_COMM_H_
#define _SMART_COMM_H_

/**
 * SmartComm
 *
 * Manage dynamically reconfigurable communications over serial and twi bus..
 */

#include <Arduino.h>
#include <SoftwareSerial.h>

class SmartComm
{
   public:

      SmartComm (SoftwareSerial&);

      // Set serial mode
      void serial (bool=true);
      void serial (uint8_t, uint8_t, uint32_t=0xFF);

      // Set two-wire mode
      /*void wire (boolean=true);
      void wire (uint8_t);*/

      void begin (uint8_t=0xFF);
      void end   ();

   private:

      SoftwareSerial& _Serial;
      uint8_t _rx, _tx;  // Serial pins used

      uint32_t probe (const char* = "\x05", const char* = "\x06", bool = false);

};

#endif  // _SMART_COMM_H_
