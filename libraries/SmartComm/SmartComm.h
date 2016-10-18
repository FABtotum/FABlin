#ifndef _SMART_COMM_H_
#define _SMART_COMM_H_

/**
 * SmartComm
 *
 * Manage dynamically reconfigurable communications over serial and twi bus..
 */

#include <Arduino.h>
#include <SoftwareSerial.h>

#define SMARTCOMM_BUS_SERIAL 1
#define SMARTCOMM_BUS_TWI    2

class SmartComm
{
   public:

      SmartComm (SoftwareSerial&);

      // Set serial mode
      void serial (bool=true);
      void serial (uint8_t, uint8_t, uint32_t=0xFF);

      SoftwareSerial& Serial;

      // Set two-wire mode
      /*void wire (boolean=true);
      void wire (uint8_t);*/

      void begin ();
      void end   ();

   private:

      uint8_t _active:1;
      uint8_t _bus:1;      // Communication bus (serial / twi)

      uint8_t _serial_rx, _serial_tx;  // Serial pins used
      uint16_t _serial_baud;    // Serial baud rate

      uint32_t probe (const char* = "\x05", const char* = "\x06", bool = false);

};

#endif  // _SMART_COMM_H_
