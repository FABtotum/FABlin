#ifndef _SMART_COMM_H_
#define _SMART_COMM_H_

/**
 * SmartComm
 *
 * Manage dynamically reconfigurable communications over serial and twi bus..
 */

#include <Arduino.h>
#include <SoftwareSerial.h>
#include <Wire.h>

#define SMARTCOMM_BUS_SERIAL 1
#define SMARTCOMM_BUS_TWI    2

#define IS_VALID_TWI_7BIT_ADDRESS(addr) (addr > B111 && addr < B1111000)

/*#define _SMARTCOMM_BUS_MIXIN(type, name) inline type name (void) { \
      switch (_bus) { \
            case SMARTCOMM_BUS_SERIAL: return Serial.name(); \
            case SMARTCOMM_BUS_TWI:    return Wire.name(); \
      }}*/

class SmartComm : public Stream
{
   public:

      SmartComm (SoftwareSerial&);

      // Set serial mode
      void serial (bool=true);
      void serial (uint8_t, uint8_t, uint32_t=0xFF);

      // Alias to attached serial
      SoftwareSerial& Serial;

      // Set two-wire mode
      void wire (bool=true);
      void wire (uint8_t);

      // No need to define alias to singleton Wire

      void begin ();
      void begin (uint16_t);
      void end   ();

      bool isListening (void);

      size_t write (uint8_t);
      size_t write (const char*);

      inline int available (void) { switch (_bus) {
            case SMARTCOMM_BUS_SERIAL: return Serial.available();
            case SMARTCOMM_BUS_TWI: return Wire.available();
            default: return 0;
      }}
      inline int read (void) { switch (_bus) {
            case SMARTCOMM_BUS_SERIAL: return Serial.read();
            case SMARTCOMM_BUS_TWI: return Wire.read();
            default: return -1;
      }}
      int peek (void);
      void flush (void);

   private:

      uint8_t _active:1;
      uint8_t _bus:2;      // Communication bus (serial / twi)

      uint8_t _serial_rx = 0, _serial_tx = 0;  // Serial pins used
      uint16_t _serial_baud;    // Serial baud rate

      uint8_t _twi_address;
      uint8_t _twi_target;

      uint32_t probe (const char* = "\x05", const char* = "\x06", bool = false);

};

#endif  // _SMART_COMM_H_
