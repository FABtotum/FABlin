//#include <Arduino.h>
//#include <SoftwareSerial.h>

#include "SmartComm.h"

// Supported speeds
static uint32_t speeds[] = { 0, 300, 600, 1200, 2400, 4800, 9600, 14400, 19200, 28800, 31250, 38400, 57600, 115200 };
#define SPEEDS_N  ((sizeof speeds / sizeof speeds[0]) - 1)

SmartComm::SmartComm (SoftwareSerial& serial):
   Serial(serial) {}

void SmartComm::begin()
{
   // TODO: disable any possible thing interfering with pins (for known pins):
   // - oscillators
   // - TWI
   // - analog converters

   switch (_bus)
   {
      case SMARTCOMM_BUS_SERIAL:
         // Begin at exact baud rate or start probe cycle
         switch (_serial_baud)
         {
            default:
               Serial.begin(_serial_baud);
         }
         break;
   }
}

void SmartComm::end()
{
   Serial.flush();
   Serial.end();

   // For good measure?
   TWCR &= ~_BV(TWEN);
}

void SmartComm::serial (bool enable)
{
   if (enable) {
      _bus = SMARTCOMM_BUS_SERIAL;
   } else {
      end();
      _bus &= ~SMARTCOMM_BUS_SERIAL;
   }
}

void SmartComm::serial (uint8_t sRX, uint8_t sTX, uint32_t baud)
{
   if (sRX != _serial_rx || sTX != _serial_tx || baud != _serial_baud) {
      end();
      _serial_baud = baud;
   }
   
   if (sRX != _serial_rx || sTX != _serial_tx)
   {
      _serial_rx = sRX;
      _serial_tx = sTX;

      // Reinstantiate SoftwareSerial if pin changed
      Serial = SoftwareSerial(_serial_rx,_serial_tx);
   }

   serial(true);
}

/**
 * Enables TWI bus and initializes communication as master, or disables TWI bus altogether.
 *
 * @param bool enable - Enables (true) or disables (false) TWI bus
 */
/*void Smart_s::wire(bool enable)
{
   if (enable) {
      Wire.begin();
   } else {
      TWCR &= ~_BV(TWEN);
   }
}*/

/**
 * Enables TWI bus and initializes communication as slave.
 *
 * @param uint8_t:7 addr - Slave address (only least significant 7 bits used)
 */
/*void Smart_s::wire (uint8_t addr)
{
   Wire.begin(addr & 0x7F);
}*/

inline boolean _probe_serial (SoftwareSerial& serial, uint32_t speed, const char *probe, const char *resp)
{
   serial.flush();
   serial.write(probe);

   uint8_t i = 0;
   for (uint8_t t = 250; t > 0; t--)
   {
      if (!serial.available())
      {
         delay(4);
      }
      else
      {
         while (serial.available() && i < 255)
         {
            char r = serial.read();
            if (r != resp[i++]) {
               return false;
            }
            if (resp[i] == 0) {
               return true;
            }
         }
      }
   }

   return false;
}

uint32_t SmartComm::probe (const char *probe, const char *resp, bool asc)
{
   static uint8_t p_speed = 0;

   for (
      p_speed = p_speed? p_speed : (sizeof (speeds) / sizeof (speeds[0]));
      p_speed > 0;
      p_speed--
   ) {
      Serial.begin(speeds[p_speed]);
      //Serial.flush();

      boolean ok = _probe_serial(Serial, speeds[p_speed], probe, resp);

      if (ok) {
         return speeds[p_speed];
      } else {
         Serial.end();
      }
   }

   return speeds[p_speed];
}
