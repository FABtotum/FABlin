#include "SmartComm.h"

// Supported speeds
static uint32_t speeds[] = { 0, 300, 600, 1200, 2400, 4800, 9600, 14400, 19200, 28800, 31250, 38400, 57600, 115200 };
#define SPEEDS_N  ((sizeof speeds / sizeof speeds[0]) - 1)

SmartComm::SmartComm (SoftwareSerial& serial):
   Serial (serial),
   _twi_address (0),
   _twi_target  (0),
   _bus(0)
{}

void SmartComm::begin()
{
   // TODO: disable any possible thing interfering with pins (for known pins):
   // - oscillators
   // - TWI
   // - analog converters

   switch (_bus)
   {
      case SMARTCOMM_BUS_SERIAL:
         // Begin at exact baud rate (TODO: or start probe cycle)
         begin(_serial_baud);
         break;

      case SMARTCOMM_BUS_TWI:
         Wire.begin();
         break;
   }
}

void SmartComm::begin(uint16_t param)
{
   switch (_bus)
   {
      case SMARTCOMM_BUS_SERIAL:
         Serial.begin(param);
         break;

      case SMARTCOMM_BUS_TWI:
         _twi_address = (uint8_t)(param & 0x7F);
         Wire.begin(_twi_address);
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

bool SmartComm::isListening ()
{
   switch (_bus)
   {
      case SMARTCOMM_BUS_SERIAL: return Serial.isListening();

      case SMARTCOMM_BUS_TWI:
         if (IS_VALID_TWI_7BIT_ADDRESS(_twi_address)) {
            return true;
         } else {
            return IS_VALID_TWI_7BIT_ADDRESS(_twi_target);
         }

      default: return false;
   }
}

size_t SmartComm::write(uint8_t value)
{
   switch (_bus)
   {
      case SMARTCOMM_BUS_SERIAL:
         return Serial.write(value);

      case SMARTCOMM_BUS_TWI:
         if (IS_VALID_TWI_7BIT_ADDRESS(_twi_address)) {
            // We're slaves, humbly write a response to a request
            return Wire.write(value);
         } else {
            // We're masters
            if (IS_VALID_TWI_7BIT_ADDRESS(_twi_target)) {
               // with slaves
               Wire.beginTransmission(_twi_target);
               size_t written = Wire.write(value);
               if (Wire.endTransmission() <= 0)
                  return written;
               else
                  return 0;
            } else {
               // without slaves
            }
         }
   }
}

size_t SmartComm::write(const char* string)
{
   switch (_bus)
   {
      case SMARTCOMM_BUS_SERIAL:
         return Serial.write(string);

      case SMARTCOMM_BUS_TWI:
         if (IS_VALID_TWI_7BIT_ADDRESS(_twi_address)) {
            // We're slaves, humbly write a response to a request
            return Wire.write(string);
         } else {
            // We're masters
            if (IS_VALID_TWI_7BIT_ADDRESS(_twi_target)) {
               // with slaves
               Wire.beginTransmission(_twi_target);
               size_t written = Wire.write(string);
               if (Wire.endTransmission() <= 0)
                  return written;
               else
                  return 0;
            } else {
               // without slaves
            }
         }
   }
}

/*int SmartComm::available()
{
   switch (_bus)
   {
      case SMARTCOMM_BUS_SERIAL:
         return Serial.available();
      case SMARTCOMM_BUS_TWI:
         return Wire.available();
      default: return 0;
   }
}*/

/*int SmartComm::read()
{
   switch (_bus)
   {
      case SMARTCOMM_BUS_SERIAL: return Serial.read();

      case SMARTCOMM_BUS_TWI: return Wire.read();

      default: return -1;
   }
}*/

int SmartComm::peek()
{
   switch (_bus)
   {
      case SMARTCOMM_BUS_SERIAL: return Serial.peek();

      case SMARTCOMM_BUS_TWI: return Wire.peek();

      default:  return -1;
   }
}

void SmartComm::flush()
{
   switch (_bus)
   {
      case SMARTCOMM_BUS_SERIAL: Serial.flush();

      case SMARTCOMM_BUS_TWI: Wire.flush();
   }
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

      // Reinstantiate SoftwareSerial if pins changed
      Serial = SoftwareSerial(_serial_rx,_serial_tx);
   }

   serial(true);
}

/**
 * Enables TWI bus and initializes communication as master, or disables TWI bus altogether.
 *
 * @param bool enable - Enables (true) or disables (false) TWI bus
 */
void SmartComm::wire(bool enable)
{
   _bus = SMARTCOMM_BUS_TWI;
}

/**
 * Enables TWI bus as master and sets default slave address.
 *
 * @param uint8_t:7 addr - Slave address (only least significant 7 bits used)
 */
void SmartComm::wire (uint8_t addr)
{
   _twi_target = addr & 0x7F;
   wire(true);
}

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
