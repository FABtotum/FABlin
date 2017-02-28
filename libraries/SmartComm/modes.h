#ifndef _SMART_COMM_MODES_H_
#define _SMART_COMM_MODES_H_

#if defined(__AVR_ATmega1280__) || defined(ARDUINO_AVR_MEGA)

   // Default SmartComm pins for TOTUMduino (ATmega1280)
   #ifndef SMART_COMM_SDA_THERM
      #define SMART_COMM_SDA_THERM 67,20,0
   #endif
   #ifndef SMART_COMM_SDA_PWM
      #define SMART_COMM_SDA_PWM   11,20,0
   #endif

   #ifndef SMART_COMM_SCL_THERM
      #define SMART_COMM_SCL_THERM 67,21,0
   #endif
   #ifndef SMART_COMM_SCL_THERM
      #define SMART_COMM_SCL_PWM   11,21,0
   #endif

   #ifndef SMART_COMM_THERM_PWM
      #define SMART_COMM_THERM_PWM     11,67,0
   #endif
   #ifndef SMART_COMM_THERM_PWM_TWI
      #define SMART_COMM_THERM_PWM_TWI 11,67,1
   #endif

   #ifndef SMART_COMM_PWM_THERM
      #define SMART_COMM_PWM_THERM     67,11,0
   #endif
   #ifndef SMART_COMM_PWM_THERM_TWI
      #define SMART_COMM_PWM_THERM_TWI 67,11,1
   #endif

#elif defined(__AVR_ATmega328P__) || defined(ARDUINO_AVR_NANO)

   // Default SmartComm pins for TOTUMduino (ATmega1280)
   #ifndef SMART_COMM_SDA_THERM
      #define SMART_COMM_SDA_THERM SDA,0,0
   #endif
   #ifndef SMART_COMM_SDA_PWM
      #define SMART_COMM_SDA_PWM   SDA,1,0
   #endif

   #ifndef SMART_COMM_SCL_THERM
      #define SMART_COMM_SCL_THERM SCL,0,0
   #endif
   #ifndef SMART_COMM_SCL_THERM
      #define SMART_COMM_SCL_PWM   SCL,1,0
   #endif

   #ifndef SMART_COMM_THERM_PWM
      #define SMART_COMM_THERM_PWM     0,1,0
   #endif
   #ifndef SMART_COMM_THERM_PWM_TWI
      #define SMART_COMM_THERM_PWM_TWI 0,1,1
   #endif

   #ifndef SMART_COMM_PWM_THERM
      #define SMART_COMM_PWM_THERM     1,0,0
   #endif
   #ifndef SMART_COMM_PWM_THERM_TWI
      #define SMART_COMM_PWM_THERM_TWI 1,0,1
   #endif

#endif

#define SMART_COMM_DEFAULT SMART_COMM_SDA_PWM

#endif // define _SMART_COMM_MODES_H_
