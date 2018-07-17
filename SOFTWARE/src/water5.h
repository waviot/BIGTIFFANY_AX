#ifndef WATER5_H_INCLUDED
#define WATER5_H_INCLUDED
#include "defines.h"


#ifdef MAGNIUM_BATTERY
#define HARDWARE_REV 03
#else
#define HARDWARE_REV 02
#endif

#define MAXPOWER    15

#define EDATAPTR        0xf800
#define SERIALPTR       0xfd00
#define KEYPTR          0xfd04

#define SENDEVERYONEHOUR

#ifdef BIGTIFFANY_AX
    #define HALL   PORTC_0
    #define LED     PORTB_0
    #define MAGN  PINC_2

        #ifdef SENDEVERYONEHOUR
            #define HIGHCURRENT
            //#define TWOCHANNELS
            #define SOFTWARE_REV 44
            #define MEASURE_RATE    32  //times per second
            #define DIV 1
        #else
            #define TWOCHANNELS
            #define SOFTWARE_REV 42
            #define MEASURE_RATE   4   //times per second
            #define DIV 1
        #endif
#endif // BIGTIFFANY_AX

void Water5OneSec();
void Water5Init(uint8_t);
bool Water5Loop();
void Water5ExecComm(uint16_t comm);
void ClearAllParams();
void WriteAllParams();

typedef struct _W5_MODE_CONFIG
{
     union
    {
        struct
        {
            uint8_t twochannels         : 1;
            uint8_t sendeveryonehour    : 1;
            uint8_t highcurrent         : 1;
            uint8_t hightreshold        : 1;
            uint8_t no_add_2_ch         : 1;
            uint8_t reserved_flags      : 3;
            uint8_t measure_rate        : 8;
            uint8_t divider             : 8;
            uint8_t reserved            : 8;
        };
        uint32_t config;
    };

} W5_MODE_CONFIG;

typedef struct _W5_TAGS_TO_SAVE
{
  uint32_t PulseCounter0;
  uint32_t PulseCounter1;
  uint16_t MesNum;
  uint8_t Protocol;
  uint8_t poehali;
  uint8_t initcal;
  uint8_t ADCLOW;
  uint8_t ADCHIGH;
  uint16_t ADCVDD;
  W5_MODE_CONFIG mode;
  uint8_t reservedMas[9];
} W5_TAGS_TO_SAVE;


extern __xdata  W5_TAGS_TO_SAVE W5_Tags;

extern int8_t  __xdata PowLev;
extern uint8_t  __xdata Calibrate;
extern uint16_t  testptr;
extern uint8_t   measure;
extern uint8_t __xdata SendBuf[8];
extern uint8_t __xdata SendFlag;

#endif // WATER5_H_INCLUDED
