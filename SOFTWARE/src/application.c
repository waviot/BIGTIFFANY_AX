#include <ax8052f143.h>
#include <libmftypes.h>
#include <libmfradio.h>
#include <libmfflash.h>
#include <libmfwtimer.h>
#include <libmfcrc.h>
#include "misc.h"
#include "defines.h"
#include "nbfi.h"
#include "application.h"
#include "hal.h"
#include "time.h"
#include "nbfi_config.h"
#include "water5.h"
#include "rf.h"


uint8_t __xdata stay_awake = 0;


int8_t pwr = 6;
uint8_t channel = 0;
uint8_t transmit_pending;
uint8_t it=0;
uint8_t  ina, inb;
uint8_t  oldina, oldinb;
uint8_t rotate_speed = 0;
uint8_t speed = 0;
uint8_t speed_counter = 0;
uint32_t serial;

uint8_t BUTT = 1;
uint8_t GoodCal;

uint8_t __xdata Voltage = 0xA0;


struct wtimer_desc __xdata everysecond_desc;
struct wtimer_desc __xdata every250ms_desc;
struct wtimer_desc __xdata checkhall_desc;

void EverySecond(struct wtimer_desc __xdata *desc);
void Every250ms(struct wtimer_desc __xdata *desc);
void CheckHALL(struct wtimer_desc __xdata *desc);

void GPIO_Init()
{
    #ifdef BIGTIFFANY_AX
    DIRA=0;
    PORTA=0;
    DIRB = 0xD7;
    PORTB &= 0xC0;
    DIRC = 0x1B;
    PORTC = 0x04;
    #endif // BIGTIFFANY_AX

    PORTR = 0x0B;
    DIRR = 0x15;

}

void GPIO_Deinit()
{
    #ifdef BIGTIFFANY_AX
    DIRA=0;
    PORTA=0;
    DIRB = 0xD7;
    PORTB &= 0xC0;
    DIRC = 0x1B;
    PORTC = 0x04;
    #endif // BIGTIFFANY_AX

}

void ColdSetup()
{

    GPIO_Init();

    #ifdef OSCCALONSTART
    RF_Init(OSC_CAL, 0, 0, 0);
    #endif

    do
    {
        uint16_t i = 0;
        LED = 1;
        delay_ms(500);
        GetVoltageOrTemp(1);
        delay_ms(500);
        LED = 0;
        if(++i > 60) break;   //wait no more than 1 min
    }
    #ifdef MAGNIUM_BATTERY
    while(0);
    #else
    while(Voltage < 0x94);
    #endif // MAGNIUM_BATTERY


    NBFI_Init(&RX_Callback);

    delay_ms(1);

    Water5Init(0);

    ScheduleTask(&everysecond_desc, EverySecond, RELATIVE, SECONDS(1));

    ScheduleTask(&every250ms_desc, Every250ms, RELATIVE, MILLISECONDS(250));

    ScheduleTask(&checkhall_desc, CheckHALL, RELATIVE, MILLISECONDS(250));

    WDTCFG |= 0x1D;

}

void WarmSetup()
{
    ax5043_commsleepexit();
    IE_4 = 1; // enable radio interrupt
}


void Loop()
{
    WDTRESET = 0xAE;

    if(rx_complete)
    {
        rx_complete = 0;
    }

    if(Water5Loop())
    {
        #ifndef MAGNIUM_BATTERY
        if(Voltage > 90) ////if Ubat>3V then PowerLevel Up
        {
            if(++PowLev > 3) PowLev = 3;
        }
        else if(Voltage < 25) { PowLev = 0;}
        else if(Voltage < 70)
        {
            if(--PowLev < 0) PowLev = 0;     //if Ubat<2.8V then PowerLevel down
        }
        #endif
        pwr = MAXPOWER-9+3*PowLev;

    }



}
/*
#define max(a,b) (a>b?a:b)
#define min(a,b) (a<b?a:b)

static uint16_t myabs(int16_t a)
{
    if(a > 0) return a;
    else return -a;
}
*/

#ifdef BIGTIFFANY_AX
void Every250ms(struct wtimer_desc __xdata *desc)
{
    static uint8_t  Old_IN1 = 0;
    static uint8_t  Old_IN2 = 0;
    uint8_t led_on = 0;

    ANALOGA = 0x3f; //port A is fully analog
    ANALOGCOMP = 0x30;
    if(W5_Tags.mode.hightreshold)
    {
        DIRA = 0x10;
        PORTA_4 = 1;
    }
    else
    {
        DIRA = 0x02;
        PORTA_1 = 1;
    }

    if(W5_Tags.mode.highcurrent)
    {
        PORTB |= 0x3C; //PB2, PB3, PB4 and PB5 = 1
        DIRB = 0xFF;
    }else    PORTB |= 0x14; //PB2 and PB4 = 1

    for(uint8_t i = 0; i < 20; i++);
    if(ANALOGCOMP&0x40)
    {
          if(Old_IN1 == 1)      {W5_Tags.PulseCounter0++; if(Calibrate) LED = 1; led_on = 1;}
          Old_IN1 = 0;

    }
    else    {Old_IN1 = 1; if(Calibrate) LED = 0;}

    if(ANALOGCOMP&0x80)
    {
          if(Old_IN2 == 1)
          {
              //#ifdef TWOCHANNELS
              if(W5_Tags.mode.twochannels)
              {
                W5_Tags.PulseCounter1++;
              }
              else
              {
                if(!W5_Tags.mode.no_add_2_ch) W5_Tags.PulseCounter0++;
              }
              if(Calibrate) LED = 1;
          }
          Old_IN2 = 0;

    }
    else    {Old_IN2 = 1;if(Calibrate&&!led_on) LED = 0;}


    if(W5_Tags.mode.hightreshold) PORTA_4 = 0;
    else PORTA_1 = 0;

    if(W5_Tags.mode.highcurrent)
    {
        PORTB &= 0xC3; //PB2, PB3, PB4 and PB5 = 0
        DIRB = 0xD7;
    } else PORTB &= 0xEB; //PB1 and PB4 = 0

    every250ms_desc.time += W5_Tags.mode.measure_rate;
    wtimer0_addabsolute(&every250ms_desc);

}
#endif


void CheckHALL(struct wtimer_desc __xdata *desc)
{
    static uint8_t runfaster_timer = 30;
    static uint8_t magnetic_read = 0;
    static uint8_t magn_period = 2;

    if(magnetic_read == 0)
    {
        HALL = 1;
        magnetic_read = 1;
        ScheduleTask(&checkhall_desc, CheckHALL, RELATIVE, MILLISECONDS(150));
        return;
    }
    if(magnetic_read == 1)
    {
        BUTT =  MAGN;
        HALL = 0;
        if(!BUTT)
        {
            magn_period = 2; runfaster_timer = 30;
            ScheduleTask(&checkhall_desc, CheckHALL, RELATIVE, MILLISECONDS(300));
        }

    }
    if(++magnetic_read >= magn_period) magnetic_read = 0;


    if(runfaster_timer)
    {
        runfaster_timer--;
    }
    else
    {
        magn_period = 10;
    }
}


void EverySecond(struct wtimer_desc __xdata *desc)
{
    everysecond_desc.time += SECONDS(1);
    wtimer0_addabsolute(&everysecond_desc);

    if(!CheckTask(&checkhall_desc)) CheckHALL(0);
    Water5OneSec();
}

void RX_Callback(uint8_t __generic* data, uint16_t length)
{
    #ifdef BIGTIFFANY_AX
    switch(data[0])
    {
        case 0xE0:
            switch(data[1])
            {
            case 0: //read flags
                data[2] = *(&W5_Tags.mode.config);
                length = 3;
                break;
            case 1: //read measure rate
                data[2] = W5_Tags.mode.measure_rate;
                length = 3;
                break;
            case 2: //read div
                data[2] = W5_Tags.mode.divider;
                length = 3;
                break;
            default:
                return;
            }
            NBFi_Send(data, length);
            break;
        case 0xE1:
            switch(data[1])
            {
            case 0: //write flags
                *((uint8_t*)(&W5_Tags.mode.config)) = data[2];
                break;
            case 1: //write measure rate
                W5_Tags.mode.measure_rate = data[2];
                break;
            case 2: //write div
                W5_Tags.mode.divider = data[2];
                break;
            default:
                return;
            }
            WriteAllParams();
            break;
    }
    #endif

}
