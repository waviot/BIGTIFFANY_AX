
#include <ax8052f143.h>
#include <libmftypes.h>
#include <libmfwtimer.h>
#include <libmfflash.h>
#include <stdlib.h>
#include "misc.h"

#include "application.h"
#include "water5.h"
#include "nbfi.h"
#include "nbfi_config.h"
#include "hal.h"


__xdata W5_TAGS_TO_SAVE  W5_Tags;


uint8_t __xdata SendBuf[8];
uint8_t __xdata LaterSendBuf[8];
uint8_t   LaterTimer = 0;

uint8_t  SendFlag;

uint8_t __xdata Seconds = 0;
uint8_t __xdata Minutes = 0;
uint8_t __xdata Hours = 0;
uint8_t __xdata Days = 0;
uint8_t __xdata Days4month = 0;

//uint8_t __xdata NewMinute = 0;

uint16_t __xdata HourMas0[24], HourMas1[24];
uint16_t __xdata DayMas0[7], DayMas1[7];

uint8_t __xdata sendweekflag = 0;
uint8_t __xdata sendmonthflag = 0;

uint32_t  __xdata PC_PrevHour0, PC_PrevHour1;
uint32_t  __xdata PC_PrevDay0, PC_PrevDay1;
uint32_t  __xdata PC_PrevMinute0, PC_PrevMinute1;

uint8_t   __xdata MaxFlow0 = 0;
uint8_t   __xdata MaxFlow1 = 0;

uint8_t  __xdata Calibrate;

uint16_t  __xdata Switch_i;


#ifdef MAGNIUM_BATTERY
int8_t  __xdata PowLev = 3;
#else
int8_t  __xdata PowLev = 0;
#endif // MAGNIUM_BATTERY


uint32_t __xdata DaysAfterDepas = 59;

uint8_t depassivation_active = 0;


uint8_t waitsomeminutes = 0;

uint16_t testptr = 0;
uint8_t measure = 0;


void mem_set(uint8_t* ptr, uint8_t data, uint16_t len) {while(len--) *ptr++ = data;}

uint8_t log10_DIV()
{
    switch(W5_Tags.mode.divider)
    {
    case 1:
      return    1<<6;
    case 10:
      return    2<<6;
    case 100:
      return    3<<6;
    }
    return 0;
}

void ReadAllParams()
{
    uint16_t __xdata *ptr = (uint16_t __xdata *)&W5_Tags;

    for(int i = 0; i < sizeof(W5_TAGS_TO_SAVE); i+=2)
    {
        *ptr++ = flash_read(EDATAPTR + i);
    }

}

void WriteAllParams()
{

    uint16_t __xdata *ptr = (uint16_t __xdata *)&W5_Tags;
    flash_unlock();
    flash_pageerase(EDATAPTR);

    for(int i = 0; i < sizeof(W5_TAGS_TO_SAVE); i += 2)
    {
        flash_write(EDATAPTR + i,*ptr++);
    }

    flash_lock();

}

void ClearAllParams()
{
  uint8_t __xdata *ptr = (uint8_t __xdata *)&W5_Tags;
  for(int i = 0; i < sizeof(W5_TAGS_TO_SAVE); i++) *ptr++ = 0;
  W5_Tags.mode.divider = DIV;
  W5_Tags.mode.measure_rate = MILLISECONDS(100)*10/MEASURE_RATE;

  #ifdef TWOCHANNELS
  W5_Tags.mode.twochannels = 1;
  #else
  W5_Tags.mode.twochannels = 0;
  #endif

  #ifdef HIGHCURRENT
  W5_Tags.mode.highcurrent = 1;
  #else
  W5_Tags.mode.highcurrent = 0;
  #endif

  #ifdef HIGHTRESHOLD
  W5_Tags.mode.hightreshold = 1;
  #else
  W5_Tags.mode.hightreshold = 0;
  #endif

  #ifdef SENDEVERYONEHOUR
  W5_Tags.mode.sendeveryonehour = 1;
  #else
  W5_Tags.mode.sendeveryonehour = 0;
  #endif

  #ifdef NO_ADD_2_CH
  W5_Tags.mode.no_add_2_ch = 1;
  #else
  W5_Tags.mode.no_add_2_ch = 0;
  #endif



  #ifdef NEPTUN
  W5_Tags.ADCLOW = DEFAULT_ADCLOW;
  W5_Tags.ADCHIGH = DEFAULT_ADCHIGH;
  W5_Tags.ADCVDD = DEFAULT_ADCVDD;
  #endif
  WriteAllParams();
}


void Water5Init(uint8_t clear)
{

    ReadAllParams();

    if(clear||(W5_Tags.Protocol == 0xff))  ClearAllParams();

    testptr = 0;

    for(int i = 0; i < 24; i++) HourMas0[i] = HourMas1[i] = 0;
    for(int i = 0; i < 7; i++) DayMas0[i] = DayMas1[i] = 0;

    PC_PrevMinute0 = PC_PrevHour0 = PC_PrevDay0 = W5_Tags.PulseCounter0;
    PC_PrevMinute1 = PC_PrevHour1 = PC_PrevDay1 = W5_Tags.PulseCounter1;

    *((uint32_t __xdata*)(&SendBuf[1])) = W5_Tags.PulseCounter0;
    *((uint32_t __xdata*)(&LaterSendBuf[1])) = W5_Tags.PulseCounter1;

    SendBuf[0] = 0x41;
    LaterSendBuf[0] = 0x49;

    SendBuf[5] = LaterSendBuf[5] = HARDWARE_REV;
    SendBuf[6] = LaterSendBuf[6] = log10_DIV() + SOFTWARE_REV;//SOFTWARE_REV;
    SendBuf[7] = LaterSendBuf[7] = CompVersion();

    SendFlag = 3;

    //nbfi.handshake_mode = HANDSHAKE_NONE;
    //NBFi_Send(SendBuf, 8);
    //nbfi.handshake_mode = HANDSHAKE_SIMPLE;
}


void PCSendDayData(uint8_t chan, uint8_t later)
{

    uint8_t i;
    uint32_t Hours2BitsLo, Hours2BitsHi;
    uint16_t maxHour;

    uint16_t __xdata* HourMas;
    uint8_t __xdata*  buf;


    Hours2BitsLo = 0;
    Hours2BitsHi = 0;

    maxHour = 1;

    EA = 0;
    WriteAllParams();
    EA = 1;

    HourMas = chan?HourMas1:HourMas0;
    buf =  later?LaterSendBuf:SendBuf;

    for(i=0;i!=24;i++) if(HourMas[i] > maxHour) maxHour = HourMas[i];

    for(i=0;i!=24;i++)
    {
        if(HourMas[i])
        {
            if(i<16) Hours2BitsLo+=(((uint32_t)(HourMas[i])*3*99/maxHour/100)+1)<<(i*2);
            else Hours2BitsHi+=(((uint32_t)(HourMas[i])*3*99/maxHour/100)+1)<<((i-16)*2);
        }

    }
    *((uint16_t __xdata*)(&buf[0])) = ((((chan?W5_Tags.PulseCounter1:W5_Tags.PulseCounter0)/W5_Tags.mode.divider)&0xFFFF)<<1)&0xfffe;
    *((uint32_t __xdata*)(&buf[2])) = Hours2BitsLo;
    *((uint16_t __xdata*)(&buf[6])) = Hours2BitsHi&0xFFFF;

    if(later)
    {
       LaterTimer = later&0x7F;
       if(later&0x80) SendFlag |= 2;
    }
    else SendFlag |= 1;

}


void PCSendWeekData(uint8_t chan, uint8_t later)
{

    uint8_t i;

    uint16_t   maxDay;
    uint32_t   DayBits;
    uint16_t __xdata * DayMas;
    uint8_t __xdata*  buf;

    DayMas = chan?DayMas1:DayMas0;
    buf =  later?LaterSendBuf:SendBuf;

    *((uint32_t __xdata*)(&buf[1])) = chan?W5_Tags.PulseCounter1:W5_Tags.PulseCounter0;


    buf[7] = Supply_Voltage;

    DayBits = 0;
    maxDay=1;
    for(i=0;i!=7;i++) if(DayMas[i] > maxDay) maxDay = DayMas[i];
    for(i=0;i!=7;i++) DayBits+=(((uint32_t)(DayMas[i])*8*99/maxDay/100))<<(i*3);

    DayBits<<=3;
    DayBits|=buf[4]&0x7;
    *((uint16_t __xdata*)(&buf[4])) = DayBits&0xFFFF;
    buf[6] = DayBits>>16;
    buf[0] = chan?0x7B:0x73;

    if(later)
    {
       LaterTimer = later&0x7F;
       if(later&0x80) SendFlag |= 2;
    }
    else SendFlag |= 1;
}

void PCSendExtInfo(uint8_t chan, uint8_t later)
{

     uint8_t __xdata*  buf;

     buf =  later?LaterSendBuf:SendBuf;

     buf[0] = chan?0x89:0x81;;
     buf[1] = HARDWARE_REV;
     buf[2] = log10_DIV() + SOFTWARE_REV;//SOFTWARE_REV;
     buf[3] = CompVersion();
     int8_t t = adc_measure_temperature() >> 8;
     buf[4] = t;
     buf[5] = Supply_Voltage;
     buf[6] = pwr;
     buf[7] = chan?MaxFlow1:MaxFlow0;

     if(later)
     {
       LaterTimer = later&0x7F;
       if(later&0x80) SendFlag |= 2;
     }
     else SendFlag |= 1;
}

void PCSendInfoData(uint8_t chan, uint8_t later)
{
     static uint8_t sendextinfo_timer = 5;

     uint8_t __xdata*  buf;
     if(!chan) sendextinfo_timer++;
     if(!(sendextinfo_timer%6))
     {
        PCSendExtInfo(chan, later);
        return;
     }

     buf =  later?LaterSendBuf:SendBuf;

     *((uint32_t __xdata*)(&buf[1])) = chan?W5_Tags.PulseCounter1:W5_Tags.PulseCounter0;
     *((uint16_t __xdata*)(&buf[5])) = nbfi_state.UL_total&0xffff;//W5_Tags.MesNum;
     //SendBuf[5] = LaterSendBuf[5] = LPOSCFREQ1;
     //SendBuf[6] = LaterSendBuf[6] = LPOSCFREQ0;

     buf[0] = chan?0x69:0x61;
     buf[7] = PowLev;
     buf[7] <<= 6;
     buf[7] += (chan?MaxFlow1:MaxFlow0)&0x3F;
     if(later)
     {
       LaterTimer = later&0x7F;
       if(later&0x80) SendFlag |= 2;
     }
     else SendFlag |= 1;
}


void PCSendComm(uint8_t comm, uint8_t chan, uint8_t later)
{
     uint8_t __xdata*  buf;

     buf =  later?LaterSendBuf:SendBuf;

     *((uint32_t __xdata*)(&buf[1])) =  chan?W5_Tags.PulseCounter1:W5_Tags.PulseCounter0;
     buf[5] = comm;
     int8_t t = adc_measure_temperature() >> 8;
     buf[6] = t;
     buf[7] = Supply_Voltage;
     buf[0] = chan?0x59:0x51;
     if(later)
     {
       LaterTimer = later&0x7F;
       if(later&0x80) SendFlag |= 2;
     }
     else SendFlag |= 1;
}


void PCSendHourData(uint8_t chan, uint8_t later)
{
     uint8_t*  buf;
     static  uint32_t iterator = 0;
     buf =  later?LaterSendBuf:SendBuf;

     *((uint32_t*)(&buf[1])) = chan?W5_Tags.PulseCounter1:W5_Tags.PulseCounter0;
     buf[0] = chan?0x99:0x91;
     if(chan == 0) iterator++;
     buf[5] = (iterator>>16)&0xff;
     buf[6] = (iterator>>8)&0xff;
     buf[7] = (iterator>>0)&0xff;

     if(later)
     {
       LaterTimer = later&0x7F;
       if(later&0x80) SendFlag |= 2;
     }
     else
     {
         SendFlag |= 1;
     }
}

void PCSendDepass()
{

    SendBuf[0] = 0xE3;
    SendBuf[1] = HARDWARE_REV;
    SendBuf[2] = log10_DIV() + SOFTWARE_REV;//SOFTWARE_REV;
    SendBuf[3] = CompVersion();
    SendBuf[4] = GetVoltageOrTemp(0);
    SendBuf[5] = Voltage;
    SendBuf[6] = pwr;
    SendBuf[7] = MaxFlow0;
    SendFlag |= 1;
}

struct wtimer_desc __xdata sendlater_desc;

void Water5SendLater(struct wtimer_desc __xdata *desc)
{
    NBFi_Send(LaterSendBuf, 8);
    W5_Tags.MesNum++;
}

bool Water5Loop()
{
    //bool wassent = 0;
    if(SendFlag&1)
    {
        SendFlag &= 0xfe;
        NBFi_Send(SendBuf, 8);
        W5_Tags.MesNum++;
        return 1;

    }
    if(SendFlag&2)
    {
        SendFlag &= 0xfd;
        if(W5_Tags.mode.twochannels)
        {
            ScheduleTask(&sendlater_desc, &Water5SendLater, RELATIVE, SECONDS(20));
        }

    }
    return 0;
}

void Water5ExecComm(uint16_t comm)
{
    switch(comm)
    {


#ifndef KARAT
            case 5: //cal
            case 33: //init cal
                #ifndef NO_SEND_TEN_INFOS
                Calibrate = 10;
                #endif
                PC_PrevMinute0 = PC_PrevHour0 = PC_PrevDay0 = W5_Tags.PulseCounter0;
                PC_PrevMinute1 = PC_PrevHour1 = PC_PrevDay1 = W5_Tags.PulseCounter1;
                Seconds = 0;//random() % 60;;
                Minutes = 0;//random() % 60;;
                Hours = 0;//12 + random() % 11;
                MaxFlow0 = MaxFlow1 = 0;
                for(int i = 0; i < 24; i++) HourMas0[i] = HourMas1[i] = 0;
                for(int i = 0; i < 7; i++) DayMas0[i] = DayMas1[i] = 0;
                EA = 0;
                WriteAllParams();
                EA = 1;
                PCSendComm(comm,0,0);
                PCSendComm(comm,1,0x80);
                break;
#else
             case 5:
             case 33: //init cal
                PC_PrevMinute0 = PC_PrevHour0 = PC_PrevDay0 = W5_Tags.PulseCounter0;
                PC_PrevMinute1 = PC_PrevHour1 = PC_PrevDay1 = W5_Tags.PulseCounter1;
                Seconds = 0;//random() % 60;;
                Minutes = 0;//random() % 60;;
                Hours = 0;//12 + random() % 11;
                MaxFlow0 = MaxFlow1 = 0;
                for(int i = 0; i < 24; i++) HourMas0[i] = HourMas1[i] = 0;
                for(int i = 0; i < 7; i++) DayMas0[i] = DayMas1[i] = 0;
                EA = 0;
                WriteAllParams();
                EA = 1;
                PCSendComm(comm,0,0);
                PCSendComm(comm,1,0x80);
                K213_Send_For_Serial();
                break;
#endif
#ifdef NEPTUN
            case 7://15:
                measure = 2;
                break;
#endif
#ifndef KARAT
            case 30:
                EA = 0;
                WriteAllParams();
                EA = 1;
                PCSendComm(30,0,0);
                waitsomeminutes = 120;
                break;
            case 10:
                if(W5_Tags.mode.twochannels)
                {
                    if(W5_Tags.Protocol)
                    {
                        PCSendDayData(0,0);
                        PCSendDayData(1,0x80);
                    }
                    else
                    {
                        PCSendDayData(0,0x80*((NBFi_Get_TX_Iter()%2) != 0));
                        PCSendDayData(1,0x80*((NBFi_Get_TX_Iter()%2) == 0));
                    }
                }
                else PCSendDayData(0,0);
                break;
            case 41:
                PCSendWeekData(0,0);
                break;
#endif
            case 0:
                break;

            default:
                #ifndef NETWORK_TESTER
                #ifndef NO_SEND_TEN_INFOS
                Calibrate = 10;
                #endif
                #endif
                #ifdef KARAT
                K213_Send();
                #endif
                EA = 0;
                WriteAllParams();
                EA = 1;
                PCSendComm(comm,0,0);
                PCSendComm(comm,1,0x80);
                break;
            }
}

void  Water5OneSec()    //run it every 1 sec
{


#ifdef NEPTUN
    if(W5_Tags.initcal > 1)
    {
        if((W5_Tags.PulseCounter0  >= 2)&&(GoodCal))
        {
            if(W5_Tags.initcal>2) {rotate_speed = 1;speed_counter = 0; speed=0; LED = 0; W5_Tags.initcal = 2;}
            if(rotate_speed == 0 )
            {
                    W5_Tags.initcal = 1;
                    EA = 0;
                    WriteAllParams();
                    EA = 1;
                    Switch_i = 66;
            }

        }
        else
        {
            if(--W5_Tags.initcal == 1) {W5_Tags.initcal = 0; LED = 0;}
            if((measure == 0)&&(GoodCal == 0)) measure = 2; //measure = 1  - no need individual calibration
        }
    }
#endif // NEPTUN

#ifdef KARAT
    if(!BUTT&&(Switch_i!=120)&&!waitsomeminutes)
#else
    if(!BUTT&&(Switch_i!=60)&&!waitsomeminutes)
#endif
    {
        Calibrate = 0;
        LED = Switch_i%2;
        Switch_i++;
    }
    else
    {
        #ifndef MAGNIUM_BATTERY
        if(!(DaysAfterDepas%60)&&(Voltage < 0x80))
        {
            if(!depassivation_active)
            {
                depassivation_active = 1;
                PCSendDepass();
            }
            //LED = 1;
        }
        if(depassivation_active || (Voltage < 60)) LED = 1;
        #endif // MAGNIUM_BATTERY

        if(Switch_i)
        {
#ifndef KARAT
            if(W5_Tags.initcal != 1)
            {

#ifdef NEPTUN
                W5_Tags.initcal = 240;
                Switch_i = 0;
                LED = 1;
                W5_Tags.PulseCounter0 = 0;
                measure = 2;   //measure = 1  - no need individual calibration
                return;
#else
                W5_Tags.initcal = 1;
                WriteAllParams();
                Switch_i = 66;
#endif
             }
#endif
            Water5ExecComm(Switch_i/2);
            Switch_i = LED = 0;
        }
    }

    if(++Seconds == 60)
    {

        Seconds = 0;

        if((W5_Tags.PulseCounter0 - PC_PrevMinute0) > MaxFlow0) MaxFlow0 = W5_Tags.PulseCounter0 - PC_PrevMinute0;
        if((W5_Tags.PulseCounter1 - PC_PrevMinute1) > MaxFlow1) MaxFlow1 = W5_Tags.PulseCounter1 - PC_PrevMinute1;

        PC_PrevMinute0 = W5_Tags.PulseCounter0;
        PC_PrevMinute1 = W5_Tags.PulseCounter1;

        if(LaterTimer)
        {
            if(--LaterTimer == 0) SendFlag |= 2;
        }

        if(Calibrate&&!SendFlag)
        {
            Calibrate--;
            #ifdef KARAT
            if(Calibrate%2) K213_Send_For_Serial();
            else K213_Send();
            #endif
            PCSendInfoData(0,0);
            PCSendInfoData(1,0x80);
        }

        if(sendweekflag)
        {
            sendweekflag = 0;
            PCSendWeekData(0,0);
            PCSendWeekData(1,0x80);
        }
        else if(sendmonthflag)
        {
            sendmonthflag = 0;
            PCSendInfoData(0,0);
            PCSendInfoData(1,0x80);
        }

        if(waitsomeminutes) waitsomeminutes--;
//        NewMinute++;
        #ifdef KARAT
        if((W5_Tags.initcal == 0))
        {
            if(energy != -1.0)
            {
                W5_Tags.initcal = 1;
                WriteAllParams();
                Water5ExecComm(33);
            } else K213_Send();

        }
        #endif

        if(++Minutes == 60)
        {
            Minutes = 0;
            LED = 0;
            HourMas0[Hours] = W5_Tags.PulseCounter0 - PC_PrevHour0;
            PC_PrevHour0 = W5_Tags.PulseCounter0;

            HourMas1[Hours] = W5_Tags.PulseCounter1 - PC_PrevHour1;
            PC_PrevHour1 = W5_Tags.PulseCounter1;

            if(W5_Tags.mode.sendeveryonehour)
            {
                EA = 0;
                WriteAllParams();
                EA = 1;
                PCSendHourData(0,0);
                PCSendHourData(1,0x80);
            }

            if((!(W5_Tags.poehali&0x1))&&(HourMas0[Hours]>2))
            {
                W5_Tags.poehali |= 1;
                PCSendInfoData(0,0);
            }
            if((!(W5_Tags.poehali&0x2))&&(HourMas1[Hours]>2))
            {
                W5_Tags.poehali |= 2;
                PCSendInfoData(1,0x80);
            }


            if(++Hours == 24)
            {
                Hours = 0;
                DaysAfterDepas++;
                #ifndef MAGNIUM_BATTERY
                if(depassivation_active)
                {
                    depassivation_active = 0;
                    EA = 0;
                    WriteAllParams();
                    EA = 1;
                    Calibrate = 6;
                    LED = 0;
                }
                #endif
                DayMas0[Days] = W5_Tags.PulseCounter0 - PC_PrevDay0;
                DayMas1[Days] = W5_Tags.PulseCounter1 - PC_PrevDay1;
                PC_PrevDay0 = W5_Tags.PulseCounter0;
                PC_PrevDay1 = W5_Tags.PulseCounter1;
                if(!W5_Tags.mode.sendeveryonehour)
                {
                    if(W5_Tags.mode.twochannels)
                    {
                        if(W5_Tags.Protocol)
                        {
                            PCSendDayData(0,0);
                            PCSendDayData(1,0x80);
                        }
                        else
                        {
                            PCSendDayData(0,0x80*((NBFi_Get_TX_Iter()%2) != 0));
                            PCSendDayData(1,0x80*((NBFi_Get_TX_Iter()%2) == 0));
                        }
                    }
                    else  PCSendDayData(0,0);

                }

                EA = 0;
                WriteAllParams();
                EA = 1;

                if(++Days == 7) {Days = 0; sendweekflag = 2;}
                if(++Days4month == 30) {Days4month = 0; sendmonthflag = 1;}

            }
        }
        #ifdef KARAT
        else if(Minutes == 59)
        {
            K213_Send();
        }
        #endif
    }
}

