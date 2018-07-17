
#include <ax8052f143.h>
#include <libmftypes.h>
#include <libmfradio.h>
#include <libmfflash.h>
#include <libmfwtimer.h>

#include <string.h>

#include "misc.h"

#include "application.h"

#include "hal.h"
#include "nbfi.h"
#include "nbfi_config.h"
#include "rf.h"
#include "water5.h"


uint8_t __data coldstart = 1; // caution: initialization with 1 is necessary! Variables are initialized upon _sdcc_external_startup returning 0 -> the coldstart value returned from _sdcc_external startup does not survive in the coldstart case

static void pwrmgmt_irq(void) __interrupt(INT_POWERMGMT)
{
    uint8_t pc = PCON;
    if (!(pc & 0x80))
        return;
    GPIOENABLE = 0;
    IE = EIE = E2IE = 0;
    for (;;)
        PCON |= 0x01;
}


void axradio_statuschange(struct axradio_status __xdata *st)
{
    switch (st->status)
    {
    case AXRADIO_STAT_TRANSMITSTART:
        axradio_set_channel(0);
        if( st->error == AXRADIO_ERR_TIMEOUT )  rf_busy = false;
        break;

    case AXRADIO_STAT_TRANSMITEND:
        //PORTB_0 = 0;
        rf_busy = false;
        NBFi_TX_Finished();
        break;

    case AXRADIO_STAT_RECEIVE:
        if(st->error == AXRADIO_ERR_NOERROR)
        {
            NBFi_ParseReceivedPacket(st);
        }
        break;

    default:
        break;
    }
}

#if defined(__ICC8051__)

#if (__CODE_MODEL__ == 2)
__near_func __root char
#else
__root char
#endif
__low_level_init(void) @ "CSTART"
#else
uint8_t _sdcc_external_startup(void)
#endif
{
    LPXOSCGM = 0x8A;
#ifdef LOWFREQ_RC
    wtimer0_setclksrc(CLKSRC_LPOSC, 1);
#elif defined LOWFREQ_QUARZ
    wtimer0_setclksrc(CLKSRC_LPXOSC, 3); // 8192 hz
#endif // LOWFREQ_RC
    wtimer1_setclksrc(CLKSRC_FRCOSC, 7);

    LPOSCCONFIG = 0x09; // Slow, PRESC /1, no cal. Does NOT enable LPOSC. LPOSC is enabled upon configuring WTCFGA (MODE_TX_PERIODIC and receive_ack() )

    coldstart = !(PCON & 0x40);

    axradio_setup_pincfg1();
    DPS = 0;
    IE = 0x40;
    EIE = 0x00;
    E2IE = 0x00;
    GPIOENABLE = 1; // unfreeze GPIO

    return coldstart;
}

void main(void)
{
    EA = 1;

    if (coldstart)
    {
        flash_apply_calibration();
        CLKCON = 0x00;
        wtimer_init();
        ColdSetup(); // after power-up
    }
    else
    {
        uint8_t lp0 = LPOSCFREQ0;
        uint8_t lp1 = LPOSCFREQ1;
        flash_apply_calibration();
        LPOSCFREQ1 = lp1;
        LPOSCFREQ0 = lp0;
        if(nbfi.additional_flags&NBFI_FLG_DO_OSCCAL)
        {
            FRCOSCFREQ0 = hp0;
            FRCOSCFREQ1 = hp1;
        }
        wtimer_init();
        WarmSetup(); // after wake-up from sleep
    }

    axradio_setup_pincfg2();

    for(;;)
    {

        Loop();
        wtimer_runcallbacks();
        EA = 0;
        uint8_t flg = WTFLAG_CANSTANDBY;
        if (axradio_cansleep() && (!rf_busy) && (rf_state == STATE_OFF))
        {
            flg |= WTFLAG_CANSLEEP;
        }
        wtimer_idle(flg);
        EA = 1;
    }
}
