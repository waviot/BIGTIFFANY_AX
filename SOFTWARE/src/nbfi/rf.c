#include "nbfi.h"
#include "nbfi_config.h"
#include "rf.h"
#include "hal.h"
#include "misc.h"
#include <libmfwtimer.h>

bool __xdata rf_busy = 0;
struct axradio_address rf_destination;

nbfi_rf_state_s rf_state = STATE_OFF;

void calibrate_oscillators(void);

uint8_t PSK_BAND;

void RF_SetFreq(uint32_t freq)
{
    axradio_phy_chanfreq[0] = axradio_conv_freq_fromhz(freq);//TODO fix freq calc
    // Dirty fix for insufficient arithmetics precision
    if(freq > 800000000)    axradio_phy_chanfreq[0]+=34; //868MHz
    else                    axradio_phy_chanfreq[0]+=18; //446MHz
}


#define FEM_MAX_DBM 26
#define FEM_GAIN 11
#define AX5043_MAX_DBM 15


const uint16_t AX5043_power[26] = {0x00aa, 0x00bf, 0x00d1, 0x00ec, 0x010f, 0x0132, 0x0156, 0x017f, 0x01af, 0x1e0, 0x207, 0x244, 0x290, 0x2eb, 0x35e, 0x3d6, 0x406, 0x4a9, 0x57c, 0x600, 0x700, 0x800, 0x9d4, 0xc00, 0xf00, 0xfff};


struct axradio_address  fastdladdress = {
	{ 0x6f, 0x6f, 0x6f, 0x6f}
};


void RF_SetModeAndPower(int8_t dBm, rf_direction_t mode, rf_antenna_t ant)
{

    if(dBm > RF_MAX_POWER) dBm = RF_MAX_POWER;
    if(dBm < RF_MIN_POWER) dBm = RF_MIN_POWER;


    if(dBm > FEM_MAX_DBM) dBm = FEM_MAX_DBM;



    if(mode == TX)
    {
        #if defined (BIGTIFFANY_AX)
            // set power
            AX5043_MODCFGA = PA_DIFFERENTIAL;
            AX5043_TXPWRCOEFFB1 = AX5043_power[dBm+10] >> 8;
            AX5043_TXPWRCOEFFB0 = AX5043_power[dBm+10] &0xFF;
        #endif

    }
    else // mode == RX or IDLE
    {
        #if defined (BIGTIFFANY_AX)
        AX5043_MODCFGA = PA_DIFFERENTIAL | PA_SHAPING;
        #endif
    }

}

nbfi_status_t RF_Init(  nbfi_phy_channel_t  phy_channel,
                        rf_antenna_t        antenna,
                        int8_t              power,
                        uint32_t            freq)
{
    uint8_t er;


    if(rf_busy) return ERR_RF_BUSY;


    rf_busy = true;

    if(phy_channel != OSC_CAL) nbfi_phy_channel = phy_channel;
    if(phy_channel == UL_CARRIER) nbfi_phy_channel = UL_DBPSK_50_PROT_D;


    if(freq > 600000000) PSK_BAND = 1;
    else PSK_BAND = 0;

    PIN_SET_OUTPUT(TCXO_DIR, TCXO_PIN); // Enable TCXO...
    TCXO = HIGH;

    PORTR = 0x0B;
    DIRR = 0x15;

    switch(phy_channel)
    {
    case UL_PSK_200:
    case UL_PSK_FASTDL:
    case UL_PSK_500:
    case UL_PSK_5000:
    case UL_DBPSK_50_PROT_C:
    case UL_DBPSK_50_PROT_D:
    case UL_DBPSK_50_PROT_E:
    case UL_DBPSK_400_PROT_C:
    case UL_DBPSK_400_PROT_D:
    case UL_DBPSK_400_PROT_E:
    case UL_DBPSK_3200_PROT_D:
    case UL_DBPSK_3200_PROT_E:
        ax5043_set_constants();
        er = axradio_init();    // Init radio registers
        if (er != AXRADIO_ERR_NOERROR)
        {
            rf_busy = false; return ERR;
        }
        er = axradio_set_mode(AXRADIO_MODE_ASYNC_TRANSMIT);
        if (er != AXRADIO_ERR_NOERROR)
        {
            rf_busy = false;return ERR;
        }
        RF_SetFreq(freq);
        RF_SetModeAndPower(power, TX, antenna);
        rf_busy = false;
        rf_state = STATE_TX;
        return OK;
    case DL_PSK_200:
    case DL_PSK_FASTDL:
    case DL_PSK_500:
    case DL_PSK_5000:
        ax5043_set_constants();
        RF_SetModeAndPower(power, RX, antenna);
        RF_SetLocalAddress((uint8_t __generic *)&fastdladdress);
        RF_SetFreq(freq);

        er = axradio_init();    // Init radio registers
        if (er != AXRADIO_ERR_NOERROR)
        {
             rf_busy = false; return ERR;
        }
        er = axradio_set_mode(AXRADIO_MODE_ASYNC_RECEIVE);
        rf_busy = false;
        if (er != AXRADIO_ERR_NOERROR)
        {
            return ERR;
        }
        rf_state = STATE_RX;
        return OK;
    case UL_CARRIER:
        ax5043_set_constants();
        er = axradio_init();    // Init radio registers

        if (er != AXRADIO_ERR_NOERROR)
        {
            rf_busy = false; return ERR;
        }
        er = axradio_set_mode(AXRADIO_MODE_ASYNC_TRANSMIT);
        if (er != AXRADIO_ERR_NOERROR)
        {
            rf_busy = false;return ERR;
        }
        RF_SetFreq(nbfi.ul_freq_base + 25000);
        RF_SetModeAndPower(power, TX, antenna);
        rf_busy = false;
        rf_state = STATE_TX;
        axradio_set_mode(AXRADIO_MODE_CW_TRANSMIT);
        return OK;
    case OSC_CAL:
        if(nbfi.rx_phy_channel == UL_CARRIER) return ERR;
        axradio_set_mode(AXRADIO_MODE_ASYNC_RECEIVE);
        calibrate_oscillators();
        axradio_set_mode(AXRADIO_MODE_OFF);
        delay_ms(2);
        rf_state = STATE_OFF;
    }
    TCXO = LOW;
    rf_busy = false;
    return ERR;
}

nbfi_status_t RF_Deinit()
{
    uint8_t er;
    if(rf_busy) return ERR_RF_BUSY;
    rf_busy = true;
    er = axradio_set_mode(AXRADIO_MODE_OFF);
    rf_busy = false;
    RF_SetModeAndPower(0, RX, 0);
    delay_ms(1);
    TCXO = LOW;
    rf_state = STATE_OFF;
    if (er != AXRADIO_ERR_NOERROR) return ERR;
    return OK;
}

void RF_SetDstAddress(uint8_t __generic * addr)
{
    for(uint8_t i = 0; i !=3; i++) rf_destination.addr[i] = addr[i];
}


void RF_SetLocalAddress(uint8_t __generic * addr)
{
    struct axradio_address_mask     localaddress = {{0,0,0,0},{0xff, 0, 0, 0}};

    localaddress.addr[0] = addr[0];

    axradio_set_local_address(&localaddress);
}



nbfi_status_t RF_Transmit(uint8_t __generic* pkt, uint8_t len,  rf_padding_t padding, rf_blocking_t blocking)
{
    if(rf_busy) return ERR_RF_BUSY;

    rf_busy = true;

    axradio_transmit(&rf_destination, pkt, len, padding);

    if(blocking == BLOCKING)
    {
        while(1) // Wait for TX complete
        {
            if(!rf_busy) break;
            wtimer_runcallbacks();
        }
    }
    return OK;
}

#define XTALFREQ   26000000UL
#define XTALDIV    (((XTALFREQ) > 40000000UL) ? 2 : ((XTALFREQ) > 20000000UL) ? 1 : 0)
#define XTALFRQDIV ((XTALFREQ) >> (XTALDIV))


static inline uint16_t myabs(int16_t a)
{
    if(a > 0) return a;
    else return -a;
}

uint8_t hp0, hp1;

void calibrate_oscillators(void)
{
    static uint16_t __xdata frcmaxerr;
    static uint16_t __xdata lpmaxerr;
    static uint16_t __xdata frcperiod;
    static uint16_t __xdata lpperiod;
    static uint8_t __xdata frccalcnt;
    static uint8_t __xdata lpcalcnt;
    static uint8_t __xdata clkconsave;
    frcperiod = 0;
    lpperiod = 0;
    frccalcnt = 0;
    lpcalcnt = 0;
    clkconsave = 0xff;
    EA = 0;
    ax5043_rclk_enable(XTALDIV);

	static uint8_t __xdata lposccfg;
	uint8_t frcosccfg;

    frcosccfg = 0x70 | ((CLKSRC_RSYSCLK) & 0x07);
    FRCOSCREF0 = (10000000UL << 8) / ((XTALFRQDIV) >> 6);
    FRCOSCREF1 = ((10000000UL << 8) / ((XTALFRQDIV) >> 6)) >> 8;
    lposccfg = 0x00 | ((CLKSRC_RSYSCLK) & 0x07);
    LPOSCREF0 = (XTALFRQDIV) / 320UL;
    LPOSCREF1 = ((XTALFRQDIV) / 320UL) >> 8;
	FRCOSCKFILT0 = 0x0;
	FRCOSCKFILT1 = 0x40;
	LPOSCKFILT1 = 0x05;
    LPOSCKFILT0 = 0x0;

    FRCOSCCONFIG = frcosccfg;
	LPOSCCONFIG = lposccfg;

    frcmaxerr = FRCOSCREF1;
    frcmaxerr <<= 8;
    frcmaxerr |= FRCOSCREF0;
    frcmaxerr /= 500;

    lpmaxerr = LPOSCREF1;
    lpmaxerr <<= 8;
    lpmaxerr |= LPOSCREF0;
    lpmaxerr /= 500;

    OSCCALIB |= 0x03;

    for (;;) {
          uint8_t osccal = OSCCALIB;
        if (osccal & 0x40) {

            frcperiod = FRCOSCPER1;
            frcperiod <<= 8;
            frcperiod |= FRCOSCPER0;

            uint16_t tmp = FRCOSCREF1;
            tmp <<= 8;
            tmp |= FRCOSCREF0;

            if (myabs(frcperiod - tmp) > frcmaxerr)
                frccalcnt = 0;
            else if (frccalcnt != 0xff)
                ++frccalcnt;
        }
        if (osccal & 0x80) {
            lpperiod = LPOSCPER1;
            lpperiod <<= 8;
            lpperiod |= LPOSCPER0;

            uint16_t tmp = LPOSCREF1;
            tmp <<= 8;
            tmp |= LPOSCREF0;

            if (myabs(lpperiod - tmp) > lpmaxerr)
                lpcalcnt = 0;
            else if (lpcalcnt != 0xff)
                ++lpcalcnt;
        }
        if (frccalcnt > 4 && lpcalcnt > 4)
            break;
        wtimer_runcallbacks();
    }
    OSCCALIB &= (uint8_t)~0x03;
    if (clkconsave != 0xff) {
        // undo
        CLKCON = 0x08 | CLKSRC_FRCOSC;
        while ((CLKCON & 0x3F) != (0x08 | CLKSRC_FRCOSC));
        CLKCON = clkconsave;
    }
    ax5043_rclk_disable();
    EA = 1;
    FRCOSCCONFIG = 0;
    hp0 = FRCOSCFREQ0;
    hp1 = FRCOSCFREQ1;
}





