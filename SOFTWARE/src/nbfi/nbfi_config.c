#include "libmfflash.h"
#include "nbfi.h"
#include "nbfi_config.h"
#include "hal.h"
#include "rf.h"


bool NBFi_Config_Tx_Idle();

nbfi_settings_t __xdata nbfi;

struct wtimer_desc __xdata nbfi_send_mode_delay;

#ifdef  BIGTIFFANY_AX
    const nbfi_settings_t __code nbfi_set_default =
    {
        NRX,//mode;
        UL_DBPSK_50_PROT_D,// tx_phy_channel;
        DL_PSK_200,// rx_phy_channel;
        HANDSHAKE_NONE,// nandshake_mode;
        MACK_1, //mack_mode
        1,//num_of_retries;
        8,//max_payload_len;
        {0},    //dl_ID[3];
        {0},    //temp_ID[3];
        {0xFF,0,0},     //broadcast_ID[3];
        {0},    //full_ID[6];
        0,//NBFI_DL_FREQ_BASE,      //tx_freq;
        0,//NBFI_DL_FREQ_BASE,      //rx_freq;
        PCB,    //tx_antenna;
        PCB,    //rx_antenna;
        RF_MAX_POWER,     //tx_pwr;
        10080,     //heartbeat_interval
        255,      //heartbeat_num
        NBFI_FLG_DO_OSCCAL,
        NBFI_UL_FREQ_BASE,
        NBFI_DL_FREQ_BASE
    };
#endif

const nbfi_settings_t __code nbfi_fastdl =
{
    CRX,//mode;
    UL_PSK_FASTDL,  // tx_phy_channel;
    DL_PSK_FASTDL,  // rx_phy_channel;
    HANDSHAKE_NONE,// handshake_mode;
    MACK_1, //mack_mode
    1,//num_of_retries;
    128,//max_payload_len;
    {0xFF,0,0},    //dl_ID[3];
    {0},    //temp_ID[3];
    {0xFF,0,0},     //broadcast_ID[3];
    {0},    //full_ID[6];
    NBFI_DL_FREQ_BASE + 1000000,      //tx_freq;
    NBFI_DL_FREQ_BASE + 1000000,      //rx_freq;
    PCB,    //tx_antenna;
    PCB,    //rx_antenna;
    RF_MAX_POWER,     //tx_pwr;
    60,     //heartbeat_interval
    0,      //heartbeat_num
    NBFI_FLG_FIXED_BAUD_RATE | NBFI_FLG_NO_RESET_TO_DEFAULTS | NBFI_FLG_NO_SENDINFO |NBFI_FLG_NO_XTEA,
    NBFI_UL_FREQ_BASE,
    NBFI_DL_FREQ_BASE//additional_flags
};





NBFi_station_info_s nbfi_station_info = 0;

extern uint8_t  __xdata string[50];

nbfi_settings_t __xdata nbfi_prev;

bool nbfi_settings_changed = 0;


#define NUM_OF_TX_RATES    3
#define NUM_OF_RX_RATES    4

nbfi_phy_channel_t __xdata TxRateTable[NUM_OF_TX_RATES] = {UL_DBPSK_50_PROT_C, UL_DBPSK_400_PROT_C, UL_DBPSK_3200_PROT_D};
const uint8_t TxSNRDegradationTable[NUM_OF_TX_RATES] = {0,9,18};
nbfi_phy_channel_t __xdata RxRateTable[NUM_OF_RX_RATES] = {DL_PSK_200, DL_PSK_500, DL_PSK_5000, DL_PSK_FASTDL};
const uint8_t RxSNRDegradationTable[NUM_OF_RX_RATES] = {0,4,14,25};

#define TX_SNRLEVEL_FOR_UP          15 //22
#define TX_SNRLEVEL_FOR_DOWN        10

#define RX_SNRLEVEL_FOR_UP          15 //18
#define RX_SNRLEVEL_FOR_DOWN        10

#define RX_SNRLEVEL_FOR_FASTUL      15

#define NUM_OF_RETRIES_FOR_FASTDL   15


uint8_t __xdata current_tx_rate = 0;
uint8_t __xdata current_rx_rate = 0;

uint8_t __xdata prev_tx_rate = 0;
uint8_t __xdata prev_rx_rate = 0;

uint8_t __xdata success_rate = 0;

uint8_t __xdata you_should_dl_power_step_up = 0;
uint8_t __xdata you_should_dl_power_step_down = 0;

static bool NBFI_Config_is_high_SNR_for_UP(uint8_t rx_tx)
{
    uint8_t delta;
    if(rx_tx & RX_CONF)
    {
        if(current_rx_rate < (NUM_OF_RX_RATES - 1))
        {
            delta = RxSNRDegradationTable[current_rx_rate + 1] - RxSNRDegradationTable[current_rx_rate];
            if(nbfi_state.aver_rx_snr  > RX_SNRLEVEL_FOR_UP + delta) return true;
            else return false;
        }
        else
        {
            if(nbfi_state.aver_rx_snr  > RX_SNRLEVEL_FOR_UP + 10) return true;
            else return false;
        }


    }

    if(rx_tx & TX_CONF)
    {
        if(current_tx_rate < (NUM_OF_TX_RATES - 1))
        {
            delta = TxSNRDegradationTable[current_tx_rate + 1] - TxSNRDegradationTable[current_tx_rate];
            if(nbfi_state.aver_tx_snr  > TX_SNRLEVEL_FOR_UP + delta) return true;
            else return false;
        }
        else
        {
            if(nbfi_state.aver_tx_snr  > TX_SNRLEVEL_FOR_UP + 10) return true;
            else return false;
        }
    }
    return false;
}

void NBFI_Config_Check_State()
{
    if(nbfi.tx_phy_channel != UL_PSK_FASTDL)
    {
        nbfi_state.UL_rating = (nbfi_state.aver_tx_snr + TxSNRDegradationTable[current_tx_rate]);
        if(nbfi_state.UL_rating > 40) nbfi_state.UL_rating = 40;
        nbfi_state.UL_rating >>= 2;

        nbfi_state.DL_rating = (nbfi_state.aver_rx_snr + RxSNRDegradationTable[current_rx_rate]);
        if(nbfi_state.DL_rating > 40) nbfi_state.DL_rating = 40;
        nbfi_state.DL_rating >>= 2;
    }


    #ifdef FIXED_BAUD_RATE
    return;
    #endif // FIXED_BAUD_RATE
    if(nbfi.mode == NRX) return;
    if(nbfi.additional_flags&NBFI_FLG_FIXED_BAUD_RATE) return;
    if(nbfi.handshake_mode == HANDSHAKE_NONE) return;
    switch(nbfi_active_pkt->state)
    {
        case PACKET_WAIT_ACK:
//        case PACKET_DL_DELAY:
        case PACKET_WAIT_FOR_EXTRA_PACKETS:
        return;
    }

    if(you_should_dl_power_step_down && (nbfi_state.aver_rx_snr < RX_SNRLEVEL_FOR_UP)) you_should_dl_power_step_down = 0;
    if(you_should_dl_power_step_up && (nbfi_state.aver_rx_snr > RX_SNRLEVEL_FOR_DOWN)) you_should_dl_power_step_up = 0;

    if(NBFI_Config_is_high_SNR_for_UP(TX_CONF)/*(nbfi_state.aver_tx_snr > TX_SNRLEVEL_FOR_UP)*/ && NBFI_Config_is_high_SNR_for_UP(RX_CONF)/*(nbfi_state.aver_rx_snr > RX_SNRLEVEL_FOR_UP)*/)
    {
        if(!NBFi_Config_Rate_Change(RX_CONF|TX_CONF, UP))
        {
            NBFi_Config_Tx_Power_Change(DOWN);
            you_should_dl_power_step_down = (1 << 7);
        }

        return;
    }

    if(NBFI_Config_is_high_SNR_for_UP(TX_CONF)/*nbfi_state.aver_tx_snr > TX_SNRLEVEL_FOR_UP*/)
    {
        if(!NBFi_Config_Rate_Change(TX_CONF, UP)) NBFi_Config_Tx_Power_Change(DOWN);
        return;
    }

    if(NBFI_Config_is_high_SNR_for_UP(RX_CONF)/*nbfi_state.aver_rx_snr > RX_SNRLEVEL_FOR_UP*/)
    {
        if(!NBFi_Config_Rate_Change(RX_CONF, UP)) you_should_dl_power_step_down = (1 << 7);
        return;
    }

    if(nbfi_state.aver_tx_snr < TX_SNRLEVEL_FOR_DOWN)
    {
        if(!NBFi_Config_Tx_Power_Change(UP)) NBFi_Config_Rate_Change(TX_CONF, DOWN);
    }

    if(nbfi_state.aver_rx_snr < RX_SNRLEVEL_FOR_DOWN)
    {
        you_should_dl_power_step_up = (1 << 6);
        NBFi_Config_Rate_Change(RX_CONF, DOWN);
        return;
    }

}

bool NBFi_Config_Rate_Change(uint8_t rx_tx, nbfi_rate_direct_t dir )
{
    uint8_t  rx = current_rx_rate;
    uint8_t  tx = current_tx_rate;
    uint8_t should_not_to_reduce_pwr = 0;
    if((rx_tx & RX_CONF) && NBFi_Config_Tx_Idle())
    {
        if((dir == UP)&& nbfi_station_info.DL_SPEED_NOT_MAX)
        {
            if(++current_rx_rate > NUM_OF_RX_RATES - 1)  current_rx_rate = NUM_OF_RX_RATES - 1;
        }
        if(dir == DOWN)
        {
            if(((int8_t)(--current_rx_rate)) < 0 ) current_rx_rate = 0;
        }
    }

    if((rx_tx & TX_CONF))
    {
        if((dir == UP)&& nbfi_station_info.UL_SPEED_NOT_MAX)
        {
            if(++current_tx_rate > NUM_OF_TX_RATES - 1)  current_tx_rate = NUM_OF_TX_RATES - 1;
            if(RxRateTable[current_rx_rate] == DL_PSK_200) while((TxRateTable[current_tx_rate] != UL_DBPSK_50_PROT_C) && (TxRateTable[current_tx_rate] != UL_DBPSK_50_PROT_D))
            {
                current_tx_rate--;
                should_not_to_reduce_pwr = 1;
            }
            if(RxRateTable[current_rx_rate] == DL_PSK_500) while((TxRateTable[current_tx_rate] != UL_DBPSK_400_PROT_C) && (TxRateTable[current_tx_rate] != UL_DBPSK_400_PROT_D))
            {
                current_tx_rate--;
                should_not_to_reduce_pwr = 1;
            }
        }
        else if(dir == DOWN)
        {
            if(((int8_t)(--current_tx_rate)) < 0 ) current_tx_rate = 0;
        }
    }
    if((nbfi.tx_phy_channel == TxRateTable[current_tx_rate]) && (nbfi.rx_phy_channel == RxRateTable[current_rx_rate]))
    {
        if(should_not_to_reduce_pwr) return true;
        else return false;
    }

    memcpy_xdata(&nbfi_prev, &nbfi, sizeof(nbfi));
    prev_rx_rate = rx;
    prev_tx_rate = tx;

    nbfi.tx_phy_channel = TxRateTable[current_tx_rate];

    if((nbfi.rx_phy_channel == RxRateTable[current_rx_rate]) && (dir == DOWN)) return true;

    nbfi.rx_phy_channel = RxRateTable[current_rx_rate];

   if(!NBFi_Config_Send_Mode(true, NBFI_PARAM_MODE))
    {
        NBFi_Config_Return();
        return true;
    }

    if(nbfi.tx_phy_channel == UL_PSK_FASTDL) nbfi.num_of_retries = NUM_OF_RETRIES_FOR_FASTDL;
    else nbfi.num_of_retries = nbfi_set_default.num_of_retries;
    return true;
}

bool NBFi_Config_Tx_Power_Change(nbfi_rate_direct_t dir)
{
    if(nbfi.additional_flags&NBFI_FLG_NO_REDUCE_TX_PWR) return false;

    int8_t old_pwr = nbfi.tx_pwr;
    if(dir == UP)
    {
        nbfi.tx_pwr += 3;
        nbfi_state.aver_tx_snr += 3;
        if(nbfi.tx_pwr > RF_MAX_POWER) nbfi.tx_pwr = RF_MAX_POWER;
    }
    else
    {
        nbfi.tx_pwr -= 3;
        nbfi_state.aver_tx_snr -= 3;
        if(nbfi.tx_pwr < RF_MIN_POWER) nbfi.tx_pwr = RF_MIN_POWER;
    }
    return (nbfi.tx_pwr != old_pwr);
}

void NBFi_Config_Send_Current_Mode(struct wtimer_desc __xdata *desc)
{
    NBFi_Config_Send_Mode(false, NBFI_PARAM_MODE);
    NBFi_Send_Clear_Cmd(0);
}

bool NBFi_Config_Send_Mode(bool ack, uint8_t param)
{
    nbfi_transport_packet_t __xdata* ack_pkt =  NBFi_AllocateTxPkt(8);

    if(!ack_pkt)
    {
        return false;
    }
    ack_pkt->phy_data.payload[0] = 0x06;
    ack_pkt->phy_data.payload[1] = (READ_PARAM_CMD << 6) + param;//NBFI_PARAM_MODE;
    NBFi_Config_Parser(&ack_pkt->phy_data.payload[1]);
    ack_pkt->phy_data.ITER = nbfi_state.UL_iter & 0x1f;;
    ack_pkt->phy_data.header |= SYS_FLAG;
    if(ack)
    {
        ack_pkt->handshake = nbfi.handshake_mode;
        ack_pkt->phy_data.header |= ACK_FLAG;
    }
    ack_pkt->state = PACKET_NEED_TO_SEND_RIGHT_NOW;
    return true;
}

void bigendian_cpy(uint8_t __xdata* from, uint8_t __xdata* to, uint8_t len)
{
    for(uint8_t i = 0; i != len; i++)
    {
        to[i] = from[len - i - 1];
    }
}

uint8_t CompVersion()
{

    const char CompTime[] = __TIME__;
    const char* ptr;
    uint8_t ver = 0;
    ptr = &CompTime[0];
    while(*ptr) ver += ((*(ptr++)) - 0x30) + (uint8_t)ptr;
    return ver;
}

bool NBFi_Config_Parser(uint8_t __xdata* buf)
{
    switch(buf[0]>>6)
    {
        case READ_PARAM_CMD:
                memset_xdata(&buf[1], 0xff, 6);
                switch(buf[0]&0x3f)
                {
                    case NBFI_PARAM_MODE:
                        buf[1] = nbfi.mode;
                        buf[2] = nbfi.mack_mode;
                        buf[3] = nbfi.tx_phy_channel;
                        buf[4] = nbfi.rx_phy_channel;
                        buf[5] = nbfi.tx_pwr;
                        buf[6] = nbfi.num_of_retries;
                        break;
                    case NBFI_PARAM_HANDSHAKE:
                        buf[1] = nbfi.handshake_mode;
                        buf[2] = nbfi.mack_mode;
                        break;
                    case NBFI_PARAM_MAXLEN:
                        buf[1] = nbfi.max_payload_len;
                        break;
                    case NBFI_PARAM_TXFREQ:
                        bigendian_cpy((uint8_t __xdata*)&nbfi.tx_freq, &buf[1], 4);
                        break;
                    case NBFI_PARAM_RXFREQ:
                        bigendian_cpy((uint8_t __xdata*)&nbfi.rx_freq, &buf[1], 4);
                        break;
                    case NBFI_PARAM_ANT:
                        buf[1] = nbfi.tx_pwr;
                        buf[2] = nbfi.tx_antenna;
                        buf[3] = nbfi.rx_antenna;
                        break;
                    case NBFI_PARAM_DL_ADD:
                        for(uint8_t i = 0; i != 3; i++)  buf[1 + i] = nbfi.dl_ID[i];
                        break;
                    case NBFI_PARAM_HEART_BEAT:
                        buf[1] = nbfi.heartbeat_num;
                        buf[2] = nbfi.heartbeat_interval >> 8;
                        buf[3] = nbfi.heartbeat_interval & 0xff;
                        break;
                    case NBFI_PARAM_TX_BRATES:
                        for(uint8_t i = 0; i != NUM_OF_TX_RATES; i++)
                        {
                            if(i > 5) break;
                            buf[i + 1] = TxRateTable[i];
                        }
                        break;
                    case NBFI_PARAM_RX_BRATES:
                        for(uint8_t i = 0; i != NUM_OF_RX_RATES; i++)
                        {
                            if(i > 5) break;
                            buf[i + 1] = RxRateTable[i];
                        }
                        break;
                    case NBFI_PARAM_VERSION:
                        buf[1] = NBFI_REV;
                        buf[2] = NBFI_SUBREV;
                        buf[3] = CompVersion();
                        buf[4] = HARDWARE_ID;
                        buf[5] = HARDWARE_REV;
                        buf[6] = BAND_ID;
                        break;
                    case NBFI_QUALITY:
                        bigendian_cpy((uint8_t __xdata*)&nbfi_state.UL_total, &buf[1], 2);
                        bigendian_cpy((uint8_t __xdata*)&nbfi_state.DL_total, &buf[3], 2);
                        buf[5] = nbfi_state.aver_rx_snr;
                        buf[6] = nbfi_state.aver_tx_snr;
                        break;
                    case NBFI_QUALITY_EX :
                        buf[1] = nbfi_state.UL_rating;
                        buf[2] = nbfi_state.DL_rating;
                        bigendian_cpy((uint8_t __xdata*)&nbfi_state.success_total, &buf[3], 2);
                        bigendian_cpy((uint8_t __xdata*)&nbfi_state.fault_total, &buf[5], 2);
                        break;
                    case NBFI_ADD_FLAGS:
                        buf[1] = nbfi.additional_flags;
                        break;
                    case NBFI_UL_BASE_FREQ:
                        bigendian_cpy((uint8_t __xdata*)&nbfi.ul_freq_base, &buf[1], 4);
                        break;
                    case NBFI_DL_BASE_FREQ:
                        bigendian_cpy((uint8_t __xdata*)&nbfi.dl_freq_base, &buf[1], 4);
                        break;
                    default:
                        return false;
                        break;
                }
            break;

           case WRITE_PARAM_AND_SAVE_CMD:
                NBFi_Config_Set_Default();
             case WRITE_PARAM_CMD:
                memcpy_xdata(&nbfi_prev, &nbfi, sizeof(nbfi));
                switch(buf[0]&0x3f)
                {
                    case NBFI_PARAM_MODE:
                        if(buf[1] != 0xff) nbfi.mode = buf[1];
                        if(buf[2] != 0xff) nbfi.mack_mode = buf[2];
                        if(buf[3] != 0xff) NBFi_Config_Set_TX_Chan(buf[3]);
                        if(buf[4] != 0xff) {NBFi_Config_Set_RX_Chan(buf[4]); rf_state = STATE_CHANGED;}
                        if(buf[5] != 0xff) nbfi.tx_pwr = buf[5];
                        if(buf[6] != 0xff) nbfi.num_of_retries = buf[6];
                        if(CheckTask(&nbfi_send_mode_delay)&&((buf[3] != 0xff)||(buf[4] != 0xff)))
                        {
                            wtimer0_remove(&nbfi_send_mode_delay);
                            NBFi_Send_Clear_Cmd(0);
                        }
                        break;
                    case NBFI_PARAM_HANDSHAKE:
                        if(buf[1] != 0xff)
                        {
                            nbfi.handshake_mode = buf[1];
                        }
                        if(buf[2] != 0xff) nbfi.mack_mode = buf[2];
                        break;
                    case NBFI_PARAM_MAXLEN:
                        nbfi.max_payload_len = buf[1];
                        break;
                    case NBFI_PARAM_TXFREQ:
                        bigendian_cpy(&buf[1], (uint8_t __xdata*)&nbfi.tx_freq, 4);
                        if(buf[5] != 0xff) nbfi.tx_pwr = buf[5];
                        if(buf[6] != 0xff) nbfi.tx_antenna = buf[6];
                        break;
                    case NBFI_PARAM_RXFREQ:
                        bigendian_cpy(&buf[1], (uint8_t __xdata*)&nbfi.rx_freq, 4);
                        rf_state = STATE_CHANGED;
                        break;
                    case NBFI_PARAM_ANT:
                        if(buf[1] != 0xff) nbfi.tx_pwr = buf[1];
                        if(buf[2] != 0xff) nbfi.tx_antenna = buf[2];
                        if(buf[3] != 0xff) {nbfi.rx_antenna = buf[3]; rf_state = STATE_CHANGED;}
                        break;
                    case NBFI_PARAM_DL_ADD:
                        for(uint8_t i = 0; i != 3; i++)  nbfi.dl_ID[i] = buf[1 + i];
                        break;
                    case NBFI_PARAM_HEART_BEAT:
                        nbfi.heartbeat_num = buf[1];
                        nbfi.heartbeat_interval  = buf[2];
                        nbfi.heartbeat_interval <<= 8;
                        nbfi.heartbeat_interval += buf[3];
                        break;
                    case NBFI_ADD_FLAGS:
                        nbfi.additional_flags = buf[1];
                        break;
                    case NBFI_UL_BASE_FREQ:
                        bigendian_cpy(&buf[1], (uint8_t __xdata*)&nbfi.ul_freq_base, 4);
                        break;
                    case NBFI_DL_BASE_FREQ:
                        bigendian_cpy(&buf[1], (uint8_t __xdata*)&nbfi.dl_freq_base, 4);
                        break;
                    default:
                        nbfi_settings_changed = 0;
                        return false;
                        break;
                }
                if(buf[0]>>6 == WRITE_PARAM_AND_SAVE_CMD)
                {
                    NBFi_WriteConfig();
                    NBFi_Config_Send_Mode(false, NBFI_PARAM_MODE);
                    return false;
                }
             break;

    }
    buf[0] |= 0x80;
    return true;
}

void NBFi_Config_Return()
{
    memcpy_xdata(&nbfi, &nbfi_prev, sizeof(nbfi));
    current_tx_rate = prev_tx_rate;
    current_rx_rate = prev_rx_rate;
    if(nbfi.mode == NRX) nbfi.handshake_mode = HANDSHAKE_NONE;
    NBFi_Config_Send_Mode(false, NBFI_PARAM_MODE);
}

void NBFi_Configure_IDs()
{
    nbfi.full_ID[0] = 0;
    nbfi.full_ID[1] = FULL_ID[2];
    nbfi.full_ID[2] = FULL_ID[1];
    nbfi.full_ID[3] = FULL_ID[0];
    nbfi.full_ID[4] = 0;
    nbfi.full_ID[5] = 0;

    nbfi.temp_ID[0] = nbfi.full_ID[1];
    nbfi.temp_ID[1] = nbfi.full_ID[2];
    nbfi.temp_ID[2] = nbfi.full_ID[3];

}

void NBFi_Config_Set_Default()
{
    NBFi_ReadConfig();

    if(nbfi.tx_phy_channel == UL_DBPSK_50_PROT_C)
    {
        TxRateTable[0] = UL_DBPSK_50_PROT_C;
        TxRateTable[1] = UL_DBPSK_400_PROT_C;
    }
    else if(nbfi.tx_phy_channel == UL_DBPSK_50_PROT_D)
    {
        TxRateTable[0] = UL_DBPSK_50_PROT_D;
        TxRateTable[1] = UL_DBPSK_400_PROT_D;
    }

    NBFi_Configure_IDs();

    for(uint8_t i = 0; i != 3; i++) nbfi.dl_ID[i] = nbfi.temp_ID[i];   //default DL address

    random_seed = nbfi.full_ID[0] ^ nbfi.full_ID[1] ^ nbfi.full_ID[2] ^ nbfi.full_ID[3] ^ nbfi.full_ID[4] ^ nbfi.full_ID[5];

    nbfi_state.aver_tx_snr = nbfi_state.aver_rx_snr = 0;
    current_tx_rate = current_rx_rate = 0;

    if((nbfi_active_pkt->state == PACKET_WAIT_ACK)/*|| (nbfi_active_pkt->state == PACKET_DL_DELAY)*/) nbfi_active_pkt->state = PACKET_LOST;

 }

void NBFi_Config_Set_FastDl(bool fast)
{
    if(fast)
    {
        wtimer0_remove(&nbfi_send_mode_delay);
        NBFi_Clear_TX_Buffer();
        for(uint8_t i = 0; i != sizeof(nbfi_settings_t); i++)
        {
            ((uint8_t __xdata*)&nbfi)[i] = ((uint8_t __code*)&nbfi_fastdl)[i];
        }
        NBFi_Configure_IDs();
        for(uint8_t i = 0; i != 3; i++) nbfi.dl_ID[i] = nbfi.temp_ID[i];   //default DL address

    }
    else
    {
        NBFi_Config_Set_Default();
        ScheduleTask(&nbfi_send_mode_delay, NBFi_Config_Send_Current_Mode, RELATIVE, SECONDS(5));
    }

    if(rf_state == STATE_RX) NBFi_RX();
}


bool NBFi_Config_Tx_Idle()
{
        switch(nbfi_active_pkt->state)
        {
        case PACKET_WAIT_ACK:
        case PACKET_WAIT_FOR_EXTRA_PACKETS:
        case PACKET_QUEUED_AGAIN:
            return false;
        default:
            return true;
        }
}

void NBFi_Go_To_Sleep(uint8_t sleep)
{
    static uint8_t old_state = 100;
    if(sleep)
    {
        nbfi.mode = NRX;
        wtimer0_remove(&nbfi_send_mode_delay);
        NBFi_Clear_TX_Buffer();
    }
    else
    {
        if(old_state != sleep)
        {
            NBFi_Config_Set_Default();
            ScheduleTask(&nbfi_send_mode_delay, NBFi_Config_Send_Current_Mode, RELATIVE, SECONDS(5));
            SLIP_Init();
            NBFi_Force_process();
        }
    }
    old_state = sleep;
}

void NBFi_ReadConfig()
{

    if(flash_read(0xfe00) == 0xffff)
    {
        for(uint8_t i = 0; i != sizeof(nbfi_settings_t); i++)
        {
            ((uint8_t __xdata*)&nbfi)[i] = ((uint8_t __code*)&nbfi_set_default)[i];
        }
    }
    else
    {
        uint16_t __xdata *ptr = (uint16_t __xdata *)&nbfi;

        for(int i = 0; i < sizeof(nbfi_settings_t) - 3; i += 2)
        {
            *ptr++ = flash_read(0xfe00 + i);
        }
    }
}



void NBFi_WriteConfig()
{

    uint16_t cal[50];
    uint16_t usr[64];
    for(int i = 0; i < 100; i += 2)
    {
        cal[i/2] = flash_read(0xfc00 + i);
    }
    for(int i = 0; i < 128; i += 2)
    {
        usr[i/2] = flash_read(0xfd00 + i);
    }
    flash_unlock();
    flash_pageerase(0xfc00);

    for(int i = 0; i < 100; i += 2)
    {
        flash_write(0xfc00 + i, cal[i/2]);
    }
    for(int i = 0; i < 128; i += 2)
    {
        flash_write(0xfd00 + i, usr[i/2]);
    }
    uint16_t __xdata *ptr = (uint16_t __xdata *)&nbfi;
    for(int i = 0; i < sizeof(nbfi_settings_t) - 3; i += 2)
    {
        flash_write(0xfe00 + i,*ptr++);
    }
    flash_lock();
}


void NBFi_Config_Set_TX_Chan(nbfi_phy_channel_t ch)
{
    uint8_t i;
    if(nbfi.additional_flags&NBFI_FLG_FIXED_BAUD_RATE) {nbfi.tx_phy_channel = ch; return;}
    for(i = 0; i != NUM_OF_TX_RATES; i++) if(TxRateTable[i] == ch) break;
    if(i == NUM_OF_TX_RATES) return;
    nbfi.tx_phy_channel = ch;
    if(current_tx_rate != i)
    {
        current_tx_rate = i;
        nbfi_state.aver_tx_snr = 15;
    }
 }

void NBFi_Config_Set_RX_Chan(nbfi_phy_channel_t ch)
{
    uint8_t i;
    if(nbfi.additional_flags&NBFI_FLG_FIXED_BAUD_RATE) {nbfi.rx_phy_channel = ch; return;}
    for(i = 0; i != NUM_OF_RX_RATES; i++) if(RxRateTable[i] == ch) break;
    if(i == NUM_OF_RX_RATES) return;
    nbfi.rx_phy_channel = ch;
    if(current_rx_rate != i)
    {
        current_rx_rate = i;
        nbfi_state.aver_rx_snr = 15;
    }
}

bool NBFi_Is_Mode_Normal()
{
    return (nbfi.tx_phy_channel != UL_PSK_FASTDL)&&(!CheckTask(&nbfi_send_mode_delay));
}


