#include <ax8052f143.h>
#include <libmftypes.h>
#include <libmfcrc.h>
#include <stdlib.h>

#include "nbfi.h"
#include "nbfi_config.h"
#include "rf.h"
#include "xtea.h"
#include "zigzag.h"
#include "hal.h"


/* PHYSICAL LAYER */


void DL_Receive_Timeout_cb(struct wtimer_desc __xdata *desc);

uint32_t __xdata last_pkt_crc = 0;

const uint8_t __code protD_preambula[] = {0x97, 0x15, 0x7A, 0x6F};
const uint8_t __code protocolC_preambula[] = {0x97, 0x15, 0x7A};


uint8_t __xdata ul_buf[64];

uint32_t __xdata tx_freq, rx_freq;



nbfi_status_t NBFi_TX_ProtocolD(nbfi_transport_packet_t __generic* pkt)
{
    uint8_t len = 0;
    static bool parity = 0;
    uint8_t lastcrc8;
    bool downlink;


    /* Prepare packet */
    memset_xdata(ul_buf,0,sizeof(ul_buf));

    if(nbfi.mode == TRANSPARENT) pkt->phy_data_length--;

    for(int i=0; i<sizeof(protD_preambula); i++)
    {
        ul_buf[len++] = protD_preambula[i];
    }

    switch(nbfi.tx_phy_channel)
    {
        case DL_DBPSK_50_PROT_D:
        case DL_DBPSK_400_PROT_D:
        case DL_DBPSK_3200_PROT_D:
        case DL_DBPSK_25600_PROT_D:
            ul_buf[len++] = nbfi.dl_ID[0];
            ul_buf[len++] = nbfi.dl_ID[1];
            ul_buf[len++] = nbfi.dl_ID[2];
            downlink = 1;
            break;
        default:
            ul_buf[len++] = nbfi.temp_ID[0];
            ul_buf[len++] = nbfi.temp_ID[1];
            ul_buf[len++] = nbfi.temp_ID[2];
            downlink = 0;
            break;

    }

    if(nbfi.tx_phy_channel == DL_DBPSK_50_PROT_D) nbfi.tx_phy_channel = UL_DBPSK_50_PROT_D;
    else if(nbfi.tx_phy_channel == DL_DBPSK_400_PROT_D) nbfi.tx_phy_channel = UL_DBPSK_400_PROT_D;
    else if(nbfi.tx_phy_channel == DL_DBPSK_3200_PROT_D) nbfi.tx_phy_channel = UL_DBPSK_3200_PROT_D;
    else if(nbfi.tx_phy_channel == DL_DBPSK_25600_PROT_D) nbfi.tx_phy_channel = UL_DBPSK_25600_PROT_D;

    ul_buf[len++] = pkt->phy_data.header;

    memcpy_xdatageneric(&ul_buf[len], pkt->phy_data.payload, pkt->phy_data_length);

    lastcrc8 =  CRC8(&ul_buf[len], 8);

    if(XTEA_Enabled() && XTEA_Available() && !(nbfi.additional_flags&NBFI_FLG_NO_XTEA))
    {
        XTEA_Encode(&ul_buf[len]);
    }
    len += 8;

    if(nbfi.mode == TRANSPARENT)
    {
        ul_buf[len++] = pkt->phy_data.payload[8];
    }
    else  ul_buf[len++] = lastcrc8;

    last_pkt_crc = crc_crc32_msb(ul_buf + 4, 13, 0xFFFFFFFF) ^ 0xFFFFFFFF;

    ul_buf[len++] = (uint8_t)(last_pkt_crc >> 16);
    ul_buf[len++] = (uint8_t)(last_pkt_crc >> 8);
    ul_buf[len++] = (uint8_t)(last_pkt_crc);

    if(nbfi.tx_freq)
    {
        tx_freq = nbfi.tx_freq ;
        parity = (nbfi.tx_freq > (nbfi.ul_freq_base + 25000));
    }
    else
    {
        if(nbfi.tx_phy_channel < UL_DBPSK_3200_PROT_D)
        {
                tx_freq = nbfi.ul_freq_base + (((*((const uint32_t __code*)FULL_ID)+lastcrc8)%226)*100);//tx_freq = NBFI_UL_FREQ_BASE + (((*((const uint32_t __code*)FULL_ID)+lastcrc8)%226)*100);
                if(parity) tx_freq = tx_freq + 27500;
        }
        else
        {
            tx_freq = nbfi.ul_freq_base + 1600 + (((*((const uint32_t __code*)FULL_ID)+lastcrc8)%210)*100);//tx_freq = NBFI_UL_FREQ_BASE + (((*((const uint32_t __code*)FULL_ID)+lastcrc8)%226)*100);
            if(parity) tx_freq = tx_freq + 27500 - 1600;
        }
    }



    if((nbfi.tx_phy_channel < UL_DBPSK_3200_PROT_D) && !downlink)
    {
                ZIGZAG_Append(&ul_buf[4], &ul_buf[len], parity);
    }
    else
    {
                ZIGZAG_Append(&ul_buf[4], &ul_buf[len], 1);
    }

    if(!nbfi.tx_freq) parity = !parity;


    if((nbfi.mode == NRX) && parity)
    {
        RF_Init(nbfi.tx_phy_channel, nbfi.tx_antenna, nbfi.tx_pwr, tx_freq);
        RF_Transmit(ul_buf, len + ZIGZAG_LEN, PADDING_4TO1, BLOCKING);
        nbfi_state.UL_total++;
        return NBFi_TX_ProtocolD(pkt);
    }

    RF_Init(nbfi.tx_phy_channel, nbfi.tx_antenna, nbfi.tx_pwr, tx_freq);
    RF_Transmit(ul_buf, len + ZIGZAG_LEN, PADDING_4TO1, NONBLOCKING);
    nbfi_state.UL_total++;

    return OK;

}



bool NBFi_Match_ID(uint8_t __generic * addr)
{
    uint8_t i;
    for( i = 0; i !=3; i++) if(nbfi.temp_ID[i] != addr[i]) break;
    if(i == 3)  return true;

    for(i = 0; i !=3; i++) if(nbfi.broadcast_ID[i] != addr[i]) break;
    if(i == 3)  return true;

    return false;
}

nbfi_status_t NBFi_TX(nbfi_transport_packet_t __generic* pkt)
{
    if((pkt->phy_data_length==0)&&(pkt->phy_data_length>240)) return ERR; // len check
    switch(nbfi.tx_phy_channel)
    {
        case UL_DBPSK_50_PROT_D:
        case UL_DBPSK_400_PROT_D:
        case UL_DBPSK_3200_PROT_D:
        case UL_DBPSK_25600_PROT_D:
        case DL_DBPSK_50_PROT_D:
        case DL_DBPSK_400_PROT_D:
        case DL_DBPSK_3200_PROT_D:
        case DL_DBPSK_25600_PROT_D:
            return NBFi_TX_ProtocolD(pkt);
        default:
            break;
    }
    return OK;
}


nbfi_status_t NBFi_RX_Controller()
{
    switch(nbfi.mode)
    {
    case  DRX:
    case  NRX:
        if(wait_RxEnd ) if(rf_state != STATE_RX)return NBFi_RX();
        else break;
        switch(nbfi_active_pkt->state)
        {
        case PACKET_WAIT_ACK:
        case PACKET_QUEUED_AGAIN:
        case PACKET_WAIT_FOR_EXTRA_PACKETS:
            if(rf_state != STATE_RX) return NBFi_RX();
            break;
        default:
            if((rf_state != STATE_OFF)&&(nbfi.rx_phy_channel != UL_CARRIER))
            return RF_Deinit();
        }
        break;
    case CRX:
    case TRANSPARENT:
        if(rf_state != STATE_RX) return NBFi_RX();
    }
}


nbfi_status_t NBFi_RX()
{
    nbfi_status_t result;
    switch(nbfi.rx_phy_channel)
    {
        case DL_PSK_200:
        case DL_PSK_500:
        case DL_PSK_5000:
        case DL_PSK_FASTDL:
             if(nbfi.rx_freq == 0) rx_freq = nbfi.dl_freq_base + ((*((const uint32_t __code*)FULL_ID)%276)*363);
             else rx_freq = nbfi.rx_freq;
             if(nbfi.rx_phy_channel == DL_PSK_FASTDL) rx_freq += 1000000;
             break;
        case UL_CARRIER:
            return OK;


    }
    result = RF_Init(nbfi.rx_phy_channel, nbfi.rx_antenna, 0, rx_freq);
    return result;
}



void NBFi_XTEA_OFB(uint8_t __xdata* buf, uint8_t len, uint8_t iter)
{
 uint8_t vector[8];
 for(uint8_t i = 0; i != 3; i++)
 {
    vector[i] = 0;
    vector[i+5] = nbfi.temp_ID[i];
 }
 vector[3] = 0;
 vector[4] = iter;

 uint8_t n = 0;// number of cyphered bytes

 while(n < len)
 {

  if((n % 8) == 0) XTEA_Encode(vector); // next block

  buf[n] = vector[n%8] ^ buf[n];
  n++;
 }
}
