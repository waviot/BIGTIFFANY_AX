#include "nbfi.h"
#include "nbfi_config.h"
#include <stdlib.h>
#include "hal.h"


#define POLY 0xa001


nbfi_transport_packet_t  __xdata* nbfi_TX_pktBuf[NBFI_TX_PKTBUF_SIZE];
nbfi_transport_packet_t __xdata* nbfi_RX_pktBuf[NBFI_RX_PKTBUF_SIZE];


uint8_t     nbfi_TXbuf_head = 0;


extern uint8_t  __xdata string[50];

nbfi_transport_packet_t __xdata* NBFi_AllocateTxPkt(uint8_t payload_length)
{
    uint8_t ptr = nbfi_TXbuf_head%NBFI_TX_PKTBUF_SIZE;

    if(nbfi_TX_pktBuf[ptr])
    {
        switch(nbfi_TX_pktBuf[ptr]->state)
        {
        case PACKET_QUEUED:
        case PACKET_QUEUED_AGAIN:
        case PACKET_WAIT_ACK:
        case PACKET_NEED_TO_SEND_RIGHT_NOW:
        case PACKET_SENT_NOACKED:
            return 0;   // tx buffer is full
        }
        free(nbfi_TX_pktBuf[ptr]);
        nbfi_TX_pktBuf[ptr] = 0;
    }

    nbfi_TX_pktBuf[ptr] = (nbfi_transport_packet_t  __xdata*) malloc(sizeof(nbfi_transport_packet_t) + payload_length);

    if(!nbfi_TX_pktBuf[ptr])
    {
         return 0;
    }

    nbfi_TX_pktBuf[ptr]->state = PACKET_ALLOCATED;

    nbfi_TX_pktBuf[ptr]->phy_data_length = payload_length;

    nbfi_TX_pktBuf[ptr]->handshake = HANDSHAKE_NONE;

    nbfi_TX_pktBuf[ptr]->retry_num = 0;

    nbfi_TX_pktBuf[ptr]->mack_num = 0;

    nbfi_TX_pktBuf[ptr]->phy_data.header = 0;

    nbfi_TXbuf_head++;

    return nbfi_TX_pktBuf[ptr];

}



nbfi_transport_packet_t __xdata* NBFi_AllocateRxPkt(uint8_t header, uint8_t payload_length)
{
    uint8_t ptr = header&0x1f;

    switch(nbfi_active_pkt->state)
    {
        case PACKET_QUEUED:
        case PACKET_QUEUED_AGAIN:
        case PACKET_WAIT_ACK:
            nbfi_active_pkt = &idle_pkt;
            break;

    }

    if(nbfi_RX_pktBuf[ptr])
    {
        free(nbfi_RX_pktBuf[ptr]);
    }


    nbfi_RX_pktBuf[ptr] = (nbfi_transport_packet_t  __xdata*) malloc(sizeof(nbfi_transport_packet_t) + payload_length);

    if(!nbfi_RX_pktBuf[ptr])
    {
        return 0;
    }

    nbfi_state.DL_iter = ptr;

    nbfi_RX_pktBuf[ptr]->state = PACKET_ALLOCATED;

    nbfi_RX_pktBuf[ptr]->phy_data_length = payload_length;

    nbfi_RX_pktBuf[ptr]->phy_data.header = header;

    return nbfi_RX_pktBuf[ptr];

}


nbfi_transport_packet_t __xdata* NBFi_GetQueuedTXPkt()
{

    for(uint8_t i = nbfi_TXbuf_head - NBFI_TX_PKTBUF_SIZE; i != nbfi_TXbuf_head; i++)
    {
        uint8_t ptr = i%NBFI_TX_PKTBUF_SIZE;
        if(nbfi_TX_pktBuf[ptr] == 0) continue;
        switch(nbfi_TX_pktBuf[ptr]->state )
        {
        case PACKET_QUEUED_AGAIN:
        case PACKET_NEED_TO_SEND_RIGHT_NOW:
            return nbfi_TX_pktBuf[ptr];
        }
    }

    for(uint8_t i = nbfi_TXbuf_head - NBFI_TX_PKTBUF_SIZE; i != nbfi_TXbuf_head; i++)
    {
        uint8_t ptr = i%NBFI_TX_PKTBUF_SIZE;
        if(nbfi_TX_pktBuf[ptr] == 0) continue;
        switch(nbfi_TX_pktBuf[ptr]->state )
        {
       case PACKET_WAIT_ACK:
            if(nbfi_TX_pktBuf[ptr] == nbfi_active_pkt) continue;
             nbfi_TX_pktBuf[ptr]->state = PACKET_QUEUED_AGAIN;
        case PACKET_QUEUED:
        case PACKET_QUEUED_AGAIN:
            return nbfi_TX_pktBuf[ptr];
        }
    }
    return 0;
}

void NBFi_TxPacket_Free(nbfi_transport_packet_t __xdata* pkt)
{
    for(uint8_t i = nbfi_TXbuf_head - NBFI_TX_PKTBUF_SIZE; i != nbfi_TXbuf_head; i++)
    {
        uint8_t ptr = i%NBFI_TX_PKTBUF_SIZE;
        if(nbfi_TX_pktBuf[ptr] != pkt) continue;
        free(pkt);
        nbfi_TX_pktBuf[ptr] = 0;
    }

}

void NBFi_RxPacket_Free(nbfi_transport_packet_t __xdata* pkt)
{
    for(uint8_t i = 0; i != NBFI_RX_PKTBUF_SIZE; i++)
    {
        if(nbfi_RX_pktBuf[i] != pkt) continue;
        free(pkt);
        nbfi_RX_pktBuf[i] = 0;
    }

}

uint8_t NBFi_Packets_To_Send()
{

    uint8_t packets_free = 0;

    for(uint16_t i = nbfi_TXbuf_head; i != (nbfi_TXbuf_head + NBFI_TX_PKTBUF_SIZE); i++)
    {
        uint8_t ptr = i%NBFI_TX_PKTBUF_SIZE;
        if(nbfi_TX_pktBuf[ptr] == 0)
        {
            packets_free++;
            continue;
        }
        switch(nbfi_TX_pktBuf[ptr]->state )
        {
       case PACKET_WAIT_ACK:
            if(nbfi_TX_pktBuf[ptr] == nbfi_active_pkt) break;
             nbfi_TX_pktBuf[ptr]->state = PACKET_QUEUED_AGAIN;
        case PACKET_QUEUED:
        case PACKET_QUEUED_AGAIN:
        case PACKET_NEED_TO_SEND_RIGHT_NOW:
        case PACKET_SENT_NOACKED:
            break;
        default:
            packets_free++;
            continue;
        }
        break;
    }

    if((rf_busy == true) && (packets_free == NBFI_TX_PKTBUF_SIZE))
    {
        packets_free--;
    }

   return NBFI_TX_PKTBUF_SIZE - packets_free;
}

uint8_t NBFi_Mark_Lost_All_Unacked()
{
    for(uint8_t i = nbfi_TXbuf_head - NBFI_TX_PKTBUF_SIZE; i != nbfi_TXbuf_head; i++)
    {
        uint8_t ptr = i%NBFI_TX_PKTBUF_SIZE;
        if(nbfi_TX_pktBuf[ptr] == 0) continue;
        if(nbfi_TX_pktBuf[ptr]->state == PACKET_SENT_NOACKED) nbfi_TX_pktBuf[ptr]->state = PACKET_LOST;
    }
    return 0;
}

uint8_t NBFi_Calc_Packets_With_State(uint8_t state)
{
    uint8_t num = 0;
    for(uint8_t i = nbfi_TXbuf_head - NBFI_TX_PKTBUF_SIZE; i != nbfi_TXbuf_head; i++)
    {
        uint8_t ptr = i%NBFI_TX_PKTBUF_SIZE;
        if(nbfi_TX_pktBuf[ptr] == 0) continue;
        if(nbfi_TX_pktBuf[ptr]->state == state) num++;
    }
    return num;
}

uint8_t NBFi_Calc_Queued_Sys_Packets_With_Type(uint8_t type)
{
    uint8_t num = 0;
    for(uint8_t i = nbfi_TXbuf_head - NBFI_TX_PKTBUF_SIZE; i != nbfi_TXbuf_head; i++)
    {
        uint8_t ptr = i%NBFI_TX_PKTBUF_SIZE;
        if(nbfi_TX_pktBuf[ptr] == 0) continue;
        if(!nbfi_TX_pktBuf[ptr]->phy_data.SYS) continue;
        if(nbfi_TX_pktBuf[ptr]->phy_data.payload[0] != type) continue;

        switch(nbfi_TX_pktBuf[ptr]->state )
        {
        case PACKET_WAIT_ACK:
        case PACKET_QUEUED:
        case PACKET_QUEUED_AGAIN:
        case PACKET_NEED_TO_SEND_RIGHT_NOW:
        case PACKET_SENT_NOACKED:
            num++;
            break;
        }
    }
    return num;
}



nbfi_transport_packet_t __xdata* NBFi_GetSentTXPkt_By_Iter(uint8_t iter)
{

    for(uint8_t i = (nbfi_TXbuf_head) - 1; i != (uint8_t)(nbfi_TXbuf_head  - NBFI_TX_PKTBUF_SIZE - 1); i--)
    {
        uint8_t ptr = i%NBFI_TX_PKTBUF_SIZE;
        if(nbfi_TX_pktBuf[ptr] == 0) continue;
        if(nbfi_TX_pktBuf[ptr]->phy_data.SYS && (nbfi_TX_pktBuf[ptr]->phy_data.payload[0] != 0x05)&&(nbfi_TX_pktBuf[ptr]->phy_data.payload[0] != 0x02)&& !(nbfi_TX_pktBuf[ptr]->phy_data.payload[0] & 0x80))
        {
            continue;
        }
        switch(nbfi_TX_pktBuf[ptr]->state)
        {
        case PACKET_SENT:
        case PACKET_SENT_NOACKED:
        case PACKET_WAIT_ACK:
        case PACKET_LOST:
        case PACKET_DELIVERED:
            if(nbfi_TX_pktBuf[ptr]->phy_data.ITER == iter)
            {
               return nbfi_TX_pktBuf[ptr];
            }
            break;
        }

    }
    return 0;
}


uint32_t NBFi_Get_RX_ACK_Mask()
{
    uint32_t mask = 0;
    uint32_t one = 1;
    for(uint8_t i = ((nbfi_state.DL_iter - 1)&0x1f); (i&0x1f) != (nbfi_state.DL_iter&0x1f); i-- , one <<= 1 )
    {
        if(!nbfi_RX_pktBuf[i&0x1f]) continue;
        switch(nbfi_RX_pktBuf[i&0x1f]->state)
        {
            case PACKET_RECEIVED:
            case PACKET_PROCESSED:
                mask |= one;
                break;
            default:
                break;
        }
    }
    return mask;
}

bool NBFi_Check_RX_Packet_Duplicate(nbfi_pfy_packet_t __xdata * pkt, uint8_t len)
{
    if(nbfi.mode == HANDSHAKE_NONE) return false;
    nbfi_pfy_packet_t __xdata *rec_pkt = &nbfi_RX_pktBuf[nbfi_state.DL_iter&0x1f]->phy_data;

    for(uint8_t i = 0; i != len; i++)
    {
        if(((uint8_t __xdata*)rec_pkt)[i] != ((uint8_t __xdata*)pkt)[i]) return false;
    }
    return true;
}


nbfi_transport_packet_t __xdata* NBFi_Get_QueuedRXPkt(uint8_t *groupe, uint16_t *total_length)
{
    nbfi_transport_packet_t __xdata* pkt;

    uint32_t i;
    for(i = nbfi_state.DL_iter + 1; i <= (nbfi_state.DL_iter + NBFI_RX_PKTBUF_SIZE*2); i++ )
    {
        *groupe = 0;
        *total_length = 0;
        uint8_t total_groupe_len = 0;

        while((pkt = nbfi_RX_pktBuf[(i + *groupe)&0x1f]) && pkt->state == PACKET_RECEIVED)
        {
            if((*groupe) == 0)
            {
                *groupe = 1;

                if((pkt->phy_data.MULTI)&&(pkt->phy_data.SYS)&&((pkt->phy_data.payload[0] == 0x05)||(pkt->phy_data.payload[0] == 0x02))) //the start packet of the groupe
                {
                    total_groupe_len = pkt->phy_data.payload[1];
                    *total_length = pkt->phy_data_length - 2;
                    continue;
                }
                else
                {
                    if((pkt->phy_data.MULTI == 1)&&!(pkt->phy_data.SYS)) break;
                    //single packet
                    *total_length = pkt->phy_data_length;
                    pkt->state = PACKET_PROCESSING;
                    break;
                }
            }
            (*total_length) += pkt->phy_data_length;
            (*groupe)++;

            if((pkt->phy_data.MULTI) && ((*total_length) < total_groupe_len) && ((*groupe) < NBFI_RX_PKTBUF_SIZE - 1))
            {
                    continue;
            }
            else
            {       if(pkt->phy_data.MULTI && ((*total_length) >= total_groupe_len))
                    {
                        (*total_length) = total_groupe_len;
                        pkt->state = PACKET_PROCESSING;
                        break;
                    }

            }
            break;

        }

        if((*groupe) && (nbfi_RX_pktBuf[(i + (*groupe) - 1)&0x1f]->state == PACKET_PROCESSING))
        {
            return nbfi_RX_pktBuf[i&0x1f];
        }

    }
    return 0;
}

void NBFi_Clear_RX_Buffer()
{
    for(uint8_t i = 0; i != NBFI_RX_PKTBUF_SIZE; i++ )
    {
        nbfi_RX_pktBuf[i]->state = PACKET_CLEARED;
    }
}

extern nbfi_transport_packet_t __xdata idle_pkt;
extern nbfi_transport_packet_t __xdata* __xdata nbfi_active_pkt;
void NBFi_Clear_TX_Buffer()
{
    for(uint8_t i = 0; i != NBFI_TX_PKTBUF_SIZE; i++ )
    {
        if(nbfi_TX_pktBuf[i])
        {
            free(nbfi_TX_pktBuf[i]);
            nbfi_TX_pktBuf[i] = 0;
        }
    }
    nbfi_active_pkt = &idle_pkt;
}


extern uint8_t __xdata nbfi_last_snr;
extern int16_t __xdata noise;
void NBFi_Send_Clear_Cmd(uint8_t iter)
{
    nbfi_transport_packet_t __xdata* pkt =  NBFi_AllocateTxPkt(8);
    if(!pkt) return;
    pkt->phy_data.payload[0] = 0x08; //clear RX buffer
    pkt->phy_data.payload[5] = nbfi_last_snr;
    pkt->phy_data.payload[6] = (uint8_t)(noise + 150);
    pkt->phy_data.payload[7] = you_should_dl_power_step_down + you_should_dl_power_step_up + (nbfi.tx_pwr & 0x3f);
    pkt->phy_data.ITER = iter;
    pkt->phy_data.header |= SYS_FLAG;
    pkt->handshake = HANDSHAKE_NONE;
    pkt->state = PACKET_NEED_TO_SEND_RIGHT_NOW;
}


void NBFi_Resend_Pkt(nbfi_transport_packet_t __xdata* act_pkt, uint32_t mask)
{
    uint8_t iter = act_pkt->phy_data.ITER;
    mask = (~mask) << 1;
    uint32_t selection;
    selection = 0;
    for(uint8_t i = (act_pkt->mack_num&0x3f) - 1; i != 0; i-- )
    {
      selection |= (((uint32_t)1) << i);
    }
    mask &= selection;
    uint32_t one = 1;

    nbfi_transport_packet_t __xdata* pkt = 0, *last_resending_pkt = 0;
    for(uint8_t i = (act_pkt->mack_num&0x3f); i > 0; i--)
    {
        pkt = NBFi_GetSentTXPkt_By_Iter(iter&0x1f);
        if(!pkt)
        {
             break;
        }
        if(one&mask)
        {
           mask &= ~one;
           pkt->state = PACKET_QUEUED_AGAIN;
           if(last_resending_pkt == 0) last_resending_pkt = pkt;
           nbfi_state.fault_total++;
        }
        else
        {
            pkt->state = PACKET_DELIVERED;
            nbfi_state.success_total++;
        }

        iter--;
        one <<= 1;
    }

    if(last_resending_pkt)
    {
        last_resending_pkt->phy_data.ACK = 1;
        last_resending_pkt->mack_num = act_pkt->mack_num - (act_pkt->phy_data.ITER - last_resending_pkt->phy_data.ITER);
        if(act_pkt->phy_data.ITER < last_resending_pkt->phy_data.ITER) last_resending_pkt->mack_num += 32;
        last_resending_pkt->mack_num |= 0x80;
    }
    else if((act_pkt->mack_num > 1) && (mask == 0))  //all packets delivered, send message to clear receiver's input buffer
    {
         NBFi_Send_Clear_Cmd(nbfi_active_pkt->phy_data.ITER);
    }

}



uint16_t CRC16(uint8_t __generic *buf, uint16_t len, uint16_t crc)
{
    while (len--)
    {
        crc ^= *buf++;
        crc = crc & 1 ? (crc >> 1) ^ POLY : crc >> 1;
        crc = crc & 1 ? (crc >> 1) ^ POLY : crc >> 1;
        crc = crc & 1 ? (crc >> 1) ^ POLY : crc >> 1;
        crc = crc & 1 ? (crc >> 1) ^ POLY : crc >> 1;
        crc = crc & 1 ? (crc >> 1) ^ POLY : crc >> 1;
        crc = crc & 1 ? (crc >> 1) ^ POLY : crc >> 1;
        crc = crc & 1 ? (crc >> 1) ^ POLY : crc >> 1;
        crc = crc & 1 ? (crc >> 1) ^ POLY : crc >> 1;
    }
    return crc;
}

static unsigned char CRC8byte(unsigned char __xdata data)
{
   uint8_t crc = 0;
   if(data & 1)     crc ^= 0x5e;
   if(data & 2)     crc ^= 0xbc;
   if(data & 4)     crc ^= 0x61;
   if(data & 8)     crc ^= 0xc2;
   if(data & 0x10)  crc ^= 0x9d;
   if(data & 0x20)  crc ^= 0x23;
   if(data & 0x40)  crc ^= 0x46;
   if(data & 0x80)  crc ^= 0x8c;
   return crc;
}

uint8_t CRC8(uint8_t __generic* data, uint8_t len)
{
    uint8_t crc = 0;
    for(uint8_t i = 0; i < len; i++)
    {
        crc = CRC8byte(data[i] ^ crc);
    }
    return crc;
}


uint16_t NBFi_Phy_To_Bitrate(nbfi_phy_channel_t ch)
{
    switch(ch)
    {
    case DL_PSK_200:
    case UL_PSK_200:
        return 200;
    case DL_PSK_500:
    case UL_PSK_500:
        return 500;
    case DL_PSK_5000:
    case UL_PSK_5000:
        return 5000;
    case DL_PSK_FASTDL:
    case UL_PSK_FASTDL:
        return 57600;
    case UL_DBPSK_50_PROT_C:
    case UL_DBPSK_50_PROT_D:
    case UL_DBPSK_50_PROT_E:
        return 50;
    case UL_DBPSK_400_PROT_C:
    case UL_DBPSK_400_PROT_D:
    case UL_DBPSK_400_PROT_E:
        return 400;
    case UL_DBPSK_3200_PROT_D:
    case UL_DBPSK_3200_PROT_E:
        return 3200;
    case UL_DBPSK_25600_PROT_D:
    case UL_DBPSK_25600_PROT_E:
        return 25600;
    }
}

uint8_t NBFi_Get_TX_Iter()
{
    return nbfi_state.UL_iter&0x1f;
}


