#ifndef _DEFINES_H
#define _DEFINES_H

#define NBFI_REV        4
#define NBFI_SUBREV     1

#define UL868800_DL446000            0
#define UL868800_DL864000            1
#define UL868800_DL446000_DL864000   2
#define UL867950_DL446000            3
#define UL868500_DL446000            4
#define UL868100_DL446000            5
#define UL864000_DL446000            6
#define UL863175_DL446000            7
#define UL864000_DL875000            8
#define UL868100_DL869550            9
#define UL868500_DL864000            10


#ifdef BIGTIFFANY_AX
    #define HARDWARE_ID     6
    #define HARDWARE_REV    0
    #define RF_MAX_POWER 12
    #define RF_MIN_POWER 0
    #define BAND_ID         UL868800_DL864000
    #define NO_SLIP
    #define MAGNIUM_BATTERY
#endif


#endif // _DEFINES_H
