#include "libmfcrc.h"

#define crc_msb_byte     crc_ccitt_msb_byte
#define crc_msbtable     crc_ccitt_msbtable
#define crc_msbtable_asm _crc_ccitt_msbtable

#if defined(SDCC)

__reentrantb uint16_t crc_msb_byte(uint16_t crc, uint8_t c) __reentrant __naked
{
	crc;
	c;
        __asm;
	mov	a,sp
	add	a,#-2
	mov	r0,a
	mov	a,@r0
	xrl	a,dph
	clr	c
	rlc	a
	mov	r2,a
	clr	a
	rlc	a
	mov	r3,a
	mov	a,#crc_msbtable_asm
	add	a,r2
	xch	a,dpl
	mov	r2,a
	mov	a,#(crc_msbtable_asm >> 8)
	addc	a,r3
	mov	dph,a
	mov	a,#1
	movc	a,@a+dptr
	xrl	a,r2
	mov	r2,a
	clr	a
	movc	a,@a+dptr
	mov	dpl,a
	mov	dph,r2
	ret
        __endasm;
}

#else

__reentrantb uint16_t crc_msb_byte(uint16_t crc, uint8_t c) __reentrant
{
        return (crc << 8) ^ crc_msbtable[((uint8_t)(crc >> 8) ^ c) & (uint8_t)0xff];
}

#endif
