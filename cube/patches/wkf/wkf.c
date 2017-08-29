/***************************************************************************
* Wiikey Fusion related patches
* emu_kidid 2015
***************************************************************************/

#include "../../reservedarea.h"
#include "../base/common.h"

extern void print_int_hex(unsigned int num);

void wkfWriteOffset(u32 offset) {
	volatile u32* wkf = (volatile u32*)0xCC006000;
	wkf[0] = 0x54;
	wkf[2] = 0xDE000000;
	wkf[3] = offset;
	wkf[4] = 0x5A000000;
	wkf[6] = 0;
	wkf[8] = 0;
	wkf[7] = 1;
	while( wkf[7] & 1);
}


void wkfRead(void* dst, int len, u32 offset) {
#ifdef DEBUG
	usb_sendbuffer_safe("wkfREAD: dst: ",14);
	print_int_hex((u32)dst);
	usb_sendbuffer_safe(" len: ",6);
	print_int_hex(len);
	usb_sendbuffer_safe(" ofs: ",6);
	print_int_hex(offset);
	usb_sendbuffer_safe("\r\n",2);
#endif
	volatile u32* dvd = (volatile u32*)0xCC006000;
	dvd[0] = 0x54;
	dvd[2] = 0xA8000000;
	dvd[3] = offset >> 2;
	dvd[4] = len;
	dvd[5] = (unsigned long)dst;
	dvd[6] = len;
	dvd[7] = 3; // enable reading!
	while( dvd[7] & 1);
}

void* mymemcpy(void* dest, const void* src, u32 count)
{
#ifdef DEBUG
	usb_sendbuffer_safe("memcpy: src: ",13);
	print_int_hex((u32)src);
	usb_sendbuffer_safe(" dst: ",6);
	print_int_hex((u32)dest);
	usb_sendbuffer_safe(" len: ",6);
	print_int_hex(count);
	usb_sendbuffer_safe("\r\n",2);
#endif
	char* tmp = (char*)dest,* s = (char*)src;

	while (count--)
		*tmp++ = *s++;

	return dest;
}

void do_read(void* dst, u32 len, u32 offset, u32 sectorLba)
{
	void *oDst = dst;
	u32 oLen = len;
#ifdef DEBUG
	usb_sendbuffer_safe("DO READ: dst: ",14);
	print_int_hex((u32)dst);
	usb_sendbuffer_safe(" len: ",6);
	print_int_hex(len);
	usb_sendbuffer_safe(" ofs: ",6);
	print_int_hex(offset);
	usb_sendbuffer_safe("\r\n",2);
#endif
	u32 unalignedBytes = (offset % 4);
	if(unalignedBytes != 0)
	{
		u8 *sector_buffer = (u8*)WKF_SECTOR;
		wkfRead(sector_buffer, WKF_SECTOR_SIZE, (offset-(offset%WKF_SECTOR_SIZE)));
		u32 off = (u32)((u32)(offset) & (WKF_SECTOR_SIZE-1));

		int rl = WKF_SECTOR_SIZE - off;
		if (rl > len)
			rl = len;
		mymemcpy(dst, sector_buffer + off, rl);	

		offset += rl;
		len -= rl;
		dst += rl;
	}
	if(len > 0) {
		wkfRead(dst, len, offset);
	}
	dcache_flush_icache_inv(oDst, oLen);
}

void wkfReinit(void) {
	asm("mfmsr	5");
	asm("rlwinm	5,5,0,17,15");
	asm("mtmsr	5");
	// TODO re-init WKF

	volatile u32* wkf = (volatile u32*)0xCC006000;
	wkf[0] = 0x54;
	wkf[1] = 0;
	wkf[2] = 0xe3000000;
	wkf[3] = 0;
	wkf[4] = 0;
	wkf[6] = 0;
	wkf[8] = 0;
	wkf[7] = 1;
	while( wkf[7] & 1);
}


//void swap_disc() {
//	int isDisc1 = (*(u32*)(VAR_DISC_1_LBA)) == (*(u32*)VAR_CUR_DISC_LBA);
//	*(u32*)VAR_CUR_DISC_LBA = isDisc1 ? *(u32*)(VAR_DISC_2_LBA) : *(u32*)(VAR_DISC_1_LBA);
//}

