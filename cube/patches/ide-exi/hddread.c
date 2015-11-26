/***************************************************************************
* HDD Read code for GC/Wii via IDE-EXI
* emu_kidid 2010-2012
***************************************************************************/

#include "../../reservedarea.h"

// NOTE: cs0 then cs1!
// ATA registers address        val  - cs0 cs1 a2 a1 a0
#define ATA_REG_DATA			0x10	//1 0000b
#define ATA_REG_COMMAND			0x17    //1 0111b 
#define ATA_REG_DEVICE			0x16	//1 0110b
#define ATA_REG_ERROR			0x11	//1 0001b
#define ATA_REG_LBAHI			0x15	//1 0101b
#define ATA_REG_LBAMID			0x14	//1 0100b
#define ATA_REG_LBALO			0x13	//1 0011b
#define ATA_REG_SECCOUNT		0x12	//1 0010b
#define ATA_REG_STATUS			0x17	//1 0111b

// ATA commands
#define ATA_CMD_READSECT		0x21
#define ATA_CMD_READSECTEXT		0x24

// ATA head register bits
#define ATA_HEAD_USE_LBA	0x40

// ATA Status register bits we care about
#define ATA_SR_BSY		0x80
#define ATA_SR_DRQ		0x08
#define ATA_SR_ERR		0x01

#define EXI_READ					0			/*!< EXI transfer type read */
#define EXI_WRITE					1			/*!< EXI transfer type write */

#define MIN(x,y) ((x) < (y) ? (x) : (y))
#define SECTOR_SIZE 		512

typedef unsigned int u32;
typedef unsigned short u16;
typedef unsigned char u8;

#define _ideexi_version *(u32*)VAR_TMP2
#define _ata48bit *(u32*)VAR_TMP1
#define last_read_dst *(u32*)VAR_LAST_DMA

#define IDE_EXI_V1 0
#define IDE_EXI_V2 1

#define exi_freq  			(*(u32*)VAR_EXI_FREQ)
// exi_channel is stored as number of u32's to index into the exi bus (0xCC006800)
#define exi_channel 		(*(u32*)VAR_EXI_SLOT)

void* mymemcpy(void* dest, const void* src, u32 count)
{
	char* tmp = (char*)dest,* s = (char*)src;

	while (count--)
		*tmp++ = *s++;

	return dest;
}

void exi_select()
{
	volatile unsigned long* exi = (volatile unsigned long*)0xCC006800;
	exi[exi_channel] = (exi[exi_channel] & 0x405) | ((1<<0)<<7) | (exi_freq << 4);
}

void exi_deselect()
{
	volatile unsigned long* exi = (volatile unsigned long*)0xCC006800;
	exi[exi_channel] &= 0x405;
}

void exi_imm_write(u32 data, int len) 
{
	volatile unsigned long* exi = (volatile unsigned long*)0xCC006800;
	exi[exi_channel + 4] = data;
	// Tell EXI if this is a read or a write
	exi[exi_channel + 3] = ((len - 1) << 4) | (EXI_WRITE << 2) | 1;
	// Wait for it to do its thing
	while (exi[exi_channel + 3] & 1);
}

u32 exi_imm_read()
{
	volatile unsigned long* exi = (volatile unsigned long*)0xCC006800;
	exi[exi_channel + 4] = -1;
	// Tell EXI if this is a read or a write
	exi[exi_channel + 3] = ((4 - 1) << 4) | (EXI_READ << 2) | 1;
	// Wait for it to do its thing
	while (exi[exi_channel + 3] & 1);
	return exi[exi_channel + 4];
}

void exi_dma(void* data, int len, int mode, int sync)
{
	if(!sync)
		last_read_dst = (u32)data;
	volatile unsigned long* exi = (volatile unsigned long*)0xCC006800;

	exi[exi_channel + 1] = (unsigned long)data;
	exi[exi_channel + 2] = len;
	exi[exi_channel + 3] = (mode << 2) | 3;
	while(sync && (exi[exi_channel + 3] & 1)); // block if sync
}

// Returns 8 bits from the ATA Status register
u8 ataReadStatusReg()
{
	// read ATA_REG_CMDSTATUS1 | 0x00 (dummy)
	u8 dat;
	exi_select();
	exi_imm_write(0x17000000, 2);
	dat=exi_imm_read()>>24;
	exi_deselect();
	return dat;
}

// Writes 8 bits of data out to the specified ATA Register
void ataWriteByte(u8 addr, u8 data)
{
	exi_select();
	exi_imm_write(0x80000000 | (addr << 24) | (data<<16), 3);
	exi_deselect();
}

// Reads 512 bytes from the ide-exi
void ata_read_buffer(u8 *dst, int sync) 
{
	u32* alignedBuf = (u32*)0x80002B00;	// TODO this buffer is gone!
	u32 i = 0;
	u32 *ptr = (u32*)dst;
	u16 dwords = 128;
	// (31:29) 011b | (28:24) 10000b | (23:16) <num_words_LSB> | (15:8) <num_words_MSB> | (7:0) 00h (4 bytes)
	exi_select();
	exi_imm_write(0x70000000 | ((dwords&0xff) << 16) | (((dwords>>8)&0xff) << 8), 4);
	
	if(!_ideexi_version) {
		exi_deselect();
		for(i = 0; i < dwords; i++) {
			exi_select();
			*ptr++ = exi_imm_read();
			exi_deselect();
		}
		exi_select();
		exi_imm_read();
	}
	else {	// v2, no deselect or extra read required.
		u32 *ptr = (u32*)dst;
		if(((u32)dst)%32) {
			ptr = alignedBuf;
		}
		dcache_flush_icache_inv(ptr, 512);
		exi_dma(ptr, 512, EXI_READ, sync);
		if(sync) {
			// It wasn't aligned, copy it.
			if(((u32)dst)%32) {
				mymemcpy(dst, ptr, 512);
			}
		}
	}
	if(sync) {
		exi_deselect();
	}
}

// Reads sectors from the specified lba, for the specified slot, 511 sectors at a time max for LBA48 drives
// Returns 0 on success, -1 on failure.
void _ataReadSector(u32 lba, void *buffer, int sync)
{
	u32 temp = 0;
  	
  	// Wait for drive to be ready (BSY to clear)
	while(ataReadStatusReg() & ATA_SR_BSY);

	// Select the device (ATA_HEAD_USE_LBA is 0x40 for master, 0x50 for slave)

	// Non LBA48
	if(!_ata48bit) {
		ataWriteByte(ATA_REG_DEVICE, 0xE0 | (u8)((lba >> 24) & 0x0F));
		ataWriteByte(ATA_REG_SECCOUNT, 1);								// Sector count (Lo)
		ataWriteByte(ATA_REG_LBALO, (u8)(lba & 0xFF));					// LBA 1
		ataWriteByte(ATA_REG_LBAMID, (u8)((lba>>8) & 0xFF));			// LBA 2
		ataWriteByte(ATA_REG_LBAHI, (u8)((lba>>16) & 0xFF));			// LBA 3
		ataWriteByte(ATA_REG_COMMAND, ATA_CMD_READSECT);
	}
	// LBA48
	else {
		ataWriteByte(ATA_REG_DEVICE, ATA_HEAD_USE_LBA);
		ataWriteByte(ATA_REG_SECCOUNT, 0);								// Sector count (Hi)
		ataWriteByte(ATA_REG_LBALO, (u8)((lba>>24)& 0xFF));				// LBA 4
		ataWriteByte(ATA_REG_LBAMID, 0);								// LBA 5
		ataWriteByte(ATA_REG_LBAHI, 0);									// LBA 6
		ataWriteByte(ATA_REG_SECCOUNT, 1);								// Sector count (Lo)
		ataWriteByte(ATA_REG_LBALO, (u8)(lba & 0xFF));					// LBA 1
		ataWriteByte(ATA_REG_LBAMID, (u8)((lba>>8) & 0xFF));			// LBA 2
		ataWriteByte(ATA_REG_LBAHI, (u8)((lba>>16) & 0xFF));			// LBA 3
		ataWriteByte(ATA_REG_COMMAND, ATA_CMD_READSECTEXT );
	}
	// Wait for BSY to clear
	while((temp = ataReadStatusReg()) & ATA_SR_BSY);

	// Wait for drive to request data transfer
	while(!(ataReadStatusReg() & ATA_SR_DRQ));
	
	// request to read data from drive but return to the game
	ata_read_buffer(buffer, sync);
	
	if(sync) {
		ataReadStatusReg();	// Burn one byte
	}
}

int do_read(void *dst,u32 size, u32 offset, u32 sectorLba) {

	// See if we've been called after a DMA has been fired off
	if(last_read_dst) {
		/*usb_sendbuffer_safe("\r\nlast: ", 8);
		print_int_hex(last_read_dst);
		usb_sendbuffer_safe("\n", 2);*/
		volatile unsigned long* exi = (volatile unsigned long*)0xCC006800;
		if (exi[exi_channel + 3] & 1) return 0;
		exi_deselect();
		ataReadStatusReg();	// Burn one byte
		// It wasn't aligned, copy it.
		if(((u32)dst)%32) {
			mymemcpy(dst, (u32*)(last_read_dst), 512);
		}
		last_read_dst = 0;
		return 512;
	}
	
	u8 sector[SECTOR_SIZE] __attribute__((aligned(32)));
	u32 lba = (offset>>9) + sectorLba;
	// Read any half sector if we need to until we're aligned
	if(offset % SECTOR_SIZE) {
		u32 size_to_copy = MIN(size, SECTOR_SIZE-(offset%SECTOR_SIZE));
		_ataReadSector(lba, sector, 1);
		mymemcpy(dst, &(sector[offset%SECTOR_SIZE]), size_to_copy);
		return size_to_copy;
	}
	// Read any whole sectors (async)
	if(size >= 512) {
		_ataReadSector(lba, dst, 0);
		//usb_sendbuffer_safe("\r\nread\r\n", 8);
		return 0;
	}
	// Read the last sector if there's any half sector
	if(size) {
		_ataReadSector(lba, sector, 1);
		mymemcpy(dst, &(sector[0]), size);
		return size;
	}
	return 0;
}


