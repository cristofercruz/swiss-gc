/***************************************************************************
# SD Read code for GC/Wii via SDGecko
# emu_kidid 2007-2012
#**************************************************************************/

#include "../../reservedarea.h"
#include "../base/common.h"

#define EXI_READ			0
#define EXI_WRITE			1

//CMD12 - Stop multiple block read command
#define CMD12				0x4C
//CMD18 - Read multiple block command
#define CMD18				(0x52)
//CMD24 - Write single block command 
#define CMD24				(0x58)

#define SECTOR_SIZE 		512
#define exi_freq  			(*(vu32*)VAR_EXI_FREQ)
// exi_channel is stored as number of u32's to index into the exi bus (0xCC006800)
#define exi_channel 		(*(vu32*)VAR_EXI_SLOT)

// EXI Functions
static inline void exi_select()
{
	volatile unsigned long* exi = (volatile unsigned long*)0xCC006800;
	exi[exi_channel] = (exi[exi_channel] & 0x405) | ((1<<0)<<7) | (exi_freq << 4);
}

static inline void exi_deselect()
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

u32 exi_imm_read(int len)
{
	volatile unsigned long* exi = (volatile unsigned long*)0xCC006800;
	exi[exi_channel + 4] = -1;
	// Tell EXI if this is a read or a write
	exi[exi_channel + 3] = ((len - 1) << 4) | (EXI_READ << 2) | 1;
	// Wait for it to do its thing
	while (exi[exi_channel + 3] & 1);

	// Read the 4 byte data off the EXI bus
	u32 d = exi[exi_channel + 4];
	if(len == 4) return d;
	return (d >> ((4 - len) * 8));
	
}

// SD Functions
#define rcvr_spi() ((u8)(exi_imm_read(1) & 0xFF))

void send_cmd(u32 cmd, u32 sector) {
	exi_deselect();
	exi_select();
	rcvr_spi();

	while(rcvr_spi() != 0xFF);
	
	exi_imm_write(cmd<<24, 1);
	exi_imm_write(sector, 4);
	exi_imm_write(1<<24, 1);
	if (cmd == CMD12) rcvr_spi();

	int timeout = 10;
	while((rcvr_spi() & 0x80) && timeout) {
		timeout--;
	}
}

void exi_read_to_buffer(void *dest, u32 len) {
	u32 *destu = (u32*)dest;
	while(len>4) {
		u32 read = exi_imm_read(4);
		if(dest) *destu++ = read;
		len-=4;
	}
	u8 *destb = (u8*)destu;
	while(len) {
		u8 read = rcvr_spi();
		if(dest) *destb++ = read;
		len--;
	}
}

void rcvr_datablock(void *dest, u32 start_byte, u32 bytes_to_read) {
#ifdef DEBUG_VERBOSE
		usb_sendbuffer_safe(" dst: ",6);
		print_int_hex((u32)dest);
		usb_sendbuffer_safe(" start_byte: ",13);
		print_int_hex(start_byte);
		usb_sendbuffer_safe(" bytes_to_read: ",17);
		print_int_hex(bytes_to_read);
#endif
	while(rcvr_spi() != 0xFE);

	// Skip the start if it's a misaligned read
	exi_read_to_buffer(0, start_byte);
	
	// Read however much we need to in this block
	exi_read_to_buffer(dest, bytes_to_read);
	
	// Read out the rest from the SD as we've requested it anyway and can't have it hanging off the bus
	u32 remainder = SECTOR_SIZE - (start_byte+bytes_to_read);
	if(remainder) {
		exi_read_to_buffer((void*)VAR_SECTOR_BUF, remainder);
	}
	exi_imm_read(2);		// discard CRC
#ifdef DEBUG_VERBOSE
	usb_sendbuffer_safe(" u32: ",5);
	print_int_hex(*(u32*)((u32)dest));
	usb_sendbuffer_safe(" u32: ",5);
	print_int_hex(*(u32*)(((u32)dest)+4));
	usb_sendbuffer_safe("\r\n",2);
#endif
}

void do_read(void *dst, u32 len, u32 offset, u32 sectorLba) {
	u32 lba = (offset>>9) + sectorLba;
	u32 startByte = (offset%SECTOR_SIZE);
	u32 numBytes = len;
	u8 lbaShift = *(vu8*)VAR_SD_SHIFT;
	vu16 *lastSectorOffset = (vu16*)VAR_SD_LASTOFS;
	vu32 *prevLba = (vu32*)VAR_TMP1;
	vu32 *readInProgress = ((vu32*)VAR_TMP2);
	// Must be same LBA + up to the right increment in the sector buffer
	u32 resume = (lba == *prevLba) && (*lastSectorOffset == startByte);
	
	// SDHC uses sector addressing, SD uses byte
	lba <<= lbaShift;
	
	// If we can't resume, issue a fresh CMD18 (read)
	if(!resume) {
		// If we were reading previously then issue a CMD12 (stop)
		if(*readInProgress) {
			// End the read by sending CMD12 + Deselect SD + Burn a cycle after it
			send_cmd(CMD12, 0);
			exi_deselect();
			rcvr_spi();
		}
		// Send multiple block read command and the LBA we want to start reading at
		send_cmd(CMD18, lba);
		*readInProgress = 1;
#ifdef DEBUG_VERBOSE
		usb_sendbuffer_safe("lba: ",5);
		print_int_hex(lba);
		usb_sendbuffer_safe("was: ",5);
		print_int_hex(*prevLba);
		usb_sendbuffer_safe("sec: ",5);
		print_int_hex(*lastSectorOffset);
		usb_sendbuffer_safe("now: ",5);
		print_int_hex(startByte);
		usb_sendbuffer_safe("\r\n",2);
#endif
	}
	lba >>= lbaShift;
		
	// Read block crossing a boundary (1..->512)
	if(startByte) {
		u32 amountInBlock = (len + startByte > SECTOR_SIZE) ? SECTOR_SIZE-startByte : len;
		if(resume) {
			*lastSectorOffset += amountInBlock;
			u8* sectorbuf = (u8*)VAR_SECTOR_BUF;
			u8* destb = (u8*)dst;
			while(amountInBlock) {
				*destb++ = *sectorbuf++;
				numBytes--;
				dst++;
				amountInBlock--;
			}
		}
		else {
			// amount to read in first block may vary if our read is small enough to fit in it
			rcvr_datablock(dst,startByte, amountInBlock);
			numBytes-=amountInBlock;
			dst+=amountInBlock;
			*lastSectorOffset = amountInBlock+startByte;
		}
		lba++;
	}
	
	// Read any full, aligned blocks (0->512)
	u32 numFullBlocks = numBytes>>9; 
	numBytes -= numFullBlocks << 9;
	lba+=numFullBlocks;
	while(numFullBlocks) {
		rcvr_datablock(dst, 0, SECTOR_SIZE);
		dst+=SECTOR_SIZE; 
		numFullBlocks--;
		*lastSectorOffset = 0;
	}
	
	// Read any trailing half block (0->..511)
	if(numBytes) {
		rcvr_datablock(dst,0, numBytes);
		dst+=numBytes;
		*lastSectorOffset = numBytes;
	}
	
	// Save current LBA
	*prevLba = lba;
}

/* End of SD functions */
