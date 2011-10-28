/*
 * 			U-BOOT DM9000A DRIVER
 *			  www.davicom.com.tw
 * 
 * This program is loaded into SRAM in bootstrap mode, where it waits
 * for commands on UART1 to read and write memory, jump to code etc.
 * A design goal for this program is to be entirely independent of the
 * target board.  Anything with a CL-PS7111 or EP7211 should be able to run
 * this code in bootstrap mode.  All the board specifics can be handled on
 * the host.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 *V1.01 -load MAC address from EEPROM
 */

#include <common.h>
#include <command.h>
#include "dm9000a.h" // ghcstop fix
#include <net.h>

#include <asm/io.h>
#include <s3c2416.h>

//#define DEBUG
#    ifdef DEBUG
#        define gprintf(x...) printf(x)
#    else
#        define gprintf(x...)
#    endif


#ifdef CONFIG_DRIVER_DM9000A

#if (CONFIG_COMMANDS & CFG_CMD_NET)
#define DM9000_ID		0x90000A46
#define DM9010_ID		0x90100A46
#define DM9KS_REG05		(RXCR_Discard_LongPkt|RXCR_Discard_CRCPkt) 
#define DM9KS_DISINTR	IMR_SRAM_antoReturn
/*
 * If your bus is 8-bit or 32-bit, you must modify below.
 * Ex. your bus is 8 bit
 *	DM9000_PPTR *(volatile u8 *)(DM9000_BASE)
 */
#define	DM9000_BASE		0x08000000
#define DM9000_PPTR   *(volatile u8 *)(DM9000_BASE)
#define DM9000_PDATA  *(volatile u8 *)(DM9000_BASE + 4)
//#define DM9000_PPTR   *(volatile u16 *)(DM9000_BASE)
//#define DM9000_PDATA  *(volatile u16 *)(DM9000_BASE + 4)

#define mdelay(n)       udelay((n)*1000)

static unsigned char ior(int);
static void iow(int, u8);
static void phy_write(int, u16);
static void move8(unsigned char *, int, int);
static void move16(unsigned char *, int, int);
static void move32(unsigned char *, int, int);
static u16 read_srom_word(int);
static void dmfe_init_dm9000(void);
static u32 GetDM9000ID(void);
void DM9000_get_enetaddr (uchar *);
static void eth_reset (void);
void eth_halt(void);
int eth_init(bd_t *);
void (*MoveData)(unsigned char *, int , int);

static void iow(int reg, u8 value)
{
	DM9000_PPTR = reg;
	DM9000_PDATA  =  value & 0xff;
}

static unsigned char ior(int reg)
{
	DM9000_PPTR = reg;
	return DM9000_PDATA & 0xff;
}

static void phy_write(int reg, u16 value)
{
	/* Fill the phyxcer register into REG_0C */
	iow(DM9KS_EPAR, DM9KS_PHY | reg);

	/* Fill the written data into REG_0D & REG_0E */
	iow(DM9KS_EPDRL, (value & 0xff));
	iow(DM9KS_EPDRH, ( (value >> 8) & 0xff));

	iow(DM9KS_EPCR, EPCR_PHY_Sele|EPCR_Write);	/* Issue phyxcer write command */
	udelay(500);			/* Wait write complete */
	iow(DM9KS_EPCR, 0x0);	/* Clear phyxcer write command */
}

/*
	leng: unit BYTE
	selec 0:input(RX)	1:output(TX)
	if selec=0, move data from FIFO to data_ptr 
	if selec=1, move data from data_ptr to FIFO
*/
static void move8(unsigned char *data_ptr, int leng, int selec)
{
	int i;
	if (selec)
		for (i=0; i<leng; i++)
			DM9000_PDATA =(data_ptr[i] & 0xff);
	else
		for (i=0; i<leng; i++)
			data_ptr[i] = DM9000_PDATA;
}	

static void move16(unsigned char *data_ptr, int leng, int selec)
{
	int i, tmpleng;
	tmpleng = (leng + 1) >> 1;
	if (selec)
		for (i=0; i<tmpleng; i++)
			DM9000_PDATA =((u16 *)data_ptr)[i];
	else
		for (i=0; i<tmpleng; i++)
			((u16 *)data_ptr)[i] = DM9000_PDATA;
}	

static void move32(unsigned char *data_ptr, int leng, int selec)
{
	int i, tmpleng;
	tmpleng = (leng + 3) >> 2;
	if (selec)
		for (i=0; i<tmpleng; i++)
			DM9000_PDATA = ((u32 *)data_ptr)[i];
	else
		for (i=0; i<tmpleng; i++)
			((u32 *)data_ptr)[i]=DM9000_PDATA;
}	

/*
 * Read a word data from EEPROM
 */
static u16 read_srom_word(int offset)
{
	iow(DM9KS_EPAR, offset);
	iow(DM9KS_EPCR, 0x4);
	udelay(200);
	iow(DM9KS_EPCR, 0x0);
	return (ior(DM9KS_EPDRL) + (ior(DM9KS_EPDRH) << 8) );
}
/* 
 *	Initilize dm9000 board
 */
static void dmfe_init_dm9000(void)
{
//	printf("dm9000 : dmfe_init_dm9000\n");

	int io_mode;
	int count;
	u8 link_status;
	
//	iow(DM9KS_PBCR, 0xe1); 	//0x61
	iow(DM9KS_PBCR, 0x41); 	// 6mA 0x61 // 090406 by shLee
	
	/* set the internal PHY power-on, GPIOs normal, and wait 20ms */
#if 0 //add by bill,070402
	iow(DM9KS_GPR, GPR_PHYUp);
	mdelay(20); 	/* wait for PHY power-on ready */
	iow(DM9KS_GPR, GPR_PHYDown);/* Power-Down PHY */
	mdelay(1000);	/* compatible with rtl8305s */
#endif
	iow(DM9KS_GPR, GPR_PHYUp);	
	mdelay(100);/* wait for PHY power-on ready */

	iow(DM9KS_NCR, NCR_MAC_loopback|NCR_Reset);
	udelay(20);/* wait 20us at least for software reset ok */
	
	iow(DM9KS_NCR, NCR_MAC_loopback|NCR_Reset);
	udelay(20);/* wait 20us at least for software reset ok */
	
	/* I/O mode */
	io_mode = ior(DM9KS_ISR) >> 6; /* ISR bit7:6 keeps I/O mode */
	switch (io_mode)
	{
		case DM9KS_BYTE_MODE:
			printf("DM9000 work in 8 bus width\n");
			MoveData = move8;
			break;
		case DM9KS_WORD_MODE:
			printf("DM9000 work in 16 bus width\n");
			MoveData = move16;
			break;
		case DM9KS_DWORD_MODE:
			printf("DM9000 work in 32 bus width\n");
			MoveData = move32;
			break;
		default:
			printf("DM9000 work in wrong bus width, error\n");
			break;
	}

	if(MoveData != move8)
	{
		MoveData = move8;		// force 8 bit mode
		printf(" -- forcing 8 bus width\n");
	}

	/* Set PHY */
	phy_write(4, 0x01e1);
	phy_write(0, 0x1200); /* N-way */

	/* Program operating register */
	iow(DM9KS_NCR, 0);
	iow(DM9KS_TCR, 0);					/* TX Polling clear */
	iow(DM9KS_BPTR, 0x30|JPT_600us);	/* Less 3kb, 600us */
	iow(DM9KS_SMCR, 0);		/* Special Mode */
	iow(DM9KS_NSR, 0x2c);	/* clear TX status */
	iow(DM9KS_ISR, 0x0f);	/* Clear interrupt status */
	iow(DM9KS_TCR2, TCR2_LedMode1);/* Set LED mode 1 */

#if 0  
	/* Data bus current driving/sinking capability  */
	iow(DM9KS_PBCR, 0x60);	/* default: 8mA */
#endif
 
	iow(0x1d, 0x80);/* receive broadcast packet */

	/* Activate DM9000A/DM9010 */
	iow(DM9KS_RXCR, DM9KS_REG05 | RXCR_RxEnable);	
	iow(DM9KS_IMR, DM9KS_DISINTR);

#if 0	 //add by bill, 070409
	count =0;
	do {
		link_status = ior(DM9KS_NSR);
		if (link_status & (1<<6))
			break;
		else
		{
			count++;
			mdelay(100);
		}
//	}while(count < 300);
	}while(count < 30);

#endif
}

/* packet page register access functions */
static u32 GetDM9000ID(void)
{
	u32	id_val;

	DM9000_PPTR = DM9KS_PID_H;
	id_val = (DM9000_PDATA & 0xff) << 8;
	DM9000_PPTR = DM9KS_PID_L;
	id_val+= (DM9000_PDATA & 0xff);
	id_val = id_val << 16;
	
	DM9000_PPTR = DM9KS_VID_H;
	id_val += (DM9000_PDATA & 0xff) << 8;
	DM9000_PPTR = DM9KS_VID_L;
	id_val += (DM9000_PDATA & 0xff);
	

	return id_val;
}


void DM9000_get_enetaddr (uchar * addr)
{
	int i;
	u8 temp;
	eth_reset();	

#if 0	
	printf ("MAC: ");
    for (i = 0x10; i <= 0x15; i++) {
    	temp = ior (i);
        *addr++ = temp;
        printf ("%x:", temp);
    }
#endif

	return;
}

static void eth_reset (void)
{
//	printf("dm9000 : eth_reset\n");
#if 0
	udelay(20);
	*(u_char *)(0x30000040) = 0x0;
	udelay(20);
	*(u_char *)(0x30000040) = 0x01;
	udelay(20);
	*(u_char *)(0x30000040) = 0x0;
	udelay(20);
	*(u_char *)(0x30000040) = 0x01;
	udelay(20);
#endif


//	u32 ID;
	u32 gpcon;
	u32	gpdat;

	gpdat = __raw_readw(GPGPU);
	gpdat &= ~(3 << 10);		// LAN reset
	gpdat |= (2 << 10);
	__raw_writew(gpdat, GPGPU);

	gpcon = __raw_readl(GPGCON);
	gpcon &= ~(3 << 10);		// LAN reset
	gpcon |= (1 << 10);
	__raw_writel(gpcon, GPGCON);

	gpdat = __raw_readl(GPGDAT);
	gpdat &= ~(1 << 5);
	__raw_writel(gpdat, GPGDAT);
	udelay(20);
	gpdat |= (1 << 5);
	__raw_writel(gpdat, GPGDAT);
//	udelay(500);
//	printf("GPGCON:%x\n",__raw_readl(GPGCON));
//	printf("GPGDAT:%x\n",__raw_readl(GPGDAT));
//	printf("GPGPU :%x\n",__raw_readl(GPGPU));

#if 0
	ID = GetDM9000ID();
	if ( ID != DM9000_ID)
	{
		printf("not found the dm9000 ID:%x\n",ID);
		return ;
	}else
		printf("found DM9000 ID:%x\n",ID);
#endif
//	eth_halt();
//	dmfe_init_dm9000();
}

void eth_halt (void)
{
	/* RESET devie */
	phy_write(0x00, 0x8000);		/* PHY RESET */
	iow(DM9KS_GPR, GPR_PHYDown); 	/* Power-Down PHY */
	iow(DM9KS_IMR, DM9KS_DISINTR);	/* Disable all interrupt */
	iow(DM9KS_RXCR, 0x00);			/* Disable RX */
}

int eth_init (bd_t * bd)
{
//	printf("dm9000 : eth_init\n");
#if 0	//1
	udelay(20);
	*(u_char *)(0x30000040) = 0x0;
	udelay(20);
	*(u_char *)(0x30000040) = 0x01;
	udelay(20);
	*(u_char *)(0x30000040) = 0x0;
	udelay(20);
	*(u_char *)(0x30000040) = 0x01;
	udelay(20);
#endif

	u32 ID;
	int i,j;
	u16 * mac =(u16 *) bd->bi_enetaddr;

	ID = GetDM9000ID();  // default
//	ID = DM9000_ID;      // for dhn shLee 090604
	if ( ID != DM9000_ID)
	{
		printf("not found the dm9000 ID:%x\n",ID);
		return 1;
	}
	
	dmfe_init_dm9000();
	for(i=0,j=0x10; i<6; i++,j++)
	{	
		iow(j,bd->bi_enetaddr[i]);
		printf("%x:",bd->bi_enetaddr[i]);
	}
	
	return 0;
}

/* Get a data block via Ethernet */
extern int eth_rx (void)
{
	unsigned short rxlen;
	unsigned char *addr = NULL;
	u8 RxRead;
	rx_t rx;
	u8 * ptr = (u8*)&rx;
	int i;

	RxRead = ior(DM9KS_MRCMDX);
	RxRead = ior(DM9KS_ISR);
	RxRead = ior(DM9KS_MRCMDX) & 0xff;
#if 0 /* HYUN_DEBUG */	
	RxRead = ior(DM9KS_MRCMDX) & 0xff;
#endif
	
	if (RxRead != 1)  /* no data */ 
		return 0;

	DM9000_PPTR = DM9KS_MRCMD; /* set read ptr ++ */

	/* Read packet status & length */
	MoveData(ptr, 4, 0);

	rxlen = rx.desc.length;		/* get len */

	if(rx.desc.status & (RX_RuntFrame | RX_PhyErr | RX_AlignErr | RX_CRCErr))
		printf ("[dm9ks]RX error %x\n", rx.desc.status);	

	if (rxlen > PKTSIZE_ALIGN + PKTALIGN)
		printf ("packet too big! %d %d\n", rxlen, PKTSIZE_ALIGN + PKTALIGN);

	addr = (unsigned char *)NetRxPackets[0];
	MoveData(addr, rxlen, 0);

	/* Pass the packet up to the protocol layers. */
	NetReceive (NetRxPackets[0], rxlen);

#if 1	/* HYUN_DEBUG */
	for(i=0;i<0x1fff;i++);	
#endif 	
	return rxlen;
}

/* Send a data block via Ethernet. */
extern int eth_send (volatile void *packet, int length)
{
	volatile unsigned char *addr;
	int	length1 = length;

	DM9000_PPTR = DM9KS_MWCMD;/* data copy ready set */

	/* copy data */
	addr = packet;
	MoveData(addr,length,1);

	/* set packet length  */
	iow(DM9KS_TXPLH, (length1 >> 8) & 0xff);  
	iow(DM9KS_TXPLL, length1 & 0xff);

	/* start transmit */
	iow(DM9KS_TCR, TCR_TX_Request);

	while (1)/* wait for tx complete */
	{

		if (ior(DM9KS_NSR)& (NSR_TX2END|NSR_TX1END))	
			break;
	}
	return 0;
}
#endif	/* COMMANDS & CFG_NET */

#endif	/* CONFIG_DRIVER_DM9000 */
