#ifndef __GF_SPI_H
#define __GF_SPI_H

#include <linux/types.h>

#define GF_BUF_STA_MASK		(0x1<<7)
#define	GF_BUF_STA_READY		(0x1<<7)
#define	GF_BUF_STA_BUSY		(0x0<<7)

#define	GF_IMAGE_MASK		(1<<6)
#define	GF_IMAGE_ENABLE		(1<<6)
#define	GF_IMAGE_DISABLE		(0x0)

#define	GF_KEY_MASK			(0x1<<5)
#define	GF_KEY_ENABLE		(0x1<<5)
#define	GF_KEY_DISABLE		(0x0)

#define	GF_KEY_STA			(0x1<<4)

typedef enum{
	GF_IMAGE_MODE = 0,
	GF_KEY_MODE,
	GF_SLEEP_MODE,
	GF_FF_MODE,
	GF_FF_RESCAN_MODE,
	GF_DEBUG_MODE = 0x56,
	GF_CALIB_MODE = 0x57,
	GF_FW_SYS_ERROR = 0xE3,
}MODE;

/**********************GF ops****************************/
#define GF_W          	0xF0
#define GF_R          	0xF1
#define GF_WDATA_OFFSET	(0x3)
#define GF_RDATA_OFFSET	(0x4)
/**********************************************************/
//for GF_IOC_CMD
#define GF_MCU_DISABLE   0xF2
#define GF_MCU_ENABLE    0xF3
#define GF_MCU_RESET	 0xF4
#define GF_MCU_UPDATE    0xF5
#define GF_GET_STATUS    0xF6
#define GF_GET_VERSION   0xF7
#define GF_DATA_SUSPEND  0xF8
#define GF_GET_CALIBDATA 0xF9
/**********************IO Magic**********************/
#define  GF_IOC_MAGIC    'g'  //define magic number
struct gf_ioc_transfer {
	u8	cmd;
	u8 reserve;
	u16	addr;
	u32 len;
	u8 *buf;
};
//define commands
/*read/write GF registers*/

#define  GF_IOC_CMD	_IOWR(GF_IOC_MAGIC, 1, struct gf_ioc_transfer)
#define  GF_IOC_REINIT	_IO(GF_IOC_MAGIC, 0)
#define  GF_IOC_SETSPEED	_IOW(GF_IOC_MAGIC, 2, u32)
#define  GF_IOC_SETMODE	_IOW(GF_IOC_MAGIC, 3,u8)
#define  GF_IOC_GETMODE	_IOR(GF_IOC_MAGIC, 4,u8)
#define  GF_IOC_WAITTOUCH      _IOW(GF_IOC_MAGIC, 5, struct gf_ioc_para)
#define  GF_IOC_WAITUNTOUCH  _IOW(GF_IOC_MAGIC, 6, struct gf_ioc_para)
#define  GF_IOC_MSG	_IOR(GF_IOC_MAGIC, 7,u8)
#define  GF_IOC_SETENMODE	_IOW(GF_IOC_MAGIC, 8,u8)
#define  GF_IOC_GETENMODE	_IOR(GF_IOC_MAGIC, 9,u8)

#define  GF_IOC_MAXNR    10

//#define  GF_IOC_UPDATE	_IOWR(GF_IOC_MAGIC, 8, struct gf_ioc_transfer)
//#define  GF_IOC_MAXNR    9


/*******define foe gf516***********/
#define CHIP_ID_ADDR 0x20
#define CHIP_ID_E 0x18E8
#define CHIP_ID_H 0x1800
#define CHIP_ID_E_ECO 0x18E9
#define CHIP_ID_H_ECO 0x1801
#define EFUSE_H_ADDR 0xFA
#define EFUSE_E_ADDR 0xEE
#define EFUSE_H_CTL_ADDR 0xF6
#define EFUSE_H_STA_ADDR 0xF8
/**********************************/


#endif
