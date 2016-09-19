#if 0
#if defined(CONFIG_TOUCHSCREEN_SHTSC_I2C)
#elif defined (CONFIG_TOUCHSCREEN_SHTSC_SPI)
#else // (CONFIG_TOUCHSCREEN_SHTSC_UNKNOWNIF??)
error
#endif // (CONFIG_TOUCHSCREEN_SHTSC_UNKNOWNIF??)
#endif /* 0 */

//#define DEBUG_IRQ_DISABLE

/*
 * Driver for sharp touch screen controller
 *
 * Copyright (c) 2013 Sharp Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/input/mt.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/kthread.h>
#include <linux/workqueue.h>
#include <linux/proc_fs.h>
//#include <generated/uapi/linux/version.h>

// should be defined automatically
#ifndef KERNEL_VERSION
#define KERNEL_VERSION(a,b,c) (((a) << 16) + ((b) << 8) + (c))
#endif /* KERNEL_VERSION */

#ifndef LINUX_VERSION_CODE
#define LINUX_VERSION_CODE (KERNEL_VERSION(3,4,0)) //dp qualcomm
#endif /* LINUX_VERSION_CODE */

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3,7,0))
#define __devinit
#define __devexit
#define __devexit_p
#endif /* (LINUX_VERSION_CODE >= KERNEL_VERSION(3,7,0)) */


//2014.10.16 added
#include <linux/string.h>
#include <linux/of_gpio.h>

#if defined(CONFIG_FB)
#include <linux/notifier.h>
#include <linux/fb.h>
#endif

#include <linux/ioctl.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>

#include <linux/mutex.h>

#if defined(CONFIG_TOUCHSCREEN_SHTSC_I2C)
#include <linux/i2c.h>
#elif defined (CONFIG_TOUCHSCREEN_SHTSC_SPI)
#include <linux/spi/spi.h>
#else // (CONFIG_TOUCHSCREEN_SHTSC_UNKNOWNIF??)
error
#endif // (CONFIG_TOUCHSCREEN_SHTSC_UNKNOWNIF??)

#include "shtsc_ioctl.h"
#include "shtsc.h"

// device code
#define DEVICE_CODE_LR388K5 2

char VersionYear = 15;
char VersionMonth = 7;
char VersionDay = 14;
char VersionSerialNumber = 22; // reset on another day
char VersionModelCode = DEVICE_CODE_LR388K5;
#define DRIVER_VERSION_LEN (5) // do not change

#define SHTSC_I2C_P_MAX		((1<<16)-1) //8000 // depends on firmware
#define SHTSC_I2C_SIZE_MAX		((1<<6)-1)

/* DEBUG */
#if 0
#ifndef DEBUG_SHTSC
#define DEBUG_SHTSC
#endif//DEBUG_SHTSC
#endif /* 0 */

#define DBGLOG(format, args...)  printk(KERN_INFO format, ## args)
#define ERRLOG(format, args...)  printk(KERN_ERR format, ## args)

#ifndef ETIMEOUT /* add by misaki on 9/28*/
#define ETIMEOUT  (116) /* add by misaki on 9/28*/
#endif /* #ifndef ETIMEOUT */ /* add by misaki on 9/28*/

/* INT STATUS */
#define SHTSC_STATUS_TOUCH_READY    (1<<0)
#define SHTSC_STATUS_POWER_UP       (1<<1)
#define SHTSC_STATUS_RESUME_PROX    (1<<2)
#define SHTSC_STATUS_WDT            (1<<3)
#define SHTSC_STATUS_DCMAP_READY    (1<<4)
#define SHTSC_STATUS_COMMAND_RESULT (1<<5)
#define SHTSC_STATUS_KNOCK_CODE     (1<<6)
#define SHTSC_STATUS_FLASH_LOAD_ERROR    (1<<8)
#define SHTSC_STATUS_PLL_UNLOCK     (1<<9)
#define SHTSC_STATUS_UNDEFINED_BIT  (0xFC80)

/* DONE IND */
#define SHTSC_IND_CMD   0x20
#define SHTSC_IND_TOUCH 0x01

/* BANK Address */
#define SHTSC_BANK_TOUCH_REPORT   0x00
#define SHTSC_BANK_COMMAND        0x02
#define SHTSC_BANK_COMMAND_RESULT 0x03
#define SHTSC_BANK_DCMAP          0x05

/* Common Register Address */
#define SHTSC_ADDR_INT0  0x00
#define SHTSC_ADDR_INTMASK0 0x01
#define SHTSC_ADDR_BANK 0x02
#define SHTSC_ADDR_IND  0x03
#define SHTSC_ADDR_INT1  0x04
#define SHTSC_ADDR_INTMASK1 0x05

/* Touch Report Register Address */
#define SHTSC_ADDR_TOUCH_NUM 0x08
#define SHTSC_ADDR_RESUME_PROX 0x09
#define SHTSC_ADDR_TOUCH_REPORT 0x10

/* Touch Parmeters */
#define SHTSC_MAX_FINGERS 10
#define SHTSC_MAX_TOUCH_1PAGE 10
#define SHTSC_LENGTH_OF_TOUCH 8

/* Touch Status */
#define SHTSC_F_TOUCH ((u8)0x01)
#define SHTSC_F_TOUCH_OUT ((u8)0x03)
//#define SHTSC_P_TOUCH ((u8)0x02)
//#define SHTSC_P_TOUCH_OUT ((u8)0x04)

#define SHTSC_TOUCHOUT_STATUS ((u8)0x80)

#define SHTSC_ADDR_COMMAND 0x08

typedef enum _cmd_state_e {
  CMD_STATE_SLEEP         = 0x00,
  CMD_STATE_IDLE          = 0x03,
  CMD_STATE_DEEP_IDLE     = 0x04,
  CMD_STATE_HOVER         = 0x06,  // if applicable
  CMD_STATE_PROX          = 0x07,  // if applicable
  CMD_STATE_GLOVE         = 0x08,  // if applicable
  CMD_STATE_COVER         = 0x0D,  // if applicable
  CMD_STATE_MAX
} dCmdState_e ;

typedef enum _cmd_CalibratioMode_e
{
  CMD_CALIB_MANUAL	  = 0x00,
  CMD_CALIB_START_OF_FORCE= 0x01,
  CMD_CALIB_END_OF_FORCE  = 0x02,
  CMD_CALIB_MAX
} dCmdCalibMode_e;

#define CMD_INIT              0x01
#define CMD_SETSYSTEM_STATE   0x02
#define CMD_EXEC_CALIBRATION  0x0F

#define CMD_PAYLOAD_LENGTH(X)              \
	X == CMD_INIT		?	0x04 : \
	X == CMD_SETSYSTEM_STATE?	0x04 : \
	X == CMD_EXEC_CALIBRATION?	0x04 : \
	0x0

#define SHTSC_ICON_KEY_NUM 2 //zhangjian modify 3->2
#ifdef SHTSC_ICON_KEY_NUM
#define USE_APPSELECT /* KEY_APPSELECT instead of KEY_MENU */
#ifdef USE_APPSELECT
const int shtsc_keyarray[SHTSC_ICON_KEY_NUM]={158,139};//zhangjian modify {158,102,0x244}; /* KEY_BACK,KEY_HOME,KEY_APPSELECT */
#else /* USE_APPSELECT */
const int shtsc_keyarray[SHTSC_ICON_KEY_NUM]={158,102,139}; /* KEY_BACK,KEY_HOME,KEY_MENU */
#endif /* USE_APPSELECT */
#endif

//2014.11.20 added
#define CMD_DELAY             16

#define WAIT_NONE   (0)
#define WAIT_CMD    (1)
#define WAIT_RESET  (2)


#define MAX_16BIT			0xffff
#define MAX_12BIT			0xfff

#define MAX_DCMAP_SIZE (37*37*2)

#define LOW_LEVEL 0
#define HIGH_LEVEL 1


/* ======== EEPROM command ======== */
#define	FLASH_CMD_WRITE_EN		(0x06)		/* Write enable command */
#define	FLASH_CMD_PAGE_WR			(0x02)		/* Page write command */
#define	FLASH_CMD_READ_ST			(0x05)		/* Read status command */
#define	FLASH_CMD_SECTOR_ERASE	(0xD7)		/* Sector erase command */
#define	FLASH_CMD_READ			(0x03)		/* Read command */

/* ======== EEPROM status ======== */
#define	FLASH_ST_BUSY		(1 << 0)	/* Busy with a write operation */


#define SHTSC_COORDS_ARR_SIZE	4 //2014.10.17 added
/* Software reset delay */
#define SHTSC_RESET_TIME	10	/* msec */
//#define SHTSC_SLEEP_TIME	100	/* msec */

#define FLASH_CHECK // experimental code - not checked yet

/*
 * The touch driver structure.
 */
struct shtsc_touch {
  u8 status;
  u8 id;
  u8 size;
  u8 type;
  u16 x;
  u16 y;
  u16 z;
};

/* force flash reprogramming */
#define FORCE_FIRM_UPDATE  // enable if required
#ifdef FORCE_FIRM_UPDATE
#include "LR388K5_firmware.h" // change to your firmware
int update_flash(void *, unsigned char *, unsigned int);
#define CMD_GETPROPERTY "\xE0\x00\x00\x11"
#define CMD_GETPROPERTY_LEN 4
#define CMD_SETSYSTEMSTATE_SLEEP "\x02\x00\x01\x00\x00"
#define CMD_SETSYSTEMSTATE_SLEEP_LEN 5
int CheckingFirmwareVersion = 0;
int FlashUpdateByNoFirm = 0;
#endif /* FORCE_FIRM_UPDATE */
//zhangjian add
int Firmware_version=0;
int Param_version=0;
int H_Firmware_version=0;
int H_Param_version=0;
int Shtsc_update_flash = 0;
int Shtsc_update_flash_sleep_no_power_off =0;
bool TP_resume_reset = false;
int buttonBit_old = 0;
/* gesture in suspend */
#define ENABLE_SUSPEND_GESTURE // enable if required
//#define FORCE_DISPLAY_ON
//zhangjian add for glove mode
#define CMD_SETSYSTEMSTATE_GLOVE "\x02\x00\x01\x00\x08"
#define CMD_SETSYSTEMSTATE_GLOVE_LEN 5

#define CMD_SETSYSTEMSTATE_COVER "\x02\x00\x01\x00\x0D"
#define CMD_SETSYSTEMSTATE_COVER_LEN 5
//add end
#ifdef ENABLE_SUSPEND_GESTURE
#define CMD_SETSYSTEMSTATE_DEEPIDLE "\x02\x00\x01\x00\x04"
#define CMD_SETSYSTEMSTATE_DEEPIDLE_LEN 5
#define CMD_SETSYSTEMSTATE_IDLE "\x02\x00\x01\x00\x03"
#define CMD_SETSYSTEMSTATE_IDLE_LEN 5
#define CMD_GETSYSTEMSTATE "\x03\x00\x00\x01"
#define CMD_GETSYSTEMSTATE_LEN 4
#define CMD_READRAM "\xD3\x00\x04\x20\x14\x68\x00\x00"
#define CMD_READRAM_LEN 8

/* user must define these value for their purpose */
#if 0 //zhangjian modify
#define KEY_GESTURE_DOUBLE_TAP 0x01
#define KEY_GESTURE_SLIDE_UP 0x02
#define	KEY_GESTURE_SLIDE_DOWN 0x03
#define KEY_GESTURE_SLIDE_RIGHT 0x04
#define KEY_GESTURE_SLIDE_LEFT 0x05
#define KEY_GESTURE_CHAR_C 0x63
#define KEY_GESTURE_CHAR_E 0x65
#define KEY_GESTURE_CHAR_M 0x6D
#define KEY_GESTURE_CHAR_O 0x6F
#define KEY_GESTURE_CHAR_V 0x76
#define KEY_GESTURE_CHAR_W 0x77
#define KEY_GESTURE_UNKNOWN 0x7F
#else
#define KEY_GESTURE_DOUBLE_TAP 0x260
#define KEY_GESTURE_CHAR_C 0x261
#define KEY_GESTURE_CHAR_E 0x262
#define KEY_GESTURE_CHAR_M 0x263
#define KEY_GESTURE_CHAR_O 0x264
#define KEY_GESTURE_CHAR_S 0x265
#define KEY_GESTURE_CHAR_W 0x266

#endif
//end modify
#endif


#if defined(CONFIG_TOUCHSCREEN_SHTSC_I2C)
struct shtsc_i2c *g_ts;
#elif defined (CONFIG_TOUCHSCREEN_SHTSC_SPI)
struct spi_device *g_spi;
struct shtsc_spi *g_ts;
#else // (CONFIG_TOUCHSCREEN_SHTSC_UNKNOWNIF??)
error
#endif // (CONFIG_TOUCHSCREEN_SHTSC_UNKNOWNIF??)
#define D_ENABLE_GESTURE_WORK_QUEUE //add by misaki on 8/6


#if defined(CONFIG_TOUCHSCREEN_SHTSC_I2C)
struct shtsc_i2c {
  u8 cmd;//2014.11.19 added
  u8 wait_state;//2014.11.19 added
  bool wait_result;//2014.11.19 added
  u8 disabled;		/* interrupt status */ //2014.11.6 added
  struct input_dev *input;
  struct shtsc_i2c_pdata *pdata;//2014.10.16 added
  char phys[32];
  struct i2c_client *client;
  int reset_pin;
  int irq_pin;
  //zhangjian addfor wakeup gesture
  int enable_wakeup_gesture;
  int suspend_enter_deepidle;
  int glove_mode;
  int cover_mode;
  int vcc_dc_pin;
  int vcc_io_pin;
  //add end
  struct shtsc_touch touch[SHTSC_MAX_FINGERS];
  struct mutex mutex;
#if defined(CONFIG_FB)
  struct notifier_block fb_notif;
  bool dev_sleep;
#endif
#if 0
#ifdef FORCE_FIRM_UPDATE
  struct work_struct workstr;
  struct workqueue_struct *workqueue;
#endif /* FORCE_FIRM_UPDATE */
#endif 
  struct work_struct workstr;
  struct workqueue_struct *workqueue;


  unsigned int            max_num_touch;
  int                     min_x;
  int                     min_y;
  int                     max_x;
  int                     max_y;
  int                     pressure_max;
  int                     touch_num_max;
  bool                    flip_x;
  bool                    flip_y;
  bool                    swap_xy;
  bool enable_irq_status; //add by misaki on 8/3
};
#elif defined (CONFIG_TOUCHSCREEN_SHTSC_SPI)
struct shtsc_spi {
  u8 cmd;//2014.11.19 added
  u8 wait_state;//2014.11.19 added
  bool wait_result;//2014.11.19 added
  struct spi_device	*spi;
  struct shtsc_touch touch[SHTSC_MAX_FINGERS];
  struct input_dev	*input;

  char			phys[32];
  struct mutex		mutex;
  unsigned int		irq;
  int reset_pin;  
  int spiss_pin;  
  spinlock_t		lock;
  struct timer_list	timer;

  unsigned int            max_num_touch;
  int                     min_x;
  int                     min_y;
  int                     max_x;
  int                     max_y;
  bool                    flip_x;
  bool                    flip_y;
  bool                    swap_xy;

  bool			opened;
  bool			suspended;
};
#else // (CONFIG_TOUCHSCREEN_SHTSC_UNKNOWNIF??)
error
#endif // (CONFIG_TOUCHSCREEN_SHTSC_UNKNOWNIF??)

#define SHTSC_DEVBUF_SIZE 1500

volatile static int buf_pos;
static u8 devbuf[SHTSC_DEVBUF_SIZE];

static int WriteMultiBytes(void *ts, u8 u8Addr, u8 *data, int len);
static int WriteOneByte(void *ts, u8 u8Addr, u8 u8Val);
static u8 ReadOneByte(void *ts, u8 u8Addr);
static void ReadMultiBytes(void *ts, u8 u8Addr, u16 u16Len, u8 *u8Buf);
static int issue_command(void*, unsigned char *, unsigned int);
static int shtsc_system_init(void *ts); //add by misaki on 9/28

//static int issue_command_msleep(void *ts, unsigned char *cmd, unsigned int len, int touch_clear, int mtime);

static int issue_command_with_touch_clear(void *ts, unsigned char *cmd, unsigned int len, int touch_clear);
int flash_access_start_shtsc(void *_ts);
int flash_access_end_shtsc(void *_ts);
int flash_erase_page_shtsc(void *_ts, int page);
int flash_write_page_shtsc(void *_ts, int page, unsigned char *data);
int flash_verify_page_shtsc(void *_ts, int page, unsigned char *data);
int flash_read(void *_ts, unsigned address, unsigned length, unsigned char *data);
static void shtsc_reset_delay(void);
static void shtsc_reset(void *, bool);
static void shtsc_reset_L_H(void *ts); //add by misaki on 8/1

u8 s_shtsc_addr;
u8 s_shtsc_buf[4*4096];
u8 s_shtsc_i2c_data[4*4096];
unsigned s_shtsc_len;

#define MAX_COMMAND_RESULT_LEN (64-8)
unsigned char CommandResultBuf[MAX_COMMAND_RESULT_LEN];

//int	G_reset_done = false;
u16 G_Irq_Mask = 0xffff;
unsigned G_touch=0;

pid_t pid = 0;
unsigned char resumeStatus; // bank 0, address 9


#if defined(CONFIG_TOUCHSCREEN_SHTSC_I2C)
static int shtsc_write_regs(struct shtsc_i2c *tsc, unsigned char addr,
				unsigned char len, unsigned char *value)
{
  struct i2c_client *client = tsc->client;
  int ret;

  s_shtsc_i2c_data[0] = addr;
  memcpy(s_shtsc_i2c_data + 1, value, len);

  ret = i2c_master_send(client, s_shtsc_i2c_data, len + 1);

  return ret;
}

static int shtsc_read_regs(struct shtsc_i2c *tsc,
			       unsigned char *data, unsigned short len, unsigned char addr)
{
  struct i2c_client *client = tsc->client;
  struct i2c_adapter *adap = client->adapter;
  struct i2c_msg msg[2];
  int ret;

  msg[0].addr = client->addr;
  msg[0].flags = client->flags & I2C_M_TEN;
  msg[0].len = 1;
  msg[0].buf = &addr;

  msg[1].addr = client->addr;
  msg[1].flags = client->flags & I2C_M_TEN;
  msg[1].flags |= I2C_M_RD;
  msg[1].len = len;
  msg[1].buf = data;

  ret = i2c_transfer(adap, msg, 2);

  if( ret < 2 ){
    dev_err(&client->dev, "Unable to read i2c bus (adr=%02x)\n",addr);
    goto out;
  }

  return 0;
 out:
  return -1;
}
#elif defined (CONFIG_TOUCHSCREEN_SHTSC_SPI)
static int shtsc_write_regs(struct shtsc_spi *tsc, u8 u8Addr, int len, u8 *data)
{
  u8 tx_buf[256+3];
  struct spi_message msg;
  struct spi_transfer t = {
    .tx_buf		= &tx_buf[0],
    .len		= (2+len),
    .bits_per_word	= 8,
    .speed_hz           = SHTSC_MAX_SPI_SPEED_IN_HZ,
  };
  int err;

  tx_buf[0] = 0x00;
  tx_buf[1] = u8Addr;
  memcpy(&(tx_buf[2]), data, len);

  spi_message_init(&msg);
  spi_message_add_tail(&t, &msg);
  err = spi_sync(tsc->spi, &msg);
  if (err) {
    dev_err(&tsc->spi->dev, "%s: failed, error: %d\n",
	    __func__, err);
    return err;
  }
  return 0;
}

static int shtsc_read_regs(struct shtsc_spi *tsc, u8 *u8Buf,  u16 u16Len, u8 u8Addr)
{
  u8 tx_buf[128];
  //  u8 tx_buf[128], rx_buf[512+128];
  struct spi_message msg;
  struct spi_transfer t = {
    .tx_buf		= &tx_buf[0],
    //    .rx_buf             = &rx_buf[0],
    .rx_buf             = &u8Buf[0],
    .len		= 2,
    .bits_per_word	= 8,
    .speed_hz           = SHTSC_MAX_SPI_SPEED_IN_HZ,
  };
  int err;
  int i;

  tx_buf[0] = 0x00;
  tx_buf[1] = u8Addr;

  spi_message_init(&msg);
  spi_message_add_tail(&t, &msg);
  err = spi_sync(tsc->spi, &msg);
  if (err) {
    dev_err(&tsc->spi->dev, "%s: failed, error: %d\n",
	    __func__, err);
    goto exitReadMultiBytes;
  }

  tx_buf[0] = 0x80;
  t.len = u16Len + 1;

  spi_message_init(&msg);
  spi_message_add_tail(&t, &msg);
  err = spi_sync(tsc->spi, &msg);
  if (err) {
    dev_err(&tsc->spi->dev, "%s: failed, error: %d\n",
	    __func__, err);
    goto exitReadMultiBytes;    
  }
  for(i = 0;i < u16Len;i++){
    u8Buf[i] = u8Buf[1 + i];
  }
  u8Buf[i] = 0; // not required. just for testing
  return 0;

 exitReadMultiBytes:  
  return err;
}
#else // (CONFIG_TOUCHSCREEN_SHTSC_UNKNOWNIF??)
error
#endif // (CONFIG_TOUCHSCREEN_SHTSC_UNKNOWNIF??)

static void shtsc_enable_irq(bool enable)
{
	
	if (enable && !g_ts->enable_irq_status) {
		enable_irq(g_ts->client->irq);
		g_ts->enable_irq_status = true;
	} else if(!enable && g_ts->enable_irq_status) {
		disable_irq(g_ts->client->irq);
		g_ts->enable_irq_status = false;
	}
}

static int WriteMultiBytes(void *_ts, u8 u8Addr, u8 *data, int len)
{
#if defined(CONFIG_TOUCHSCREEN_SHTSC_I2C)
  struct shtsc_i2c *ts = _ts;
#elif defined (CONFIG_TOUCHSCREEN_SHTSC_SPI)
  struct shtsc_spi *ts = _ts;
#else // (CONFIG_TOUCHSCREEN_SHTSC_UNKNOWNIF??)
  error
#endif // (CONFIG_TOUCHSCREEN_SHTSC_UNKNOWNIF??)

  return shtsc_write_regs(ts, u8Addr, len, data);
}

static int WriteOneByte(void *_ts, u8 u8Addr, u8 u8Val)
{
#if defined(CONFIG_TOUCHSCREEN_SHTSC_I2C)
  struct shtsc_i2c *ts = _ts;
#elif defined (CONFIG_TOUCHSCREEN_SHTSC_SPI)
  struct shtsc_spi *ts = _ts;
#else // (CONFIG_TOUCHSCREEN_SHTSC_UNKNOWNIF??)
  error
#endif // (CONFIG_TOUCHSCREEN_SHTSC_UNKNOWNIF??)
  u8 wData[1];
  wData[0] = u8Val;

  return shtsc_write_regs(ts, u8Addr, 1, wData);
}

static u8 ReadOneByte(void *_ts, u8 u8Addr)
{  
#if defined(CONFIG_TOUCHSCREEN_SHTSC_I2C)
  struct shtsc_i2c *ts = _ts;
#elif defined (CONFIG_TOUCHSCREEN_SHTSC_SPI)
  struct shtsc_spi *ts = _ts;
#else // (CONFIG_TOUCHSCREEN_SHTSC_UNKNOWNIF??)
  error
#endif // (CONFIG_TOUCHSCREEN_SHTSC_UNKNOWNIF??)
  u8 rData[1+1]; //requires one more byte to hold
  
  shtsc_read_regs(ts, rData, 1, u8Addr);
  return rData[0];
}

static void ReadMultiBytes(void *_ts, u8 u8Addr, u16 u16Len, u8 *u8Buf)
{
#if defined(CONFIG_TOUCHSCREEN_SHTSC_I2C)
  struct shtsc_i2c *ts = _ts;
#elif defined (CONFIG_TOUCHSCREEN_SHTSC_SPI)
  struct shtsc_spi *ts = _ts;
#else // (CONFIG_TOUCHSCREEN_SHTSC_UNKNOWNIF??)
error
#endif // (CONFIG_TOUCHSCREEN_SHTSC_UNKNOWNIF??)
  shtsc_read_regs(ts, u8Buf, u16Len, u8Addr);
}

//change WaitAsync to m_WaitAsync by misaki 9/28 static int WaitAsync(void *_ts)
static int m_WaitAsync(void *_ts)
{
#if defined(CONFIG_TOUCHSCREEN_SHTSC_I2C)
  struct shtsc_i2c *ts = _ts;
#elif defined (CONFIG_TOUCHSCREEN_SHTSC_SPI)
  struct shtsc_spi *ts = _ts;
#else // (CONFIG_TOUCHSCREEN_SHTSC_UNKNOWNIF??)
  error
#endif // (CONFIG_TOUCHSCREEN_SHTSC_UNKNOWNIF??)

		int i;
	for(i=0;i<62;i++) { /* 20150925 change 100 to 62: that is 62 * 16 = 992[ms] *///for(i=0;i<100;i++) {
		msleep(CMD_DELAY);//16ms	
		DBGLOG("shtsc: wait 16ms for state change(ts->wait_state:%d, result:%d gpio: %d\n", ts->wait_state, ts->wait_result, gpio_get_value(ts->irq_pin));//modified by misaki on 8/3
#if 1 //debug add by misaki
    { u8 regcommonbuf[11];      
      u16 u16status;
      ReadMultiBytes(ts,SHTSC_ADDR_INT0,11,regcommonbuf);
      u16status = ((regcommonbuf[SHTSC_ADDR_INT1] << 8) | regcommonbuf[SHTSC_ADDR_INT0]);
      G_Irq_Mask = ~((regcommonbuf[SHTSC_ADDR_INTMASK1] << 8) | regcommonbuf[SHTSC_ADDR_INTMASK0]);
      DBGLOG("shtsc: int:0x%x, mask:0x%x\n", u16status,G_Irq_Mask);
	}
#endif
		switch(ts->wait_state) {
	case WAIT_RESET:
		break;
	case WAIT_CMD:
		break;
	case WAIT_NONE:
		if (ts->wait_result == true) {
			DBGLOG("shtsc: wait state change: success\n");
			return 0;
		} else
			return -EIO;
	default:
		break;
		}
	}
	DBGLOG("wait state change: failure\n");

	return -ETIMEOUT; //ETIMEDOUT:116 //return -EIO; edit by misaki 9/28
}

//modified for timeout by misaki 9/28 
static int WaitAsync(void *_ts)
{
	int r; 

	r = m_WaitAsync(_ts);
	if (r == -ETIMEOUT) {
		shtsc_enable_irq(false);//add by misaki 8/3

		shtsc_reset(g_ts, false);

		//delete by misaki on 8/4   G_Irq_Mask = 0xffff;
#ifdef GET_REPORT
		reportBuf[0] = (unsigned char)0;
#endif /* GET_REPORT */

		g_ts->wait_state = WAIT_RESET;
		g_ts->wait_result = false;

		msleep(10); //add by misaki on 8/3

		shtsc_reset(g_ts, true);

		shtsc_enable_irq(true);//add by misaki 8/3

		// wait
		r = m_WaitAsync(g_ts); //modified by misaki
		shtsc_system_init(g_ts);
		return r; //modified by misaki 
	} 

	return r;
}

#ifndef DEBUG_IRQ_DISABLE
//static void SetBankAddr(void *_ts, u8 u8Bank) //modified by misaki on 8/1
static int SetBankAddr(void *_ts, u8 u8Bank)
{
  int r; //add by misaki on 8/1
#if defined(CONFIG_TOUCHSCREEN_SHTSC_I2C)
  struct shtsc_i2c *ts = _ts;
#elif defined (CONFIG_TOUCHSCREEN_SHTSC_SPI)
  struct shtsc_spi *ts = _ts;
#else // (CONFIG_TOUCHSCREEN_SHTSC_UNKNOWNIF??)
error
#endif // (CONFIG_TOUCHSCREEN_SHTSC_UNKNOWNIF??)

  DBGLOG("shtsc: bank command(%d)\n", u8Bank);

  r = WriteOneByte(ts, SHTSC_ADDR_BANK, u8Bank); //modified by misaki on 8/1
  if (r < 0) {//modified by misaki on 8/1
	printk("SetBankAddr: WriteOneByte error %d.", r);//modified by misaki on 8/1
  }//modified by misaki on 8/1

  return r;//modified by misaki on 8/1
}

static void ClearInterrupt(void *_ts, u16 u16Val)
{
#if defined(CONFIG_TOUCHSCREEN_SHTSC_I2C)
  struct shtsc_i2c *ts = _ts;
#elif defined (CONFIG_TOUCHSCREEN_SHTSC_SPI)
  struct shtsc_spi *ts = _ts;
#else // (CONFIG_TOUCHSCREEN_SHTSC_UNKNOWNIF??)
error
#endif // (CONFIG_TOUCHSCREEN_SHTSC_UNKNOWNIF??)

  if(u16Val & 0x00FF)
    WriteOneByte(ts, SHTSC_ADDR_INT0, (u16Val & 0x00FF));
  if((u16Val & 0xFF00) >> 8)
    WriteOneByte(ts, SHTSC_ADDR_INT1, ((u16Val & 0xFF00) >> 8));
}

static void SetIndicator(void *_ts, u8 u8Val)
{
#if defined(CONFIG_TOUCHSCREEN_SHTSC_I2C)
  struct shtsc_i2c *ts = _ts;
#elif defined (CONFIG_TOUCHSCREEN_SHTSC_SPI)
  struct shtsc_spi *ts = _ts;
#else // (CONFIG_TOUCHSCREEN_SHTSC_UNKNOWNIF??)
error
#endif // (CONFIG_TOUCHSCREEN_SHTSC_UNKNOWNIF??)

  WriteOneByte(ts, SHTSC_ADDR_IND, u8Val);
}
#endif /* DEBUG_IRQ_DISABLE */

static int shtsc_system_init(void *ts)
{
#if defined(CONFIG_TOUCHSCREEN_SHTSC_I2C)
  struct input_dev *input_dev = ((struct shtsc_i2c *)ts)->input;
#elif defined (CONFIG_TOUCHSCREEN_SHTSC_SPI)
  struct input_dev *input_dev = ((struct shtsc_spi *)ts)->input;
#else // (CONFIG_TOUCHSCREEN_SHTSC_UNKNOWNIF??)
  error
#endif // (CONFIG_TOUCHSCREEN_SHTSC_UNKNOWNIF??)

  /* PLL unlock - not used*/
  //  WriteOneByte(ts, SHTSC_ADDR_INTMASK1, (unsigned char)0x02);
  //  WriteOneByte(ts, (unsigned char)0x04, (unsigned char)0x02);
#if 0
  int i;
  u8 txbuf[8];
  u8 rxbuf[8];
  for(i=0;i<10;i++) {
    txbuf[0] = 0x02;
    txbuf[1] = i;
    i2c_master_send(((struct shtsc_i2c *)ts)->client, txbuf,2);
    i2c_master_send(((struct shtsc_i2c *)ts)->client, txbuf,1);
    i2c_master_recv(((struct shtsc_i2c *)ts)->client, rxbuf,1);
    printk(KERN_INFO "i2c master: tx=0x%x, rx=0x%x\n",i,rxbuf[0]);
  }
#endif
  {
    unsigned int cnt;
    for(cnt=0;cnt<SHTSC_MAX_FINGERS;cnt++){
      input_mt_slot(input_dev, cnt);
      input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, false);
#if defined(DEBUG_SHTSC)
      printk(KERN_INFO "shtsc_system_init() clear mt_slot(%d)\n",cnt);
#endif
    }
  }
  input_report_key(input_dev, BTN_TOUCH, false );

#ifdef SHTSC_ICON_KEY_NUM
  {
    unsigned int cnt;
    for(cnt=0;cnt<SHTSC_ICON_KEY_NUM;cnt++){
      input_report_key( input_dev, shtsc_keyarray[cnt] , false );
#if defined(DEBUG_SHTSC)
      printk(KERN_INFO "shtsc_system_init() clear report_key(%d)\n",cnt);
#endif
    }
  }
#endif
  input_sync(input_dev);

  DBGLOG("shtsc: shtsc_system_init done");//add by misaki on 8/3

  return 0;
}


#ifndef DEBUG_IRQ_DISABLE
static void GetTouchReport(void *ts, u8 u8Num, u8 *u8Buf)
{
#if defined(CONFIG_TOUCHSCREEN_SHTSC_I2C)
  struct shtsc_touch *touch = ((struct shtsc_i2c *)ts)->touch;
  struct input_dev *input_dev = ((struct shtsc_i2c *)ts)->input;
#elif defined (CONFIG_TOUCHSCREEN_SHTSC_SPI)
  struct shtsc_touch *touch = ((struct shtsc_spi *)ts)->touch;
  struct input_dev *input_dev = ((struct shtsc_spi *)ts)->input;
#else // (CONFIG_TOUCHSCREEN_SHTSC_UNKNOWNIF??)
  error
#endif // (CONFIG_TOUCHSCREEN_SHTSC_UNKNOWNIF??)

  int i;
  u8 ID,Status;
  int touchNum = 0;//2014.11.12 added

  if( u8Num > SHTSC_MAX_TOUCH_1PAGE ){
#if defined(DEBUG_SHTSC)
      printk(KERN_INFO "shtsc touch number erro (num=%d)\n",u8Num);
#endif
    return;
  }

    for(i = 0;i < u8Num;i++){
      Status = (u8Buf[i * SHTSC_LENGTH_OF_TOUCH] & 0xF0);
      touch[i].id = ID = u8Buf[i * SHTSC_LENGTH_OF_TOUCH] & 0x0F;    
      touch[i].size = u8Buf[i * SHTSC_LENGTH_OF_TOUCH + 1] & 0x3F;
      touch[i].type = (u8Buf[i * SHTSC_LENGTH_OF_TOUCH + 1] & 0xC0) >> 6;
      touch[i].x =
	(u8Buf[i * SHTSC_LENGTH_OF_TOUCH + 2] << 0) |
	(u8Buf[i * SHTSC_LENGTH_OF_TOUCH + 3] << 8);
      touch[i].y =
	(u8Buf[i * SHTSC_LENGTH_OF_TOUCH + 4] << 0) |
	(u8Buf[i * SHTSC_LENGTH_OF_TOUCH + 5] << 8);
      touch[i].z =
	(u8Buf[i * SHTSC_LENGTH_OF_TOUCH + 6] << 0) |
	(u8Buf[i * SHTSC_LENGTH_OF_TOUCH + 7] << 8);

#if 0 // no hover?
      if (!touch[i].z) {//14.11.28 added
	touch[i].z = 1;
      }
#endif /* 0 */

#if defined(DEBUG_SHTSC)
      printk(KERN_INFO "shtsc ID=%2d, Status=%02x, Size=%3d, X=%5d, Y=%5d, z=%5d, num=%2d\n"
	     ,ID
	     ,Status
	     ,touch[i].size
	     ,touch[i].x
	     ,touch[i].y
	     ,touch[i].z
	     ,u8Num);
#endif

      input_mt_slot(input_dev, ID);
      if(Status & SHTSC_TOUCHOUT_STATUS){
	touch[i].status = SHTSC_F_TOUCH_OUT;
	input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, false); // the 2nd parameter is DON'T CARE
	continue;
      }

      if(((struct shtsc_i2c *)ts)->swap_xy){//2014.11.12 added
	int tmp;
	tmp = touch[i].x;
	touch[i].x = touch[i].y;
	touch[i].y = tmp;
      }
      if(((struct shtsc_i2c *)ts)->flip_x){
	touch[i].x = (((struct shtsc_i2c *)ts)->max_x - touch[i].x) < 0 ? 0 : ((struct shtsc_i2c *)ts)->max_x - touch[i].x;
      }
      if(((struct shtsc_i2c *)ts)->flip_y){
	touch[i].y = (((struct shtsc_i2c *)ts)->max_y - touch[i].y) < 0 ? 0 : ((struct shtsc_i2c *)ts)->max_y - touch[i].y;
      }
      touchNum++;

      touch[i].status = SHTSC_F_TOUCH;
      input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, true);

      input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR, touch[i].size);
      input_report_abs(input_dev, ABS_MT_POSITION_X, touch[i].x);
      input_report_abs(input_dev, ABS_MT_POSITION_Y, touch[i].y);
      input_report_abs(input_dev, ABS_MT_PRESSURE  , touch[i].z);
    }
  input_report_key(input_dev, BTN_TOUCH, touchNum?true:false );//2014.11.12 added
}
#endif /* DEBUG_IRQ_DISABLE */


#ifdef FORCE_FIRM_UPDATE

//void update_flash_func(struct work_struct *work)
void update_flash_func(void)
{
  int err = 0;//zhangjian add
  printk(KERN_INFO "shtsc: update_flash_func\n");

  if (FlashUpdateByNoFirm == 1) {//update by misaki on 8/3 //if (FlashUpdateByNoFirm) {
    // no firmware found in flash
	unsigned char *firmverbin,*paramverbin,*firmbin;
	unsigned int firmlen;
	CheckingFirmwareVersion = 2; /* no more update */
	firmverbin = (unsigned char *)FIRM_VERSION;
	paramverbin = (unsigned char *)PARAM_VERSION;
	firmbin = (unsigned char *)FIRM_IMAGE;
	firmlen = FIRM_SIZE;
	/* force flash reprogramming */

    printk(KERN_INFO "shtsc: force updating flash by no valid firmware\n");
    update_flash(g_ts, firmbin, firmlen);
    printk(KERN_INFO "shtsc: force updating flash by no valid firmware.... done\n");
    FlashUpdateByNoFirm = 3; //update finished //add by misaki on 8/3


  } else {
      if (CheckingFirmwareVersion == 0) {
	CheckingFirmwareVersion = 1;

	printk(KERN_INFO "shtsc: issue_command in probe.\n");
//	printk(KERN_INFO "shtsc: issue_command in workqueue\n");

	/* issue GetProperty command to read out the firmware/parameter version */
	issue_command(g_ts, CMD_GETPROPERTY, CMD_GETPROPERTY_LEN);

	printk(KERN_INFO "shtsc: issue_command in probe... done\n");
      }

      if (CheckingFirmwareVersion == 1) {
	unsigned char *firmverbin,*paramverbin,*firmbin;
	unsigned int firmlen;
	CheckingFirmwareVersion = 2; /* no more update */
	firmverbin = (unsigned char *)FIRM_VERSION;
	paramverbin = (unsigned char *)PARAM_VERSION;
	firmbin = (unsigned char *)FIRM_IMAGE;
	firmlen = FIRM_SIZE;
    //zhangjian add for Firmware version read
    Firmware_version = CommandResultBuf[0x0D-0x08]<<24|CommandResultBuf[0x0C-0x08]<<16
    	  |CommandResultBuf[0x0B-0x08]<<8|CommandResultBuf[0x0A-0x08];
    Param_version = CommandResultBuf[0x15-0x08]<<24|CommandResultBuf[0x14-0x08]<<16
    |CommandResultBuf[0x13-0x08]<<8 |CommandResultBuf[0x12-0x08];
    H_Firmware_version = FIRM_VERSION[3]<<24|FIRM_VERSION[2]<<16
    	  |FIRM_VERSION[1]<<8|FIRM_VERSION[0];
    H_Param_version = PARAM_VERSION[3]<<24|PARAM_VERSION[2]<<16
    |PARAM_VERSION[1]<<8 |PARAM_VERSION[0];
    printk("Firmware_version is %X,Param_version is %X ,H_Firmware_version is %X,H_Param_version is %X \n", 
    Firmware_version,Param_version,H_Firmware_version,H_Param_version);
    //zhangjian add end

    #if 0
	if ( firmverbin && 
	    ( memcmp(firmverbin, (unsigned char *)&(CommandResultBuf[0x0a-0x08]),4)
	    || memcmp(paramverbin, (unsigned char *)&(CommandResultBuf[0x12-0x08]),4)))
    #else
	if (( H_Firmware_version > Firmware_version)||(H_Param_version > Param_version))
	#endif
	 {

      issue_command_with_touch_clear(g_ts, CMD_SETSYSTEMSTATE_SLEEP, CMD_SETSYSTEMSTATE_SLEEP_LEN, true); //modified by misaki on 8/4 //	  issue_command(g_ts, CMD_SETSYSTEMSTATE_SLEEP, CMD_SETSYSTEMSTATE_SLEEP_LEN);
	  printk(KERN_INFO "shtsc: now K5 becomes sleep state.\n");

	  /* the written firmware is not intended version - so overwrite */
	  printk(KERN_INFO "shtsc: Update firmware version: f%02X%02X%02X%02X p%02X%02X%02X%02X to f%02X%02X%02X%02X p%02X%02X%02X%02X\n", 
		 CommandResultBuf[0x0D-0x08], // current firm version
		 CommandResultBuf[0x0C-0x08],
		 CommandResultBuf[0x0B-0x08],
		 CommandResultBuf[0x0A-0x08],
		 CommandResultBuf[0x15-0x08], // current param version
		 CommandResultBuf[0x14-0x08],
		 CommandResultBuf[0x13-0x08],
		 CommandResultBuf[0x12-0x08],
		 firmverbin[3], // target firm version
		 firmverbin[2],
		 firmverbin[1],
		 firmverbin[0],
		 paramverbin[3], // target param version
		 paramverbin[2],
		 paramverbin[1],
		 paramverbin[0]);

	  err = update_flash(g_ts, firmbin, firmlen); //zhangjian modify  update_flash(g_ts, firmbin, firmlen);
      //zhangjian add
      Firmware_version =H_Firmware_version;
      Param_version = H_Param_version;
      //add end
      //zhangjian modify 
      #if 0
      FlashUpdateByNoFirm = 3; //update finished //add by misaki on 8/3
      #else
      if (err)
      {
          msleep(200);
		  update_flash(g_ts, firmbin, firmlen);
          FlashUpdateByNoFirm = 3;
          printk(KERN_INFO "shtsc:Firmware update again.\n");
      }
      else
      {
          FlashUpdateByNoFirm = 3;
          printk(KERN_INFO "shtsc:Firmware update ok.\n");
      }
      #endif 
      //end modify
	} else {
	  printk(KERN_INFO "shtsc: Firmware version update NOT required. Current version: f%02X%02X%02X%02X p%02X%02X%02X%02X\n", 
		 CommandResultBuf[0x0D-0x08], // firm version
		 CommandResultBuf[0x0C-0x08],
		 CommandResultBuf[0x0B-0x08],
		 CommandResultBuf[0x0A-0x08],
		 CommandResultBuf[0x15-0x08], // param version
		 CommandResultBuf[0x14-0x08],
		 CommandResultBuf[0x13-0x08],
		 CommandResultBuf[0x12-0x08]);
	}

      }

  }
  return;
}
#endif /* FORCE_FIRM_UPDATE */
#ifdef D_ENABLE_GESTURE_WORK_QUEUE //add by misaki on 8/6 */

static void irq_gesture_thread(struct work_struct *work)
{
     SetBankAddr(g_ts, SHTSC_BANK_TOUCH_REPORT);
     resumeStatus = ReadOneByte(g_ts, SHTSC_ADDR_RESUME_PROX);

#if defined(DEBUG_SHTSC)
      printk(KERN_INFO "[IRQ] shtsc resume prox from DeepIdle : ewg(%d) resumeStatus(%d)\n", g_ts->enable_wakeup_gesture, resumeStatus); //modified by misaki on 8/5
#endif
#ifdef ENABLE_SUSPEND_GESTURE
      // input_report_key( ts->input, code , true ); to kernel
	  
     if (g_ts->enable_wakeup_gesture) //zhangjian add for wakeup gesture
     {
         int key_gesture_code = 0x00;
         switch (resumeStatus) {
         case 0x01:
         key_gesture_code = KEY_GESTURE_DOUBLE_TAP;
         break;         
         case 0x63:
         key_gesture_code = KEY_GESTURE_CHAR_C;
         break;
         case 0x65:
         key_gesture_code = KEY_GESTURE_CHAR_E;
         break;
         case 0x6D:
         key_gesture_code = KEY_GESTURE_CHAR_M;
         break;
         case 0x6F:
         key_gesture_code = KEY_GESTURE_CHAR_O;
         break;
         case 0x73:
         key_gesture_code = KEY_GESTURE_CHAR_S;
         break;
         case 0x77:
         key_gesture_code = KEY_GESTURE_CHAR_W;
         break;
         default:
         printk(KERN_INFO "shtsc GESTURE_UNKNOWN \n");
         break;
         }
         if (key_gesture_code) 
		 {
             input_report_key( g_ts->input, key_gesture_code, true );//modified by misaki on 8/6
             input_sync(g_ts->input);//modified by misaki on 8/6
             input_report_key(g_ts->input, key_gesture_code, false );//modified by misaki on 8/6
             input_sync(g_ts->input);//modified by misaki on 8/6
         }
#if 1 // zhangjian modify defined(DEBUG_SHTSC)
         printk(KERN_INFO "[IRQ] shtsc resumeStatus %02X\n", resumeStatus);
#endif
     }
	 else
	 {
		  struct siginfo info;
		  struct task_struct *task;
	      
		  memset(&info, 0, sizeof(struct siginfo));
		  info.si_signo = SHTSC_SIGNAL;
		  info.si_code = SI_QUEUE;
		  info.si_int = 0;

		  rcu_read_lock();
		  task = find_task_by_vpid(pid);
		  rcu_read_unlock();

		  if (task) {
				send_sig_info(SHTSC_SIGNAL, &info, task); //send signal to user land
				printk(KERN_INFO "[SIGNAL] sending shtsc signal (resume_prox)\n");
		  }
    }
#else /* ENABLE_SUSPEND_GESTURE */
      // throw interrupt to userland
        struct siginfo info;
        struct task_struct *task;
        
        memset(&info, 0, sizeof(struct siginfo));
        info.si_signo = SHTSC_SIGNAL;
        info.si_code = SI_QUEUE;
        info.si_int = 0;
        
        rcu_read_lock();
        task = find_task_by_vpid(pid);
        rcu_read_unlock();
        
        if (task) {
			send_sig_info(SHTSC_SIGNAL, &info, task); //send signal to user land
			printk(KERN_INFO "[SIGNAL] sending shtsc signal (resume_prox)\n");
        }
#endif /* ENABLE_SUSPEND_GESTURE */
 

      //zhangjian add for gestrue detect again
      // issue_command(g_ts, CMD_SETSYSTEMSTATE_DEEPIDLE, CMD_SETSYSTEMSTATE_DEEPIDLE_LEN);	  
       issue_command_with_touch_clear(g_ts, CMD_SETSYSTEMSTATE_DEEPIDLE, CMD_SETSYSTEMSTATE_DEEPIDLE_LEN, true); //add by misaki on 8/4
     // msleep(20); //debug
     // DBGLOG("shtsc: issue deepidle command\n");
      //add end
#ifdef ENABLE_SUSPEND_GESTURE

#ifdef FORCE_DISPLAY_ON
      /* set display on without any control of the system */
      /* the system must control the display without this */
      if (g_ts->enable_wakeup_gesture) //zhangjian add for wakeup gesture
      {
          input_report_key(g_ts->input, KEY_POWER, true );//add by misaki on 8/6
          input_sync(g_ts->input);//add by misaki on 8/6
          input_report_key(g_ts->input, KEY_POWER, false );//add by misaki on 8/6
          input_sync(g_ts->input);//add by misaki on 8/6
          msleep(100); /* wait HSYNC becomes stable */
      }
#endif /* FORCE_DISPLAY_ON */
#endif /*  D_ENABLE_SUSPEND_GESTURE */
}
#endif /* #ifdef D_ENABLE_GESTURE_WORK_QUEUE //add by misaki on 8/6 */

//#define GET_REPORT  // experimental

#ifdef GET_REPORT
#include <linux/time.h>

#define REP_SIZE (4+4+4+120)
#define MAX_REPORTS 14

unsigned char reportBuf[4+ MAX_REPORTS*REP_SIZE];
volatile unsigned char Mutex_GetReport = false;

#endif /* GET_REPORT */

u8 dcmapBuf[MAX_DCMAP_SIZE+128];
u8 dcmap[31*18*2+128]; // greater than 17*32*2

#ifndef DEBUG_IRQ_DISABLE
static irqreturn_t shtsc_irq_thread(int irq, void *_ts)
{
#if defined(CONFIG_TOUCHSCREEN_SHTSC_I2C)
  struct shtsc_i2c *ts = _ts;
#elif defined (CONFIG_TOUCHSCREEN_SHTSC_SPI)
  struct shtsc_spi *ts = _ts;
#else // (CONFIG_TOUCHSCREEN_SHTSC_UNKNOWNIF??)
  error
#endif // (CONFIG_TOUCHSCREEN_SHTSC_UNKNOWNIF??)

  u16 u16status;
  u8 u8Num = 0;
  u8 tmpbuf[128];
  u8 numDriveLine2, numSenseLine2;
  u8 num_adc_dmy[3];
  u8 regcommonbuf[11];
#if 0
  {
    struct sched_param param;
    int ret;
    param.sched_priority = 64;
    ret = sched_setscheduler_nocheck(current, SCHED_FIFO, &param);
#if defined(DEBUG_SHTSC)
      printk(KERN_INFO "[IRQ] shtsc sched_setscheduler_nocheck %d\n", ret);
#endif
  }
#endif


#if defined(DEBUG_SHTSC)
  if (G_touch) {
    printk(KERN_INFO "[IRQ] shtsc touch-ready %d\n", G_touch);
  }
#endif

#if defined(DEBUG_SHTSC)
  printk(KERN_INFO "[ENTER] shtsc_irq\n");
#endif

  /* Get Interrupt State */
  ReadMultiBytes(ts,SHTSC_ADDR_INT0,11,regcommonbuf);
  u16status = ((regcommonbuf[SHTSC_ADDR_INT1] << 8) | regcommonbuf[SHTSC_ADDR_INT0]);
  G_Irq_Mask = ~((regcommonbuf[SHTSC_ADDR_INTMASK1] << 8) | regcommonbuf[SHTSC_ADDR_INTMASK0]);
  u16status &= G_Irq_Mask;
#if defined(DEBUG_SHTSC)
  if ((u16status != 0x0001) && (u16status != 0x0000)) {
	  printk(KERN_CRIT "[IRQ] shtsc_irq: %04X\n irq_mask: %04X", u16status, G_Irq_Mask);
  }
#endif
  if(u16status == 0)
  {
      printk(KERN_INFO "[ENTER] shtsc_irq u16status & mask is 0 \n"); //modified by misaki on 8/4
  }
  while(u16status != 0){
    if (u16status & (SHTSC_STATUS_WDT | SHTSC_STATUS_UNDEFINED_BIT)) { //modified by misaki on 8/4
      ClearInterrupt(ts, SHTSC_STATUS_WDT);
	  u16status = 0;//modified by misaki on 8/6 // u16status &=~SHTSC_STATUS_WDT;
	  shtsc_reset(g_ts, false);//add by misaki on 8/4
      msleep(10); //add by misaki on 8/4
      shtsc_reset(g_ts, true);//add by misaki on 8/4
      msleep(200);
      shtsc_enable_irq(true);//add by misaki on 8/4
	  //ERRLOG("[IRQ] shtsc_irq WDT power(%d)", gpio_is_valid(data->vcc_dc_pin));//modified by misaki on 8/6
	  break; //add by misaki on 8/6
    }

    if (u16status == SHTSC_STATUS_FLASH_LOAD_ERROR) {
      //
      // FLASH_LOAD_ERROR
      // occurs when flash is erased
      // nothing can be done
      //
#if defined(DEBUG_SHTSC)
      printk(KERN_INFO "[IRQ] shtsc flash load error\n");
#endif
      if (ts->wait_state == WAIT_RESET) {//2014.11.20 added
		ts->wait_state = WAIT_NONE;
		ts->wait_result = true;
      } 
      // from now on, no int for this
      //G_Irq_Mask &= ~SHTSC_STATUS_FLASH_LOAD_ERROR;

      /* Clear Interrupt */
      ClearInterrupt(ts, SHTSC_STATUS_FLASH_LOAD_ERROR);
      u16status &= ~SHTSC_STATUS_FLASH_LOAD_ERROR;

      // mask it
      WriteOneByte(ts, (unsigned char)SHTSC_ADDR_INTMASK1, (unsigned char)(regcommonbuf[SHTSC_ADDR_INTMASK1] | 0x01));//add by misaki on 8/4

#ifdef FORCE_FIRM_UPDATE
#if 0 
      printk(KERN_INFO "shtsc: calling firmware update workqueue\n");
      FlashUpdateByNoFirm = 1;
      queue_work(ts->workqueue, &ts->workstr);
      //      flush_workqueue(ts->workqueue);
#else /* 0 */
      // update here, not in the queue
      
	  if (FlashUpdateByNoFirm == 0) { //add by misaki on 8/3
		FlashUpdateByNoFirm = 1; //add by misaki on 8/3
        DBGLOG("shtsc: FLASH_LOAD_ERROR: FlashUpdateByNoFirm: %d\n", FlashUpdateByNoFirm);
	  } //add by misaki on 8/3
#endif /* 0 */
#endif /* FORCE_FIRM_UPDATE */
    }

    if (u16status & SHTSC_STATUS_PLL_UNLOCK) {
      //
      // PLL unlock
      //

#if defined(DEBUG_SHTSC)
      printk(KERN_INFO "[IRQ] shtsc_irq PLL_UNLOCK: %04X\n", u16status);
#endif
#if defined(DEBUG_SHTSC)
      printk(KERN_INFO "[IRQ] shtsc pll-unlock\n");
#endif
      // mask it
      WriteOneByte(ts, (unsigned char)SHTSC_ADDR_INTMASK1, (unsigned char)(regcommonbuf[SHTSC_ADDR_INTMASK1] | 0x02));//modified by misaki on 8/4
      //delete by misaki on 8/4 //regcommonbuf[SHTSC_ADDR_INTMASK1] = 0x03;
      // from now on, no int for this
      //delete by misaki on 8/4 //G_Irq_Mask &= ~SHTSC_STATUS_PLL_UNLOCK;
      /* Clear Interrupt */
      ClearInterrupt(ts, SHTSC_STATUS_PLL_UNLOCK);
      u16status &= ~SHTSC_STATUS_PLL_UNLOCK;

#if defined(DEBUG_SHTSC)
      printk(KERN_INFO "[IRQ] shtsc_irq PLL_UNLOCK: %04X\n", u16status);
#endif
    }

    if (u16status & SHTSC_STATUS_POWER_UP) {
      //
      // Power-up
      //

#if defined(DEBUG_SHTSC)
      printk(KERN_INFO "[IRQ] shtsc power-up\n");
#endif

      /* Clear Interrupt */
      ClearInterrupt(ts, SHTSC_STATUS_POWER_UP);
      u16status &= ~SHTSC_STATUS_POWER_UP;

      if (ts->wait_state == WAIT_RESET) {//2014.11.20 added
         ts->wait_state = WAIT_NONE;
         ts->wait_result = true;
         }
#ifdef FORCE_FIRM_UPDATE
	  if (FlashUpdateByNoFirm == 0) { //add by misaki on 8/3
		FlashUpdateByNoFirm = 2; //add by misaki on 8/3
        DBGLOG("shtsc: SHTSC_STATUS_POWER_UP: FlashUpdateByNoFirm: %d\n", FlashUpdateByNoFirm);
	  } //add by misaki on 8/3
#endif
#if 0
#ifdef FORCE_FIRM_UPDATE
      printk(KERN_INFO "shtsc: calling firmware update workqueue\n");
      queue_work(g_ts->workqueue, &g_ts->workstr);
      //      flush_workqueue(ts->workqueue);
#endif /* FORCE_FIRM_UPDATE */
#endif /* 0 */
   }

    if (u16status & SHTSC_STATUS_KNOCK_CODE) {
      ClearInterrupt(ts, SHTSC_STATUS_KNOCK_CODE);
      u16status &= ~SHTSC_STATUS_KNOCK_CODE;
      ERRLOG("[IRQ] shtsc_irq KNOCK_CODE");
    }

    if (u16status & SHTSC_STATUS_TOUCH_READY) {
      //
      // Touch report
      //

#if defined(DEBUG_SHTSC)
      printk(KERN_INFO "[IRQ] shtsc touch-ready cleared\n");
#endif

#ifdef GET_REPORT
      {
	/*
	  header:
	  1: counts of report (not a number of touch)
	  3: reserved

	  repeated contents if "counts of report"  is more than one.
	  1: cyclic counter (from 0 to 255. return to zero on next to 255)
	  3: reserved
	  4: report time (sec)
	  4: report time (usec)
	  120: bank0 report
	*/

	static unsigned char cyclic = 255;
	volatile unsigned char count;
	unsigned char bank0[120];
	struct timeval tv;

	while (Mutex_GetReport)
	  ;

	Mutex_GetReport = true;

	count = reportBuf[0];

	if (cyclic == 255) {
	  cyclic = 0;
	} else {
	  cyclic++;
	}
	
#if defined(DEBUG_SHTSC)
	//	printk(KERN_INFO "touch report: count %d, cyclic %d\n", count, cyclic);
#endif
	if (count == MAX_REPORTS) {
	  //	  printk(KERN_INFO "touch report buffer full\n");
	  ;
	} else {	
	  do_gettimeofday(&tv);
#if defined(DEBUG_SHTSC)
	  //	  printk(KERN_INFO "touch time: %ld.%ld, bank0pos:%d\n", (long)tv.tv_sec, (long)tv.tv_usec, (1+3+ count*REP_SIZE + 12));
#endif

	  reportBuf[1+3+ count*REP_SIZE +0] = cyclic;
	  reportBuf[1+3+ count*REP_SIZE +1] = 0;
	  reportBuf[1+3+ count*REP_SIZE +2] = 0;
	  reportBuf[1+3+ count*REP_SIZE +3] = 0;

	  reportBuf[1+3+ count*REP_SIZE +4+0] = (tv.tv_sec & 0xff);
	  reportBuf[1+3+ count*REP_SIZE +4+1] = ((tv.tv_sec >> 8) & 0xff);
	  reportBuf[1+3+ count*REP_SIZE +4+2] = ((tv.tv_sec >> 16) & 0xff);
	  reportBuf[1+3+ count*REP_SIZE +4+3] = ((tv.tv_sec >> 24) & 0xff);

	  reportBuf[1+3+ count*REP_SIZE +4+4+0] = (tv.tv_usec & 0xff);
	  reportBuf[1+3+ count*REP_SIZE +4+4+1] = ((tv.tv_usec >> 8) & 0xff);
	  reportBuf[1+3+ count*REP_SIZE +4+4+2] = ((tv.tv_usec >> 16) & 0xff);
	  reportBuf[1+3+ count*REP_SIZE +4+4+3] = ((tv.tv_usec >> 24) & 0xff);
	
	  SetBankAddr(ts, SHTSC_BANK_TOUCH_REPORT);
	  ReadMultiBytes(ts, 0x08, 120, bank0);

	  memcpy((unsigned char *)(&reportBuf[1+3+ count*REP_SIZE + 12]), (unsigned char *)bank0, 120);
	  reportBuf[0] = (unsigned char)(count+1);
	}

	Mutex_GetReport = false;
      }
#endif /* GET_REPORT */

      /* Get number of touches */
      {
	u8 u8Buf[128];
	if( regcommonbuf[SHTSC_ADDR_BANK] != SHTSC_BANK_TOUCH_REPORT ){
	  WriteOneByte(ts, SHTSC_ADDR_BANK, SHTSC_BANK_TOUCH_REPORT);
	  ReadMultiBytes(ts,SHTSC_ADDR_TOUCH_NUM,3,regcommonbuf+SHTSC_ADDR_TOUCH_NUM);
	}
	u8Num = regcommonbuf[SHTSC_ADDR_TOUCH_NUM];

#if defined(DEBUG_SHTSC)
	printk(KERN_INFO "shtsc touch num=%d\n", u8Num);
#endif

#ifdef SHTSC_ICON_KEY_NUM
	{
	  unsigned int buttonBit,cnt;
	  buttonBit = ReadOneByte(ts, 0x0A);
#if defined(DEBUG_SHTSC)
	  printk(KERN_INFO "shtsc(k5) icon key bitfield = %02x\n", buttonBit );
#endif
      for(cnt=0;cnt<SHTSC_ICON_KEY_NUM;cnt++)
      {
          //zhangjian add
    	  if (buttonBit_old != buttonBit)
    	  {
    	      buttonBit_old = buttonBit;
    		  printk(KERN_INFO "icon key bitfield = %02x\n", buttonBit );
    	  }
          //add end
          if(buttonBit&(0x01<<cnt))
          {
              input_report_key( ts->input, shtsc_keyarray[cnt] , true );
              
          }
          else
          {
              input_report_key( ts->input, shtsc_keyarray[cnt] , false );
              
          }
	  }
	}
#endif
	/* Retrieve touch report */
	if (u8Num > 0){
	  ReadMultiBytes((struct shtsc_i2c *)ts, SHTSC_ADDR_TOUCH_REPORT, SHTSC_LENGTH_OF_TOUCH * u8Num, u8Buf);
	}
	/* Clear Interrupt */
	u16status &= ~SHTSC_STATUS_TOUCH_READY;
	regcommonbuf[SHTSC_ADDR_INT0] = SHTSC_STATUS_TOUCH_READY;
	regcommonbuf[SHTSC_ADDR_BANK] = SHTSC_BANK_TOUCH_REPORT;
	regcommonbuf[SHTSC_ADDR_IND] = SHTSC_IND_TOUCH;
	WriteMultiBytes(ts,SHTSC_ADDR_INT0,regcommonbuf,4);
	if (u8Num > 0){
	  GetTouchReport(ts, u8Num, u8Buf);
	}

	input_sync(ts->input);
      }
    }

    if (u16status & SHTSC_STATUS_RESUME_PROX) {
#ifdef D_ENABLE_GESTURE_WORK_QUEUE //add by misaki on 8/6
       /* Clear Interrupt */
        ClearInterrupt(g_ts, SHTSC_STATUS_RESUME_PROX);
        u16status &= ~SHTSC_STATUS_RESUME_PROX;
        msleep(10); //add by misaki on 8/5
		DBGLOG("shtsc: entering gesture queue_work enter\n");
		queue_work(ts->workqueue, &ts->workstr); //add by misaki on 8/6
		

#else //#ifdef D_ENABLE_WORK_QUEU //add by misaki on 8/6
      //
      // Resume from DeepIdle
      // or
      // PROXIMITY
      //
      

      SetBankAddr(ts, SHTSC_BANK_TOUCH_REPORT);
      resumeStatus = ReadOneByte(ts, SHTSC_ADDR_RESUME_PROX);

#if defined(DEBUG_SHTSC)
      printk(KERN_INFO "[IRQ] shtsc resume prox from DeepIdle : ewg(%d) resumeStatus(%d)\n", g_ts->enable_wakeup_gesture, resumeStatus); //modified by misaki on 8/5
#endif
#ifdef ENABLE_SUSPEND_GESTURE
      DBGLOG("shtsc: entering gesture interrupt(%d)\n", g_ts->enable_wakeup_gesture);//modified by misaki on 8/3

      // input_report_key( ts->input, code , true ); to kernel
     if (g_ts->enable_wakeup_gesture) //zhangjian add for wakeup gesture
     {
         int key_gesture_code = 0x00;
         switch (resumeStatus) {
         case 0x01:
         key_gesture_code = KEY_GESTURE_DOUBLE_TAP;
         break;         
         case 0x63:
         key_gesture_code = KEY_GESTURE_CHAR_C;
         break;
         case 0x65:
         key_gesture_code = KEY_GESTURE_CHAR_E;
         break;
         case 0x6D:
         key_gesture_code = KEY_GESTURE_CHAR_M;
         break;
         case 0x6F:
         key_gesture_code = KEY_GESTURE_CHAR_O;
         break;
         case 0x73:
         key_gesture_code = KEY_GESTURE_CHAR_S;
         break;
         case 0x77:
         key_gesture_code = KEY_GESTURE_CHAR_W;
         break;
         default:
         printk(KERN_INFO "shtsc GESTURE_UNKNOWN \n");
         break;
         }
         if (key_gesture_code) 
		 {
             input_report_key( ts->input, key_gesture_code, true );
             input_sync(ts->input);
             input_report_key( ts->input, key_gesture_code, false );
             input_sync(ts->input);
         }
#if 1 // zhangjian modify defined(DEBUG_SHTSC)
         printk(KERN_INFO "[IRQ] shtsc resumeStatus %02X\n", resumeStatus);
#endif
     }
	 else
	 {
      struct siginfo info;
      struct task_struct *task;
      
      memset(&info, 0, sizeof(struct siginfo));
      info.si_signo = SHTSC_SIGNAL;
      info.si_code = SI_QUEUE;
      info.si_int = 0;

      rcu_read_lock();
      task = find_task_by_vpid(pid);
      rcu_read_unlock();

      if (task) {
	send_sig_info(SHTSC_SIGNAL, &info, task); //send signal to user land
	printk(KERN_INFO "[SIGNAL] sending shtsc signal (resume_prox)\n");
      }
    }
#else /* ENABLE_SUSPEND_GESTURE */
      // throw interrupt to userland
        struct siginfo info;
        struct task_struct *task;
        
        memset(&info, 0, sizeof(struct siginfo));
        info.si_signo = SHTSC_SIGNAL;
        info.si_code = SI_QUEUE;
        info.si_int = 0;
        
        rcu_read_lock();
        task = find_task_by_vpid(pid);
        rcu_read_unlock();
        
        if (task) {
        send_sig_info(SHTSC_SIGNAL, &info, task); //send signal to user land
        printk(KERN_INFO "[SIGNAL] sending shtsc signal (resume_prox)\n");
        }
#endif /* ENABLE_SUSPEND_GESTURE */
 
      /* Clear Interrupt */
      ClearInterrupt(ts, SHTSC_STATUS_RESUME_PROX);
      u16status &= ~SHTSC_STATUS_RESUME_PROX;
      msleep(10); //add by misaki on 8/5
      //zhangjian add for gestrue detect again
	  issue_command_msleep(g_ts, CMD_SETSYSTEMSTATE_DEEPIDLE, CMD_SETSYSTEMSTATE_DEEPIDLE_LEN, 1, 100); //add by misaki on 8/4
      DBGLOG("shtsc: issue deepidle command\n");
#ifdef ENABLE_SUSPEND_GESTURE

#ifdef FORCE_DISPLAY_ON
      /* set display on without any control of the system */
      /* the system must control the display without this */
      if (g_ts->enable_wakeup_gesture) //zhangjian add for wakeup gesture
      {
          input_report_key( ts->input, KEY_POWER, true );
          input_sync(ts->input);
          input_report_key( ts->input, KEY_POWER, false );
          input_sync(ts->input);
          msleep(100); /* wait HSYNC becomes stable */
      }
#endif /* FORCE_DISPLAY_ON */

	//DBGLOG("shtsc: issue IDLE command\n");
	//      msleep(SHTSC_GESTURE_WAIT_TIME);

	//      issue_command(ts, CMD_GETSYSTEMSTATE, CMD_GETSYSTEMSTATE_LEN); //debug
	//	msleep(300); //debug

	//issue_command(ts, CMD_SETSYSTEMSTATE_IDLE, CMD_SETSYSTEMSTATE_IDLE_LEN);
	//	msleep(300); //debug

	//      issue_command(ts, CMD_GETSYSTEMSTATE, CMD_GETSYSTEMSTATE_LEN); //debug
	//      msleep(300); //debug


#endif /* ENABLE_SUSPEND_GESTURE */
#endif //D_ENABLE_GESTURE_WORK_QUEUE //add by misaki
    }

    if (u16status & SHTSC_STATUS_COMMAND_RESULT) {
#if defined(DEBUG_SHTSC)
      printk(KERN_INFO "[IRQ] shtsc command result\n");
#endif
      SetBankAddr(ts,SHTSC_BANK_COMMAND_RESULT);
      ReadMultiBytes(ts, 0x08, MAX_COMMAND_RESULT_LEN, CommandResultBuf); // always read entire result. TEGAPP will handle it.

#if defined(DEBUG_SHTSC)
      printk(KERN_INFO "[IRQ] shtsc command result, %02X, %02X, %02X\n", CommandResultBuf[0], CommandResultBuf[1], CommandResultBuf[2]);
#endif

#if 0
      {
	int i;
	printk(KERN_INFO "[IRQ] shtsc command result (long) ");
	for (i = 0; i < 34; i++) {
	  printk(KERN_INFO "%02X ", CommandResultBuf[i]);
	}
      }
#endif // debug

      /* Clear Interrupt */
      ClearInterrupt(ts, SHTSC_STATUS_COMMAND_RESULT);
      u16status &= ~SHTSC_STATUS_COMMAND_RESULT;

      if (ts->wait_state == WAIT_CMD) {
	if ((CommandResultBuf[0] != ts->cmd) || (CommandResultBuf[1] != 0)) {
	  ts->wait_state = WAIT_NONE;
	  ts->wait_result = false;
	  DBGLOG("shtsc: CmdResult %d is ERROR(%d)\n", ts->cmd, CommandResultBuf[1]);//add by misaki on 8/3
	} else {
	  ts->wait_state = WAIT_NONE;
	  ts->wait_result = true;
	}
      }
    }

    if (u16status & SHTSC_STATUS_DCMAP_READY) {
#if defined(DEBUG_SHTSC)
      printk(KERN_INFO "[IRQ] shtsc DCMAP READY\n");
#endif
      /* Clear Interrupt */
      ClearInterrupt(ts, SHTSC_STATUS_DCMAP_READY);
      u16status &= ~SHTSC_STATUS_DCMAP_READY;

      { // L2
	unsigned char dsFlag, readingSenseNum, ram_addr[2];
	unsigned vramAddr;
	unsigned readingSize;

	// get SD/DS and size
	ram_addr[0] = 0x58;
	ram_addr[1] = 0xBF;
	WriteMultiBytes(ts,(unsigned char)0x06,ram_addr,2);

	SetBankAddr(ts,SHTSC_BANK_DCMAP);
	ReadMultiBytes(ts, 0x08, 5, tmpbuf);

	numSenseLine2 = tmpbuf[0];
	numDriveLine2 = tmpbuf[1];

	dsFlag = tmpbuf[4]; // 1 for DS, 0 for SD
	vramAddr = ((tmpbuf[3]<<8) | tmpbuf[2]);
	// readingSenseNum is greater or equal to itself, but a multiply of 4
	readingSenseNum = (unsigned char)((numSenseLine2+3)/4); // not a double but int
	readingSenseNum *= 4;

	readingSize = readingSenseNum * numDriveLine2 * 2; /* 2:16bit */

	num_adc_dmy[0] = num_adc_dmy[2] = 0; //top and left have no 0-filled lines
	num_adc_dmy[1] = readingSenseNum - numSenseLine2; //right side

	//	      printk(KERN_INFO "%s(%d): num_adc_dmy[1]:%d\n", __FILE__, __LINE__, num_adc_dmy[1]);
	//	      printk(KERN_INFO "%s(%d):Sense:%d, Drive:%d\n", __FILE__, __LINE__, numSenseLine2, numDriveLine2);
	//	      printk(KERN_INFO "%s(%d): dsFlag:%d\n", __FILE__, __LINE__, dsFlag);

	// read DCmap values from register
	// store it to read buffer memory for read action
	{ // L1
	  /* read 120 bytes from Bank5, address 8 */
	  /* read loop required */
	  int bytes = readingSize;
	  int size;
	  int index = 0;
	  //SetBankAddr(ts,SHTSC_BANK_DCMAP);

	  //	      printk(KERN_INFO "%s(%d):readingSize:%d\n", __FILE__, __LINE__, readingSize);
	  while (bytes > 0) {
	    ram_addr[0] = (unsigned char)(vramAddr&0xff); // address to read (Lower)
	    ram_addr[1] = (unsigned char)((vramAddr&0xff00)>>8); // address to read (Higher)
	    WriteMultiBytes(ts,(unsigned char)0x06,ram_addr,2);

	    size = ((bytes >= 120) ? 120 : bytes);
	    //		printk(KERN_INFO "%s(%d):bytes:%d, size:%d, index:%d, vramAddr:%x\n", __FILE__, __LINE__, bytes, size, index, vramAddr);

	    ReadMultiBytes(ts, 0x08, size, &(dcmapBuf[index]));
	    index += size;
	    bytes -= size;
	    vramAddr += size;
	  } // while
	} // L1

#if 1 //add by misaki on 8/17
	printk(KERN_INFO "DCmap Drive x Sense: %d x %d, dsFlag:%02X, num_adc_dmy:%02X %02X %02X\n", numDriveLine2, numSenseLine2, dsFlag, num_adc_dmy[0], num_adc_dmy[1], num_adc_dmy[2]);
#endif /* 0 */

	{ //L3
	  int sindex = 0, dindex = 0;
	  int l, x, y;
#if 0 // debug
	  static unsigned char frm_count = 0;
#endif /* 1 */

	  // dcmap header
	  // [0]: horizontal data num (in short, not byte)
	  // [1]: vertical data num

	  x = dcmap[dindex++] = numSenseLine2;
	  y = dcmap[dindex++] = numDriveLine2;
	  dcmap[dindex++] = dsFlag;
#if 0 // debug
	  dcmap[dindex++] = frm_count++; // just for debug
#else /* 1 */
	  dcmap[dindex++] = 0x00; // reserved
#endif /* 1 */

	  //top
	  sindex = (num_adc_dmy[0] + x + num_adc_dmy[1]) * num_adc_dmy[2] * 2;

	  // contents line
	  for (l = 0; l < y; l++) {
	    // left
	    sindex += (num_adc_dmy[0] * 2);

	    // contents
	    memcpy((u8 *)&(dcmap[dindex]), (u8 *)&(dcmapBuf[sindex]), (x*2));
	    dindex += (x*2);
	    sindex += (x*2);
		
	    // right
	    sindex += (num_adc_dmy[1] * 2);
	  }

	  // for read()
	  //	      printk(KERN_INFO "check buf_pos: %d\n", buf_pos);
	  if (buf_pos == 0) {
	    memcpy((u8 *)devbuf, (u8 *)dcmap, (4+x*y*2));
	    //		printk(KERN_INFO "setting buf_pos: %d\n", buf_pos);
	    buf_pos = (4+x*y*2);
	    //		printk(KERN_INFO "set buf_pos: %d\n", buf_pos);
	  }

#if 0 // DCmap debug
	  printk(KERN_INFO "DC map size HxV: %d x %d = %d\n", dcmap[0], dcmap[1], dcmap[0]*dcmap[1]);
	  printk(KERN_INFO "[0-3, %d-%d]: %02X %02X %02X %02X, %02X %02X %02X %02X\n", (x*y-4), (x*y-1), dcmap[4], dcmap[5], dcmap[6], dcmap[7], dcmap[x*y*2+4-4], dcmap[x*y*2+4-3], dcmap[x*y*2+4-2], dcmap[x*y*2+4-1]);
#endif /* 0 */
#if 0 // DCmap debug
	  for (j = 0; j < y; j++) {
	    for (i = 0; i < x; i++) {
	      printk(KERN_INFO "%d: %02X ", (y*j+i), dcmap[y*j + i + 4]);
	    }
	    printk(KERN_INFO "\n");
	  }
#endif
	} //L3
      } // L2 DEVICE_LR388K5 block
      //*************************
    }
    if (u16status != 0) {
      printk(KERN_INFO "[IRQ] shtsc unknown interrupt status %04X\n", u16status);
#if 0
      /* Clear all interrupts and mask.  */
      WriteOneByte(ts, (unsigned char)SHTSC_ADDR_INTMASK0, (unsigned char)0xFF);
      WriteOneByte(ts, (unsigned char)SHTSC_ADDR_INTMASK1, (unsigned char)0x03);
      ClearInterrupt(ts, u16status);
      u16status = 0;
#else /* 0 */
      u16status = 0;

			shtsc_reset(g_ts, false);	//delete by misaki on 8/4       G_Irq_Mask = 0xffff;
			msleep(10); //add by misaki on 9/28
#ifdef GET_REPORT
      reportBuf[0] = (unsigned char)0;
#endif /* GET_REPORT */
			g_ts->wait_state = WAIT_RESET;
			g_ts->wait_result = false;
			shtsc_reset(g_ts, true);
			msleep(200); //add by misaki 9/28
			shtsc_enable_irq(true);//add by misaki on 9/28
			// wait
			//delete by misaki 9/28 WaitAsync(g_ts);
			shtsc_system_init(g_ts);
#endif /* 0 */
    }
  }

  if (u8Num != 0 ) {
#if defined(DEBUG_SHTSC)
    printk(KERN_INFO "shtsc flush touch input(%d)\n", u8Num);
#endif
    /*
      input_report_key(ts->input, BTN_TOUCH, touchNum?true:false );
		
      input_sync(ts->input);
    */
  }

  return IRQ_HANDLED;
}
#endif /* DEBUG_IRQ_DISABLE */

static ssize_t 
dev_read( struct file* filp, char* buf, size_t count, loff_t* pos )
{
  int copy_len;
  int i;

  //	printk( KERN_INFO "shtsc : read()  called, buf_pos: %d, count: %d\n", buf_pos, count);
  if ( count > buf_pos )
    copy_len = buf_pos;
  else
    copy_len = count;

  //	printk( KERN_INFO "shtsc : copy_len = %d\n", copy_len );
  if ( copy_to_user( buf, devbuf, copy_len ) ) {
    printk( KERN_INFO "shtsc : copy_to_user failed\n" );
    return -EFAULT;
  }

  *pos += copy_len;

  for ( i = copy_len; i < buf_pos; i ++ )
    devbuf[ i - copy_len ] = devbuf[i];

  buf_pos -= copy_len;

  //	printk( KERN_INFO "shtsc : buf_pos = %d\n", buf_pos );
  return copy_len;
}
/*ZSW_MODIFY begin,add for get tp_gesture status in KERNEL,yangchaofeng,2010610*/
bool lcd_get_sharp_gesture_state(void)
{
	return g_ts->enable_wakeup_gesture;
}
void Glove_mod_enter_deppidle(void)
{
    if(g_ts->enable_wakeup_gesture)
    {        
		if((1 == g_ts->glove_mode) || (1 == g_ts->cover_mode))//glove enable
        {
            DBGLOG("shtsc: Glove or cover enter deppidle first enter idle \n");
            shtsc_reset_L_H(g_ts);//add by misaki on 8/17
            //delete by misaki on 8/17issue_command_with_touch_clear(g_ts,CMD_SETSYSTEMSTATE_IDLE, CMD_SETSYSTEMSTATE_IDLE_LEN, true);//modified by misaki on 8/4 //issue_command(g_ts,CMD_SETSYSTEMSTATE_IDLE, CMD_SETSYSTEMSTATE_IDLE_LEN);
            DBGLOG("shtsc: issue idle command\n");
        }
    }

}
/*ZSW_MODIFY end,add for get tp_gesture status in KERNEL,yangchaofeng,2010610*/
static void shtsc_reset(void *_ts, bool reset)
{
#if defined(CONFIG_TOUCHSCREEN_SHTSC_I2C)
  struct shtsc_i2c *ts = _ts;
#elif defined (CONFIG_TOUCHSCREEN_SHTSC_SPI)
  struct shtsc_spi *ts = _ts;
#else // (CONFIG_TOUCHSCREEN_SHTSC_UNKNOWNIF??)
  error
#endif // (CONFIG_TOUCHSCREEN_SHTSC_UNKNOWNIF??)

   if ((ts->reset_pin) && ( !Shtsc_update_flash)){  // if (ts->reset_pin) {
//      G_reset_done = reset;
      if (reset) {
	shtsc_reset_delay();
      }
      printk(KERN_INFO "shtsc: shtsc_reset: %d\n", reset);
      gpio_direction_output(ts->reset_pin, reset);
//delete by misaki on 8/4       if (! reset) {
//delete by misaki on 8/4 	G_Irq_Mask = 0xffff;
//delete by misaki on 8/4       }
    }
}

#if 1 //add by misaki on 8/1
static void shtsc_reset_L_H(void *ts)
{
  shtsc_enable_irq(false);//add by misaki 8/3

  shtsc_reset(g_ts, false);

//delete by misaki on 8/4   G_Irq_Mask = 0xffff;
#ifdef GET_REPORT
  reportBuf[0] = (unsigned char)0;
#endif /* GET_REPORT */

  g_ts->wait_state = WAIT_RESET;
  g_ts->wait_result = false;

  msleep(10); //add by misaki on 8/3

  shtsc_reset(g_ts, true);

  shtsc_enable_irq(true);//add by misaki 8/3

  // wait
  WaitAsync(g_ts);
  shtsc_system_init(g_ts);
}
#endif //#if 1
int flash_access_start_shtsc(void *_ts)
{
#if defined(CONFIG_TOUCHSCREEN_SHTSC_I2C)
  struct shtsc_i2c *ts = _ts;
#elif defined (CONFIG_TOUCHSCREEN_SHTSC_SPI)
  struct shtsc_spi *ts = _ts;
#else // (CONFIG_TOUCHSCREEN_SHTSC_UNKNOWNIF??)
  error
#endif // (CONFIG_TOUCHSCREEN_SHTSC_UNKNOWNIF??)

  // mask everything
  WriteOneByte(ts, (unsigned char)SHTSC_ADDR_INTMASK0, (unsigned char)0xff);
  WriteOneByte(ts, (unsigned char)SHTSC_ADDR_INTMASK1, (unsigned char)0x03);
  msleep(100);

  /* TRIM_OSC = 0 */
  WriteOneByte(ts, (unsigned char)0x02, (unsigned char)0x18); // bank change reg
  WriteOneByte(ts, (unsigned char)0x11, (unsigned char)0x19);
  return 0;
}

int flash_access_end_shtsc(void *_ts)
{
#if defined(CONFIG_TOUCHSCREEN_SHTSC_I2C)
  struct shtsc_i2c *ts = _ts;
#elif defined (CONFIG_TOUCHSCREEN_SHTSC_SPI)
  struct shtsc_spi *ts = _ts;
#else // (CONFIG_TOUCHSCREEN_SHTSC_UNKNOWNIF??)
  error
#endif // (CONFIG_TOUCHSCREEN_SHTSC_UNKNOWNIF??)

    msleep(100);

  shtsc_reset(ts, false);
  msleep(100);
  shtsc_reset(ts, true);
  msleep(10);
  shtsc_system_init(ts);

  msleep(100);

  return 0;
}

#define RETRY_COUNT (2000*10) //experimental
int flash_erase_page_shtsc(void *_ts, int page)
{
#if defined(CONFIG_TOUCHSCREEN_SHTSC_I2C)
  struct shtsc_i2c *ts = _ts;
#elif defined (CONFIG_TOUCHSCREEN_SHTSC_SPI)
  struct shtsc_spi *ts = _ts;
#else // (CONFIG_TOUCHSCREEN_SHTSC_UNKNOWNIF??)
  error
#endif // (CONFIG_TOUCHSCREEN_SHTSC_UNKNOWNIF??)

    //  int chan= 0;
    volatile unsigned char readData;
  int retry=0;

  /* BankChange */
  //SPI_ArrieRegWrite(0x02,0x1C);
  WriteOneByte(ts, (unsigned char)0x02, (unsigned char)0x1C); // bank change reg

  /* CLKON_CTL0 */
  //SPI_ArrieRegWrite(0x14,0x22);	/* CLKON_FLC,CLKON_REGBUS */
  WriteOneByte(ts, (unsigned char)0x14, (unsigned char)0x22);

  /* FLC_CTL CS_HIGH,WP_DISABLE */
  //	SPI_ArrieRegWrite(0x3C,0x14);
  WriteOneByte(ts, (unsigned char)0x3C, (unsigned char)0x14);

  /* FLC_CTL CS_LOW,WP_DISABLE */
  //	SPI_ArrieRegWrite(0x3C,0x16);
  WriteOneByte(ts, (unsigned char)0x3C, (unsigned char)0x16);
  /* FLC_TxDATA WRITE_ENABLE */
  //SPI_ArrieRegWrite(0x3D,SPI_CMD_WRITE_EN);
  WriteOneByte(ts, (unsigned char)0x3D, (unsigned char)FLASH_CMD_WRITE_EN);
  /* FLC_CTL CS_HIGH,WP_DISABLE */
  //	SPI_ArrieRegWrite(0x3C,0x14);
  WriteOneByte(ts, (unsigned char)0x3C, (unsigned char)0x14);

  /* FLC_CTL CS_LOW,WP_DISABLE */
  //	SPI_ArrieRegWrite(0x3C,0x16);
  WriteOneByte(ts, (unsigned char)0x3C, (unsigned char)0x16);


  /* FLC_TxDATA CHIP_ERASE_COMMAND */
  //	SPI_ArrieRegWrite(0x3D,SPI_CMD_CHIP_ERASE);
  // not a chip erase, but a sector erase for the backward compatibility!!
  WriteOneByte(ts, (unsigned char)0x3D, (unsigned char)FLASH_CMD_SECTOR_ERASE);
  // 24bit address. 4kByte=001000H. 00x000H:x=0-f -> 4kB*16=64kB
  WriteOneByte(ts, (unsigned char)0x3D, (unsigned char)0x00);
  WriteOneByte(ts, (unsigned char)0x3D, (unsigned char)((page<<4)&0xFF));
  WriteOneByte(ts, (unsigned char)0x3D, (unsigned char)0x00);

  /* FLC_CTL CS_HIGH,WP_DISABLE */
  //	SPI_ArrieRegWrite(0x3C,0x14);
  WriteOneByte(ts, (unsigned char)0x3C, (unsigned char)0x14);


  /* wait until 'BUSY = LOW' */
  do {
    retry++;
    ////		msleep(10);
    if (retry > RETRY_COUNT)
      goto RETRY_ERROR;

    /* FLC_CTL CS_LOW,WP_DISABLE */
    //		SPI_ArrieRegWrite(0x3C,0x16);
    WriteOneByte(ts, (unsigned char)0x3C, (unsigned char)0x16);
    /* FLC_TxDATA READ_STATUS_COMMAND*/
    //		SPI_ArrieRegWrite(0x3D,SPI_CMD_READ_ST);
    WriteOneByte(ts, (unsigned char)0x3D, FLASH_CMD_READ_ST);
    /* Dummy data */
    //		SPI_ArrieRegWrite(0x3D,0);
    WriteOneByte(ts, (unsigned char)0x3D, (unsigned char)0x00);
    /* FLC_RxDATA */
    //		readData = SPI_ArrieRegRead(0x3F);
    readData = ReadOneByte(ts, 0x3F);
    /* FLC_CTL CS_HIGH,WP_DISABLE */
    //		SPI_ArrieRegWrite(0x3C,0x14);
    WriteOneByte(ts, (unsigned char)0x3C, (unsigned char)0x14);
  } while (readData & FLASH_ST_BUSY); 		/* check busy bit */

  return 0;

 RETRY_ERROR:
  printk(KERN_INFO "FATAL: flash_erase_page_shtsc retry %d times for page %d - FAILED!\n", retry, page);
  return 1;
}
#define FLASH_PAGE_SIZE (4<<10) // 4k block for each page
#define FLASH_PHYSICAL_PAGE_SIZE (256) // can write 256bytes at a time.

int flash_write_page_shtsc(void *_ts, int page, unsigned char *data)
{
#if defined(CONFIG_TOUCHSCREEN_SHTSC_I2C)
  struct shtsc_i2c *ts = _ts;
#elif defined (CONFIG_TOUCHSCREEN_SHTSC_SPI)
  struct shtsc_spi *ts = _ts;
#else // (CONFIG_TOUCHSCREEN_SHTSC_UNKNOWNIF??)
  error
#endif // (CONFIG_TOUCHSCREEN_SHTSC_UNKNOWNIF??)

    //  int chan = 0;
    int retry = 0;
  //  unsigned addr; // in-page address (from 0 to FLASH_PAGE_SIZE-1)
  unsigned paddr; // address (32 or 64kB area)
  volatile unsigned char readData;
  int cnt, idx;

  /* BankChange */
  //	SPI_ArrieRegWrite(0x02,0x1C);
  WriteOneByte(ts, (unsigned char)0x02, (unsigned char)0x1C); // bank change reg

  /* CLKON_CTL0 */
  //	SPI_ArrieRegWrite(0x14,0x22);	/* CLKON_FLC,CLKON_REGBUS */
  WriteOneByte(ts, (unsigned char)0x14, (unsigned char)0x22);

  /* FLC_CTL */
  //	SPI_ArrieRegWrite(0x3C,0x14);
  WriteOneByte(ts, (unsigned char)0x3C, (unsigned char)0x14);

  /* 256 bytes / Flash write page, 4kByte / logical(virtual) flash page */
  for (cnt = 0; cnt < (FLASH_PAGE_SIZE / FLASH_PHYSICAL_PAGE_SIZE); cnt++) {
    paddr = (page * FLASH_PAGE_SIZE) + (cnt * FLASH_PHYSICAL_PAGE_SIZE);
    // 4k page offset + in-page offset. 4k*n+256*m

    /* FLC_CTL CS_LOW,WP_DISABLE */
    //		SPI_ArrieRegWrite(0x3C,0x16);
    WriteOneByte(ts, (unsigned char)0x3C, (unsigned char)0x16);
    /* FLC_TxDATA WRITE_ENABLE */
    //		SPI_ArrieRegWrite(0x3D,SPI_CMD_WRITE_EN);
    WriteOneByte(ts, (unsigned char)0x3D, FLASH_CMD_WRITE_EN);
    /* FLC_CTL CS_HIGH,WP_DISABLE */
    //		SPI_ArrieRegWrite(0x3C,0x14);
    WriteOneByte(ts, (unsigned char)0x3C, (unsigned char)0x14);

    /* FLC_CTL CS_LOW,WP_DISABLE */
    //		SPI_ArrieRegWrite(0x3C,0x16);
    WriteOneByte(ts, (unsigned char)0x3C, (unsigned char)0x16);

    /* FLC_TxDATA PAGE_PROGRAM_COMMAND */
    //		SPI_ArrieRegWrite(0x3D,SPI_CMD_PAGE_WR);
    WriteOneByte(ts, (unsigned char)0x3D, FLASH_CMD_PAGE_WR);
    /* FLC_TxDATA Address(bit16~23) */
    //		SPI_ArrieRegWrite(0x3D,((address>>16)&0xFF));
    WriteOneByte(ts, (unsigned char)0x3D, ((paddr>>16)&0xFF));
    /* FLC_TxDATA Address(bit8~15) */
    //		SPI_ArrieRegWrite(0x3D,((address>>8)&0xFF));
    WriteOneByte(ts, (unsigned char)0x3D, ((paddr>>8)&0xFF));
    /* FLC_TxDATA Address(bit0~7) */
    //		SPI_ArrieRegWrite(0x3D,(address&0xFF));
    WriteOneByte(ts, (unsigned char)0x3D, (paddr&0xFF));
    /* Data write 1page = 256byte */
    for(idx=0;idx<256;idx++){
      //			SPI_ArrieRegWrite(0x3D,*pData++);
      // addr=in-page(virtual, in 4k block) 256xN
      WriteOneByte(ts, (unsigned char)0x3D, data[(cnt*FLASH_PHYSICAL_PAGE_SIZE) +idx]);
    }
    /* FLC_CTL CS_HIGH,WP_DISABLE */
    //		SPI_ArrieRegWrite(0x3C,0x14);
    WriteOneByte(ts, (unsigned char)0x3C, (unsigned char)0x14);


    /* wait until 'BUSY = LOW' */
    do {
      retry++;
      ////		  msleep(10);
      if (retry > RETRY_COUNT)
	goto RETRY_ERROR;

      /* FLC_CTL CS_LOW,WP_DISABLE */
      //		SPI_ArrieRegWrite(0x3C,0x16);
      WriteOneByte(ts, (unsigned char)0x3C, (unsigned char)0x16);
      /* FLC_TxDATA READ_STATUS_COMMAND*/
      //		SPI_ArrieRegWrite(0x3D,SPI_CMD_READ_ST);
      WriteOneByte(ts, (unsigned char)0x3D, FLASH_CMD_READ_ST);
      /* Dummy data */
      //		SPI_ArrieRegWrite(0x3D,0);
      WriteOneByte(ts, (unsigned char)0x3D, (unsigned char)0x00);
      /* FLC_RxDATA */
      //		readData = SPI_ArrieRegRead(0x3F);
      readData = ReadOneByte(ts, 0x3F);
      /* FLC_CTL CS_HIGH,WP_DISABLE */
      //		SPI_ArrieRegWrite(0x3C,0x14);
      WriteOneByte(ts, (unsigned char)0x3C, (unsigned char)0x14);
    } while (readData & FLASH_ST_BUSY); 		/* check busy bit */
  }

  return 0;

 RETRY_ERROR:
  printk(KERN_INFO "FATAL: flash_write_page_shtsc retry %d times for page %d, addr %04X - FAILED!\n", retry, page, paddr);
  return 1;
}

#define FLASH_VERIFY_SIZE 512
unsigned char readBuf[FLASH_VERIFY_SIZE];

int flash_verify_page_shtsc(void *_ts, int page, unsigned char *data)
{
#if defined(CONFIG_TOUCHSCREEN_SHTSC_I2C)
  struct shtsc_i2c *ts = _ts;
#elif defined (CONFIG_TOUCHSCREEN_SHTSC_SPI)
  struct shtsc_spi *ts = _ts;
#else // (CONFIG_TOUCHSCREEN_SHTSC_UNKNOWNIF??)
  error
#endif // (CONFIG_TOUCHSCREEN_SHTSC_UNKNOWNIF??)
    unsigned addr; // in-page address (from 0 to FLASH_PAGE_SIZE-1)
  int cnt;

  /* BankChange */
  //	SPI_ArrieRegWrite(0x02,0x1C);
  WriteOneByte(ts, (unsigned char)0x02, (unsigned char)0x1C); // bank change reg

  /* CLKON_CTL0 */
  //	SPI_ArrieRegWrite(0x14,0x22);	/* CLKON_FLC,CLKON_REGBUS */
  WriteOneByte(ts, (unsigned char)0x14, (unsigned char)0x22);

  /* FLC_CTL CS_HIGH,WP_ENABLE */
  //	SPI_ArrieRegWrite(0x3C,0x10);
  WriteOneByte(ts, (unsigned char)0x3C, (unsigned char)0x10);
	
  /* FLC_CTL CS_LOW,WP_ENABLE */
  //	SPI_ArrieRegWrite(0x3C,0x12);
  WriteOneByte(ts, (unsigned char)0x3C, (unsigned char)0x12);

  /* FLC_TxDATA READ_COMMAND*/
  //	SPI_ArrieRegWrite(0x3D,SPI_CMD_READ);
  WriteOneByte(ts, (unsigned char)0x3D, FLASH_CMD_READ);
  /* FLC_TxDATA Address(bit16~23) */
  //	SPI_ArrieRegWrite(0x3D,((address>>16)&0xFF));
  WriteOneByte(ts, (unsigned char)0x3D, (unsigned char)0x00);
  /* FLC_TxDATA Address(bit8~15) */
  //	SPI_ArrieRegWrite(0x3D,((address>>8)&0xFF));
  WriteOneByte(ts, (unsigned char)0x3D, ((page<<4)&0xFF));
  /* FLC_TxDATA Address(bit0~7) */
  //	SPI_ArrieRegWrite(0x3D,(address&0xFF));
  WriteOneByte(ts, (unsigned char)0x3D, (unsigned char)0x00);
  /* FLC_TxDATA Dummy data */
  //	SPI_ArrieRegWrite(0x3D,0);
  WriteOneByte(ts, (unsigned char)0x3D, (unsigned char)0x00);

  for (addr = 0; addr < FLASH_PAGE_SIZE; addr += FLASH_VERIFY_SIZE) {
    for(cnt=0; cnt<FLASH_VERIFY_SIZE; cnt++){
      /* FLC_RxDATA */
      //		*pData++ = SPI_ArrieRegRead(0x3F);
      readBuf[cnt] = ReadOneByte(ts, 0x3F);
    }
    if (memcmp((unsigned char *)&(data[addr]), (unsigned char *)readBuf, FLASH_VERIFY_SIZE)) {
      goto VERIFY_ERROR;
    }
  }

  /* FLC_CTL CS_LOW,WP_ENABLE */
  //	SPI_ArrieRegWrite(0x3C,0x10);
  WriteOneByte(ts, (unsigned char)0x3C, (unsigned char)0x10);

  return 0;

 VERIFY_ERROR:
  /* FLC_CTL CS_LOW,WP_ENABLE */
  //	SPI_ArrieRegWrite(0x3C,0x10);
  WriteOneByte(ts, (unsigned char)0x3C, (unsigned char)0x10);
  // verify error
  printk(KERN_INFO "FATAL: flash_verify_page_shtsc for page %d - FAILED!\n", page);

  return 1;
}

int flash_read(void *_ts, unsigned address, unsigned length, unsigned char *data)
{
#if defined(CONFIG_TOUCHSCREEN_SHTSC_I2C)
  struct shtsc_i2c *ts = _ts;
#elif defined (CONFIG_TOUCHSCREEN_SHTSC_SPI)
  struct shtsc_spi *ts = _ts;
#else // (CONFIG_TOUCHSCREEN_SHTSC_UNKNOWNIF??)
  error
#endif // (CONFIG_TOUCHSCREEN_SHTSC_UNKNOWNIF??)
    int cnt;

  /* BankChange */
  //	SPI_ArrieRegWrite(0x02,0x1C);
  WriteOneByte(ts, (unsigned char)0x02, (unsigned char)0x1C); // bank change reg

  /* CLKON_CTL0 */
  //	SPI_ArrieRegWrite(0x14,0x22);	/* CLKON_FLC,CLKON_REGBUS */
  WriteOneByte(ts, (unsigned char)0x14, (unsigned char)0x22);

  /* FLC_CTL CS_HIGH,WP_ENABLE */
  //	SPI_ArrieRegWrite(0x3C,0x10);
  WriteOneByte(ts, (unsigned char)0x3C, (unsigned char)0x10);
	
  /* FLC_CTL CS_LOW,WP_ENABLE */
  //	SPI_ArrieRegWrite(0x3C,0x12);
  WriteOneByte(ts, (unsigned char)0x3C, (unsigned char)0x12);

  /* FLC_TxDATA READ_COMMAND*/
  //	SPI_ArrieRegWrite(0x3D,SPI_CMD_READ);
  WriteOneByte(ts, (unsigned char)0x3D, FLASH_CMD_READ);
  /* FLC_TxDATA Address(bit16~23) */
  //	SPI_ArrieRegWrite(0x3D,((address>>16)&0xFF));
  WriteOneByte(ts, (unsigned char)0x3D, (unsigned char)((address>>16)&0xFF));
  /* FLC_TxDATA Address(bit8~15) */
  //	SPI_ArrieRegWrite(0x3D,((address>>8)&0xFF));
  WriteOneByte(ts, (unsigned char)0x3D, (unsigned char)((address>>8)&0xFF));
  /* FLC_TxDATA Address(bit0~7) */
  //	SPI_ArrieRegWrite(0x3D,(address&0xFF));
  WriteOneByte(ts, (unsigned char)0x3D, (unsigned char)(address&0xFF));
  /* FLC_TxDATA Dummy data */
  //	SPI_ArrieRegWrite(0x3D,0);
  WriteOneByte(ts, (unsigned char)0x3D, (unsigned char)0x00);

  for(cnt=0; cnt<length; cnt++){
    /* FLC_RxDATA */
    //		*pData++ = SPI_ArrieRegRead(0x3F);
    data[cnt] = ReadOneByte(ts, 0x3F);
  }

  /* FLC_CTL CS_LOW,WP_ENABLE */
  //	SPI_ArrieRegWrite(0x3C,0x10);
  WriteOneByte(ts, (unsigned char)0x3C, (unsigned char)0x10);

  return 0;
}

#ifdef FORCE_FIRM_UPDATE
#define FLASH_WAIT 500
int Shtsc_power_enable(int enable)
{
    int rc = 0;
	if (enable)
	{
        if (gpio_is_valid(g_ts->vcc_io_pin)) 
        {
            rc = gpio_request(g_ts->vcc_io_pin,"TP_vddio_en_gpio");
            if (rc)
            {
                printk("request TP_vddio_en_gpio failed, rc=%d\n",rc);
            }
            else
            {            
                gpio_direction_output(g_ts->vcc_io_pin, 1);//config gpio to output
                printk("SHTP %s: enable TP_vddio_en_gpio \n", __func__);
                gpio_set_value((g_ts->vcc_io_pin), 1);
                msleep(5);
                gpio_free(g_ts->vcc_io_pin);
            }
        } 
        if (gpio_is_valid(g_ts->vcc_dc_pin)) 
        {
            rc = gpio_request(g_ts->vcc_dc_pin,"TP_vdddc_en_gpio");
            if (rc) 
            {
                printk("request TP_vdddc_en_gpio failed, rc=%d\n",rc);
            }
            else
            {
            
                gpio_direction_output(g_ts->vcc_dc_pin, 1);//config gpio to output
                printk("%s: enable tp_vddio_en_gpio \n", __func__);
                gpio_set_value((g_ts->vcc_dc_pin), 1);
                msleep(5);
                
                gpio_free(g_ts->vcc_dc_pin);
            }
        }
	}
	else
	{
        if (gpio_is_valid(g_ts->vcc_dc_pin)) 
        {
            rc = gpio_request(g_ts->vcc_dc_pin,"TP_vdddc_en_gpio");
            if (rc) 
            {
                printk("request TP_vdddc_en_gpio failed, rc=%d\n",rc);
            }
            else
            {
            
                gpio_direction_output(g_ts->vcc_dc_pin, 1);//config gpio to output
                printk("%s: disable tp_vddio_en_gpio \n", __func__);
                gpio_set_value((g_ts->vcc_dc_pin), 0);
                msleep(5);
                
                gpio_free(g_ts->vcc_dc_pin);
            }
        }
        if (gpio_is_valid(g_ts->vcc_io_pin)) 
        {
            rc = gpio_request(g_ts->vcc_io_pin,"TP_vddio_en_gpio");
            if (rc)
            {
                printk("request TP_vddio_en_gpio failed, rc=%d\n",rc);
            }
            else
            {            
                gpio_direction_output(g_ts->vcc_io_pin, 1);//config gpio to output
                printk("SHTP %s: disable TP_vddio_en_gpio \n", __func__);
                gpio_set_value((g_ts->vcc_io_pin), 0);
                msleep(5);
                gpio_free(g_ts->vcc_io_pin);
            }
        }
    }
	return rc;
}

/*
 * force re-programming the firm image held in the driver
 */
int update_flash(void *_ts, unsigned char *data, unsigned int len)
{
  int page;
  int err = 0;//zhangjian add
  int flash_update_error = 0;
  printk(KERN_INFO "shtsc: force updating K5 firmware....\n");
  printk(KERN_INFO "shtsc: flash_access start\n");
  Shtsc_update_flash =1;
  //Shtsc_power_enable(1);//zhangjian add
  flash_access_start_shtsc(_ts);

  for (page = 0; page < (len/FLASH_PAGE_SIZE); page++) {
    msleep(FLASH_WAIT);
    flash_erase_page_shtsc(_ts, page);
    printk(KERN_INFO "shtsc: flash_erase_page_shtsc done: page %d\n",  page);
    msleep(FLASH_WAIT);
    flash_write_page_shtsc(_ts, page, &(data[page*FLASH_PAGE_SIZE]));
    printk(KERN_INFO "shtsc: flash_write_page_shtsc done: page %d\n",  page);
    msleep(FLASH_WAIT);
    //zhangjian modify 
    #if 0
    flash_verify_page_shtsc(_ts, page, &(data[page*FLASH_PAGE_SIZE]));
    printk(KERN_INFO "shtsc: flash_verify_page_shtsc done: page %d\n",  page);
    #else
    err = flash_verify_page_shtsc(_ts, page, &(data[page*FLASH_PAGE_SIZE]));
    if (err)
    {
        printk(KERN_INFO "shtsc: flash_verify_page_shtsc error: page %d\n",  page);
        flash_update_error = 1;
    }
	else
	{
        printk(KERN_INFO "shtsc: flash_verify_page_shtsc done: page %d\n",  page);
	}
	#endif
    //end modify
  }

  printk(KERN_INFO "shtsc: flash_access end\n");
  
  Shtsc_update_flash =0;
  flash_access_end_shtsc(_ts);

  printk(KERN_INFO "shtsc: force updating K5 firmware....done\n");
  //zhangjian add 
  if (Shtsc_update_flash_sleep_no_power_off)
  {
      Shtsc_power_enable(0);
  }
  //add end
  if( flash_update_error )
  {
    flash_update_error = 0;
	return 1;
  }
  else
  {		
    return 0;
  }
}
#endif /* FORCE_FIRM_UPDATE */
#if 0 //zhangjian delete
int issue_command_msleep(void *ts, unsigned char *cmd, unsigned int len, int touch_clear, int mtime)
{
    int err = 0;
	
	SetBankAddr(ts, SHTSC_BANK_COMMAND);
    // set command
    WriteMultiBytes(ts, SHTSC_ADDR_COMMAND, cmd, len);
	//DBGLOG("shtsc: issue_command_msleep set command %d: %s\n", cmd[4] == 0x08 ? "GLOVE" : cmd[4] == 0x0D ? "COVER" : cmd[4] == 0x04 ? "DEEPIDLE" : cmd[4] == 0x03 ? "IDLE", "undefined");
	
    // prepare waiting
    ((struct shtsc_i2c *)ts)->cmd = cmd[0];
    ((struct shtsc_i2c *)ts)->wait_state = WAIT_CMD;
    ((struct shtsc_i2c *)ts)->wait_result = true;

    // do it
    SetIndicator(ts, SHTSC_IND_CMD);
    //delete by misaki on 8/5 //DBGLOG("do it\n");

    SetBankAddr(ts, SHTSC_BANK_TOUCH_REPORT);//add by misaki on 8/1

	msleep(mtime);

	if (touch_clear) {
		shtsc_system_init(ts);
	}

    return err;
}
#endif
int issue_command(void *ts, unsigned char *cmd, unsigned int len)
{
    int err = 0;

    shtsc_enable_irq(false);//moidfied by misaki 8/3 //disable_irq(g_ts->client->irq); //add by misaki on 8/1
	//delete by misaki on 8/5: DBGLOG("shtsc: issue_command irq : %d\n", g_ts->client->irq);

    SetBankAddr(ts, SHTSC_BANK_COMMAND);
    // set command
    WriteMultiBytes(ts, SHTSC_ADDR_COMMAND, cmd, len);
	DBGLOG("shtsc: issue_command set command %d: \n", cmd[4]);//modified by misaki on 8/5

    // prepare waiting
    ((struct shtsc_i2c *)ts)->cmd = cmd[0];
    ((struct shtsc_i2c *)ts)->wait_state = WAIT_CMD;
    ((struct shtsc_i2c *)ts)->wait_result = true;

    // do it
    SetIndicator(ts, SHTSC_IND_CMD);
//delete by misaki on 8/5    DBGLOG("do it\n");

    SetBankAddr(ts, SHTSC_BANK_TOUCH_REPORT);//add by misaki on 8/1

	shtsc_enable_irq(true);//add by misaki 8/3

	// wait
    err = WaitAsync(ts);

    return err;
}

int issue_command_with_touch_clear(void *ts, unsigned char *cmd, unsigned int len, int touch_clear)
{
    int err = 0, cnt; //add by misaki on 8/6

	for (cnt = 0; cnt < 3; cnt++) {//add by misaki on 8/6
		err = issue_command(ts, cmd, len);
		if (err == 0) {//add by misaki on 8/6
			if (touch_clear) 
				shtsc_system_init(ts);
			break;//add by misaki on 8/6
		}//add by misaki on 8/6
	}//add by misaki on 8/6

	return err;
}

int detectDevice(void)
{
  return DEVICE_CODE_LR388K5;
}

static void shtsc_reset_delay(void)
{
  msleep(SHTSC_RESET_TIME);
}

static void touch_enable (struct shtsc_i2c *ts)
{
  if(ts->disabled)
    {
#ifndef DEBUG_IRQ_DISABLE
		if(ts->client->irq) shtsc_enable_irq(true);//modified by misaki 8/3 //if(ts->client->irq) enable_irq(ts->client->irq);
#endif
      ts->disabled = false;
    }
}


static	void	touch_disable(struct shtsc_i2c *ts)
{
  if(!ts->disabled)	{
    if(ts->client->irq)	shtsc_enable_irq(false);//modified by misaki 8/3	//disable_irq(ts->client->irq);
    ts->disabled = true;
  }
}

static int shtsc_input_open(struct input_dev *dev)
{
  struct shtsc_i2c *ts = input_get_drvdata(dev);

  touch_enable(ts);

  printk("%s\n", __func__);

  return 0;
}

static void shtsc_input_close(struct input_dev *dev)
{
  struct shtsc_i2c *ts = input_get_drvdata(dev);

  touch_disable(ts);

  printk("%s\n", __func__);
}

#ifdef CONFIG_OF
static int shtsc_get_dt_coords(struct device *dev, char *name,
			       struct shtsc_i2c_pdata *pdata)
{
  u32 coords[SHTSC_COORDS_ARR_SIZE];
  struct property *prop;
  struct device_node *np = dev->of_node;
  int coords_size, rc;

  prop = of_find_property(np, name, NULL);
  if (!prop)
    return -EINVAL;
  if (!prop->value)
    return -ENODATA;

  coords_size = prop->length / sizeof(u32);
  if (coords_size != SHTSC_COORDS_ARR_SIZE) {
    dev_err(dev, "invalid %s\n", name);
    return -EINVAL;
  }

  rc = of_property_read_u32_array(np, name, coords, coords_size);
  if (rc && (rc != -EINVAL)) {
    dev_err(dev, "Unable to read %s\n", name);
    return rc;
  }

  if (strncmp(name, "sharp,panel-coords",
	      sizeof("sharp,panel-coords")) == 0) {
    pdata->panel_minx = coords[0];
    pdata->panel_miny = coords[1];
    pdata->panel_maxx = coords[2];
    pdata->panel_maxy = coords[3];
  } else if (strncmp(name, "sharp,display-coords",
		     sizeof("sharp,display-coords")) == 0) {
    pdata->disp_minx = coords[0];
    pdata->disp_miny = coords[1];
    pdata->disp_maxx = coords[2];
    pdata->disp_maxy = coords[3];
  } else {
    dev_err(dev, "unsupported property %s\n", name);
    return -EINVAL;
  }

  return 0;
}


static int shtsc_parse_dt(struct device *dev, struct shtsc_i2c_pdata *pdata)
{
  struct device_node *np = dev->of_node;
  int rc;
  u32 temp_val;

  rc = shtsc_get_dt_coords(dev, "sharp,panel-coords", pdata);
  if (rc)
    return rc;

  rc = shtsc_get_dt_coords(dev, "sharp,display-coords", pdata);
  if (rc)
    return rc;

  /* regulator info */
  pdata->i2c_pull_up = of_property_read_bool(np, "sharp,i2c-pull-up");

  /* reset, irq gpio info */
  pdata->reset_gpio = of_get_named_gpio_flags(np, "sharp,reset-gpio",0, &pdata->reset_gpio_flags);
  pdata->irq_gpio = of_get_named_gpio_flags(np, "sharp,irq-gpio",0, &pdata->irq_gpio_flags);
  //zhangjian add
  pdata->vcc_io_gpio = of_get_named_gpio_flags(np, "sharp,vcc-io-gpio",0, &pdata->vcc_io_gpio_flags);
  pdata->vcc_dc_gpio = of_get_named_gpio_flags(np, "sharp,vcc-dc-gpio",0, &pdata->vcc_dc_gpio_flags); 
  //add end
  rc = of_property_read_u32(np, "ts_touch_num_max", &temp_val);
  if( !rc ) pdata->ts_touch_num_max = temp_val;
  rc = of_property_read_u32(np, "ts_pressure_max", &temp_val);
  if( !rc ) pdata->ts_pressure_max = temp_val;
  rc = of_property_read_u32(np, "ts_flip_x", &temp_val);
  if( !rc ) pdata->ts_flip_x = temp_val;
  rc = of_property_read_u32(np, "ts_flip_y", &temp_val);
  if( !rc ) pdata->ts_flip_y = temp_val;
  rc = of_property_read_u32(np, "ts_swap_xy", &temp_val);
  if( !rc ) pdata->ts_swap_xy = temp_val;
#if 1
  printk(KERN_INFO "[SHTP PARSE DT] reset gpio = %d\n",pdata->reset_gpio);
  printk(KERN_INFO "[SHTP PARSE DT] irq gpio = %d\n",pdata->irq_gpio);
#endif
  return 0;
}
#else
static int shtsc_parse_dt(struct device *dev, struct shtsc_i2c_pdata *pdata)
{
  return -ENODEV;
}
#endif

#if defined(CONFIG_FB)
static int shtsc_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct shtsc_i2c *data = i2c_get_clientdata(client);
	//	struct input_dev *input_dev = data->input;
    int rc;
	if (data->dev_sleep) {
	        DBGLOG("Device already in sleep\n");
		return 0;
	} else { //add by misaki on 8/1
		DBGLOG("shtsc_suspend  START en_deep:%d, cover:%d glove:%d vcc_dc_pin:%d \n", 
			g_ts->enable_wakeup_gesture, g_ts->cover_mode, g_ts->glove_mode, gpio_is_valid(data->vcc_dc_pin)); //add by misaki on 8/1
	} //add by misaki on 8/1

#ifdef ENABLE_SUSPEND_GESTURE
//zhangjian modify
#if 0
	/* issue SetSystemState(DeepIdle) command to activate gesture */

	issue_command(g_ts, CMD_SETSYSTEMSTATE_DEEPIDLE, CMD_SETSYSTEMSTATE_DEEPIDLE_LEN);
	DBGLOG("shtsc: issue deepidle command\n");
	DBGLOG("shtsc: enable irq IRQ_NO:%d\n", data->client->irq);
	enable_irq_wake(data->client->irq);
#else
    if (g_ts->enable_wakeup_gesture)
    {
        g_ts->suspend_enter_deepidle = 1;      	
		/* issue SetSystemState(DeepIdle) command to activate gesture */
       
       	issue_command_with_touch_clear(g_ts, CMD_SETSYSTEMSTATE_DEEPIDLE, CMD_SETSYSTEMSTATE_DEEPIDLE_LEN, true);//modified by misaki on 8/4 //issue_command(g_ts, CMD_SETSYSTEMSTATE_DEEPIDLE, CMD_SETSYSTEMSTATE_DEEPIDLE_LEN);
       	//delete by misaki on 8/4 msleep(10); 
       	DBGLOG("shtsc: issue deepidle command\n");
       	DBGLOG("shtsc: enable irq IRQ_NO:%d\n", data->client->irq);
       	enable_irq_wake(data->client->irq);
    }
	else
	{
        shtsc_enable_irq(false); //modified by misaki 8/3 //disable_irq(data->client->irq);
        g_ts->suspend_enter_deepidle = 0;
        DBGLOG("Device in sleep\n");
        //zhangjian add
        if(!Shtsc_update_flash)
        {
            Shtsc_update_flash_sleep_no_power_off = 0;
			shtsc_reset(g_ts, false);
            if (gpio_is_valid(data->vcc_dc_pin)) 
            {
                rc = gpio_request(data->vcc_dc_pin,"TP_vdddc_en_gpio");
                if (rc) 
                {
                    printk("request TP_vdddc_en_gpio failed, rc=%d\n",rc);
                }
                else
                {
                
                    gpio_direction_output(data->vcc_dc_pin, 1);//config gpio to output
                    printk("%s: disable tp_vddio_en_gpio \n", __func__);
                    gpio_set_value((data->vcc_dc_pin), 0);
                    msleep(5);
                    
                    gpio_free(data->vcc_dc_pin);
                }
            }
            if (gpio_is_valid(data->vcc_io_pin)) 
            {
                rc = gpio_request(data->vcc_io_pin,"TP_vddio_en_gpio");
                if (rc)
                {
                    printk("request TP_vddio_en_gpio failed, rc=%d\n",rc);
                }
                else
                {            
                    gpio_direction_output(data->vcc_io_pin, 1);//config gpio to output
                    printk("SHTP %s: disable TP_vddio_en_gpio \n", __func__);
                    gpio_set_value((data->vcc_io_pin), 0);
                    msleep(5);
                    gpio_free(data->vcc_io_pin);
                }
            }
        }
        else
        {
            Shtsc_update_flash_sleep_no_power_off = 1;
            printk("SHTP %s: TP sleep Shtsc update flash no power off \n", __func__);
        }
	}
#endif
#else /* ENABLE_SUSPEND_GESTURE */
	shtsc_enable_irq(false);//modified by misaki 8/3 //disable_irq(data->client->irq);

	DBGLOG("Device in sleep\n");
#endif /* ENABLE_SUSPEND_GESTURE */
	
	data->dev_sleep = true;

	DBGLOG("shtsc_suspend END en_deep:%d, cover:%d glove:%d vcc_dc_pin:%d \n", 
		   g_ts->suspend_enter_deepidle, g_ts->cover_mode, g_ts->glove_mode, gpio_is_valid(data->vcc_dc_pin)); //add by misaki on 8/1
	return 0;
}

static int shtsc_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct shtsc_i2c *data = i2c_get_clientdata(client);
	//	struct input_dev *input_dev = data->input;

	if (!data->dev_sleep) {
	        DBGLOG( "Device already in resume\n");
		return 0;
	} else { //add by misaki on 8/1
		DBGLOG("shtsc_resume  START en_deep:%d, cover:%d glove:%d \n", 
			   g_ts->suspend_enter_deepidle, g_ts->cover_mode, g_ts->glove_mode); //add by misaki on 8/1
	} //add by misaki on 8/1

    Shtsc_update_flash_sleep_no_power_off = 0;
#ifdef ENABLE_SUSPEND_GESTURE
//zhangjian add for wake up gesture
#if 0
	DBGLOG("shtsc: disable irq wake IRQ_NO:%d\n", data->client->irq);
	disable_irq_wake(data->client->irq);
    //shtsc_reset_L_H(g_ts); //reset and to idle //add by misaki on 8/1 
    enable_irq(g_ts->client->irq);//add by misaki on 8/1

#if 0 // no reset on resume
	// reset K5 but not enable_irq()
	shtsc_reset(g_ts, false);

//delete by misaki on 8/4 	G_Irq_Mask = 0xffff;
#ifdef GET_REPORT
	reportBuf[0] = (unsigned char)0;
#endif /* GET_REPORT */

	//	shtsc_reset_delay();

	g_ts->wait_state = WAIT_RESET;
	g_ts->wait_result = false;
	shtsc_reset(g_ts, true);
	// wait
	WaitAsync(g_ts);

	//	shtsc_reset(g_ts, true);
	//	msleep(10);
	shtsc_system_init(g_ts);
	//	msleep(100);

#endif // no reset on resume

	DBGLOG("Device in active\n");

#else
  if (g_ts->suspend_enter_deepidle)
    {
        DBGLOG("shtsc: disable irq wake IRQ_NO:%d\n", data->client->irq);

        disable_irq_wake(data->client->irq);//move to here by misaki on 8/17
        shtsc_reset_L_H(g_ts); //add by misaki on 8/3
        TP_resume_reset = true;//zhangjian add
       #if 0 //zhangjian delete ,no need
       if(g_ts->cover_mode)
       {
           issue_command_with_touch_clear(g_ts,CMD_SETSYSTEMSTATE_COVER, CMD_SETSYSTEMSTATE_COVER_LEN, true);//modified by misaki on 8/4 //issue_command(g_ts,CMD_SETSYSTEMSTATE_COVER, CMD_SETSYSTEMSTATE_COVER_LEN);
           DBGLOG("shtsc: issue cover command\n");
       }
	   else if(g_ts->glove_mode)//glove enable
       {
           issue_command_with_touch_clear(g_ts,CMD_SETSYSTEMSTATE_GLOVE, CMD_SETSYSTEMSTATE_GLOVE_LEN, true);//modified by misaki on 8/4//issue_command(g_ts,CMD_SETSYSTEMSTATE_GLOVE, CMD_SETSYSTEMSTATE_GLOVE_LEN);
           DBGLOG("shtsc: issue glove command\n");
       }
      #endif //end modify
      DBGLOG("Device in active finished(%d)\n", g_ts->suspend_enter_deepidle); //modified by misaki on 8/17
     
		
    }
    else
	{
        DBGLOG("shtsc: else g_ts->suspend_enter_deepidle irq wake IRQ_NO:%d\n", data->client->irq);
		shtsc_reset_L_H(g_ts); //add by misaki on 8/1
		TP_resume_reset = true;//zhangjian add
    #if 0 //zhangjian delete ,no need
	if(g_ts->cover_mode)
    {
		issue_command_with_touch_clear(g_ts,CMD_SETSYSTEMSTATE_COVER, CMD_SETSYSTEMSTATE_COVER_LEN, true);//modified by misaki on 8/4//issue_command(g_ts,CMD_SETSYSTEMSTATE_COVER, CMD_SETSYSTEMSTATE_COVER_LEN);
        DBGLOG("shtsc: issue cover command\n");
    }
    else if(g_ts->glove_mode)//glove enable
    {
        issue_command_with_touch_clear(g_ts,CMD_SETSYSTEMSTATE_GLOVE, CMD_SETSYSTEMSTATE_GLOVE_LEN, true);//modified by misaki on 8/4//issue_command(g_ts,CMD_SETSYSTEMSTATE_GLOVE, CMD_SETSYSTEMSTATE_GLOVE_LEN);
        DBGLOG("shtsc: issue glove command\n");
    }
    #endif //end modify
    DBGLOG("Device in active finished(%d)\n", g_ts->suspend_enter_deepidle); //modified by misaki on 8/17
	}
#endif
//end modify 
#else /* ENABLE_SUSPEND_GESTURE */
  DBGLOG("shtsc: else g_ts->suspend_enter_deepidle irq wake IRQ_NO:%d\n", data->client->irq);
  shtsc_reset_L_H(g_ts); //add by misaki on 8/1


#endif /* ENABLE_SUSPEND_GESTURE */

	data->dev_sleep = false;

	DBGLOG("shtsc_resume END en_deep:%d, cover:%d glove:%d \n", 
		   g_ts->suspend_enter_deepidle, g_ts->cover_mode, g_ts->glove_mode); //add by misaki on 8/1

	return 0;
}

static int fb_notifier_callback(struct notifier_block *self,
				 unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	int *blank;

	struct shtsc_i2c *shtsc_dev_data =
		container_of(self, struct shtsc_i2c, fb_notif);

	if (evdata && evdata->data && shtsc_dev_data && shtsc_dev_data->client) {
		if (event == FB_EVENT_BLANK) {
			blank = evdata->data;
			if (*blank == FB_BLANK_UNBLANK)
				shtsc_resume(&shtsc_dev_data->client->dev);
			else if (*blank == FB_BLANK_POWERDOWN)
				shtsc_suspend(&shtsc_dev_data->client->dev);
		}
	}
	return 0;
}
#endif

//zhangjian add for wake_gesture

static ssize_t shtsc_wake_gesture_show(struct kobject *kobj, struct kobj_attribute *attr,char *buf)
{
   	
   return snprintf(buf, PAGE_SIZE, "%u,\n",
			g_ts->enable_wakeup_gesture);
}

static ssize_t shtsc_wake_gesture_store(struct kobject *kobj, struct kobj_attribute *attr,
			 const char *buf, size_t count)
 {   
	unsigned int input;

	if (sscanf(buf, "%u", &input) != 1)
		return -EINVAL;

	input = input > 0 ? 1 : 0;
    #if defined(DEBUG_SHTSC)
    printk(KERN_INFO "wake_gesture input is %d \n",input);
	#endif
    g_ts->enable_wakeup_gesture = input;
	return count;

 }

static ssize_t shtsc_glove1_show(struct kobject *kobj, struct kobj_attribute *attr,char *buf)
{
   

   return snprintf(buf, PAGE_SIZE, "%d\n", g_ts->glove_mode);
}

static ssize_t shtsc_glove1_store(struct kobject *kobj, struct kobj_attribute *attr,
			 const char *buf, size_t count)
 {
   
	unsigned int input;

	if (sscanf(buf, "%u", &input) != 1)
		return -EINVAL;

	input = input > 0 ? 1 : 0;
    g_ts->glove_mode = 0;//input; zhangjian modify delete glove mode
	if(input)//glove enable
	{
        shtsc_reset_L_H(g_ts);
		issue_command_with_touch_clear(g_ts,CMD_SETSYSTEMSTATE_GLOVE, CMD_SETSYSTEMSTATE_GLOVE_LEN,true);//modified by misaki on 8/4//issue_command(g_ts,CMD_SETSYSTEMSTATE_GLOVE, CMD_SETSYSTEMSTATE_GLOVE_LEN);
        //delete by misaki on 8/4 msleep(10);
        
        DBGLOG("shtsc: issue glove command\n");
	}
	else//glove disable
    {
       shtsc_reset_L_H(g_ts);
	   issue_command_with_touch_clear(g_ts, CMD_SETSYSTEMSTATE_IDLE, CMD_SETSYSTEMSTATE_IDLE_LEN,true);//modified by misaki on 8/4//issue_command(g_ts, CMD_SETSYSTEMSTATE_IDLE, CMD_SETSYSTEMSTATE_IDLE_LEN);
       //delete by misaki on 8/4 msleep(10);
        
        DBGLOG("shtsc: issue idle command\n");
	}
	return count;

 }

static ssize_t shtsc_glove_show(struct kobject *kobj, struct kobj_attribute *attr,char *buf)
{
   

   return snprintf(buf, PAGE_SIZE, "%d\n", g_ts->cover_mode);
}

static ssize_t shtsc_glove_store(struct kobject *kobj, struct kobj_attribute *attr,
			 const char *buf, size_t count)
 {
   
	unsigned int input;

	if (sscanf(buf, "%u", &input) != 1)
		return -EINVAL;

	input = input > 0 ? 1 : 0;
    g_ts->cover_mode = input;
	DBGLOG("shtsc: g_ts->cover_mode is %d\n",g_ts->cover_mode);
    if(!g_ts->dev_sleep)
    {
        if(input)//cover enable
        {
            if (TP_resume_reset)
            {
                DBGLOG("shtsc: TP_resume_reset is true no need reset\n");
				TP_resume_reset =false;            
            }
            else	
            {                
                DBGLOG("shtsc: TP_resume_reset is false reset\n");
				shtsc_reset_L_H(g_ts);                
            }
			issue_command_with_touch_clear(g_ts,CMD_SETSYSTEMSTATE_COVER, CMD_SETSYSTEMSTATE_COVER_LEN, true);//modified by misaki on 8/4//issue_command(g_ts,CMD_SETSYSTEMSTATE_COVER, CMD_SETSYSTEMSTATE_COVER_LEN);
            //delete by misaki on 8/4 msleep(10);            
            DBGLOG("shtsc: issue COVER command\n");
        }
        else//cover disable
        {
            if(g_ts->glove_mode)//glove enable
            {
                if (TP_resume_reset)
                {
                    DBGLOG("shtsc: TP_resume_reset is true no need reset\n");
					TP_resume_reset =false;                
                }
                else	
                {                
                    DBGLOG("shtsc: TP_resume_reset is false reset\n");
					shtsc_reset_L_H(g_ts);
                }
				issue_command_with_touch_clear(g_ts,CMD_SETSYSTEMSTATE_GLOVE, CMD_SETSYSTEMSTATE_GLOVE_LEN, true);//modified by misaki on 8/4//issue_command(g_ts,CMD_SETSYSTEMSTATE_GLOVE, CMD_SETSYSTEMSTATE_GLOVE_LEN);
                //delete by misaki on 8/4 msleep(10);
                DBGLOG("shtsc: issue glove command\n");
            }
            else
            {
                if (TP_resume_reset)
                {
                    DBGLOG("shtsc: TP_resume_reset is true no need reset\n");
					TP_resume_reset =false;                
                }
                else	
                {                
                    DBGLOG("shtsc: TP_resume_reset is false reset\n");
					shtsc_reset_L_H(g_ts);
                }
				issue_command_with_touch_clear(g_ts, CMD_SETSYSTEMSTATE_IDLE, CMD_SETSYSTEMSTATE_IDLE_LEN, true);//modified by misaki on 8/4//issue_command(g_ts, CMD_SETSYSTEMSTATE_IDLE, CMD_SETSYSTEMSTATE_IDLE_LEN);
                //delete by misaki on 8/4 msleep(10);
                DBGLOG("shtsc: issue idle command\n");
            }
        }
    }
	return count;

 }


static struct kobj_attribute shtsc_wake_gesture_attribute =
	__ATTR(wake_gesture, 0666,
	shtsc_wake_gesture_show,
	shtsc_wake_gesture_store);

static struct kobj_attribute shtsc_glove_attribute =
	__ATTR(glove_mode, 0666,
	shtsc_glove_show,
	shtsc_glove_store);
static struct kobj_attribute shtsc_glove1_attribute =
	__ATTR(glove1_mode, 0666,
	shtsc_glove1_show,
	shtsc_glove1_store);

static struct attribute *wake_gesture_attrs[] = {
	&shtsc_wake_gesture_attribute.attr,
	&shtsc_glove_attribute.attr,
	&shtsc_glove1_attribute.attr,
	NULL,	/* need to NULL terminate the list of attributes */
};

static struct attribute_group attr_group = {
	.attrs = wake_gesture_attrs,
};

static struct kobject *shtsc_wake_gesture_kobj;

static ssize_t sh_tpd_read(struct file *file, char __user *page, size_t size, loff_t *ppos)
{
   int len = 0;
   uint8_t buffer_tpd[800];
	

   len += sprintf(buffer_tpd+len, "name : %s\n", "Sharp");
   len += sprintf(buffer_tpd+len, "I2C address: 0x%x\n", 0x18);

    len += sprintf(buffer_tpd+len, "Mstar IC type : %s\n", "lR388K5");

    len += sprintf(buffer_tpd+len, "module : %s\n", "Sharp");
    len += sprintf(buffer_tpd+len, "Firmware version : %x \n",Firmware_version);
	
    len += sprintf(buffer_tpd+len, "Param version : %x \n",Param_version);
 	
	return simple_read_from_buffer(page, size, ppos, buffer_tpd, len);
	
}


static const struct file_operations shtsc_id = {
        .owner = THIS_MODULE,
        .read = sh_tpd_read,
        .write = NULL,
    };

//zhangjian add for I2C test
#if 0
static s8 Shtp_i2c_test(void *_ts)
{
    struct shtsc_i2c *ts = _ts;
	u8 chip_id =0; 
	u8 retry=0;
    WriteOneByte(ts, SHTSC_ADDR_BANK, 0x18);
    while(retry++ < 5)
    {
        chip_id = ReadOneByte(ts, 0x0D);
        if (chip_id == 0x1F)
        {
            return chip_id;
        }
        printk("Shtcs i2c test failed time %d.",retry);
        msleep(10);
    }
    return chip_id;
}
#else
static int Shtp_i2c_test(void *_ts, u8 *chip_id)
{
    struct shtsc_i2c *ts = _ts;
	u8 retry=0;
	int r;

	r = SetBankAddr(ts, 0x18);
	if (r < 0)
		return r;

	while(retry++ < 5)
    {
        *chip_id = ReadOneByte(ts, 0x0D);
        if (*chip_id == 0x1F)
        {		
		    break;
        }
		printk("shtsc: i2c test failed time %d.",retry);
        msleep(10);
    }
	printk("shtsc: chip_id is %x\n",*chip_id);
	r = SetBankAddr(ts, SHTSC_BANK_TOUCH_REPORT);

	return r;
}

#endif
// add end
#if defined(CONFIG_TOUCHSCREEN_SHTSC_I2C)
static int  shtsc_i2c_probe(struct i2c_client *client,
				     const struct i2c_device_id *id)
{
  //  const struct shtsc_pdata *pdata = client->dev.platform_data;
  struct shtsc_i2c_pdata *pdata;
  struct shtsc_i2c *ts;
  struct input_dev *input_dev;
  int err;
  int cnt , r;  
  struct pinctrl *pinctrl; /*zhangjian add for shtsc_irq_pins */
  struct proc_dir_entry *mt_entry = NULL;//zhangjian add
  u8 shtsc_chip_id = 0;//zhangjian add
#if defined(DEBUG_SHTSC)
  printk(KERN_INFO "[ENTER] shtsc_probe\n");
#endif

  //2014.10.16 added
  if (client->dev.of_node) {
    pdata = devm_kzalloc(&client->dev,
			 sizeof(struct shtsc_i2c_pdata), GFP_KERNEL);
    if (!pdata) {
      dev_err(&client->dev, "Failed to allocate memory\n");
      return -ENOMEM;
    }
    err = shtsc_parse_dt(&client->dev, pdata);
    if (err){
      kfree(pdata);
      return err;
    }
  } else{
    pdata = client->dev.platform_data;
  }

  /* No pdata no way forward */
  if (pdata == NULL) {
    dev_err(&client->dev, "no pdata\n");
#if defined(DEBUG_SHTSC)
    printk(KERN_INFO "%s(%d):\n", __FILE__, __LINE__);
#endif
    return -ENODEV;
  }

  if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_READ_WORD_DATA))
    return -EIO;

  ts = kzalloc(sizeof(struct shtsc_i2c), GFP_KERNEL);
  g_ts = ts;

  input_dev = input_allocate_device();
  if (!ts || !input_dev) {
    err = -ENOMEM;
    goto err_free_mem;
  }

  ts->client = client;
  ts->input = input_dev;
  ts->pdata = pdata;//2014.10.16 added

  ts->min_x = pdata->panel_minx;
  ts->max_x = pdata->panel_maxx;
  ts->min_y = pdata->panel_miny;
  ts->max_y = pdata->panel_maxy;
  ts->pressure_max = pdata->ts_pressure_max;
  ts->touch_num_max = pdata->ts_touch_num_max;//2014.10.16 added
  ts->flip_x = pdata->ts_flip_x;
  ts->flip_y = pdata->ts_flip_y;
  ts->swap_xy = pdata->ts_swap_xy;

  ts->reset_pin = pdata->reset_gpio;
  ts->irq_pin = pdata->irq_gpio;
  //zhangjian add
  ts->vcc_dc_pin = pdata->vcc_dc_gpio;
  ts->vcc_io_pin = pdata->vcc_io_gpio;
  ts->enable_wakeup_gesture = 0;
  ts->glove_mode = 0;
  ts->cover_mode = 0;
  //add end
  mutex_init(&(ts->mutex));

  snprintf(ts->phys, sizeof(ts->phys),
	   "%s/input0", dev_name(&client->dev));

  input_dev->name = SHTSC_DRIVER_NAME;
  input_dev->phys = ts->phys;
  input_dev->id.bustype = BUS_I2C;
  input_dev->dev.parent = &client->dev;
  input_dev->open = shtsc_input_open;//2014.10.16 added
  input_dev->close = shtsc_input_close;//2014.10.16 added

  __set_bit(EV_ABS, input_dev->evbit);
  __set_bit(EV_KEY, input_dev->evbit);
  __set_bit(BTN_TOUCH, input_dev->keybit);//2014.10.16 added
  __set_bit(INPUT_PROP_DIRECT, input_dev->propbit);//2014.10.16 added

  /* For multi-touch */
#if 1 //zhangjian modify //(LINUX_VERSION_CODE >= KERNEL_VERSION(3,7,0))
  err = input_mt_init_slots(input_dev, SHTSC_MAX_FINGERS, INPUT_MT_DIRECT);
#else /*  KERNEL_3_10 */
  err = input_mt_init_slots(input_dev, SHTSC_MAX_FINGERS);
#endif /* (LINUX_VERSION_CODE >= KERNEL_VERSION(3,7,0)) */

  if (err)
    goto err_free_mem;
  input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, SHTSC_I2C_SIZE_MAX, 0, 0);
  input_set_abs_params(input_dev, ABS_MT_POSITION_X,
		       pdata->disp_minx, pdata->disp_maxx, 0, 0);
  input_set_abs_params(input_dev, ABS_MT_POSITION_Y,
		       pdata->disp_miny, pdata->disp_maxy, 0, 0);
  input_set_abs_params(input_dev, ABS_MT_PRESSURE  , 0, SHTSC_I2C_P_MAX, 0, 0);
  input_set_abs_params(input_dev, ABS_MT_TOOL_TYPE , 0, MT_TOOL_MAX, 0, 0);

#ifdef SHTSC_ICON_KEY_NUM
  {
    int cnt;
    for(cnt=0;cnt<SHTSC_ICON_KEY_NUM;cnt++){
      input_set_capability( input_dev , EV_KEY , shtsc_keyarray[cnt] );
    }
  }
#endif

#ifdef ENABLE_SUSPEND_GESTURE
  input_set_capability( input_dev , EV_KEY , KEY_GESTURE_DOUBLE_TAP );
  input_set_capability( input_dev , EV_KEY , KEY_GESTURE_CHAR_C );
  input_set_capability( input_dev , EV_KEY , KEY_GESTURE_CHAR_E );
  input_set_capability( input_dev , EV_KEY , KEY_GESTURE_CHAR_M );
  input_set_capability( input_dev , EV_KEY , KEY_GESTURE_CHAR_O );
  input_set_capability( input_dev , EV_KEY , KEY_GESTURE_CHAR_S );
  input_set_capability( input_dev , EV_KEY , KEY_GESTURE_CHAR_W );
  input_set_capability( input_dev , EV_KEY , KEY_POWER );
#endif /* ENABLE_SUSPEND_GESTURE */

  if (ts->reset_pin) {
    err = gpio_request(ts->reset_pin, NULL);
    if (err) {
      dev_err(&client->dev,
	      "Unable to request GPIO pin %d.\n",
	      ts->reset_pin);
      goto err_free_mem;
    }
  }

  shtsc_reset(ts, false);
  //shtsc_reset_delay(ts);//2014.11.12 added

  if (gpio_is_valid(ts->irq_pin)) {//2014.11.12 added
   /*zhangjian add for shtsc_irq_pins start*/
	pinctrl = devm_pinctrl_get_select(&(client->dev), "shtsc_irq_active");
	if (IS_ERR(pinctrl)){
		dev_err(&client->dev,
			"pins are not configured from the driver\n");
	}
   /*zhangjian add for shtsc_irq_pins end*/

  
    /* configure touchscreen irq gpio */
    err = gpio_request(ts->irq_pin, "shtsc_irq_gpio");
    if (err) {
      dev_err(&client->dev, "unable to request gpio [%d]\n",
	      ts->irq_pin);
      goto err_free_irq_gpio;
    }
  }
  err = gpio_direction_input(ts->irq_pin);
  if (err < 0) {
    dev_err(&client->dev,
	    "Failed to configure input direction for GPIO %d, error %d\n",
	    ts->irq_pin, err);
    goto err_free_irq_gpio;
  }

  client->irq = gpio_to_irq(ts->irq_pin);
  if (client->irq < 0) {
    err = client->irq;
    dev_err(&client->dev,
	    "Unable to get irq number for GPIO %d, error %d\n",
	    ts->irq_pin, err);
    goto err_free_irq_gpio;
  }
 //zhangjian add for I2C test
 #if 0 
 shtsc_reset(ts, true);
  shtsc_chip_id = Shtp_i2c_test(ts);
  printk("shtsc_chip_id is %x\n",shtsc_chip_id);
  if(shtsc_chip_id != 0x1F)
  {
      printk("I2C test fail\n");
      goto err_free_irq_gpio;
  }
  shtsc_reset(ts, false);
 #endif 
#if 0
#ifdef FORCE_FIRM_UPDATE
  ts->workqueue = create_singlethread_workqueue("shtsc_work");
  INIT_WORK(&ts->workstr, update_flash_func);
#endif /* FORCE_FIRM_UPDATE */
#endif /* 0 */

#ifdef D_ENABLE_GESTURE_WORK_QUEUE //add by misaki on 8/6
  ts->workqueue = create_singlethread_workqueue("shtsc_work");
  INIT_WORK(&ts->workstr, irq_gesture_thread);
#endif /* D_ENABLE_GESTURE_WORK_QUEUE //add by misaki on 8/6 */

#ifndef DEBUG_IRQ_DISABLE
  err = request_threaded_irq(client->irq, NULL, shtsc_irq_thread,
                             (IRQF_TRIGGER_HIGH|IRQF_ONESHOT), client->dev.driver->name, ts);
#endif

  if (err < 0) {
    dev_err(&client->dev,
	    "irq %d busy? error %d\n", client->irq, err);
    goto err_free_irq_gpio;
  }

  input_set_drvdata(input_dev, ts);//2014.10.16 added
  err = input_register_device(input_dev);
  if (err)
    goto err_free_irq;

  i2c_set_clientdata(client, ts);
  device_init_wakeup(&client->dev, 1);
//zhangjian modify
#if 0
  ts->wait_state = WAIT_RESET;
  ts->wait_result = false;
  shtsc_reset(ts, true);
  // wait
  err = WaitAsync(ts);
  if (err)
    goto err_free_irq;
#else
  ts->wait_state = WAIT_RESET;
  ts->wait_result = false;
  disable_irq(g_ts->client->irq);//modified by misaki on 8/3
  g_ts->enable_irq_status = false;//modified by misaki on 8/3
  shtsc_reset(ts, true);
  for( cnt=0 ;; cnt++ ){
    if( gpio_get_value(ts->irq_pin) ) break;
    if( cnt>60 ) goto err_free_irq;
    msleep(10);
  }
  r = Shtp_i2c_test(ts, &shtsc_chip_id);
  if (r < 0)
    goto err_free_irq;
  if(shtsc_chip_id != 0x1F) 
    goto err_free_irq;
  shtsc_enable_irq(true);//modified by misaki on 8/3 //enable_irq(client->irq);
  // wait
  err = WaitAsync(ts);
  if (err)
    goto err_free_irq;
#endif
//end modify
#ifdef FORCE_FIRM_UPDATE
  for( cnt=0 ;; cnt++ ){ //add by misaki on 8/3
	if (FlashUpdateByNoFirm > 0) break; 
    if( cnt>60 ) goto err_free_irq; 
    msleep(10); 
  } //add by misaki on 8/3
  printk(KERN_INFO "shtsc: force firmware update from probe...\n");
  update_flash_func();
  printk(KERN_INFO "shtsc: force firmware update from probe... done\n");
#endif /* FORCE_FIRM_UPDATE */

  VersionModelCode = detectDevice();
  shtsc_system_init(ts);

#if defined(CONFIG_FB)
  ts->fb_notif.notifier_call = fb_notifier_callback;
  ts->dev_sleep = false;
  err = fb_register_client(&ts->fb_notif);

  if (err)
    dev_err(&ts->client->dev, "Unable to register fb_notifier: %d\n",
      err);
#endif
  //zhangjain add for shtsc_wake_gesture and glove mod
  
  shtsc_wake_gesture_kobj = kobject_create_and_add("syna_wake_gesture", NULL);
  if (!shtsc_wake_gesture_kobj)
  {
      err = -EINVAL;
      dev_err(&ts->client->dev, "Unable to create shtsc_wake_gesture_kobj\n");
  }
  /* Create the files associated with this kobject */
  err = sysfs_create_group(shtsc_wake_gesture_kobj, &attr_group);
  if (err)
  {
      kobject_put(shtsc_wake_gesture_kobj);
      dev_err(&ts->client->dev, "Unable to create sysfs group\n");
  }
 //zhangjian add for wakeup gesture
  mt_entry = proc_create("driver/ts_information", 0644, NULL, &shtsc_id);
  if (mt_entry)
  {
      printk("create tsc_id success\n");
  }
  else
  printk("create tsc_id fail\n");

#if defined(DEBUG_SHTSC)
  printk(KERN_INFO "[EXIT] shtsc_probe\n");
#endif

  printk(KERN_INFO "SHARP touchscreen controller I2C driver\n");

  return 0;

 err_free_irq:
  free_irq(client->irq, ts);
 err_free_irq_gpio:
  gpio_free(ts->irq_pin);
  shtsc_reset(ts, true);
  if (ts->reset_pin)
    gpio_free(ts->reset_pin);
 err_free_mem:
  input_free_device(input_dev);
  kfree(ts);

  return err;
}

static int  shtsc_i2c_remove(struct i2c_client *client)
{
  struct shtsc_i2c *ts = i2c_get_clientdata(client);

  shtsc_reset(ts, true);

  mutex_init(&(ts->mutex));

  //	sysfs_remove_group(&client->dev.kobj, &mxt_attr_group);
  free_irq(ts->client->irq, ts);
  input_unregister_device(ts->input);
#if defined(CONFIG_FB)
  if (fb_unregister_client(&ts->fb_notif))
    dev_err(&client->dev, "Error occurred while unregistering fb_notifier.\n");
#elif defined(CONFIG_HAS_EARLYSUSPEND)
  //	unregister_early_suspend(&ts->early_suspend);
#endif
  if (gpio_is_valid(ts->pdata->reset_gpio))
    gpio_free(ts->pdata->reset_gpio);

  if (gpio_is_valid(ts->pdata->irq_gpio))
    gpio_free(ts->pdata->irq_gpio);

  if (client->dev.of_node) {
    kfree(ts->pdata);
  }


  //  kfree(ts->object_table);
  kfree(ts);

  //	debugfs_remove_recursive(debug_base);

  return 0;
}

#elif defined (CONFIG_TOUCHSCREEN_SHTSC_SPI)

/* must be called with ts->mutex held */
static void shtsc_disable(struct shtsc_spi *ts)
{
}

/* must be called with ts->mutex held */
static void shtsc_enable(struct shtsc_spi *ts)
{
}

static int shtsc_open(struct input_dev *input)
{
  struct shtsc_spi *ts = input_get_drvdata(input);

#if defined(DEBUG_SHTSC)
  printk(KERN_INFO "[ENTER] shtsc_open\n");
#endif
  mutex_lock(&ts->mutex);

  if (!ts->suspended)
    shtsc_enable(ts);

  ts->opened = true;

  mutex_unlock(&ts->mutex);

#if defined(DEBUG_SHTSC)
  printk(KERN_INFO "[EXIT] shtsc_open\n");
#endif
  return 0;
}

static void shtsc_close(struct input_dev *input)
{
  struct shtsc_spi *ts = input_get_drvdata(input);

  mutex_lock(&ts->mutex);

  if (!ts->suspended)
    shtsc_disable(ts);

  ts->opened = false;

  mutex_unlock(&ts->mutex);
}

static int __devinit shtsc_spi_probe(struct spi_device *spi)
{
  //  const struct shtsc_pdata *pdata = spi->dev.platform_data;
  struct shtsc_i2c_pdata *pdata;

  struct shtsc_spi *ts;
  struct input_dev *input_dev;

  int error;

  g_spi = spi;

#ifdef DEBUG_SHTSC
  printk(KERN_INFO "[ENTER] shtsc_probe\n");
#endif /* DEBUG_SHTSC */

  //2014.10.16 added
  if (client->dev.of_node) {
    pdata = devm_kzalloc(&client->dev,
			 sizeof(struct shtsc_i2c_pdata), GFP_KERNEL);
    if (!pdata) {
      dev_err(&client->dev, "Failed to allocate memory\n");
      return -ENOMEM;
    }
    err = shtsc_parse_dt(&client->dev, pdata);
    if (err){
      kfree(pdata);
      return err;
    }
  } else{
    pdata = client->dev.platform_data;
  }

  /* No pdata no way forward */
  if (!pdata) {
    dev_dbg(&spi->dev, "no platform data\n");
    return -ENODEV;
  }

  if (spi->irq <= 0) {
    dev_dbg(&spi->dev, "no irq\n");
    return -ENODEV;
  }
	
#ifdef DEBUG_SHTSC
  printk(KERN_INFO "[2] shtsc_probe %d\n", spi->irq);
#endif /* DEBUG_SHTSC */
	
  spi->mode = SPI_MODE_3;
  spi->bits_per_word = 8;
  if (!spi->max_speed_hz)
    spi->max_speed_hz = SHTSC_MAX_SPI_SPEED_IN_HZ;

  error = spi_setup(spi);
  if (error)
    return error;

#ifdef DEBUG_SHTSC
  printk(KERN_INFO "[3] shtsc_probe\n");
#endif /* DEBUG_SHTSC */

  ts = kzalloc(sizeof(*ts), GFP_KERNEL);
  g_ts = ts;
  input_dev = input_allocate_device();
	
  if (!ts || !input_dev) {
    error = -ENOMEM;
    goto err_free_mem;
  }
	
#ifdef DEBUG_SHTSC
  printk(KERN_INFO "[4] shtsc_probe\n");
#endif /* DEBUG_SHTSC */
	
  ts->spi = spi;
  ts->input = input_dev;

  ts->pdata = pdata;//2014.10.16 added

  ts->min_x = pdata->panel_minx;
  ts->max_x = pdata->panel_maxx;
  ts->min_y = pdata->panel_miny;
  ts->max_y = pdata->panel_maxy;
  ts->pressure_max = pdata->ts_pressure_max;
  ts->touch_num_max = pdata->ts_touch_num_max;//2014.10.16 added
  ts->flip_x = pdata->ts_flip_x;
  ts->flip_y = pdata->ts_flip_y;
  ts->swap_xy = pdata->ts_swap_xy;

  ts->reset_pin = pdata->reset_gpio;
  ts->irq_pin = pdata->irq_gpio;
  /*
    ts->irq = spi->irq;

    ts->reset_pin = OMAP4PANDA_GPIO_SHTSC_RESET;
  */
  mutex_init(&ts->mutex);
  spin_lock_init(&ts->lock);
  //	setup_timer(&ts->timer, shtsc_timer, (unsigned long)ts); 20141014

  snprintf(ts->phys, sizeof(ts->phys),
	   "%s/input-ts", dev_name(&spi->dev));

  input_dev->name = SHTSC_DRIVER_NAME;
  input_dev->phys = ts->phys;
  input_dev->id.bustype = BUS_SPI;
  input_dev->dev.parent = &spi->dev;
  input_dev->open = shtsc_input_open;//2014.10.16 added
  input_dev->close = shtsc_input_close;//2014.10.16 added

  __set_bit(EV_ABS, input_dev->evbit);
  __set_bit(EV_KEY, input_dev->evbit);
  __set_bit(BTN_TOUCH, input_dev->keybit);//2014.10.16 added
  __set_bit(INPUT_PROP_DIRECT, input_dev->propbit);//2014.10.16 added

  error = input_mt_init_slots(input_dev, SHTSC_MAX_FINGERS);

  if (error)
    goto err_free_mem;
	
  input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, SHTSC_I2C_SIZE_MAX, 0, 0);
  input_set_abs_params(input_dev, ABS_MT_POSITION_X,
		       pdata->disp_minx, pdata->disp_maxx, 0, 0);
  input_set_abs_params(input_dev, ABS_MT_POSITION_Y,
		       pdata->disp_miny, pdata->disp_maxy, 0, 0);
  input_set_abs_params(input_dev, ABS_MT_PRESSURE  , 0, SHTSC_I2C_P_MAX, 0, 0);
  input_set_abs_params(input_dev, ABS_MT_TOOL_TYPE , 0, MT_TOOL_MAX, 0, 0);

  if (ts->reset_pin) {
    error = gpio_request(ts->reset_pin, NULL);
    if (error) {
      dev_err(&spi->dev,
	      "Unable to request GPIO pin %d.\n",
	      ts->reset_pin);
      goto err_free_mem;
    }
  }

  if (ts->spiss_pin) {
    error = gpio_request(ts->spiss_pin, NULL);
    if (error) {
      dev_err(&spi->dev,
	      "Unable to request GPIO pin %d.\n",
	      ts->spiss_pin);
      goto err_free_mem;
    }
  }
	
  input_dev->open = shtsc_open;
  input_dev->close = shtsc_close;

  input_set_drvdata(input_dev, ts);

#ifndef DEBUG_IRQ_DISABLE
  error = request_threaded_irq(spi->irq, NULL, shtsc_irq_thread,
			       pdata->irqflags, client->dev.driver->name, ts);
#endif

#if defined(DEBUG_SHTSC)
  printk(KERN_INFO "%s(%d): %d\n", __FILE__, __LINE__, error);
#endif
  if (error) {
    dev_err(&spi->dev, "Failed to request irq, err: %d\n", error);
    goto err_free_mem;
  }

#if defined(DEBUG_SHTSC)
  printk(KERN_INFO "[5] shtsc_probe\n");
#endif

  spi_set_drvdata(spi, ts);
  input_set_drvdata(ts->input, ts);//2014.10.16 added
  error = input_register_device(ts->input);
  if (error) {
    dev_err(&spi->dev,
	    "Failed to register input device, err: %d\n", error);
    goto err_clear_drvdata;
  }

  shtsc_reset(g_ts, false);
  msleep(100);
  shtsc_reset(g_ts, true);
  msleep(10);

  VersionModelCode = detectDevice();

  shtsc_system_init(ts);
  msleep(100);

#if defined(DEBUG_SHTSC)
  printk(KERN_INFO "[EXIT] shtsc_probe\n");
#endif

  printk(KERN_INFO "SHARP touchscreen controller SPI driver\n");

  return 0;

 err_clear_drvdata:
  spi_set_drvdata(spi, NULL);
  free_irq(spi->irq, ts);
 err_free_mem:
  input_free_device(input_dev);
  kfree(ts);
  return error;
}

static int __devexit shtsc_spi_remove(struct spi_device *spi)
{
  struct shtsc_spi *ts = spi_get_drvdata(spi);

  free_irq(ts->spi->irq, ts);
  input_unregister_device(ts->input);
  kfree(ts);

  spi_set_drvdata(spi, NULL);
  return 0;
}
#else // (CONFIG_TOUCHSCREEN_SHTSC_UNKNOWNIF??)
error
#endif // (CONFIG_TOUCHSCREEN_SHTSC_UNKNOWNIF??)

int current_page = 0;

char iobuf[4*1024];
static long dev_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
  long ret = true;

  //  unsigned int index;
  u8 addr, val;
  //  unsigned long r;
  int r;

#if defined(DEBUG_SHTSC)
  printk(KERN_INFO "[ENTER] shtsc_ioctl\n");    
#endif

#if defined(DEBUG_SHTSC)
  printk(KERN_INFO "shtsc dev_ioctl switch - cmd: %x, arg: %lx\n", cmd, arg);
#endif

  switch(cmd) {

  case SHTSC_IOCTL_SET_PAGE:
#if defined(DEBUG_SHTSC)
    printk(KERN_INFO "shtsc dev_ioctl case - SET_PAGE; cmd %x, arg %lx\n", cmd, arg);
#endif
    current_page = arg;
    break;

  case SHTSC_IOCTL_FLASH_ACCESS_START:
#if defined(DEBUG_SHTSC)
    printk(KERN_INFO "shtsc dev_ioctl case - FLASH_ACCESS_START; cmd %x\n", cmd);
#endif
    flash_access_start_shtsc(g_ts);
    break;

  case SHTSC_IOCTL_FLASH_ACCESS_END:
#if defined(DEBUG_SHTSC)
    printk(KERN_INFO "shtsc dev_ioctl case - FLASH_ACCESS_END; cmd %x\n", cmd);
#endif
    flash_access_end_shtsc(g_ts);
    break;

  case SHTSC_IOCTL_ERASE:
#if defined(DEBUG_SHTSC)
    printk(KERN_INFO "shtsc dev_ioctl case - ERASE; cmd %x, current_page %d\n", cmd, current_page);
#endif
#ifdef FLASH_CHECK
    if (flash_erase_page_shtsc(g_ts, current_page))
      return -1;
#else /* FLASH_CHECK */
    flash_erase_page_shtsc(g_ts, current_page);
#endif /* FLASH_CHECK */
    break;

  case SHTSC_IOCTL_WRITE:
#if defined(DEBUG_SHTSC)
    printk(KERN_INFO "shtsc dev_ioctl case - WRITE; cmd %x, current_page %d\n", cmd, current_page);
#endif
    if (copy_from_user(iobuf, (char *)arg, (4*1024))) {
      printk(KERN_INFO "shtsc dev_ioctl ERROR by copy_from_user\n");
      return -1;
    }
#ifdef FLASH_CHECK
    if (flash_write_page_shtsc(g_ts, current_page, iobuf))
      return -1;
#else /* FLASH_CHECK */
    flash_write_page_shtsc(g_ts, current_page, iobuf);
#endif /* FLASH_CHECK */
    break;

  case SHTSC_IOCTL_VERIFY:
#if defined(DEBUG_SHTSC)
    printk(KERN_INFO "shtsc dev_ioctl case - VERIFY; cmd %x, current_page %d\n", cmd, current_page);
#endif
    if (copy_from_user(iobuf, (char *)arg, (4*1024))) {
      printk(KERN_INFO "shtsc dev_ioctl ERROR by copy_from_user\n");
      return -1;
    }
#ifdef FLASH_CHECK
    if (flash_verify_page_shtsc(g_ts, current_page, iobuf))
      return -1;
#else /* FLASH_CHECK */
    flash_verify_page_shtsc(g_ts, current_page, iobuf);
#endif /* FLASH_CHECK */
    break;

  case SHTSC_IOCTL_ERASE_ALL:
    {
#define LAST_PAGE 16 
      int page;
      printk(KERN_INFO "%s(%d): flash_access start\n", __FILE__, __LINE__);
      flash_access_start_shtsc(g_ts);
      for (page = 0; page < LAST_PAGE; page++) {
	flash_erase_page_shtsc(g_ts, page);
	printk(KERN_INFO "flash_erase_page_shtsc done: page %d\n",  page);
      }
      printk(KERN_INFO "%s(%d): flash_access end\n", __FILE__, __LINE__);
      flash_access_end_shtsc(g_ts);
      printk(KERN_INFO "%s(%d): flash erased.\n", __FILE__, __LINE__);
    }
    break;
  case SHTSC_IOCTL_REG_1WRITE:
    //    printk(KERN_INFO "SHTSC_IOCTL_REG_1WRITE\n");
    addr = (arg >> 8) & 0xFF;
    val = arg & 0xFF;
    //    printk(KERN_INFO "SHTSC_IOCTL_REG_1WRITE: cmd %x, arg %lx a:0x%x d:0x%x(%d)\n", cmd, arg, addr, val, val);
#if defined(DEBUG_SHTSC)
    printk(KERN_INFO "SHTSC_IOCTL_REG_1WRITE: addr: %02X (Hex), data: %02X (Hex)\n", addr, val);
#endif
    WriteOneByte(g_ts, addr, val);
    //    printk(KERN_INFO "SHTSC_IOCTL_REG_1WRITE done\n");
    break;

  case SHTSC_IOCTL_REG_1READ:
    //    printk(KERN_INFO "SHTSC_IOCTL_REG_1READ\n");
    //addr = 0xFF & arg;
    val = ReadOneByte(g_ts, ((struct reg *)arg)->addr);
    ((struct reg *)arg)->data = val;

#if defined(DEBUG_SHTSC)
    printk(KERN_INFO "SHTSC_IOCTL_REG_1READ: cmd %x, arg %lx a:0x%x d:0x%x(%d)\n", cmd, arg, ((struct reg *)arg)->addr, val, val);
#endif
    //    printk(KERN_INFO "SHTSC_IOCTL_REG_1READ done\n");
    break;

  case SHTSC_IOCTL_REG_N_RW_SET_ADDR:
    s_shtsc_addr = 0xFF & (arg >> 16);
    s_shtsc_len = 0xFFFF & arg;
#if defined(DEBUG_SHTSC)
    printk(KERN_INFO "SHTSC_IOCTL_REG_N_RW_SET_ADDR: cmd %x, arg %lx a:0x%x len:%d\n", cmd, arg, s_shtsc_addr, s_shtsc_len);
#endif
    break;

  case SHTSC_IOCTL_REG_N_WRITE_1ADDR_GO:
    /* s_shtsc_len is initialized __IF__ SHTSC_IOCTL_REG_N_RW_SET_ADDR is issued before */
    r = copy_from_user(s_shtsc_buf, (char *)arg, s_shtsc_len);
#if defined(DEBUG_SHTSC)
    printk(KERN_INFO "Driver Multibyte write. addr %02X, len: %x, data[0-3]: %02X %02X %02X %02X ....\n", 
	   s_shtsc_addr, s_shtsc_len, s_shtsc_buf[0], s_shtsc_buf[1], s_shtsc_buf[2], s_shtsc_buf[3]);
#endif
    if (r != 0) {
      printk(KERN_INFO "shtsc dev_ioctl ERROR by copy_from_user(%d)\n", r);
      return -1;
    }
    //    printk(KERN_INFO "SHTSC_IOCTL_REG_N_WRITE_1ADDR: cmd %x, arg %lx a:0x%x d:0x%x(%d)\n", cmd, arg, addr, val, val);
#if defined(DEBUG_SHTSC)
    printk(KERN_INFO "SHTSC_IOCTL_REG_N_WRITE_1ADDR: cmd %x, arg %lx a:0x%x d:0x%x(%d)\n", cmd, arg, s_shtsc_addr, s_shtsc_len, s_shtsc_len);
#endif
    WriteMultiBytes(g_ts, s_shtsc_addr, s_shtsc_buf, s_shtsc_len);

    break;

  case SHTSC_IOCTL_REG_N_READ_1ADDR_GO:
    /* s_shtsc_len is initialized __IF__ SHTSC_IOCTL_REG_N_RW_SET_ADDR is issued before */
    ReadMultiBytes(g_ts, s_shtsc_addr, s_shtsc_len, s_shtsc_buf);
    //msleep(10); // not checked yet
#if defined(DEBUG_SHTSC)
    printk(KERN_INFO "Driver Multibyte read done. addr %02X, len: %x, data[0-3]: %02X %02X %02X %02X ....\n", 
	   s_shtsc_addr, s_shtsc_len, s_shtsc_buf[0], s_shtsc_buf[1], s_shtsc_buf[2], s_shtsc_buf[3]);
#endif
    r = copy_to_user((char *)arg, s_shtsc_buf, s_shtsc_len);
    if (r != 0) {
      printk(KERN_INFO "shtsc dev_ioctl ERROR by copy_to_user(%d)\n", r);
      return -1;
    }

    break;

  case SHTSC_IOCTL_SETIRQMASK:
#if defined(CONFIG_TOUCHSCREEN_SHTSC_I2C)
    if (arg) {
      shtsc_enable_irq(true); //modified by misaki on 8/3 //enable_irq(g_ts->client->irq);
    } else {
      shtsc_enable_irq(false); //modified by misaki on 8/3//disable_irq(g_ts->client->irq);
    }
#elif defined (CONFIG_TOUCHSCREEN_SHTSC_SPI)
    if (arg) {
      enable_irq(g_ts->spi->irq);
    } else {
      disable_irq(g_ts->spi->irq);
    }
#else // (CONFIG_TOUCHSCREEN_SHTSC_UNKNOWNIF??)
    error
#endif // (CONFIG_TOUCHSCREEN_SHTSC_UNKNOWNIF??)
#if defined(DEBUG_SHTSC)
    printk(KERN_INFO "shtsc dev_ioctl case - SETIRQMASK; cmd %x, arg %lx\n", cmd, arg);
#endif
    break;

  case SHTSC_IOCTL_RESET:
    if (arg) {
      g_ts->wait_state = WAIT_RESET;
      g_ts->wait_result = false;
      shtsc_reset(g_ts, true);
      // wait
      WaitAsync(g_ts);
      shtsc_system_init(g_ts);
      //      msleep(100);
    } else {
      shtsc_reset(g_ts, false);
    }
#if defined(DEBUG_SHTSC)
    printk(KERN_INFO "shtsc dev_ioctl case - RESET; cmd %x, arg %lx\n", cmd, arg);
#endif
    break;

  case SHTSC_IOCTL_DEBUG:
#if defined(DEBUG_SHTSC)
    printk(KERN_INFO "shtsc dev_ioctl case - DEBUG; cmd %x, arg %lx\n", cmd, arg);
#endif
    break;

  case SHTSC_IOCTL_CMD_ISSUE_RESULT:
    {
      unsigned int magicnumber;	/* 'int' width is 32bit for LP64 and LLP64 mode */
      if ( copy_from_user(&magicnumber, (char *) arg, 4) ){
	magicnumber = 0;
      }
      if( magicnumber != 0xA5A5FF00 ){
	msleep(100);
      }
      r = copy_to_user((char *) arg, CommandResultBuf,MAX_COMMAND_RESULT_LEN);
      if( magicnumber == 0xA5A5FF00 ){
	CommandResultBuf[0] = 0;
      }
    }
    if (r != 0) {
      printk(KERN_INFO "shtsc dev_ioctl ERROR by copy_to_user(%d)\n", r);
      return -1;
    }
    break;

  case SHTSC_IOCTL_DRIVER_VERSION_READ:
    {
      char versionBuf[16];
      versionBuf[0] = VersionYear;
      versionBuf[1] = VersionMonth;
      versionBuf[2] = VersionDay;
      versionBuf[3] = VersionSerialNumber;
      versionBuf[4] = VersionModelCode;

      r = copy_to_user((char *)arg, versionBuf, DRIVER_VERSION_LEN); 
      if (r != 0) {
	printk(KERN_INFO "shtsc dev_ioctl ERROR by copy_to_user(%d)\n", r);
	return -1;
      }
    }
    break;

  case SHTSC_IOCTL_DCMAP:
    {
      int x = dcmap[0];
      int y = dcmap[1];
      int len = (x * y * 2) + 4;

#if defined(DEBUG_SHTSC)
      printk(KERN_INFO "shtsc dev_ioctl case - DCMAP; cmd %x, arg %lx, %d*%d+4=%d\n", cmd, arg, x, y, len);
#endif

      if (buf_pos) {
	/* DC map ready to send out */
	if ( copy_to_user( (char *)arg, dcmap, len ) ) {
	  printk( KERN_INFO "shtsc : copy_to_user failed\n" );
	  return -EFAULT;
	}
	buf_pos = 0;
      }
      break;
    }

#ifdef GET_REPORT
  case SHTSC_IOCTL_GET_REPORT:
    {
      volatile int count;
      int len;


      while (Mutex_GetReport)
	;
      Mutex_GetReport = true;
      
      count = reportBuf[0];
      len = 4+ count*REP_SIZE;

#if defined(DEBUG_SHTSC)
      printk(KERN_INFO "shtsc dev_ioctl case - GET_REPORT; cmd %x, arg %lx, count=%d, len=%d\n", cmd, arg, count, len);
#endif

      r = copy_to_user((char *)arg, reportBuf, len);
      if (r != 0) {
	printk(KERN_INFO "shtsc dev_ioctl ERROR by copy_to_user(%d)\n", r);
	return -1;
      }

      reportBuf[0] = (unsigned char)0;

      Mutex_GetReport = false;
    }
    break;
#endif /* GET_REPORT */

  case SHTSC_IOCTL_FLASH_READ:
    {
      /* arg: pointer to the content buffer */
      /* arg[3:0]: address to read (little endian) */
      /* arg[5:4]: length to read (little endian) */

      unsigned address;
      unsigned length;

      if (copy_from_user(iobuf, (char *)arg, (4+2))) {
	printk(KERN_INFO "shtsc dev_ioctl ERROR by copy_from_user\n");
	return -1;
      }
      address = (iobuf[3] << 24) | (iobuf[2] << 16) | (iobuf[1] << 8) | (iobuf[0] << 0);
      length = (iobuf[5] << 8) | (iobuf[4] << 0);

#if defined(DEBUG_SHTSC)
      printk(KERN_INFO "shtsc dev_ioctl case - FLASH_READ; addr %x, arg %x\n", address, length);
#endif

      flash_read(g_ts, address, length, iobuf);
      if ( copy_to_user( (char *)arg, iobuf, length ) ) {
	printk( KERN_INFO "shtsc : copy_to_user failed\n" );
	return -EFAULT;
      }
	break;
    }

  case SHTSC_IOCTL_NOTIFY_PID:
    pid = arg; // save pid for later kill();
    printk(KERN_INFO "SHTSC_IOCTL_NOTIFY_PID: pid: %d\n", pid);
#if defined(DEBUG_SHTSC)
    printk(KERN_INFO "SHTSC_IOCTL_NOTIFY_PID: pid: %d\n", pid);
#endif
    break;

  case SHTSC_IOCTL_GET_INTERRUPT_STATUS:
#if defined(DEBUG_SHTSC)
    printk(KERN_INFO "Received SHTSC_IOCTL_GET_INTERRUPT_STATUS, %d\n", resumeStatus);
#endif

    r = copy_to_user((char *)arg, &resumeStatus, 1); // copy one-byte status
    if (r != 0) {
      printk(KERN_INFO "shtsc dev_ioctl ERROR by copy_to_user(%d)\n", r);
      return -1;
    }
    break;

  default:
    ret = false;
    break;
  }

#if defined(DEBUG_SHTSC)
  printk(KERN_INFO "[EXIT] shtsc_ioctl\n");
#endif

  return ret;
}


#if defined(CONFIG_TOUCHSCREEN_SHTSC_I2C)
static int dev_open(struct inode *inode, struct file *filp)
{
#if defined(DEBUG_SHTSC)
  printk(KERN_INFO "shtsc dev_open\n");
#endif
  return 0;
}

static int dev_release(struct inode *inode, struct file *filp)
{
#if defined(DEBUG_SHTSC)
  printk(KERN_INFO "shtsc dev_release\n");
#endif
  return 0;
}

#elif defined (CONFIG_TOUCHSCREEN_SHTSC_SPI)

static int dev_open(struct inode *inode, struct file *filp)
{
  struct shtsc_spi *ts = spi_get_drvdata(g_spi);

  if(!ts)
    return -EFAULT;

  return 0;
}

static int dev_release(struct inode *inode, struct file *filp)
{
  return 0;
}
#else // (CONFIG_TOUCHSCREEN_SHTSC_UNKNOWNIF??)
error
#endif // (CONFIG_TOUCHSCREEN_SHTSC_UNKNOWNIF??)

static const struct file_operations dev_fops = {
  .owner = THIS_MODULE,
  .open = dev_open,
  .release = dev_release,
  .read = dev_read,
  //	.write = dev_write,
  .unlocked_ioctl = dev_ioctl,
};

static struct miscdevice shtsc_miscdev = {
  .minor = MISC_DYNAMIC_MINOR,
  .name = SHTSC_DRIVER_NAME, // should be "/dev/shtsc"
  .fops = &dev_fops,
};

#if defined(CONFIG_TOUCHSCREEN_SHTSC_I2C)
static struct i2c_device_id shtsc_i2c_idtable[] = {
  { SHTSC_DRIVER_NAME, 0 },
  { }
};
#ifdef CONFIG_OF
static struct of_device_id shtsc_match_table[] = {
  { .compatible = "sharp,shtsc_i2c",},
  { },
};
#else
#define shtsc_match_table NULL
#endif

MODULE_DEVICE_TABLE(i2c, shtsc_i2c_idtable);

static struct i2c_driver shtsc_i2c_driver = {
  .driver		= {
    .owner	= THIS_MODULE,
    .name	= SHTSC_DRIVER_NAME,
    .of_match_table = shtsc_match_table,//2014.10.16 added
  },
  .id_table	= shtsc_i2c_idtable,
  .probe	= shtsc_i2c_probe,
  .remove	= shtsc_i2c_remove,//__devexit_p(shtsc_i2c_remove),
};

static int __init shtsc_init(void)
{
  int ret;
  ret = misc_register(&shtsc_miscdev);
  if (ret) {
    printk(KERN_INFO "%s(%d): misc_register returns %d. Failed.\n", __FILE__, __LINE__, ret);
  }
#if defined(DEBUG_SHTSC)
  printk(KERN_INFO "%s(%d): loaded successfully\n", __FILE__, __LINE__);
#endif

  return i2c_add_driver(&shtsc_i2c_driver);
}
static void __exit shtsc_exit(void)
{
  misc_deregister(&shtsc_miscdev);
  i2c_del_driver(&shtsc_i2c_driver);
}

#elif defined (CONFIG_TOUCHSCREEN_SHTSC_SPI)

static struct spi_driver shtsc_spi_driver = {
  .driver		= {
    .owner	= THIS_MODULE,
    .name	= SHTSC_DRIVER_NAME,
  },
  .probe	= shtsc_spi_probe,
  .remove	= __devexit_p(shtsc_spi_remove),
};

static int __init shtsc_init(void)
{
  int ret;
  ret = misc_register(&shtsc_miscdev);
  if (ret) {
    printk(KERN_INFO "%s(%d): misc_register returns %d. Failed.\n", __FILE__, __LINE__, ret);
  }
#if defined(DEBUG_SHTSC)
  printk(KERN_INFO "%s(%d): loaded successfully\n", __FILE__, __LINE__);
#endif

  return spi_register_driver(&shtsc_spi_driver);
}
static void __exit shtsc_exit(void)
{
  misc_deregister(&shtsc_miscdev);
  spi_unregister_driver(&shtsc_spi_driver);
}
#else // (CONFIG_TOUCHSCREEN_SHTSC_UNKNOWNIF??)
error
#endif // (CONFIG_TOUCHSCREEN_SHTSC_UNKNOWNIF??)

module_init(shtsc_init);
module_exit(shtsc_exit);

MODULE_DESCRIPTION("shtsc SHARP Touchscreen controller Driver");
MODULE_LICENSE("GPL v2");
