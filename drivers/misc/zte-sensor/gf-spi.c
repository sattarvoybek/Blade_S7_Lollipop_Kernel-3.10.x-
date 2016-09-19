/* Simple synchronous userspace interface to SPI devices
 * copyright (C) 2006 SWAPP
 *	Andrea Paterniani <a.paterniani@swapp-eng.it>
 * Copyright (C) 2007 David Brownell (simplification, cleanup)
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
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/input.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/compat.h>
#include <linux/poll.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/io.h>

#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>

#include <asm/uaccess.h>
#include <linux/ktime.h>

#include <linux/interrupt.h>
#include <linux/irq.h>
#include <mach/irqs.h>  //irq no.
#include <linux/completion.h>
#include <linux/gpio.h>
#include <linux/kthread.h>
#include <linux/timer.h>
#include <asm/atomic.h>
#include <linux/unistd.h>
#include "gf-spi.h"
#include <linux/of_gpio.h>
#include <linux/sensor_power.h>
#include <linux/wakelock.h>
/*spi device name*/
#define SPI_DEV_NAME   "spidev"
/*device name after register in charater*/
#define DEV_NAME "goodix_fp"

/*
 * This supports access to SPI devices using normal userspace I/O calls.
 * Note that while traditional UNIX/POSIX I/O semantics are half duplex,
 * and often mask message boundaries, full SPI support requires full duplex
 * transfers.  There are several kinds of internal message boundaries to
 * handle chipselect management and other protocol options.
 *
 * SPI has a character major number assigned.  We allocate minor numbers
 * dynamically using a bitmask.  You must use hotplug tools, such as udev
 * (or mdev with busybox) to create and destroy the /dev/spidevB.C device
 * nodes, since there is no fixed association of minor numbers with any
 * particular SPI bus or device.
 */
#define	CHRD_DRIVER_NAME		"goodix_fp_spi"
#define	CLASS_NAME				"goodix_fp"
#define SPIDEV_MAJOR			154	/* assigned */
#define N_SPI_MINORS			32	/* ... up to 256 */
static DECLARE_BITMAP(minors, N_SPI_MINORS);

#define GF_FASYNC 		1//If support fasync mechanism.
//#undef GF_FASYNC

#define SLEEP_MODE_TEST	1

#define CRC_16_POLYNOMIALS 0x8005

#define MSG_INVALID    0x00
u8 g_msg = MSG_INVALID;

u32 debug_count = 0;

const u8 cmd_hold[3] = {0xEA, 0x00, 0x00};		
const u8 cmd_release[3] = {0xE5, 0x00, 0x00};		
#if SLEEP_MODE_TEST
const u8 cmd_diablepower[3] = {0xF8, 0x00, 0x00};	
const u8 cmd_diableldo18[3] = {0xF5, 0x00, 0x00};	
const u8 cmd_diablepmu[3] = {0xF4, 0x00, 0x00};	
const u8 cmd_diableosc[3] = {0xF7, 0x00, 0x00};	
#endif
//static struct task_struct* fpthread = NULL;  //bernard 20150911
static int read_flag = 0;   
//static DECLARE_WAIT_QUEUE_HEAD(waiter);      //bernard 20150911
struct mutex frame_lock;

/**************************debug******************************/
#define GF_TEST    1
#undef GF_TEST

#define TEST_BUF_LEN 2 
#define TEST_CNT 10000
#define ESD_CHECK_TIME	2 //Sec

#define SPI_ASYNC   1

#define DEFAULT_DEBUG   (0x1<<0)
#define SUSPEND_DEBUG   (0x1<<1)
#define SPI_DEBUG       (0x1<<2)
#define TIME_DEBUG      (0x1<<3)
#define FLOW_DEBUG      (0x1<<4)

#define gf_debug(level, fmt, args...) do{ \
    if(g_debug & level) {\
	pr_info("gf " fmt, ##args); \
    } \
}while(0)

#define FUNC_ENTRY()  gf_debug(FLOW_DEBUG, "gf:%s, entry\n", __func__)
#define FUNC_EXIT()  gf_debug(FLOW_DEBUG,"gf:%s, exit\n", __func__)

/*************************************************************/
struct gf_dev {
    dev_t			devt;
    spinlock_t		spi_lock;
    struct spi_device	*spi;
    struct list_head	device_entry;
    struct input_dev        *input;
    struct workqueue_struct *spi_wq;
    struct work_struct     spi_work;
	struct delayed_work    delay_work;
    /* buffer is NULL unless this device is open (users > 0) */
    struct mutex buf_lock;
    unsigned		users;
    u16 mode;
    u16 esdv1;
    u16 esdv2;
    u8 fw_ver[16];
    u8			*buffer;
    u8			buf_status;
    struct timer_list  	gf_timer;
#ifdef GF_FASYNC
    struct  fasync_struct *async;
#endif
	int irq_gpio;
	int rst_gpio;
};
static LIST_HEAD(device_list);
static DEFINE_MUTEX(device_list_lock);

#define SPI_MHZ (1000 * 1000)
#define SPI_DEFAULT_SPEED (8 * SPI_MHZ)
#define SPI_GET_DATA_SPEED (8 * SPI_MHZ)
#define SPI_MAX_SPEED (8 * SPI_MHZ)

#define GOODIX_HW_VERSION (0x0603)
#define GOODIX_FW_DATA_LEN (512)
#define GOODIX_FW_CONF_LEN (120)


//(68*1.5 + 10) * 118 + 2
#define RAWDATA_LENGTH     (112*118)
#define FRAME_LENGTH      (RAWDATA_LENGTH + 2)
u8 g_frame_buf[FRAME_LENGTH] = {0};

static unsigned bufsiz = 2048 * 8;
atomic_t isSleep = ATOMIC_INIT(0);
atomic_t isPause = ATOMIC_INIT(0);
atomic_t isIrq = ATOMIC_INIT(0);
//atomic_t isSwitchMode = ATOMIC_INIT(0);
static struct task_struct *thread = NULL;
static int fp_event_handler(void *para);
static int fp_flag = 0;
static DECLARE_WAIT_QUEUE_HEAD(int_waiter);
int async_flag;
int mp1_flag = 0;
struct wake_lock wl;
static u16 get_status = 0; 
unsigned long g_debug = DEFAULT_DEBUG;
static struct workqueue_struct *esd_workqueue = NULL;

module_param(bufsiz, uint, S_IRUGO);
MODULE_PARM_DESC(bufsiz, "data bytes in biggest supported SPI message");

#define TIME_START     0
#define TIME_STOP      1

/*Confure the IRQ pin for GF irq if necessary*/
inline static void gf_irq_cfg(struct gf_dev* gf_dev)
{
	/*Config IRQ pin, referring to platform.*/
	gpio_direction_input(gf_dev->irq_gpio);

}
/********************************************************************
*CPU output low level in RST pin to reset GF. This is the MUST action for GF.
*Take care of this function. IO Pin driver strength / glitch and so on.
********************************************************************/
inline static void gf_hw_reset(struct gf_dev* gf_dev)
{
	gpio_direction_output(gf_dev->rst_gpio, 1);
	udelay(100);

	gpio_set_value(gf_dev->rst_gpio, 0);
	udelay(200);
	gpio_set_value(gf_dev->rst_gpio, 1);
	mdelay(3);
}
static long int prev_time, cur_time;

void kernel_time(unsigned int step)
{
    cur_time = ktime_to_us(ktime_get());
    if(step == TIME_START)
    {
	prev_time = cur_time;
    } 
    else if(step == TIME_STOP) 
    {
		pr_info("gf:use: %ld us\n", (cur_time - prev_time));
    }
    //prev_time = cur_time;
}

/* -------------------------------------------------------------------- */
/* devfs                                */
/* -------------------------------------------------------------------- */
static ssize_t gf_debug_show(struct device *dev, 
	struct device_attribute *attr, char *buf)
{
    printk("Show.\n");
    return 0;
}
static ssize_t gf_debug_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
    int debug_level = 0;
    sscanf(buf, "%d", &debug_level);
    printk("Store. debug_level = %d\n", debug_level);
    return strnlen(buf, count);
}

static DEVICE_ATTR(debug, S_IRUGO|S_IWUSR, gf_debug_show, gf_debug_store);

static struct attribute *gf_debug_attrs[] = {
    &dev_attr_debug.attr,
    NULL
};

static const struct attribute_group gf_debug_attr_group = {
    .attrs = gf_debug_attrs,
    .name = "debug"
};

#ifdef SPI_ASYNC
static void gf_spi_complete(void *arg)
{
    complete(arg);
}
#endif //SPI_ASYNC

static int gf_spi_write_bytes(struct gf_dev *gf_dev,
	u16 addr, u32 data_len, u8 *tx_buf)
{
#ifdef SPI_ASYNC
    DECLARE_COMPLETION_ONSTACK(read_done);
#endif
    struct spi_message msg;
    struct spi_transfer *xfer;
    int ret = 0;

    xfer = kzalloc(sizeof(*xfer), GFP_KERNEL);
    if( xfer == NULL){
		pr_warn("No memory for command.\n");
	return -ENOMEM;
    }

    /*send gf command to device.*/
    spi_message_init(&msg);
    tx_buf[0] = GF_W;
    tx_buf[1] = (u8)((addr >> 8)&0xFF);
    tx_buf[2] = (u8)(addr & 0xFF);
    xfer[0].tx_buf = tx_buf;
    xfer[0].len = data_len + 3;
    xfer[0].delay_usecs = 5;
    spi_message_add_tail(xfer, &msg);
#ifdef SPI_ASYNC
    msg.complete = gf_spi_complete;
    msg.context = &read_done;

    spin_lock_irq(&gf_dev->spi_lock);
    ret = spi_async(gf_dev->spi, &msg);
    spin_unlock_irq(&gf_dev->spi_lock);
    if(ret == 0) {
	wait_for_completion(&read_done);
	if(msg.status == 0)
	    ret = msg.actual_length - GF_WDATA_OFFSET;
    }
#else
    ret = spi_sync(gf_dev->spi, &msg);
    if(ret == 0) {
	ret = msg.actual_length - GF_WDATA_OFFSET;
    }
#endif
    gf_debug(SPI_DEBUG, "ret = %d, actual_length = %d write addr 0x%x \n", ret, msg.actual_length, addr);
    kfree(xfer);
    if(xfer != NULL)
	xfer = NULL;

    return ret;
}

/*************************************************************
 *First message:
 *	write cmd   |  ADDR_H |ADDR_L  |
 *    1B         |   1B    |  1B    |
 *Second message:
 *	read cmd   |  data stream  |
 *    1B        |   length    |
 *
 * read buffer length should be 1 + 1 + 1 + 1 + data_length
 **************************************************************/
static int gf_spi_read_bytes(struct gf_dev *gf_dev,
	u16 addr, u32 data_len, u8 *rx_buf)
{
#ifdef SPI_ASYNC
    DECLARE_COMPLETION_ONSTACK(write_done);
#endif //SPI_ASYNC
    struct spi_message msg;
    struct spi_transfer *xfer;
    int ret = 0;

    xfer = kzalloc(sizeof(*xfer)*2, GFP_KERNEL);
    if( xfer == NULL){
	pr_warn("No memory for command.\n");
	return -ENOMEM;
    }

    /*send gf command to device.*/
    spi_message_init(&msg);
    rx_buf[0] = GF_W;
    rx_buf[1] = (u8)((addr >> 8)&0xFF);
    rx_buf[2] = (u8)(addr & 0xFF);
    xfer[0].tx_buf = rx_buf;
    xfer[0].len = 3;
    xfer[0].delay_usecs = 5;
    spi_message_add_tail(&xfer[0], &msg);

    /*if wanted to read data from gf. 
     *Should write Read command to device
     *before read any data from device.
     */
    spi_sync(gf_dev->spi, &msg);
    spi_message_init(&msg);
    memset(rx_buf, 0xff, data_len +4);
    rx_buf[3] = GF_R;
    xfer[1].tx_buf = &rx_buf[3];

    xfer[1].rx_buf = &rx_buf[3];
    xfer[1].len = data_len + 1;
    xfer[1].delay_usecs = 5;

    spi_message_add_tail(&xfer[1], &msg);

#ifdef SPI_ASYNC
    msg.complete = gf_spi_complete;
    msg.context = &write_done;

    spin_lock_irq(&gf_dev->spi_lock);
    ret = spi_async(gf_dev->spi, &msg);
    spin_unlock_irq(&gf_dev->spi_lock);
    if(ret == 0) {
	wait_for_completion(&write_done);
	if(msg.status == 0)
	    ret = msg.actual_length - 1;//GF_RDATA_OFFSET;
    }
#else
    ret = spi_sync(gf_dev->spi, &msg);
    if(ret == 0){
	ret = msg.actual_length - 1;//GF_RDATA_OFFSET;
    }
#endif
    gf_debug(SPI_DEBUG, "ret = %d, actual_length = %d read addr 0x%x \n", ret, msg.actual_length, addr);
    kfree(xfer);
    if(xfer != NULL)
	xfer = NULL;

    return ret;
}

static int gf_spi_read_word(struct gf_dev *gf_dev, u16 addr, u16* value)
{
    int status = 0;
    u8* buf = NULL;
    mutex_lock(&gf_dev->buf_lock);

    status = gf_spi_read_bytes(gf_dev, addr, 2, gf_dev->buffer);
    buf = gf_dev->buffer + GF_RDATA_OFFSET;
    *value = (u16)buf[0]<<8 | buf[1];
    mutex_unlock(&gf_dev->buf_lock);
    return status;
}

static int gf_spi_write_word(struct gf_dev *gf_dev, u16 addr, u16 value)
{
    int status = 0;

    mutex_lock(&gf_dev->buf_lock);
    gf_dev->buffer[GF_WDATA_OFFSET] = 0x00;
    gf_dev->buffer[GF_WDATA_OFFSET+1] = 0x01;
    gf_dev->buffer[GF_WDATA_OFFSET+2] = (u8)(value>>8);
    gf_dev->buffer[GF_WDATA_OFFSET+3] = (u8)(value & 0x00ff);


    status = gf_spi_write_bytes(gf_dev, addr, 4, gf_dev->buffer);
    mutex_unlock(&gf_dev->buf_lock);

    return status;
}

static int gf_spi_send_cmd(struct gf_dev* gf_dev, unsigned char* cmd, int len)
{
    struct spi_message msg;
    struct spi_transfer *xfer;
    int ret;

    spi_message_init(&msg);
    xfer = kzalloc(sizeof(*xfer), GFP_KERNEL);
    xfer->tx_buf = cmd;
    xfer->len = len;
    xfer->delay_usecs = 20;

    spi_message_add_tail(xfer, &msg);
    ret = spi_sync(gf_dev->spi, &msg);

    kfree(xfer);
    if(xfer != NULL)
	xfer = NULL;
	return ret;

}
static void endian_exchange(int len, u8* buf)
{
    int i;
    u8 buf_tmp;
    for(i=0; i< len/2; i++)
    {   
	buf_tmp = buf[2*i+1];
	buf[2*i+1] = buf[2*i] ;
	buf[2*i] = buf_tmp;
    }
}

static int gf_spi_read_data(struct gf_dev* gf_dev, u16 addr, int len, u8* value)
{
    int status;

    mutex_lock(&gf_dev->buf_lock);
    status = gf_spi_read_bytes(gf_dev, addr, len, gf_dev->buffer);
    memcpy(value, gf_dev->buffer+GF_RDATA_OFFSET, len);
    mutex_unlock(&gf_dev->buf_lock);	

    endian_exchange(len,value);

    return status;
}


static int gf_spi_read_data_big_end(struct gf_dev* gf_dev, u16 addr, int len, u8* value)
{
    int status;
    mutex_lock(&gf_dev->buf_lock);
    status = gf_spi_read_bytes(gf_dev, addr, len, gf_dev->buffer);
    memcpy(value, gf_dev->buffer+GF_RDATA_OFFSET, len);
    mutex_unlock(&gf_dev->buf_lock);	

    //endian_exchange(len,value);

    return status;
}

static int gf_spi_write_data(struct gf_dev* gf_dev, u16 addr, int len, u8* value)
{
    int status =0;
    unsigned short addr_len = 0;
    unsigned char* buf = NULL;

    if (len > 1024 * 10){
		return -1;
    }

    addr_len = len / 2;  

    buf = kzalloc(len + 2, GFP_KERNEL);
    if (buf == NULL){
	return -1;
    }

    buf[0] = (unsigned char) ((addr_len & 0xFF00) >> 8);
    buf[1] = (unsigned char) (addr_len & 0x00FF);
    memcpy(buf+2, value, len);
    endian_exchange(len,buf+2);

    mutex_lock(&gf_dev->buf_lock);
    memcpy(gf_dev->buffer+GF_WDATA_OFFSET, buf, len+2);
    kfree(buf);

    status = gf_spi_write_bytes(gf_dev, addr, len+2, gf_dev->buffer);
    mutex_unlock(&gf_dev->buf_lock);

    return status;
}


void GF11_Register_En(struct gf_dev* gf_dev, unsigned short addr, unsigned char bit_num, unsigned char en_sel)
{
    unsigned char temp[2];
    unsigned short Read_Temp;	

    gf_spi_read_data(gf_dev, addr, 2, temp);

    Read_Temp = (unsigned short)temp[0] | ((unsigned short)temp[1] << 8 );

    if(en_sel == 1)                              //bis
	Read_Temp = Read_Temp | ( (unsigned short)1 << bit_num );
    else if(en_sel == 0 )                                    //bic 
	Read_Temp = Read_Temp &  (~((unsigned short)1 << bit_num ));

    temp[0] = (unsigned char)Read_Temp;
    temp[1] = (unsigned char)(Read_Temp >> 8) ;	

    gf_spi_write_data(gf_dev, addr,2,temp);

}



#if 1
unsigned char gf_fw_data[526] = {
0x10,0x01,0x47,0x46,0x58,0x31,0x58,0x00,0x00,0x00,0x01,0x13,0xB2,0x40,0x38,0x02,0xBC,0x02,0xB2,0x40,
0xD8,0xF6,0x28,0x02,0xB2,0x40,0x14,0xF7,0x10,0x02,0xB2,0x40,0x08,0xF7,0x16,0x02,0xB2,0x40,0x40,0xF7,
0x2C,0x02,0xB0,0x12,0xD0,0xFB,0xB0,0x12,0x48,0xFC,0xB2,0x40,0x13,0x01,0x06,0x02,0xB0,0x12,0x8A,0xFB,
0x7F,0x40,0x20,0x00,0xB0,0x12,0x14,0xFC,0xB2,0xD0,0x00,0x11,0x30,0x00,0x32,0xD2,0xD2,0x42,0xC2,0x02,
0xBB,0x02,0xB0,0x12,0xAA,0xF9,0xF2,0x90,0x57,0x00,0x31,0x02,0x40,0x24,0xC2,0x93,0xB5,0x02,0x17,0x20,
0xF2,0x90,0x03,0x00,0xC2,0x02,0x07,0x24,0xB2,0xD0,0x0A,0x00,0x64,0x00,0x92,0x53,0x00,0x03,0xA2,0xC3,
0x64,0x00,0xC2,0x93,0x34,0x02,0x2C,0x38,0xF2,0x90,0xB0,0xFF,0xB1,0x02,0x25,0x24,0xB2,0x40,0x1E,0xF7,
0x24,0x02,0xB0,0x12,0xBC,0xFF,0xB0,0x12,0x06,0xFB,0xB0,0x12,0x26,0xFA,0xB0,0x12,0xEC,0xF9,0xB0,0x12,
0xD8,0xFA,0xF2,0x90,0x57,0x00,0x31,0x02,0x10,0x24,0xC2,0x93,0x34,0x02,0x08,0x38,0xD2,0x93,0x32,0x02,
0x05,0x24,0xB0,0x12,0x50,0xFB,0xB0,0x12,0x36,0xFC,0xC6,0x3F,0x3F,0x40,0x64,0x00,0xB0,0x12,0x60,0xF9,
0xF6,0x3F,0xB2,0xC0,0x00,0x01,0x3C,0x00,0xEC,0x3F,0xD2,0x93,0x31,0x02,0xD8,0x27,0xB0,0x12,0xCC,0xFF,
0xDA,0x3F,0xD2,0x43,0xB0,0x02,0xBD,0x3F,0x5F,0x42,0x45,0x02,0xB2,0x40,0xA2,0xFD,0x24,0x02,0xB0,0x12,
0xEC,0xFE,0xB2,0xF0,0xFF,0xFE,0xB4,0x00,0xB2,0x40,0x04,0x05,0x22,0x00,0x82,0x43,0x3C,0x00,0x82,0x43,
0x40,0x00,0x82,0x43,0x44,0x00,0xB2,0x40,0x40,0x03,0xC6,0x00,0x7F,0xF3,0x30,0x41,0xB0,0x12,0xD2,0xFC,
0xB2,0x40,0x51,0x00,0x54,0x00,0x30,0x41,0xB0,0x12,0x66,0xFC,0xF2,0xD2,0x0D,0x00,0x30,0x41,0xB0,0x12,
0xA2,0xFD,0x92,0x43,0x2C,0x00,0xB2,0x40,0x05,0x00,0x2C,0x00,0x1F,0x43,0xB0,0x12,0x54,0xF9,0xB2,0x40,
0x06,0x00,0x2C,0x00,0xB1,0x40,0x32,0xFE,0x00,0x00,0x30,0x41,0x0B,0x12,0x0A,0x12,0x0A,0x4F,0x4B,0x4E,
0x6E,0x9F,0x04,0x24,0xB0,0x12,0x36,0xFC,0x6B,0x9A,0xFC,0x23,0x3A,0x41,0x3B,0x41,0x30,0x41,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0xE4,0xAF
};

unsigned char gf_cfg_data[120] = {
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x20,0x00,0x02,0x0F,0x0E,0x01,0x00,0x02,
0x01,0x35,0x40,0x60,0x44,0x55,0x66,0x4E,0xE7,0xEA,0x50,0xC0,0x70,0x38,0x01,0x01,
0x40,0x10,0x00,0x42,0x09,0x00,0x3C,0x30,0x11,0x44,0x31,0x00,0x54,0x51,0x3B,0x58,
0xC0,0x20,0x5A,0x02,0x00,0xC6,0x0B,0x04,0x5E,0x40,0x00,0x56,0x00,0x00,0x52,0x41,
0x14,0xC6,0x40,0x04,0x38,0x20,0x20,0x40,0x10,0x00,0x24,0xC8,0x10,0x3C,0x00,0x11,
0x40,0x10,0x00,0x42,0x09,0x00,0x44,0x31,0x00,0x58,0x78,0x20,0x5A,0x02,0x00,0x64,
0x48,0x00,0x6A,0x02,0x00,0xC6,0x68,0x04,0x5E,0x50,0x00,0x56,0x10,0x00,0x52,0x41,
0x24,0x5C,0x50,0x00,0x75,0xE7,0x01,0x00
};

unsigned char gf_cfg_data_ff[120] = {
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x20,0x00,0x02,0x0F,0x0E,0x01,0x00,0x02,
0x01,0x35,0x40,0x60,0x44,0x55,0x66,0x4E,0xE5,0xFA,0x50,0xC0,0x50,0x38,0x01,0x01,
0x40,0x10,0x00,0x42,0x09,0x00,0x3C,0x30,0x11,0x44,0x31,0x00,0x54,0x51,0x3B,0x58,
0xC0,0x20,0x5A,0x02,0x00,0xC6,0x0B,0x04,0x5E,0x40,0x00,0x56,0x00,0x00,0x52,0x41,
0x14,0xC6,0x40,0x04,0x38,0x20,0x20,0x40,0x10,0x00,0x24,0xC8,0x10,0x3C,0x00,0x11,
0x40,0x10,0x00,0x42,0x09,0x00,0x44,0x31,0x00,0x58,0x78,0x20,0x5A,0x02,0x00,0x64,
0x48,0x00,0x6A,0x02,0x00,0xC6,0x68,0x04,0x5E,0x50,0x00,0x56,0x10,0x00,0x52,0x41,
0x24,0x5C,0x50,0x00,0x97,0xD7,0x01,0x00
};
unsigned char gf_cfg_data_eco[120] = {
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x20,0x00,0x02,0x0F,0x0E,0x01,0x00,0x02,
0x01,0x35,0x40,0x60,0x44,0x55,0x66,0x4E,0xE5,0xEA,0x50,0xC0,0x60,0x38,0x01,0x01,
0x40,0x10,0x00,0x42,0x09,0x00,0x3C,0x30,0x11,0x44,0x31,0x00,0x54,0x51,0x3B,0x58,
0xC0,0x20,0x5A,0x02,0x00,0xC6,0x0B,0x04,0x5E,0x40,0x00,0x56,0x00,0x00,0x52,0x41,
0x14,0xC6,0x40,0x04,0x38,0x20,0x20,0x40,0x10,0x00,0x24,0xC8,0x10,0x3C,0x00,0x11,
0x40,0x10,0x00,0x42,0x09,0x00,0x44,0x31,0x00,0x58,0x78,0x20,0x5A,0x02,0x00,0x64,
0x48,0x00,0x6A,0x02,0x00,0xC6,0x68,0x04,0x5E,0x50,0x00,0x56,0x10,0x00,0x52,0x41,
0x24,0x5C,0x50,0x00,0x87,0xE7,0x01,0x00
};

unsigned char gf_cfg_data_ff_eco[120] = {
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x20,0x00,0x02,0x0F,0x0E,0x01,0x00,0x02,
0x01,0x35,0x40,0x60,0x44,0x55,0x66,0x4E,0xE3,0xFA,0x50,0xC0,0x40,0x38,0x01,0x01,
0x40,0x10,0x00,0x42,0x09,0x00,0x3C,0x30,0x11,0x44,0x31,0x00,0x54,0x51,0x3B,0x58,
0xC0,0x20,0x5A,0x02,0x00,0xC6,0x0B,0x04,0x5E,0x40,0x00,0x56,0x00,0x00,0x52,0x41,
0x14,0xC6,0x40,0x04,0x38,0x20,0x20,0x40,0x10,0x00,0x24,0xC8,0x10,0x3C,0x00,0x11,
0x40,0x10,0x00,0x42,0x09,0x00,0x44,0x31,0x00,0x58,0x78,0x20,0x5A,0x02,0x00,0x64,
0x48,0x00,0x6A,0x02,0x00,0xC6,0x68,0x04,0x5E,0x50,0x00,0x56,0x10,0x00,0x52,0x41,
0x24,0x5C,0x50,0x00,0xA9,0xD7,0x01,0x00
};

#endif

#if SLEEP_MODE_TEST
void enter_sleep_mode(struct gf_dev* gf_dev)
{
	u16 mode = 0;
	pr_info("----gf enter_sleep_mode--start----\n");

	// 1.save mode
    gf_spi_read_word(gf_dev, 0x0B18, &mode);
    mode = (mode & 0x00FF) | (GF_SLEEP_MODE << 8);
    gf_dev->mode = mode; //save mode 
    gf_spi_write_word(gf_dev, 0x0B18, mode);  
    //gf_spi_read_word(gf_dev, 0x0B18, &mode);
	// 2.disable timer
//	del_timer_sync(&gf_dev->gf_timer); // disable ESD Check

	
	#if 1 // need enable for sleep mode current
	gf_spi_write_word(gf_dev, 0x0042, 0x0008); //disable tx,pixel cancel,evenodd
	gf_spi_write_word(gf_dev, 0x00C6, 0x0340); //disable dac
	gf_spi_write_word(gf_dev, 0x0044, 0x0000); //disable scint
	gf_spi_write_word(gf_dev, 0x0052, 0x1440); //disable ranpstart
	
	gf_spi_write_word(gf_dev, 0x0040, 0x0000); //disable bias
	gf_spi_write_word(gf_dev, 0x004E, 0x6089); //disable wdt enable£¬Enable vdd18l vdd24l
	gf_spi_write_word(gf_dev, 0x0050, 0x2020); //disable wdt 32k
	gf_spi_write_word(gf_dev, 0x0022, 0x4584); //reset wakeup
	#endif
	udelay(20);
	mutex_lock(&gf_dev->buf_lock);
	gf_spi_send_cmd(gf_dev, (u8*)cmd_diablepower, 1);
	gf_spi_send_cmd(gf_dev, (u8*)cmd_diableldo18, 1);
	gf_spi_send_cmd(gf_dev, (u8*)cmd_diablepmu, 1);
	gf_spi_send_cmd(gf_dev, (u8*)cmd_diableosc, 1);
	mutex_unlock(&gf_dev->buf_lock);

	pr_info("----gf enter_sleep_mode--end----\n");
}
#endif

// 0x9c is MCU Clock's Address
// we can change its frequency via low 4 bits
// they must be used in pair
static u16 _9c_r = 0x0000;
static u16 _9c_w = 0x0000;
void modify_mcu_clk(struct gf_dev* gf_dev)
{
    gf_spi_read_word(gf_dev, 0x009c, &_9c_r);
    _9c_w = _9c_r & 0xFFF0;
    gf_spi_write_word(gf_dev, 0x009c, _9c_w);
}

void recovery_mcu_clk(struct gf_dev* gf_dev)
{
    gf_spi_write_word(gf_dev, 0x009c, _9c_r);
}

static void mcu_disable(struct gf_dev* gf_dev)
{
    mutex_lock(&gf_dev->buf_lock);
    gf_spi_send_cmd(gf_dev, (u8*)cmd_hold, 1);
    mutex_unlock(&gf_dev->buf_lock);	
}

static void mcu_enable(struct gf_dev* gf_dev)
{
    mutex_lock(&gf_dev->buf_lock);
    gf_spi_send_cmd(gf_dev, (u8*)cmd_release, 1);

    mutex_unlock(&gf_dev->buf_lock);
}

static unsigned short gf_get_chip_version(struct gf_dev *gf_dev)
{
    unsigned short version;	

    mcu_disable(gf_dev);
    gf_spi_read_word(gf_dev, 0x0020, &version);
    mcu_enable(gf_dev);

    return version;
}

static void gf_read_fw_version(struct gf_dev* gf_dev, u8* buf)
{

    int i = 0;
    for (i=0; i<512; i++){
        if((buf[i] == 0xB2) &&  (buf[i+1] == 0x40) && (buf[i+4] == 0x06) && (buf[i+5] == 0x02)){
	    gf_dev->fw_ver[0] = buf[i+2];
            gf_dev->fw_ver[1] = buf[i+3];
	    gf_debug(DEFAULT_DEBUG,"FW_VER:0x%x, 0x%x\n" , gf_dev->fw_ver[0], gf_dev->fw_ver[1]);
            return;
	}   
    }

    gf_dev->fw_ver[0] = 0;
    gf_dev->fw_ver[1] = 0;
    gf_debug(DEFAULT_DEBUG,"FW_VER:0x%x, 0x%x\n" , gf_dev->fw_ver[0], gf_dev->fw_ver[1]);
        
}



u8 buf_01[1024] = {0};
static char gf_memset(struct gf_dev* gf_dev, u16 addr, u16 len, u8* buf)
{
    char ret = 0;
    char loop = 3;
    u16 i = 0;
	
    if ((len > sizeof(buf_01)) || (buf == NULL)){
        dev_err(&gf_dev->spi->dev, "%s:addr=0x%x, len=%d, buf=%d\n", __func__, addr, len, (int)buf);
        return -1;
    }
    
    do {
		for (i=0;i<len;i+=2)
			{
        gf_spi_write_data(gf_dev, addr+i/2, 2, buf+i);
			}
        udelay(20);
        gf_spi_read_data(gf_dev, addr, len, buf_01);

        ret = memcmp(buf, buf_01, len);
        if(ret != 0){
    		dev_err(&gf_dev->spi->dev, "gf:mem is differ with temp_buf addr=0x%X ret=%d, loop=%d\n", addr, ret, loop);
    		loop--;
        }
        else{
            return 0;
        }

    }while(loop > 0);
    
	return -1;

    
}


//////////////////////////////////////////////////////////////////////
////// re-calibration via efuse with E & H version chip
//////////////////////////////////////////////////////////////////////
static void re_calibration_via_efuse(struct gf_dev* gf_dev)
{
    u16 tmpBuf;
    u16 efuse_reg;
    u16 cali_reg;
    u16 chip_id_reg;

    // 1. must hold mcu first
    // if u already hold mcu, needn't do this
    //mcu_disable(gf_dev);
    // 2. read chip_id
    gf_spi_read_word(gf_dev, CHIP_ID_ADDR, &chip_id_reg);
    // 3. read efuse via different chip_id : E or H
    switch(chip_id_reg)
    {
	case CHIP_ID_E :
	case CHIP_ID_E_ECO :
	    tmpBuf = 0x8000;
	    // enable to read efuse register
	    gf_spi_write_word(gf_dev, EFUSE_E_ADDR, tmpBuf);
	    // delay 200us is
	    udelay(400);
	    // read efuse register
	    gf_spi_read_word(gf_dev, EFUSE_E_ADDR, &efuse_reg);
	    break;
	case CHIP_ID_H :
	case CHIP_ID_H_ECO :
	    tmpBuf = 0x0010;
	    // enable to read efuse register
	    gf_spi_write_word(gf_dev, EFUSE_H_CTL_ADDR, tmpBuf);
	    // delay 200us is ok,so we double it for safety
	    udelay(400);
	    // read efuse register
	    do
	    {
		gf_spi_read_word(gf_dev, EFUSE_H_ADDR, &efuse_reg);
		gf_spi_read_word(gf_dev, EFUSE_H_STA_ADDR, &tmpBuf);
	    }while((tmpBuf & 0x0001) == 0);
	    break;
	default:
	    //maybe wrong chip or no chip, print some message for debug
	    break;
    }

    if(efuse_reg == 0x0000)
    {
	// maybe not be calibrated when out of factory, print some message for debug
	// we encounter this BUG when debug GF66XX at MEIZU
    }
#define CALI_ADDR 0x68
    // 4. calculate CALI value via effuse_reg's low two 6bit
    cali_reg = (u16)((efuse_reg & 0x003F) | ((efuse_reg & 0x0FC0) >> 6));
    // 5. write effuse_reg to calibration reg

    //pr_info("%s cali reg 0x%x \n",__func__, cali_reg);
    gf_spi_write_word(gf_dev, CALI_ADDR, cali_reg);
    // 6. for matching hold mcu
    //mcu_enable(gf_dev);
}


static int gf_hw_initial(struct gf_dev *gf_dev);

#if 0
static void gf_timer_work(struct work_struct *work)
{
    struct gf_dev *gf_dev;

    if(work == NULL) {
	pr_info("[info] %s wrong work\n",__func__);
	return;
    }
    gf_dev = container_of(work, struct gf_dev, spi_work);


    //pr_info("mode 0x%x read flag 0x%x\n",gf_dev->mode, read_flag);
    if((((gf_dev->mode & 0xFF00)>> 8) == GF_FF_MODE) 
		#if SLEEP_MODE_TEST
		||(((gf_dev->mode & 0xFF00)>> 8) == GF_SLEEP_MODE)
		#endif
		|| read_flag) {
		pr_info("gf: Invaild timer work gf_dev->mode 0x%x read flag 0x%x\n",gf_dev->mode, read_flag);
	return;
    }

    mutex_lock(&frame_lock);
    gf_spi_read_word(gf_dev, 0x0200, &gf_dev->esdv1);

    pr_info("gf: ESD Protection mode 0x%x CHECK 0x200=%04x %04x\n", gf_dev->mode,  gf_dev->esdv1, gf_dev->esdv2);
    if ((gf_dev->mode == 0x03c6) || (gf_dev->mode == 0x02c6)){
		mutex_unlock(&frame_lock);
		return;
    }

    if (gf_dev->esdv1 != gf_dev->esdv2) {
	    gf_dev->esdv2 = gf_dev->esdv1;
    } else {
	    //reset
	    pr_info("gf: something wrong happened, we check again \n");
	    gf_spi_read_word(gf_dev, 0x0200, &gf_dev->esdv1);
	    if (gf_dev->esdv1 != gf_dev->esdv2) {
	        gf_dev->esdv2 = gf_dev->esdv1;
	        goto exit;
	    }

	    pr_warn("gf: warning : fw has't repose ,do reset \n");
	    disable_irq_wake(gf_dev->spi->irq);
	    gf_hw_initial(gf_dev);
	    //gf_dev->gf_timer.expires = jiffies + ESD_CHECK_TIME * HZ;
	    mod_timer(&gf_dev->gf_timer, jiffies + ESD_CHECK_TIME * HZ);
	    enable_irq_wake(gf_dev->spi->irq);


    }

exit:
    mod_timer(&gf_dev->gf_timer, jiffies + ESD_CHECK_TIME*HZ);//??whether 2s is ok
    mutex_unlock(&frame_lock);

}

static void gf_timer_func(unsigned long arg)
{
    struct gf_dev* gf_dev = (struct gf_dev*)arg;
    schedule_work(&gf_dev->spi_work);
}
#else 
static void gf_delay_work(struct work_struct *work)
{
    struct gf_dev *gf_dev;

    if(work == NULL) {
	pr_info("[info] %s wrong work\n",__func__);
	return;
    }
    gf_dev = container_of((struct delayed_work *)work, struct gf_dev, delay_work);


    //pr_info("mode 0x%x read flag 0x%x\n",gf_dev->mode, read_flag);
    if((((gf_dev->mode & 0xFF00)>> 8) == GF_FF_MODE) 
		#if SLEEP_MODE_TEST
		||(((gf_dev->mode & 0xFF00)>> 8) == GF_SLEEP_MODE)
		#endif
		|| read_flag) {
		pr_info("gf: Invaild timer work gf_dev->mode 0x%x read flag 0x%x\n",gf_dev->mode, read_flag);
		return;
    }

    mutex_lock(&frame_lock);
    gf_spi_read_word(gf_dev, 0x0200, &gf_dev->esdv1);

    pr_info("gf: ESD Protection mode 0x%x CHECK 0x200=%04x %04x\n", gf_dev->mode,  gf_dev->esdv1, gf_dev->esdv2);
    

    if (gf_dev->esdv1 != gf_dev->esdv2) {
	    gf_dev->esdv2 = gf_dev->esdv1;
    } else {
	    //reset
	    pr_info("gf: something wrong happened, we check again \n");
	    gf_spi_read_word(gf_dev, 0x0200, &gf_dev->esdv1);
	    if (gf_dev->esdv1 != gf_dev->esdv2) {
	        gf_dev->esdv2 = gf_dev->esdv1;
	        goto exit;
	    }

	    pr_warn("gf: warning : fw has't repose ,do reset \n");
	    disable_irq_wake(gf_dev->spi->irq);
	    gf_hw_initial(gf_dev);    
	    enable_irq_wake(gf_dev->spi->irq);
    }

exit:
    queue_delayed_work(esd_workqueue, &gf_dev->delay_work,  ESD_CHECK_TIME * HZ);
    mutex_unlock(&frame_lock);

}
#endif

void gf_esd_pet_init(struct gf_dev* gf_dev)
{
#if 0
    INIT_WORK(&gf_dev->spi_work, gf_timer_work);
    init_timer(&gf_dev->gf_timer);
    gf_dev->gf_timer.function = gf_timer_func;
    gf_dev->gf_timer.expires = jiffies + ESD_CHECK_TIME*HZ;
    gf_dev->gf_timer.data = (unsigned long)gf_dev;
    //add_timer(&gf_dev->gf_timer);
#else
    INIT_DELAYED_WORK(&gf_dev->delay_work, gf_delay_work);
	esd_workqueue = create_workqueue("gf_esd");
#endif
}



//u8 temp_buffer[1024]={0};


static int gf_fw_conf_update(struct gf_dev* gf_dev, u8 *fw_buf, u32 fw_len, u8* cfg_buf, u32 cfg_len)
{
    char ret = 0;
    u8 reg_buf[2];
    u16 reg_value = 0;
    u16 version = 0;

	dev_info(&gf_dev->spi->dev, "FW_DATA:0x%x, 0x%x, 0x%x, 0x%x\n" , 
		fw_buf[GOODIX_FW_DATA_LEN - 4], 
		fw_buf[GOODIX_FW_DATA_LEN - 3], 
		fw_buf[GOODIX_FW_DATA_LEN - 2], 
		fw_buf[GOODIX_FW_DATA_LEN - 1]);


    //1. do reset   
    gf_hw_reset(gf_dev);    

    gf_spi_read_word(gf_dev, 0x002e, &reg_value);
    reg_value &= 0xFFFE;
    gf_spi_write_word(gf_dev, 0x002e, reg_value);

    version = gf_get_chip_version(gf_dev);
    gf_debug(DEFAULT_DEBUG, "chip verison = %x \n", version);
    if(version == 0)
    return -1;

    //2. hold mcu
    mcu_disable(gf_dev);
    gf_spi_write_word(gf_dev, 0x0064, 0x0008); 

    //gf_spi_write_word(gf_dev, 0x009c, 0x0100);
    gf_spi_read_data(gf_dev, 0x009C, 2, reg_buf);
    reg_buf[0] = 0x00;
    reg_buf[1] = 0x01;
    gf_spi_write_data(gf_dev, 0x009C, 2, reg_buf);


    //3. begin to write fw data
    ret = gf_memset(gf_dev, 0x0a00, GOODIX_FW_DATA_LEN, fw_buf);
    if (ret != 0){
        pr_err("gf:fw update error\n");
        mcu_enable(gf_dev);
        return -1;
    }
 
/*
    gf_spi_write_data(gf_dev, 0x0a00, GOODIX_FW_DATA_LEN, buf);
    mdelay(1);
    gf_spi_read_data(gf_dev, 0x0a00, GOODIX_FW_DATA_LEN, temp_buffer);


    ret = memcmp(buf, temp_buffer, GOODIX_FW_DATA_LEN);
    if(ret != 0){
		pr_info("gf:fw_buf is differ with temp_buf ret=%d\n", ret);
		mcu_enable(gf_dev);
		return -1;
    }
*/
    //Update fw version
    gf_read_fw_version(gf_dev, fw_buf);

    //4. begin to write config data

	dev_info(&gf_dev->spi->dev, "gf:CFG_DATA:0x%x, 0x%x, 0x%x, 0x%x\n" , 
		cfg_buf[GOODIX_FW_CONF_LEN - 4],
		cfg_buf[GOODIX_FW_CONF_LEN - 3],
		cfg_buf[GOODIX_FW_CONF_LEN - 2],
		cfg_buf[GOODIX_FW_CONF_LEN - 1]
		);

/*
    gf_spi_write_data(gf_dev, 0x0b1c, GOODIX_FW_CONF_LEN, (buf+GOODIX_FW_DATA_LEN));
    mdelay(1);
    gf_spi_read_data(gf_dev, 0x0b1c, GOODIX_FW_CONF_LEN, temp_buffer);
    ret = memcmp(buf+GOODIX_FW_DATA_LEN, temp_buffer, GOODIX_FW_CONF_LEN);
    if(ret != 0){
		printk("cfg_data is differ with temp_buf,ret=%d\n", ret);
		mcu_enable(gf_dev);
		return -1;
    }
*/
    ret = gf_memset(gf_dev, 0x0b1c, GOODIX_FW_CONF_LEN, cfg_buf);
    if (ret != 0){
        dev_err(&gf_dev->spi->dev, "cfg update error\n");
        mcu_enable(gf_dev);
        return -1;
    }


    re_calibration_via_efuse(gf_dev);
    gf_spi_write_word(gf_dev, 0x0B18, gf_dev->mode);

    //5. release mcu
    mcu_enable(gf_dev);
 
    return 0;
}


static int gf_fw_conf_update_ex(struct gf_dev* gf_dev, u8 *buf, u32 len)
{
    int ret = 0;

    if (len < (GOODIX_FW_DATA_LEN + GOODIX_FW_CONF_LEN)){
        dev_err(&gf_dev->spi->dev, "%s: length error\n", __func__);
    }
    disable_irq_wake(gf_dev->spi->irq);    
    del_timer_sync(&gf_dev->gf_timer);    

    //update fw & cfg
    ret = gf_fw_conf_update(gf_dev, buf, len, buf + GOODIX_FW_DATA_LEN, GOODIX_FW_CONF_LEN);
    if (0 == ret){

        printk("UPDATED FW&CFG\n");
    }

    gf_dev->gf_timer.expires = jiffies + 2 * HZ;
    add_timer(&gf_dev->gf_timer);
    enable_irq_wake(gf_dev->spi->irq);

    return ret;
}



static int gf_mode_switch(struct gf_dev *gf_dev, u8 new_mode)
{
    static u32 cnt_all = 0; 
    static u32 cnt_bad = 0;
    u8 *gf_cfg_tmp = NULL;
    u16 mode = 0;
    u8 ret = 0;
    //u16 temp = 0;

    gf_hw_reset(gf_dev);
    disable_irq_wake(gf_dev->spi->irq);
    udelay(100);   
	
    mcu_disable(gf_dev);
    udelay(100);
	gf_debug(DEFAULT_DEBUG, "Old Mode = 0x%x, New Mode=0x%x, read_flag=%d\n", gf_dev->mode, new_mode, read_flag);


#if 0
	if ((((gf_dev->mode & 0xff00) >> 8) == GF_IMAGE_MODE) && (new_mode != GF_IMAGE_MODE)){
		printk("gf: leave image mode, del timer\n");
		del_timer(&gf_dev->gf_timer); // disable ESD Check
	}
	if ((((gf_dev->mode & 0xff00) >> 8) != GF_IMAGE_MODE) && (new_mode == GF_IMAGE_MODE)){
		gf_dev->gf_timer.expires = jiffies + ESD_CHECK_TIME * HZ;
		add_timer(&gf_dev->gf_timer);
		printk("gf: start image mode, add timer\n");
	}
#else
	if ((((gf_dev->mode & 0xff00) >> 8) == GF_IMAGE_MODE) && (new_mode != GF_IMAGE_MODE)){
		printk("gf: leave image mode, cancel work\n");
		cancel_delayed_work(&gf_dev->delay_work);
	}
	if ((((gf_dev->mode & 0xff00) >> 8) != GF_IMAGE_MODE) && (new_mode == GF_IMAGE_MODE)){
		queue_delayed_work(esd_workqueue, &gf_dev->delay_work, ESD_CHECK_TIME * HZ);
		printk("gf: start image mode, start work\n");
	}
#endif
	
	#if SLEEP_MODE_TEST
	if(new_mode == GF_SLEEP_MODE){
		enter_sleep_mode(gf_dev);
	}
	else
	#endif
	{
	
	pr_info("gf:----resume_from_sleep_mode----\n");

    gf_spi_write_word(gf_dev, 0x009c, 0x0100);		
    gf_spi_write_word(gf_dev, 0x2e, 0x0000); 
    re_calibration_via_efuse(gf_dev);

    gf_spi_read_word(gf_dev, 0x0B18, &mode);
    mode = (mode & 0x00FF) | (new_mode << 8);
    gf_dev->mode = mode; //save mode for esd reset
    gf_spi_write_word(gf_dev, 0x0B18, mode);  
    gf_spi_read_word(gf_dev, 0x0B18, &mode);
    dev_info(&gf_dev->spi->dev, "set gf mode 0x%x", mode);
    switch(new_mode){

    case GF_DEBUG_MODE:
    case GF_CALIB_MODE:
       atomic_set(&isPause, 0);
       break;

    default:
       atomic_set(&isPause, 1);
       break;
    } 

    //4. begin to write config data

    //find out ff or normal mode
	if(mp1_flag == 1)
	{
	    if(new_mode == GF_FF_MODE)
			gf_cfg_tmp = gf_cfg_data_ff;
	    else
			gf_cfg_tmp = gf_cfg_data;    
	}
	else
	{
		if(new_mode == GF_FF_MODE)
			gf_cfg_tmp = gf_cfg_data_ff_eco;
	    else
			gf_cfg_tmp = gf_cfg_data_eco;
	}


//    gf_spi_write_data(gf_dev, 0x0b1c, 120, gf_cfg_tmp);
 /* 
    for(i = 0; i < 120; i +=2,j++){
    temp = gf_cfg_tmp[i] | gf_cfg_tmp[i+1]<<8; 
    gf_spi_write_word(gf_dev, 0x0b1c+j, temp);
    }

    mdelay(1);


    gf_spi_read_data(gf_dev, 0x0b1c, 120, buf_01);

    ret = memcmp(gf_cfg_tmp, buf_01, 120);
    if(ret != 0){
    cnt_bad++;
    pr_err("cfg_data is differ with temp_buf,ret=%d\n", ret);
    }
*/
    ret = gf_memset(gf_dev, 0x0b1c, 120, gf_cfg_tmp);
    if (ret != 0){
        cnt_bad++;
    }
 
    cnt_all++;

    //pr_info("cnt all 0x%x cnt bad 0x%x \n", cnt_all, cnt_bad);
    mcu_enable(gf_dev); 
	}
    
    enable_irq_wake(gf_dev->spi->irq);
    
   return 0;    
}


static int gf_hw_initial(struct gf_dev *gf_dev)
{
    int timeout = 5;
    int ret = 0;
    //for gf
	
	timeout = 0;
    do {
		//download firmware
		if(mp1_flag == 1)
		{
			ret = gf_fw_conf_update(gf_dev, gf_fw_data + 12, GOODIX_FW_DATA_LEN, 
                                        gf_cfg_data,GOODIX_FW_CONF_LEN);
		}
		else
		{
			ret = gf_fw_conf_update(gf_dev, gf_fw_data + 12, GOODIX_FW_DATA_LEN, 
                                        gf_cfg_data_eco,GOODIX_FW_CONF_LEN);
		}
		if(0 == ret)
		{
			return 0;
		}
		timeout ++;
		gf_debug(DEFAULT_DEBUG,"retry Count=%d\n", timeout);
    }while(timeout < 5 );

    return -1;
}


uint16_t  cal_crc(uint8_t * pchMsg, uint16_t	wDataLen)
{
    uint8_t i, chChar;
    uint16_t  wCRC = 0xFFFF;
    while (wDataLen--)
    {
		chChar = *pchMsg++;
		wCRC ^= (((uint16_t) chChar) << 8);
		for (i = 0; i < 8; i++)
		{
			if (wCRC & 0x8000)
			wCRC = (wCRC << 1) ^ CRC_16_POLYNOMIALS;
			else wCRC <<= 1;
		}
    }

    return wCRC;
}

/*-------------------------------------------------------------------------*/
/* Read-only message with current device setup */
static ssize_t gf_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
    ssize_t			status = 0;

    FUNC_ENTRY();
    if ((count > bufsiz)||(count == 0)) {
		pr_warn("gf:Max size for write buffer is %d. wanted length is %d\n", bufsiz, count);
		FUNC_EXIT();
		return -EMSGSIZE;
    }
    if ((g_msg & GF_IMAGE_MASK) == GF_IMAGE_ENABLE)
    {
		mutex_lock(&frame_lock);
		status = copy_to_user(buf, &g_frame_buf, count);
		mutex_unlock(&frame_lock);

		if(status)
		{
			pr_info("gf:fail to copy data to user space %d.\n", status);
			status = -1;
		}
		else
		{
			g_msg &= ~GF_IMAGE_ENABLE;   
			pr_info("gf_read raw data\n");  
			//            gf_read_fw_version(gf_dev);
			return count;
		}
    }
    else{
		pr_info("gf:raw data not ready\n");
    }
    return 0;
}

/* Write-only message with current device setup */
static ssize_t gf_write(struct file *filp, const char __user *buf,
	size_t count, loff_t *f_pos)
{
    struct gf_dev *gf_dev = filp->private_data;
    ssize_t			status = 0;
    FUNC_ENTRY();

    if(count > bufsiz) {
		dev_warn(&gf_dev->spi->dev, "gf: Max size for write buffer is %d\n", bufsiz);
		return -EMSGSIZE;
    } 

    mutex_lock(&gf_dev->buf_lock);
    status = copy_from_user(gf_dev->buffer + GF_WDATA_OFFSET, buf, count);
    if(status == 0) {
		gf_dev->spi->max_speed_hz=SPI_DEFAULT_SPEED;

		spi_setup(gf_dev->spi);

    } else {
		dev_err(&gf_dev->spi->dev, "Failed to xfer data through SPI bus.\n");
		status = -EFAULT;
    }
    mutex_unlock(&gf_dev->buf_lock);
    FUNC_EXIT();

    return status;
}


static u8 buf[1024] = {0}; 
static long gf_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    struct gf_dev *gf_dev = NULL;
    struct gf_ioc_transfer *ioc = NULL;
    int	err = 0;
    u32	tmp = 0;
    u8  u8_tmp = 0;
    int retval = 0;
    u16 mode = 0xFFFF;  
    int ret = 0; 

    FUNC_ENTRY();
    if (_IOC_TYPE(cmd) != GF_IOC_MAGIC)
		return -ENOTTY;
    /* Check access direction once here; don't repeat below.
     * IOC_DIR is from the user perspective, while access_ok is
     * from the kernel perspective; so they look reversed.
     */
    if (_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
    if (err == 0 && _IOC_DIR(cmd) & _IOC_WRITE)
		err = !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
    if (err)
		return -EFAULT;

    gf_dev = (struct gf_dev *)filp->private_data;

    switch(cmd) 
    {
	case GF_IOC_CMD:
	    ioc = kzalloc(sizeof(*ioc), GFP_KERNEL);
	    /*copy command data from user to kernel.*/
	    if(copy_from_user(ioc, (struct gf_ioc_transfer*)arg, sizeof(*ioc))){
			dev_err(&gf_dev->spi->dev, "Failed to copy command from user to kernel.\n");
			retval = -EFAULT;
			break;
	    }

	    if((ioc->len > bufsiz)||(ioc->len == 0)) {
			dev_warn(&gf_dev->spi->dev, "The request length[%d] is longer than supported maximum buffer length[%d].\n", 
				ioc->len, bufsiz);
			retval = -EMSGSIZE;
			break;
	    }

	    //mutex_lock(&gf_dev->buf_lock);
	    //gf_dev->spi->max_speed_hz=1*1000*1000;
	    //spi_setup(gf_dev->spi);
	    if(ioc->cmd == GF_R) {
			/*if want to read data from hardware.*/
			gf_debug(DEFAULT_DEBUG,"Read data from 0x%x, len = 0x%x buf = 0x%p\n", 
					ioc->addr, ioc->len, ioc->buf);
			mutex_lock(&frame_lock);
			mcu_disable(gf_dev);
			gf_spi_read_data(gf_dev, ioc->addr, ioc->len, buf);
			mcu_enable(gf_dev);
			mutex_unlock(&frame_lock);

			mutex_lock(&gf_dev->buf_lock);
			ret = copy_to_user(ioc->buf, buf, ioc->len);
			mutex_unlock(&gf_dev->buf_lock);

			if(ret) {
				dev_err(&gf_dev->spi->dev, "Failed to copy data from kernel to user.\n");
				retval = -EFAULT;
				break;
			}
	    } else if (ioc->cmd == GF_W) {
			/*if want to read data from hardware.*/
			gf_debug(DEFAULT_DEBUG,"Write data from 0x%x, len = 0x%x\n", ioc->addr, ioc->len);
//			pr_info("Write data from 0x%x, len = 0x%x\n", ioc->addr, ioc->len);
			
			ret = copy_from_user(buf, ioc->buf, ioc->len);

			gf_debug(DEFAULT_DEBUG,"Write data:0x%x,0x%x\n", buf[0],buf[1]);
			    
			if(ret){
				dev_err(&gf_dev->spi->dev, "Failed to copy data from user to kernel.\n");
				retval = -EFAULT;
				break;
			}
			mutex_lock(&frame_lock);
			mcu_disable(gf_dev);
			gf_spi_write_data(gf_dev, ioc->addr, ioc->len, buf);
			mcu_enable(gf_dev);
			mutex_unlock(&frame_lock);

	    } else if (ioc->cmd == GF_MCU_DISABLE) {
			mcu_disable(gf_dev);
	    } else if (ioc->cmd == GF_MCU_ENABLE) {
			mcu_enable(gf_dev);
	    } else if (ioc->cmd == GF_MCU_RESET) {
			gf_hw_reset(gf_dev);    
	    } else if (ioc->cmd == GF_MCU_UPDATE) {
			ret = gf_fw_conf_update_ex(gf_dev, ioc->buf, ioc->len);
			if(ret) {
				dev_err(&gf_dev->spi->dev, "Failed to update fw and conf.\n");
				retval = -EFAULT;
				break;
			}
                      
                        
	    } else if (ioc->cmd == GF_GET_STATUS){
			ret = copy_to_user(ioc->buf, &get_status, ioc->len);
			if(ret) {
				dev_err(&gf_dev->spi->dev, "Failed to copy data from kernel to user.\n");
				retval = -EFAULT;
				break;
			}
	    } else if (ioc->cmd == GF_GET_VERSION){
		ret = copy_to_user(ioc->buf, gf_dev->fw_ver, ioc->len);
		if(ret) {
		    dev_err(&gf_dev->spi->dev, "Failed to copy data from kernel to user.\n");
		    retval = -EFAULT;
		    break;
		}
	    } else if (ioc->cmd == GF_DATA_SUSPEND) {
            atomic_set(&isPause , 1);
	    }else {
			dev_warn(&gf_dev->spi->dev, "Error command for gf.\n");	
			retval = -EFAULT;
	    }

	    if(ioc != NULL) {
			kfree(ioc);
			ioc = NULL;
	    }
	    break;
	case GF_IOC_REINIT:
	    disable_irq_wake(gf_dev->spi->irq);
	    //gf_hw_reset1(gf_dev);
	    enable_irq_wake(gf_dev->spi->irq);
	    //gf_hw_init(gf_dev);
	    gf_debug(FLOW_DEBUG,"wake-up gf\n");
	    break;
	case GF_IOC_SETSPEED:
	    retval = __get_user(tmp, (u32 __user*)arg);
	    if(tmp > SPI_MAX_SPEED) {
			dev_warn(&gf_dev->spi->dev, "The maximum SPI speed is %dMHz.\n", SPI_MAX_SPEED / (1000*1000));
			retval = -EMSGSIZE;
			break;
	    }
	    if(retval == 0) {
			mutex_lock(&frame_lock);
			gf_dev->spi->max_speed_hz=tmp;
			mcu_disable(gf_dev);
			spi_setup(gf_dev->spi);
			mcu_enable(gf_dev);
			mutex_unlock(&frame_lock);
			gf_debug(DEFAULT_DEBUG, "spi speed changed to %d\n", tmp);
	    }	
	    break;
	case GF_IOC_SETMODE:
		//gf_debug(DEFAULT_DEBUG, "--  %s  switch mode \n", __func__);
	    retval = __get_user(u8_tmp, (u8 __user*)arg);
	    if (retval==0) {
		mutex_lock(&frame_lock);
		gf_mode_switch(gf_dev, u8_tmp);
		mutex_unlock(&frame_lock);
	    }

	    break;
#if 1
	case GF_IOC_GETMODE:
	    mutex_lock(&frame_lock);
	    mcu_disable(gf_dev);
	    gf_spi_read_word(gf_dev, 0x0B18, &mode);
	    mcu_enable(gf_dev);  
	    mutex_unlock(&frame_lock);

	    mode = (mode >>8) & 0xff;
	    tmp = mode;
	    retval = __put_user(tmp, (u8 __user*)arg);
	    break;
#endif
	case GF_IOC_SETENMODE:
	    retval = __get_user(u8_tmp, (u8 __user*)arg);
	    dev_info(&gf_dev->spi->dev, "GF retval=%x, enhace mode=%x\n", retval, u8_tmp);

	    if (retval==0) {
			mutex_lock(&frame_lock);
			mcu_disable(gf_dev);
			gf_spi_read_word(gf_dev, 0x0B19, &mode);
			mode = mode | u8_tmp;  
			dev_info(&gf_dev->spi->dev, "%s mode before 0x%x \n", __func__, mode);            
			gf_spi_write_word(gf_dev, 0x0B19, mode);  
			gf_spi_read_word(gf_dev, 0x0B19, &mode);
			dev_info(&gf_dev->spi->dev, "%s mode after0x%x \n", __func__, mode); 
			mcu_enable(gf_dev);
			mutex_unlock(&frame_lock);
	    }

	    break;
	case GF_IOC_GETENMODE:
	    mutex_lock(&frame_lock);
	    mcu_disable(gf_dev);
	    gf_spi_read_word(gf_dev, 0x0B19, &mode);
	    mcu_enable(gf_dev);  
	    mutex_unlock(&frame_lock);

	    mode = mode & 0xff;
	    tmp = mode;
	    retval = __put_user(tmp, (u8 __user*)arg);
	    dev_info(&gf_dev->spi->dev, "GF retval=%x, mode=%x.\n", retval, tmp);
	    break;
	case GF_IOC_MSG:
	    mutex_lock(&frame_lock);
	    retval = __put_user(g_msg, (u8 __user*)arg);   
		//printk("gf: %s g_msg = 0x%02X\n", __func__, g_msg);
	    mutex_unlock(&frame_lock);

	    break;
	default:
	    dev_warn(&gf_dev->spi->dev, "gf doesn't support this command(%d)\n", cmd);
	    retval = -EFAULT;
	    break;
    }

    FUNC_EXIT();
    return retval;
}

static unsigned int gf_poll(struct file *filp, struct poll_table_struct *wait)
{
    struct gf_dev *gf_dev = filp->private_data;
    //debug for GF
    //gf_spi_read_byte(gf_dev, GF_BUFFER_STATUS, &gf_dev->buf_status);
    if((gf_dev->buf_status & (0x1<<7)) == GF_BUF_STA_READY) {
		return (POLLIN|POLLRDNORM);
    } else {
		gf_debug(DEFAULT_DEBUG, "Poll no data.\n");
    }

    return 0;
}
/*
   static void gf_timer_func(unsigned long arg)
   {
   struct gf_dev* gf_dev = (struct gf_dev*)arg;
   gf_debug(DEFAULT_DEBUG, "gf_timer_func\n");
//schedule_work(&gf_dev->spi_work);	
}
 */
#if 1  //bernard 20150911
static int gf_frame_handler(struct gf_dev* gf_dev)
{
    //struct gf_dev* gf_dev = (struct gf_dev*)para;
    //int status = 0;    
    int timeout = 0;
//    int retval = 0;
    u16 fifo_r, fifo_w;
    u16 len = 0;
    u16 crc_val, crc_read;
    u32 total_len = 0;
    u16 read_addr = 0x0200;
    u8 *ptr = g_frame_buf;
    u16 value = 0;
    u8 count  = 0;
#if SPI_DEFAULT_SPEED != SPI_MAX_SPEED
    long spi_speed = SPI_DEFAULT_SPEED;
#endif
   // do
    //{
	//	gf_debug(DEFAULT_DEBUG,"========== wait =============\n");
	//	retval = wait_event_interruptible(waiter, read_flag != 0);
		//gf_debug(DEFAULT_DEBUG,"%s   entry\n", __func__);
		read_addr = 0x0200;
		len = 0;
		total_len = 0;
		ptr = g_frame_buf;
		timeout = 50;

//		mutex_lock(&frame_lock);

		//fifo read
		do {
			count = 0;
			do{  
			    //check 0x24[bit15]
			    mdelay(1);
			    gf_spi_read_word(gf_dev, 0x0024, &value);
			    //pr_info("%s, check 0x24 bit15 0x%x\n", __func__, value);
			    count++;
	        }while((!(value&0x8000)) && (count < 80) && ((FRAME_LENGTH - total_len) > (0x400*2)));

	        if(count >= 80){ 
			    pr_err("[%s]: failed to check 0x24 bit15\n", __func__);
			    timeout = 0;
			    break;
	        }    

	    //  0xD0----read
	    //  0xD2----write
	    gf_spi_read_word(gf_dev, 0x00D0, &fifo_r);
	    gf_spi_read_word(gf_dev, 0x00D2, &fifo_w);

	    if (fifo_w > fifo_r) {
			len = (fifo_w - fifo_r);
	    } else if (fifo_w < fifo_r) {	
			len = (fifo_w + 0x400 - fifo_r);
	    } else {
			len = 0;
	    }

	    len *= 2;

	    if (len > 0)
	    {
		if((total_len + len) > FRAME_LENGTH){
		    len = FRAME_LENGTH - total_len;
		}

#if SPI_DEFAULT_SPEED != SPI_MAX_SPEED
		spi_speed = gf_dev->spi->max_speed_hz;
		gf_dev->spi->max_speed_hz = SPI_GET_DATA_SPEED;
		spi_setup(gf_dev->spi);

		gf_spi_read_data_big_end(gf_dev, read_addr, len, ptr);

		gf_dev->spi->max_speed_hz = spi_speed;
		spi_setup(gf_dev->spi);
#else

		gf_spi_read_data_big_end(gf_dev, read_addr, len, ptr);
#endif
			read_addr += (len/2);
			if (read_addr >= 0x0600) {
				read_addr = 0x0200 + (read_addr - 0x0600);
			}

			ptr += len;
			total_len += len;
			timeout = 5; //retry 5ms
	    }else{
			timeout--;
			msleep(1);
	    }
	}while((timeout > 0) && (total_len < FRAME_LENGTH));

	kernel_time(TIME_STOP);

	if (timeout > 0){
	    crc_read = (u16)(g_frame_buf[FRAME_LENGTH - 2] << 8) + g_frame_buf[FRAME_LENGTH - 1];
	    crc_val = cal_crc(g_frame_buf, RAWDATA_LENGTH);
	    if ( crc_val  == crc_read){
			//printk("gf: %s g_msg = 0x%02X\n ", __func__, g_msg);
		g_msg |= GF_IMAGE_ENABLE;
#ifdef GF_FASYNC
		if((gf_dev->async)/*&& (async_flag !=0)*/) {
			//printk("gf:async_flag = %d\n", async_flag);
		    kill_fasync(&gf_dev->async, SIGIO, POLL_IN);
		}
		async_flag++;
#endif
	    }
	    else{
			pr_err("gf:crc error!\n");
	    }
	}
	else{
	    pr_warn("gf:read data timeout \n");
	    debug_count = 0;   
	}
	gf_spi_write_word(gf_dev, 0x0B1A, 0);
	gf_spi_write_word(gf_dev, 0x0B05, 0xAAAA);

	mcu_enable(gf_dev);	    
	read_flag = 0;
	//printk("gf: %s   read_flag = %d\n", __func__, read_flag);
//	mutex_unlock(&frame_lock);

//    }while(!kthread_should_stop());

    return 0;
}
#else
static int gf_frame_handler(void* para)
{
    struct gf_dev* gf_dev = (struct gf_dev*)para;
    //int status = 0;    
    int timeout = 0;
    int retval = 0;
    u16 fifo_r, fifo_w;
    u16 len = 0;
    u16 crc_val, crc_read;
    u32 total_len = 0;
    u16 read_addr = 0x0200;
    u8 *ptr = g_frame_buf;
    u16 value = 0;
    u8 count  = 0;
#if SPI_DEFAULT_SPEED != SPI_MAX_SPEED
    long spi_speed = SPI_DEFAULT_SPEED;
#endif
    do
    {
		gf_debug(DEFAULT_DEBUG,"========== wait =============\n");
		retval = wait_event_interruptible(waiter, read_flag != 0);

		read_addr = 0x0200;
		len = 0;
		total_len = 0;
		ptr = g_frame_buf;
		timeout = 50;

		mutex_lock(&frame_lock);

		//fifo read
		do {
			count = 0;
			do{  
			    //check 0x24[bit15]
			    mdelay(1);
			    gf_spi_read_word(gf_dev, 0x0024, &value);
			    //pr_info("%s, check 0x24 bit15 0x%x\n", __func__, value);
			    count++;
	        }while((!(value&0x8000)) && (count < 80) && ((FRAME_LENGTH - total_len) > (0x400*2)));

	        if(count >= 80){ 
			    pr_err("[%s]: failed to check 0x24 bit15\n", __func__);
			    timeout = 0;
			    break;
	        }    

	    //  0xD0----read
	    //  0xD2----write
	    gf_spi_read_word(gf_dev, 0x00D0, &fifo_r);
	    gf_spi_read_word(gf_dev, 0x00D2, &fifo_w);

	    if (fifo_w > fifo_r) {
			len = (fifo_w - fifo_r);
	    } else if (fifo_w < fifo_r) {	
			len = (fifo_w + 0x400 - fifo_r);
	    } else {
			len = 0;
	    }

	    len *= 2;

	    if (len > 0)
	    {
		if((total_len + len) > FRAME_LENGTH){
		    len = FRAME_LENGTH - total_len;
		}

#if SPI_DEFAULT_SPEED != SPI_MAX_SPEED
		spi_speed = gf_dev->spi->max_speed_hz;
		gf_dev->spi->max_speed_hz = SPI_GET_DATA_SPEED;
		spi_setup(gf_dev->spi);

		gf_spi_read_data_big_end(gf_dev, read_addr, len, ptr);

		gf_dev->spi->max_speed_hz = spi_speed;
		spi_setup(gf_dev->spi);
#else

		gf_spi_read_data_big_end(gf_dev, read_addr, len, ptr);
#endif
			read_addr += (len/2);
			if (read_addr >= 0x0600) {
				read_addr = 0x0200 + (read_addr - 0x0600);
			}

			ptr += len;
			total_len += len;
			timeout = 5; //retry 5ms
	    }else{
			timeout--;
			msleep(1);
	    }
	}while((timeout > 0) && (total_len < FRAME_LENGTH));

	kernel_time(TIME_STOP);

	if (timeout > 0){
	    crc_read = (u16)(g_frame_buf[FRAME_LENGTH - 2] << 8) + g_frame_buf[FRAME_LENGTH - 1];
	    crc_val = cal_crc(g_frame_buf, RAWDATA_LENGTH);
	    if ( crc_val  == crc_read){
		g_msg |= GF_IMAGE_ENABLE;
#ifdef GF_FASYNC
		if((gf_dev->async)/*&& (async_flag !=0)*/) {
			//printk("gf:async_flag = %d\n", async_flag);
		    kill_fasync(&gf_dev->async, SIGIO, POLL_IN);
		}
		async_flag++;
#endif
	    }
	    else{
			pr_err("gf:crc error!\n");
	    }
	}
	else{
	    pr_warn("gf:read data timeout \n");
	    debug_count = 0;   
	}
	gf_spi_write_word(gf_dev, 0x0B1A, 0);
	gf_spi_write_word(gf_dev, 0x0B05, 0xAAAA);

	mcu_enable(gf_dev);	    
	read_flag = 0;
	mutex_unlock(&frame_lock);

    }while(!kthread_should_stop());

    return 0;
}
#endif


static irqreturn_t gf_irq(int irq, void* handle)
{
	fp_flag = 1;
	
	wake_up_interruptible(&int_waiter);
    //printk("gf:%s--Exit\n", __func__);
    return IRQ_HANDLED;
}

static int fp_event_handler(void *para)
{
	struct gf_dev* gf_dev = (struct gf_dev*)para;   
    u16 mode = 0x55;
    u16	sys_status = 0;
    u16	status = 0;
    u16	pause = 0;
    u16 regs[4] = {0}; //0x0B18, 0x0B19, 0x0B1A, 0x0B1B
    int _isPause = 0;
    int _isSleep = 0;
    u16 reset_flag = 0;
	u16 clock = 0;

	do {
		wake_unlock(&wl);
        atomic_set(&isIrq, 0);
		wait_event_interruptible(int_waiter, fp_flag != 0);
		wake_lock(&wl);
        mutex_lock(&frame_lock);
        atomic_set(&isIrq, 1);
        //printk("gf: event handle\n");
		fp_flag = 0;

       //printk("gf: %s  isPause \n", __func__); 

		_isPause = atomic_read(&isPause);

		_isSleep = atomic_read(&isSleep);
		//printk("gf: %s  after isSleep \n", __func__);
        reset_flag = 0;
		
		
		
        if ((read_flag == 1) || (_isSleep == 1)){
            gf_debug(DEFAULT_DEBUG, "read_flag = %d, isSleep = %d\n", read_flag, _isSleep);
            mutex_unlock(&frame_lock);
            continue;
	    }
	    else{
	        //gf_debug(DEFAULT_DEBUG,"IRQ IN\n");
	    }
	 //printk("gf: %s  mcu_disable \n", __func__);	
	    mcu_disable(gf_dev);	
			gf_spi_read_word(gf_dev, 0x002e, &reset_flag);
			//printk("gf: reset_flag = 0x%02X\n", reset_flag);	
			if(reset_flag & 0x0001)	{		
				gf_spi_write_word(gf_dev, 0x002e, 0x0000);		
				//printk("gf: clear reset int flag\n");
           mutex_unlock(&frame_lock);
				continue;	
			}	

	    gf_spi_read_data(gf_dev, 0x0B18, 8, (u8*)regs);
	    mode = (regs[0] >>8) & 0xff;
	    pause = regs[1];
	    status = regs[2] & 0xff;
	    sys_status = (regs[3]>>8)&0xff;

	 	gf_spi_read_word(gf_dev, 0x009c, &clock);
	    gf_debug(DEFAULT_DEBUG, "IRQ status = 0x%x, mode = 0x%x sys_status 0x%x pause=%d\n",
			(u8)(status), (u8)(mode), (u8)sys_status, (u8)_isPause);


	    if(sys_status == 0xE3 || sys_status == 0xE0 || sys_status == 0xE1 || clock == 0x0005) {
			pr_err("gf:Sensor error, sys_status=0x%x \n", sys_status);
			//pr_err("we need reset fw and download fw cfg");
			gf_hw_initial(gf_dev);
			mcu_enable(gf_dev);
            mutex_unlock(&frame_lock);
			continue;
	    }


	    if((status & GF_BUF_STA_MASK) != GF_BUF_STA_READY) {
			mcu_enable(gf_dev);
			gf_debug(DEFAULT_DEBUG, "Invalid IRQ_status= 0x%x mode=0x%x sys_status=0x%x\n", status, mode, sys_status);
            mutex_unlock(&frame_lock);
			continue;
	    } 

	    //save status for app
	    get_status = status;

	    switch(mode){
		case GF_FF_MODE:			
		case GF_IMAGE_MODE:
		case GF_CALIB_MODE:
		case GF_DEBUG_MODE:

		    if((status & GF_KEY_MASK) == GF_KEY_ENABLE)
		    {
			    g_msg = (u8)((status & 0x007f) | (1<<5));
				//printk("gf: %s  g_msg =((status & 0x007f) | (1<<5))=0x%02X\n", __func__, g_msg);
			    if(status & GF_KEY_STA)
			    {
			        if (mode == GF_FF_MODE){
	            		gf_debug(DEFAULT_DEBUG, "Flash finger mode. Wake!\n");
	            		input_report_key(gf_dev->input, KEY_FINGER_PRINT, 1);
	            		input_sync(gf_dev->input);	    
	            		input_report_key(gf_dev->input, KEY_FINGER_PRINT, 0);
	            		input_sync(gf_dev->input);
			        }
			        g_msg |= (1<<4);    
					//printk("gf: %s  g_msg |= (1<<4) =0x%02X\n", __func__, g_msg);
		            gf_spi_write_word(gf_dev, 0x0B19, pause & 0xFF00);
	                atomic_set(&isPause , 0);
			    }
			    else
			    {
			        g_msg &= ~(1<<4);
					//printk("gf: %s  g_msg &= (1<<4) =0x%02X\n", __func__, g_msg);
	                gf_spi_write_word(gf_dev, 0x0B19, pause | 0x0001);
					async_flag = 0;

			    }
			    gf_spi_write_word(gf_dev, 0x0B1A, 0x00);
			    mcu_enable(gf_dev);
	    #ifdef GF_FASYNC
			    if(gf_dev->async) {
			        kill_fasync(&gf_dev->async, SIGIO, POLL_IN);
			    }
	    #endif
		    }
		    else if(status & (0x1 << 6))
		    {
		        if (_isPause == 0){
		            //read_flag = 1;
					//printk("gf: %s   read_flag = %d\n", __func__, read_flag);
		            kernel_time(TIME_START);
		            //wake_up_interruptible(&waiter);
		            gf_frame_handler(gf_dev);    //bernard 20150911
	            }
	            else{
	                gf_spi_write_word(gf_dev, 0x0B19, pause | 0x0001);
	                gf_spi_write_word(gf_dev, 0x0B1A, 0x00);
			        mcu_enable(gf_dev);
	            }
		    }
		    break;
		case GF_KEY_MODE:	
		    if  ((status & GF_KEY_MASK) && ((status & GF_BUF_STA_MASK) == GF_BUF_STA_READY)) {
				input_report_key(gf_dev->input, KEY_HOME, (status & GF_KEY_STA)>>4);
				input_sync(gf_dev->input);
				gf_debug(DEFAULT_DEBUG,"gf key report\n");
		    }
		    gf_spi_write_word(gf_dev, 0x0B1A, 0x00);
		    mcu_enable(gf_dev);
		    break;
		case GF_SLEEP_MODE:
		    dev_warn(&gf_dev->spi->dev, "Should not happen in sleep mode.\n");
		    mcu_enable(gf_dev);
		    break;
		default:
		    pr_warn("Unknown mode. mode = 0x%x\n", mode);
		    gf_spi_write_word(gf_dev, 0x0B1A, 0x00);
		    mcu_enable(gf_dev);
		    break;
	    }
		
        mutex_unlock(&frame_lock);
		usleep(500);
	}while(!kthread_should_stop());
	return 0;
}

static int gf_open(struct inode *inode, struct file *filp)
{
    struct gf_dev *gf_dev;
    int			status = -ENXIO;

    FUNC_ENTRY();
    printk("BUILD INFO:%s,%s\n",__DATE__, __TIME__);
    
    mutex_lock(&device_list_lock);

    list_for_each_entry(gf_dev, &device_list, device_entry) {
	if(gf_dev->devt == inode->i_rdev) {
	   // gf_debug(DEFAULT_DEBUG, "Found\n");
	    status = 0;
	    break;
	}
    }

    if(status == 0){
	mutex_lock(&gf_dev->buf_lock);
	if( gf_dev->buffer == NULL) {
	    gf_dev->buffer = kzalloc(bufsiz + GF_RDATA_OFFSET, GFP_KERNEL);
	    if(gf_dev->buffer == NULL) {
		dev_dbg(&gf_dev->spi->dev, "open/ENOMEM\n");
		status = -ENOMEM;
	    }
	}
	mutex_unlock(&gf_dev->buf_lock);

	if(status == 0) {
	    gf_dev->users++;
	    filp->private_data = gf_dev;
	    nonseekable_open(inode, filp);
	    gf_debug(DEFAULT_DEBUG, "Succeed to open device, bufsiz=%d. irq = %d\n", bufsiz, gf_dev->spi->irq);
	    enable_irq_wake(gf_dev->spi->irq);
	}
    } else {
	dev_err(&gf_dev->spi->dev, "No device for minor %d\n", iminor(inode));
    }
    mutex_unlock(&device_list_lock);


    FUNC_EXIT();
    return status;
}

#ifdef GF_FASYNC
static int gf_fasync(int fd, struct file *filp, int mode)
{
    struct gf_dev *gf_dev = filp->private_data;
    int ret;

    FUNC_ENTRY();
    ret = fasync_helper(fd, filp, mode, &gf_dev->async);
    FUNC_EXIT();
    return ret;
}
#endif

static int gf_release(struct inode *inode, struct file *filp)
{
    struct gf_dev *gf_dev;
    int    status = 0;
    FUNC_ENTRY();
    mutex_lock(&device_list_lock);
    gf_dev = filp->private_data;
    filp->private_data = NULL;

    /*last close??*/
    gf_dev->users --;
    if(!gf_dev->users) {
	dev_info(&gf_dev->spi->dev, "gf_realease called. \n");
	//gf_debug(DEFAULT_DEBUG, "disble_irq. irq = %d\n", gf_dev->spi->irq);
	disable_irq_wake(gf_dev->spi->irq);
    }
    mutex_unlock(&device_list_lock);
    FUNC_EXIT();
    return status;
}

static const struct file_operations gf_fops = {
    .owner =	THIS_MODULE,
    /* REVISIT switch to aio primitives, so that userspace
     * gets more complete API coverage.  It'll simplify things
     * too, except for the locking.
     */
    .write =	gf_write,
    .read =		gf_read,
    .unlocked_ioctl = gf_ioctl,
    .open =		gf_open,
    .release =	gf_release,
    .poll   = gf_poll,
#ifdef GF_FASYNC
    .fasync = gf_fasync,
#endif
};
/* The main reason to have this class is to make mdev/udev create the
 * /dev/spidevB.C character device nodes exposing our userspace API.
 * It also simplifies memory management.
 */
static int gf_parse_dt(struct gf_dev *gf_dev)
{
    struct device_node *np = gf_dev->spi->dev.of_node;
    int ret = 0;

    ret = of_get_named_gpio(np, "gf,gpio_irq", 0);
    if (ret < 0) {
        pr_err("failed to get \"gf,gpio_irq\"\n");
        goto err;
    }
    gf_dev->irq_gpio = ret;
	gpio_request(gf_dev->irq_gpio, "gf_irq");
	
    ret = of_get_named_gpio(np, "gf,gpio_reset", 0);
    if (ret < 0) {
        pr_err("failed to get \"gf,gpio_reset\"\n");
        goto err;
    }
	gf_dev->rst_gpio = ret;
	gpio_request(gf_dev->rst_gpio, "gf_rst");

err:
	return ret;
}

static struct class *gf_spi_class;

/*-------------------------------------------------------------------------*/

static int gf_probe(struct spi_device *spi)
{
    struct gf_dev *gf_dev;
    int	status;
    unsigned long minor;
    int err = 0;  
	int chip_version;
	 
    FUNC_ENTRY();
    printk("BUILD INFO:%s,%s\n", __DATE__, __TIME__);
    /* Allocate driver data */
    gf_dev = kzalloc(sizeof(*gf_dev), GFP_KERNEL);
    if (!gf_dev){
	dev_warn(&spi->dev, "Failed to alloc memory for gf device.\n");
	FUNC_EXIT();
	return -ENOMEM;
    }
    
   
    /* Initialize the driver data */
    gf_dev->spi = spi;
	gf_parse_dt(gf_dev);
	sensor_power_onoff(true);
    spin_lock_init(&gf_dev->spi_lock);
    mutex_init(&gf_dev->buf_lock);

    mutex_init(&frame_lock);

    INIT_LIST_HEAD(&gf_dev->device_entry);

    /* If we can allocate a minor number, hook up this device.
     * Reusing minors is fine so long as udev or mdev is working.
     */
    mutex_lock(&device_list_lock);
    minor = find_first_zero_bit(minors, N_SPI_MINORS);
    if (minor < N_SPI_MINORS) {
	struct device *dev;

	status = sysfs_create_group(&spi->dev.kobj,&gf_debug_attr_group);
	if(status){
	    dev_err(&spi->dev, "Failed to create sysfs file.\n");
		mutex_unlock(&device_list_lock);
	    goto err_sysfs;
	}

	gf_dev->devt = MKDEV(SPIDEV_MAJOR, minor);
	dev = device_create(gf_spi_class, &spi->dev, gf_dev->devt,
		gf_dev, DEV_NAME);
	status = IS_ERR(dev) ? -PTR_ERR(dev) : 0;
    } else {
	dev_err(&spi->dev, "no minor number available!\n");
	status = -ENODEV;
    }
    if (status < 0) {
		mutex_unlock(&device_list_lock);
		goto err_device;
    }
	set_bit(minor, minors);
	list_add(&gf_dev->device_entry, &device_list);
    mutex_unlock(&device_list_lock);
			
	gf_dev->buffer = kzalloc(bufsiz + GF_RDATA_OFFSET, GFP_KERNEL);
	if(gf_dev->buffer == NULL) {
	    status = -ENOMEM;
	    goto err_alloc_buffer;
	}
	spi_set_drvdata(spi, gf_dev);

	/*register device within input system.*/
	gf_dev->input = input_allocate_device();
	if(gf_dev->input == NULL) {
	    dev_err(&spi->dev, "Failed to allocate input device.\n");
	    status = -ENOMEM;
	    goto err_input;
	}

	__set_bit(EV_KEY, gf_dev->input->evbit);
	__set_bit(KEY_HOME, gf_dev->input->keybit);	
	__set_bit(KEY_FINGER_PRINT, gf_dev->input->keybit);

	gf_dev->input->name = "tiny4412-key";
	if(input_register_device(gf_dev->input)) {
	    dev_err(&spi->dev, "Failed to register input device.\n");
		input_free_device(gf_dev->input);
		status = -ENOMEM;
	    goto err_input;
	}
	/*setup gf configurations.*/
	//gf_debug(DEFAULT_DEBUG, "Setting gf device configuration.\n");
	/*SPI parameters.*/
#if SPI_DEFAULT_SPEED != SPI_MAX_SPEED
        gf_debug(DEFAULT_DEBUG , "SPI_DEFAULT_SPEED=%dMHz\n", SPI_DEFAULT_SPEED/ SPI_MHZ );
#endif
	gf_dev->spi->mode = SPI_MODE_0; //CPOL=CPHA=0
	gf_dev->spi->max_speed_hz = SPI_DEFAULT_SPEED; //1MHZ
	gf_dev->spi->chip_select = 0;
	gf_dev->spi->irq = gpio_to_irq(gf_dev->irq_gpio);
	gf_dev->spi->bits_per_word = 8; //?
	spi_setup(gf_dev->spi);
    gf_dev->mode = 0x00AA;
	wake_lock_init(&wl, WAKE_LOCK_SUSPEND, "fp_ff");
	
	chip_version = gf_get_chip_version(gf_dev);
	if(chip_version == 0x18e8)
	{
		mp1_flag = 1;
	}
	if (gf_hw_initial(gf_dev) < 0) {
		status = -ENODEV;
		goto err_hw_initial;
	}

	gf_irq_cfg(gf_dev);
	thread = kthread_run(fp_event_handler, (void*)gf_dev, "thread_int");
	if (IS_ERR(thread))
	{
		gf_debug(DEFAULT_DEBUG, "gf thread ERR\n");
		goto err_hw_initial;
	}
	
	read_flag = 0;
	gf_debug(DEFAULT_DEBUG, "gf interrupt NO. = %d\n", gf_dev->spi->irq);
#if 1
	err = request_threaded_irq(spi->irq, NULL, gf_irq, 
		IRQF_TRIGGER_RISING | IRQF_ONESHOT,
		dev_name(&spi->dev), gf_dev);
#else
	err = request_irq(gf_dev->spi->irq, gf_irq, 
		IRQ_TYPE_EDGE_RISING,//IRQ_TYPE_LEVEL_HIGH,
		dev_name(&gf_dev->spi->dev), gf_dev);
#endif
	if(!err) {
	  //  disable_irq_wake(gf_dev->spi->irq);
	}
#if 0  //bernard 20150911
        fpthread = kthread_run(gf_frame_handler, (void*)gf_dev, "thread_fp");
        if(IS_ERR(fpthread)) {
            dev_err(&spi->dev, "Failed to create kernel thread: %ld\n", PTR_ERR(fpthread));
			goto err_kthread;
        } 
#endif

	/*init esd*/
	gf_esd_pet_init(gf_dev);

	dev_info(&spi->dev, "GF installed.\n");
	return 0;

/*err_kthread:     //bernard 20150915
	free_irq(spi->irq, gf_dev);
	*/
err_hw_initial:
	input_unregister_device(gf_dev->input);
err_input:
	kfree(gf_dev->buffer);
err_alloc_buffer:
    list_del(&gf_dev->device_entry);
err_device:
	device_destroy(gf_spi_class, gf_dev->devt);
	sysfs_remove_group(&spi->dev.kobj, &gf_debug_attr_group);
err_sysfs:
	sensor_power_onoff(false);
	gpio_free(gf_dev->irq_gpio);
	gpio_free(gf_dev->rst_gpio);
	kfree(gf_dev);
    FUNC_EXIT();
    return status;
}



static int gf_remove(struct spi_device *spi)
{
    struct gf_dev *gf_dev = spi_get_drvdata(spi);
    FUNC_ENTRY();


    mcu_disable(gf_dev);
    gf_spi_write_word(gf_dev, 0x0B05, 0xAAAA);
    mcu_enable(gf_dev);

 /*   if (fpthread){         //bernard 20150911
	gf_debug(DEFAULT_DEBUG, "stop fpthread\n");
	read_flag = 0;
	printk("gf: %s   read_flag = %d\n", __func__, read_flag);
	kthread_stop(fpthread);  
	fpthread = NULL;
    }
*/
    /* make sure ops on existing fds can abort cleanly */
    if(gf_dev->spi->irq) {
	free_irq(gf_dev->spi->irq, gf_dev);
    }



    spin_lock_irq(&gf_dev->spi_lock);
    gf_dev->spi = NULL;
    spi_set_drvdata(spi, NULL);
    spin_unlock_irq(&gf_dev->spi_lock);
    
    del_timer_sync(&gf_dev->gf_timer);
    cancel_work_sync(&gf_dev->spi_work);
    
    /*
       if(gf_dev->spi_wq != NULL) {
       flush_workqueue(gf_dev->spi_wq);
       destroy_workqueue(gf_dev->spi_wq);
       }
     */
    /* prevent new opens */
    mutex_lock(&device_list_lock);
    sysfs_remove_group(&spi->dev.kobj, &gf_debug_attr_group);
    list_del(&gf_dev->device_entry);
    device_destroy(gf_spi_class, gf_dev->devt);
    clear_bit(MINOR(gf_dev->devt), minors);
	sensor_power_onoff(false);
	gpio_free(gf_dev->irq_gpio);
	gpio_free(gf_dev->rst_gpio);
    if (gf_dev->users == 0) {
	if(gf_dev->input != NULL)
	    input_unregister_device(gf_dev->input);

	if(gf_dev->buffer != NULL)
	    kfree(gf_dev->buffer);
	kfree(gf_dev);
    }
    mutex_unlock(&device_list_lock);

    FUNC_EXIT();
    return 0;
}

static int gf_suspend_test(struct device *dev)
{
  //  struct spi_device *spi = to_spi_device(dev); 
  //  struct gf_dev *gf_dev = spi_get_drvdata(spi);
	atomic_set(&isSleep , 1);
    printk(KERN_ERR"gf_suspend_test.\n");
    g_debug |= SUSPEND_DEBUG;

    return 0;
}

static int gf_resume_test(struct device *dev)
{
   // struct spi_device *spi = to_spi_device(dev); 
  //  struct gf_dev *gf_dev = spi_get_drvdata(spi);

    printk(KERN_ERR"gf_resume_test.\n");
    g_debug &= ~SUSPEND_DEBUG;

    //gf_hw_initial(gf_dev);
    //printk("%s %d \n", __func__, __LINE__);
	
    atomic_set(&isSleep , 0);
    
    return 0;
}
static const struct dev_pm_ops gf_pm = {
    .suspend = gf_suspend_test,
    .resume = gf_resume_test
};

static struct of_device_id gf_match_table[] = {
    { .compatible = "gf,gf316",},
    { }, 
};

static struct spi_driver gf_spi_driver = {
    .driver = {
	.name =		SPI_DEV_NAME,
	.owner =	THIS_MODULE,
	.pm = &gf_pm,
	.of_match_table = gf_match_table,
    },
    .probe =	gf_probe,
    .remove =	gf_remove,
    //.suspend = gf_suspend_test,
    //.resume = gf_resume_test,

    /* NOTE:  suspend/resume methods are not necessary here.
     * We don't do anything except pass the requests to/from
     * the underlying controller.  The refrigerator handles
     * most issues; the controller driver handles the rest.
     */
};

/*-------------------------------------------------------------------------*/

static int __init gf_init(void)
{
    int status;

    /* Claim our 256 reserved device numbers.  Then register a class
     * that will key udev/mdev to add/remove /dev nodes.  Last, register
     * the driver which manages those device numbers.
     */
    BUG_ON(N_SPI_MINORS > 256);
    status = register_chrdev(SPIDEV_MAJOR, CHRD_DRIVER_NAME, &gf_fops);
    if (status < 0){
	pr_warn("GF:Failed to register char device!\n");
	FUNC_EXIT();
	return status;
    }
    gf_spi_class = class_create(THIS_MODULE, CLASS_NAME);
    if (IS_ERR(gf_spi_class)) {
	unregister_chrdev(SPIDEV_MAJOR, gf_spi_driver.driver.name);
	pr_warn("GF:Failed to create class.\n");
	FUNC_EXIT();
	return PTR_ERR(gf_spi_class);
    }
    status = spi_register_driver(&gf_spi_driver);
    if (status < 0) {
	class_destroy(gf_spi_class);
	unregister_chrdev(SPIDEV_MAJOR, gf_spi_driver.driver.name);
	pr_warn("GF:Failed to register SPI driver.\n");
    }
    return status;
}
module_init(gf_init);

static void __exit gf_exit(void)
{
    spi_unregister_driver(&gf_spi_driver);
    class_destroy(gf_spi_class);
    unregister_chrdev(SPIDEV_MAJOR, gf_spi_driver.driver.name);
}
module_exit(gf_exit);

MODULE_DESCRIPTION("User mode SPI device interface");
MODULE_LICENSE("GPL");
MODULE_ALIAS("spi:gf-spi");
