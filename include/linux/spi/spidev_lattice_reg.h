
#ifndef _SB_REGISTER_ADDR_H_
#define _SB_REGISTER_ADDR_H_

/******************************************************************
@GPIO_INTR : Interrupt to the AP from ice device,
	provide approperiate GPIO pin number.

@CLK_FREQ : Clock frequncy used when transfering data to 
	ice device via SPI

@INPUT_DEVICE_NAME : Its name of the input device created,
	which is used to report an KEY_POWER press event 
	to the OS.

@READ_CLKEN_REG	: if defined, reads the clken register before enabling
	any of its bit(like shake, double tap, IR functionality), and 
	doesn't overwrite the other bits

*******************************************************************/

#include <linux/proc_fs.h>


#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/compat.h>
//#include <mach/rpm-regulator.h>
#include <linux/regulator/rpm-smd-regulator.h>
#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/delay.h>
#include <asm/uaccess.h>
#include <linux/workqueue.h>
#include <linux/irq.h>
#include <linux/of_irq.h>
#include <linux/interrupt.h>
#include <linux/sensors.h>
#include <linux/wakelock.h>
#define GPIO_OFFSET  902
//#define GPIO_INTR 			(GPIO_OFFSET+98)		/* GPIO IRQ */	
//#define GPIO_BLUE_LED 			(GPIO_OFFSET+98)		/* GPIO BLUE LED */	
//add gpio by tony
//#define GPIO_RED_LED  (20+GPIO_OFFSET)
#define GPIO_VCC_1_2  (120+GPIO_OFFSET)
#define GPIO_VCC_1_8  (0+GPIO_OFFSET)
#define GPIO_VCC_3_3  (121+GPIO_OFFSET)
#define GPIO_VCC_IO2  (2+GPIO_OFFSET)
#define GPIO_CDONE    (3+GPIO_OFFSET)
#define GPIO_SPI_CS    (GPIO_OFFSET+10)
//add end
#define CLK_FREQ			1000000		/* CLOCK frequncy for SB tranfer */
#define INPUT_DEVICE_NAME_SMA 		"ice_SB_sma"	/* Input device name*/	
#define INPUT_DEVICE_NAME 		"ice_SB"	/* Input device name*/	
#define READ_CLKEN_REG
#define SHK_SMA_FEATURE_EN		1 /* Enable/Disable shake detect & smart alert*/
#define SMA_FEATURE_EN			1 /* Enable/Disable smart alert */
#define SHK_FEATURE_EN			1 /* Enable/Disable shake detect */
#define PEDOMETER_FEATURE_EN		0 /* Enable/disable pedometer feature*/
//add by tony for pv fastmmi
#define TEST_SUCCESS 0
#define TEST_FAIL -1
#define TEST_CMD_ERR -2
//add end	
#define STEP_COUNT_DEV_NAME 	"stepcounter"
#define ICE_STEP_DEV_NAME 	"ice_step"
#define STEP_DETECT_DEV_NAME 	"stepdetector"
#define SIGNIFICANT_MOTION_DEV_NAME "significant_motion"

/*Sensor Bridge Design register addresses*/
#define COMMAND_REGISTER 	 0x0000
#define STATUS_REGISTER  	 0x0001
#define TX_LENGTH_LB_REG_ADR  	 0x0002
#define TX_LENGTH_HB_REG_ADR  	 0x0003
#define RX_LENGTH_LB_REG_ADR  	 0x0004
#define RX_LENGTH_HB_REG_ADR  	 0x0005
#define CARRIER_DIV_LB_REG_ADR   0x0006
#define CARRIER_DIV_HB_REG_ADR   0x0007
#define SYSTEM_CLK_LB_REG_ADR    0x0008
#define SYSTEM_CLK_HB_REG_ADR    0x0009
#define GLB_RST_REG_ADR		 0x1000
#define FUNC_EN_REG_ADR		 0x1003
#define RST_REG_ADR		 0x1002
#define CLK_EN_REG_ADR		 0x1001
#define INTR_STAT_REG_ADR	 0x1004				 
#define REV_ID_REG_ADR	 	 0x1005
#define LED_CTRL_REG_ADR	 0x1010
#define LED_CYC_LB_REG_ADR	 0x1011
#define LED_CYC_HB_REG_ADR	 0x1012
#define LED_ON_LB_REG_ADR	 0x1013
#define LED_ON_HB_REG_ADR	 0x1014
#define LED_LGHT_CTRL_REG_ADR	 0x1015

#define ACC_EN			 0x04
#define IDLE			0x02
#define APVLD			0x01
#define SHAKE_DETECT_EN		 0x04
#define SHAKE_DETECT_RST	 0x04
#define DOUBLE_TAP_EN		 0x08
#define DOUBLE_TAP_RST		 0x08
#define SMA_DETECT_EN		 0x20
#define SMA_DETECT_RST		 0x20
#define SHAKE_INTR_DETECT	 0x04	
#define SMA_INTR_DETECT		 0x20	
				 
/*Buffer address*/
#define RX_BUFFER 	 	 0x0800
#define TX_BUFFER 	 	 0x0800

/*SPI read/write Operation prefix*/
#define SPI_WRITE 	 	 0x02
#define SPI_READ 	 	 0x03

/*Extract Lower, Higher bytes*/
#define L_BYTE(X)   ((uint8_t) ((X) & 0x00ff))
#define H_BYTE(X)   ((uint8_t) ((X >> 8) & 0x00ff))

/*Pedometer register design */
#define	I2C_RETRY_DELAY		5
#define	I2C_RETRIES		10
#define STEP_COUNTER_REG	0x2000
#define INTR_CTRL_REG		0x03
#define INTR_STATUS_REG		0x1004
#define INTR_MASK_REG		0x1009
#define STEP_DETECT_EN 		0x10
#define SIG_MOTION_EN 		0x40
#define PEDOMETER_EN 		0x10
#define OVER_FLOW 		0x80
#define G_MAX			16000

#define LATTICE_SPIDEV_LOG(fmt, args...) printk("[lattice]:" fmt"\n", ##args) 
#define LATTICE_SPIDEV_ERR(fmt, args...) printk(KERN_ERR"[lattice]" fmt"\n", ##args )

struct spidev_data {
	dev_t			devt;
	spinlock_t		spi_lock;
	struct spi_device	*spi;
	struct list_head	device_entry;

	struct delayed_work input_work;
	/* buffer is NULL unless this device is open (users > 0) */
	struct mutex		buf_lock;
	unsigned		users;
	u8			*buffer;
	u8			*bufferrx;
#if defined(CONFIG_FB)
	struct notifier_block fb_notif;
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	struct early_suspend early_suspend;
#endif
	struct input_dev *input_dev_count;
	struct input_dev *input_dev_step_detect;
	struct input_dev *input_dev_motion;
	struct input_dev *input_dev_detect;
	struct input_dev *input_dev_detect_sma;
	int enabled_detector;
	int enabled_motion;
	int enabled_counter;
	int poll_interval;

	#ifdef GPIO_INTR 
		int irq;
	#endif
	int dummy_counter;
};

int ice_step_register(struct spidev_data *step);
int ice_step_unregister(struct spidev_data *step);
//int create_sysfs_interfaces_pm(struct device *dev);
#endif
