/* Copyright (c) 2012, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/pm.h>
#include <linux/err.h>
#include <linux/of.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/regulator/consumer.h>


struct i2c_client * i2c_tps65132_client = NULL;
int tps65132_set_vsp_vsn_58v(void);
int ti65132b_true =0x0;
int ti65132b_probe =0x0;
#define LCM_VSP_VSN_58V  0x12
#define LCM_VSP_VSN_50V  0x0a
int tps65132_read_reg1(struct i2c_client *client, u8 *buf,  u8 *data)
{
    struct i2c_msg msgs[2];
    int ret=-1;
    int retries = 0;

    msgs[0].flags = !I2C_M_RD;
    msgs[0].addr  = client->addr;
    msgs[0].len   = 1;
    msgs[0].buf   = &buf[0];

    msgs[1].flags = I2C_M_RD;
    msgs[1].addr  = client->addr;
    msgs[1].len   = 2;
    msgs[1].buf   = data;
    while(retries < 5)
    {
        ret = i2c_transfer(client->adapter, msgs, 2);
        if(ret == 2)break;
        retries++;
		msleep_interruptible(5);
    }
	printk("tps65132_read_reg1 retries=%d,ret=%d\n",retries,ret);
 	if (ret != 2) {
		printk("read transfer error\n");
		ret=-1;
    }
 
    return ret;
}
int tps65132_write_reg1(struct i2c_client *client, u8 *buf, int len)
{
	int err;
	int tries = 0;

	struct i2c_msg msgs[] = {
		{
		 .addr = client->addr,
		 .flags = client->flags & I2C_M_TEN,
		 .len = len + 1,
		 .buf = buf,
		 },
	};

	do {
		err = i2c_transfer(client->adapter, msgs, 1);
		if (err != 1)
			msleep_interruptible(5);
	} while ((err != 1) && (++tries < 5));

	if (err != 1) {
		printk("write transfer error\n");
		err = -1;
	}

	return err;
}
/*static int tps65132_read_reg(struct i2c_client *client, u32 reg)
{
	int rc;

	rc = i2c_smbus_read_byte_data(client, reg);
	if (rc < 0)
		dev_err(&client->dev, "i2c reg read for 0x%x failed\n", reg);
	return rc;
}

static int tps65132_write_reg(struct i2c_client *client, u32 reg, u8 val)
{
	int rc;

	rc = i2c_smbus_write_byte_data(client, reg, val);
	if (rc < 0)
		dev_err(&client->dev, "i2c reg write for 0x%xfailed\n", reg);

	return rc;
}
*/
int tps65132_set_vsp_vsn_58v(void)
{
	int rc=0,rc1=0;
	u8 buf_temp[2] = {0x00,0x12};
	u8 buf_temp1[2] = {0x01,0x12};	

	rc=tps65132_write_reg1(i2c_tps65132_client, buf_temp, 1);

	rc1=tps65132_write_reg1(i2c_tps65132_client, buf_temp1, 1);	
	
	printk("tps65132 driver set 5V8 rc=%d rc1=%d \n",rc,rc1);
	return rc;
}

static int tps65132_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
     u8 buf_temp[2] = {0x00,0x12};
     u8 buf_temp1[2] = {0x01,0x12};
     u8 buf_temp2[2] = {0xFF,0x01};//set read data from EEPROM
     u8 buf_temp3[2] = {0xFF,0xF0};//make dat to EEPROM
     int rc=0,rc1=0,rc2=0;
     u8 dat[2] = {0x00,0x00};
	
    i2c_tps65132_client = client;
    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) 
    {
        dev_err(&client->dev, "client not i2c capable\n");
        return -1;
    }

    tps65132_read_reg1(i2c_tps65132_client, &buf_temp[0],&dat[0]);
    if(dat[0]==0xFF && dat[1]==0x24)
    {
            rc=tps65132_write_reg1(i2c_tps65132_client, buf_temp, 1);
            rc1=tps65132_write_reg1(i2c_tps65132_client, buf_temp1, 1);
            printk("driver probe:KTD2151 VPOS & VNEG Register:dat[0]=0x%x dat[1]=0x%x rc=%d rc1=%d\n",dat[0],dat[1],rc,rc1);
    }
    else
    {
            rc=tps65132_write_reg1(i2c_tps65132_client, buf_temp2, 1);
            mdelay(2);
            tps65132_read_reg1(i2c_tps65132_client, &buf_temp[0],&dat[0]);
            printk("TPS65132 VPOS & VNEG Register:dat[0]=0x%x dat[1]=0x%x rc=%d\n",dat[0],dat[1],rc);
            if(dat[0]==0x12 && dat[1]==0x12)
            {
                    printk("TPS65132  VPOS &  VNEG Register already set!\n");
            }
            else
            {
                    rc=tps65132_write_reg1(i2c_tps65132_client, buf_temp, 1);
                    rc1=tps65132_write_reg1(i2c_tps65132_client, buf_temp1, 1);
                    rc2=tps65132_write_reg1(i2c_tps65132_client, buf_temp3, 1);
                    printk("TPS65132 driver probe rc=%d rc1=%d rc2=%d\n",rc,rc1,rc2);
            }
    }
    return 0;
}

static int tps65132_remove(struct i2c_client *client)
{
	kfree(i2c_get_clientdata(client));
	return 0;
}

static const struct i2c_device_id tps65132_id_table[] = {
	{"ti65132b", 0},
	{ },
};
MODULE_DEVICE_TABLE(i2c, tps65132_id_table);

#ifdef CONFIG_OF
static const struct of_device_id tps65132_of_id_table[] = {
	{.compatible = "tps,ti65132b"},
	{ },
};
#else
#define tps65132_of_id_table NULL
#endif

static struct i2c_driver tps65132_i2c_driver = {
	.driver = {
		.name = "ti65132b",
		.owner = THIS_MODULE,
		.of_match_table = tps65132_of_id_table,
	},
	.probe = tps65132_probe,
	.remove = tps65132_remove,
	.id_table = tps65132_id_table,
};

module_i2c_driver(tps65132_i2c_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("tps65132 chip driver");

