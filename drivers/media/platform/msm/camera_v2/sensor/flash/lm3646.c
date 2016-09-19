/* Copyright (c) 2013-2014, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

 
#include <linux/module.h>
#include <linux/export.h>


//#include <mach/gpiomux.h>

#include "msm_camera_io_util.h"
#include "../cci/msm_cci.h"
#include "msm_led_flash.h"
#include "../../../../../../../arch/arm/mach-msm/include/mach/gpiomux.h"
#define FLASH_NAME "qcom-flash,lm3646"

#define CONFIG_MSMB_CAMERA_DEBUG
#undef CDBG
#ifdef CONFIG_MSMB_CAMERA_DEBUG
#define CDBG(fmt, args...) pr_err(fmt, ##args)
#else
#define CDBG(fmt, args...) do { } while (0)
#endif


//REGISTERS and VALUES define
#define LM3646_DEVICE_ID_REG 0x0
#define LM3646_OUTPUT_MODE_REG 0x01
#define LM3646_TORCH_TIME_REG 0x03
#define LM3646_FLASH_TIME_REG 0x04
#define LM3646_MAX_CURRENT_REG 0x05
#define LM3646_LED1_CURRENT_FLASH_REG 0x06
#define LM3646_LED1_CURRENT_TORCH_REG 0x07

#define LM3646_ID                        0x11  //BIT[5:3]=010,BIT[2:0]=001
#define LM3646_STANDBY_MODE   0x00
#define LM3646_TORCH_MODE       0x02
#define LM3646_FLASH_MODE        0x03


static struct msm_led_flash_ctrl_t fctrl;
static struct i2c_driver lm3646_i2c_driver;


static int msm_flash_lm3646_add_attr(struct platform_device *pdev, struct msm_led_flash_ctrl_t *fctrl);

static void __exit msm_flash_lm3646_i2c_remove(void)
{
	i2c_del_driver(&lm3646_i2c_driver);
	return;
}

static const struct of_device_id lm3646_trigger_dt_match[] = {
	{.compatible = "qcom-flash,lm3646", .data = &fctrl},
	{}
};

MODULE_DEVICE_TABLE(of, lm3646_trigger_dt_match);

static const struct i2c_device_id flash_i2c_id[] = {
	{"qcom-flash,lm3646", (kernel_ulong_t)&fctrl},
	{ }
};

static const struct i2c_device_id lm3646_i2c_id[] = {
	{FLASH_NAME, (kernel_ulong_t)&fctrl},
	{ }
};

static int msm_flash_lm3646_i2c_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	if (!id) {
		pr_err("msm_flash_lm3646_i2c_probe: id is NULL");
		id = lm3646_i2c_id;
	}

	return msm_flash_i2c_probe(client, id);
}

static struct i2c_driver lm3646_i2c_driver = {
	.id_table = lm3646_i2c_id,
	.probe  = msm_flash_lm3646_i2c_probe,
	.remove = __exit_p(msm_flash_lm3646_i2c_remove),
	.driver = {
		.name = FLASH_NAME,
		.owner = THIS_MODULE,
		.of_match_table = lm3646_trigger_dt_match,
	},
};

static int msm_flash_lm3646_platform_probe(struct platform_device *pdev)
{
	const struct of_device_id *match;
    int ret = -1;

	match = of_match_device(lm3646_trigger_dt_match, &pdev->dev);
	if (!match)
		return -EFAULT;

	//return msm_flash_probe(pdev, match->data);
	ret = msm_flash_probe(pdev, match->data);
    if (0 == ret)
    {
        msm_flash_lm3646_add_attr(pdev, (struct msm_led_flash_ctrl_t *)match->data);
    }

    return ret;
}

static struct platform_driver lm3646_platform_driver = {
	.probe = msm_flash_lm3646_platform_probe,
	.driver = {
		.name = "qcom-flash,lm3646",
		.owner = THIS_MODULE,
		.of_match_table = lm3646_trigger_dt_match,
	},
};

static int __init msm_flash_lm3646_init_module(void)
{
	int32_t rc = 0;

	rc = platform_driver_register(&lm3646_platform_driver);
	if (!rc)
		return rc;
	pr_debug("%s:%d rc %d\n", __func__, __LINE__, rc);
	return i2c_add_driver(&lm3646_i2c_driver);
}

static void __exit msm_flash_lm3646_exit_module(void)
{
	if (fctrl.pdev)
		platform_driver_unregister(&lm3646_platform_driver);
	else
		i2c_del_driver(&lm3646_i2c_driver);
}

static int msm_flash_lm3646_led_init(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;
	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;
	CDBG("%s:%d called\n", __func__, __LINE__);

	flashdata = fctrl->flashdata;
	power_info = &flashdata->power_info;
       fctrl->led_state = MSM_CAMERA_LED_RELEASE;//yuxin add 2015.01.07
	if (power_info->gpio_conf->cam_gpiomux_conf_tbl != NULL) {
		pr_err("%s:%d mux install\n", __func__, __LINE__);
		msm_gpiomux_install(
			(struct msm_gpiomux_config *)
			power_info->gpio_conf->cam_gpiomux_conf_tbl,
			power_info->gpio_conf->cam_gpiomux_conf_tbl_size);
	}

	/* CCI Init */
	if (fctrl->flash_device_type == MSM_CAMERA_PLATFORM_DEVICE) {
		rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_util(
			fctrl->flash_i2c_client, MSM_CCI_INIT);
		if (rc < 0)
			pr_err("cci_init failed\n");
	}

	rc = msm_camera_request_gpio_table(
		power_info->gpio_conf->cam_gpio_req_tbl,
		power_info->gpio_conf->cam_gpio_req_tbl_size, 1);
	if (rc < 0) {
		pr_err("%s: request gpio failed\n", __func__);
		return rc;
	}
	msleep(20);

      CDBG("SENSOR_GPIO_FL_EN is %d,SENSOR_GPIO_FL_NOW is %d,SENSOR_GPIO_FL_RESET is %d\n",
	  	power_info->gpio_conf->gpio_num_info->gpio_num[SENSOR_GPIO_FL_EN],
	  	power_info->gpio_conf->gpio_num_info->gpio_num[SENSOR_GPIO_FL_NOW],
	  	power_info->gpio_conf->gpio_num_info->gpio_num[SENSOR_GPIO_FL_RESET]);
	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_EN],
		GPIO_OUT_LOW);

	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_NOW],
		GPIO_OUT_LOW);

      gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_RESET],
		GPIO_OUT_HIGH);
	fctrl->led_state = MSM_CAMERA_LED_INIT;//yuxin add 2015.01.07
  
	return rc;

}

static int msm_flash_lm3646_led_release(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;
	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;

	flashdata = fctrl->flashdata;
	power_info = &flashdata->power_info;
	CDBG("%s:%d called\n", __func__, __LINE__);
	if (!fctrl) {
		pr_err("%s:%d fctrl NULL\n", __func__, __LINE__);
		return -EINVAL;
	}
	//yuxin add 2015.01.07
       if (fctrl->led_state != MSM_CAMERA_LED_INIT) {
		pr_err("%s:%d invalid led state\n", __func__, __LINE__);
		return -EINVAL;
	}
	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_EN],
		GPIO_OUT_LOW);
    
	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_NOW],
		GPIO_OUT_LOW);
	
	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_RESET],
		GPIO_OUT_LOW);

	rc = msm_camera_request_gpio_table(
		power_info->gpio_conf->cam_gpio_req_tbl,
		power_info->gpio_conf->cam_gpio_req_tbl_size, 0);
	if (rc < 0) {
		pr_err("%s: request gpio failed\n", __func__);
		return rc;
	}
	fctrl->led_state = MSM_CAMERA_LED_RELEASE;//yuxin add 2015.01.07

	/* CCI deInit */
	if (fctrl->flash_device_type == MSM_CAMERA_PLATFORM_DEVICE) {
		rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_util(
			fctrl->flash_i2c_client, MSM_CCI_RELEASE);
		if (rc < 0)
			pr_err("cci_deinit failed\n");
	}

	return 0;
}

static int msm_flash_lm3646_led_off(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;
	uint16_t output_mode = 0;

	
	CDBG("%s:%d called\n", __func__, __LINE__);
	if (!fctrl) {
		pr_err("%s:%d fctrl NULL\n", __func__, __LINE__);
		return -EINVAL;
	}
    
	rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_read(fctrl->flash_i2c_client, 
			LM3646_OUTPUT_MODE_REG, &output_mode, MSM_CAMERA_I2C_BYTE_DATA);
        if(rc < 0)
        {
             pr_err("%s read 0x01 register failed, rc = %d \n",__func__, rc);
              return rc;
	}
	
	output_mode &= 0xfc;
	output_mode |= LM3646_STANDBY_MODE;

	rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write(fctrl->flash_i2c_client, 
			LM3646_OUTPUT_MODE_REG, output_mode, MSM_CAMERA_I2C_BYTE_DATA);
	 if(rc < 0)
        {
             pr_err("%s write 0x01 register failed, rc = %d \n",__func__, rc);
              return rc;
	}
	return rc;
}

static int msm_flash_lm3646_led_low(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;
       uint16_t output_mode = 0;

	CDBG("%s:%d called\n", __func__, __LINE__);
      if (!fctrl) {
		pr_err("%s:%d fctrl NULL\n", __func__, __LINE__);
		return -EINVAL;
	}

	/*set torch current*/
	 rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write(fctrl->flash_i2c_client, 
			LM3646_MAX_CURRENT_REG, 0x7f, MSM_CAMERA_I2C_BYTE_DATA); 
	if(rc < 0) {
		pr_err("%s:i2c write MAX_CURRENT_REG error:%d\n", __func__, rc);
		return rc;
	}
	/***   case 1: led1 torch current = 0 if register value set to 0              ***/
	/***           led2 torch current = max torch current                         ***/
	/***   case 2: led1 torch current = 2.53 + 1.46 *(register value - 1)         ***/
	/***           led2 torch current = max torch current - led1 torch current    ***/
	//current_set= 0x10; /* led 1 = 25mA led 2 =  162*/
	rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write(fctrl->flash_i2c_client, 
			// LM3646_LED1_CURRENT_TORCH_REG, 0x3f, MSM_CAMERA_I2C_BYTE_DATA);
            LM3646_LED1_CURRENT_TORCH_REG, 0x6A, MSM_CAMERA_I2C_BYTE_DATA);
	if(rc < 0) {
		pr_err("%s:i2c write LED1_CURRENT_TORCH_REG error:%d\n", __func__, rc);
		return rc;
	}

	/*enable torch mode*/
	rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_read(fctrl->flash_i2c_client, 
			LM3646_OUTPUT_MODE_REG, &output_mode, MSM_CAMERA_I2C_BYTE_DATA);
	if(rc < 0) {
		pr_err("%s:i2c read error:%d\n", __func__, rc);
		return rc;
	}
	
	output_mode &= 0xfc; /* mask the high 6 bits, and clean low 2 bits */
	output_mode |= LM3646_TORCH_MODE; /* set to torch mode */
      
	rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write(fctrl->flash_i2c_client, 
			LM3646_OUTPUT_MODE_REG, output_mode, MSM_CAMERA_I2C_BYTE_DATA);
	if(rc < 0) {
		pr_err("%s:i2c write error:%d\n", __func__, rc);
		return rc;
	}

	return rc;
}

static int msm_flash_lm3646_led_high(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;
       uint16_t output_mode = 0;
       uint16_t timer_set = 0;
	CDBG("%s:%d called\n", __func__, __LINE__);
      if (!fctrl) {
		pr_err("%s:%d fctrl NULL\n", __func__, __LINE__);
		return -EINVAL;
	}

	/*set flash current*/
	 rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write(fctrl->flash_i2c_client, 
			LM3646_MAX_CURRENT_REG, 0x7f, MSM_CAMERA_I2C_BYTE_DATA); 
	if(rc < 0) {
		pr_err("%s:i2c write MAX_CURRENT_REG error:%d\n", __func__, rc);
		return rc;
	}
	/***   case 1: led1 torch current = 0 if register value set to 0              ***/
	/***           led2 torch current = max torch current                         ***/
	/***   case 2: led1 torch current = 2.53 + 1.46 *(register value - 1)         ***/
	/***           led2 torch current = max torch current - led1 torch current    ***/
	//current_set= 0x10; /* led 1 = 25mA led 2 =  162*/
	rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write(fctrl->flash_i2c_client, 
			// LM3646_LED1_CURRENT_FLASH_REG, 0x3f, MSM_CAMERA_I2C_BYTE_DATA);
            LM3646_LED1_CURRENT_TORCH_REG, 0x6A, MSM_CAMERA_I2C_BYTE_DATA);
	if(rc < 0) {
		pr_err("%s:i2c write LED1_CURRENT_TORCH_REG error:%d\n", __func__, rc);
		return rc;
	}

	 /*set flash timer*/
      rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_read(fctrl->flash_i2c_client, 
			LM3646_FLASH_TIME_REG, &timer_set, MSM_CAMERA_I2C_BYTE_DATA);
	if(rc < 0) {
		pr_err("%s:i2c read error:%d\n", __func__, rc);
		return rc;
	}

	timer_set &= 0xf8; /* mask the high 5 bits, and clean low 3 bits */
	timer_set |= 0x07; /* set flash time-out 400ms */
	
	rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write(fctrl->flash_i2c_client, 
			LM3646_FLASH_TIME_REG, timer_set , MSM_CAMERA_I2C_BYTE_DATA);
	if(rc < 0) {
		pr_err("%s:i2c write error:%d\n", __func__, rc);
		return rc;
	}
	
	/*enable flash mode*/
	rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_read(fctrl->flash_i2c_client, 
			LM3646_OUTPUT_MODE_REG, &output_mode, MSM_CAMERA_I2C_BYTE_DATA);
	if(rc < 0) {
		pr_err("%s:i2c read error:%d\n", __func__, rc);
		return rc;
	}
	
	output_mode &= 0xfc; /* mask the high 6 bits, and clean low 2 bits */
	output_mode |= LM3646_FLASH_MODE; /* set to flash mode */
      
	rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write(fctrl->flash_i2c_client, 
			LM3646_OUTPUT_MODE_REG, output_mode, MSM_CAMERA_I2C_BYTE_DATA);
	if(rc < 0) {
		pr_err("%s:i2c write error:%d\n", __func__, rc);
		return rc;
	}

	return rc;
}

#define FASTMMI_TEST_FLASH_ON      1
#define FASTMMI_TEST_FLASH_OFF     0

unsigned long g_flag1 = 0xFF;

static ssize_t
show_led_status(struct device *dev, struct device_attribute *attr,
		char *buf)
{
    int ret = 0;

    sprintf(buf, "%ld\n", g_flag1);
	ret = strlen(buf) + 1;

	return ret;
}

static ssize_t
write_led_status(struct device *dev, struct device_attribute *attr,
 const char *buf, size_t count)
{
	int rc = -1;
	struct msm_led_flash_ctrl_t *fctrl = dev_get_drvdata(dev);
    rc = kstrtoul(buf, 10, &g_flag1);
    if (rc)
		return rc;

    if (FASTMMI_TEST_FLASH_ON == g_flag1) {
        // init
        msm_flash_lm3646_led_init(fctrl);

        // off
    	msm_flash_lm3646_led_off(fctrl);

        // low
        msm_flash_lm3646_led_low(fctrl);
    }else if (FASTMMI_TEST_FLASH_OFF == g_flag1) {
        // off
    	msm_flash_lm3646_led_off(fctrl);

        // release
        msm_flash_lm3646_led_release(fctrl);
    }

	return count;
}

static DEVICE_ATTR(led_onoff, 0664, show_led_status, write_led_status);

static struct attribute *dev_attrs[] = {
	&dev_attr_led_onoff.attr,
	NULL,
};

static struct attribute_group dev_attr_grp = {
	.attrs = dev_attrs,
};

static int msm_flash_lm3646_add_attr(struct platform_device *pdev, struct msm_led_flash_ctrl_t *fctrl)
{
	int ret = -1;

    if (NULL != pdev && NULL != fctrl)
    {
        pr_err("wangjunfeng pdev->name=%s   %s\n",pdev->name,pdev->dev.kobj.name);
        ret = sysfs_create_group(&pdev->dev.kobj, &dev_attr_grp);
        if (ret)
            pr_err("%s: Failed to create sysfs node: %d\n", __func__, ret);

        dev_set_drvdata(&pdev->dev, fctrl);
        //pr_err("wangjunfeng s_ctrl = %d\n", (int)fctrl);
    }

    return ret;
}

static struct msm_camera_i2c_client lm3646_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
};


static struct msm_flash_fn_t lm3646_func_tbl = {
	.flash_get_subdev_id = msm_led_i2c_trigger_get_subdev_id,
	.flash_led_config = msm_led_i2c_trigger_config,
	.flash_led_init = msm_flash_lm3646_led_init,
	.flash_led_release = msm_flash_lm3646_led_release,
	.flash_led_off = msm_flash_lm3646_led_off,
	.flash_led_low = msm_flash_lm3646_led_low,
	.flash_led_high = msm_flash_lm3646_led_high,
};

static struct msm_led_flash_ctrl_t fctrl = {
	.flash_i2c_client = &lm3646_i2c_client,
	//.reg_setting = &lm3646_regs,
	.func_tbl = &lm3646_func_tbl,
};

/*subsys_initcall(msm_flash_i2c_add_driver);*/
module_init(msm_flash_lm3646_init_module);
module_exit(msm_flash_lm3646_exit_module);
MODULE_DESCRIPTION("lm3646 FLASH");
MODULE_LICENSE("GPL v2");

 
