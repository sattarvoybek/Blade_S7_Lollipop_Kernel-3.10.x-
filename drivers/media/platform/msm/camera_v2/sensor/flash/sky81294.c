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
#include "msm_led_flash.h"

//#include <mach/gpiomux.h>
#include "msm_camera_io_util.h"
#include "../cci/msm_cci.h"

#define FLASH_NAME "qcom-flash,sky81294"

#define CONFIG_MSMB_CAMERA_DEBUG
#undef CDBG
#ifdef CONFIG_MSMB_CAMERA_DEBUG
#define CDBG(fmt, args...) pr_err(fmt, ##args)
#else
#define CDBG(fmt, args...) do { } while (0)
#endif

static struct msm_led_flash_ctrl_t fctrl;
static struct i2c_driver sky81294_i2c_driver;

static struct msm_camera_i2c_reg_array sky81294_init_array[] = {
	//{0x00, 0x00},
	//{0x03,0x08},
};

static struct msm_camera_i2c_reg_array sky81294_off_array[] = {
	//{0x0A, 0x00}, modified by wanggang
	{0x03,0x08},
};

static struct msm_camera_i2c_reg_array sky81294_release_array[] = {
	//{0x0A, 0x00},modified by wanggang
	{0x03,0x08},
};

static struct msm_camera_i2c_reg_array sky81294_low_array[] = {
/*
	{0x06,  0x00},
    //{0x09,  0x3f},
    //{0x0A,  0x22},
    {0x09,  0x1a},
    {0x0A,  0x02}, modified by wanggang */
{0x02,0x03},
{0x03,0x09},
    
};

static struct msm_camera_i2c_reg_array sky81294_high_array[] = {
/*
	{0x08,  0x17},
    //{0x09,  0x3f},
    //{0x0A,  0x23},
    {0x09,  0x1a},
    {0x0A,  0x03}, modified by wanggang */
	//{0x00,0x0F},
	{0x00,0x13},
	{0x03,0x0A},

};

static int msm_flash_sky81294_add_attr(struct platform_device *pdev, struct msm_led_flash_ctrl_t *fctrl);

static void __exit msm_flash_sky81294_i2c_remove(void)
{
	i2c_del_driver(&sky81294_i2c_driver);
	return;
}

static const struct of_device_id sky81294_trigger_dt_match[] = {
	{.compatible = "qcom-flash,sky81294", .data = &fctrl},
	{}
};

MODULE_DEVICE_TABLE(of, sky81294_trigger_dt_match);

static const struct i2c_device_id flash_i2c_id[] = {
	{"qcom-flash,sky81294", (kernel_ulong_t)&fctrl},
	{ }
};

static const struct i2c_device_id sky81294_i2c_id[] = {
	{FLASH_NAME, (kernel_ulong_t)&fctrl},
	{ }
};

static int msm_flash_sky81294_i2c_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	if (!id) {
		pr_err("msm_flash_sky81294_i2c_probe: id is NULL");
		id = sky81294_i2c_id;
	}

	return msm_flash_i2c_probe(client, id);
}

static struct i2c_driver sky81294_i2c_driver = {
	.id_table = sky81294_i2c_id,
	.probe  = msm_flash_sky81294_i2c_probe,
	.remove = __exit_p(msm_flash_sky81294_i2c_remove),
	.driver = {
		.name = FLASH_NAME,
		.owner = THIS_MODULE,
		.of_match_table = sky81294_trigger_dt_match,
	},
};

static int msm_flash_sky81294_platform_probe(struct platform_device *pdev)
{
	const struct of_device_id *match;
    int ret = -1;

	match = of_match_device(sky81294_trigger_dt_match, &pdev->dev);
	if (!match)
		return -EFAULT;

	//return msm_flash_probe(pdev, match->data);
	ret = msm_flash_probe(pdev, match->data);
    if (0 == ret)
    {
        msm_flash_sky81294_add_attr(pdev, (struct msm_led_flash_ctrl_t *)match->data);
    }

    return ret;
}

static struct platform_driver sky81294_platform_driver = {
	.probe = msm_flash_sky81294_platform_probe,
	.driver = {
		.name = "qcom-flash,sky81294",
		.owner = THIS_MODULE,
		.of_match_table = sky81294_trigger_dt_match,
	},
};

static int __init msm_flash_sky81294_init_module(void)
{
	int32_t rc = 0;

	rc = platform_driver_register(&sky81294_platform_driver);
	if (!rc)
		return rc;
	pr_debug("%s:%d rc %d\n", __func__, __LINE__, rc);
	return i2c_add_driver(&sky81294_i2c_driver);
}

static void __exit msm_flash_sky81294_exit_module(void)
{
	if (fctrl.pdev)
		platform_driver_unregister(&sky81294_platform_driver);
	else
		i2c_del_driver(&sky81294_i2c_driver);
}

static int msm_flash_sky81294_led_init(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0, ret = 0;
       uint16_t flag_reg = 0x0;
	struct msm_camera_sensor_board_info *flashdata = NULL;
	//struct msm_camera_power_ctrl_t *power_info = NULL;
	CDBG("%s:%d called\n", __func__, __LINE__);

	flashdata = fctrl->flashdata;
	#if 0
	power_info = &flashdata->power_info;
	if (power_info->gpio_conf->cam_gpiomux_conf_tbl != NULL) {
		pr_err("%s:%d mux install\n", __func__, __LINE__);
		msm_gpiomux_install(
			(struct msm_gpiomux_config *)
			power_info->gpio_conf->cam_gpiomux_conf_tbl,
			power_info->gpio_conf->cam_gpiomux_conf_tbl_size);
	}

	

	rc = msm_camera_request_gpio_table(
		power_info->gpio_conf->cam_gpio_req_tbl,
		power_info->gpio_conf->cam_gpio_req_tbl_size, 1);
	if (rc < 0) {
		pr_err("%s: request gpio failed\n", __func__);
		return rc;
	}
	msleep(20);

	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_EN],
		GPIO_OUT_LOW);

	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_NOW],
		GPIO_OUT_LOW);
	#endif

     /* CCI Init */
	if (fctrl->flash_device_type == MSM_CAMERA_PLATFORM_DEVICE) {
		rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_util(
			fctrl->flash_i2c_client, MSM_CCI_INIT);
		if (rc < 0)
			pr_err("cci_init failed\n");
	}
        ret = fctrl->flash_i2c_client->i2c_func_tbl->i2c_read(fctrl->flash_i2c_client, 
                0x0B, &flag_reg , MSM_CAMERA_I2C_BYTE_DATA);
      
         if(ret < 0)
             pr_err("%s read flag register failed, rc = %d \n",__func__, rc);
         
         pr_err("%s read flag register = %d \n",__func__, flag_reg);
    
	return rc;
}

static int msm_flash_sky81294_led_release(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;
	struct msm_camera_sensor_board_info *flashdata = NULL;
	//struct msm_camera_power_ctrl_t *power_info = NULL;

	flashdata = fctrl->flashdata;
	#if 0
	power_info = &flashdata->power_info;
	CDBG("%s:%d called\n", __func__, __LINE__);
	if (!fctrl) {
		pr_err("%s:%d fctrl NULL\n", __func__, __LINE__);
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

	rc = msm_camera_request_gpio_table(
		power_info->gpio_conf->cam_gpio_req_tbl,
		power_info->gpio_conf->cam_gpio_req_tbl_size, 0);
	if (rc < 0) {
		pr_err("%s: request gpio failed\n", __func__);
		return rc;
	}
    #endif
	
	/* CCI deInit */
	if (fctrl->flash_device_type == MSM_CAMERA_PLATFORM_DEVICE) {
		rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_util(
			fctrl->flash_i2c_client, MSM_CCI_RELEASE);
		if (rc < 0)
			pr_err("cci_deinit failed\n");
	}

	return 0;
}

static int msm_flash_sky81294_led_off(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;
	struct msm_camera_sensor_board_info *flashdata = NULL;
	//struct msm_camera_power_ctrl_t *power_info = NULL;

	flashdata = fctrl->flashdata;
	//power_info = &flashdata->power_info;
	CDBG("%s:%d called\n", __func__, __LINE__);
	if (!fctrl) {
		pr_err("%s:%d fctrl NULL\n", __func__, __LINE__);
		return -EINVAL;
	}
    
	if (fctrl->flash_i2c_client && fctrl->reg_setting) {
		rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write_table(
			fctrl->flash_i2c_client,
			fctrl->reg_setting->off_setting);
		if (rc < 0)
			pr_err("%s:%d failed\n", __func__, __LINE__);
	}
	
     #if 0
	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_EN],
		GPIO_OUT_LOW);
    
	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_NOW],
		GPIO_OUT_LOW);
    #endif
	
	return rc;
}

static int msm_flash_sky81294_led_low(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;
	struct msm_camera_sensor_board_info *flashdata = NULL;
	//struct msm_camera_power_ctrl_t *power_info = NULL;
	CDBG("%s:%d called\n", __func__, __LINE__);

	flashdata = fctrl->flashdata;
	#if 0
	power_info = &flashdata->power_info;

    gpio_set_value_cansleep(
        power_info->gpio_conf->gpio_num_info->
        gpio_num[SENSOR_GPIO_FL_EN],
        GPIO_OUT_LOW);

    gpio_set_value_cansleep(
        power_info->gpio_conf->gpio_num_info->
        gpio_num[SENSOR_GPIO_FL_NOW],
        //GPIO_OUT_HIGH);
        GPIO_OUT_LOW);   //not use gpio contol
    #endif

	if (fctrl->flash_i2c_client && fctrl->reg_setting) {
		rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write_table(
			fctrl->flash_i2c_client,
			fctrl->reg_setting->low_setting);
		if (rc < 0)
			pr_err("%s:%d failed\n", __func__, __LINE__);
	}

	return rc;
}

static int msm_flash_sky81294_led_high(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;
	struct msm_camera_sensor_board_info *flashdata = NULL;
	//struct msm_camera_power_ctrl_t *power_info = NULL;
	CDBG("%s:%d called\n", __func__, __LINE__);

	flashdata = fctrl->flashdata;
	#if 0
	power_info = &flashdata->power_info;

    gpio_set_value_cansleep(
        power_info->gpio_conf->gpio_num_info->
        gpio_num[SENSOR_GPIO_FL_EN],
        //GPIO_OUT_HIGH);
        GPIO_OUT_LOW);   //not use gpio contol

    gpio_set_value_cansleep(
        power_info->gpio_conf->gpio_num_info->
        gpio_num[SENSOR_GPIO_FL_NOW],
        GPIO_OUT_LOW);
    #endif
	
	if (fctrl->flash_i2c_client && fctrl->reg_setting) {
		rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write_table(
			fctrl->flash_i2c_client,
			fctrl->reg_setting->high_setting);
		if (rc < 0)
			pr_err("%s:%d failed\n", __func__, __LINE__);
	}

	return rc;
}

#define FASTMMI_TEST_FLASH_ON      1
#define FASTMMI_TEST_FLASH_OFF     0

unsigned long g_flag_sky = 0xFF;

static ssize_t
show_led_status(struct device *dev, struct device_attribute *attr,
		char *buf)
{
    int ret = 0;

    sprintf(buf, "%ld\n", g_flag_sky);
	ret = strlen(buf) + 1;

	return ret;
}

static ssize_t
write_led_status(struct device *dev, struct device_attribute *attr,
 const char *buf, size_t count)
{
	int rc = -1;
	struct msm_led_flash_ctrl_t *fctrl = dev_get_drvdata(dev);
    rc = kstrtoul(buf, 10, &g_flag_sky);
    if (rc)
		return rc;

    if (FASTMMI_TEST_FLASH_ON == g_flag_sky) {
        // init
        msm_flash_sky81294_led_init(fctrl);

        // off
    	msm_flash_sky81294_led_off(fctrl);

        // low
        msm_flash_sky81294_led_low(fctrl);
    }else if (FASTMMI_TEST_FLASH_OFF == g_flag_sky) {
        // off
    	msm_flash_sky81294_led_off(fctrl);

        // release
        msm_flash_sky81294_led_release(fctrl);
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

static int msm_flash_sky81294_add_attr(struct platform_device *pdev, struct msm_led_flash_ctrl_t *fctrl)
{
	int ret = -1;

    if (NULL != pdev && NULL != fctrl)
    {
        pr_err("wangjunfeng pdev->name=%s   %s\n",pdev->name,pdev->dev.kobj.name);
        ret = sysfs_create_group(&pdev->dev.kobj, &dev_attr_grp);
        if (ret)
            pr_err("%s: Failed to create sysfs node: %d\n", __func__, ret);

        dev_set_drvdata(&pdev->dev, fctrl);
        pr_err("wangjunfeng s_ctrl = %d\n", (int)fctrl);
    }

    return ret;
}


static struct msm_camera_i2c_client sky81294_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
};

static struct msm_camera_i2c_reg_setting sky81294_init_setting = {
	.reg_setting = sky81294_init_array,
	.size = ARRAY_SIZE(sky81294_init_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_camera_i2c_reg_setting sky81294_off_setting = {
	.reg_setting = sky81294_off_array,
	.size = ARRAY_SIZE(sky81294_off_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};


static struct msm_camera_i2c_reg_setting sky81294_release_setting = {
	.reg_setting = sky81294_release_array,
	.size = ARRAY_SIZE(sky81294_release_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_camera_i2c_reg_setting sky81294_low_setting = {
	.reg_setting = sky81294_low_array,
	.size = ARRAY_SIZE(sky81294_low_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_camera_i2c_reg_setting sky81294_high_setting = {
	.reg_setting = sky81294_high_array,
	.size = ARRAY_SIZE(sky81294_high_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_led_flash_reg_t sky81294_regs = {
	.init_setting = &sky81294_init_setting,
	.off_setting = &sky81294_off_setting,
	.low_setting = &sky81294_low_setting,
	.high_setting = &sky81294_high_setting,
	.release_setting = &sky81294_release_setting,
};

static struct msm_flash_fn_t sky81294_func_tbl = {
	.flash_get_subdev_id = msm_led_i2c_trigger_get_subdev_id,
	.flash_led_config = msm_led_i2c_trigger_config,
	.flash_led_init = msm_flash_sky81294_led_init,
	.flash_led_release = msm_flash_sky81294_led_release,
	.flash_led_off = msm_flash_sky81294_led_off,
	.flash_led_low = msm_flash_sky81294_led_low,
	.flash_led_high = msm_flash_sky81294_led_high,
};

static struct msm_led_flash_ctrl_t fctrl = {
	.flash_i2c_client = &sky81294_i2c_client,
	.reg_setting = &sky81294_regs,
	.func_tbl = &sky81294_func_tbl,
};

/*subsys_initcall(msm_flash_i2c_add_driver);*/
module_init(msm_flash_sky81294_init_module);
module_exit(msm_flash_sky81294_exit_module);
MODULE_DESCRIPTION("sky81294 FLASH");
MODULE_LICENSE("GPL v2");
