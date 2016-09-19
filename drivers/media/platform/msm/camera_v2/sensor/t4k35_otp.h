#ifndef T4K35_OTP_H
#define T4K35_OTP_H
#include "msm_sensor.h"

 

struct otp_module_id_struct
{
  uint8_t year;
  uint8_t month;
  uint8_t date;
  uint8_t module_vendor_id;  
  uint8_t lens_id;
  uint8_t vcm_vendor_id;
  uint8_t vcm_driverIC_id;
};

 struct t4k35_otp_struct 
{
  uint8_t LSC[53];              /* LSC */
  uint8_t AWB[8];               /* AWB */
  struct otp_module_id_struct  Module_Info;
  uint16_t AF_Macro;
  uint16_t AF_Inifity;
  int read_completed_flag;
} ;


#define SET_T4k35_REG(reg, val) t4k35_sensor_i2c_write(s_ctrl,reg,val,MSM_CAMERA_I2C_BYTE_DATA)
#define GET_T4k35_REG(reg, val) t4k35_sensor_i2c_read(s_ctrl,reg,&val,MSM_CAMERA_I2C_BYTE_DATA)

int t4k35_sensor_i2c_read(struct msm_sensor_ctrl_t *s_ctrl,uint16_t addr, uint16_t *data,enum msm_camera_i2c_data_type data_type);
int t4k35_sensor_i2c_write(struct msm_sensor_ctrl_t *s_ctrl,uint16_t addr, uint16_t data,enum msm_camera_i2c_data_type data_type);
void t4k35_otp_set_page(struct msm_sensor_ctrl_t *s_ctrl, uint16_t page);
void t4k35_otp_access(struct msm_sensor_ctrl_t *s_ctrl);
void t4k35_otp_read_enble(struct msm_sensor_ctrl_t *s_ctrl, uint8_t enble);
int32_t t4k35_otp_read_data(struct msm_sensor_ctrl_t *s_ctrl, uint16_t* otp_data);
int32_t t4k35_otp_read_module_info(struct msm_sensor_ctrl_t *s_ctrl, struct t4k35_otp_struct *p_otp);
int32_t t4k35_otp_read_lsc_awb(struct msm_sensor_ctrl_t *s_ctrl, struct t4k35_otp_struct *p_otp);
int32_t t4k35_otp_read_af(struct msm_sensor_ctrl_t *s_ctrl,struct t4k35_otp_struct *p_otp);
void t4k35_update_awb(struct msm_sensor_ctrl_t *s_ctrl, struct t4k35_otp_struct *p_otp);
void t4k35_update_lsc(struct msm_sensor_ctrl_t *s_ctrl, struct t4k35_otp_struct *p_otp);
int32_t  t4k35_otp_read_setting(struct msm_sensor_ctrl_t *s_ctrl,struct t4k35_otp_struct *p_otp);
void t4k35_otp_apply_setting(struct msm_sensor_ctrl_t *s_ctrl,struct t4k35_otp_struct *p_otp);

#endif
