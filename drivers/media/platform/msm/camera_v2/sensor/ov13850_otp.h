#ifndef OV13850_OTP_H
#define OV13850_OTP_H
#include "msm_sensor.h"

struct otp_awb_module_struct
{
  uint8_t flag;
  uint8_t module_integrator_id;
  uint8_t lens_id;
  uint8_t production_year;
  uint8_t production_month;
  uint8_t production_day;
  int rg_ratio;
  int bg_ratio;
};
struct otp_lsc_struct
{
  uint8_t flag;
  uint8_t lenc[186];
  uint8_t decoded_lenc[360];
  uint8_t checksumLSC;
  uint8_t checksumOTP;
  uint8_t checksumTotal;
};
struct otp_vcm_struct
{
  uint8_t flag;
  int VCM_start;
  int VCM_end;
  int VCM_dir;
};
struct ov13850_otp_struct
{
    int flag; // bit[7]: info, bit[6]:wb, bit[5]:vcm, bit[4]:lenc
    struct otp_awb_module_struct module_awb_info;
    struct otp_lsc_struct lsc_info;
    struct otp_vcm_struct vcm_info;
};
#define RG_Ratio_Typical 264
#define BG_Ratio_Typical 293
#define SET_OV13850_REG(reg, val) ov13850_sensor_i2c_write(s_ctrl,reg,val,MSM_CAMERA_I2C_BYTE_DATA)
#define GET_OV13850_REG(reg, val) ov13850_sensor_i2c_read(s_ctrl,reg,&val,MSM_CAMERA_I2C_BYTE_DATA)
int ov13850_sensor_i2c_read(struct msm_sensor_ctrl_t *s_ctrl,uint16_t addr, uint16_t *data,enum msm_camera_i2c_data_type data_type);
int ov13850_sensor_i2c_write(struct msm_sensor_ctrl_t *s_ctrl,uint16_t addr, uint16_t data,enum msm_camera_i2c_data_type data_type);
int ov13850_read_module_awb_info(struct msm_sensor_ctrl_t *s_ctrl,struct ov13850_otp_struct *otp_ptr);
int ov13850_read_vcm_info(struct msm_sensor_ctrl_t *s_ctrl,struct ov13850_otp_struct *otp_ptr);
int ov13850_read_lsc_info(struct msm_sensor_ctrl_t *s_ctrl,struct ov13850_otp_struct *otp_ptr);
int ov13850_read_otp(struct msm_sensor_ctrl_t *s_ctrl,struct ov13850_otp_struct *otp_ptr);
int ov13850_apply_awb_otp(struct msm_sensor_ctrl_t *s_ctrl,struct ov13850_otp_struct *otp_ptr);
int ov13850_apply_lsc_otp(struct msm_sensor_ctrl_t *s_ctrl,struct ov13850_otp_struct *otp_ptr);
int ov13850_apply_otp(struct msm_sensor_ctrl_t *s_ctrl,struct ov13850_otp_struct *otp_ptr);
int ov13850_otp_init(struct msm_sensor_ctrl_t *s_ctrl);
void ov13850_LumaDecoder(uint8_t *pData, uint8_t *pPara);
void ov13850_ColorDecoder(uint8_t *pData, uint8_t *pPara);
void ov13850_R2A_LENC_Decoder(uint8_t *pData, uint8_t *pPara);
int32_t ov13850_otp_read_byte(struct msm_sensor_ctrl_t *s_ctrl,uint16_t address,uint8_t* data);
#endif
