/******************************/
/*yuxin create this file for T4k35 sensor OTP code 2015.09.18 ++*/

#include "t4k35_otp.h"

/* Logging macro */
#undef CDBG
#define CDBG(fmt, args...) pr_debug(fmt, ##args)

#define T4k35_OTP_PSEL 0x3502
#define T4k35_OTP_CTRL 0x3500
#define T4k35_OTP_DATA_BEGIN_ADDR 0x3504
#define T4k35_OTP_DATA_END_ADDR 0x3543

#define MAX_RW_TRY_TIMES  3 //I2c read or write try times 


static uint16_t t4k35_r_golden_value=0x50;
static uint16_t t4k35_g_golden_value=0x90;
static uint16_t t4k35_b_golden_value=0x5D;
static uint16_t t4k35_af_macro_pos=500;
static uint16_t t4k35_af_inifity_pos=100;



 int32_t t4k35_sensor_i2c_read(struct msm_sensor_ctrl_t *s_ctrl,
	uint16_t addr, uint16_t *data,
	enum msm_camera_i2c_data_type data_type)
{
    int32_t rc = 0;
    int i=0;
    for(i=0;i<MAX_RW_TRY_TIMES;i++)
    {
	rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
	                           s_ctrl->sensor_i2c_client,addr,data, data_type);
	if(rc==0)
	        return rc;
   }
    pr_err("%s:read otp data failed,addr is 0x%x\n",__func__,addr);
    return rc;
}
 int32_t t4k35_sensor_i2c_write(struct msm_sensor_ctrl_t *s_ctrl,
	uint16_t addr, uint16_t data,
	enum msm_camera_i2c_data_type data_type)
{
    int32_t rc = 0;
     int i=0;
    for(i=0;i<MAX_RW_TRY_TIMES;i++)
    {
     rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
	 	                    s_ctrl->sensor_i2c_client, addr, data, data_type);
	if(rc==0)
	        return rc;
    }
	pr_err("%s:write otp data failed,addr is 0x%x\n",__func__,addr);
	return rc;
}



 void t4k35_otp_set_page(struct msm_sensor_ctrl_t *s_ctrl, uint16_t page)
{
    SET_T4k35_REG(T4k35_OTP_PSEL, page);
}
 void t4k35_otp_access(struct msm_sensor_ctrl_t *s_ctrl)
{
	uint16_t reg_val;
	GET_T4k35_REG(T4k35_OTP_CTRL, reg_val);
	SET_T4k35_REG(T4k35_OTP_CTRL, reg_val | 0x80);
	usleep(30);
}
 void t4k35_otp_read_enble(struct msm_sensor_ctrl_t *s_ctrl, uint8_t enble)
{
    if (enble)
        SET_T4k35_REG(T4k35_OTP_CTRL, 0x01);
    else
        SET_T4k35_REG(T4k35_OTP_CTRL, 0x00);
}
 int32_t t4k35_otp_read_data(struct msm_sensor_ctrl_t *s_ctrl, uint16_t* otp_data)
{
    uint16_t i = 0;
    for (i = 0; i <= (T4k35_OTP_DATA_END_ADDR - T4k35_OTP_DATA_BEGIN_ADDR); i++)
	{
        GET_T4k35_REG(T4k35_OTP_DATA_BEGIN_ADDR+i,otp_data[i]);
    }
    return 0;
}

 int32_t t4k35_otp_read_module_info(struct msm_sensor_ctrl_t *s_ctrl, struct t4k35_otp_struct *p_otp)
{
  int i,pos;
  uint16_t t4k35_otp_data[T4k35_OTP_DATA_END_ADDR - T4k35_OTP_DATA_BEGIN_ADDR + 1] = {0x00};
  uint16_t t4k35_otp_data_backup[T4k35_OTP_DATA_END_ADDR - T4k35_OTP_DATA_BEGIN_ADDR + 1] = {0x00};
  uint16_t check_sum=0x00;

  
  t4k35_otp_read_enble(s_ctrl, 1);
  t4k35_otp_set_page(s_ctrl, 4);
  t4k35_otp_access(s_ctrl);
  CDBG("data area data:\n");
  t4k35_otp_read_data(s_ctrl, t4k35_otp_data);
  t4k35_otp_set_page(s_ctrl, 10);
  t4k35_otp_access(s_ctrl);
  t4k35_otp_read_data(s_ctrl, t4k35_otp_data_backup);
  t4k35_otp_read_enble(s_ctrl, 0);		
  for(i = 0; i < 64; i++) 
  {
	  t4k35_otp_data[i]=t4k35_otp_data[i]|t4k35_otp_data_backup[i];
  }
  if(t4k35_otp_data[32])  //the  programmed flag of the 2nd time program is enabled 
  {
	  pos=32;//use the 2nd module ID data
  }
  else if(t4k35_otp_data[0])
  {
  	  pos=0;//use the 1st module ID data
  }
  else
  {
  	  CDBG("otp no module information!\n");
  	  return -1;
  }
  for(i = pos+2; i <pos+32; i++) 
  {
     check_sum=check_sum+t4k35_otp_data[i];
  }
  if((check_sum&0xFF)==t4k35_otp_data[pos+1])
  {
	p_otp->Module_Info.module_vendor_id= t4k35_otp_data[pos + 6];
	p_otp->Module_Info.lens_id= t4k35_otp_data[pos + 10];
	p_otp->Module_Info.vcm_vendor_id= t4k35_otp_data[pos + 8];
	p_otp->Module_Info.vcm_driverIC_id= t4k35_otp_data[pos + 9];

      CDBG("%s:module id is %d,lens_id is %d,vcm_vendor_id is %d,vcm_driverIC is %d\n",__func__,
	  	             p_otp->Module_Info.module_vendor_id,p_otp->Module_Info.lens_id,p_otp->Module_Info.vcm_vendor_id,p_otp->Module_Info.vcm_driverIC_id);
	
	if((t4k35_otp_data[pos+15]==0x00)&&(t4k35_otp_data[pos+16]==0x00)
		&&(t4k35_otp_data[pos+17]==0x00)&&(t4k35_otp_data[pos+18]==0x00)
		&&(t4k35_otp_data[pos+19]==0x00)&&(t4k35_otp_data[pos+20]==0x00)
		&&(t4k35_otp_data[pos+21]==0x00)&&(t4k35_otp_data[pos+22]==0x00))
		return 0;
	
	t4k35_r_golden_value=t4k35_otp_data[pos+16]+(t4k35_otp_data[pos+15]<<8);
	t4k35_g_golden_value=(t4k35_otp_data[pos+18]+(t4k35_otp_data[pos+17]<<8)+t4k35_otp_data[pos+20]+(t4k35_otp_data[pos+19]<<8))/2;
	t4k35_b_golden_value=t4k35_otp_data[pos+22]+(t4k35_otp_data[pos+21]<<8);
	return 0;
  }
  else
  {
	CDBG("otp module info checksum error!\n");
	return -1;
  }
}

 int32_t t4k35_otp_read_lsc_awb(struct msm_sensor_ctrl_t *s_ctrl, struct t4k35_otp_struct *p_otp)
{
  int i,j;
  uint16_t t4k35_otp_data[T4k35_OTP_DATA_END_ADDR - T4k35_OTP_DATA_BEGIN_ADDR + 1] = {0x00};
  uint16_t t4k35_otp_data_backup[T4k35_OTP_DATA_END_ADDR - T4k35_OTP_DATA_BEGIN_ADDR + 1] = {0x00};
  uint16_t check_sum=0x00;
  for(i = 3; i >= 0; i--) 
  {
       check_sum=0x00;//clear the checksum
	t4k35_otp_read_enble(s_ctrl, 1);
	t4k35_otp_set_page(s_ctrl, i);
	t4k35_otp_access(s_ctrl);
	CDBG("otp lsc data area data: %d \n", i);
	t4k35_otp_read_data(s_ctrl, t4k35_otp_data);
	CDBG("otp lsc backup area data: %d\n", i + 6);
	t4k35_otp_set_page(s_ctrl, i+6);
	t4k35_otp_access(s_ctrl);
	t4k35_otp_read_data(s_ctrl, t4k35_otp_data_backup);
	t4k35_otp_read_enble(s_ctrl, 0);
	for(j = 0; j < 64; j++) 
	{
		t4k35_otp_data[j]=t4k35_otp_data[j]|t4k35_otp_data_backup[j];
	}
	if (0 == t4k35_otp_data[0]) 
	{
		//if the program flag is 0,the page is not programmed,read the next page
		continue;
	} else {
	  for(j = 2; j < 64; j++) 
	  {
            check_sum=check_sum+t4k35_otp_data[j];
         }
	  if((check_sum & 0xFF) == t4k35_otp_data[1])
	  {
	  	CDBG("otp lsc checksum ok!\n");
		for(j=3;j<=55;j++)
		{
			p_otp->LSC[j-3]=t4k35_otp_data[j];
		}
		for(j=56;j<=63;j++)
		{
			p_otp->AWB[j-56]=t4k35_otp_data[j];
		}
		return 0;
	  }
	  else
	  {
		CDBG("otp lsc checksum error!\n");
		return -1;
	  }
    }
  }
  if (i < 0) 
  {
    return -1;
    CDBG("No otp lsc data on sensor t4k35\n");
  }
  else 
  {
    return 0;
  }
}

 int32_t t4k35_otp_read_af(struct msm_sensor_ctrl_t *s_ctrl,struct t4k35_otp_struct *p_otp)
{
  int i,pos;
  uint16_t t4k35_otp_data[T4k35_OTP_DATA_END_ADDR - T4k35_OTP_DATA_BEGIN_ADDR + 1] = {0x00};
  uint16_t t4k35_otp_data_backup[T4k35_OTP_DATA_END_ADDR - T4k35_OTP_DATA_BEGIN_ADDR + 1] = {0x00};
  uint16_t check_sum=0x00;
  
  t4k35_otp_read_enble(s_ctrl, 1);
  t4k35_otp_set_page(s_ctrl, 5);
  t4k35_otp_access(s_ctrl);
  CDBG("data area data:\n");
  t4k35_otp_read_data(s_ctrl, t4k35_otp_data);
  t4k35_otp_set_page(s_ctrl, 11);
  t4k35_otp_access(s_ctrl);
  t4k35_otp_read_data(s_ctrl, t4k35_otp_data_backup);
  t4k35_otp_read_enble(s_ctrl, 0);		
  for(i = 0; i < 64; i++) 
  {
	  t4k35_otp_data[i]=t4k35_otp_data[i]|t4k35_otp_data_backup[i];
  }
  if(t4k35_otp_data[24])
  {
	  pos=24;
  }
  else if(t4k35_otp_data[16])
  {
  	  pos=16;
  }
  else if(t4k35_otp_data[8])
  {
  	  pos=8;
  }
  else if(t4k35_otp_data[0])
  {
  	  pos=0;
  }
  else
  {
  	  CDBG("no otp macro AF information!\n");
  	  return -1;
  }
  check_sum=0x00;
  for(i = pos+2; i <pos+8; i++) 
  {
     check_sum=check_sum+t4k35_otp_data[i];
  }
  if((check_sum&0xFF)==t4k35_otp_data[pos+1])
  {
	  CDBG("otp macro AF checksum ok!\n");
        p_otp->AF_Macro=(t4k35_otp_data[pos+3]<<8)+t4k35_otp_data[pos+4];
        if(p_otp->AF_Macro==0x00)
        {
           p_otp->AF_Macro=t4k35_af_macro_pos;
        }
  }
  else
  {
	CDBG("otp macro AF checksum error!\n");
       p_otp->AF_Macro=t4k35_af_macro_pos;
  }
  if(t4k35_otp_data[56])
  {
	  pos=56;
  }
  else if(t4k35_otp_data[48])
  {
  	  pos=48;
  }
  else if(t4k35_otp_data[40])
  {
  	  pos=40;
  }
  else if(t4k35_otp_data[32])
  {
  	  pos=32;
  }
  else
  {
  	  CDBG("no otp inifity AF information!\n");
  	  return -1;
  }
  check_sum=0x00;
  for(i = pos+2; i <pos+8; i++) 
  {
     check_sum=check_sum+t4k35_otp_data[i];
  }
  if((check_sum&0xFF)==t4k35_otp_data[pos+1])
  {
	  CDBG("otp inifity AF checksum ok!\n");
        p_otp->AF_Inifity=(t4k35_otp_data[pos+4]<<8)+t4k35_otp_data[pos+5];
        if(p_otp->AF_Inifity==0x00)
        {
           p_otp->AF_Inifity=t4k35_af_inifity_pos;
        }
  }
  else
  {
	CDBG("otp inifity AF checksum error!\n");
       p_otp->AF_Inifity=t4k35_af_inifity_pos;
  }
  return 0;
}

 void t4k35_update_awb(struct msm_sensor_ctrl_t *s_ctrl, struct t4k35_otp_struct *p_otp)
{
  uint16_t rg,bg,r_otp,g_otp,b_otp;
  r_otp=p_otp->AWB[1]+(p_otp->AWB[0]<<8);
  g_otp=(p_otp->AWB[3]+(p_otp->AWB[2]<<8)+p_otp->AWB[5]+(p_otp->AWB[4]<<8))/2;
  b_otp=p_otp->AWB[7]+(p_otp->AWB[6]<<8);
  rg = 256*(t4k35_r_golden_value *g_otp)/(r_otp*t4k35_g_golden_value);
  bg = 256*(t4k35_b_golden_value*g_otp)/(b_otp*t4k35_g_golden_value);
  CDBG("r_golden=0x%x,g_golden=0x%x, b_golden=0x%0x\n", t4k35_r_golden_value,t4k35_g_golden_value,t4k35_b_golden_value);
  CDBG("r_otp=0x%x,g_opt=0x%x, b_otp=0x%0x\n", r_otp,g_otp,b_otp);
  CDBG("rg=0x%x, bg=0x%0x\n", rg,bg);
  SET_T4k35_REG(0x0212, rg >> 8);//0x0212[1:0]:DG_GA_RED[9:8]
  SET_T4k35_REG(0x0213, rg & 0xff);//0x0213[7:0]:DG_GA_RED[7:0]
  SET_T4k35_REG(0x0214, bg >> 8);//0x0214[1:0]:DG_GA_BLUE[9:8]
  SET_T4k35_REG(0x0215, bg & 0xff);//0x0215[7:0]:DG_GA_BLUE[7:0]
}
 void t4k35_update_lsc(struct msm_sensor_ctrl_t *s_ctrl, struct t4k35_otp_struct *p_otp)
{
  uint16_t addr;
  int i;
  addr = 0x323A;
  SET_T4k35_REG(addr, p_otp->LSC[0]);
  addr = 0x323E;
  for(i = 1; i < 53; i++) 
  {
    SET_T4k35_REG(addr++, p_otp->LSC[i]);
  }
  SET_T4k35_REG(0x3237,0x80);
}
int32_t  t4k35_otp_read_setting(struct msm_sensor_ctrl_t *s_ctrl,struct t4k35_otp_struct *p_otp)
{
	int32_t rc = 0;
	rc=t4k35_otp_read_module_info(s_ctrl, p_otp);

	rc |= t4k35_otp_read_lsc_awb(s_ctrl, p_otp);

	rc |=t4k35_otp_read_af(s_ctrl,p_otp);

	if(rc==0x00)
	{
		p_otp->read_completed_flag = 1; 
	}else {
		p_otp->read_completed_flag = 0; 
	}
    return rc;
}

  void t4k35_otp_apply_setting(struct msm_sensor_ctrl_t *s_ctrl,struct t4k35_otp_struct *p_otp)
  {
	 t4k35_update_awb(s_ctrl,p_otp);
	 t4k35_update_lsc(s_ctrl,p_otp);
  }



/*yuxin create this file for T4k35 sensor OTP code 2015.09.18 --*/

