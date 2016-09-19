#include "ov13850_otp.h"
#include <linux/delay.h>
struct ov13850_otp_struct g_ov13850_otp_data;
// return value:
// bit[7]: 0 no otp info, 1 valid otp info
// bit[6]: 0 no otp wb, 1 valib otp wb
// bit[5]: 0 no otp vcm, 1 valid otp vcm
// bit[4]: 0 no otp lenc, 1 valid otp lenc
int ov13850_sensor_i2c_read(struct msm_sensor_ctrl_t *s_ctrl,uint16_t addr, uint16_t *data,enum msm_camera_i2c_data_type data_type)
{
    int32_t rc = 0;
    //rc = s_ctrl->i2c_client.i2c_func_tbl->i2c_read(&(s_ctrl->i2c_client),addr,data, data_type);
    rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(s_ctrl->sensor_i2c_client,addr,data, data_type);
	if (rc < 0)
	{
        pr_err("%s,i2c read eroro  addr:0x%x,data:%d,rc:%d\n", __func__,addr,*data,rc);
		msleep(50);
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(s_ctrl->sensor_i2c_client,addr,data, data_type);
	}
    return rc;
}

int ov13850_sensor_i2c_write(struct msm_sensor_ctrl_t *s_ctrl,uint16_t addr, uint16_t data,enum msm_camera_i2c_data_type data_type)
{
    int32_t rc = 0;
    //pr_err("ov13850 i2c write, addr:0x%0x, val:0x%0x\n", addr,data);
    //rc = s_ctrl->i2c_client.i2c_func_tbl->i2c_write(&(s_ctrl->i2c_client),addr,data,data_type);
    rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, addr, data, data_type);
    if (rc < 0) 
    {
        pr_err("ov13850 i2c write failed, addr:0x%0x, val:0x%0x,retry \n", addr,data);
        msleep(100);
        //rc = s_ctrl->i2c_client.i2c_func_tbl->i2c_write(&(s_ctrl->i2c_client),addr,data,data_type);
        rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, addr, data, data_type);
    }
    return rc;
}

int32_t ov13850_otp_read_byte(struct msm_sensor_ctrl_t *s_ctrl,uint16_t address,uint8_t* data)
{
    uint16_t tmp = 0;
	int32_t rc = 0;
    rc=GET_OV13850_REG(address,tmp);
	if(rc<0)
	{
       return rc;
	}
    *data = (uint8_t)(tmp);
    return 0;
}

int ov13850_read_module_awb_info(struct msm_sensor_ctrl_t *s_ctrl,struct ov13850_otp_struct *otp_ptr)
{
    uint16_t flag = 0;
    int addr = 0;
    uint8_t awb_rg_msb =0,awb_bg_msb = 0,awb_lsb = 0;
    ov13850_otp_read_byte(s_ctrl,0x7220,&(otp_ptr->module_awb_info.flag));
    pr_err("%s,group setting:0x%x\n", __func__,otp_ptr->module_awb_info.flag);
    flag = otp_ptr->module_awb_info.flag;
    if((flag & 0xc0) == 0x40)//read group 1
    {
        addr = 0x7221;
    }else if((flag & 0x30) == 0x10)//read group2
    {
        addr = 0x7229;
    }else
    {
        otp_ptr->module_awb_info.flag = 0x00;
        return -1;
    }
    ov13850_otp_read_byte(s_ctrl,addr,&(otp_ptr->module_awb_info.module_integrator_id));
    ov13850_otp_read_byte(s_ctrl,addr+1,&(otp_ptr->module_awb_info.lens_id));
    ov13850_otp_read_byte(s_ctrl,addr+2,&(otp_ptr->module_awb_info.production_year));
    ov13850_otp_read_byte(s_ctrl,addr+3,&(otp_ptr->module_awb_info.production_month));
    ov13850_otp_read_byte(s_ctrl,addr+4,&(otp_ptr->module_awb_info.production_day));
    ov13850_otp_read_byte(s_ctrl,addr+5,&(awb_rg_msb));
    ov13850_otp_read_byte(s_ctrl,addr+6,&(awb_bg_msb));
    ov13850_otp_read_byte(s_ctrl,addr+7,&(awb_lsb));
    otp_ptr->module_awb_info.rg_ratio = ((awb_rg_msb<<2)+((awb_lsb>>6)&0x03));
    otp_ptr->module_awb_info.bg_ratio = ((awb_bg_msb<<2)+((awb_lsb>>4)&0x03));
    pr_err("%s,data:0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x", __func__,otp_ptr->module_awb_info.module_integrator_id,otp_ptr->module_awb_info.lens_id,
    otp_ptr->module_awb_info.production_year,otp_ptr->module_awb_info.production_month,otp_ptr->module_awb_info.production_day,
    awb_rg_msb,awb_bg_msb,awb_lsb);
    return 0;

}
int ov13850_read_vcm_info(struct msm_sensor_ctrl_t *s_ctrl,struct ov13850_otp_struct *otp_ptr)
{
    uint16_t flag = 0;
    int addr = 0;
    uint8_t vcmst_msb = 0,vcmed_msb = 0,vcm_lsb_dir = 0;
    ov13850_otp_read_byte(s_ctrl,0x7220,&(otp_ptr->vcm_info.flag));
    pr_err("%s,group setting:0x%x\n", __func__,otp_ptr->vcm_info.flag);
    flag = otp_ptr->vcm_info.flag;
    addr = 0;
    if((flag & 0xc0) == 0x40)
    {
        addr = 0x73ad; // base address of VCM Calibration group 1
    }
    else if((flag & 0x30) == 0x10)
    {
        addr = 0x73b0; // base address of VCM Calibration group 2
    }
	else
    {
        otp_ptr->vcm_info.flag = 0x00;
        return -1;
    }
    ov13850_otp_read_byte(s_ctrl,addr,&(vcmst_msb));
    ov13850_otp_read_byte(s_ctrl,addr+1,&(vcmed_msb));
    ov13850_otp_read_byte(s_ctrl,addr+2,&(vcm_lsb_dir));
    otp_ptr->vcm_info.VCM_start = (vcmst_msb<<2)|((vcm_lsb_dir>>6) & 0x03);
    otp_ptr->vcm_info.VCM_end = (vcmed_msb << 2)|((vcm_lsb_dir>>4) & 0x03);
    otp_ptr->vcm_info.VCM_dir = (vcm_lsb_dir>>2) & 0x03;
    pr_err("%s,vcm data:0x%x,0x%x,0x%x\n", __func__,vcmst_msb,vcmed_msb,vcm_lsb_dir);
    return 0;
}
int ov13850_read_lsc_info(struct msm_sensor_ctrl_t *s_ctrl,struct ov13850_otp_struct *otp_ptr)
{
    uint16_t flag = 0;
    int addr = 0;
    int checksumLSC = 0, checksumOTP = 0, checksumTotal = 0;
    int i = 0;
    ov13850_otp_read_byte(s_ctrl,0x7231,&(otp_ptr->lsc_info.flag));
    pr_err("%s,group setting:0x%x\n", __func__,otp_ptr->lsc_info.flag);
    flag = otp_ptr->lsc_info.flag;
    if((flag & 0xc0) == 0x40)
    {
        addr = 0x7232; // base address of Lenc Calibration group 1
    }
    else if((flag & 0x30) == 0x10)
    {
        addr = 0x72ef; // base address of Lenc Calibration group 2
    }
	else
    {
        otp_ptr->lsc_info.flag = 0x00;
        return -1;
    }
    for(i=0;i<186;i++)
    {
        ov13850_otp_read_byte(s_ctrl,addr+i,&(otp_ptr->lsc_info.lenc[i]));
        checksumLSC += otp_ptr->lsc_info.lenc[i];
        //pr_err("%s,addr:0x%x,lsc[%d]:0x%x\n", __func__,addr+i,i,otp_ptr->lsc_info.lenc[i]);
    }
    ov13850_otp_read_byte(s_ctrl,addr+186,&(otp_ptr->lsc_info.checksumLSC));
    ov13850_otp_read_byte(s_ctrl,addr+187,&(otp_ptr->lsc_info.checksumOTP));
    ov13850_otp_read_byte(s_ctrl,addr+188,&(otp_ptr->lsc_info.checksumTotal));
    //decoder
    ov13850_R2A_LENC_Decoder(otp_ptr->lsc_info.lenc, otp_ptr->lsc_info.decoded_lenc);
    for(i=0;i<360;i++) 
    {
        checksumOTP += otp_ptr->lsc_info.decoded_lenc[i];
        //pr_err("%s,decoded lsc[%d]:0x%x\n", __func__,i,otp_ptr->lsc_info.decoded_lenc[i]);
    }
    checksumLSC = (checksumLSC)%255 +1;
    checksumOTP = (checksumOTP)%255 +1;
    checksumTotal = (checksumLSC) ^ (checksumOTP);
    pr_err("%s,checksum:lsc:%d,otp:%d,total:%d;lscReg:%d,otpReg:%d,totalReg:%d\n", __func__,
            checksumLSC,checksumOTP,checksumTotal,
            otp_ptr->lsc_info.checksumLSC,otp_ptr->lsc_info.checksumOTP,otp_ptr->lsc_info.checksumTotal);
    if(otp_ptr->lsc_info.checksumLSC == checksumLSC
        &&otp_ptr->lsc_info.checksumOTP== checksumOTP)
    {
        return 0;
    }else if(otp_ptr->lsc_info.checksumTotal == checksumTotal)
    {
        return 0;
    }
    {
        //checksum erro
        pr_err("%s,checksum error\n", __func__);
        return -1;
    }
}

int ov13850_read_otp(struct msm_sensor_ctrl_t *s_ctrl,struct ov13850_otp_struct *otp_ptr)
{
    uint8_t temp1;
    //uint8_t s_id = 0;
    //ov13850_otp_read_byte(s_ctrl,0x300a,&s_id);
    //pr_err("%s,s_id:%d\n", __func__,s_id);
    ov13850_otp_init(s_ctrl);
    ov13850_read_module_awb_info(s_ctrl,otp_ptr);
    ov13850_read_vcm_info(s_ctrl,otp_ptr);
    ov13850_read_lsc_info(s_ctrl,otp_ptr);
    //set 0x5002[1] to "1"
    ov13850_otp_read_byte(s_ctrl,0x5002,&(temp1));
    //SET_OV13850_REG(0x5002,(0x02 & 0x02) | (temp1 & (~0x02)));
    SET_OV13850_REG(0x5002,(temp1 | 0x02));
    SET_OV13850_REG(0x0100, 0x00);
    otp_ptr->flag = 1;
    return 0;
}
// return value:
// bit[7]: 0 no otp info, 1 valid otp info
// bit[6]: 0 no otp wb, 1 valib otp wb
// bit[5]: 0 no otp vcm, 1 valid otp vcm
// bit[4]: 0 no otp lenc, 1 valid otp lenc
int ov13850_apply_awb_otp(struct msm_sensor_ctrl_t *s_ctrl,struct ov13850_otp_struct *otp_ptr)
{
    int rg, bg, R_gain, G_gain, B_gain, Base_gain;
    rg = otp_ptr->module_awb_info.rg_ratio;
    bg = otp_ptr->module_awb_info.bg_ratio;
    //calculate G gain
    R_gain = (RG_Ratio_Typical*1000) / rg;
    B_gain = (BG_Ratio_Typical*1000) / bg;
    G_gain = 1000;
    if (R_gain < 1000 || B_gain < 1000)
    {
        if (R_gain < B_gain)
        {
            Base_gain = R_gain;
        }
        else
        {
            Base_gain = B_gain;
        }
    }
    else
    {
        Base_gain = G_gain;
    }
    R_gain = 0x400 * R_gain / (Base_gain);
    B_gain = 0x400 * B_gain / (Base_gain);
    G_gain = 0x400 * G_gain / (Base_gain);
    // update sensor WB gain
    if (R_gain>0x400)
    {
        SET_OV13850_REG(0x5056, R_gain>>8);
        SET_OV13850_REG(0x5057, R_gain & 0x00ff);
    }
    if (G_gain>0x400)
    {
        SET_OV13850_REG(0x5058, G_gain>>8);
        SET_OV13850_REG(0x5059, G_gain & 0x00ff);
    }
    if (B_gain>0x400)
    {
        SET_OV13850_REG(0x505A, B_gain>>8);
        SET_OV13850_REG(0x505B, B_gain & 0x00ff);
    }
	pr_err("%s\n", __func__);
    return 0;
}
int ov13850_apply_lsc_otp(struct msm_sensor_ctrl_t *s_ctrl,struct ov13850_otp_struct *otp_ptr)
{
    uint8_t temp = 0;
    int i = 0;
    ov13850_otp_read_byte(s_ctrl,0x5000,&(temp));
    temp = 0x01 | temp;
    SET_OV13850_REG(0x5000, temp);
    //Decode the lenc buffer from OTP , from 186 bytes to 360 bytes
    for(i=0;i<360 ;i++) 
    {
        SET_OV13850_REG(0x5200+i,otp_ptr->lsc_info.decoded_lenc[i]);
    }
	pr_err("%s\n", __func__);
    return 0;
}
int ov13850_apply_otp(struct msm_sensor_ctrl_t *s_ctrl,struct ov13850_otp_struct *otp_ptr)
{
    if (otp_ptr->module_awb_info.flag != 0x00)
    {
    ov13850_apply_awb_otp(s_ctrl,otp_ptr);
    }
	if (otp_ptr->lsc_info.flag != 0x00)
	{
    ov13850_apply_lsc_otp(s_ctrl,otp_ptr);
	}
	//if (otp_ptr->vcm_info.flag != 0x00)
    //{
    //	ov13850_apply_af_otp(s_ctrl,otp_ptr);
    //}
    return 0;
}

void ov13850_LumaDecoder(uint8_t *pData, uint8_t *pPara)
{
    uint32_t Offset, Bit, Option;
    uint32_t i, k;
    uint8_t pCenter[16], pMiddle[32], pCorner[72];
    Offset = pData[0];
    Bit = pData[1]>>4;
    Option = pData[1]&0xf;
    if(Bit <= 5)
    {
        for(i=0,k=2; i<120; i+=8,k+=5)
        {
            pPara[i] = pData[k]>>3; // 7~3 (byte0)
            pPara[i+1] = ((pData[k]&0x7)<<2)|(pData[k+1]>>6); // 2~0 (byte0) and 7~6(byte1)
            pPara[i+2] = (pData[k+1]&0x3e)>>1; // 5~1 (byte1)
            pPara[i+3] = ((pData[k+1]&0x1)<<4)|(pData[k+2]>>4); // 0 (byte1) and 7~4(byte2)
            pPara[i+4] = ((pData[k+2]&0xf)<<1)|(pData[k+3]>>7); // 3~0 (byte2) and 7(byte3)
            pPara[i+5] = (pData[k+3]&0x7c)>>2; // 6~2 (byte3)
            pPara[i+6] = ((pData[k+3]&0x3)<<3)|(pData[k+4]>>5); // 1~0 (byte3) and 7~5(byte4)
            pPara[i+7] = pData[k+4]&0x1f; // 4~0 (byte4)
        }
    }else
    {
        for(i=0,k=2; i<48; i+=8,k+=5)
        {
            pPara[i] = pData[k]>>3; // 7~3 (byte0)
            pPara[i+1] = ((pData[k]&0x7)<<2)|(pData[k+1]>>6); // 2~0 (byte0) and 7~6(byte1)
            pPara[i+2] = (pData[k+1]&0x3e)>>1; // 5~1 (byte1)
            pPara[i+3] = ((pData[k+1]&0x1)<<4)|(pData[k+2]>>4); // 0 (byte1) and 7~4(byte2)
            pPara[i+4] = ((pData[k+2]&0xf)<<1)|(pData[k+3]>>7); // 3~0 (byte2) and 7(byte3)
            pPara[i+5] = (pData[k+3]&0x7c)>>2; // 6~2 (byte3)
            pPara[i+6] = ((pData[k+3]&0x3)<<3)|(pData[k+4]>>5); // 1~0 (byte3) and 7~5(byte4)
            pPara[i+7] = pData[k+4]&0x1f; // 4~0 (byte4)
        }
        for(i=48,k=32; i<120; i+=4,k+=3)
        {
            pPara[i] = pData[k]>>2; // 7~2 (byte0)
            pPara[i+1] = ((pData[k]&0x3)<<4)|(pData[k+1]>>4); // 1~0(byte0)and 7~4(byte1)
            pPara[i+2] = ((pData[k+1]&0xf)<<2)|(pData[k+2]>>6); // 3~0 (byte1) and7~6(byte2)
            pPara[i+3] = pData[k+2]&0x3f; // 5~0 (byte2)
        }
        memcpy(pCenter, pPara, 16);
        memcpy(pMiddle, pPara+16, 32);
        memcpy(pCorner, pPara+48, 72);
        for(i=0; i<32; i++)
        {
            pMiddle[i] <<= (Bit-6);
        }
        for(i=0; i<72; i++)
        {
            pCorner[i] <<= (Bit-6);
        }
        if(Option == 0)
        { // 10x12
            memcpy(pPara, pCorner, 26);
            memcpy(pPara+26, pMiddle, 8);
            memcpy(pPara+34, pCorner+26, 4);
            memcpy(pPara+38, pMiddle+8, 2);
            memcpy(pPara+40, pCenter, 4);
            memcpy(pPara+44, pMiddle+10, 2);
            memcpy(pPara+46, pCorner+30, 4);
            memcpy(pPara+50, pMiddle+12, 2);
            memcpy(pPara+52, pCenter+4, 4);
            memcpy(pPara+56, pMiddle+14, 2);
            memcpy(pPara+58, pCorner+34, 4);
            memcpy(pPara+62, pMiddle+16, 2);
            memcpy(pPara+64, pCenter+8, 4);
            memcpy(pPara+68, pMiddle+18, 2);
            memcpy(pPara+70, pCorner+38, 4);
            memcpy(pPara+74, pMiddle+20, 2);
            memcpy(pPara+76, pCenter+12, 4);
            memcpy(pPara+80, pMiddle+22, 2);
            memcpy(pPara+82, pCorner+42, 4);
            memcpy(pPara+86, pMiddle+24, 8);
            memcpy(pPara+94, pCorner+46, 26);
        }
        else
        { // 12x10
            memcpy(pPara, pCorner, 22);
            memcpy(pPara+22, pMiddle, 6);
            memcpy(pPara+28, pCorner+22, 4);
            memcpy(pPara+32, pMiddle+6, 6);
            memcpy(pPara+38, pCorner+26, 4);
            memcpy(pPara+42, pMiddle+12, 1);
            memcpy(pPara+43, pCenter, 4);
            memcpy(pPara+47, pMiddle+13, 1);
            memcpy(pPara+48, pCorner+30, 4);
            memcpy(pPara+52, pMiddle+14, 1);
            memcpy(pPara+53, pCenter+4, 4);
            memcpy(pPara+57, pMiddle+15, 1);
            memcpy(pPara+58, pCorner+34, 4);
            memcpy(pPara+62, pMiddle+16, 1);
            memcpy(pPara+63, pCenter+8, 4);
            memcpy(pPara+67, pMiddle+17, 1);
            memcpy(pPara+68, pCorner+38, 4);
            memcpy(pPara+72, pMiddle+18, 1);
            memcpy(pPara+73, pCenter+12, 4);
            memcpy(pPara+77, pMiddle+19, 1);
            memcpy(pPara+78, pCorner+42, 4);
            memcpy(pPara+82, pMiddle+20, 6);
            memcpy(pPara+88, pCorner+46, 4);
            memcpy(pPara+92, pMiddle+26, 6);
            memcpy(pPara+98, pCorner+50, 22);
        }
    }
    for(i=0; i<120; i++)
    {
        pPara[i] += Offset;
    }
}
//
void ov13850_ColorDecoder(uint8_t *pData, uint8_t *pPara)
{
    uint32_t Offset, Bit, Option;
    uint32_t i, k;
    uint8_t pBase[30];
    Offset = pData[0];
    Bit = pData[1]>>7;
    Option = (pData[1]&0x40)>>6;
    pPara[0] = (pData[1]&0x3e)>>1; // 5~1 (byte1)
    pPara[1] = ((pData[1]&0x1)<<4)|(pData[2]>>4); // 0 (byte1) and 7~4 (byte2)
    pPara[2] = ((pData[2]&0xf)<<1)|(pData[3]>>7); // 3~0 (byte2) and 7 (byte3)
    pPara[3] = (pData[3]&0x7c)>>2; // 6~2 (byte3)
    pPara[4] = ((pData[3]&0x3)<<3)|(pData[4]>>5); // 1~0 (byte3) and 7~5 (byte4)
    pPara[5] = pData[4]&0x1f; // 4~0 (byte4)
    for(i=6,k=5; i<30; i+=8,k+=5)
    {
        pPara[i] = pData[k]>>3; // 7~3 (byte0)
        pPara[i+1] = ((pData[k]&0x7)<<2)|(pData[k+1]>>6); // 2~0 (byte0) and 7~6 (byte1)
        pPara[i+2] = (pData[k+1]&0x3e)>>1; // 5~1 (byte1)
        pPara[i+3] = ((pData[k+1]&0x1)<<4)|(pData[k+2]>>4); // 0 (byte1) and 7~4 (byte2)
        pPara[i+4] = ((pData[k+2]&0xf)<<1)|(pData[k+3]>>7); // 3~0 (byte2) and 7 (byte3)
        pPara[i+5] = (pData[k+3]&0x7c)>>2; // 6~2 (byte3)
        pPara[i+6] = ((pData[k+3]&0x3)<<3)|(pData[k+4]>>5); // 1~0 (byte3) and 7~5 (byte4)
        pPara[i+7] = pData[k+4]&0x1f; // 4~0 (byte4)
    }

    memcpy(pBase, pPara, 30);
    for(i=0,k=20; i<120; i+=4,k++)
    {
        pPara[i] = pData[k]>>6;
        pPara[i+1] = (pData[k]&0x30)>>4;
        pPara[i+2] = (pData[k]&0xc)>>2;
        pPara[i+3] = pData[k]&0x3;
    }
    if(Option == 0)
    { // 10x12
        for(i=0; i<5; i++)
        {
            for(k=0; k<6; k++)
            {
                pPara[i*24+k*2] += pBase[i*6+k];
                pPara[i*24+k*2+1] += pBase[i*6+k];
                pPara[i*24+k*2+12] += pBase[i*6+k];
                pPara[i*24+k*2+13] += pBase[i*6+k];
            }
        }
    }else
    { // 12x10
        for(i=0; i<6; i++)
        {
            for(k=0; k<5; k++)
            {
                pPara[i*20+k*2] += pBase[i*5+k];
                pPara[i*20+k*2+1] += pBase[i*5+k];
                pPara[i*20+k*2+10] += pBase[i*5+k];
                pPara[i*20+k*2+11] += pBase[i*5+k];
            }
        }
    }
    for(i=0; i<120; i++)
    {
        pPara[i] = (pPara[i]<<Bit) + Offset;
    }
}
//
void ov13850_R2A_LENC_Decoder(uint8_t *pData, uint8_t *pPara)
{
    ov13850_LumaDecoder(pData, pPara);
    ov13850_ColorDecoder(pData+86, pPara+120);
    ov13850_ColorDecoder(pData+136, pPara+240);
}
int ov13850_otp_init(struct msm_sensor_ctrl_t *s_ctrl)
{
  uint8_t temp1;
  SET_OV13850_REG(0x0100, 0x01);
  msleep(10);
  //temp1 = ov13850_otp_read_byte(s_ctrl,0x5002,&temp1);
  ov13850_otp_read_byte(s_ctrl, 0x5002, &temp1);
  //SET_OV13850_REG(0x5002, (0x00 & 0x02) | (temp1 & (~0x02)));
  SET_OV13850_REG(0x5002,(temp1 & (~0x02)));
  // read OTP into buffer
  SET_OV13850_REG(0x3d84, 0xC0);
  SET_OV13850_REG(0x3d88, 0x72); // OTP start address
  SET_OV13850_REG(0x3d89, 0x20);
  SET_OV13850_REG(0x3d8A, 0x73); // OTP end address
  SET_OV13850_REG(0x3d8B, 0xBE);
  SET_OV13850_REG(0x3d81, 0x01); // load otp into buffer
  msleep(10);
  return 0;
}
int32_t ov13850_set_otp(struct msm_sensor_ctrl_t *s_ctrl)
{
  pr_err("zoupeng ov13850_set_otp,sensor name:%s,flag:%d\n",s_ctrl->sensordata->sensor_name,g_ov13850_otp_data.flag);
    if((strcmp(s_ctrl->sensordata->sensor_name, "ov13850")) != 0)
  {
        pr_err("ov13850_set_otp: sensor name is not ov13850\n");
      return -1;
  }
	if(g_ov13850_otp_data.flag == 0)
	{
		ov13850_read_otp(s_ctrl,&g_ov13850_otp_data);
	}
    ov13850_apply_otp(s_ctrl,&g_ov13850_otp_data);
    return 0;
}
