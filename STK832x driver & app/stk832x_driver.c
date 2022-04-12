/* STK832x PSEUDO CODE
 * STK832x SA0/SDO(Pin1) Must be GND or connected to VDD and cannot be float.
 * #define STKI2C_ADDRESS	0x0F  //Pin1 connected to GND
 * #define STKI2C_ADDRESS	0x1F  //Pin1 connected to VDD
 * 
 * 
 */


/* SPI mode needs to open this macro, IIC mode is blocked */
#include "stk832x_driver.h"
#include "software_spi.h"
#include "in_flash_manage.h"
#include "iic_io.h"
#include "bsp.h"





uint8_t last_index = 0;

void ReadAccRegister(uint8_t Reg, uint8_t* Data)
{
	uint8_t SPIMast_Buf[SPI_BUFSIZE];
    uint8_t SPISla_Buf[SPI_BUFSIZE];
//	memset(SPISla_Buf,0,SPI_BUFSIZE);
	
	
	
#if STK_SPI_MODE
	SPIMast_Buf[0] = Reg|0x80;
	SPI_Enable_CS();
    software_spi00_send_dat(SPIMast_Buf,1);
    software_spi00_receiver_buf(SPISla_Buf,1);
    SPI_Disable_CS();
#else
	SPIMast_Buf[0] = Reg;
	hrs_iic_transfer(0x1E,SPIMast_Buf,1,TWI_ISSUE_STOP);
	hrs_iic_transfer(0x1F,SPISla_Buf,1,TWI_ISSUE_STOP);
#endif
	
	*Data = SPISla_Buf[0];
}

void ReadMultiRegister(uint8_t Reg, uint8_t *Data, uint8_t len)
{
	uint8_t SPIMast_Buf[SPI_BUFSIZE]; 
	uint8_t SPISla_Buf[SPI_BUFSIZE];
//	memset(SPISla_Buf,0,SPI_BUFSIZE);
	
	
	
#if STK_SPI_MODE
	SPIMast_Buf[0] = Reg|0x80;
	SPI_Enable_CS();
    software_spi00_send_dat(SPIMast_Buf,1);
	software_spi00_receiver_buf(SPISla_Buf,len);
	SPI_Disable_CS();
#else	
	SPIMast_Buf[0] = Reg;
	hrs_iic_transfer(0x1E,SPIMast_Buf,1,TWI_ISSUE_STOP);
	hrs_iic_transfer(0x1F,SPISla_Buf,len,TWI_ISSUE_STOP);
#endif
	
	memcpy(Data,SPISla_Buf,len);
}

void WriteAccRegister(uint8_t Reg, uint8_t Data)
{
	uint8_t SPIMast_Buf[SPI_BUFSIZE];

//  SPIReadLength = 0;
    
    SPIMast_Buf[1] = Data;
	
#if STK_SPI_MODE
	SPIMast_Buf[0] = Reg&0x7F;
	SPI_Enable_CS();
    software_spi00_send_dat(SPIMast_Buf,2);
    SPI_Disable_CS();
#else
	SPIMast_Buf[0] = Reg;
	hrs_iic_transfer(0x1E,SPIMast_Buf,2,TWI_ISSUE_STOP);
#endif
}

uint8_t stk8xxx_pid_list[3] = {STK8327_CHIPID_VAL, STK8329_CHIPID_VAL, STK832x_CHIPID_VAL};
uint8_t chipid = 0;

/* Read WHO_AM_I register */
bool STK832x_Check_chipid(void)
{
	
    uint16_t i = 0, pid_num = sizeof(stk8xxx_pid_list);

    for (i = 0; i < pid_num; i++)
    {
    	
        if (chipid == stk8xxx_pid_list[i])
        {
        	BLE_RTT("\r\nread stkchip id ok, chip_id = 0x%x\n", chipid);
            return true;
        }
    }
	BLE_RTT("read stkchip id fail, chip_id = 0x%x\n", chipid);
    return false;
}

void STK832x_read_chipid(void)
{
	unsigned char RegAddr = STK_REG_CHIPID;
	
	ReadAccRegister(RegAddr, &chipid);
	
}

void STK832x_REG_COMMAND_SET(STK_REG reg,uint8_t data)
{
	uint8_t RegReadValue;
	WriteAccRegister(reg, data);
	ReadAccRegister(reg, &RegReadValue);
	BLE_RTT("reg : 0x%x,value : 0x%x\r\n",reg,RegReadValue);
}

void STK832x_Suspend_mode(bool en ,uint8_t ODR)
{
	unsigned char RegAddr, RegWriteValue, RegReadValue;
	
    /* suspend mode enable */
	RegAddr       = STK_REG_POWMODE;
	ReadAccRegister(RegAddr,&RegReadValue);
	
	if(en)//enable Gsensor
		RegWriteValue = ODR;
	else
		RegWriteValue = STK_SUSPEND_MODE;//idle
	
    STK832x_REG_COMMAND_SET((STK_REG)RegAddr, RegWriteValue);
}



void STK_SOFT_RESET(void)
{
	WriteAccRegister(STK_REG_SWRST,0xB6);
	nrf_delay_ms(50);
}

void STK832x_SPI_MODE_SET(void)
{
	unsigned char RegAddr, RegWriteValue;
	#if STK_SPI_MODE
		uint8_t tmp_data = 0;
		/* set at SPI mode, resolution */
		RegAddr       = 0x70;
		RegWriteValue = 0x5A;
		ReadAccRegister(RegAddr, &tmp_data);
		if(0xE1 == tmp_data)
		{
			WriteAccRegister(0x33, RegWriteValue);
		}
	#endif
		
	STK832x_REG_COMMAND_SET(STK_REG_INTFCFG,STK_SPI_3WM_DIS);
}

void STK832x_RANGE_SET(uint8_t range)
{
	STK832x_REG_COMMAND_SET(STK_REG_RANGESEL,range);
}

void STK832x_BANDWIDTH_SET(uint8_t bw)
{
	STK832x_REG_COMMAND_SET(STK_REG_BWSEL,bw);
}

void STK832x_ESM_MODE_SET(bool esm_mode)
{
	if(esm_mode)
		STK832x_REG_COMMAND_SET(STK_REG_ODRMODE,STK_ESM_MODE_EN);
	else
		STK832x_REG_COMMAND_SET(STK_REG_ODRMODE,STK_ESM_MODE_DIS);
}

void STK832x_INT1_SET(bool int_en,bool x_en,bool y_en,bool z_en, uint8_t act_lv, uint8_t output_type)
{
	//xyz any-motion interrupt enable
	unsigned char RegWriteValue = 0;
	if(x_en)
		RegWriteValue |= STK_SLP_EN_X;
	if(y_en)
		RegWriteValue |= STK_SLP_EN_Y;
	if(z_en)
		RegWriteValue |= STK_SLP_EN_Z;
	
	STK832x_REG_COMMAND_SET(STK_REG_INTEN1,RegWriteValue);
	
	//int1 mapping
	RegWriteValue = 0;
	if(int_en)
		RegWriteValue |= STK_INT1_EN;
		
	STK832x_REG_COMMAND_SET(STK_REG_INTMAP1,RegWriteValue);
	
	RegWriteValue = 0;
	RegWriteValue = act_lv | output_type;
	STK832x_REG_COMMAND_SET(STK_REG_INTCFG1,RegWriteValue);
}

void STK832x_FIFOMODE_SET(uint8_t fifo_mode)
{
	STK832x_REG_COMMAND_SET(STK_REG_FIFOMODE,fifo_mode);
}
/*
 * Initializes an example function
 * The initial configuration is in low-power mode.
 * STK832x is in the gear of ODR=50Hz, and the range is set to +/-8G
 *
 */
int STK832x_Initialization(void)
{
	
	STK832x_read_chipid();

	if(STK832x_Check_chipid())
	{
//		goto STK832x_SUCCESS_INIT;
	}else{
//		goto STK832x_FAIL_INIT;
		return -1 ;
	}
	
//STK832x_SUCCESS_INIT:	
	STK_SOFT_RESET();

	STK832x_SPI_MODE_SET();
		
		
		/* set range, resolution */
		STK832x_RANGE_SET(STK832x_RANGE_2G);
		
		/* set eng mode */
		STK832x_REG_COMMAND_SET(STK_REG_ENG_MODE,0xC0);
		
		 /* set bandwidth */
		STK832x_BANDWIDTH_SET(STK_BW_250HZ);
		
		/* set es mode */
		STK832x_ESM_MODE_SET(true);
		
		/* set int config */
		STK832x_INT1_SET(true,true,true,true,
							STK_INT1_ACT_LEVEl_L,
							STK_INT1_PUSH_PULL);
		
		/* set fifo mode */
		STK832x_FIFOMODE_SET(FIFO_STREAM_MODE);
		
		/* set power mode */
		STK832x_Suspend_mode(true,STK832x_SET_ODR_10HZ);
		

		BLE_RTT("STK832x_Initialization successful");
		return 0;
	
//STK832x_FAIL_INIT:
//	printf("STK832x_Initialization fail");
//	return -1;
}




/* Read FIFO inside how many still have data at present */
uint8_t STK832x_Read_fifo_len(void)
{
	unsigned char RegAddr = STK_REG_FIFOSTS;
	static uint8_t tmp = 0, frameSize = 0;
	
	ReadAccRegister(RegAddr, &tmp);
	tmp &= FIFO_CNT_MASK;
	if(FIFO_FRAME_MAX_CNT < tmp)
	{
		frameSize = FIFO_FRAME_MAX_CNT;
	}else{
		frameSize = tmp;
	}
//	BLE_RTT("fifo cnts %d\r\n",frameSize);
	return frameSize;
}
/*
 * To read triaxial data from FIFO
 * it is necessary to determine how many pieces of data are stored in FIFO before reading
 *
 */
void STK832x_Getfifo_buf(uint8_t frameSize,AxesRaw_t *axes)
{
	
    uint8_t RegAddr, RegReadValue[frameSize*6];
	
	if(frameSize==0)return;
	
	memset(RegReadValue,0,sizeof(RegReadValue));
	
    RegAddr      = STK_REG_FIFODATA;		
    ReadMultiRegister(RegAddr, RegReadValue, frameSize*6);
	
    for(int i = 0; i < frameSize; i++)
    {
#if STK_8328_C
		axes[i].AXIS_X = (((uint16_t)RegReadValue[i*6+1]) << 8) | (uint16_t)RegReadValue[i*6+0];  //resolution = 16 bit
		axes[i].AXIS_Y = (((uint16_t)RegReadValue[i*6+3]) << 8) | (uint16_t)RegReadValue[i*6+2];  //resolution = 16 bit
		axes[i].AXIS_Z = (((uint16_t)RegReadValue[i*6+5]) << 8) | (uint16_t)RegReadValue[i*6+4];  //resolution = 16 bit
#else
		axes[i].AXIS_X = ((int16_t)((((uint16_t)RegReadValue[i*6+1]) << 8) | (uint16_t)RegReadValue[i*6+0]))>>4;  //resolution = 12 bit
		axes[i].AXIS_Y = ((int16_t)((((uint16_t)RegReadValue[i*6+3]) << 8) | (uint16_t)RegReadValue[i*6+2]))>>4;  //resolution = 12 bit
		axes[i].AXIS_Z = ((int16_t)((((uint16_t)RegReadValue[i*6+5]) << 8) | (uint16_t)RegReadValue[i*6+4]))>>4;  //resolution = 12 bit
#endif	
		
		last_index = i;
		
    }
//	raw_change_To_mg(frameSize,axes);
	BLE_RTT("xyz: %x, %x, %x\r\n",axes[last_index].AXIS_X,axes[last_index].AXIS_Y,axes[last_index].AXIS_Z);
//	BLE_RTT("xyz: %dmg, %dmg, %dmg\r\n",axes[last_index].AXIS_X,axes[last_index].AXIS_Y,axes[last_index].AXIS_Z);
	
}


//void raw_change_To_mg(uint8_t frameSize,AxesRaw_t *axes)
//{
//#if STK_8328_C
//	for(int i = 0; i < frameSize; i++)
//    {
//		axes[i].AXIS_X = (int32_t)axes[i].AXIS_X*61/1000;
//		axes[i].AXIS_Y = (int32_t)axes[i].AXIS_Y*61/1000;
//		axes[i].AXIS_Z = (int32_t)axes[i].AXIS_Z*61/1000;
//	}
//#endif
//}

void read_single_axis(void)
{
	uint8_t RegAddr=0,RegReadValue[6];
	AxesRaw_t axes;
	
	memset(RegReadValue,0,6);
	
	RegAddr      = STK_REG_XOUTL;
	ReadAccRegister(RegAddr,RegReadValue);
	RegAddr      = STK_REG_XOUTH;
	ReadAccRegister(RegAddr,RegReadValue+1);
	
	RegAddr      = STK_REG_YOUTL;
	ReadAccRegister(RegAddr,RegReadValue+2);
	RegAddr      = STK_REG_YOUTH;
	ReadAccRegister(RegAddr,RegReadValue+3);
	
	RegAddr      = STK_REG_ZOUTL;
	ReadAccRegister(RegAddr,RegReadValue+4);
	RegAddr      = STK_REG_ZOUTH;
	ReadAccRegister(RegAddr,RegReadValue+5);
	
	axes.AXIS_X = (((uint16_t)RegReadValue[1]) << 8) | RegReadValue[0];  //resolution = 12 bit
	axes.AXIS_Y = (((uint16_t)RegReadValue[3]) << 8) | RegReadValue[2];  //resolution = 12 bit
	axes.AXIS_Z = (((uint16_t)RegReadValue[5]) << 8) | RegReadValue[4];  //resolution = 12 bit
	
	
	BLE_RTT("xyz: %d, %d, %d\r\n",axes.AXIS_X,axes.AXIS_Y,axes.AXIS_Z);
}

