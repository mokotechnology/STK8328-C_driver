#include "stk832x_app.h"
#include "stk832x_driver.h"
#include "software_spi.h"
#include "app_timer.h"
#include "in_flash_manage.h"
#include "iic_io.h"
#include "ble_nus.h"
#include "bsp.h"


APP_TIMER_DEF(read_3dh_timer);  
static uint8_t read_3dh_enable_300ms =0;

#define MAX_AXES_DATA   100
AxesRaw_t axes[MAX_AXES_DATA];

static void read_3dh_timeout(void * p_context)
{
	read_3dh_enable_300ms =1;
}

static void init_read_3dh_timer(void)
{
//    uint32_t err_code;
//    
//    err_code = 
	app_timer_create(&read_3dh_timer,
                                APP_TIMER_MODE_REPEATED,
                                read_3dh_timeout);
//    APP_ERROR_CHECK(err_code);
	
	
}

void restart_3dh_timer(void)
{
	app_timer_start(read_3dh_timer,APP_TIMER_TICKS(300),NULL);
}

void  stop_3dh_timer(void)
{
	app_timer_stop(read_3dh_timer);
}


void STK832x_Init(void)
{
#if STK_SPI_MODE
	nrf_gpio_cfg_output(StuHis.SPI_pin.CSN);
    nrf_gpio_pin_set(StuHis.SPI_pin.CSN);
    software_spi_init(StuHis.SPI_pin.MISO,StuHis.SPI_pin.MOSI,StuHis.SPI_pin.CLK);
#else
	hrs_iic_init(StuHis.SPI_pin.MOSI,StuHis.SPI_pin.CLK);
#endif
	
	STK832x_Initialization();
	init_read_3dh_timer();
	
}

void STK832x_start(void)
{
	restart_3dh_timer();
	STK832x_Suspend_mode(true,STK832x_SET_ODR_10HZ);
}
	
void STK832x_stop(void)
{
	stop_3dh_timer();
	STK832x_Suspend_mode(false,0);
}

void read_data_3dh(void)
{
	uint8_t times = 0;
	uint8_t buf[6],buf_len;
	
	times = STK832x_Read_fifo_len();
//	read_xyz();
//	set_mode();
	STK832x_Getfifo_buf(times,axes);
	
	for(uint8_t i=0;i<times;i++)
	{
		buf_len = 0;
		buf[buf_len++] = axes[i].AXIS_X>>8;
		buf[buf_len++] = axes[i].AXIS_X;
		buf[buf_len++] = axes[i].AXIS_Y>>8;
		buf[buf_len++] = axes[i].AXIS_Y;
		buf[buf_len++] = axes[i].AXIS_Z>>8;
		buf[buf_len++] = axes[i].AXIS_Z;
		
		ble_notice_send(sensor_3dh_handle_n.value_handle,sensor_3dh_notice,buf,buf_len);
	}
}


void check_int(void)
{
	if(nrf_gpio_pin_read(INT1_IO)==0)
	{
		
		BLE_RTT("device move\r\n");
	}
}
	


void task_read_3dh(void)
{
//	if(STK832x_Check_chipid()==false)return;
	if(read_3dh_enable_300ms==0)return;
	read_3dh_enable_300ms = 0;
	
	read_data_3dh();
	
	check_int();
}

