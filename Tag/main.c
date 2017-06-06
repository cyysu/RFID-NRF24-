/****************************************Copyright (c)****************************************************
**                                        
**                                 
**
**--------------File Info---------------------------------------------------------------------------------
** File name:			main.c
** Last modified Date:          
** Last Version:		1.1
** Descriptions:		
**						
**--------------------------------------------------------------------------------------------------------
** Created by:			FiYu
** Created date:		2014-11-12
** Version:			    1.0
** Descriptions:		有源RFID实验程序-TAG
**						      发送数据格式 
**                  长度：6个字节    
**                  ID长度：2字节
字节:    1     2   3    4  5   6
意义:   命令  IDH IDL  VH  VL  校验

**--------------------------------------------------------------------------------------------------------
** Modified by:			
** Modified date:		
** Version:				
** Descriptions:		
**
** Rechecked by:			
**********************************************************************************************************/
#include <reg24le1.h>
#include <stdint.h>
#include <stdbool.h>
#include "config.h"
#include "hal_nrf.h"
#include "hal_nrf_hw.h"
#include "hal_clk.h"
#include "hal_rtc.h"
#include "hal_delay.h"	
#include "hal_adc.h"
#include "hal_wdog.h"

/*-------------------管脚定义--------------------------------------------------*/
#define  D4    P00  // 开发板上的指示灯D4
#define  D5    P01  // 开发板上的指示灯D5
#define  S2    P12  // 开发板上的按键S2
#define  S3    P13  // 开发板上的按键S3
#define  ADC   P06

/* 开发板中nRF24LE1管脚配置
P00：输出，驱动D4	         P12：输入，按键检测S2  
P01：输出，驱动D5			     P13：输入，按键检测S3
P02：输出					         P14：输出
P03：输出，UART TXD			   P15：输出
P04：输入，UART RXD			   P16：输出
P06：输入，AIN6  AD检测
*/

/*******************************************************************************************************
 * 变量定义
 *******************************************************************************************************/
xdata bool  radio_busy;
xdata uint8_t  TxPayload[32];
xdata uint8_t  RxPayload[32];
uint16_t PipeAndLen;
uint8_t  TX_ADDRESS[5]  = {0xE7,0xE7,0xE7,0xE7,0xE7}; // TX address  

xdata TagInformation TagInfo;


/*******************************************************************************************************
 * 描  述 : 初始化IO
 * 入  参 : 无
 * 返回值 : 无
 *******************************************************************************************************/ 
void IoInit(void)
{
  P0DIR = 0x00;
  P1DIR = 0x00;		 	
}

/*********************************************************************************************************
** 描  述:  adc初始化															
** 入  参:  NONE		
** 返回值:  NONE																															
*********************************************************************************************************/
void adc_init(void)
{
  hal_adc_set_input_channel(HAL_INP_VDD1_3);          //设置通道 检测1/3 VDD电压                  
  hal_adc_set_reference(HAL_ADC_REF_INT);             //设置参考电压 内部1.22V                  
  hal_adc_set_input_mode(HAL_ADC_SINGLE);             //单端输入                
  hal_adc_set_conversion_mode(HAL_ADC_SINGLE_STEP);   //单次采样模式 
  hal_adc_set_sampling_rate(HAL_ADC_2KSPS);           //采样速率为	2ksps
  hal_adc_set_resolution(HAL_ADC_RES_12BIT);          //12位ADC                     
  hal_adc_set_data_just(HAL_ADC_JUST_RIGHT);          //数据右对齐
}

/*******************************************************************************************************
 * 描  述 : 设置休眠时间
 * 入  参 : period:休眠时间
 * 返回值 : 无
 *******************************************************************************************************/
void set_timer_period(uint16_t period)
{
	if((period<10) && (period>65536))period = 32768;
	
	hal_rtc_start(false);                             
  hal_rtc_start(true);                               
  hal_rtc_set_compare_value(period - 1);
}

/*******************************************************************************************************
 * 描  述 : 配置无线参数
 * 入  参 : 无
 * 返回值 : 无
 *******************************************************************************************************/
void RfCofig(void)
{
  RFCKEN = 1;	     //使能RF时钟
	
	hal_nrf_close_pipe(HAL_NRF_ALL);           //先关闭所有的通道.
	hal_nrf_open_pipe(HAL_NRF_PIPE0,false);	   //再打开通道0.

  hal_nrf_set_operation_mode(HAL_NRF_PTX);    // 模式：发送
  hal_nrf_set_rf_channel(TAG_CH);		          // RF信道：50。接收和发送必须处于同一信道
  hal_nrf_set_datarate(HAL_NRF_250KBPS);	    // RF速率：250KBPS
  hal_nrf_set_output_power(HAL_NRF_0DBM);	    // 功率：0DBM
  hal_nrf_set_crc_mode(HAL_NRF_CRC_16BIT);    //设置CRC校验：16位CRC。必须和接收设备一致。
  hal_nrf_set_address(HAL_NRF_TX,TX_ADDRESS);  //设置发射机地址
  hal_nrf_set_auto_retr(0,1500);			            //自动重发:0 
   
  hal_nrf_set_power_mode(HAL_NRF_PWR_UP);	        //发射机上电
  RF = 1;       //使能无线中断
  EA = 1;	     // 使能全局中断
}

/*******************************************************************************************************
 * 描  述 : 时钟和RTC唤醒配置
 * 入  参 : 无
 * 返回值 : 无
 *******************************************************************************************************/
void mcu_init(void)
{
	hal_rtc_start(false);
  hal_clklf_set_source(HAL_CLKLF_RCOSC32K);           // Use 32.768KHz的时钟源为内部RC	   

	hal_rtc_set_compare_mode(HAL_RTC_COMPARE_MODE_0);   // Use 32 KHz timer mode 0
	set_timer_period(TAG_TIME);	                        // Set the RTC2 time，card sleep time
	hal_clk_set_16m_source(HAL_CLK_XOSC16M);            // Always run on 16MHz crystal oscillator
	hal_clk_regret_xosc16m_on(0);                       // Keep XOSC16M off in register retention

  hal_rtc_start(true);              
	
  while((CLKLFCTRL&0x80)==0x80);	                    // Wait for the 32kHz to startup (change phase)
  while((CLKLFCTRL&0x80)!=0x80); 
}

/*******************************************************************************************************
 * 描  述 : 组装数据
 * 入  参 : 无
 * 返回值 : 无
 *******************************************************************************************************/
void Assemble_Data(void)
{
  xdata uint8_t fcs = 0,i;
	
	
	TxPayload[0] = 0x1E;
	TxPayload[1] = TagInfo.id.id8[0];  //IDL
	TxPayload[2] = TagInfo.id.id8[1];  //IDH
	TxPayload[3] = TagInfo.CellVoltageH;
	TxPayload[4] = TagInfo.CellVoltageL;	
	for(i=0;i<(MAX_TX_PAYLOAD-1);i++)fcs += TxPayload[i];

	TxPayload[MAX_TX_PAYLOAD - 1] = (256 - fcs)%256;
}

/*******************************************************************************************************
 * 描  述 : 主函数
 * 入  参 : 无
 * 返回值 : 无
 *******************************************************************************************************/ 
void main()
{		
  uint8_t RfReceLen;
	xdata   uint32_t  loopCount = ADC_TIME-1;
  
  TagInfo.id.id16 = TAG_ID;
		
	IoInit();  
  mcu_init();	

  #ifdef DEBUG_UART
  hal_uart_init(UART_BAUD_57K6);  //初始化UART，波特率57600
  while(hal_clk_get_16m_source() != HAL_CLK_XOSC16M) //等待时钟稳定
    ;
	#endif
	
  adc_init();
	RfCofig();
 
	#ifdef  USE_WDT
	hal_wdog_init(WDT_TIME);//配置看门狗超时时间2s，使能看门狗
	#endif
	
  while(1)
  {	
    loopCount++;
		
		#ifdef  USE_WDT
	  hal_wdog_restart(); //喂狗
	  #endif
		
		if(loopCount == ADC_TIME)    //启动后执行一次AD检测，以后，每2小时检测一次
    {
      hal_adc_start();           //启动ADC
      while( hal_adc_busy())     //等待ADC转换结束
        ;
			TagInfo.CellVoltageH = hal_adc_read_MSB(); //读取ADC采样值
      TagInfo.CellVoltageL = hal_adc_read_LSB();                     
      loopCount=0;
    }
		
		#ifdef DEBUG_LED
    D4 = ~D4;
		#endif
		
    PWRDWN = 0x04;    // 进入寄存器维持低功耗模式
    PWRDWN = 0x00;

		Assemble_Data();  // 数据打包
		hal_nrf_write_tx_payload(TxPayload,MAX_TX_PAYLOAD); 		

    CE_PULSE();	            //无线发射数据 
    radio_busy = true;    
    while(radio_busy)		    //等待操作完成
      ;
  }                                           
}

/*******************************************************************************************************
 * 描  述 : 无线中断服务函数
 * 入  参 : 无
 * 返回值 : 无
 *******************************************************************************************************/
void rf_irq() interrupt INTERRUPT_RFIRQ
{
  uint8_t  irq_flags;
   
  irq_flags = hal_nrf_get_clear_irq_flags(); //读取并清除无线中断标志

  if(irq_flags & (1<<HAL_NRF_RX_DR))  //接收到数据?
  {
	
	  while(!hal_nrf_rx_fifo_empty())// Read payload
    {
			PipeAndLen = hal_nrf_read_rx_payload(RxPayload);//读取数据
    }
	  radio_busy = false;
  }

  if(irq_flags & ((1<<HAL_NRF_TX_DS)))			// transimmter finish 
  {
    radio_busy = false;			
  }

  if(irq_flags & ((1<<HAL_NRF_MAX_RT)))			// re-transimmter
  {
    radio_busy = false;
    hal_nrf_flush_tx();
  }
}
/********************************************END FILE*****************************************************/			
													
