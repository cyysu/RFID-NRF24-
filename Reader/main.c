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
** Version:			    1.1
** Descriptions:		READER实验程序
**						
**                      RF配置
**                      模式：接收设备
**                      信道：50
**                      通道：PIPE0
**                      速率：250kbps
**                      功率：0dbm
**                      CRC： 16位
**                      地址：5字节，TX_ADDRESS
**                      自动应答：关闭
** 串口波特率：57600
**  
**  
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
#include "hal_nrf.h"
#include "hal_nrf_hw.h"
#include "hal_uart.h"
#include "hal_clk.h"
#include "hal_delay.h"
#include "config.h"
#include "hal_wdog.h"


/*******************************************************************************************************
 * 变量定义
 *******************************************************************************************************/
xdata uint8_t RxPayload[32];
xdata uint8_t AckPayLoad[32];
uint8_t  RF_Recv_Flag;
uint16_t PipeAndLen;
xdata uint8_t  RX_ADDRESS[5]  = {0xE7,0xE7,0xE7,0xE7,0xE7}; // RX address 

xdata ReaderInformation ReaderInfo;

xdata bool  RepFlag = false;
xdata uint8_t dest_addr;
xdata uint8_t UARTSendbuf[MAX_REP_IDNUM*5+10];
xdata uint8_t ID_Buf[MAX_TAG_BUFNUM][5];
xdata uint8_t ID_BufTmp[6];// CMD IDH IDL VH VL 校验和

/******** ID_Buf数据格式  ******************
字节    1      2     3    4   5  
      计时器  IDH   IDL  VH  VL
*******************************************/
xdata uint8_t TimeCount=0;
xdata uint8_t SecondFlag = 0;
xdata uint8_t UartSN = 0;
/*******************************************************************************************************
 * Function DEFINITIONS
 *******************************************************************************************************/
void DataInBuffer(uint8_t dat);
uint8_t Calculate_Fcs(uint8_t *buf);



/*******************************************************************************************************
 * 描  述 : 初始化IO
 * 入  参 : 无
 * 返回值 : 无
 *******************************************************************************************************/ 
void IoInit(void)
{
  P0DIR = 0xF6;
  P1DIR = 0xFF;		 
}
/*******************************************************************************************************
 * 描  述 : Timer0初始化
 * 入  参 : 无
 * 返回值 : 无
 *******************************************************************************************************/ 
void Timer0Init(void)
{
  TMOD = 0x01;	                     //16位定时器
  TH0  = (65536-TIMER0_VALUE)/256;	 //写入初值
  TL0  = (65536-TIMER0_VALUE)%256;
  ET0  = 1;		 //使能Timer0溢出中断
  TR0  = 1;		 //启动Timer0
}

/*******************************************************************************************************
 * 描  述 : 初始化时钟
 * 入  参 : 无
 * 返回值 : 无
 *******************************************************************************************************/ 
void ClockInit(void)
{  
  hal_clk_set_16m_source(HAL_CLK_XOSC16M);   //使用外部16MHz晶振	   
}

/*******************************************************************************************************
 * 描  述 : 配置无线参数
 * 入  参 : 无
 * 返回值 : 无
 *******************************************************************************************************/ 
void RfCofig(void)
{
  RFCKEN = 1;	     //使能RF时钟

  
  hal_nrf_close_pipe(HAL_NRF_ALL);          //先关闭所有的通道
  hal_nrf_open_pipe(HAL_NRF_PIPE0,false);	  //再打开通道0. 	
  
	hal_nrf_set_operation_mode(HAL_NRF_PRX);	// 模式：接收机
  hal_nrf_set_rf_channel(READER_CH);		    // RF信道：50。接收和发送必须处于同一信道
  hal_nrf_set_datarate(HAL_NRF_250KBPS);	  // RF速率：250KBPS
  hal_nrf_set_output_power(HAL_NRF_0DBM);	  // 功率：0DBM
  hal_nrf_set_crc_mode(HAL_NRF_CRC_16BIT);       //设置CRC校验：16位CRC。必须和发送设备一致。
  hal_nrf_set_address(HAL_NRF_PIPE0,RX_ADDRESS); //设置接收机地址 
  hal_nrf_set_rx_payload_width(HAL_NRF_PIPE0, RX_PAYLOAD_LEN); // 接收模式下需要设置数据长度
  hal_nrf_set_power_mode(HAL_NRF_PWR_UP);	    //接收机上电 
  
  RF = 1;       //使能无线中断
  EA = 1;	     // 使能全局中断
	
	CE_HIGH(); // 使能接收
}
/*******************************************************************************************************
 * 描  述 : 标签ID入缓存
 * 入  参 : 无
 * 返回值 : 无
 *******************************************************************************************************/ 
void ID_Inbuf(void)
{
	uint8_t i,idx;
	xdata uint8_t iswrite,lastcnt = 0,row = 0;
	
	idx = 0xFF;

	if(ID_BufTmp[0] == CMD_TAG_REPID)
	{
		iswrite = true;
		
		for(i=0;i<MAX_TAG_BUFNUM;i++)
		{
			if(ID_Buf[i][0] != 0)
			{
				//字节:    1     2   3    4  5   6
				//意义:   命令  IDH IDL  VH  VL  校验
				if((ID_Buf[i][1] == ID_BufTmp[1]) && (ID_Buf[i][2] == ID_BufTmp[2]))//标签ID存在
				{
					if(ID_Buf[i][0] < ID_OVERTIME)  //未超过20秒收到相同的标签号不上报
					{
						iswrite = false;
						break;
					}
					else 
					{
            ID_Buf[i][0] = TAG_NEED_REP;
						iswrite = false;
						break;
          }
				}
				else
				{
					if(ID_Buf[i][0]>lastcnt)
					{
						lastcnt = ID_Buf[i][0];
						row = i;
					}
				}
			}
			else
			{
				if(idx == 0xFF)idx = i;
			}
		}
		if(iswrite == true) //ID需要写入缓存
		{
			if(idx < MAX_TAG_BUFNUM)
			{
				for(i=0;i<(RX_PAYLOAD_LEN-2);i++)ID_Buf[idx][i+1] = ID_BufTmp[i+1];
				ID_Buf[idx][0] = TAG_NEED_REP;
			}
			else 
			{
				if(row < MAX_TAG_BUFNUM)
				{
					for(i=0;i<(RX_PAYLOAD_LEN-2);i++)ID_Buf[row][i+1] = ID_BufTmp[i+1];
					ID_Buf[row][0] = TAG_NEED_REP;
				}
			}
		}	
	}
}

/*******************************************************************************************************
 * 描  述 : UART上报数据打包
 * 入  参 : 无

			读卡器上报给上位机数据格式
			
			帧头     目的地址    源地址     数据长度     流水号       数据          累加和校验
			33 CC       DA         SA         IL           SN        Package        verification

 * 返回值 : 无
 *******************************************************************************************************/ 
void Uart_PackAndRepDat(void)
{
	xdata uint8_t i,tagnum = 0,fcs;
	
	RepFlag = false;

	for(i=0;i<MAX_TAG_BUFNUM;i++)
	{
		if(ID_Buf[i][0] > 1)
		{
			ID_Buf[i][0]++;
			if(ID_Buf[i][0] >= ID_OVERTIME)ID_Buf[i][0] = 0;
		}
	}
	
	UARTSendbuf[0] = FRAME_FIR_HEAD; //帧头
	UARTSendbuf[1] = FRAME_SEC_HEAD;
	UARTSendbuf[2] = DEST_ADDR;     // DA
	UARTSendbuf[3] = READER_ADDR;   // SA	
	
	for(i=0;i<MAX_TAG_BUFNUM;i++)
	{
		if(ID_Buf[i][0] == TAG_NEED_REP)
		{
			// 8 9 10 11  ID 和 电池电压
			UARTSendbuf[4*tagnum + 8]  = ID_Buf[i][1];
			UARTSendbuf[4*tagnum + 9]  = ID_Buf[i][2];
			UARTSendbuf[4*tagnum + 10]  = ID_Buf[i][3];
			UARTSendbuf[4*tagnum + 11] = ID_Buf[i][4];
			ID_Buf[i][0] = 2;
			tagnum++;
			RepFlag = true;
			if(tagnum == MAX_REP_IDNUM)break;
		}	
	}
  UartSN++;
	if(UartSN == 0xFF)UartSN = 1;
	UARTSendbuf[4] = tagnum*4+6;      // 长度
	UARTSendbuf[5] = UartSN;          // 流水号  	
	UARTSendbuf[6] = REP_ID_INFO;     // 命令
	UARTSendbuf[7] = tagnum;          // TAG数量
	   
	fcs = 0;
	for(i=0;i<UARTSendbuf[4];i++)fcs = fcs + UARTSendbuf[i+2];
	UARTSendbuf[UARTSendbuf[4] + 2] = (256 - fcs)%256;
	for(i=0;i<(UARTSendbuf[4] + 3);i++)  hal_uart_putchar(UARTSendbuf[i]);
}
/*******************************************************************************************************
 * 描  述 : 主函数
 * 入  参 : 无
 * 返回值 : 无
 *******************************************************************************************************/
void main()
{
  uint8_t i;

  IoInit(); 
	
  ClockInit();		        
  RfCofig();  

  hal_uart_init(UART_BAUD_57K6);  //初始化UART，波特率57600

  while(hal_clk_get_16m_source() != HAL_CLK_XOSC16M) //等待时钟稳定
    ;
  Timer0Init();

  for(i=0;i<MAX_TAG_BUFNUM;i++)ID_Buf[i][0] = 0;     
	#ifdef  USE_WDT
	hal_wdog_init(WDT_TIME);
	#endif

	
	while(1)
  {	   
		#ifdef  USE_WDT
	  hal_wdog_restart();
	  #endif		
		
		if(RF_Recv_Flag == true)//接收到Tag信息
	  {
	    #ifdef DEBUG_LED
      D4 = ~D4;
		  #endif
			
	    RF_Recv_Flag = false;//接收有效标志清零
			ID_Inbuf();          //ID信息写入缓存
    }
		
		if(SecondFlag == true)//1秒定时时间到
		{
			SecondFlag = false;	//清零秒定时标志		
			Uart_PackAndRepDat();//串口上报数据
    }
  }
}
/*******************************************************************************************************
 * 描  述 : UART上报数据打包  读取出来的数据放到一个本地的数组中  如果需要的时候将这些数据所发送的数据存在缓冲里面  然后整体发送到上微机
 * 入  参 : 无
 * 返回值 : 无
 *******************************************************************************************************/
void rf_irq() interrupt INTERRUPT_RFIRQ
{
  uint8_t irq_flags,i;

  irq_flags = hal_nrf_get_clear_irq_flags(); //读取并清除中断标志
  
  if(irq_flags & (1<<HAL_NRF_RX_DR))//接收中断
  {
    while(!hal_nrf_rx_fifo_empty())
    {
      PipeAndLen = hal_nrf_read_rx_payload(RxPayload);

		  if((PipeAndLen&0xFF) == RX_PAYLOAD_LEN)  //检查长度是否为6
		  {
			  if(((RxPayload[0]+RxPayload[1]+RxPayload[2]+RxPayload[3]+RxPayload[4]+RxPayload[5])%256) == 0x00)//校验正确
			  {
			    for(i=0;i<(RX_PAYLOAD_LEN-1);i++)ID_BufTmp[i]	= RxPayload[i];		
				  RF_Recv_Flag = true;
			  }
		  }
		  hal_nrf_flush_rx();
	  }
  }
  if(irq_flags & ((1<<HAL_NRF_TX_DS)))				 // transimmter finish 
  {
    hal_nrf_flush_tx();			
  }

  if(irq_flags & ((1<<HAL_NRF_MAX_RT)))				 // re-transimmter
  {
    hal_nrf_flush_tx();
  }
}
/*******************************************************************************************************
 * 描  述 : 计算累加和校核
 * 入  参 : 待校验的数组地址
 * 返回值 : 校验结果
 *******************************************************************************************************/
uint8_t Calculate_Fcs(uint8_t *buf)
{
	xdata uint8_t i,fcs;
	fcs = 0;
	for(i=0;i<6;i++)fcs  = fcs + *(buf+i);
	return (256 - fcs);
}
/*******************************************************************************************************
 * 描  述 : Timer0中断服务函数
 * 入  参 : 无
 * 返回值 : 无
 *******************************************************************************************************/
void Timer0_irq() interrupt INTERRUPT_T0
{
  
  TH0=(65536-TIMER0_VALUE)/256;
  TL0=(65536-TIMER0_VALUE)%256;
  TimeCount++;
  
	if(TimeCount == 50)	//1000ms 
  {
    TimeCount=0;
		SecondFlag = true;
  }
}
/*********************************END FILE****************************************************************/																
