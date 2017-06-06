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
** Descriptions:		READERʵ�����
**						
**                      RF����
**                      ģʽ�������豸
**                      �ŵ���50
**                      ͨ����PIPE0
**                      ���ʣ�250kbps
**                      ���ʣ�0dbm
**                      CRC�� 16λ
**                      ��ַ��5�ֽڣ�TX_ADDRESS
**                      �Զ�Ӧ�𣺹ر�
** ���ڲ����ʣ�57600
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
 * ��������
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
xdata uint8_t ID_BufTmp[6];// CMD IDH IDL VH VL У���

/******** ID_Buf���ݸ�ʽ  ******************
�ֽ�    1      2     3    4   5  
      ��ʱ��  IDH   IDL  VH  VL
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
 * ��  �� : ��ʼ��IO
 * ��  �� : ��
 * ����ֵ : ��
 *******************************************************************************************************/ 
void IoInit(void)
{
  P0DIR = 0xF6;
  P1DIR = 0xFF;		 
}
/*******************************************************************************************************
 * ��  �� : Timer0��ʼ��
 * ��  �� : ��
 * ����ֵ : ��
 *******************************************************************************************************/ 
void Timer0Init(void)
{
  TMOD = 0x01;	                     //16λ��ʱ��
  TH0  = (65536-TIMER0_VALUE)/256;	 //д���ֵ
  TL0  = (65536-TIMER0_VALUE)%256;
  ET0  = 1;		 //ʹ��Timer0����ж�
  TR0  = 1;		 //����Timer0
}

/*******************************************************************************************************
 * ��  �� : ��ʼ��ʱ��
 * ��  �� : ��
 * ����ֵ : ��
 *******************************************************************************************************/ 
void ClockInit(void)
{  
  hal_clk_set_16m_source(HAL_CLK_XOSC16M);   //ʹ���ⲿ16MHz����	   
}

/*******************************************************************************************************
 * ��  �� : �������߲���
 * ��  �� : ��
 * ����ֵ : ��
 *******************************************************************************************************/ 
void RfCofig(void)
{
  RFCKEN = 1;	     //ʹ��RFʱ��

  
  hal_nrf_close_pipe(HAL_NRF_ALL);          //�ȹر����е�ͨ��
  hal_nrf_open_pipe(HAL_NRF_PIPE0,false);	  //�ٴ�ͨ��0. 	
  
	hal_nrf_set_operation_mode(HAL_NRF_PRX);	// ģʽ�����ջ�
  hal_nrf_set_rf_channel(READER_CH);		    // RF�ŵ���50�����պͷ��ͱ��봦��ͬһ�ŵ�
  hal_nrf_set_datarate(HAL_NRF_250KBPS);	  // RF���ʣ�250KBPS
  hal_nrf_set_output_power(HAL_NRF_0DBM);	  // ���ʣ�0DBM
  hal_nrf_set_crc_mode(HAL_NRF_CRC_16BIT);       //����CRCУ�飺16λCRC������ͷ����豸һ�¡�
  hal_nrf_set_address(HAL_NRF_PIPE0,RX_ADDRESS); //���ý��ջ���ַ 
  hal_nrf_set_rx_payload_width(HAL_NRF_PIPE0, RX_PAYLOAD_LEN); // ����ģʽ����Ҫ�������ݳ���
  hal_nrf_set_power_mode(HAL_NRF_PWR_UP);	    //���ջ��ϵ� 
  
  RF = 1;       //ʹ�������ж�
  EA = 1;	     // ʹ��ȫ���ж�
	
	CE_HIGH(); // ʹ�ܽ���
}
/*******************************************************************************************************
 * ��  �� : ��ǩID�뻺��
 * ��  �� : ��
 * ����ֵ : ��
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
				//�ֽ�:    1     2   3    4  5   6
				//����:   ����  IDH IDL  VH  VL  У��
				if((ID_Buf[i][1] == ID_BufTmp[1]) && (ID_Buf[i][2] == ID_BufTmp[2]))//��ǩID����
				{
					if(ID_Buf[i][0] < ID_OVERTIME)  //δ����20���յ���ͬ�ı�ǩ�Ų��ϱ�
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
		if(iswrite == true) //ID��Ҫд�뻺��
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
 * ��  �� : UART�ϱ����ݴ��
 * ��  �� : ��

			�������ϱ�����λ�����ݸ�ʽ
			
			֡ͷ     Ŀ�ĵ�ַ    Դ��ַ     ���ݳ���     ��ˮ��       ����          �ۼӺ�У��
			33 CC       DA         SA         IL           SN        Package        verification

 * ����ֵ : ��
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
	
	UARTSendbuf[0] = FRAME_FIR_HEAD; //֡ͷ
	UARTSendbuf[1] = FRAME_SEC_HEAD;
	UARTSendbuf[2] = DEST_ADDR;     // DA
	UARTSendbuf[3] = READER_ADDR;   // SA	
	
	for(i=0;i<MAX_TAG_BUFNUM;i++)
	{
		if(ID_Buf[i][0] == TAG_NEED_REP)
		{
			// 8 9 10 11  ID �� ��ص�ѹ
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
	UARTSendbuf[4] = tagnum*4+6;      // ����
	UARTSendbuf[5] = UartSN;          // ��ˮ��  	
	UARTSendbuf[6] = REP_ID_INFO;     // ����
	UARTSendbuf[7] = tagnum;          // TAG����
	   
	fcs = 0;
	for(i=0;i<UARTSendbuf[4];i++)fcs = fcs + UARTSendbuf[i+2];
	UARTSendbuf[UARTSendbuf[4] + 2] = (256 - fcs)%256;
	for(i=0;i<(UARTSendbuf[4] + 3);i++)  hal_uart_putchar(UARTSendbuf[i]);
}
/*******************************************************************************************************
 * ��  �� : ������
 * ��  �� : ��
 * ����ֵ : ��
 *******************************************************************************************************/
void main()
{
  uint8_t i;

  IoInit(); 
	
  ClockInit();		        
  RfCofig();  

  hal_uart_init(UART_BAUD_57K6);  //��ʼ��UART��������57600

  while(hal_clk_get_16m_source() != HAL_CLK_XOSC16M) //�ȴ�ʱ���ȶ�
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
		
		if(RF_Recv_Flag == true)//���յ�Tag��Ϣ
	  {
	    #ifdef DEBUG_LED
      D4 = ~D4;
		  #endif
			
	    RF_Recv_Flag = false;//������Ч��־����
			ID_Inbuf();          //ID��Ϣд�뻺��
    }
		
		if(SecondFlag == true)//1�붨ʱʱ�䵽
		{
			SecondFlag = false;	//�����붨ʱ��־		
			Uart_PackAndRepDat();//�����ϱ�����
    }
  }
}
/*******************************************************************************************************
 * ��  �� : UART�ϱ����ݴ��  ��ȡ���������ݷŵ�һ�����ص�������  �����Ҫ��ʱ����Щ���������͵����ݴ��ڻ�������  Ȼ�����巢�͵���΢��
 * ��  �� : ��
 * ����ֵ : ��
 *******************************************************************************************************/
void rf_irq() interrupt INTERRUPT_RFIRQ
{
  uint8_t irq_flags,i;

  irq_flags = hal_nrf_get_clear_irq_flags(); //��ȡ������жϱ�־
  
  if(irq_flags & (1<<HAL_NRF_RX_DR))//�����ж�
  {
    while(!hal_nrf_rx_fifo_empty())
    {
      PipeAndLen = hal_nrf_read_rx_payload(RxPayload);

		  if((PipeAndLen&0xFF) == RX_PAYLOAD_LEN)  //��鳤���Ƿ�Ϊ6
		  {
			  if(((RxPayload[0]+RxPayload[1]+RxPayload[2]+RxPayload[3]+RxPayload[4]+RxPayload[5])%256) == 0x00)//У����ȷ
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
 * ��  �� : �����ۼӺ�У��
 * ��  �� : ��У��������ַ
 * ����ֵ : У����
 *******************************************************************************************************/
uint8_t Calculate_Fcs(uint8_t *buf)
{
	xdata uint8_t i,fcs;
	fcs = 0;
	for(i=0;i<6;i++)fcs  = fcs + *(buf+i);
	return (256 - fcs);
}
/*******************************************************************************************************
 * ��  �� : Timer0�жϷ�����
 * ��  �� : ��
 * ����ֵ : ��
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
