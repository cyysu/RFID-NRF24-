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
** Descriptions:		��ԴRFIDʵ�����-TAG
**						      �������ݸ�ʽ 
**                  ���ȣ�6���ֽ�    
**                  ID���ȣ�2�ֽ�
�ֽ�:    1     2   3    4  5   6
����:   ����  IDH IDL  VH  VL  У��

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

/*-------------------�ܽŶ���--------------------------------------------------*/
#define  D4    P00  // �������ϵ�ָʾ��D4
#define  D5    P01  // �������ϵ�ָʾ��D5
#define  S2    P12  // �������ϵİ���S2
#define  S3    P13  // �������ϵİ���S3
#define  ADC   P06

/* ��������nRF24LE1�ܽ�����
P00�����������D4	         P12�����룬�������S2  
P01�����������D5			     P13�����룬�������S3
P02�����					         P14�����
P03�������UART TXD			   P15�����
P04�����룬UART RXD			   P16�����
P06�����룬AIN6  AD���
*/

/*******************************************************************************************************
 * ��������
 *******************************************************************************************************/
xdata bool  radio_busy;
xdata uint8_t  TxPayload[32];
xdata uint8_t  RxPayload[32];
uint16_t PipeAndLen;
uint8_t  TX_ADDRESS[5]  = {0xE7,0xE7,0xE7,0xE7,0xE7}; // TX address  

xdata TagInformation TagInfo;


/*******************************************************************************************************
 * ��  �� : ��ʼ��IO
 * ��  �� : ��
 * ����ֵ : ��
 *******************************************************************************************************/ 
void IoInit(void)
{
  P0DIR = 0x00;
  P1DIR = 0x00;		 	
}

/*********************************************************************************************************
** ��  ��:  adc��ʼ��															
** ��  ��:  NONE		
** ����ֵ:  NONE																															
*********************************************************************************************************/
void adc_init(void)
{
  hal_adc_set_input_channel(HAL_INP_VDD1_3);          //����ͨ�� ���1/3 VDD��ѹ                  
  hal_adc_set_reference(HAL_ADC_REF_INT);             //���òο���ѹ �ڲ�1.22V                  
  hal_adc_set_input_mode(HAL_ADC_SINGLE);             //��������                
  hal_adc_set_conversion_mode(HAL_ADC_SINGLE_STEP);   //���β���ģʽ 
  hal_adc_set_sampling_rate(HAL_ADC_2KSPS);           //��������Ϊ	2ksps
  hal_adc_set_resolution(HAL_ADC_RES_12BIT);          //12λADC                     
  hal_adc_set_data_just(HAL_ADC_JUST_RIGHT);          //�����Ҷ���
}

/*******************************************************************************************************
 * ��  �� : ��������ʱ��
 * ��  �� : period:����ʱ��
 * ����ֵ : ��
 *******************************************************************************************************/
void set_timer_period(uint16_t period)
{
	if((period<10) && (period>65536))period = 32768;
	
	hal_rtc_start(false);                             
  hal_rtc_start(true);                               
  hal_rtc_set_compare_value(period - 1);
}

/*******************************************************************************************************
 * ��  �� : �������߲���
 * ��  �� : ��
 * ����ֵ : ��
 *******************************************************************************************************/
void RfCofig(void)
{
  RFCKEN = 1;	     //ʹ��RFʱ��
	
	hal_nrf_close_pipe(HAL_NRF_ALL);           //�ȹر����е�ͨ��.
	hal_nrf_open_pipe(HAL_NRF_PIPE0,false);	   //�ٴ�ͨ��0.

  hal_nrf_set_operation_mode(HAL_NRF_PTX);    // ģʽ������
  hal_nrf_set_rf_channel(TAG_CH);		          // RF�ŵ���50�����պͷ��ͱ��봦��ͬһ�ŵ�
  hal_nrf_set_datarate(HAL_NRF_250KBPS);	    // RF���ʣ�250KBPS
  hal_nrf_set_output_power(HAL_NRF_0DBM);	    // ���ʣ�0DBM
  hal_nrf_set_crc_mode(HAL_NRF_CRC_16BIT);    //����CRCУ�飺16λCRC������ͽ����豸һ�¡�
  hal_nrf_set_address(HAL_NRF_TX,TX_ADDRESS);  //���÷������ַ
  hal_nrf_set_auto_retr(0,1500);			            //�Զ��ط�:0 
   
  hal_nrf_set_power_mode(HAL_NRF_PWR_UP);	        //������ϵ�
  RF = 1;       //ʹ�������ж�
  EA = 1;	     // ʹ��ȫ���ж�
}

/*******************************************************************************************************
 * ��  �� : ʱ�Ӻ�RTC��������
 * ��  �� : ��
 * ����ֵ : ��
 *******************************************************************************************************/
void mcu_init(void)
{
	hal_rtc_start(false);
  hal_clklf_set_source(HAL_CLKLF_RCOSC32K);           // Use 32.768KHz��ʱ��ԴΪ�ڲ�RC	   

	hal_rtc_set_compare_mode(HAL_RTC_COMPARE_MODE_0);   // Use 32 KHz timer mode 0
	set_timer_period(TAG_TIME);	                        // Set the RTC2 time��card sleep time
	hal_clk_set_16m_source(HAL_CLK_XOSC16M);            // Always run on 16MHz crystal oscillator
	hal_clk_regret_xosc16m_on(0);                       // Keep XOSC16M off in register retention

  hal_rtc_start(true);              
	
  while((CLKLFCTRL&0x80)==0x80);	                    // Wait for the 32kHz to startup (change phase)
  while((CLKLFCTRL&0x80)!=0x80); 
}

/*******************************************************************************************************
 * ��  �� : ��װ����
 * ��  �� : ��
 * ����ֵ : ��
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
 * ��  �� : ������
 * ��  �� : ��
 * ����ֵ : ��
 *******************************************************************************************************/ 
void main()
{		
  uint8_t RfReceLen;
	xdata   uint32_t  loopCount = ADC_TIME-1;
  
  TagInfo.id.id16 = TAG_ID;
		
	IoInit();  
  mcu_init();	

  #ifdef DEBUG_UART
  hal_uart_init(UART_BAUD_57K6);  //��ʼ��UART��������57600
  while(hal_clk_get_16m_source() != HAL_CLK_XOSC16M) //�ȴ�ʱ���ȶ�
    ;
	#endif
	
  adc_init();
	RfCofig();
 
	#ifdef  USE_WDT
	hal_wdog_init(WDT_TIME);//���ÿ��Ź���ʱʱ��2s��ʹ�ܿ��Ź�
	#endif
	
  while(1)
  {	
    loopCount++;
		
		#ifdef  USE_WDT
	  hal_wdog_restart(); //ι��
	  #endif
		
		if(loopCount == ADC_TIME)    //������ִ��һ��AD��⣬�Ժ�ÿ2Сʱ���һ��
    {
      hal_adc_start();           //����ADC
      while( hal_adc_busy())     //�ȴ�ADCת������
        ;
			TagInfo.CellVoltageH = hal_adc_read_MSB(); //��ȡADC����ֵ
      TagInfo.CellVoltageL = hal_adc_read_LSB();                     
      loopCount=0;
    }
		
		#ifdef DEBUG_LED
    D4 = ~D4;
		#endif
		
    PWRDWN = 0x04;    // ����Ĵ���ά�ֵ͹���ģʽ
    PWRDWN = 0x00;

		Assemble_Data();  // ���ݴ��
		hal_nrf_write_tx_payload(TxPayload,MAX_TX_PAYLOAD); 		

    CE_PULSE();	            //���߷������� 
    radio_busy = true;    
    while(radio_busy)		    //�ȴ��������
      ;
  }                                           
}

/*******************************************************************************************************
 * ��  �� : �����жϷ�����
 * ��  �� : ��
 * ����ֵ : ��
 *******************************************************************************************************/
void rf_irq() interrupt INTERRUPT_RFIRQ
{
  uint8_t  irq_flags;
   
  irq_flags = hal_nrf_get_clear_irq_flags(); //��ȡ����������жϱ�־

  if(irq_flags & (1<<HAL_NRF_RX_DR))  //���յ�����?
  {
	
	  while(!hal_nrf_rx_fifo_empty())// Read payload
    {
			PipeAndLen = hal_nrf_read_rx_payload(RxPayload);//��ȡ����
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
													
