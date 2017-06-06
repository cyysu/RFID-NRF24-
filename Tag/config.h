#ifndef CONFIG_H__
#define CONFIG_H__

/******************************************************************************/
/* ������Ϣ*/

#define  DEBUG_LED    1


/******************************************************************************/
/* Tag ����  */
#define  TAG_ID          1000	    // ID
#define  TAG_CH          50	      // �ŵ�
#define  TAG_TIME        32768U	  // ����ʱ�䣺1000ms

#define  MAX_TX_PAYLOAD  6        // ���߷������ݳ���
#define  TAG_ID_LEN      2        // ID�ֽ���
#define  ADC_TIME        7200U      // ADC���ʱ����

/******************************************************************************/
/* Watchdog*/
/*
** ���Ź���ʱʱ�����	
**  ��ʱʱ�� = (WDSV * 256)/32768
**  ���ԣ���С���Ź���ʱ���� = 7.8125ms
**        ����Ź���ʱ���� = 512s	
*/
#define USE_WDT   1
#define WDT_TIME  256  //2S


/******************************************************************************/
/* ADC */
/*  
** Descriptions: ÿ����һ�ΰ���S2��nRF24LE1��VDD����һ�β�������������ֵͨ���������	
** ADC���ã�
**   ��׼��ѹ���ڲ�1.2V
**   ͨ�������1/3 VDD��ѹ
**   �ֱ��ʣ�12λ
**   ����ģʽ�����β���
**   �����ٶȣ�2ksps
**   ���ݶ��뷽ʽ���Ҷ���
** ���ڲ����ʣ�57600	
** ��ѹ���㹫ʽ��V =��1.2*3��*����ֵ/4096		
*/

typedef struct Tag_Information 
{
  union
  {
	  uint16_t id16;
	  uint8_t id8[2];
  }id;
  
  uint8_t   CellVoltageH;	  // ��ص�ѹ���ֽ�			
  uint8_t   CellVoltageL;		// ��ص�ѹ���ֽ�	

}TagInformation;

extern  xdata TagInformation TagInfo;


#endif
