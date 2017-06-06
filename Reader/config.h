#ifndef CONFIG_H__
#define CONFIG_H__

/* Reader parm  */
#define  READER_ADDR       0x01	 // READER��ַ
#define  READER_CH         50	   // �ŵ�
#define  RX_PAYLOAD_LEN    6     // �������ݳ���




/* ��ʱ��  */
#define TIMER0_VALUE  26666


/* ��ǩ�ϱ������ */
#define CMD_TAG_REPID      0x1E  //
#define MAX_TAG_BUFNUM     60

#define TAG_NEED_REP       0x01

/* �������ϱ���Ϣ  */
#define MAX_REP_IDNUM       20   //ÿ���ϱ���ǩ������� 
#define DEST_ADDR           0x01 //Ŀ�ĵ�ַ
#define FRAME_FIR_HEAD      0x33 
#define FRAME_SEC_HEAD      0xCC

#define REP_ID_INFO         0x20
#define ID_OVERTIME         10   //ID���泬ʱʱ��

/*
** ���Ź���ʱʱ�����	
**  ��ʱʱ�� = (WDSV * 256)/32768
**  ���ԣ���С���Ź���ʱ���� = 7.8125ms
**        ����Ź���ʱ���� = 512s	
*/
//#define USE_WDT   1
#define WDT_TIME  256  //2S
/* ������Ϣ */
#define  DEBUG_LED    1
//#define  DEBUG_UART   1
typedef struct Reader_Information 
{
  uint8_t   addr;     // ��������ַ
  uint8_t   pwr;	    // ���书��			
  uint8_t   ch;		    // �ŵ�
  uint16_t  Interval;	// ������
	
	uint8_t   uartcnt;
	uint8_t   uartstartcnt;
	uint8_t   uartflag;
	uint8_t   uartsuccess;

}ReaderInformation;

extern  xdata ReaderInformation ReaderInfo;


/*-------------------�ܽŶ���--------------------------------------------------*/
#define  D4    P00  // �������ϵ�ָʾ��D4
#define  D5    P01  // �������ϵ�ָʾ��D5
#define  S2    P12  // �������ϵİ���S2
#define  S3    P13  // �������ϵİ���S3
#define  ADC   P06

/* ��������nRF24LE1�ܽ�����
P00�����������D4	         P12�����룬�������S2  
P01�����������D5			 P13�����룬�������S3
P02�����					 P14�����
P03�������UART TXD			 P15�����
P04�����룬UART RXD			 P16�����
P06�����룬AIN6  AD���
*/

#endif