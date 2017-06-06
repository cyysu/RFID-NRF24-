#ifndef CONFIG_H__
#define CONFIG_H__

/* Reader parm  */
#define  READER_ADDR       0x01	 // READER地址
#define  READER_CH         50	   // 信道
#define  RX_PAYLOAD_LEN    6     // 接收数据长度




/* 定时器  */
#define TIMER0_VALUE  26666


/* 标签上报命令表 */
#define CMD_TAG_REPID      0x1E  //
#define MAX_TAG_BUFNUM     60

#define TAG_NEED_REP       0x01

/* 接收器上报信息  */
#define MAX_REP_IDNUM       20   //每次上报标签最大数量 
#define DEST_ADDR           0x01 //目的地址
#define FRAME_FIR_HEAD      0x33 
#define FRAME_SEC_HEAD      0xCC

#define REP_ID_INFO         0x20
#define ID_OVERTIME         10   //ID缓存超时时间

/*
** 看门狗超时时间计算	
**  超时时间 = (WDSV * 256)/32768
**  所以：最小看门狗超时周期 = 7.8125ms
**        最大看门狗超时周期 = 512s	
*/
//#define USE_WDT   1
#define WDT_TIME  256  //2S
/* 调试信息 */
#define  DEBUG_LED    1
//#define  DEBUG_UART   1
typedef struct Reader_Information 
{
  uint8_t   addr;     // 接收器地址
  uint8_t   pwr;	    // 发射功率			
  uint8_t   ch;		    // 信道
  uint16_t  Interval;	// 发射间隔
	
	uint8_t   uartcnt;
	uint8_t   uartstartcnt;
	uint8_t   uartflag;
	uint8_t   uartsuccess;

}ReaderInformation;

extern  xdata ReaderInformation ReaderInfo;


/*-------------------管脚定义--------------------------------------------------*/
#define  D4    P00  // 开发板上的指示灯D4
#define  D5    P01  // 开发板上的指示灯D5
#define  S2    P12  // 开发板上的按键S2
#define  S3    P13  // 开发板上的按键S3
#define  ADC   P06

/* 本例程中nRF24LE1管脚配置
P00：输出，驱动D4	         P12：输入，按键检测S2  
P01：输出，驱动D5			 P13：输入，按键检测S3
P02：输出					 P14：输出
P03：输出，UART TXD			 P15：输出
P04：输入，UART RXD			 P16：输出
P06：输入，AIN6  AD检测
*/

#endif