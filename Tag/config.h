#ifndef CONFIG_H__
#define CONFIG_H__

/******************************************************************************/
/* 调试信息*/

#define  DEBUG_LED    1


/******************************************************************************/
/* Tag 参数  */
#define  TAG_ID          1000	    // ID
#define  TAG_CH          50	      // 信道
#define  TAG_TIME        32768U	  // 休眠时间：1000ms

#define  MAX_TX_PAYLOAD  6        // 无线发送数据长度
#define  TAG_ID_LEN      2        // ID字节数
#define  ADC_TIME        7200U      // ADC检测时间间隔

/******************************************************************************/
/* Watchdog*/
/*
** 看门狗超时时间计算	
**  超时时间 = (WDSV * 256)/32768
**  所以：最小看门狗超时周期 = 7.8125ms
**        最大看门狗超时周期 = 512s	
*/
#define USE_WDT   1
#define WDT_TIME  256  //2S


/******************************************************************************/
/* ADC */
/*  
** Descriptions: 每按下一次按键S2，nRF24LE1对VDD进行一次采样，并将采样值通过串口输出	
** ADC配置：
**   基准电压：内部1.2V
**   通道：检测1/3 VDD电压
**   分辨率：12位
**   采样模式：单次采样
**   采样速度：2ksps
**   数据对齐方式：右对齐
** 串口波特率：57600	
** 电压计算公式：V =（1.2*3）*采样值/4096		
*/

typedef struct Tag_Information 
{
  union
  {
	  uint16_t id16;
	  uint8_t id8[2];
  }id;
  
  uint8_t   CellVoltageH;	  // 电池电压高字节			
  uint8_t   CellVoltageL;		// 电池电压低字节	

}TagInformation;

extern  xdata TagInformation TagInfo;


#endif
