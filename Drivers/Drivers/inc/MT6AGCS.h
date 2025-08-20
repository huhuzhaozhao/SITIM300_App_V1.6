#ifndef __MT6AGCS_H_
#define __MT6AGCS_H_

#include "MT6AGCS.h"
#include "main.h"
#define MAX_SAMPLE_COUNT 							10  	// 实际采样次数
#define CONSENSUAL_CONTROL_FLAG 			1    	//1:温度陀螺协议（1k采集原始数据取10次累加100hz发出）   int16
																						//2：客户协议 2k 			需要把算法的融合打开							float
																						//3：温度陀螺协议2k 	需要把算法的融合打开						    float
																						//4：温度加计陀螺原始数据协议（选择4的时候输出频率必须配置为100hz）float

#define FREQUENCY_XX_HZ_PACK					2			//1:2000HZ   2：1000HZ  3：100HZ

#if(1 == FREQUENCY_XX_HZ_PACK)

#define Frequency_PACK								200		// 200帧以后在发送第一包
#elif(2 == FREQUENCY_XX_HZ_PACK)
#define Frequency_PACK								100		//100帧以后在发送第一包
#elif(3 == FREQUENCY_XX_HZ_PACK)
#define Frequency_PACK								10		//10帧以后在发送第一包

#endif

#define MT6AGCS_NUM										8
#define GST_SPI2_CSn1_SELECT()        HAL_GPIO_WritePin(SPI2_CSn1_GPIO_Port, SPI2_CSn1_Pin,GPIO_PIN_RESET)
#define GST_SPI2_CSn1_DESELECT()      HAL_GPIO_WritePin(SPI2_CSn1_GPIO_Port,SPI2_CSn1_Pin, GPIO_PIN_SET)

#define GST_SPI2_CSn2_SELECT()        HAL_GPIO_WritePin(SPI2_CSn2_GPIO_Port, SPI2_CSn2_Pin,GPIO_PIN_RESET)
#define GST_SPI2_CSn2_DESELECT()      HAL_GPIO_WritePin(SPI2_CSn2_GPIO_Port, SPI2_CSn2_Pin,GPIO_PIN_SET)

#define GST_SPI3_CSn1_SELECT()        HAL_GPIO_WritePin(SPI3_CSn1_GPIO_Port, SPI3_CSn1_Pin,GPIO_PIN_RESET)
#define GST_SPI3_CSn1_DESELECT()      HAL_GPIO_WritePin(SPI3_CSn1_GPIO_Port, SPI3_CSn1_Pin,GPIO_PIN_SET)

#define GST_SPI3_CSn2_SELECT()        HAL_GPIO_WritePin(SPI3_CSn2_GPIO_Port, SPI3_CSn2_Pin,GPIO_PIN_RESET)
#define GST_SPI3_CSn2_DESELECT()      HAL_GPIO_WritePin(SPI3_CSn2_GPIO_Port, SPI3_CSn2_Pin,GPIO_PIN_SET)

#define GST_SPI4_CSn1_SELECT()        HAL_GPIO_WritePin(SPI4_CSn1_GPIO_Port, SPI4_CSn1_Pin,GPIO_PIN_RESET)
#define GST_SPI4_CSn1_DESELECT()      HAL_GPIO_WritePin(SPI4_CSn1_GPIO_Port, SPI4_CSn1_Pin,GPIO_PIN_SET)

#define GST_SPI4_CSn2_SELECT()        HAL_GPIO_WritePin(SPI4_CSn2_GPIO_Port, SPI4_CSn2_Pin,GPIO_PIN_RESET)
#define GST_SPI4_CSn2_DESELECT()      HAL_GPIO_WritePin(SPI4_CSn2_GPIO_Port, SPI4_CSn2_Pin,GPIO_PIN_SET)

#define GST_SPI6_CSn1_SELECT()        HAL_GPIO_WritePin(SPI6_CSn1_GPIO_Port, SPI6_CSn1_Pin,GPIO_PIN_RESET)
#define GST_SPI6_CSn1_DESELECT()      HAL_GPIO_WritePin(SPI6_CSn1_GPIO_Port, SPI6_CSn1_Pin,GPIO_PIN_SET)

#define GST_SPI6_CSn2_SELECT()        HAL_GPIO_WritePin(SPI6_CSn2_GPIO_Port, SPI6_CSn2_Pin,GPIO_PIN_RESET)
#define GST_SPI6_CSn2_DESELECT()      HAL_GPIO_WritePin(SPI6_CSn2_GPIO_Port, SPI6_CSn2_Pin,GPIO_PIN_SET)

#define GST_A_ODR_CFG_REG           0x10
#define GST_G_ODR_CFG_REG           0x11
#define GST_G_BW_CFG_REG            0x15
#define GST_A_BW_CFG_REG            0x17

#define GST_OUT_TEMP_L_REG          0x20
#define GST_OUT_TEMP_H_REG          0x21
#define GST_OUT_GYR_X_L_REG         0x22
#define GST_OUT_GYR_X_H_REG         0x23
#define GST_OUT_GYR_Y_L_REG         0x24
#define GST_OUT_GYR_Y_H_REG         0x25
#define GST_OUT_GYR_Z_L_REG         0x26
#define GST_OUT_GYR_Z_H_REG         0x27
#define GST_OUT_ACC_X_L_REG         0x28
#define GST_OUT_ACC_X_H_REG         0x29
#define GST_OUT_ACC_Y_L_REG         0x2A
#define GST_OUT_ACC_Y_H_REG         0x2B
#define GST_OUT_ACC_Z_L_REG         0x2C
#define GST_OUT_ACC_Z_H_REG         0x2D

#define GST_ACC_ODR_1667Hz          (0x80 | 0x06)
#define GST_ACC_ODR_3333Hz          (0x90 | 0x06)
#define GST_ACC_ODR_6667Hz          (0xA0 | 0x06)
								    
#define GST_GYR_ODR_1667Hz          (0x80 | 0x04)
#define GST_GYR_ODR_3333Hz          (0x90 | 0x04)
#define GST_GYR_ODR_6667Hz          (0xA0 | 0x04)

#define GST_GYR_BW_280HZ            (0x00)
#define GST_GYR_BW_220HZ            (0x01)
#define GST_GYR_BW_150HZ            (0x02)
#define GST_GYR_BW_390_460HZ        (0x03)

#define GST_ACC_BW_ODR_DIV4         (0x00 << 5)
#define GST_ACC_BW_ODR_DIV10        (0x01 << 5)
#define GST_ACC_BW_ODR_DIV20        (0x02 << 5)
#define GST_ACC_BW_ODR_DIV45        (0x03 << 5)
#define GST_ACC_BW_ODR_DIV100       (0x04 << 5)
#define GST_ACC_BW_ODR_DIV200       (0x05 << 5)
#define GST_ACC_BW_ODR_DIV400       (0x06 << 5)
#define GST_ACC_BW_ODR_DIV800       (0x07 << 5)

#pragma pack(1)
typedef struct
{
	volatile int32_t  sTempRaw;
	volatile int16_t  sGyrXRaw;
	volatile int16_t  sGyrYRaw;
	volatile int16_t  sGyrZRaw;
    volatile int16_t  sAccXRaw;
	volatile int16_t  sAccYRaw;
	volatile int16_t  sAccZRaw;
}MT6_SumRawType;

typedef struct
{
	uint8_t  aucBuffer[14];
	
	volatile int16_t  sTempRaw;
	volatile int16_t  sGyrXRaw;
	volatile int16_t  sGyrYRaw;
	volatile int16_t  sGyrZRaw;
	volatile int16_t  sAccXRaw;
	volatile int16_t  sAccYRaw;
	volatile int16_t  sAccZRaw;
}MT6_RawType;

#pragma pack(4)
#if(1 == CONSENSUAL_CONTROL_FLAG)
//typedef union
//{
//	struct
//	{
//		float     fGyrXValue;
//		float     fGyrYValue;
//		float     fGyrZValue;
//		float     fTempValue;
//	}tData;
//	
//	uint8_t aucBuffer[16];
//}MT6_DataType;
typedef union
{
	struct
	{
		float     fGyrXValue;
		float     fGyrYValue;
		float     fGyrZValue;
		float     fAccXValue;
		float     fAccYValue;
		float     fAccZValue;
		float     fTempValue;
	}tData;
	
	uint8_t aucBuffer[28];
}MT6_DataType;
#elif(2 == CONSENSUAL_CONTROL_FLAG)
typedef union
{
	struct
	{
		float     fGyrXValue;
		float     fGyrYValue;
		float     fGyrZValue;
		float     fTempValue;
	}tData;
	
	uint8_t aucBuffer[16];
}MT6_DataType;
#elif(3 == CONSENSUAL_CONTROL_FLAG)
typedef union
{
	struct
	{
		float     fGyrXValue;
		float     fGyrYValue;
		float     fGyrZValue;
		float     fTempValue;
	}tData;
	
	uint8_t aucBuffer[16];
}MT6_DataType;

#elif(4 == CONSENSUAL_CONTROL_FLAG)
typedef union
{
	struct
	{
		float     fGyrXValue;
		float     fGyrYValue;
		float     fGyrZValue;
		float     fAccXValue;
		float     fAccYValue;
		float     fAccZValue;
		float     fTempValue;
	}tData;
	
	uint8_t aucBuffer[28];
}MT6_DataType;

#endif
typedef union
{
	struct
	{
		float     fGyrXValue;
		float     fGyrYValue;
		float     fGyrZValue;
	}tData;
	
	uint8_t aucBuffer[12];
}MT6_DataType1;

#pragma pack(1)
//#if(1 == CONSENSUAL_CONTROL_FLAG)
typedef struct
{
	uint16_t  usSOF;
	uint32_t  usFrameCnt;
	uint8_t   data[16*8];
	uint8_t   ucCheckSum;
}MT6_Send_Data1;
//#elif(2 == CONSENSUAL_CONTROL_FLAG)
typedef struct
{
	uint16_t  usSOF;	
	uint8_t		aucBuffer[9];
	uint8_t		ucReserve[2];	
	uint8_t		ucFrameNumber;	
	uint8_t   ucCheck;
	uint8_t  	ucSOE;
	
}MT6_Send_Data2;

//#elif(3 == CONSENSUAL_CONTROL_FLAG)
typedef struct
{
	uint16_t  usSOF;
	uint32_t  usFrameCnt;
	uint8_t   data[28];
	uint8_t   ucCheckSum;
}MT6_Send_Data3;

//#elif(4 == CONSENSUAL_CONTROL_FLAG)
//typedef struct
//{
//	uint16_t  usSOF;
//	uint32_t  usFrameCnt;
//	uint8_t   data[224];
//	uint8_t   ucCheckSum;
//}MT6_Send_Data;
//#endif

#pragma pack()

extern uint8_t  g_ucTimeFlag;

extern void MT6_Init(void);
extern void MT6_ReadData(void);
extern void MT6_ReadDataTest(void);
extern void Frame_Protocol_Packet_Send1(void);
extern void Frame_Protocol_Packet_Send2(void);
extern void Frame_Protocol_Packet_Send3(void);
extern uint8_t AddSample();

#endif /* __GST30_H_ */

