#include "MT6AGCS.h"
#include <stdint.h>
#include "spi.h"
#include "string.h"
#include "main.h"
#include "calibration.h"

extern SENSOR_DATA sensor_data;	// 融合后传感器数据(主函数初始化)
extern IMU_ARRAY imu_array;		// n组IMU数据(主函数初始化)
 __attribute__((section (".RAM_D1"))) uint8_t  Send_data[256];
MT6_RawType  g_tMT6_Raw1[MT6AGCS_NUM];
MT6_DataType g_tMT6_Data1[MT6AGCS_NUM];
MT6_SumRawType g_tMT6_SumData[8] = {0};

uint8_t  g_ucTimeFlag = 0;
static uint32_t ui_FrameSum = 0;
static uint8_t  g_aucMt6_Cmd[2];
static MT6_Send_Data1 g_aucSend_Data1;
static MT6_Send_Data2 g_aucSend_Data2;
static MT6_Send_Data3 g_aucSend_Data3;

static SPI_HandleTypeDef* SPIn[MT6AGCS_NUM] = 
{
	&hspi6,
	&hspi2,
	&hspi3,
	&hspi4,
	&hspi6,
	&hspi2,
	&hspi3,
	&hspi4
	
};
static const uint16_t SPIn_CSn_Pin[MT6AGCS_NUM] = 
{
	SPI6_CSn1_Pin,
	SPI2_CSn1_Pin,
	SPI3_CSn1_Pin,
	SPI4_CSn1_Pin,
	SPI6_CSn2_Pin,
	SPI2_CSn2_Pin,
	SPI3_CSn2_Pin,
	SPI4_CSn2_Pin
};
static GPIO_TypeDef* SPIn_CSn_Port[MT6AGCS_NUM] = 
{
	
	SPI6_CSn1_GPIO_Port,
	SPI2_CSn1_GPIO_Port,
	SPI3_CSn1_GPIO_Port,
	SPI4_CSn1_GPIO_Port,
	SPI6_CSn2_GPIO_Port,
	SPI2_CSn2_GPIO_Port,
	SPI3_CSn2_GPIO_Port,
	SPI4_CSn2_GPIO_Port
};

static void MT6_WriteReg(SPI_HandleTypeDef *hspi, uint8_t ucReg, uint8_t ucData);
static uint8_t MT6_ReadReg(SPI_HandleTypeDef *hspi, uint8_t ucReg);
static void MT6_Read(SPI_HandleTypeDef *hspi, uint8_t ucReg, uint8_t ucLen, uint8_t *pData);
static void float_to_3byte(float input, uint8_t out[3]);
static uint8_t sum_bytes(uint32_t data) ;
static void MT6_SendData(uint8_t data[], uint8_t len);
/*============================================================================*/
/*============================================================================*/

static void MT6_WriteReg(SPI_HandleTypeDef *hspi, uint8_t ucReg, uint8_t ucData)
{
		g_aucMt6_Cmd[0] = ucReg & 0x7F;
		g_aucMt6_Cmd[1] = ucData;
		HAL_SPI_Transmit(hspi, g_aucMt6_Cmd ,2,100);
}

static void MT6_Read(SPI_HandleTypeDef *hspi, uint8_t ucReg, uint8_t ucLen, uint8_t *pData)
{
	uint8_t ucRetval[20];
	g_aucMt6_Cmd[0] = ucReg | 0x80;
	
	HAL_SPI_TransmitReceive(hspi, g_aucMt6_Cmd, ucRetval, 1, 100);
	HAL_SPI_TransmitReceive(hspi, ucRetval, pData, ucLen, 100);
	
}

void MT6_ReadData(void)
{
		uint8_t i = 0;
		memset(&g_tMT6_Raw1,0x00,sizeof(g_tMT6_Raw1));
	
//#if(1 == CONSENSUAL_CONTROL_FLAG)	
			
			for(i = 0; i < 8; i++)
			{
					HAL_GPIO_WritePin(SPIn_CSn_Port[i],SPIn_CSn_Pin[i], GPIO_PIN_RESET);
					MT6_Read( SPIn[i],GST_OUT_TEMP_L_REG, 8, g_tMT6_Raw1[i].aucBuffer );
					HAL_GPIO_WritePin(SPIn_CSn_Port[i],SPIn_CSn_Pin[i], GPIO_PIN_SET);
				
					g_tMT6_Raw1[i].sTempRaw = (int16_t)(g_tMT6_Raw1[i].aucBuffer[0] + (g_tMT6_Raw1[i].aucBuffer[1] << 8));
					g_tMT6_Raw1[i].sGyrXRaw = (int16_t)(g_tMT6_Raw1[i].aucBuffer[2] + (g_tMT6_Raw1[i].aucBuffer[3] << 8));
					g_tMT6_Raw1[i].sGyrYRaw = (int16_t)(g_tMT6_Raw1[i].aucBuffer[4] + (g_tMT6_Raw1[i].aucBuffer[5] << 8));
					g_tMT6_Raw1[i].sGyrZRaw = (int16_t)(g_tMT6_Raw1[i].aucBuffer[6] + (g_tMT6_Raw1[i].aucBuffer[7] << 8));

					g_tMT6_Data1[i].tData.fTempValue = (float)g_tMT6_Raw1[i].sTempRaw / 256 + 25;
					g_tMT6_Data1[i].tData.fGyrXValue = (float)g_tMT6_Raw1[i].sGyrXRaw / 57.143f;
					g_tMT6_Data1[i].tData.fGyrYValue = (float)g_tMT6_Raw1[i].sGyrYRaw / 57.143f;
					g_tMT6_Data1[i].tData.fGyrZValue = (float)g_tMT6_Raw1[i].sGyrZRaw / 57.143f;
				
			}


//#elif(2 == CONSENSUAL_CONTROL_FLAG)

//			for(i = 0; i < 8; i++)
//			{
//					HAL_GPIO_WritePin(SPIn_CSn_Port[i],SPIn_CSn_Pin[i], GPIO_PIN_RESET);
//					MT6_Read( SPIn[i],GST_OUT_TEMP_L_REG, 8, g_tMT6_Raw1[i].aucBuffer );
//					HAL_GPIO_WritePin(SPIn_CSn_Port[i],SPIn_CSn_Pin[i], GPIO_PIN_SET);
//			
//					g_tMT6_Raw1[i].sTempRaw = (int16_t)(g_tMT6_Raw1[i].aucBuffer[0] + (g_tMT6_Raw1[i].aucBuffer[1] << 8));
//					g_tMT6_Raw1[i].sGyrXRaw = (int16_t)(g_tMT6_Raw1[i].aucBuffer[2] + (g_tMT6_Raw1[i].aucBuffer[3] << 8));
//					g_tMT6_Raw1[i].sGyrYRaw = (int16_t)(g_tMT6_Raw1[i].aucBuffer[4] + (g_tMT6_Raw1[i].aucBuffer[5] << 8));
//					g_tMT6_Raw1[i].sGyrZRaw = (int16_t)(g_tMT6_Raw1[i].aucBuffer[6] + (g_tMT6_Raw1[i].aucBuffer[7] << 8));

//					
//					g_tMT6_Data1[i].tData.fTempValue = (float)g_tMT6_Raw1[i].sTempRaw / 256 + 25;
//					g_tMT6_Data1[i].tData.fGyrXValue = (float)g_tMT6_Raw1[i].sGyrXRaw / 57.143f;
//					g_tMT6_Data1[i].tData.fGyrYValue = (float)g_tMT6_Raw1[i].sGyrYRaw / 57.143f;
//					g_tMT6_Data1[i].tData.fGyrZValue = (float)g_tMT6_Raw1[i].sGyrZRaw / 57.143f;
//			}

//#elif(3 == CONSENSUAL_CONTROL_FLAG)

//			for(i = 0; i < 8; i++)
//			{
//					HAL_GPIO_WritePin(SPIn_CSn_Port[i],SPIn_CSn_Pin[i], GPIO_PIN_RESET);
//					MT6_Read( SPIn[i],GST_OUT_TEMP_L_REG, 14, g_tMT6_Raw1[i].aucBuffer );
//					HAL_GPIO_WritePin(SPIn_CSn_Port[i],SPIn_CSn_Pin[i], GPIO_PIN_SET);
//				
//					g_tMT6_Raw1[i].sTempRaw = (int16_t)(g_tMT6_Raw1[i].aucBuffer[0] + (g_tMT6_Raw1[i].aucBuffer[1] << 8));
//					g_tMT6_Raw1[i].sGyrXRaw = (int16_t)(g_tMT6_Raw1[i].aucBuffer[2] + (g_tMT6_Raw1[i].aucBuffer[3] << 8));
//					g_tMT6_Raw1[i].sGyrYRaw = (int16_t)(g_tMT6_Raw1[i].aucBuffer[4] + (g_tMT6_Raw1[i].aucBuffer[5] << 8));
//					g_tMT6_Raw1[i].sGyrZRaw = (int16_t)(g_tMT6_Raw1[i].aucBuffer[6] + (g_tMT6_Raw1[i].aucBuffer[7] << 8));
//					g_tMT6_Raw1[i].sAccXRaw = (int16_t)(g_tMT6_Raw1[i].aucBuffer[8] + (g_tMT6_Raw1[i].aucBuffer[9] << 8));
//					g_tMT6_Raw1[i].sAccYRaw = (int16_t)(g_tMT6_Raw1[i].aucBuffer[10] + (g_tMT6_Raw1[i].aucBuffer[11] << 8));
//					g_tMT6_Raw1[i].sAccZRaw = (int16_t)(g_tMT6_Raw1[i].aucBuffer[12] + (g_tMT6_Raw1[i].aucBuffer[13] << 8));
//				
//					g_tMT6_Data1[i].tData.fTempValue = (float)g_tMT6_Raw1[i].sTempRaw / 256 + 25;
//					g_tMT6_Data1[i].tData.fGyrXValue = (float)g_tMT6_Raw1[i].sGyrXRaw / 57.143f;
//					g_tMT6_Data1[i].tData.fGyrYValue = (float)g_tMT6_Raw1[i].sGyrYRaw / 57.143f;
//					g_tMT6_Data1[i].tData.fGyrZValue = (float)g_tMT6_Raw1[i].sGyrZRaw / 57.143f;
//			}


//#elif(4 == CONSENSUAL_CONTROL_FLAG)

//			for(i = 0; i < 8; i++)
//			{
//					HAL_GPIO_WritePin(SPIn_CSn_Port[i],SPIn_CSn_Pin[i], GPIO_PIN_RESET);
//					MT6_Read( SPIn[i],GST_OUT_TEMP_L_REG, 14, g_tMT6_Raw1[i].aucBuffer );
//					HAL_GPIO_WritePin(SPIn_CSn_Port[i],SPIn_CSn_Pin[i], GPIO_PIN_SET);
//			
//					g_tMT6_Raw1[i].sTempRaw = (int16_t)(g_tMT6_Raw1[i].aucBuffer[0] + (g_tMT6_Raw1[i].aucBuffer[1] << 8));
//					g_tMT6_Raw1[i].sGyrXRaw = (int16_t)(g_tMT6_Raw1[i].aucBuffer[2] + (g_tMT6_Raw1[i].aucBuffer[3] << 8));
//					g_tMT6_Raw1[i].sGyrYRaw = (int16_t)(g_tMT6_Raw1[i].aucBuffer[4] + (g_tMT6_Raw1[i].aucBuffer[5] << 8));
//					g_tMT6_Raw1[i].sGyrZRaw = (int16_t)(g_tMT6_Raw1[i].aucBuffer[6] + (g_tMT6_Raw1[i].aucBuffer[7] << 8));
//					g_tMT6_Raw1[i].sAccXRaw = (int16_t)(g_tMT6_Raw1[i].aucBuffer[8] + (g_tMT6_Raw1[i].aucBuffer[9] << 8));
//					g_tMT6_Raw1[i].sAccYRaw = (int16_t)(g_tMT6_Raw1[i].aucBuffer[10] + (g_tMT6_Raw1[i].aucBuffer[11] << 8));
//					g_tMT6_Raw1[i].sAccZRaw = (int16_t)(g_tMT6_Raw1[i].aucBuffer[12] + (g_tMT6_Raw1[i].aucBuffer[13] << 8));
//			
//					g_tMT6_Data1[i].tData.fTempValue = (float)g_tMT6_Raw1[i].sTempRaw / 256 + 25;
//					g_tMT6_Data1[i].tData.fGyrXValue = (float)g_tMT6_Raw1[i].sGyrXRaw / 57.143f;
//					g_tMT6_Data1[i].tData.fGyrYValue = (float)g_tMT6_Raw1[i].sGyrYRaw / 57.143f;
//					g_tMT6_Data1[i].tData.fGyrZValue = (float)g_tMT6_Raw1[i].sGyrZRaw / 57.143f;
//					g_tMT6_Data1[i].tData.fAccXValue = (float)g_tMT6_Raw1[i].sAccXRaw / 1875;
//					g_tMT6_Data1[i].tData.fAccYValue = (float)g_tMT6_Raw1[i].sAccYRaw / 1875;
//					g_tMT6_Data1[i].tData.fAccZValue = (float)g_tMT6_Raw1[i].sAccZRaw / 1875;
//			}
//	
//#endif
               

}




/*============================================================================*/
/*============================================================================*/

void MT6_Init(void)
{
	static uint8_t uc_RegData[7] = {0};
	
	for(uint8_t	i = 0; i < 8; i++)
	{
		
			HAL_GPIO_WritePin(SPIn_CSn_Port[i],SPIn_CSn_Pin[i], GPIO_PIN_SET);
		//
			HAL_GPIO_WritePin(SPIn_CSn_Port[i],SPIn_CSn_Pin[i], GPIO_PIN_RESET);
			MT6_WriteReg( SPIn[i],GST_A_ODR_CFG_REG, GST_ACC_ODR_3333Hz );
			HAL_GPIO_WritePin(SPIn_CSn_Port[i],SPIn_CSn_Pin[i], GPIO_PIN_SET);
		//
			HAL_GPIO_WritePin(SPIn_CSn_Port[i],SPIn_CSn_Pin[i], GPIO_PIN_RESET);
			MT6_WriteReg( SPIn[i],GST_G_ODR_CFG_REG, GST_GYR_ODR_3333Hz );
			HAL_GPIO_WritePin(SPIn_CSn_Port[i],SPIn_CSn_Pin[i], GPIO_PIN_SET);
		//
			HAL_GPIO_WritePin(SPIn_CSn_Port[i],SPIn_CSn_Pin[i], GPIO_PIN_RESET);
			MT6_WriteReg( SPIn[i],0x12, 0x44 );
			HAL_GPIO_WritePin(SPIn_CSn_Port[i],SPIn_CSn_Pin[i], GPIO_PIN_SET);
		//
			HAL_GPIO_WritePin(SPIn_CSn_Port[i],SPIn_CSn_Pin[i], GPIO_PIN_RESET);
			MT6_WriteReg( SPIn[i],0x13, 0x06 );
			HAL_GPIO_WritePin(SPIn_CSn_Port[i],SPIn_CSn_Pin[i], GPIO_PIN_SET);
		//
			HAL_GPIO_WritePin(SPIn_CSn_Port[i],SPIn_CSn_Pin[i], GPIO_PIN_RESET);
			MT6_WriteReg( SPIn[i],0x14, 0x60 );
			HAL_GPIO_WritePin(SPIn_CSn_Port[i],SPIn_CSn_Pin[i], GPIO_PIN_SET);
		//
			HAL_GPIO_WritePin(SPIn_CSn_Port[i],SPIn_CSn_Pin[i], GPIO_PIN_RESET);
			MT6_WriteReg( SPIn[i],GST_G_BW_CFG_REG, GST_GYR_BW_280HZ );
			HAL_GPIO_WritePin(SPIn_CSn_Port[i],SPIn_CSn_Pin[i], GPIO_PIN_SET);
		//
			HAL_GPIO_WritePin(SPIn_CSn_Port[i],SPIn_CSn_Pin[i], GPIO_PIN_RESET);
			MT6_WriteReg( SPIn[i],GST_A_BW_CFG_REG, GST_ACC_BW_ODR_DIV4 );
			HAL_GPIO_WritePin(SPIn_CSn_Port[i],SPIn_CSn_Pin[i], GPIO_PIN_SET);
			
			HAL_GPIO_WritePin(SPIn_CSn_Port[i],SPIn_CSn_Pin[i], GPIO_PIN_RESET);
			MT6_Read( SPIn[i],GST_A_ODR_CFG_REG,7,  uc_RegData);
			HAL_GPIO_WritePin(SPIn_CSn_Port[i],SPIn_CSn_Pin[i], GPIO_PIN_SET);		
	}
	
}

void Frame_Protocol_Packet_Send3(void)
{
	static uint8_t uc_PackCnt3 = 0;
	MT6_DataType g_tMT6_Data3 = {0};
	
	memset(&g_aucSend_Data3,0x00,sizeof(MT6_Send_Data3));
	memset(&Send_data,0x00,sizeof(MT6_Send_Data3));
	
	uc_PackCnt3++;
	
	if(uc_PackCnt3 >= Frequency_PACK)
	{
		uc_PackCnt3 = Frequency_PACK;
		g_aucSend_Data3.usSOF = 0x55AA;
		g_aucSend_Data3.usFrameCnt = ui_FrameSum++;
    
		g_tMT6_Data3.tData.fTempValue = sensor_data.T;
		g_tMT6_Data3.tData.fGyrXValue = sensor_data.gyro[0];
		g_tMT6_Data3.tData.fGyrYValue = sensor_data.gyro[1];
		g_tMT6_Data3.tData.fGyrZValue = sensor_data.gyro[2];

	
		for(uint8_t i = 0; i < 16; i++)
		{
			 g_aucSend_Data3.ucCheckSum += g_tMT6_Data3.aucBuffer[i];	
			 g_aucSend_Data3.data[i] = g_tMT6_Data3.aucBuffer[i];
		}
		
		g_aucSend_Data3.ucCheckSum += sum_bytes(g_aucSend_Data3.usFrameCnt);
		memcpy(Send_data,&g_aucSend_Data3,sizeof(g_aucSend_Data3));
		MT6_SendData(Send_data,sizeof(g_aucSend_Data3));
	}
}

void Frame_Protocol_Packet_Send2(void)
{
	static uint8_t uc_PackCnt2 = 0;
	memset(&g_aucSend_Data2,0x00,sizeof(MT6_Send_Data2));
	memset(&Send_data,0x00,sizeof(MT6_Send_Data2));
	
	uc_PackCnt2++;
	if(uc_PackCnt2 >= Frequency_PACK)
	{
		uc_PackCnt2 = Frequency_PACK;
		g_aucSend_Data2.usSOF = 0xAA55;
	
		float_to_3byte(sensor_data.gyro[0],&g_aucSend_Data2.aucBuffer[0]);
		float_to_3byte(sensor_data.gyro[1],&g_aucSend_Data2.aucBuffer[3]);
		float_to_3byte(sensor_data.gyro[2],&g_aucSend_Data2.aucBuffer[6]);
	
		for(uint8_t i = 0; i < 9; i++)
		{
			 g_aucSend_Data2.ucCheck ^= g_aucSend_Data2.aucBuffer[i];	

		}
		g_aucSend_Data2.ucFrameNumber =(uint8_t) ui_FrameSum++;
		g_aucSend_Data2.ucCheck ^= g_aucSend_Data2.ucFrameNumber;
		g_aucSend_Data2.ucSOE = 0xF0;
		memcpy(Send_data,&g_aucSend_Data2,sizeof(g_aucSend_Data2));
		MT6_SendData(Send_data,sizeof(g_aucSend_Data2));
	}
}
void Frame_Protocol_Packet_Send1(void)
{
//#if(1 == CONSENSUAL_CONTROL_FLAG)	
	
	static uint8_t uc_PackCnt = 0;
	static uint8_t uc_ImuCnt = 0;
	
	memset(&g_aucSend_Data1,0x00,sizeof(MT6_Send_Data1));
	memset(&Send_data,0x00,sizeof(MT6_Send_Data1));

	if(AddSample()  ==  1)
	{
			uc_PackCnt++;
		  memcpy(g_aucSend_Data1.data,&g_tMT6_SumData,sizeof(g_tMT6_SumData));
//		if(uc_PackCnt >= Frequency_PACK)
		{
				uc_PackCnt = Frequency_PACK;
				g_aucSend_Data1.usSOF = 0x55AA;
				g_aucSend_Data1.usFrameCnt = ui_FrameSum++;
				
				for(uint8_t i = 0; i < 10*8; i++)
				{
					g_aucSend_Data1.ucCheckSum += g_aucSend_Data1.data[i];	
				}
					
				g_aucSend_Data1.ucCheckSum += sum_bytes(g_aucSend_Data1.usFrameCnt);
				memcpy(Send_data,&g_aucSend_Data1,sizeof(g_aucSend_Data1));
				MT6_SendData(Send_data,sizeof(g_aucSend_Data1));

				memset(&g_tMT6_SumData,0x00,sizeof(g_tMT6_SumData));
				
		}
	
	}	
	

//#elif(4 == CONSENSUAL_CONTROL_FLAG)

//	static uint8_t uc_PackCnt = 0;
//	MT6_DataType g_tMT6_Data = {0};
//	
//	memset(&g_aucSend_Data,0x00,sizeof(MT6_Send_Data));
//	memset(&Send_data,0x00,sizeof(MT6_Send_Data));
//	
//	uc_PackCnt++;
//	
//	if(uc_PackCnt >= Frequency_PACK)
//	{
//		uc_PackCnt = Frequency_PACK;
//		g_aucSend_Data.usSOF = 0x55AA;
//		g_aucSend_Data.usFrameCnt = ++ui_FrameSum;
//    	
//		memcpy(g_aucSend_Data.data,&g_tMT6_Data1,sizeof(g_tMT6_Data1));
//		
//		for(uint8_t i = 0; i < 224; i++)
//		{
//			 g_aucSend_Data.ucCheckSum += g_aucSend_Data.data[i];
//		}
//		
//		g_aucSend_Data.ucCheckSum += sum_bytes(g_aucSend_Data.usFrameCnt);
//		memcpy(Send_data,&g_aucSend_Data,sizeof(g_aucSend_Data));
//		MT6_SendData(Send_data,sizeof(g_aucSend_Data));
//	}	
//#endif	
}
void float_to_3byte(float input, uint8_t out[3]) 
{
   
    int32_t scaled = (int32_t)(round(input * 10000.0f));  // 保留4位小数，四舍五入

    // 保留低24位
    uint32_t encoded = (uint32_t)(scaled & 0xFFFFFF);  // 取低24位

    //拆成3个字节（高位在前）
    out[0] = (encoded >> 16) & 0xFF;  // 高字节
    out[1] = (encoded >> 8) & 0xFF;   // 中字节
    out[2] = encoded & 0xFF;          // 低字节
}
uint8_t sum_bytes(uint32_t data) 
{
    uint8_t byte1 = (data & 0xFF);         // 获取最低字节
    uint8_t byte2 = ((data >> 8) & 0xFF);  // 获取第二字节
    uint8_t byte3 = ((data >> 16) & 0xFF); // 获取第三字节
    uint8_t byte4 = ((data >> 24) & 0xFF); // 获取最高字节
    
    //相加
    uint8_t sum = byte1 + byte2 + byte3 + byte4;

    return sum;
}

uint8_t AddSample(void)
{
    static uint8_t Sample_Cnt = 0;
    uint8_t uc_Result = 0;

    // 累加采样
    if (Sample_Cnt < MAX_SAMPLE_COUNT)
    {
        for (uint8_t i = 0; i < 8; i++)
        {
            g_tMT6_SumData[i].sTempRaw += (int32_t)g_tMT6_Raw1[i].sTempRaw;
            g_tMT6_SumData[i].sGyrXRaw += g_tMT6_Raw1[i].sGyrXRaw;
            g_tMT6_SumData[i].sGyrYRaw += g_tMT6_Raw1[i].sGyrYRaw;
            g_tMT6_SumData[i].sGyrZRaw += g_tMT6_Raw1[i].sGyrZRaw;
        }
        Sample_Cnt++;
    }

    // 到达采样次数，计算平均并清空
    if (Sample_Cnt == MAX_SAMPLE_COUNT)
    {
        for (uint8_t i = 0; i < 8; i++)
        {
            g_tMT6_SumData[i].sTempRaw /= 1;
            g_tMT6_SumData[i].sGyrXRaw /= 1;
            g_tMT6_SumData[i].sGyrYRaw /= 1;
            g_tMT6_SumData[i].sGyrZRaw /= 1;
        }

        Sample_Cnt = 0;     // 重置采样计数器
        uc_Result = 1;      // 表示平均完成
    }

    return uc_Result;
}

