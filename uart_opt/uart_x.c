#include "uart_x.h"
#include "usart.h"
#include "stmflash.h"
#include "calibration.h"
#include <stdio.h>
#define FRAME_BUF_LEN 		10
extern CALIB_PARA  calib_para;
uint8_t uart_rx_byte = 0;
uint8_t frame_buf[FRAME_BUF_LEN] = {0};
uint8_t frame_index = 0;
recv_state state = 0;
uint32_t ul_CmdData = 0;

uint8_t uc_cmd = 0;
void MT6_SendData(uint8_t data[], uint8_t len)
{
			HAL_UART_Transmit_DMA(&huart1,data, len);
//		HAL_UART_Transmit_IT(&huart1,Send_data, sizeof(g_aucSend_Data));
}

void process_command_frame(uint8_t *frame, uint8_t len)
{
    uint8_t uc_Type = frame[1];  // 第一个是cmd，第二个是type
		uint8_t uc_data[20] = {0};
		uint8_t uc_cnt = 0;
		uc_data[uc_cnt++] = 0xAA;
		uc_data[uc_cnt++] = 0x55;
		uc_data[uc_cnt++] = frame[0];
		uc_data[uc_cnt++] = frame[1];

		switch (frame[0])
    {
        case 0x0A:
            // 
						if(0x01 == frame[1])
						{
								uc_data[uc_cnt++] = 0x01;
								ul_CmdData = 0;
								HAL_UART_Transmit_DMA(&huart1,uc_data, 5);
								STMFLASH_Write(ADDR_FLASH_SECTOR_2_BANK1,(uint32_t *)frame,1);	
						}
						else if(0x02 == frame[1])
						{
								uc_data[uc_cnt++] = 0x01;
								ul_CmdData = 0;
								HAL_UART_Transmit_DMA(&huart1,uc_data, 5);
								STMFLASH_Write(ADDR_FLASH_SECTOR_2_BANK1,(uint32_t *)frame,1);
						}
						else if(0x03 == frame[1])
						{
								uc_data[uc_cnt++] = 0x01;
								ul_CmdData = 0;
								HAL_UART_Transmit_DMA(&huart1,uc_data, 5);
								STMFLASH_Write(ADDR_FLASH_SECTOR_2_BANK1,(uint32_t *)frame,1);
						}
            break;

        case 0x0B:
					
						if(0x01 == frame[1])
						{
								uc_data[uc_cnt++] = 0x01;
								ul_CmdData = 0;
								sprintf((char* )&uc_data[uc_cnt++], "%f", calib_para.Version);
								HAL_UART_Transmit_DMA(&huart1,uc_data, 12);
						}
            break;

        default:
            // 未知命令
            break;
    }
		

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        switch (state)
        {
            case WAIT_HEAD1:
                if (uart_rx_byte == 0xAA) 
								{
                    state = WAIT_HEAD2;
                    frame_index = 0;
                }
                break;

            case WAIT_HEAD2:
                if (uart_rx_byte == 0x55) 
								{
                    state = WAIT_CMD;
                } 
								else 
								{
                    state = WAIT_HEAD1;  // 错误恢复
                }
                break;

            case WAIT_CMD:
                uc_cmd = uart_rx_byte;
                if ((0x0A != uc_cmd) && (0x0B != uc_cmd)) 
								{
                    state = WAIT_HEAD1;  // 
                } 
								else 
								{
                    state = WAIT_DATA;
                    frame_buf[frame_index++] = uc_cmd;
                }
                break;

            case WAIT_DATA:
                frame_buf[frame_index++] = uart_rx_byte;
                if (frame_index >= 2) 
								{
                    
                    process_command_frame(frame_buf, 2);
                    state = WAIT_HEAD1;
                    frame_index = 0;
                }
                break;
        }

        // 接收下一个字节
        HAL_UART_Receive_IT(&huart1, &uart_rx_byte, 1);
    }
}

