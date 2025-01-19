//
// Created by 刘嘉俊 on 25-1-9.
//

#include "usart2_send.h"
#include "stm32f4xx_hal.h"
#include "main.h"

extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart2_tx;

// 数据帧长度 (帧头 + 数据区(6 * 4字节) + 校验 + 帧尾)
#define FRAME_SIZE 28

uint8_t dma_tx_buffer[2][FRAME_SIZE]; // 双缓冲区
static uint8_t frame_seq = 0; // 帧序号
uint8_t current_buffer = 0;           // 当前缓冲区索引
volatile uint8_t dma_busy = 0;        // DMA状态标志位

extern QueueHandle_t xQueue;    // FreeRTOS 队列句柄
static float encoder_values[6] = {0, 0, 0, 0, 0, 0};              // 存储6个编码器值
static float angles[6] = {0.0f}; // 用于临时存储队列中读取的编码器值

void PackData(float *values, uint8_t *buffer)
{
    uint8_t i;
    uint8_t checksum = 0;

    // 帧头
    buffer[0] = 0xAA;

    // 帧序号
    buffer[1] = frame_seq++;
    if (frame_seq > 255) frame_seq = 0;

    // 数据区：6个float（每个编码器值4字节）
    for (i = 0; i < 6; i++) {
        uint8_t *p = (uint8_t *)&values[i];
        buffer[2 + i * 4] = p[0];
        buffer[3 + i * 4] = p[1];
        buffer[4 + i * 4] = p[2];
        buffer[5 + i * 4] = p[3];
    }

    // 校验和 (仅计算帧序号和数据区)
    for (i = 1; i <= 25; i++) {
        checksum += buffer[i];
    }
    buffer[26] = checksum;

    // 帧尾
    buffer[27] = 0x55;
}

void DMA_Send_Frame(void)
{
    if (dma_busy == 0) { // 判断DMA是否空闲
        dma_busy = 1;    // 标志位置为忙

        // 启动DMA发送
       HAL_UART_Transmit_DMA(&huart2, dma_tx_buffer[current_buffer], FRAME_SIZE);

        // 切换到另一个缓冲区
        current_buffer = (current_buffer == 0) ? 1 : 0;
    }
}

// DMA发送完成回调函数
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
        dma_busy = 0; // 标志位置为空闲
    }
}

void USART2_SendEntry(void const * argument)
{
    /* USER CODE BEGIN USART2_SendEntry */

    /* Infinite loop */
    for(;;)
    {
        // 从队列中获取编码器值（阻塞时间为 10ms，如果队列为空，则跳过）
        if (xQueueReceive(xQueue, angles, pdMS_TO_TICKS(10)) == pdPASS) {
            // 更新全局的 encoder_values 数组（可选）
            for (int i = 0; i < 6; i++) {
                encoder_values[i] = angles[i];
            }

            // 将编码器值打包到当前缓冲区
            PackData(encoder_values, dma_tx_buffer[current_buffer]);

            // 启动 DMA 发送
            DMA_Send_Frame();
        }

        vTaskDelay(10); // 延迟 10ms

    }
    /* USER CODE END USART2_SendEntry */
}

