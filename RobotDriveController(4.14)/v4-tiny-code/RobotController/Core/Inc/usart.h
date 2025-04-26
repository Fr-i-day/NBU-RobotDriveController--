/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    usart.h
 * @brief   This file contains all the function prototypes for
 *          the usart.c file
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern UART_HandleTypeDef huart2;

extern UART_HandleTypeDef huart3;

/* USER CODE BEGIN Private defines */

#define FRAME_BYTE_LENGTH 9  /* 帧长度,包含帧头和帧尾,第2位控制方向，第3位控制步进电机 */
#define FRAME_START 0x42    /* 起始帧格式 */
#define FRAME_END 0x5A      /* 结束帧格式 */

/* 速度档位掩码和偏移 (高2位) */
#define SPEED_MASK 0xC0 /* 11000000b */
#define SPEED_SHIFT 6

/* 速度档位定义 */
#define SPEED_STOP 0x00 /* 00b: 停止 */
#define SPEED_LOW 0x01  /* 01b: 50速度 */
#define SPEED_MID 0x02  /* 10b: 100速度 */
#define SPEED_HIGH 0x03 /* 11b: 200速度 */

/* 底盘旋转角度掩码和偏移 (次2位) */
#define ROTATION_MASK 0x30 /* 00110000b */
#define ROTATION_SHIFT 4

/* 底盘旋转角速度定义 */
#define ROT_NONE 0x00 /* 00b: 不旋转 */
#define ROT_30 0x01   /* 01b: 旋转角速度30 */
#define ROT_N30 0x02  /* 10b: 旋转角速度-30度 */
#define ROT_45 0x03  /* 11b: 旋转角速度45 */

/* 运动角度掩码 (低4位) */
#define ANGLE_MASK 0x0F    /* 00001111b */

/* 运动角度定义 (举例) */
#define ANGLE_0 0x00   /* 0000b: 0度(向前) */
#define ANGLE_45 0x01  /* 0001b: 45度 */
#define ANGLE_90 0x02  /* 0010b: 90度 */
#define ANGLE_135 0x03 /* 0011b: 135度 */
#define ANGLE_180 0x04 /* 0100b: 180度 */
#define ANGLE_225 0x05 /* 0101b: -135度 */
#define ANGLE_270 0x06 /* 0110b: -90度 */
#define ANGLE_315 0x07 /* 0111b: -45度 */

#define MOTOR_FORWARD 0x42 /* 前进 */
#define MOTOR_BACK 0x46    /* 后退 */
#define MOTOR_LEFT 0x4C    /* 左转 */
#define MOTOR_RIGHT 0x52   /* 右转 */
#define MOTOR_STOP 0x00    /* 停止 */

/* 步进电机角度控制命令 */
#define STEPPER_10_DEG 0x01   /* 升降10度 */
#define STEPPER_180_DEG 0x02  /* 升降180度 */
#define STEPPER_360_DEG 0x03  /* 升降360度 */
#define STEPPER_1080_DEG 0x04 /* 升降1080度 */
    typedef struct
    {
        char RxBuffer[FRAME_BYTE_LENGTH]; /* 接收缓冲区 */
        uint8_t aRxBuffer;                /* 接收缓冲 */
        uint8_t Rx_Cnt;                   /* 接收缓冲计数 */
        uint8_t USART_FrameFlag;          /* 接收完整数据帧标志，1完整，0不完整 */
    } UartStruct;

    extern UartStruct uart2Data; /* USART2的数据结构体 */
    extern UartStruct uart3Data; /* USART3的数据结构体 */

    extern uint8_t uart3_rx;
    extern uint8_t uart2_rx;

    //void USART2_Init(void);
    //  void USART_OUT(USART_TypeDef* USARTx, uint8_t *Data,...);
    //  char *itoa(int value, char *string, int radix);
    //  int fputc(int ch, FILE *f);

void USART_GetChar(UartStruct *Uartn, uint8_t nChar); /* 串口接收到一个字节 */
void USART_Process(void);

/* USER CODE END Private defines */

void MX_USART2_UART_Init(void);
void MX_USART3_UART_Init(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

