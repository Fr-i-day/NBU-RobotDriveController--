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

#define FRAME_BYTE_LENGTH 9  /* ֡����,����֡ͷ��֡β,��2λ���Ʒ��򣬵�3λ���Ʋ������ */
#define FRAME_START 0x42    /* ��ʼ֡��ʽ */
#define FRAME_END 0x5A      /* ����֡��ʽ */

/* �ٶȵ�λ�����ƫ�� (��2λ) */
#define SPEED_MASK 0xC0 /* 11000000b */
#define SPEED_SHIFT 6

/* �ٶȵ�λ���� */
#define SPEED_STOP 0x00 /* 00b: ֹͣ */
#define SPEED_LOW 0x01  /* 01b: 50�ٶ� */
#define SPEED_MID 0x02  /* 10b: 100�ٶ� */
#define SPEED_HIGH 0x03 /* 11b: 200�ٶ� */

/* ������ת�Ƕ������ƫ�� (��2λ) */
#define ROTATION_MASK 0x30 /* 00110000b */
#define ROTATION_SHIFT 4

/* ������ת���ٶȶ��� */
#define ROT_NONE 0x00 /* 00b: ����ת */
#define ROT_30 0x01   /* 01b: ��ת���ٶ�30 */
#define ROT_N30 0x02  /* 10b: ��ת���ٶ�-30�� */
#define ROT_45 0x03  /* 11b: ��ת���ٶ�45 */

/* �˶��Ƕ����� (��4λ) */
#define ANGLE_MASK 0x0F    /* 00001111b */

/* �˶��Ƕȶ��� (����) */
#define ANGLE_0 0x00   /* 0000b: 0��(��ǰ) */
#define ANGLE_45 0x01  /* 0001b: 45�� */
#define ANGLE_90 0x02  /* 0010b: 90�� */
#define ANGLE_135 0x03 /* 0011b: 135�� */
#define ANGLE_180 0x04 /* 0100b: 180�� */
#define ANGLE_225 0x05 /* 0101b: -135�� */
#define ANGLE_270 0x06 /* 0110b: -90�� */
#define ANGLE_315 0x07 /* 0111b: -45�� */

#define MOTOR_FORWARD 0x42 /* ǰ�� */
#define MOTOR_BACK 0x46    /* ���� */
#define MOTOR_LEFT 0x4C    /* ��ת */
#define MOTOR_RIGHT 0x52   /* ��ת */
#define MOTOR_STOP 0x00    /* ֹͣ */

/* ��������Ƕȿ������� */
#define STEPPER_10_DEG 0x01   /* ����10�� */
#define STEPPER_180_DEG 0x02  /* ����180�� */
#define STEPPER_360_DEG 0x03  /* ����360�� */
#define STEPPER_1080_DEG 0x04 /* ����1080�� */
    typedef struct
    {
        char RxBuffer[FRAME_BYTE_LENGTH]; /* ���ջ����� */
        uint8_t aRxBuffer;                /* ���ջ��� */
        uint8_t Rx_Cnt;                   /* ���ջ������ */
        uint8_t USART_FrameFlag;          /* ������������֡��־��1������0������ */
    } UartStruct;

    extern UartStruct uart2Data; /* USART2�����ݽṹ�� */
    extern UartStruct uart3Data; /* USART3�����ݽṹ�� */

    extern uint8_t uart3_rx;
    extern uint8_t uart2_rx;

    //void USART2_Init(void);
    //  void USART_OUT(USART_TypeDef* USARTx, uint8_t *Data,...);
    //  char *itoa(int value, char *string, int radix);
    //  int fputc(int ch, FILE *f);

void USART_GetChar(UartStruct *Uartn, uint8_t nChar); /* ���ڽ��յ�һ���ֽ� */
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

