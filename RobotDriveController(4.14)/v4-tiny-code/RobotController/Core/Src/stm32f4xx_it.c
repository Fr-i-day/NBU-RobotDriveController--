/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "motor_driver.h"
#include "motor_controller.h"
#include "backend_loop.h"
#include "usart.h"
#include "amt1450_uart.h"
#include "Stepper_Motor.h"
#include "ArmSolution.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define CMD_HEADER      0x42  // 'B'
#define CMD_MOVE        0x4D  // 'M'
#define CMD_CORRECT     0x43  // 'C' 
#define CMD_ARM         0x41  // 'A'
#define CMD_REVOLVE     0x52  // 'R'
#define CMD_STOP        0x53  // 'S'
#define CMD_GRAB        0x47  // 'G'
#define CMD_PUT         0x50  // 'P'
#define CMD_TAIL        0x5A  // 'Z'

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim7;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */

  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */

  /* USER CODE END TIM3_IRQn 1 */
}

/**
  * @brief This function handles TIM4 global interrupt.
  */
void TIM4_IRQHandler(void)
{
  /* USER CODE BEGIN TIM4_IRQn 0 */

  /* USER CODE END TIM4_IRQn 0 */
  HAL_TIM_IRQHandler(&htim4);
  /* USER CODE BEGIN TIM4_IRQn 1 */

  /* USER CODE END TIM4_IRQn 1 */
}

/**
  * @brief This function handles USART2 global interrupt.
  */
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */

  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */

  /* USER CODE END USART2_IRQn 1 */
}

/**
  * @brief This function handles USART3 global interrupt.
  */
void USART3_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_IRQn 0 */

  /* USER CODE END USART3_IRQn 0 */
  HAL_UART_IRQHandler(&huart3);
  /* USER CODE BEGIN USART3_IRQn 1 */

  /* USER CODE END USART3_IRQn 1 */
}

/**
  * @brief This function handles TIM5 global interrupt.
  */
void TIM5_IRQHandler(void)
{
  /* USER CODE BEGIN TIM5_IRQn 0 */

  /* USER CODE END TIM5_IRQn 0 */
  HAL_TIM_IRQHandler(&htim5);
  /* USER CODE BEGIN TIM5_IRQn 1 */

  /* USER CODE END TIM5_IRQn 1 */
}

/**
  * @brief This function handles TIM6 global interrupt, DAC1 and DAC2 underrun error interrupts.
  */
void TIM6_DAC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_DAC_IRQn 0 */

  /* USER CODE END TIM6_DAC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim6);
  /* USER CODE BEGIN TIM6_DAC_IRQn 1 */

  /* USER CODE END TIM6_DAC_IRQn 1 */
}

/**
  * @brief This function handles TIM7 global interrupt.
  */
void TIM7_IRQHandler(void)
{
  /* USER CODE BEGIN TIM7_IRQn 0 */

  /* USER CODE END TIM7_IRQn 0 */
  HAL_TIM_IRQHandler(&htim7);
  /* USER CODE BEGIN TIM7_IRQn 1 */

  /* USER CODE END TIM7_IRQn 1 */
}

/* USER CODE BEGIN 1 */

//定时器结束后回调函数，tim2~5都是编码器相关
//tim6是转速控制，tim7是后台程序
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)   
{
	if(htim==(&htim4))
	{
		if((TIM4->CR1 & TIM_COUNTERMODE_DOWN) == TIM_COUNTERMODE_DOWN) MotorDirver_Tim4_Update_Count--;  //向下溢出
		else 	MotorDirver_Tim4_Update_Count++; //向上溢出
	}
	else if(htim==(&htim5))
	{
		if((TIM5->CR1 & TIM_COUNTERMODE_DOWN) == TIM_COUNTERMODE_DOWN) MotorDirver_Tim5_Update_Count--;
		else 	MotorDirver_Tim5_Update_Count++;			
	}
	else if(htim==(&htim3))
	{
		if((TIM3->CR1 & TIM_COUNTERMODE_DOWN) == TIM_COUNTERMODE_DOWN) MotorDirver_Tim3_Update_Count--;
		else 	MotorDirver_Tim3_Update_Count++;			
	}
	else if(htim==(&htim2))
	{
		if((TIM2->CR1 & TIM_COUNTERMODE_DOWN) == TIM_COUNTERMODE_DOWN) MotorDirver_Tim2_Update_Count--;
		else 	MotorDirver_Tim2_Update_Count++;			
	}
	else if(htim==(&htim6))
	{
		MotorController_SpeedTunner();
	}else if(htim==(&htim7)){
    Backend_Loop();
  }
}





//UART串口通信的中断函数
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(huart);

	
	if(huart->Instance==USART3)
    {//处理步进电机控制
		USART_GetChar(&uart3Data,uart3Data.aRxBuffer);//字节数据保存到缓冲区中
		HAL_UART_Receive_IT(&huart3, (uint8_t *)&uart3Data.aRxBuffer, 1);   //再开启接收中断
		
		if(uart3Data.USART_FrameFlag==1){//如果数据帧完整，则发回数据
				HAL_UART_Transmit(&huart3, (uint8_t *)&uart3Data.RxBuffer, FRAME_BYTE_LENGTH,10); //将收到的信息发送出去
				while(HAL_UART_GetState(&huart3) == HAL_UART_STATE_BUSY_TX);//检测UART发送结束
				uart3Data.Rx_Cnt = 0;
				uart3Data.USART_FrameFlag=0;
		}
		
	}
		if(huart->Instance==USART2)
        {//处理主机与单片机通信
            USART_GetChar(&uart2Data,uart2Data.aRxBuffer);//字节数据保存到缓冲区中
            HAL_UART_Receive_IT(&huart2, (uint8_t *)&uart2Data.aRxBuffer, 1);   //再开启接收中断uart2Data
		
	 	    if(uart2Data.USART_FrameFlag==1)
                {//如果数据帧完整，则发回数据
                    HAL_UART_Transmit(&huart2, (uint8_t *)&uart2Data.RxBuffer, FRAME_BYTE_LENGTH,10); //将收到的信息发送出去
                    while(HAL_UART_GetState(&huart2) == HAL_UART_STATE_BUSY_TX);//检测UART发送结束
                    uart2Data.Rx_Cnt = 0;
                    uart2Data.USART_FrameFlag=0;
										
									switch(uart2Data.RxBuffer[1])
									{
										case CMD_MOVE:
										{
											if(uart2Data.RxBuffer[2] == 'T')
											{
												chassis_move(0, 0, 0.0f);
											}else
											{
												float dir_move = (((uart2Data.RxBuffer[2] - '0')*100 + (uart2Data.RxBuffer[3] - '0')*10 + (uart2Data.RxBuffer[4] - '0')) - 180)/ 180 * pi;
												float speed_move = (uart2Data.RxBuffer[5] - '0')*100 + (uart2Data.RxBuffer[6] - '0')*10 + (uart2Data.RxBuffer[7] - '0');											
												chassis_move(speed_move, dir_move, 0.0f);
											}
											
										}
										
//										case CMD_HEADER:
//										{
//										
//										}
//										
//										case CMD_CORRECT:
//										{
//										
//										}
//										
										case CMD_ARM:
										{
											float angle_arm = (uart2Data.RxBuffer[2] - '0')*200 + (uart2Data.RxBuffer[3] - '0');
											Stepper_motor_goto_target_angle(angle_arm);
											
										}
										
//										case CMD_REVOLVE:
//										{
//										
//										}
//									
//										case CMD_STOP:
//										{
//										
//										}
//										
//										case CMD_GRAB:
//										{
//											int layer = uart2Data.RxBuffer[2] - '0';
//											if(layer == 1)
//											{
//												SetServoAngle(3,0);
//												SetServoAngle(4,90);
//												SetServoAngle(5,90);
//												SetServoAngle(6,90);
//												SetServoAngle(7,90);
//												//Delay();
//												SetServoAngle(3,180);
//												//Delay();
//												SetServoAngle(7,180);
//												//Delay();
//												SetServoAngle(3,0);
//											}
//										}
//										
//										case CMD_PUT:
//										{
//										
//										}
//										
//										case CMD_TAIL:
//										{
//										
//										}
										
									}





									
                 //解析底盘运动指令
//                 switch(uart2Data.RxBuffer[1]){// 解析电机参数并控制电机
//                 	case MOTOR_FORWARD:
//                         chassis_move(70.0f, 0 , 0.0f); // 前进
//                         break;
//                     case MOTOR_BACK:
//                         chassis_move(70.0f, pi , 0.0f); //
//                         break;
//                     case MOTOR_LEFT:
//                         chassis_move(70.0f, pi / 2, 0.0f); // 左转
//                         break;
//                     case MOTOR_RIGHT:
//                         chassis_move(70.0f, -pi / 2, 0.0f); // 右转
//                         break;
//                 	case MOTOR_STOP:
//                         chassis_move(0.0f, 0.0f, 0.0f); // 停止
//                         break;
//				}

////--------------------------------------------------------//
//					                //解析步进电机指令	
//                switch(uart2Data.RxBuffer[2]){
//                    case STEPPER_10_DEG:
//                        Stepper_motor_goto_target_angle(10);
//                        break;
//                    case STEPPER_180_DEG:
//                        Stepper_motor_goto_target_angle(180);
//                        break;
//                    case STEPPER_360_DEG:
//                        Stepper_motor_goto_target_angle(360);
//                        break;
//                    case STEPPER_1080_DEG:
//                        Stepper_motor_goto_target_angle(1080);
//                        break;
//				}
				
//				through_BT_setAngle();

	 	        }
		 }
        // if(huart->Instance==USART2){
        //     //AMT1450_GetChar(uart2_rx,&amt1450_1_Rx);//字节数据保存到缓冲区中
        //     HAL_UART_Receive_IT(&huart2, (uint8_t *)&uart2_rx, 1);   //再开启接收中断
		
		//}
}



/* USER CODE END 1 */
