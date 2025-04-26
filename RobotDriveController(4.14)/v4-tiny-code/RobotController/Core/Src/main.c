/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
#include "adc.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "stm32f4xx_hal_cortex.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "motor_driver.h"
#include "motor_controller.h"
#include "keys.h"
#include "led.h"
#include "mpu6500dmp.h"
#include "amt1450_uart.h"
#include "ArmSolution.h"
#include "Stepper_Motor.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

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
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
   int i = 0;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();
  HAL_NVIC_SetPriority(SysTick_IRQn, 1, 0);
  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM6_Init();
  MX_USART3_UART_Init();
  MX_TIM7_Init();
  MX_ADC3_Init();
  MX_I2C1_Init();
  MX_TIM8_Init();
  MX_TIM9_Init();
  MX_TIM12_Init();
  MX_TIM13_Init();
  MX_TIM14_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  MPU6500_DMP_Init(); // MPU6500���ٶȴ�������ʼ��
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  //===================�������================
  MotorDriver_Init();
  //һ������start�󣬾Ϳ�ʼ�ĵ��ˣ�һ��������120ma�ĵ��������ݵ�����������,MotorDriver_OFF���ܹرպĵ硣
  MotorDriver_Start(4, MOTOR_DRIVER_PWM_DUTY_LIMIT / 2);
  MotorDriver_Start(3, MOTOR_DRIVER_PWM_DUTY_LIMIT / 2);
  MotorDriver_Start(2, MOTOR_DRIVER_PWM_DUTY_LIMIT / 2);//���֣�SpeedΪ��ʱǰ��
  MotorDriver_Start(1, MOTOR_DRIVER_PWM_DUTY_LIMIT / 2);//���֣�SpeedΪ��ʱǰ��

  Encoder_Init();
  //==================���ת�ٿ���������===============
  MotorController_Init();                // ��ʼ��������
  MotorController_Enable(ENABLE);
  int nSpeed = 0; // ת�ٱ���
  chassis.angle = 0;    // ������̳�ʼ�Ƕ�Ϊ0
  chassis.Radius = 295.0f; // �������ĵ����ӵĳ���Ϊ295mm��0.295m��

  // ����Ŀ���ٶȺͷ���
  float target_speed = 200.0f; // Ŀ���ٶȣ�1.0 m/s
  float target_dir = 0.0f;     // Ŀ�귽��0���ȣ�X��������
  float target_omega = pi / 6; // Ŀ����ת���ٶȣ�0������ת��

  //===================���⴫��������===============
//  AMT1450_UART_Cmd(ENABLE);

  //=================led����=================
  FnLED_SetRGB(FnLED2, 33, 0, 0, 1);
  uint8_t led_val = 0; // RGB�任��Greenɫ��ֵ�仯����
  HAL_Delay(500);      // �ӳ�500ms��ر�led2
  FnLED_OFF(FnLED2);

  //=================������Ʋ���=============
  ArmDriver_Init();

//  uint16_t servo_pwm = 0;
//  uint16_t cnt = 0;
    // ʹ�ܲ������
	
  //=================����������Ʋ���=============
    Emm_V5_En_Control(1, 1, 0);
    HAL_Delay(10);  // �ȴ�ʹ�����

    // ȷ����ǰλ��Ϊ���
    Emm_V5_Reset_CurPos_To_Zero(1);
    HAL_Delay(10);  // �ȴ��������
//		chassis_move(50, 0, 0.0f);   //ǰ
//		HAL_Delay(1000);
//		chassis_move(50, pi/2, 0.0f);  //��
//		HAL_Delay(1000);
//		chassis_move(50, - pi/2, 0.0f);  //��
//		HAL_Delay(1000);
//		chassis_move(50, pi, 0.0f);    //��
//		HAL_Delay(1000);
//		chassis_move(0, 0, 0.0f);
//	  Stepper_motor_goto_target_angle(3440);
//	  HAL_Delay(10000);
//		Stepper_motor_goto_target_angle(0);
//	  HAL_Delay(100);
//		Stepper_motor_goto_target_angle(2000);
//	  HAL_Delay(1000);
//		Stepper_motor_goto_target_angle(3000);
//	  HAL_Delay(1000);
//	  Stepper_motor_goto_target_angle(1000);
//	  HAL_Delay(1000);
//		Stepper_motor_goto_target_angle(0);
//	  HAL_Delay(1000);
  while (1)
  {
    /* USER CODE END WHILE */

//	chassis_move(50,pi/3,0); //����ǰ��
//	HAL_Delay(2000);
//	chassis_move(50,-pi/4,0);//����ǰ��
//	HAL_Delay(2000);
//	chassis_move(50,-3*pi/4,0); 
//	HAL_Delay(2000);
//	chassis_move(50,3*pi/4,0);
//	HAL_Delay(1500);
    /* USER CODE BEGIN 3 */

    //
    // ����ѭ��ִ�еĴ����
    //
    //============================����ʵ�ָ߶ȸı�=====================
//    if (Key_Released(1) == 1)
//    {
//			Stepper_motor_goto_target_angle(3200+i);
//			HAL_Delay(100);
//			i = i + 10;
//    }
//    if (Key_Released(2) == 1)
//    {
//			Stepper_motor_goto_target_angle(0);
//			HAL_Delay(100);
//			i = 0;
//    }

    //===============LED���Գ���---��ɫ����ʵ��=============
    FnLED_SetRGB(FnLED3, 0, led_val, 0, 1);
    led_val += 1;
    if (led_val > 66)
      led_val = 0;
    HAL_Delay(7);

    //===================MPU6500���ԣ����ٶȴ�������===============
    /* ��ȡmpu6500���ݣ������������ں����ڲ����Կ� */
    Get_MPU6500_DMP_Data();

    //==================���⴫�������ݲɼ�========================
    //get_AMT1450Data_UART���������ں����ڲ�˵����
//    get_AMT1450Data_UART(&begin, &jump, count);
//    if (jump == 2)
//      position = 0.5f * (count[0] + count[1]);
    //���ݼ�⵽��ѭ����λ�ã���ʾ��ͬ��LED��ɫ��
//    if (position>72)
//    {
//      FnLED_SetRGB(FnLED2, 33, 0, 0, 1);
//    }else{
//      FnLED_SetRGB(FnLED2, 0, 0, 33, 1);
//    }
    //===================������Ʋ���=======================
//    servo_pwm += 5;
//    if (servo_pwm > 180)
//      servo_pwm = 0;
//    for (uint8_t i = 0; i < 8; i++)
//    {
//      SetServoAngle(i, servo_pwm);
//    }
	//===================小车=======================
//	if(cnt == 0){MotorController_SetSpeed(1, -200);MotorController_SetSpeed(2, 200);}
//	if(cnt == 10){MotorController_SetSpeed(1, 0);}//MotorController_SetSpeed(2, 200);��ת
//	if(cnt <= 11 )cnt++;
    HAL_Delay(500); 

}
  
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
