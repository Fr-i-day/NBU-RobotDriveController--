#include "motor_controller.h"
#include "motor_driver.h"
#include "tim.h"
#include "gpio.h"
#include "delay.h"
#include "math.h"  // 提供 sin() 的声明

typedef struct
{
    int16_t SpeedCur;       // 当前速度
    int16_t SpeedSet;       // 目标速度
    int16_t SpeedPWM;       // 电机PWM占空比
    float SpeedErr1;        // 上一次速度误差
    float SpeedErr2;        // 上上次速度误差
    int32_t EncCnt;         // 编码器计数
} Motor;

typedef struct
{
    float KP;               // 比例系数
    float KI;               // 积分系数
    float KD;               // 微分系数
    int16_t Acc;            // 加速度
    float WheelDiameter;    // 轮子直径
    int32_t EncoderResolution;  // 编码器分辨率 一圈多少脉冲
    int16_t MotorEnabledCount;  // 启用的电机数量
    Motor Motors[4];        // 电机数组
} MotorController;

MotorController controller;  // 控制器对象

// MotorController_Init() 初始化函数
void MotorController_Init(void)
{
    __HAL_TIM_CLEAR_IT(&htim6, TIM_IT_UPDATE);  // 清除定时器中断标志
    HAL_TIM_Base_Start_IT(&htim6);  // 启动定时器并使能中断

    MotorController_Enable(DISABLE);  // 禁用电机控制器

    // 初始化PID参数
    controller.KP = MOTOR_CONTROLLER_KP;  // 比例系数
    controller.KI = MOTOR_CONTROLLER_KI;  // 积分系数
    controller.KD = MOTOR_CONTROLLER_KD;  // 微分系数

    controller.Acc = MOTOR_CONTROLLER_ACC_LIMIT;                         // 默认加速度
    controller.WheelDiameter = MOTOR_WHEEL_DIAMETER;                     // 轮子直径
    controller.EncoderResolution = MOTOR_CONTROLLER_ENCODER_RESOLUTION;  // 编码器分辨率
    controller.MotorEnabledCount = MOTOR_COUNT;                          // 启用的电机数量
}

// MotorController_SetAcceleration() 设置轮子的加速度值，单位mm/s/s，设为0相当于最小值1。
void MotorController_SetAcceleration(uint16_t nAcc)
{
    controller.Acc = nAcc * MOTOR_CONTROLLER_PERIOD / 1000 + 1;
}

// MotorController_SetPIDParam() 设置PID参数
void MotorController_SetPIDParam(float Kp, float Ki, float Kd)
{
    controller.KP = Kp;  // 比例系数
    controller.KI = Ki;  // 积分系数
    controller.KD = Kd;  // 微分系数
}

void MotorController_Enable(FunctionalState NewState)
{
	assert_param(IS_FUNCTIONAL_STATE(NewState));

	// 初始编码器值和pwm设置。
	// 有一个问题，如果重启板子的时候，电机仍有一定速度，导致此时设置的电机初始编码器信息有误差，会导致初始时震动一下。
	// 延迟50毫秒可以等待电机停止运动
	HAL_Delay(50);
	for (int8_t i = 0; i < 4; i++)
	{
		controller.Motors[i] = (Motor){0}; // 启用速度调节器前把所有中间变量都清零。
		controller.Motors[i].EncCnt = Encoder_GetEncCount(i + 1);        //读取当前电机编码器数据，避免突变
		controller.Motors[i].SpeedPWM = MOTOR_DRIVER_PWM_DUTY_LIMIT / 2; //设置默认pwm值，此时为一半，正好停止。
	}

	if (NewState != DISABLE)
	{
		HAL_TIM_Base_Start_IT(&htim6);
	}
	else
	{
		HAL_TIM_Base_Stop_IT(&htim6);
	}
}

// 设置轮子转速，nMotor电机编号，nSpeed轮子线速度，单位：mm/s
void MotorController_SetSpeed(uint8_t nMotor, int16_t nSpeed) 
{
	if (nMotor > 4)
        return;
	controller.Motors[nMotor-1].SpeedSet = nSpeed;
}

void MotorController_SpeedTunner(void)
{	
	//循环设定的电机数量
	for (int i = 0; i < controller.MotorEnabledCount; i++)
	{
		Motor *motor = &controller.Motors[i];
		int16_t nSpeedExpect;

		// 如果当前速度小于设定速度
		if (motor->SpeedCur < motor->SpeedSet)
		{
			// 预期速度增加一个加速度值
			nSpeedExpect = motor->SpeedCur + controller.Acc;
			// 如果预期速度超过设定速度，则将预期速度设为设定速度
			if (nSpeedExpect > motor->SpeedSet)
				nSpeedExpect = motor->SpeedSet;
		}
		else if (motor->SpeedCur > motor->SpeedSet)
		{   // 预期速度减少一个加速度值
			nSpeedExpect = motor->SpeedCur - controller.Acc;
			// 如果预期速度低于设定速度，则将预期速度设为设定速度
			if (nSpeedExpect < motor->SpeedSet)
				nSpeedExpect = motor->SpeedSet;
		}
		else
		{
			// 当前速度等于设定速度，则预期速度也设为设定速度
			nSpeedExpect = motor->SpeedSet;
		}
		// 更新当前速度为预期速度
		motor->SpeedCur = nSpeedExpect;

		int32_t nCnt = Encoder_GetEncCount(i + 1);		// 获取编码器计数值
		// 根据编码器计数值计算当前速度
		float fSpeedCur = 3.14f * (nCnt - motor->EncCnt) * controller.WheelDiameter * 1000 / (controller.EncoderResolution * 4 * MOTOR_CONTROLLER_PERIOD);

		float fError = nSpeedExpect - fSpeedCur;		// 计算速度误差
		// 根据速度误差计算PWM增量
		int16_t pwmDelta = controller.KP * (fError - motor->SpeedErr1) + controller.KI * (fError + motor->SpeedErr1) / 2 + controller.KD * (fError - 2 * motor->SpeedErr1 + motor->SpeedErr2);
		// 根据PWM增量更新PWM设定值
		int16_t pwmSet = motor->SpeedPWM + pwmDelta;

		// 如果PWM设定值超过限制，则设为限制值
		if (pwmSet > MOTOR_DRIVER_PWM_DUTY_LIMIT)
		{
			pwmSet = MOTOR_DRIVER_PWM_DUTY_LIMIT;
		}
		// 如果PWM设定值小于0，则设为0
		else if (pwmSet < 0)
		{
			pwmSet = 0;
		}

		// 更新电机的PWM设定值、速度误差和编码器计数值
		motor->SpeedPWM = pwmSet;
		motor->SpeedErr2 = motor->SpeedErr1;
		motor->SpeedErr1 = fError;
		motor->EncCnt = nCnt;
	}

	// 遍历所有电机，将计算得到的PWM设定值设置给电机驱动器
	//pwm设置与参数设置分离，主要是为了能让所有电机同时启动。
	for (int i = 0; i < controller.MotorEnabledCount; i++)
	{
		Motor *motor = &controller.Motors[i];
		MotorDriver_SetPWMDuty(i + 1, motor->SpeedPWM);
	}
}

//运动解算函数
Chassis chassis;

void chassis_move(float target_speed, float target_dir, float target_omega) {
    float speed_cal[4];
    float sin_ang = sin(chassis.angle);
    float cos_ang = cos(chassis.angle);
    float speed_X = target_speed * cos(target_dir);
    float speed_Y = target_speed * sin(target_dir);

    speed_cal[0] = ((-cos_ang - sin_ang) * speed_X + (-sin_ang + cos_ang) * speed_Y + chassis.Radius * target_omega) / sqrt(2);
    speed_cal[1] = ((-cos_ang + sin_ang) * speed_X + (-sin_ang - cos_ang) * speed_Y + chassis.Radius * target_omega) / sqrt(2);
    speed_cal[2] = ((cos_ang + sin_ang) * speed_X + (sin_ang - cos_ang) * speed_Y + chassis.Radius * target_omega) / sqrt(2);
    speed_cal[3] = ((cos_ang - sin_ang) * speed_X + (sin_ang + cos_ang) * speed_Y + chassis.Radius * target_omega) / sqrt(2);

	
	MotorController_SetSpeed( 1 ,(int)speed_cal[3]);
	MotorController_SetSpeed( 2 ,(int)speed_cal[2]);
	MotorController_SetSpeed( 3 ,(int)speed_cal[1]);
	MotorController_SetSpeed( 4 ,(int)speed_cal[0]);
	
}


//void chassis_move(float target_speed, float target_dir_global, float target_omega) {
//    float speed_cal[4];
//    
//    // 1. 获取实时底盘角度（确保已通过传感器更新）
//    float current_angle = chassis.angle;
//    float sin_ang = sin(current_angle);
//    float cos_ang = cos(current_angle);
//    
//    // 2. 将全局目标速度转换到底盘本地坐标系
//    // target_dir_global：目标方向（全局坐标系，0弧度指向世界坐标系X轴正方向）
//    float global_speed_X = target_speed * cos(target_dir_global);
//    float global_speed_Y = target_speed * sin(target_dir_global);
//    
//    // 本地坐标系下的速度（车头方向为X轴正方向）
//    float local_speed_X = global_speed_X * cos_ang + global_speed_Y * sin_ang;
//    float local_speed_Y = -global_speed_X * sin_ang + global_speed_Y * cos_ang;
//    
//    // 3. 计算各轮速度（含旋转分量）
//    // 电机顺序：1右前轮，2右后轮，3左后轮，4左前轮
//    // 公式符号调整说明：
//    // - 旋转分量符号根据电机布局调整，确保逆时针旋转时合力矩正确
//    speed_cal[0] = ((-local_speed_X - local_speed_Y) - chassis.Radius * target_omega) / sqrt(2); // 右前轮
//    speed_cal[1] = ((-local_speed_X + local_speed_Y) + chassis.Radius * target_omega) / sqrt(2); // 右后轮
//    speed_cal[2] = ((local_speed_X - local_speed_Y) + chassis.Radius * target_omega) / sqrt(2);  // 左后轮
//    speed_cal[3] = ((local_speed_X + local_speed_Y) - chassis.Radius * target_omega) / sqrt(2);  // 左前轮
//    
//    // 4. 输出到电机（根据实际转向调整符号）
//    MotorController_SetSpeed(1, (int)speed_cal[3]); // 右前轮
//    MotorController_SetSpeed(2, (int)speed_cal[2]); // 右后轮
//    MotorController_SetSpeed(3, (int)speed_cal[1]); // 左后轮
//    MotorController_SetSpeed(4, (int)speed_cal[0]); // 左前轮
//}

