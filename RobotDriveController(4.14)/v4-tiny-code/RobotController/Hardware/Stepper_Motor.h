#ifndef _STEPPER_MOTOR_H_
#define _STEPPER_MOTOR_H_
#include "main.h"

#define StepMotorVAL 600 //步进电机运行速度(RPM)，范围0 - 5000RPM(600 角度范围0-3440)

extern uint32_t now_angle;

void Stepper_motor_goto_target_angle(uint32_t target_angle); //电机运行到指定角度
void Emm_V5_Pos_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, uint32_t clk, int raF, int snF); // 位置模式控制
void Emm_V5_Reset_CurPos_To_Zero(uint8_t addr); // 将当前位置清零
void Emm_V5_En_Control(uint8_t addr, uint8_t state, int snF); // 电机使能控制
void Emm_V5_Origin_Trigger_Return(uint8_t addr, uint8_t o_mode, int snF); // 发送命令触发回零
void Emm_V5_Stop_Now(uint8_t addr, uint8_t snF); // 让电机立即停止运动
#endif
