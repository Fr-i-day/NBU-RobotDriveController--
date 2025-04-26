#ifndef _STEPPER_MOTOR_H_
#define _STEPPER_MOTOR_H_
#include "main.h"

#define StepMotorVAL 600 //������������ٶ�(RPM)����Χ0 - 5000RPM(600 �Ƕȷ�Χ0-3440)

extern uint32_t now_angle;

void Stepper_motor_goto_target_angle(uint32_t target_angle); //������е�ָ���Ƕ�
void Emm_V5_Pos_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, uint32_t clk, int raF, int snF); // λ��ģʽ����
void Emm_V5_Reset_CurPos_To_Zero(uint8_t addr); // ����ǰλ������
void Emm_V5_En_Control(uint8_t addr, uint8_t state, int snF); // ���ʹ�ܿ���
void Emm_V5_Origin_Trigger_Return(uint8_t addr, uint8_t o_mode, int snF); // �������������
void Emm_V5_Stop_Now(uint8_t addr, uint8_t snF); // �õ������ֹͣ�˶�
#endif
