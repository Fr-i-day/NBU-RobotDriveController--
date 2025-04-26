#include "Stepper_Motor.h"
#include "stdlib.h"
#include "delay.h"
#include "usart.h"

//步进电机控制程序，步距角1.8°，16细分，3200个脉冲转一圈

uint32_t now_angle = 0;

/**
  * @brief    让步进电机旋转到设定角度
  * @param    target_angle目标角度(范围)
*/
void Stepper_motor_goto_target_angle(uint32_t target_angle)
{
    uint8_t dir;
    uint32_t clk;
    uint32_t time;
    if(target_angle > now_angle){
        dir = 0;
        clk = (target_angle - now_angle) /1.8 * 16 ;
        time = (target_angle - now_angle)/360/StepMotorVAL*60000;
    }
    else if(target_angle < now_angle){
        dir = 1;
        clk = (now_angle - target_angle) /1.8 * 16 ;
        time = (now_angle - target_angle)/360/StepMotorVAL*60000;
    }
    else return;
    Emm_V5_Pos_Control(1, dir, StepMotorVAL, 0, clk, 0, 0);
    now_angle = target_angle;
    HAL_Delay(time+1000);
}

/**
  * @brief    位置模式
  * @param    addr：电机地址
  * @param    dir ：方向，0为顺时针，1为逆时针
  * @param    vel ：速度(RPM)，范围0 - 5000RPM
  * @param    acc ：加速度，范围0 - 255，注意：0是直接启动
  * @param    clk ：脉冲数，范围0- (2^32 - 1)个
  * @param    raF ：相位/绝对标志，false为相对运动，true为绝对值运动
  * @param    snF ：多机同步标志，false为不启用，true为启用
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void Emm_V5_Pos_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, uint32_t clk, int raF, int snF)
{
  uint8_t cmd[13] = {0};

  // 装载命令
  cmd[0]  =  addr;                      // 地址
  cmd[1]  =  0xFD;                      // 功能码
  cmd[2]  =  dir;                       // 方向
  cmd[3]  =  (uint8_t)(vel >> 8);       // 速度(RPM)高8位字节
  cmd[4]  =  (uint8_t)(vel >> 0);       // 速度(RPM)低8位字节 
  cmd[5]  =  acc;                       // 加速度，注意：0是直接启动
  cmd[6]  =  (uint8_t)(clk >> 24);      // 脉冲数(bit24 - bit31)
  cmd[7]  =  (uint8_t)(clk >> 16);      // 脉冲数(bit16 - bit23)
  cmd[8]  =  (uint8_t)(clk >> 8);       // 脉冲数(bit8  - bit15)
  cmd[9]  =  (uint8_t)(clk >> 0);       // 脉冲数(bit0  - bit7 )
  cmd[10] =  raF;                       // 相位/绝对标志，false为相对运动，true为绝对值运动
  cmd[11] =  snF;                       // 多机同步运动标志，false为不启用，true为启用
  cmd[12] =  0x6B;                      // 校验字节
  
  // 发送命令
  HAL_UART_Transmit(&huart3, cmd, 13, HAL_MAX_DELAY);
}

/**
  * @brief    将当前位置清零
  * @param    addr  ：电机地址
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void Emm_V5_Reset_CurPos_To_Zero(uint8_t addr)
{
  uint8_t cmd[4] = {0};
  
  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0x0A;                       // 功能码
  cmd[2] =  0x6D;                       // 辅助码
  cmd[3] =  0x6B;                       // 校验字节
  
  // 发送命令
	HAL_UART_Transmit(&huart3, cmd, 4, HAL_MAX_DELAY);
  HAL_Delay(5);
}

/**
  * @brief    使能信号控制
  * @param    addr  ：电机地址
  * @param    state ：使能状态     ，true为使能电机，false为关闭电机
  * @param    snF   ：多机同步标志 ，0为不启用，1为启用
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void Emm_V5_En_Control(uint8_t addr, uint8_t state, int snF)
{
  uint8_t cmd[6] = {0};
  
  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0xF3;                       // 功能码
  cmd[2] =  0xAB;                       // 辅助码
  cmd[3] =  state;             // 使能状态
  cmd[4] =  snF;                        // 多机同步运动标志
  cmd[5] =  0x6B;                       // 校验字节
  
  // 发送命令
  HAL_UART_Transmit(&huart3, cmd, 6, HAL_MAX_DELAY);
  HAL_Delay(5);
}

/**
  * @brief    触发回零
  * @param    addr   ：电机地址
  * @param    o_mode ：回零模式，0为单圈就近回零，1为单圈方向回零，2为多圈无限位碰撞回零，3为多圈有限位开关回零
  * @param    snF   ：多机同步标志，false为不启用，true为启用
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void Emm_V5_Origin_Trigger_Return(uint8_t addr, uint8_t o_mode, int snF)
{
  uint8_t cmd[5] = {0};
  
  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0x9A;                       // 功能码
  cmd[2] =  o_mode;                     // 回零模式，0为单圈就近回零，1为单圈方向回零，2为多圈无限位碰撞回零，3为多圈有限位开关回零
  cmd[3] =  snF;                        // 多机同步运动标志，false为不启用，true为启用
  cmd[4] =  0x6B;                       // 校验字节
  
  // 发送命令
  HAL_UART_Transmit(&huart3, cmd, 5, HAL_MAX_DELAY);
  HAL_Delay(5);
}

/**
  * @brief    立即停止（所有控制模式都通用）
  * @param    addr  ：电机地址
  * @param    snF   ：多机同步标志，0为不启用，1为启用
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void Emm_V5_Stop_Now(uint8_t addr, uint8_t snF)
{
  uint8_t cmd[5] = {0};
  
  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0xFE;                       // 功能码
  cmd[2] =  0x98;                       // 辅助码
  cmd[3] =  snF;                        // 多机同步运动标志
  cmd[4] =  0x6B;                       // 校验字节
  
  // 发送命令
  HAL_UART_Transmit(&huart3, cmd, 5, HAL_MAX_DELAY);
  HAL_Delay(5);
}
