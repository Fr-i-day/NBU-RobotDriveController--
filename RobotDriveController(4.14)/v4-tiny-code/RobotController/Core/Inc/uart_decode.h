#ifndef __UART_DECODE_H
#define __UART_DECODE_H

#include "main.h"

// 串口接收回调
void UART_RxCpltCallback(uint8_t byte);

// 命令处理函数
void handle_command(void);

// TODO: 在这里声明机器人控制函数
void Robot_Move(float angle, int speed);
void Robot_SetArmHeight(int height);
void Robot_Rotate(int angle);
void Robot_Stop(void);
void Robot_Grab(int layer);
void Robot_Put(void);
void Robot_Correct(void);

#endif
