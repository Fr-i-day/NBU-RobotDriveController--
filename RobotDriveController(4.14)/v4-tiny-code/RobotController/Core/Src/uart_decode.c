#include "uart_decode.h"

// 指令类型定义
#define CMD_HEADER      0x42  // 'B'
#define CMD_MOVE        0x4D  // 'M'
#define CMD_CORRECT     0x43  // 'C' 
#define CMD_ARM         0x41  // 'A'
#define CMD_REVOLVE     0x52  // 'R'
#define CMD_STOP        0x53  // 'S'
#define CMD_GRAB        0x47  // 'G'
#define CMD_PUT         0x50  // 'P'
#define CMD_TAIL        0x5A  // 'Z'

// 接收缓冲区
uint8_t rx_buffer[16];
uint8_t rx_count = 0;

// 将ASCII字符转换为数值
int ascii_to_int(uint8_t *buf, int len) {
    int val = 0;
    for(int i = 0; i < len; i++) {
        if(buf[i] >= '0' && buf[i] <= '9') {
            val = val * 10 + (buf[i] - '0');
        }
    }
    return val;
}

// 处理接收到的字节
void UART_RxCpltCallback(uint8_t byte) {
    static uint8_t is_receiving = 0;
    
    // 检测帧头
    if(byte == CMD_HEADER) {
        rx_count = 0;
        is_receiving = 1;
    }
    
    // 接收数据
    if(is_receiving) {
        rx_buffer[rx_count++] = byte;
        
        // 检查是否接收到完整帧
        if(byte == CMD_TAIL) {
            handle_command();
            is_receiving = 0;
        }
        
        // 防止缓冲区溢出
        if(rx_count >= sizeof(rx_buffer)) {
            is_receiving = 0;
        }
    }
}

// 命令处理函数
void handle_command(void) {
    // 检查帧头和帧尾
    if(rx_buffer[0] != CMD_HEADER || rx_buffer[rx_count-1] != CMD_TAIL) {
        return;
    }
    
    // 根据第二个字节判断命令类型
    switch(rx_buffer[1]) {
        case CMD_MOVE: {
            // BM角度(3位)速度(3位)Z
            char angle_str[4] = {0};
            char speed_str[4] = {0};
            memcpy(angle_str, &rx_buffer[2], 3);
            memcpy(speed_str, &rx_buffer[5], 3);
            
            float angle = (float)ascii_to_int((uint8_t*)angle_str, 3);
            int speed = ascii_to_int((uint8_t*)speed_str, 3);
            
            // TODO: 调用底盘移动函数
            Robot_Move(angle, speed);
            break;
        }
        
        case CMD_ARM: {
            // BA高度(2位)TTTTZ
            char height_str[3] = {0};
            memcpy(height_str, &rx_buffer[2], 2);
            int height = ascii_to_int((uint8_t*)height_str, 2);
            
            // TODO: 调用机械臂控制函数
            Robot_SetArmHeight(height);
            break;
        }
        
        case CMD_REVOLVE: {
            // BR角度(3位)TTTTTZ
            char angle_str[4] = {0};
            memcpy(angle_str, &rx_buffer[2], 3);
            int angle = ascii_to_int((uint8_t*)angle_str, 3);
            
            // TODO: 调用底盘旋转函数
            Robot_Rotate(angle);
            break;
        }
        
        case CMD_STOP: {
            // BSTTTTTTZ
            // TODO: 调用停止函数
            Robot_Stop();
            break;
        }
        
        case CMD_GRAB: {
            // BG层数(1位)TTTTTZ
            int layer = rx_buffer[2] - '0';
            // TODO: 调用抓取函数
            Robot_Grab(layer);
            break;
        }
        
        case CMD_PUT: {
            // BPTTTTTTZ
            // TODO: 调用放置函数
            Robot_Put();
            break;
        }
        
        case CMD_CORRECT: {
            // BCTTTTTTZ
            // TODO: 调用姿态校正函数
            Robot_Correct();
            break;
        }
    }
}
