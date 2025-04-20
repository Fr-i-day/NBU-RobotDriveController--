import serial
from serial.serialutil import SerialException
import time

class SerialController:
    def __init__(self, port='COM6', baudrate=115200, timeout=1):
        """初始化串口通信"""
        try:
            self.serial = serial.Serial(
                port=port,
                baudrate=baudrate,
                timeout=timeout,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE
            )
            print(f"成功打开串口 {port}")
        except SerialException as e:
            print(f"打开串口失败: {e}")
            raise

    def send_command(self, command):
        """发送并接收十六进制指令"""
        try:
            hex_command = bytes.fromhex(command)
            self.serial.write(hex_command)
            print(f"已发送十六进制指令: {command}")
            
            time.sleep(0.1)
            if self.serial.in_waiting:
                response = self.serial.read(self.serial.in_waiting)
                hex_response = ' '.join([f"{b:02X}" for b in response])
                print(f"收到十六进制响应: {hex_response}")
                return hex_response
        except Exception as e:
            print(f"发送指令时出错: {e}")
            return None

    def close(self):
        """关闭串口连接"""
        if hasattr(self, 'serial') and self.serial.is_open:
            self.serial.close()
            print("串口已关闭")



def main():
    controller = None
    try:
        controller = SerialController(port='COM6', baudrate=115200)
        
        # 发送十六进制指令示例
        commands = [
            "A5 4C 5A",  # 
        ]
        
        for cmd in commands:
            controller.send_command(cmd.replace(" ", ""))  # 移除空格
            time.sleep(1)
            
    except Exception as e:
        print(f"出错: {e}")
    finally:
        if controller:
            controller.close()

if __name__ == "__main__":
    main()