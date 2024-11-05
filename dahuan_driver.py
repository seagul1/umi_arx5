from typing import Union, Optional, Callable
import enum
import struct
import serial
import multiprocessing as mp
import crcmod
import modbus_tk.defines as cst
from modbus_tk import modbus_rtu
from termcolor import cprint
import time

GRIPPER_ID = 0X01


class RegisterAddress(enum.IntEnum):
    """
    基础控制寄存器地址表
    """
    init_gripper_addr= 0x0100,
    set_force_addr = 0x0101,
    set_width_addr = 0x0103,
    get_width_addr = 0x0202,
    set_vel_addr = 0x0104,
    init_state_fb_addr = 0x0200,
    grasp_state_fb_addr = 0x0201,
    position_fb_addr = 0x0202,


class DahuanBinaryDriver:
    """
    DaHuan PGI Gripper Controller using Modbus-RTU 
    """
    def __init__(self, 
                 port: str,
                 # RS485 default setup
                 #  baudrate: int = 115200,
                 #  bytesize: int = 8,
                 #  parity: str = 'N', 
                 #  stopbits: int = 1, 
                ) -> None:
        """
        Parameters:
        -----------------------
        port: str, the communication port of Dahuan gripper.
        baudrate:波特率,
        bytesize:字节大小
        parity:校验位
        stopbits:停止位
        timeout:读超时设置 
        """
        super(DahuanBinaryDriver).__init__()
        self.port = port
        self.max_width = 1000  # 千分比
        self.min_width = 0
        self.max_force = 100   # 百分比, 20%-100%
        self.min_force = 20
        self.max_vel = 100     # 速度范围，1%~100%
        self.min_vel = 1

        self.ser = None

        # create crc16-CCITT 
        self.crc16 = crcmod.mkCrcFun(0x18005, 
                                     rev=True, 
                                     initCrc=0xFFFF, 
                                     xorOut=0x0000)
        
    def start(self):
        self.ser = serial.Serial(port=self.port)
        self.ser.open()
        assert self.ser.is_open(), cprint("Fail to connect to Gripper!", "red")

    
    def stop(self):
        self.stop_cmd()
        self.disconnect()
        self.ser.close()
        return
    
    def __enter__(self):
        self.start()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        self.stop()

    # ================= low level API ================
    def cal_crc(self, commands):
        '''Calculate the CRC.
        
        Args:
            commands: The commands to be calculated.
            
        Returns:
            crc_l: The low value of the CRC.
            crc_h: The high value of the CRC.
        '''

        assert len(commands) == 6, print("Wrong command format!")
        bytes_ = b''
        for i in range(len(commands)):
            if type(commands[i]) is not bytes:
                bytes_ = bytes_ + commands[i].to_bytes(1, 
                                                    byteorder='big', 
                                                    signed=True)
            else:
                bytes_ = bytes_ + commands[i]
        crc = self.crc16(bytes_).to_bytes(2, 
                                          byteorder='big', 
                                          signed=False)
        crc_h = int.from_bytes(crc[0:1], 
                              byteorder='big', 
                              signed=False)
        crc_l = int.from_bytes(crc[1:2], 
                              byteorder='big', 
                              signed=False)
        return crc_l, crc_h

    def msg_send(self,  
                 modbus_addr, 
                 val=0x0001, 
                 is_set=True,
                 is_read=True):
        '''
        Send the command to the gripper. 
        Command format: 
        地址码 + 功能码 + 寄存器地址 + 寄存器数据 + CRC校验码
          01  +    06  +   01 00   +    00 01   +   49 F6

        Args:
            modbus_addr: The high address of the modbus. 
            val: The value to be set. 
            is_set: If the value is to be set. 
            is_read: If the return is to be read.

        Returns:
            data: The data read from the serial port.
        '''
        
        # Set the function code
        if is_set:
            set_code = 0x06
        else:
            set_code = 0x03

        # process modbus address
        modbus_addr_b = modbus_addr.to_bytes(2, 'big')
        modbus_high_addr_b = modbus_addr_b[0:1]
        modbus_low_addr_b = modbus_addr_b[1:2]
        modbus_high_addr = int.from_bytes(modbus_high_addr_b, 'big')
        modbus_low_addr = int.from_bytes(modbus_low_addr_b, 'big')

        # process the register data
        val = val if val >= 0 else val - 1
        bytes_ = val.to_bytes(2, 
                              byteorder='big', 
                              signed=True)
        val_l = int.from_bytes(bytes_[0:1], 
                               byteorder='big', 
                               signed=True)
        val_h = int.from_bytes(bytes_[1:2], 
                               byteorder='big', 
                               signed=True)
        
        # calculate crc code
        crc_l, crc_h = self.cal_crc([GRIPPER_ID, 
                                     set_code, 
                                     modbus_high_addr, 
                                     modbus_low_addr, 
                                     val_l, 
                                     val_h])
        
        # Set the command, modbus addr Big-Endian, value Little-Endian
        cdata = [GRIPPER_ID, 
                 set_code, 
                 modbus_high_addr, 
                 modbus_low_addr, 
                 val_l, 
                 val_h,
                 crc_l,
                 crc_h]
        
        for i in range(len(cdata)):
            cdata[i] = cdata[i] if cdata[i] >= 0 else cdata[i] + 256
        # Write the data
        res = self.ser.write(cdata)
        if res < 0:
            raise RuntimeError("Message send failed.")
        
        if is_read:
            data_b = self.read_info()
            data = int.from_bytes(data_b[3:5], 
                                  byteorder='big', 
                                  signed=True)
            if data < 0:
                data = data + 1
            self.ser.flush()
            return data
        else:
            time.sleep(0.005)
            return

    def read_info(self):
        '''Read the data from the serial port.

        Args:
            None
        
        Returns:
            udata: The data read from the serial port.
        '''

        time.sleep(0.08)
        udata = self.ser.read_all()
        return udata
    

    def cmd_msg_receive(self) -> dict:
        return
        # read gripper id
        device_id_b = self.ser.read(1)
        device_id = int.from_bytes(device_id_b, "little")

        # read function code
        set_code_b = self.ser.read(1)
        set_code = int.from_bytes(set_code_b, "little")

        # TODO: 读出来的两个字节寄存器地址是高位在前还是低位在前？
        reg_addr_b = self.ser.read(2)
        print(reg_addr_b)
        # reg_addr_high = 

        value_b = self.ser.read(2)

        crc_b = self.ser.read(2)

        result = {
            'device_id': device_id,
            'function_code': set_code,
            'register_address_b': reg_addr_b,
            'value_b': value_b
        }
        return result
    
    def cmd_submit(self, 
                 modbus_addr, 
                 val=0x0001, 
                 is_set=True,
                 is_read=True, 
                 pending: bool=True, 
                 ignore_other=False):
        return
        res = self.msg_send(modbus_addr, val, is_set, is_read)
        if res < 0:
            raise RuntimeError("Message send failed.")
        
        # receive response, repeat if pending
        # TODO: WSG 中的pending暂未实现
        msg = None
        keep_running = True
        # while keep_running:
        #     msg = self.cmd_msg_receive()
        #     if ignore_other and msg['command_id'] != cmd_id:
        #         continue

        #     if msg['command_id'] != cmd_id:
        #         raise RuntimeError(
        #             "Response ID ({:02X}) does not match submitted command ID ({:02X})\n".format(
        #             msg['command_id'], cmd_id))
        #     if pending:
        #         status = msg['status_code']
        #     keep_running = pending and status == StatusCode.E_CMD_PENDING.value
        msg = self.cmd_msg_receive()

        return msg


    # ============== mid level API ================

    def act(self):
        raise NotImplemented
    
    # =============== high level API ===============
    
    def disconnect(self):
        raise NotImplemented

    def homing(self):
        raise NotImplemented
    
    def pre_position(self):
        raise NotImplemented
    
    def custom_script(self):
        raise NotImplemented
    
    # ============================================

    def calibration(self):
        modbus_addr = RegisterAddress.init_gripper_addr.value
        return self.msg_send(modbus_addr, 0xA5)

    def reset_width(self):
        modbus_addr = RegisterAddress.init_gripper_addr.value
        return self.msg_send(modbus_addr, 0x01)

    def set_width(self, width, blocking=True):
        assert width >=0 and width <= self.max_width, cprint("width is out of range!", "red")
        modbus_addr = RegisterAddress.set_width_addr.value
        self.msg_send(modbus_addr, width)

        if blocking:
            while True:
                current_width = self.get_current_width()
                if current_width == width:
                    break
                time.sleep(0.01)

    def get_current_width(self):
        modbus_addr = RegisterAddress.get_width_addr.value
        return self.msg_send(modbus_addr, is_set=False)
    
    def get_setting_width(self):
        modbus_addr = RegisterAddress.set_width_addr.value
        return self.msg_send(modbus_addr, is_set=False)
    
    def set_velocity(self, vel):
        assert vel >= self.min_vel and vel <= self.max_vel, \
        cprint("velocity is out of range!", "red")
        modbus_addr = RegisterAddress.set_vel_addr.value
        self.msg_send(modbus_addr, vel)
    
    def get_velocity(self):
        modbus_addr = RegisterAddress.set_vel_addr.value
        self.msg_send(modbus_addr, is_set=False)
    
    def set_force(self, force):
        assert force >= self.min_force and force <= self.max_force, \
        cprint("force is out of range!", "red")

        modbus_addr = RegisterAddress.set_force_addr.value
        self.msg_send(modbus_addr, force)
    
    def get_force(self):
        modbus_addr = RegisterAddress.set_force_addr.value
        return self.msg_send(modbus_addr, is_set=False)
    
        
    
    
    

def test():
    dh_controller = DahuanBinaryDriver('/dev/ttyUSB0')
    # dh_controller.calibration()
    # dh_controller.reset_width()
    # dh_controller.set_width(500)
    dh_controller.get_current_width()
    dh_controller.get_setting_width()


    




if __name__ == "__main__":
    ini_gripper_addr = 0x0100
    a = ini_gripper_addr.to_bytes(length=2,byteorder="big")
    print(int.from_bytes(a[0:1],byteorder="big"))
    crc = crcmod.mkCrcFun(0x18005,rev=True, initCrc=0xFFFF, xorOut=0x0000)
    data = []
    b = 0x01
    c = 0x06
    d1 = 0x01
    d2 = 0x01
    e1 = 0x00
    e2 = 0x1E
    data.append(b.to_bytes(1, byteorder='big', signed=True))
    data.append(c.to_bytes(1, byteorder='big', signed=True))
    data.append(d1.to_bytes(1, byteorder='big', signed=True))
    data.append(d2.to_bytes(1, byteorder='big', signed=True))
    data.append(e1.to_bytes(1, byteorder='big', signed=True))
    data.append(e2.to_bytes(1, byteorder='big', signed=True))
    print(data)
    bytes_ = b''
    for da in data:
        bytes_ += da

    result = crc(bytes_).to_bytes(2, 
                                          byteorder='big', 
                                          signed=False)
    print(result)
    crc_h = int.from_bytes(result[0:1], 
                              byteorder='big', 
                              signed=False)
    crc_l = int.from_bytes(result[1:2], 
                              byteorder='big', 
                              signed=False)
    print(crc_h, crc_l)

    # -------------------------------------------------
    val = 0xA5
    bytes_ = val.to_bytes(2, 
                              byteorder='big', 
                              signed=True)
    print(bytes_)
    val_l = int.from_bytes(bytes_[0:1], 
                               byteorder='big', 
                               signed=True)
    val_h = int.from_bytes(bytes_[1:2], 
                               byteorder='big', 
                               signed=True)
    test_data = [0x01, 0x06, 0x01, 0x00, val_l, val_h]
    print(test_data)
    bytes_ = b''
    for i in range(len(test_data)):
        bytes_ = bytes_ + test_data[i].to_bytes(1, 
                                                byteorder='big', 
                                                signed=True)
    crc = crc(bytes_).to_bytes(2, 
                                          byteorder='big', 
                                          signed=False)
    crc_h = int.from_bytes(crc[0:1], 
                              byteorder='big', 
                              signed=False)
    crc_l = int.from_bytes(crc[1:2], 
                              byteorder='big', 
                              signed=False)
    print(crc_l, crc_h)

    value = 0x0100
    value_b = value.to_bytes(2, 'big')
    print(value_b[0:1])
    print(RegisterAddress.init_gripper_addr.value.to_bytes(2, 'big'))
    
    test()


    