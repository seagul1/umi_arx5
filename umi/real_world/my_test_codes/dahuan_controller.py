from typing import Union, Optional, Callable
import socket
import enum
import struct
import serial
import multiprocessing as mp
import crcmod
import modbus_tk.defines as cst
from modbus_tk import modbus_rtu
from termcolor import cprint

class RegisterAddress(enum.Enum):
    """
    基础控制地址表
    """
    ini_gripper_addr = 0x0100,
    set_force_addr = 0x0101,
    set_width_addr = 0x0103,
    vel_addr = 0x0104,
    ini_state_feedback_addr = 0x0200,
    grasp_state_feedback_addr = 0x0201,
    position_feedback_addr = 0x0202,


class DahuanBinaryDriver:
    """
    DaHuan PGI Gripper Controller using Modbus-RTU 
    """
    def __init__(self, 
                 port: str,
                 # RS485 default setup
                 baudrate: int = 115200,
                 bytesize: int = 8,
                 parity: str = 'N', 
                 stopbits: int = 1, 
                ) -> None:
        """
        Parameters:
        -----------------------
        port: str, the communication port of Dahuan gripper.
        # baudrate:波特率,
        # bytesize:字节大小
        # parity:校验位
        # stopbits:停止位
        # timeout:读超时设置
        # writeTimeout:写超时
        # xonxoff:软件流控
        # rtscts:硬件流控
        # dsrdtr:硬件流控           
        """
        super(DahuanBinaryDriver).__init__()
        # -------------- establish connection --------------
        self.master = modbus_rtu.RtuMaster(
            serial=serial.Serial(port=port, baudrate=baudrate, 
                                 bytesize=bytesize, parity=parity,stopbits=stopbits,)
            )
        self.master.open()
        assert self.master._is_opened, "[Gripper] Port {} failed to connect.".format(port)
        self.master.set_timeout(2.0)
        self.master.set_verbose(True)
        # -------------- gripper parameters --------------
        self.max_width = 1000  # 千分比
        self.max_force = 100   # 百分比, 20%-100%
        
     
    def init_state(self):
        self.master.execute(1, cst.WRITE_SINGLE_REGISTER, 0x1000, 2, 0x0001)
        
        pass
    
    def set_width(self, width: int):
        assert width <= 1000 and width >=0, cprint("width value is out of range!", "red")
        info = self.master.execute(1, cst.WRITE_SINGLE_REGISTER, RegisterAddress.set_width_addr, 2, width)[1]
        pass
    
    def set_force(self, force: int):
        pass
    
    def get_info(self):
        pass
    
    def open_gripper(self):
        pass
    
    def close_gripper(self):
        pass
    