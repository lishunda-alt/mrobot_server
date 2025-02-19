# -*- coding:utf-8 -*-
from ament_index_python.packages import get_package_share_directory
from .dependent_library.log import log
from rclpy.node import Node
from ctypes import *
import platform
import threading
import time
from .asyncioServer import tcpServer
import os
'''
 Device Type
'''
LCAN_USBCAN1          = c_uint(3)
LCAN_USBCAN2          = c_uint(4)
LCAN_USBCANFD2        = c_uint(41)
LCAN_USBCANFD1        = c_uint(42)
LCAN_USBCANFDMini     = c_uint(43)
'''
 Interface return status
'''
LCAN_STATUS_ERR         = 0
LCAN_STATUS_OK          = 1

'''
 CAN type
'''
LCAN_TYPE_CAN    = c_uint(0)
LCAN_TYPE_CANFD  = c_uint(1)

LCAN_CANFD_BRS = 0x08
LCAN_CANFD_ESI = 0x10

'''
    deviceInd
'''
LCAN_DeviceInd = c_uint(0)
LCAN_CanInd = c_uint(0)

class LCAN_INIT_CONFIG(Structure):
    _fields_ = [("accCode", c_uint),
                ("accMask", c_uint),
                ("reserved", c_uint),
                ("filter",   c_ubyte),
                ("timing0",  c_ubyte),
                ("timing1",  c_ubyte),
                ("mode",     c_ubyte)]

class LCAN_INITFD_CONFIG(Structure):
    _fields_ = [("abitBaudHz",   c_uint),
                ("dbitBaudHz",   c_uint),
                ("abit_timing",  c_uint),
                ("dbit_timing",  c_uint),
                ("mode",         c_ubyte),
                ("fdEn",         c_ubyte),
                ("isoEn",        c_ubyte),
                ("rev1",         c_ubyte)]

class LCAN_FLITER_CONFIG(Structure):
    _fields_ = [("cmd", c_ubyte),
                ("fliterMode", c_ubyte),
                ("idType", c_ubyte),
                ("rev1", c_ubyte),
                ("fliterStardId", c_uint),
                ("filterEndId", c_uint)]

class LCAN_CAN_OBJ(Structure):
    _fields_ = [("id",            c_uint),
                ("timeStamp",     c_uint),
                ("timeFlag",      c_ubyte),
                ("sendType",      c_ubyte),
                ("remoteFlag",    c_ubyte),
                ("externFlag",    c_ubyte),
                ("dataLen",       c_ubyte),
                ("data",      c_ubyte * 8),
                ("reserved",  c_ubyte * 3)]

class LCAN_CANFD_OBJ(Structure):
    _fields_ = [("id",            c_uint),
                ("timeStamp",     c_uint),
                ("flag",          c_ubyte),
                ("sendType",      c_ubyte),
                ("remoteFlag",    c_ubyte),
                ("externFlag",    c_ubyte),
                ("dataLen",       c_ubyte),
                ("data",      c_ubyte * 64),
                ("reserved",  c_ubyte * 3)]

class LCAN(object):
    def __init__(self):
        if platform.system() == "Linux":
            libusb = os.path.join(get_package_share_directory('mr_robot_server'), 'lib', 'libusb-1.0.so')
            libcan = os.path.join(get_package_share_directory('mr_robot_server'), 'lib', 'libControlCAN.so')
            CDLL(libusb, mode = RTLD_GLOBAL)
            self.__dll = cdll.LoadLibrary(libcan)
        else:
            print(platform.system()+":" +"No support now!")
        if self.__dll == None:
            print("DLL couldn't be loaded!")

    def LCAN_OpenDevice(self, devType, devIndex):
        try:
            return self.__dll.LCAN_OpenDevice(devType, devIndex)
        except:
            print("Exception on LCAN_OpenDevice!")
            raise

    def LCAN_CloseDevice(self, devType, devIndex):
        try:
            return self.__dll.LCAN_CloseDevice(devType, devIndex)
        except:
            print("Exception on LCAN_CloseDevice!")
            raise

    def LCAN_SetFliter(self, devType, devIndex, canIndex, fliter_config):
        try:
            return self.__dll.LCAN_SetFliter(devType, devIndex, canIndex, byref(fliter_config))
        except:
            print("Exception on LCAN_SetFliter!")
            raise

    def LCAN_InitCAN(self, devType, devIndex, canIndex, init_config):
        try:
            return self.__dll.LCAN_InitCAN(devType, devIndex, canIndex, byref(init_config))
        except:
            print("Exception on ZCAN_InitCAN!")
            raise

    def LCAN_InitCANFD(self, devType, devIndex, canIndex, initFD_config):
        try:
            return self.__dll.LCAN_InitCANFD(devType, devIndex, canIndex, byref(initFD_config))
        except:
            print("Exception on LCAN_InitCANFD!")
            raise

    def LCAN_ClearBuffer(self, devType, devIndex, canIndex):
        try:
            return self.__dll.LCAN_ClearBuffer(devType, devIndex, canIndex)
        except:
            print("Exception on LCAN_ClearBuffer!")
            raise

    def LCAN_GetReceiveNum(self, devType, devIndex, canIndex, type):
        try:
            return self.__dll.LCAN_GetReceiveNum(devType, devIndex, canIndex, type)
        except:
            print("Exception on LCAN_GetReceiveNum!")
            raise

    def LCAN_Transmit(self, devType, devIndex, canIndex, canObjPtr, len):
        try:
            return self.__dll.LCAN_Transmit(devType, devIndex, canIndex, byref(canObjPtr), len)
        except:
            print("Exception on LCAN_Transmit!")
            raise

    def LCAN_TransmitFD(self, devType, devIndex, canIndex, canFDObjPtr, len):
        try:
            return self.__dll.LCAN_TransmitFD(devType, devIndex, canIndex, byref(canFDObjPtr), len)
        except:
            print("Exception on LCAN_TransmitFD!")
            raise

    def LCAN_Receive(self, devType, devIndex, canIndex, rcv_num, wait_time = c_int(-1)):
        try:
            rcv_can_objs = (LCAN_CAN_OBJ * rcv_num)()
            ret = self.__dll.LCAN_Receive(devType, devIndex, canIndex, byref(rcv_can_objs), rcv_num, wait_time)
            return rcv_can_objs, ret
        except:
            print("Exception on LCAN_Receive!")
            raise

    def LCAN_ReceiveFD(self, devType, devIndex, canIndex, rcv_num, wait_time = c_int(-1)):
        try:
            rcv_canFD_objs = (LCAN_CANFD_OBJ * rcv_num)()
            ret = self.__dll.LCAN_ReceiveFD(devType, devIndex, canIndex, byref(rcv_canFD_objs), rcv_num, wait_time)
            return rcv_canFD_objs, ret
        except:
            print("Exception on LCAN_ReceiveFD!")
            raise
###############################################################################
'''
USBCAN1 Demo
'''

class BMS(Node):
    def __init__(self, lowest_soc, lowest_vol, max_charge_power) -> None:
        self.lowest_soc = lowest_soc
        self.lowest_vol = lowest_vol
        self.defalut_power = max_charge_power

        self.status = {
            "baterryState": {
                "voltage": 0,
                "electric": 0,
                "soc": 0,
                "SOH": 0,
                "alarm": 0,     # 动力电池告警级别
                "state": 0,     # 动力电池状态 0 上电 1 就绪 2 运行 3 错误 4 关机
            },
            "volt": {
                "MaxCellVolt": 0,       # 最高单体电压
                "MinCellVolt": 0,       # 最低单体电压
                "MaxCellVoltModule": 0, # 最高单体电压模块号
                "MaxCellVoltNo": 0,     # 最高单体电压模块内序号
                "MinCellVoltModule": 0, # 最低单体电压模块号
                "MinCellVoltNo": 0      # 最低单体电压模块内序号
            },
            "temp": {
                "MaxTemp": 0,           # 最高温度
                "MinTemp": 0,           # 最低温度
                "MaxTempModule": 0,     # 最高温度模块号
                "MaxTempNo": 0,         # 最高温度模块内序号
                "MinTempModule": 0,     # 最低温度模块号
                "MinTempNo": 0          # 最低温度模块内序号
            },
            "relay": {
                "PosRlyStr": 0,      # 总正继电器状态    0:Open；1:Close；2:Error
                "NegRlyStr": 0,      # 总负继电器状态
                "PreRlyStr": 0,      # 预充继电器状态
                "OffChrRlyStr": 0,   # 非车载充电（快充）继电器状态
                "ChrOFFCC2": 0,      # 车载充电机连接确认
                "ChrCommunication": 0,   # 充电机是否在线
                "ChrState": 0        # 充电状态
            },
            "alarm": {
                "ALM_CELL_OV": 0,            # 单体过压  0:无告警 1:1 级告警 2:2 级告警 3:3 级告警 4:4 级告警（最严重）
                "ALM_CELL_UV": 0,            # 单体欠压 
                "ALM_CELL_OT": 0,            # 电池高温
                "ALM_CELL_UT": 0,            # 电池低温
                "ALM_BATT_OV": 0,            # 整组过压
                "ALM_BATT_UV": 0,            # 整组欠压
                "ALM_CHRG_OCS": 0,           # 充电过流
                "ALM_DSCH_OCS": 0,           # 放电过流
                "ALM_BSU_OFFLINEBSU": 0,     # 离线
                "ALM_LEAK_OC": 0,            # 漏电流超限
                "ALM_BATT_DV": 0,            # 单体压差过大
                "ALM_BATT_DT": 0,            # 电池温差过大
                "ALM_HVREL_FAIL": 0,         # 电池高压异常
                "ALM_HALL_BREAK": 0,         # 霍尔断线
                "ALM_BATT_ERROR": 0,         # 电池异常保护
                "ALM_CELL_LBK": 0,           # 单体电压断线
                "ALM_CELL_TBK": 0,           # 单体温度断线
                "ALM_PRECHRG_FAIL": 0,       # 预充失败  
                "ALM_AUX_FAIL": 0,           # 继电器故障
                "ALM_BSU_FAULTBSU": 0        # 均衡故障
            }
        }
        self.has_usbToCan = False
        self.has_readBuf = False
        self.LCAN = LCAN()
        ret = self.LCAN.LCAN_OpenDevice(LCAN_USBCAN1, 0)
        if ret != LCAN_STATUS_OK:
            log.info("LYS USBCAN1 Open Device failed!")
        ret = self.initCAN()
        if ret:
            threading.Thread(target=self.recv, name="recv").start()
            # self.create_timer(1.0, self.recv)
        
    def initCAN(self):
        can_init_cfg = LCAN_INIT_CONFIG()
        can_init_cfg.accCode = 0x00000000
        can_init_cfg.accMask = 0xFFFFFFFF
        can_init_cfg.reserved = 0
        #single fliter
        can_init_cfg.fliter = 1
        #250k
        can_init_cfg.timing0 = 1
        can_init_cfg.timing1 = 28
        #normal
        can_init_cfg.mode = 0
        ret = self.LCAN.LCAN_InitCAN(LCAN_USBCAN1, 0, 0, can_init_cfg)
        if ret != LCAN_STATUS_OK:
            log.info("LYS USBCAN1 Init Failed!")
        else:
            print("LYS USBCAN1 Init Success!")
            self.has_usbToCan = True
        return self.has_usbToCan
        
    def recv(self):
        while True:
            try:    
                rcv_num = self.LCAN.LCAN_GetReceiveNum(LCAN_USBCAN1, LCAN_DeviceInd, LCAN_CanInd, 0) # 返回值为接收缓存里面的CAN帧数
                if rcv_num:
                    self.has_readBuf = True
                    rcv_msg, rcv_num = self.LCAN.LCAN_Receive(LCAN_USBCAN1, LCAN_DeviceInd, LCAN_CanInd, rcv_num)
                    for i in range(rcv_num):
                        if rcv_msg[i].id == 0x186040F0:
                            self.BaterryState(rcv_msg[i].data)
                        elif rcv_msg[i].id == 0x186140F0:
                            self.volt(rcv_msg[i].data)
                        elif rcv_msg[i].id == 0x186240F0:
                            self.temp(rcv_msg[i].data)
                        elif rcv_msg[i].id == 0x186340F0:
                            self.relay(rcv_msg[i].data)
                        elif rcv_msg[i].id == 0x186540F0:
                            self.alarm(rcv_msg[i].data)
                    # self.lowest()
                else:
                    self.has_readBuf = False
            except Exception as e:
                log.info("can recv error: {}".format(e))
            time.sleep(1)
    
    def BaterryState(self, data):
        data = list(data)
        self.status["baterryState"]["voltage"] = (data[1] * 255 + data[0]) * 0.1 
        self.status["baterryState"]["electric"] = (data[3] * 255 + data[2]) * 0.1 - 3200 + 12       # 偏移量待定
        self.status["baterryState"]["soc"] = data[4]
        self.status["baterryState"]["SOH"] = data[5]
        self.status["baterryState"]["alarm"] = data[6] & 0b1111
        self.status["baterryState"]["state"] = (data[6] >> 4) & 0b1111

    def volt(self, data):
        data = list(data)
        self.status["volt"]["MaxCellVolt"] = (data[1] * 255 + data[0]) 
        self.status["volt"]["MinCellVolt"] = (data[3] * 255 + data[2])
        self.status["volt"]["MaxCellVoltModule"] = data[4]
        self.status["volt"]["MinCellVoltModule"] = data[5]
        self.status["volt"]["MaxCellVoltNo"] = data[6]
        self.status["volt"]["MinCellVoltNo"] = data[7]
    
    def temp(self, data):
        data = list(data)
        self.status["temp"]["MaxTemp"] = data[0] -40 
        self.status["temp"]["MinTemp"] = data[1] - 40
        self.status["temp"]["MaxTempModule"] = data[2]
        self.status["temp"]["MaxTempNo"] = data[3]
        self.status["temp"]["MinTempModule"] = data[4]
        self.status["temp"]["MinTempNo"] = data[5]
    
    def relay(self , data):
        data = list(data)
        self.status["relay"]["PosRlyStr"] = data[0] & 0b11      # 1:Open；0:Close；2:Error
        self.status["relay"]["NegRlyStr"] = (data[0] >> 2) & 0b11
        self.status["relay"]["PreRlyStr"] = (data[0] >> 4) & 0b11
        self.status["relay"]["OffChrRlyStr"] =(data[1]) & 0b11
        self.status["relay"]["ChrOFFCC2"] = (data[2] >> 4) & 0b1
        self.status["relay"]["ChrCommunication"] = (data[2] >> 5) & 0b1
        self.status["relay"]["ChrState"] = data[3] & 0b1111
    
    def alarm(self, data):
        data = list(data)
        self.status["alarm"]["ALM_CELL_OV"] = data[0] & 0b1111
        self.status["alarm"]["ALM_CELL_UV"] = (data[0] >> 4) & 0b1111

        self.status["alarm"]["ALM_CELL_OT"] = data[1] & 0b1111
        self.status["alarm"]["ALM_CELL_UT"] = (data[1] >> 4) & 0b1111
        
        self.status["alarm"]["ALM_BATT_OV"] = data[2] & 0b1111
        self.status["alarm"]["ALM_BATT_UV"] = (data[2] >> 4) & 0b1111

        self.status["alarm"]["ALM_CHRG_OCS"] = data[3] & 0b1111
        self.status["alarm"]["ALM_DSCH_OCS"] = (data[3] >> 4) & 0b1111
        
        self.status["alarm"]["ALM_BSU_OFFLINEBSU"] = data[4] & 0b1111
        self.status["alarm"]["ALM_LEAK_OC"] = (data[4] >> 4) & 0b1111
        
        self.status["alarm"]["ALM_BATT_DV"] = data[5] & 0b1111
        self.status["alarm"]["ALM_BATT_DT"] = (data[5] >> 4) & 0b1111

        self.status["alarm"]["ALM_HVREL_FAIL"] = data[6] & 0b11
        self.status["alarm"]["ALM_HALL_BREAK"] = (data[6] >> 2) & 0b11
        self.status["alarm"]["ALM_BATT_ERROR"] = (data[6] >> 4) & 0b11
        self.status["alarm"]["ALM_CELL_LBK"] = (data[6] >> 6) & 0b11

        self.status["alarm"]["ALM_CELL_TBK"] = data[7] & 0b11
        self.status["alarm"]["ALM_PRECHRG_FAIL"] = (data[7] >> 2) & 0b11
        self.status["alarm"]["ALM_AUX_FAIL"] = (data[7] >> 4) & 0b11
        self.status["alarm"]["ALM_BSU_FAULTBSU"] = (data[7] >> 6) & 0b11

    def lowest(self):
        if self.status["baterryState"]["soc"] <= self.lowest_soc:
            log.info("电池电量不足，禁止充电")
            log.info("lowest_soc = {}".format(self.lowest_soc))



