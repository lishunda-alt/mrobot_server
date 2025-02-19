import asyncio
from asyncio import Lock
import threading
import time
import threading
from .dependent_library import log
import json
import asyncio
import struct
from datetime import datetime, timedelta
from crcmodbus import checksum
import random
import string
import os
import math
from ament_index_python.packages import get_package_share_directory
config_file = os.path.join(
    get_package_share_directory('robot_server'),
    'billingModel.json')
with open(config_file, 'r', encoding='UTF-8') as f:
    defaultBillingModel = json.load(f)

class TcpServer():
    def __init__(self):
        self.connected = False
        self.registed = False
        self.chargerNumber = None
        self.serialNumber = 0
        self.startChargeSemaphore = threading.Semaphore(0)
        self.stopChargeSemaphore = threading.Semaphore(0)
        self.billingModelSemaphore = threading.Semaphore(0)
        self.rechargeShotStatus = 0
        self.chargerModel = 0   # 充电模式 0表示自动充电 1表示按金额充电 2表示按电量充电 3表示按时间充电
        self.package = {
            "packHead": (0x55 << 8) + 0xAA,     # 2 bytes, head
            "frameLength": None,                # 2 bytes
            "frameLengthNot": None,             # 2 bytes
            "serialNumber": None,               # 1 bytes, 
            "dataObject": None,                 # 1 bytes
            "frameCode": None,                  # 1 bytes
            "data": None,                       # N bytes
            "frameCRC": None,                   # 2 bytes
            "frameEnd": 0x0D                    # 1 bytes
        }
        self.task = {
            "openCharge": {
                "status": False,
                "data": None,
                "packageData": None
            },
            "closeCharge": {
                "status": False,
                "data": None
            },
            "configBillingModel":{
                "status": False,
                "data": None,
                "packageData": None
            }
        }
        self.sendPack = self.package
        self.endBilld = False
        self.chargeSerialNumber = ""
        self.userID = ""        
        self.status = {
            "heartBeat": {
                "status": 0,
                "temperature": 0
            },
            "rechargeShot": {
                "accumulatedChargingPower": 0,   # 总充电电量
                "accumulatedRunningTime": 0,     # 总运行时间
                "accumulatedChargingTime": 0,    # 总的充电时间 
                "chargingStatus": 0, # 0x00表示待机中；0x01表示启动充电中；0x02表示充电中；0x03 禁止充电
                "rechargeShotStatus": 0, # 0x00表示枪已归位；0x01表示未连接；0x02表示枪已连接；
                "floorLock": 0,  # 0x00表示地锁打开；0x01表示地锁锁住；
                "requestsVlotage": 0,    # BMS电压需求
                "requestsElectric": 0,   # BMS电流需求
                "chargeMode": 0,     # BMS充电模式 1：恒压充电 2：恒流充电
                "chargeVlotage": 0,  # 充电电压
                "chargeElectric": 0, # 充电电流
                "currentChargingPower": 0,   # 已充电量
                "currentChargingTime": 0,    # 已充电时间
                "residualTime": 0,   # 估算剩余充电时间
                "soc": 0,
                "maxMonomerVlotage": 0,  # 最高单体动力蓄电池电池电压
                "maxMonomerVlotageIndex": 0,     # 最高单体动力蓄电池电压所在编号
                "minMonomerVlotage": 0,      # 最低单体动力蓄电池电池电压
                "minMonomerVlotageIndex": 0, # 最低单体动力蓄电池电压所在编号
                "maxTemperature": 0,     # 最高动力蓄电池温度
                "maxTemperaturePoint": 0,     # 最高温度检测点
                "Electricityfreee": 0,    # 当前电费金额
                "serviceFree": 0,         # 当前服务费金额
                "serialNumbers": "",      # 本次充电流水号
                "topCharingPower": 0,    # 尖时段充电电量
                "topElectricityfreee": 0,    # 尖时段电费金额
                "topServiceFree": 0,     # 尖时段服务费金额
                "peakCharingPower": 0,   # 峰时段充电电量
                "peakElectricityfreee": 0,   # 峰时段电费金额
                "peakServiceFree": 0,        # 峰时段服务费金额
                "levelCharingPower": 0,      # 平时段充电电量
                "levelElectricityfreee": 0,  # 平时段电费金额
                "levelServiceFree": 0,       # 平时段服务费金额
                "valleyCharingPower": 0,     # 谷时段充电电量
                "valleyElectricityfreee": 0, # 谷时段电费金额
                "valleyServiceFree": 0       # 谷时段服务费金额
            },
            "chargeError": {
                "commonAlarm": 0,    # 充电机故障总告警  0x00：表示未发生  0x01：发生
                "notRectifierModule": 0,    # 无可用整流模块 0x00：表示未发生  0x01：发生
                "chargeTemperature": 0,     # 充电机温度过高 0x00：表示未发生  0x01：发生
                "ED10ConnectError": 0,          # ED10通信中断 0x00：表示未发生  0x01：发生
                "controlConnectError": 0,        # 充电控制器通信中断 0x00：表示未发生  0x01：发生
                "electricMeter": 0,              # 电度表通信中断 0x00：表示未发生  0x01：发生
                "recordingAeraOver": 0,          # 记录区已满 0x00：表示未发生  0x01：发生
                "dcFuse": 0,                     # 直流熔断器故障
                "H.T_contactor": 0,              # 高压接触器故障  
                "chargingUncontrol": 0,          # 充电机输出不可控
                "chargeDischargeCircuit": 0,     # 充电机泄放回路故障
                "outputShortCircuit": 0,         # 充电机输出短路
                "dcContactorClose": 0,           # 直流接触器不能闭合
                "dcContactorDisconnect": 0       # 直流接触器不能断开
            },
            "settleBilld": {
                "serialNumbers": "",              # 充电流水号
                "userID": "",                     # 用户卡号      
                "power": 0,                      # 本次充电电量
                "electricityfreee": 0,           # 电费总额
                "serviceFree": 0,                # 服务费总额
                "startTime": 0,                  # 充电起始时间
                "stopTime": 0,                   # 充电结束时间
                "endSOC": 0,                     # 结束SOC
                "stopReason": 0,                 # 停机原因
                "topCharingPower": 0,            # 尖时段充电电量
                "topElectricityfreee": 0,        # 尖时段电费金额
                "topServiceFree": 0,             # 尖时段服务费金额
                "peakCharingPower": 0,           # 峰时段充电电量
                "peakElectricityfreee": 0,       # 峰时段电费金额
                "peakServiceFree": 0,            # 峰时段服务费金额
                "levelCharingPower": 0,          # 平时段充电电量
                "levelElectricityfreee": 0,      # 平时段电费金额
                "levelServiceFree": 0,           # 平时段服务费金额
                "valleyCharingPower": 0,         # 谷时段充电电量
                "valleyElectricityfreee": 0,     # 谷时段电费金额
                "valleyServiceFree": 0,          # 谷时段服务费金额
                "startModel": 0,                 # 启动充电方式
                "timeBilling": []
            },
            "requestsBilld":{
                "serialNumbers": "",
                "userID": "", 
                "chargeModel": 0,
                "data": 0,
                "balance": 0,
                "electricityDiscount": 0,
                "serverDiscount": 0,
                "startTime": ""
            },
            "requestsBillingModel":{
                "0": {
                    "startTime": "00:00:00",
                    "endTime": "24:00:00",
                    "electricPrice":1.00,       # 电费
                    "servicePrice":1.00         # 服务费
                }
            },
            "displayBilling": {
                "serialNumbers": "",
                "parkingNumber": "",
                "carNumber": "",
                "chargeModel": None,
                "chargeData": None
            },
            "ratedCapacity": None,
            "rateVoltage": None
        }
    

    def register(self, recvPackage, writer):
        PACK_HEAD_FMT_STR = '!16s16sBBB16s4B4B'
        msg = struct.unpack(PACK_HEAD_FMT_STR, recvPackage["data"])
        log.info("Charging pile start to registed!")
        self.chargerNumber = msg[0]
        cp56time2a_bytes = self.cp56time2a_hex_str()
        PACK_HEAD_FMT_STR = "!B16s8s"
        data = struct.pack(PACK_HEAD_FMT_STR, 0, self.chargerNumber, cp56time2a_bytes)

        recvPackage["frameCode"] = 2
        recvPackage["data"] = data
        recvPackage["frameLength"] = 5 + len(data)
        recvPackage["frameLengthNot"] = ~recvPackage["frameLength"] + 65536
        data = self.packData(recvPackage)
        writer.write(data)

    def heartBeat(self, recvPackage, writer):
        self.status["heartBeat"]["status"] = recvPackage["data"][0]
        self.status["heartBeat"]["temperature"] = recvPackage["data"][1] - 50
        
        recvPackage["frameCode"] = 0x04
        recvPackage["data"] = bytes([0])
        data = self.packData(recvPackage)
        writer.write(data)
        
    def billingModel(self, recvPackage, writer):
        recvPackage["frameCode"] = 0x09
        recvPackage["data"] = struct.pack("!HHHHIIIIBHB", 100, 100, 100, 100, 100000, 100000, 100000, 100000, 1, 1440, 1) + bytes(141)

        data = self.packData(recvPackage)
        writer.write(data)

    def defaultBillingModel(self, param):
        time.sleep(1)
        self.configBillingModel(param)

    def configBillingModel(self, param):
        servicePrice = bytes()
        electricPrice = bytes()
        price = bytes()
        periodLen = 0
        for i in range(4):
            param[str(i)]["type"] = str(i)
            servicePrice += struct.pack("!H", int(param[str(i)]["servicePrice"] * 100))
            electricPrice += struct.pack("!I", int(param[str(i)]["electricPrice"] * 100000))
            if (param[str(i)]["startTime"] == param[str(i)]["endTime"]):
                param.pop(str(i))
                continue
            periodLen += 1
        res = bytes()
        for i in range(48 - periodLen):
            res += struct.pack("!HB", 0, 0)
        periodLen = struct.pack("!B", periodLen)
        sorted_data = sorted(param.values(), key=lambda x: x["startTime"])
        for data in sorted_data:
            price += struct.pack("!HB", self.toMinute(data["startTime"], data["endTime"]), int(data["type"]) + 1)
        
        self.task["configBillingModel"]["packageData"] = servicePrice + electricPrice + periodLen + price + res
        self.task["configBillingModel"]["status"] = True
        self.billingModelSemaphore.acquire()
        if self.task["configBillingModel"]["data"] == 0:
            self.status["requestsBillingModel"] = param
      
    def getBytes(self, period, name, ranks):
        data = bytes()
        for i in range(4):
            if (i + 1) > len(period):
                data += struct.pack("!H", 0)
            else:
                data += struct.pack("!H", period[ranks[i]][name])
        return data

    def requestsConfigBillingModel(self, writer):
        sendPackage = self.package
        sendPackage["frameCode"] = 0x79
        sendPackage["serialNumber"] = self.serialNumber
        sendPackage["dataObject"] = 0
        sendPackage["data"] = self.task["configBillingModel"]["packageData"]
        data = self.packData(sendPackage)
        log.info("send requests config billing model data: {}".format(data))
        writer.write(data)
        self.serialNumber += 1
        if self.serialNumber > 255:
            self.serialNumber = 0
        self.endBilld = False

    def getChargeDataCommand(self, writer):
        sendPack = self.package
        sendPack["frameCode"] = 0x012
        sendPack["serialNumber"] = self.serialNumber
        sendPack["dataObject"] = 1      # 直流桩, 1号枪
        sendPack["data"] = bytes()
        data = self.packData(sendPack)
        writer.write(data)
        self.serialNumber += 1
        if self.serialNumber > 255:
            self.serialNumber = 0

    def getChargeData(self, recvPackage):
        try:
            data = struct.unpack("!IIIBBBBHHBHHIHHBHBHBBBII22s48s", recvPackage["data"])       
            self.status["rechargeShot"]["accumulatedChargingPower"] = data[0]
            self.status["rechargeShot"]["accumulatedRunningTime"] = data[1]
            self.status["rechargeShot"]["accumulatedChargingTime"] = data[2]
            self.status["rechargeShot"]["chargingStatus"] = data[3]
            self.rechargeShotStatus = data[4]
            self.status["rechargeShot"]["floorLock"] = data[6]
            self.status["rechargeShot"]["requestsVlotage"] = data[7] * 0.1
            self.status["rechargeShot"]["requestsElectric"] = data[8] * 0.1
            self.status["rechargeShot"]["chargeMode"] = data[9]
            self.status["rechargeShot"]["chargeVlotage"] = data[10] * 0.1
            self.status["rechargeShot"]["chargeElectric"] = data[11] * 0.1
            self.status["rechargeShot"]["currentChargingPower"] = data[12] * 0.001
            self.status["rechargeShot"]["currentChargingTime"] = data[13]
            # self.status["rechargeShot"]["residualTime"] = data[14]
            self.status["rechargeShot"]["soc"] = data[15]
            self.status["rechargeShot"]["maxMonomerVlotage"] = data[16] * 0.01
            self.status["rechargeShot"]["minMonomerVlotageIndex"] = data[17]
            self.status["rechargeShot"]["minMonomerVlotage"] = data[18] * 0.01
            self.status["rechargeShot"]["minMonomerVlotageIndex"] = data[19]
            self.status["rechargeShot"]["maxTemperature"] = data[20] - 50
            self.status["rechargeShot"]["maxTemperaturePoint"] = data[21]

            self.status["rechargeShot"]["Electricityfreee"] = data[22] * 0.01
            self.status["rechargeShot"]["serviceFree"] = data[23] * 0.01
            self.status["rechargeShot"]["serialNumbers"] = ""
            for byte in data[24]:
                if byte != 0x00:
                    self.status["rechargeShot"]["serialNumbers"] = data[24].decode()

        except Exception as e:
            log.info("getChargeData error: {}".format(e))

    def getChargeErrorCommand(self, writer):
        sendPack = self.package
        sendPack["frameCode"] = 0x014
        sendPack["serialNumber"] = self.serialNumber
        sendPack["dataObject"] = 1      # 直流桩, 1号枪
        sendPack["data"] = bytes()
        data = self.packData(sendPack)
        writer.write(data)
        self.serialNumber += 1
        if self.serialNumber > 255:
            self.serialNumber = 0
    
    def getChargeError(self, recvPackage):
        try:
            data = recvPackage["data"]
            self.status["chargeError"]["commonAlarm"] = data[0]
            self.status["chargeError"]["notRectifierModule"] = data[2]
            self.status["chargeError"]["chargeTemperature"] = data[3]
            self.status["chargeError"]["ED10ConnectError"] = data[6]
            self.status["chargeError"]["controlConnectError"] = data[7]
            self.status["chargeError"]["electricMeter"] = data[8]
            self.status["chargeError"]["recordingAeraOver"] = data[10]
            self.status["chargeError"]["dcFuse"] = data[12]
            self.status["chargeError"]["H.T_contactor"] = data[13]
            self.status["chargeError"]["chargingUncontrol"] = data[14]
            self.status["chargeError"]["chargeDischargeCircuit"] = data[15]
            self.status["chargeError"]["outputShortCircuit"] = data[16]
            self.status["chargeError"]["dcContactorClose"] = data[21]
            self.status["chargeError"]["dcContactorDisconnect"] = data[22]
        except Exception as e:
            log.info("getChargeError error: {}".format(e))
    
    def getCarStatusCommand(self, writer):
        sendPack = self.package
        sendPack["frameCode"] = 0x018
        sendPack["serialNumber"] = self.serialNumber
        sendPack["dataObject"] = 1      # 直流桩, 1号枪
        sendPack["data"] = bytes()
        data = self.packData(sendPack)
        writer.write(data)
        self.serialNumber += 1
        if self.serialNumber > 255:
            self.serialNumber = 0
    
    def openCharge(self, param, targetDegree=0):
        if param:
            chargeSerialNumber = self.chargeSerialNumber = (22 - len(param.get("serialNumbers"))) * "0" + param.get("serialNumbers")
            userID = self.userID = self.generate_random_serial(16)
            chargerModel = self.chargerModel = param.get("chargeModel")
            configData = 0
            if param.get("chargeModel") != 0:
                configData = param.get("data")
            balance = int(param.get("balance") * 100)
            electricityDiscount = int(param.get("electricityfreeeDiscount"))
            serverDiscount = int(param.get("serviceFreeeDiscount"))
            data = struct.pack("!22s16sBHIBB", self.chargeSerialNumber.encode(), self.userID.encode(), self.chargerModel, 
                                          configData, balance, electricityDiscount, serverDiscount)
        else:
            chargeSerialNumber = self.generate_random_serial(22)
            userID = self.generate_random_serial(16)
            if targetDegree == -1:
                chargerModel = 0 
                configData = 0
            else:
                chargerModel = 2
                configData = targetDegree
            balance = 1000 * 10000
            electricityDiscount = 100
            serverDiscount = 100
            data = struct.pack("!22s16sBHIBB", chargeSerialNumber.encode(), userID.encode(), chargerModel, 
                                          configData, balance, electricityDiscount, serverDiscount)
        self.status["requestsBilld"]["serialNumbers"] = chargeSerialNumber
        self.status["requestsBilld"]["userID"] = userID
        self.status["requestsBilld"]["chargeModel"] = chargerModel
        self.status["requestsBilld"]["data"] = configData
        self.status["requestsBilld"]["balance"] = balance
        self.status["requestsBilld"]["electricityDiscount"] = electricityDiscount
        self.status["requestsBilld"]["serverDiscount"] = serverDiscount
        self.status["requestsBilld"]["startTime"] = time.time()

        self.task["openCharge"]["status"] = True
        self.task["openCharge"]["packageData"] = data
        self.startChargeSemaphore.acquire()
    
    def closeCharge(self):
        self.task["closeCharge"]["status"] = True
        self.stopChargeSemaphore.acquire()        

    def requestsCharge(self, writer):
        sendPackage = self.package
        sendPackage["frameCode"] = 0x44
        sendPackage["serialNumber"] = self.serialNumber
        sendPackage["dataObject"] = 1
        sendPackage["data"] = self.task["openCharge"]["packageData"]
        data = self.packData(sendPackage)
        log.info("send requests charge data: {}".format(data))
        writer.write(data)
        self.serialNumber += 1
        if self.serialNumber > 255:
            self.serialNumber = 0
        self.endBilld = False

    def getRequestsCharge(self, recvPackage):
        data = recvPackage["data"][0]
        if data == 0:
            log.info("充电无效")
        elif data == 1:
            log.info("正在启动充电")
        elif data == 2:
            log.info("枪未连接，不可启动充电")
        elif data == 3:
            log.info("正在充电，不可启动充电")
        elif data == 4:
            log.info("设备处于维护，不可启动充电")
        elif data == 5:
            log.info("桩被锁定，不可启动充电")
        elif data == 5:
            log.info("桩处于本地操作交互流程，不能响应远程启机")
        else:
            log.info("其它问题，data = {}".format(data))
        return data
    
    def requestsStopCharge(self, writer):
        sendPackage = self.package
        sendPackage["frameCode"] = 0x46
        sendPackage["data"] = struct.pack("!BB", 1, 0)  # APP停机
        sendPackage["serialNumber"] = self.serialNumber
        sendPackage["dataObject"] = 1
        data = self.packData(sendPackage)
        writer.write(data)
        self.serialNumber += 1
        if self.serialNumber > 255:
            self.serialNumber = 0
    
    def getRequestsStopCharge(self, recvPackage):
        data = recvPackage["data"][0]
        if data == 0:
            log.info("停机")
        else:
            log.info("停机失败")
        return data
    
    def settleBills(self, recvPackage, writer):
        try:
            log.info("settleBills recvPackage: {}".format(recvPackage))
            print(recvPackage["data"])
            if 1: #recvPackage["data"]:
                data = struct.unpack("!22s16sIII8s8sBBIIIIIIIIIIIIB18s", recvPackage["data"])
                self.status["settleBilld"]["serialNumbers"] = data[0].decode()
                self.status["settleBilld"]["userID"] = data[1].decode()
                self.status["settleBilld"]["power"] = data[2]
                self.status["settleBilld"]["electricityfreee"] = data[3] * 0.01
                self.status["settleBilld"]["serviceFree"] = data[4] * 0.01
                self.status["settleBilld"]["startTime"] = self.parse_cp56time2a(data[5])
                self.status["settleBilld"]["stopTime"] = self.parse_cp56time2a(data[6])
                self.status["settleBilld"]["endSOC"] = data[7]
                self.status["settleBilld"]["stopReason"] = data[8]

                self.status["settleBilld"]["topCharingPower"] = data[9] * 0.001
                self.status["settleBilld"]["topElectricityfreee"] = data[10] * 0.01
                self.status["settleBilld"]["topServiceFree"] = data[11] * 0.01

                self.status["settleBilld"]["peakCharingPower"] = data[12] * 0.001
                self.status["settleBilld"]["peakElectricityfreee"] = data[13] * 0.01
                self.status["settleBilld"]["peakServiceFree"] = data[14] * 0.01

                self.status["settleBilld"]["levelCharingPower"] = data[15] * 0.001
                self.status["settleBilld"]["levelElectricityfreee"] = data[16] * 0.01
                self.status["settleBilld"]["levelServiceFree"] = data[17] * 0.01

                self.status["settleBilld"]["valleCharingPower"] = data[18] * 0.001
                self.status["settleBilld"]["valleElectricityfreee"] = data[19] * 0.01
                self.status["settleBilld"]["valleServiceFree"] = data[20] * 0.01
                startNumbers = endNumbers = None
                for i, t in self.status["requestsBillingModel"].items():
                    _list = list(range(self.toSec(self.status["requestsBillingModel"][i]["startTime"]), 
                                                                            self.toSec(self.status["requestsBillingModel"][i]["endTime"])))
                    self.status["requestsBillingModel"][i]["range"] = (_list[0], _list[-1])
                    _startTime = self.toSec(self.status["settleBilld"]["startTime"][-8:])
                    _stopTime = self.toSec(self.status["settleBilld"]["stopTime"][-8:])
                    if _startTime in range(self.status["requestsBillingModel"][i]["range"][0], self.status["requestsBillingModel"][i]["range"][1]):
                        startNumbers = i
                    if _stopTime in range(self.status["requestsBillingModel"][i]["range"][0], self.status["requestsBillingModel"][i]["range"][1]):
                        endNumbers = i
                
                timeBilling = list()
                j = 0
                if startNumbers and endNumbers:
                    if startNumbers == endNumbers:
                        billing = {
                        "ruleType": startNumbers,
                        "electricityFreee": data[10 + int(startNumbers) * 3] * 0.01,
                        "serviceFree": data[11 + int(startNumbers) * 3] * 0.01,
                        "ruleBeginTime": self.status["requestsBillingModel"][startNumbers]["startTime"],
                        "ruleEndTime": self.status["requestsBillingModel"][endNumbers]["endTime"],
                        "chargeBeginTime": self.status["settleBilld"]["startTime"],
                        "chargeEndTime": self.status["settleBilld"]["stopTime"]
                        }
                        timeBilling.append(billing)
                    else:
                        for i in range(int(endNumbers) - int(startNumbers) + 1):
                            startTime = endTime = ""
                            if self.status["requestsBillingModel"].get(str(i)):
                                startTime = self.status["requestsBillingModel"][str(i)]["startTime"]
                                endTime = self.status["requestsBillingModel"][str(i)]["endTime"]
                            chargeBeginTime = startTime
                            chargeEndTime = endTime
                            if str(i) == startNumbers:
                                chargeBeginTime = self.status["settleBilld"]["startTime"]
                            if str(i) == endNumbers:
                                chargeEndTime = self.status["settleBilld"]["stopTime"]
                            billing = {
                                "ruleType": i,
                                "electricityFreee": data[10 + j] * 0.01,
                                "serviceFree": data[11 + j] * 0.01,
                                "ruleBeginTime": startTime,
                                "ruleEndTime": endTime,
                                "chargeBeginTime": chargeBeginTime,
                                "chargeEndTime": chargeEndTime
                            }
                            j += 3
                            timeBilling.append(billing)
                
                self.status["settleBilld"]["startModel"] = data[21]

                self.status["settleBilld"]["timeBilling"] =timeBilling

                recvPackage["frameCode"] = 0x48
                recvPackage["data"] = struct.pack("!B22s16sI", 0, self.status["settleBilld"]["serialNumbers"].encode(), 
                                                self.status["settleBilld"]["userID"].encode(), 100000)     # 结算余额
                self.endBilld = True
                data = self.packData(recvPackage)
                writer.write(data)
            else:
                log.info("settleBills error!")
        except Exception as e:
            log.info("settleBills error: {}".format(e))

    def getBillingModel(self, recvPackage):
        return recvPackage["data"][0]

    def requestsCarStatus(self, writer):
        sendPackage = self.package
        sendPackage["frameCode"] = 0x18
        sendPackage["serialNumber"] = self.serialNumber
        sendPackage["dataObject"] = 0
        sendPackage["data"] = bytes()
        data = self.packData(sendPackage)
        log.info("send requests car status data: {}".format(data))
        writer.write(data)
        self.serialNumber += 1
        if self.serialNumber > 255:
            self.serialNumber = 0
    
    def getCarStatus(self, recvPackage):
        try:
            print(recvPackage)
            self.status["ratedCapacity"] = (recvPackage["data"][1] * 256 + recvPackage["data"][2]) * 0.1
            self.status["rateVoltage"] = (recvPackage["data"][3] * 256 + recvPackage["data"][4]) * 0.1
        except Exception as e:
            log.info("get Car Status error: {}".format(e))

    def packData(self, recvPackage):
        recvPackage["frameLength"] = 5 + len(recvPackage["data"])
        recvPackage["frameLengthNot"] = ~recvPackage["frameLength"] + 65536
        p_data = struct.pack("!BBB", recvPackage["serialNumber"], 
                             recvPackage["dataObject"], recvPackage["frameCode"]) + recvPackage["data"]
        recvPackage["frameCRC"] = checksum(p_data)
        
        data = struct.pack("!HHHBBB", recvPackage["packHead"], recvPackage["frameLength"], recvPackage["frameLengthNot"], 
                           recvPackage["serialNumber"], recvPackage["dataObject"], recvPackage["frameCode"]) \
                            + recvPackage["data"] + struct.pack("!HB", recvPackage["frameCRC"], recvPackage["frameEnd"])
        return data
    
    def unpackHead(self, data):
        try:
            dataLen = len(data) - 12
            PACK_HEAD_FMT_STR = "!HHHBBB" + str(dataLen) + "s" + "HB"
            recvPackage = self.package
            msg = struct.unpack(PACK_HEAD_FMT_STR, data)
            recvPackage["frameLength"] = msg[1]
            recvPackage["frameLengthNot"] = msg[2]
            recvPackage["serialNumber"] = msg[3]
            recvPackage["dataObject"] = msg[4]
            recvPackage["frameCode"] = msg[5]
            recvPackage["data"] = msg[6]
            recvPackage["frameCRC"] = msg[7]
            return recvPackage
        except Exception as e:
            log.info("unpack head error!e: {}".format(e))
    
    def cp56time2a_hex_str(self):
        now = datetime.now()
        s = now.strftime("%Y%m%d%H%M%S")[2:]
        cp56time2a = [s[i:i + 2] for i in range(0, len(s), 2)]
        seconds = cp56time2a[-1:][0]
        cp56time2a = cp56time2a[:-1]
        seconds_hex = hex(int(seconds) * 1000)[2:]  # ms 转hex
        seconds_hex_split = [seconds_hex[i:i + 2] for i in range(0, len(seconds_hex), 2)]
        cp56time2a = [hex(int(x))[2:].zfill(2) for x in cp56time2a]
        cp56time2a_hex_list = cp56time2a + seconds_hex_split
        _list = [int(item, 16) for item in cp56time2a_hex_list]
        _list.append(0)
        return bytes(_list[::-1])
    
    def parse_cp56time2a(self, data):
        time_list = data[::-1]
        year = time_list[0]
        month = time_list[1]
        day = time_list[2]
        hour = time_list[3]
        minute = time_list[4]
        second = time_list[5]
        time_str = '20' + f"{year:02}-{month:02}-{day:02} {hour:02}:{minute:02}:{second:02}"
        return time_str


    def generate_random_serial(self, length):
        characters = string.ascii_letters + string.digits
        serial = ''.join(random.choice(characters) for _ in range(length))
        return serial

    def toMinute(self, _time1, _time2):
        t1 = _time1.split(':')
        t2 = _time2.split(':')
        return (int(t2[0]) * 60 + int(t2[1])) - (int(t1[0]) * 60 + int(t1[1]))
    
    def toSec(self, t):
        t1 = t.split(':')
        return int(t1[0]) * 60 * 60 + int(t1[1]) * 60

tcpServer = TcpServer()

class EchoServerProtocol(asyncio.Protocol):
    def connection_made(self, transport):
        global tcpServer
        peername = transport.get_extra_info('peername')
        print('Connection from {}'.format(peername))
        self.transport = transport
        self.server = self
        tcpServer.connected = True
        self.lock = Lock()
        time.sleep(1)
        
    def data_received(self, data):
        global tcpServer
        recvPackage = tcpServer.unpackHead(data)
        if not recvPackage:
            return
        if recvPackage["frameCode"] == 0x01:    
            tcpServer.register(recvPackage, self.transport)
        elif recvPackage["frameCode"] == 0x03:  
            tcpServer.heartBeat(recvPackage, self.transport)
        elif recvPackage["frameCode"] == 0x05:
            tcpServer.billingModel(recvPackage, self.transport)
            tcpServer.registed = True
            threading.Thread(target=tcpServer.defaultBillingModel, name="defaultBillingModel", args=(defaultBillingModel,)).start()
            new_loop = asyncio.new_event_loop()
            threading.Thread(target=self.start_async_server, name="start_async_server", 
                             args=(new_loop,)).start()
            
        elif recvPackage["frameCode"] == 0x1E:
            tcpServer.getChargeData(recvPackage)
        elif recvPackage["frameCode"] == 0x1F:
            tcpServer.getChargeError(recvPackage)
        elif recvPackage["frameCode"] == 0x43:
            tcpServer.task["openCharge"]["data"] = tcpServer.getRequestsCharge(recvPackage)
            tcpServer.task["openCharge"]["status"] = False
            tcpServer.startChargeSemaphore.release()
            tcpServer.requestsCarStatus(self.transport)
        elif recvPackage["frameCode"] == 0x45:
            tcpServer.task["closeCharge"]["data"] = tcpServer.getRequestsStopCharge(recvPackage)
            tcpServer.task["closeCharge"]["status"] = False
            tcpServer.stopChargeSemaphore.release()    
            tcpServer.status["ratedCapacity"] = None   
        elif recvPackage["frameCode"] == 0x54:
            tcpServer.settleBills(recvPackage, self.transport)
        elif recvPackage["frameCode"] == 0x73:
            tcpServer.task["configBillingModel"]["data"] = list(recvPackage["data"])[0]  # tcpServer.getBillingModel(recvPackage)
            tcpServer.task["configBillingModel"]["status"] = False
            tcpServer.billingModelSemaphore.release()
        elif recvPackage["frameCode"] == 0x1D:
            tcpServer.getCarStatus(recvPackage)
    
    def start_async_server(self, loop):
        asyncio.set_event_loop(loop)
        loop.run_until_complete(self.writeServer())

    async def writeServer(self):
        while tcpServer.registed:
            if tcpServer.status["ratedCapacity"]:
                try:
                    data = None
                    # print(tcpServer.status["ratedCapacity"], tcpServer.status["rateVoltage"])
                    if tcpServer.status["ratedCapacity"]:
                        if tcpServer.status["requestsBilld"]["chargeModel"] == 0:
                            data = (tcpServer.status["ratedCapacity"] - ((tcpServer.status["ratedCapacity"] / 100) * tcpServer.status["rechargeShot"]["soc"])) / tcpServer.status["rechargeShot"]["chargeElectric"]
                        elif tcpServer.status["requestsBilld"]["chargeModel"] == 2:
                            data = 1000 * (tcpServer.status["requestsBilld"]["data"] - tcpServer.status["rechargeShot"]["currentChargingPower"]) / tcpServer.status["rateVoltage"] / tcpServer.status["rechargeShot"]["chargeElectric"]
                        elif tcpServer.status["requestsBilld"]["chargeModel"] == 1:
                            for name, info in tcpServer.status["requestsBillingModel"].items():
                                start_time = datetime.strptime(info["startTime"], "%H:%M:%S").time()
                                if info["endTime"] == "24:00:00":
                                    end_time = datetime.strptime("23:59:59", "%H:%M:%S").time()
                                else:
                                    end_time = datetime.strptime(info["endTime"], "%H:%M:%S").time()
                                current_time = datetime.now().time()
                                if start_time <= current_time <= end_time:
                                    free = info["servicePrice"] + info["electricPrice"]
                                    degree = tcpServer.status["requestsBilld"]["data"] / free
                                    data = 1000 * (degree - tcpServer.status["rechargeShot"]["currentChargingPower"]) / tcpServer.status["rateVoltage"] / tcpServer.status["rechargeShot"]["chargeElectric"]
                    tcpServer.status["rechargeShot"]["residualTime"] = int(math.ceil(data * 60))
                except Exception as e:
                    print(e)
            async with self.lock:
                if tcpServer.task["openCharge"]["status"]:
                    tcpServer.requestsCharge(self.transport)
                    time.sleep(0.1)
                elif tcpServer.task["closeCharge"]["status"]:
                    tcpServer.requestsStopCharge(self.transport)
                    time.sleep(0.1)
                elif tcpServer.task["configBillingModel"]["status"]:
                    tcpServer.requestsConfigBillingModel(self.transport)
                    time.sleep(0.1)
                tcpServer.getChargeDataCommand(self.transport)
                tcpServer.getChargeErrorCommand(self.transport)
            time.sleep(1)

    def connection_lost(self, exc):
        tcpServer.connected = False
        tcpServer.registed = False
        log.info('connection lost')
        if hasattr(self, 'periodic_task'):
            self.periodic_task.cancel()
            print("cancel")

    def send_message(self, message):
        self.transport.write(message.encode())

async def main(ip, port):
    global server
    loop = asyncio.get_running_loop()
    server = await loop.create_server(
        lambda: EchoServerProtocol(),
        ip, port)
    async with server:
        await server.serve_forever()

def startServer(ip = "0.0.0.0", port = 2888):
    asyncio.run(main(ip, port))
    

threading.Thread(target=startServer, args=("192.168.192.170", 2888)).start()
