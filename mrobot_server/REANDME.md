# 简介 AGV服务程序

# 1、规范：
小车和后台服务之间采用MQTT-tcp方式进行通信
mqtt服务器配置：
    IP：8.135.36.112
    端口：1883
    协议：tcp

    小车账号：mrclient
    密码：client@2023

    调度账号：mrbrain
    密码：mr@2023


# 2、消息主题规范

    小车发布状态消息主题：mrclient/{id}/status

    小车注册报告消息主题：mrclient/regist/{id}

    小车读取服务器发布的消息主题：mrrserver/{id}/cmd

    小车接受服务器发布的消息主题之后回复的话题: mrclient/{id}/return

    其中id为小车唯一身份ID

# 3、消息格式
    
    接受的话题以及发布的话题均采用以下方式：

    {"id":"","cmd":"","tick":1641953334284,["msgId":"1"],"payload":{}}

    id：发布消息者的身份ID，调度身份ID为mgrsys；机器人的ID由自己计算，必须全局唯一，重启后ID不变化，和消息主题携带的ID一致
    
    cmd：指令，对于需要回复的指令，仍使用该值
    
    tick：指令发布时间，秒
    
    msgId：消息ID，可选，对于需要回复的消息，可在下发指令时携带该字段，对方回复时同样携带该字段作为响应。
    
    payload：cmd指令对应的JSON参数

# 4、 指令说明

    (1) 小车发布状态话题；小车-> MQTT -> 调度

        主题：mrclient/{id}/status
        cmd:/api/v1/status/
        msgId：不需要
        payload：{
                      "1": run_status,    # 0=None/1=WAITING/2=RUNING/3=SUSPENDED/4=COMPLETED/5=FAILED/6=CANCELED
                      "2": target_id,     # 当前任务要去的站点，仅当task_type为2或3时有效
                      "3": now_point,     # 现在/刚经过的站点
                      "4": mission_code,  # 操作码
                      "5": mission_id,    # 任务编号
                      "6": mission_status,# 0未执行 1正在执行 2执行成功 3执行失败
                      "7": [0, 0, 0, 0, 0, 0, 0]    #  can连接 无can数据 xbox 连接 mqtt连接 急停按钮是否按下0：没有按下 1：按下 ; 连接充电枪客户端；生成充电模型
                      "8": x,           # x坐标, 单位米
                      "9": y,           # y坐标, 单位米
                      "10": angle,      # angle
                      "11": confidence, # 定位置信度，范围[0,1]
                      "12": obstacle,   # 有无障碍 0无障碍 1有障碍
                      "15": map_name,         # 地图名称
                      "16": {
                          "1": soc,             # 电池电量
                          "2": BatAlmLv,        # 动力电池告警级别 0:无告警 1:一级告警（最轻微）2:二级告警 3:三级告警 4:四级告警 (最严重)
                          "3": BatState         # 动力电池状态 0 （上电） 1（就绪）2 （运行）3 （错误）4 （关机） 

                          "4": ChrOFFCC2        # 非车载充电机连接确认 0：未连接；1：已连接
                          "5": ChrCommunication # 充电机是否在线    0：通信超时；1：在线
                          "6": ChrState         # 补电状态  0：未开始；1：加热；2：充电；3：退出；4：充电完成； # 待测试

                          "7": charge status    # 充电状态  0表示待机中；1表示启动充电中；
                          "8": rechargeShot status  # 充电枪状态 0表示枪已归位；1表示未连接；2表示枪已连接；    # 待测试
                          "9": chargingPile     # 充电桩状态 0表示正常；1表示有故障；
                      },
                      "17": get_sound,        # 音乐名
                      "18": get_number,       # 剩余播放次数
                  }
    
    (2) 小车发布状态话题；小车-> MQTT -> 调度
        在充电中主动上报，同时可查询
        主题：mrclient/{id}/baterry/status
        cmd:/api/v1/baterry/status/
        msgId：不需要
        payload：{
            "baterryState": {
                "voltage": 0,
                "electric": 0,
                "SOH": 0,
            },
            "rechargeShot": {
                "chargingStatus": None, # 0x00表示待机中；0x01表示启动充电中；0x02表示充电中；0x03 禁止充电
                "rechargeShotStatus": None, # 0x00表示枪已归位；0x01表示未连接；0x02表示枪已连接；
                "floorLock": None,  # 0x00表示地锁打开；0x01表示地锁锁住；
                "chargeMode": None,     # 1：恒压充电 2：恒流充电
                "chargeVlotage": None,  # 充电电压
                "chargeElectric": None, # 充电电流
                "currentChargingPower": None,   # 已充电量
                "currentChargingTime": None,    # 已充电时间
                "residualTime": None,   # 估算剩余充电时间
                "soc": None,
                "Electricityfreee": None,    # 当前电费金额
                "serviceFree": None,         # 当前服务费金额
                "serialNumbers": None,      # 本次充电流水号        # 不同用户同一个流水号
            },
        }
    
    (3) BMS和DCDC全状态查询 ：调度 -> MQTT -> 小车

        消息主题：mrrserver/{id}/cmd
        
        cmd：/api/v1/chargePileStatus/
        
        msgId：调度生成，小车回复携带该字段，值不变

        小车收到该指令并执行成功后，回复消息如下：
        
        消息主题：mrclient/{id}/return
        
        cmd：/api/v1/chargePileStatus/
        
        msgId：原msgId值
       
        payload：{   
            "rechargeShot"{
                "rechargeShotStatus": {
                    "accumulatedChargingPower": None,   # 总充电电量
                    "accumulatedRunningTime": None,     # 总运行时间
                    "accumulatedChargingTime": None,    # 总的充电时间 
                    "chargingStatus": None, # 0x00表示待机中；0x01表示启动充电中；0x02表示充电中；0x03 禁止充电
                    "rechargeShotStatus": None, # 0x00表示枪已归位；0x01表示未连接；0x02表示枪已连接；      # 待修改 0 1状态不对
                    "floorLock": None,  # 0x00表示地锁打开；0x01表示地锁锁住；
                    "requestsVlotage": None,    # BMS电压需求
                    "requestsElectric": None,   # BMS电流需求
                    "chargeMode": None,     # BMS充电模式 1：恒压充电 2：恒流充电
                    "chargeVlotage": None,  # 充电电压
                    "chargeElectric": None, # 充电电流
                    "currentChargingPower": None,   # 已充电量
                    "currentChargingTime": None,    # 已充电时间
                    "residualTime": None,   # 估算剩余充电时间
                    "soc": None,
                    "maxMonomerVlotage": None,  # 最高单体动力蓄电池电池电压
                    "maxMonomerVlotageIndex": None,     # 最高单体动力蓄电池电压所在编号
                    "minMonomerVlotage": None,      # 最低单体动力蓄电池电池电压
                    "minMonomerVlotageIndex": None, # 最低单体动力蓄电池电压所在编号
                    "maxTemperature": None,     # 最高动力蓄电池温度
                    "maxTemperaturePoint": None,     # 最高温度检测点
                    "Electricityfreee": None,    # 当前电费金额
                    "serviceFree": None,         # 当前服务费金额
                    "serialNumbers": None,      # 本次充电流水号
                    "topCharingPower": None,    # 尖时段充电电量
                    "topElectricityfreee": None,    # 尖时段电费金额
                    "topServiceFree": None,     # 尖时段服务费金额
                    "peakCharingPower": None,   # 峰时段充电电量
                    "peakElectricityfreee": None,   # 峰时段电费金额
                    "peakServiceFree": None,        # 峰时段服务费金额
                    "levelCharingPower": None,      # 平时段充电电量
                    "levelElectricityfreee": None,  # 平时段电费金额
                    "levelServiceFree": None,       # 平时段服务费金额
                    "valleyCharingPower": None,     # 谷时段充电电量
                    "valleyElectricityfreee": None, # 谷时段电费金额
                    "valleyServiceFree": None       # 谷时段服务费金额
                },
                "chargeError": {
                    "commonAlarm": None,    # 充电机故障总告警  0x00：表示未发生  0x01：发生
                    "notRectifierModule": None,    # 无可用整流模块 0x00：表示未发生  0x01：发生
                    "chargeTemperature": None,     # 充电机温度过高 0x00：表示未发生  0x01：发生
                    "ED10ConnectError": None,          # ED10通信中断 0x00：表示未发生  0x01：发生
                    "controlConnectError": None,        # 充电控制器通信中断 0x00：表示未发生  0x01：发生
                    "electricMeter": None,              # 电度表通信中断 0x00：表示未发生  0x01：发生
                    "recordingAeraOver": None,          # 记录区已满 0x00：表示未发生  0x01：发生
                    "dcFuse": None,                     # 直流熔断器故障
                    "H.T_contactor": None,              # 高压接触器故障  
                    "chargingUncontrol": None,          # 充电机输出不可控
                    "chargeDischargeCircuit": None,     # 充电机泄放回路故障
                    "outputShortCircuit": None,         # 充电机输出短路
                    "dcContactorClose": None,           # 直流接触器不能闭合
                    "dcContactorDisconnect": None       # 直流接触器不能断开
                }
            },
            "bms": {
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
                    "PosRlyStr": None,      # 总正继电器状态    0:Close；1:Open；2:Error
                    "PreRlyStr": None,      # 总负继电器状态
                    "OffChrRlyStr": None,   # 非车载充电（快充）继电器状态
                    "ChrOFFCC2": None,      # 车载充电机连接确认
                    "ChrCommunication": None,   # 充电机是否在线
                    "ChrState": None        # 充电状态
                },
                "alarm": {
                    "ALM_CELL_OV": None,            # 单体过压  0:无告警 1:1 级告警 2:2 级告警 3:3 级告警 4:4 级告警（最严重）
                    "ALM_CELL_UV": None,            # 单体欠压 
                    "ALM_CELL_OT": None,            # 电池高温
                    "ALM_CELL_UT": None,            # 电池低温
                    "ALM_BATT_OV": None,            # 整组过压
                    "ALM_BATT_UV": None,            # 整组欠压
                    "ALM_CHRG_OCS": None,           # 充电过流
                    "ALM_DSCH_OCS": None,           # 放电过流
                    "ALM_BSU_OFFLINEBSU": None,     # 离线
                    "ALM_LEAK_OC": None,            # 漏电流超限
                    "ALM_BATT_DV": None,            # 单体压差过大
                    "ALM_BATT_DT": None,            # 电池温差过大
                    "ALM_HVREL_FAIL": None,         # 电池高压异常
                    "ALM_HALL_BREAK": None,         # 霍尔断线
                    "ALM_BATT_ERROR": None,         # 电池异常保护
                    "ALM_CELL_LBK": None,           # 单体电压断线
                    "ALM_CELL_TBK": None,           # 单体温度断线
                    "ALM_PRECHRG_FAIL": None,       # 预充失败  
                    "ALM_AUX_FAIL": None,           # 继电器故障
                    "ALM_BSU_FAULTBSU": None        # 均衡故障
                }
            }
        }
    
    (4) 充电模型下发: 调度 -> MQTT -> 小车

        消息主题：mrrserver/{id}/cmd
        
        cmd：/api/v1/configBillingModel/
        
        msgId：调度生成，小车回复携带该字段，值不变,在小车MQTT登陆之后，立刻确定充电费用模型
        
        payload：{
            "0":{
                "startTime":"00:00:00",     # 起始时间
                "endTime":"10:00:00",       # 结束时间
                "electricPrice":1.00,       # 电费
                "servicePrice":0.80         # 服务费
            },
            "1":{
                "startTime":"10:00:00",     # 起始时间
                "endTime":"20:00:00",       # 结束时间
                "electricPrice":1.00,       # 电费
                "servicePrice":0.80         # 服务费
            }
            "2"{
                "startTime":"20:00:00",     # 起始时间
                "endTime":"24:00:00",       # 结束时间
                "electricPrice":1.00,       # 电费
                "servicePrice":0.80         # 服务费
            },
            "3"{
                "startTime":"00:00:00",     # 起始时间
                "endTime":"00:00:00",       # 结束时间
                "electricPrice":1.00,       # 电费
                "servicePrice":0.80         # 服务费
            }
        }
                只支持4个时间段，且时间总和是24小时

        小车收到该指令并执行成功后，回复消息如下：
        
        消息主题：mrclient/{id}/return
        
        cmd：/api/v1/tasks/
        
        msgId：原msgId值
       
        payload：{      
                  "code": "200",      # 执行成功
                  "message": "OK",
                  "data": {}
                }
                或
                {                     # 执行失败
                  "code": "400",
                  "message": "Tasks stopped",
                  "data": {}
                }

    
    (2) 任务下发: 调度 -> MQTT -> 小车

        消息主题：mrrserver/{id}/cmd
        
        cmd：/api/v1/tasks/
        
        msgId：调度生成，小车回复携带该字段，值不变
        
        payload：{
                    "location": "LM22",
                    "mission_id": "任务编号",
                    "mission_code": "操作码-楼层",
                    "map_name": "目标地点地图名称"
                }

        小车收到该指令并执行成功后，回复消息如下：
        
        消息主题：mrclient/{id}/return
        
        cmd：/api/v1/tasks/
        
        msgId：原msgId值
       
        payload：{      
                  "code": "200",      # 执行成功
                  "message": "OK",
                  "data": {}
                }
                或
                {                     # 执行失败
                  "code": "400",
                  "message": "Tasks stopped",
                  "data": {}
                }

    (3) 小车注册报告：小车-》MQTT->调度

          小车软件启动后，发布一次消息
          
          消息主题：mrclient/regist/{id}
          
          cmd：/api/v1/regist
          
          msgId：不需要
          
          payload：{"building":"南山停车场","floor":-1}
    了
    (4) 音乐播放接口，调度-> MQTT -> 小车

            调度通知小车播放指定音乐文件，在操作参数中指定音乐文件及循环播放次数

            消息主题：mrrserver/{id}/cmd

            cmd：/api/v1/tasks/

            msgId：调度生成，小车回复携带该字段，值不变

            payload：
            {
                "fileName":"a.mp3",   # 指定小车播放的文件名称
                "loop":0      # loop：指定循环次数，0表示循环播放，其他数字表示播放次数
            }
            
            小车收到后回复遵循话题 mrclient/{id}/return

      (5) 取消播放音乐接口，调度 -> MQTT -> 小车

          消息主题：mrrserver/{id}/cmd
          
          cmd：/api/v1/music/cancel/
          
          msgId：调度生成，小车回复携带该字段，值不变
          
          payload：
          {  				
          }

      
      (6) 下发充电指令，调度 -> MQTT -> 小车

            该指令通过功能码及携带参数定义
            
            由调度下发，消息主题：mrrserver/{id}/cmd
            
            cmd：/api/v1/tasks/
            
            msgId：调度生成，小车回复携带该字段，值不变
            payload：
            {
              "location": "LM22",
              "mission_id": "任务编号",
              "mission_code": "200",
              "map_name": "目标地点地图名称",
              "param":
                      {
                        "serialNumbers": "",     # 本次充电流水号, 不同次不能下发同一个流水号
                        "chargeModel": None      # 充电方式 0表示自动充电；1表示按金额充电；2表示按电量充电；3表示按时间充电；
                        "data": None             # 设定数值 自动充电方式时，本字段无效；按金额充电时，表示设定的金额（元）；
                                                    # 按电量充电时，表示设定的电量  （KWh）；按时间充电时，表示设定的时间（分钟）；
                        "balance": 0                # 账户余额
                        "electricityfreeeDiscount": 0   # 电费折扣  
                        "serviceFreeeDiscount": 0   # 服务费折扣
                      }
            }
            其中功能码200定义为充电
            param里面的target_degree定义了需要的充电量

            小车收到该指令并执行成功后，回复消息如下：
            
            消息主题：mrclient/{id}/return

            payload：
            {  		
                "data": None        #   00表示无效
                                        01表示已接收到命令，正在启动过程，回复此信息，后面充电桩无论实际启动充电成功或失败，都会上传结算帐单；
                                        02枪未连接，不可启动充电；
                                        03正在充电，不可启动充电；
                                        04表示设备故障，不可启动充电；
                                        05表示设备处于维护，不可启动充电；
                                        06表示桩被锁定，不可启动充电
                                        07表示桩处于本地操作交互流程，不能响应远程启机；
                                        当充电桩上传02~07结果时，运营平台认为本次创建充电过程不成功，但本次充电流水号及帐单信息需保存在服务器，充电桩不需上传结算帐单。	
            }
      (7) 下发查询账单指令，调度 -> MQTT -> 小车
            由调度下发，消息主题：mrrserver/{id}/cmd
            
            cmd：/api/v1/settleBilld/
            
            msgId：调度生成，小车回复携带该字段，值不变
            payload：
            {
             
            }
            消息主题：mrclient/{id}/return
            payload：
            {  	         
                "serialNumbers": None,              # 充电流水号
                "userID": None,                     # 用户卡号      
                "power": None,                      # 本次充电电量
                "electricityfreee": None,           # 电费总额
                "serviceFree": None,                # 服务费总额
                "startTime": None,                  # 充电起始时间
                "stopTime": None,                   # 充电结束时间
                "endSOC": None,                     # 结束SOC
                "stopReason": None,                 # 停机原因
                "startModel": None,                 # 启动充电方式     
                "billing":{
                    "0":{
                        "charingPower: 0,
                        "electricityfreee": 0,
                        "ServiceFree": 0
                    }
                }
            }

      (8) 下发小车状态报告间隔周期，调度-》MQTT->小车
            
            为了节约小车上网卡流量，小车默认情况下不主动发送状态报文。小车订阅到该消息，
            
            拿到当前上报小车状态的间隔周期，上位机以缓存方式发布该消息，小车软件启动后将读到最新缓存消息
            
            cmd：/api/v1/statusCfg
            
            msgId：无
            payload：
            {
              "period":100,
              "name":""
            }
            period：小车需要间隔该周期主动报告一次状态，单位毫秒
            name：描述本次下发配置的命名，仅用于说明本次配置，如：mission(任务执行期)、idle(空闲期)、offline(下线)、close_time(非营业时间)等

            小车收到该指令并执行成功后，回复消息如下：
            
            消息主题：mrclient/{id}/return

        (9) 上传本地地图到调度，小车->MQTT->调度
            
            cmd：/api/v1/get_map
            
            msgId：无
            payload：
            {
              "map_name":"1.smap",
            }

            小车收到该指令并执行成功后，回复消息如下：
            
            消息主题：mrclient/{id}/return

            cmd:/api/v1/status/
            msgId：不需要
            payload：{
                       
                    }
    


