# ---import system library --------------
import base64
import time
import json
import os
import threading
import time
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2
from info_msgs.msg import RobotStatus
from info_msgs.msg import GoalPoint, GoalPoints, TargetInstanceNames, GoalInstanceNames
from info_msgs.srv import  InfoMusic
from flask import Flask, request
from ament_index_python.packages import get_package_share_directory
from datetime import datetime
from concurrent.futures import ThreadPoolExecutor
from ._ext import *

config_file = os.path.join(
  get_package_share_directory('robot_server'),
  'AGVServer_Setting.json'
  )


# read config file and sleep
agvserver_setting = {}
agvserver_setting = read_setting_file(config_file)
_wait_time = int(agvserver_setting.get('wait_times'))
system_sleep(_wait_time)

# ---import dependent library --------------
from .dependent_library.MyModbus import MyModbus
from .dependent_library.log import log
from .dependent_library.mqtt import MQTT
from .zlgcan import BMS
from.asyncioServer import tcpServer

# golbal variable
_broker = agvserver_setting.get("broker")
_port = agvserver_setting.get("port")
_agv_id = agvserver_setting.get("agv_id")

tasks_code = agvserver_setting.get("tasks_code")
# -------------- global get status ------------------------------
run_status = 0
task_type = 0
target_id = ""
target_point = ""
target_map = ""
finished_path = []
now_point = ""
last_point = ""
dict_num = None
next_point = ""
mission_id = 0
mission_code = 0
mission_status = 0
mechanical_failure_status = "0"
vx = ""
vy = ""
w = ""
x = ""
y = ""
angle = ""
confidence = ""
obstacle = ""
obstacle_onoff = "1"
online_status = "1"
emergency_button = "0"
map_name = ""
floor = None
cancelCharge = False
update_charge_status = False
# 任务执行所需的标志， 标志变动退出原本的线程
exec_flag = None


# 初始化是否成功
init_flag = True
init_error_message = None

# 模块名称和实例映射的字典
mode_name_2_instance = {}

local_ip = agvserver_setting.get('local_ip')
app = Flask(__name__)
_stop_task = False
executor = ThreadPoolExecutor(9)
mr_ultrasonic = None
ultrasonic_obstacle = dict()   
update_advancedAreaList_flag = 2
play_param = {
    "fileName": "",
    "loop": 0
}
# print(agvserver_setting["power"])
bms = BMS(agvserver_setting["power"]["lowest_soc"], 
                              agvserver_setting["power"]["lowest_vlo"], 
                              agvserver_setting["power"]["max_charge_power"])

post_period = 0 #    小车发送报文周期，单位毫秒 None 不发送
lastSendStatusTime=0 # 最近发状态报文时间

http_status = {}
return_date = {}

tasks_pub_topic = "mrclient/" + str(_agv_id) 
tasks_sub_topic = "mrrserver/" + str(_agv_id) + "/cmd"

mqtt_client = None
node_server = None
xbox_status = "disconnect"
charge_stop = False
mqttLock = threading.Lock()

class mr_node(Node):
    def __init__(self, robot_ip=local_ip, robot_port=8080) -> None:
        super().__init__("msg_server")
        self.run_status_dict = {
            "warned": 1,
            "running": 2,
            "succeeded": 4,
            "canceled": 6,
            "failed": 5,
            "suspended": 3

        }
        self.pose = {
            "x": .0,
            "y": .0,
            "angle": .0,
            "confidence": .0,
            "timestamp": time.time()
        }

        self.odometry = {
            "vx": .0,
            "vy": .0,
            "timestamp": time.time()
        }
        # subscription
        self.status_sub = self.create_subscription(
            RobotStatus,
            'car_status',
            self.status_callback,
            10
        )
        self.angle = 0
        # publisher
        self.paused_pub = self.create_publisher(String, 'is_paused', 10)
        self.planner_selector_pub = self.create_publisher(String, 'planner_selector', 10)
        self.command_pub = self.create_publisher(String, 'car_command', 10)
        self.robot_ip = robot_ip
        self.robot_port = robot_port

        self.xbox_sub = self.create_subscription(String, "xbox_status", self.xbox_sub_fun, 10)

        self.goal_points_pub = self.create_publisher(GoalPoints, 'goal_points_without_angle', 10)
        self.goal_instance_names_pub = self.create_publisher(GoalInstanceNames, 'goal_instance_names', 10)
        self.site_pub = self.create_publisher(String, 'site', 10)

        self.camera_01_sub = self.create_subscription(PointCloud2, "/camera_01/depth/pointCloud", self.camera_01_hander, 10)
        self.camera_02_sub = self.create_subscription(PointCloud2, "/camera_02/depth/pointCloud", self.camera_02_hander, 10)
        self.camera_driver1_sub = self.create_subscription(PointCloud2, "/camera_01/depth/points", self.camera_driver1_hander, 10)
        self.camera_driver2_sub = self.create_subscription(PointCloud2, "/camera_02/depth/points", self.camera_driver2_hander, 10)
        self.front_camera_stop = False
        self.back_camera_stop = False
        self.last_run_status = 0
        self.camera_driver1_num = 0
        self.camera_driver2_num = 0
        self.front_stop_times = 0
        self.back_stop_times = 0
        self.music_player = self.create_client(InfoMusic, 'music_name')
   
    def camera_driver1_hander(self, msg):
        # print(time.time())
        self.camera_driver1_num = msg.width

    def camera_driver2_hander(self, msg):
        self.camera_driver2_num = msg.width

    def xbox_sub_fun(self, msg):
        global xbox_status
        xbox_status = True if msg.data == "connected" else False

    def camera_01_hander(self, msg):
        global run_status, charge_stop
        # print(str(run_status), self.odometry["vx"], self.odometry["vy"])
        if (msg.width > 5 or self.camera_driver1_num < 1000) and str(run_status) == "2" and (self.odometry["vx"] > 0.05 or abs(self.odometry["vy"]) > 0.05):
            self.paused()
            self.front_camera_stop = True
            parms = {"fileName": "16","loop": 1}
            load_play(parms)
            print("前方摄像头阻挡!")
        elif (msg.width > 5 or self.camera_driver1_num < 1000) and str(run_status) == "3" and self.front_camera_stop == True:
            self.paused()
            self.front_camera_stop = True
            parms = {"fileName": "16","loop": 1}
            load_play(parms)
            print("前方摄像头阻挡!")
        elif (msg.width <= 5 and self.camera_driver1_num > 10000) and self.front_camera_stop == True and charge_stop == False:
            self.front_stop_times += 1
            if self.front_stop_times > 5:
                self.resume()
                self.front_stop_times = 0
                self.front_camera_stop = False
                print("恢复导航")

    def camera_02_hander(self, msg):
        global run_status
        if (msg.width > 5 or self.camera_driver2_num < 1000) and str(run_status) == "2" and (self.odometry["vx"] < -0.05 or abs(self.odometry["vy"]) > 0.05):
            self.paused()
            self.back_camera_stop = True
            print("后方摄像头阻挡!")
            parms = {"fileName": "17","loop": 1}
            load_play(parms)
        elif (msg.width > 5 or self.camera_driver2_num < 1000) and str(run_status) == "3" and self.back_camera_stop == True:
            self.paused()
            self.back_camera_stop = True
            print("后方摄像头阻挡!")
            parms = {"fileName": "17","loop": 1}
            load_play(parms)
        elif (msg.width <= 5 and self.camera_driver2_num > 10000) and self.back_camera_stop == True and charge_stop == False:
            self.back_stop_times += 1
            if self.back_stop_times > 5:
                self.resume()
                self.back_camera_stop = False
                self.back_stop_times = 0
        
    def quaternion_to_yaw_degrees(self, quaternion):
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w
        yaw_radians = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))  # 计算偏航角（弧度）
        yaw_degrees = math.degrees(yaw_radians)     # 将弧度转换为度
        return yaw_degrees

    def send_request(self, points_list):
        try:
            goal_points_msg = GoalPoints()
            goal_points_msg.goal_point = []
            points_list = points_list
            for points in points_list:
                goal_pose = GoalPoint()
                goal_pose.x = points.get("x")
                goal_pose.y = points.get("y")
                goal_pose.angle = float(self.angle)
                goal_points_msg.goal_point.append(goal_pose)
            self.goal_points_pub.publish(goal_points_msg)
        except Exception as e:
            print("send_request error: {}".format(e))
    
    def send_instance_names(self, instance_names):
        try:
            goal_instance_names_msg = GoalInstanceNames()
            goal_instance_names_msg.points = []
            point_instance_names = instance_names
            for points in point_instance_names:
                target_struct = TargetInstanceNames()
                target_struct.run_model = points.get("runModel")
                target_struct.target_instance_names =  points.get("point")
                target_struct.angle = str(points.get("angle"))
                goal_instance_names_msg.points.append(target_struct)
            self.goal_instance_names_pub.publish(goal_instance_names_msg)
        except Exception as e:
            print("send_instance_names error: {}".format(e))

    def status_callback(self, msg):
        try:
            global obstacle, map_name, target_id, target_point, run_status, now_point, next_point, finished_path, obstacle_onoff
            # print(msg)
            self.pose["confidence"] = round(float(msg.confidence), 3) if msg.confidence else "0"
            self.pose["x"] = round(float(msg.x), 3)
            self.pose["y"] = round(float(msg.y), 3)
            self.pose["angle"] = math.radians(float(msg.w))
            self.odometry["vx"] = round(float(msg.vx), 3)
            self.odometry["vy"] = round(float(msg.vy), 3)
            obstacle = msg.obstacle
            map_name = msg.map_name
            if msg.target_id:
                target_id = msg.target_id
            if msg.target_point:
                point = json.loads(msg.target_point)
                target_point = [point["x"], point["y"]]
            if charge_stop:
                run_status = self.last_run_status
            else:
                self.last_run_status = run_status
                run_status = self.run_status_dict.get(msg.run_status)
            if msg.now_point:
                now_point = msg.now_point
            if msg.next_point:
                next_point = msg.next_point
            if msg.finished_path:
                finished_path = msg.finished_path
            obstacle_onoff = msg.obstacle_onoff
        except Exception as e:
            log.info("status_callback error: {}".format(e))

    def music_play(self, music_name):
        while not self.music_player.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        request = InfoMusic.Request()
        request.music_name = music_name
        print(music_name)
        future = self.music_player.call_async(request)
    
    def cancel_goal(self):
        msg = String()
        msg.data = "0x05"
        self.command_pub.publish(msg)

    def paused(self):
        msg = String()
        msg.data = "0x03"
        self.command_pub.publish(msg)
    
    def resume(self):
        msg = String()
        msg.data = "0x04"
        self.command_pub.publish(msg)

    def navfn_planner(self):    # 启动自由导航
        planner_selector_msg = String()
        planner_selector_msg.data = "GridBased"
        self.planner_selector_pub.publish(planner_selector_msg)
    
    def send_location(self, location):
        msg = String()
        msg.data = location
        self.site_pub.publish(msg)

def init_moduls():
    """
    根据配置文件里面的模型，生成模型的实例
    并且将模型名称和，模型的实例存储到字典中
    一一对应
    """
    global init_flag, init_error_message, mode_name_2_instance, agvserver_setting
    try:
        for _mode, _mode_parameter in agvserver_setting['mode'].items():
            _instance_name = str(_mode) + '_instance'
            _mode_parameter_com = _mode_parameter.get('COM')
            _mode_parameter_baudrate = _mode_parameter.get('baudrate')
            _mode_parameter_bytesize = _mode_parameter.get('bytesize')
            _mode_parameter_parity = _mode_parameter.get('parity')
            _mode_parameter_stopbits = _mode_parameter.get('stopbits')

            _instance_name = MyModbus(_mode_parameter_com, _mode_parameter_baudrate, _mode_parameter_bytesize, _mode_parameter_parity, _mode_parameter_stopbits)
            mode_name_2_instance[_mode] = _instance_name 
    except Exception as _e:
        log.error(_e)
        init_flag = False
        init_error_message = 'Failed to initialize module'

    print("mode_name_2_instance = {}".format(mode_name_2_instance))

def init_tasks():
    t1 = threading.Thread(target=baterryPub, name='baterryPub')
    t1.start()

    t2 = threading.Thread(target=exec_th_blocking, name='exec_th_blocking')
    t2.start()

    t3 = threading.Thread(target=status, name='status')
    t3.start()

    t4 = threading.Thread(target=start_app, name='start_app')
    t4.start()


@app.route('/api/v1/status/', methods=['GET', 'PUT'])
def httpStatus():
    global obstacle_onoff, online_status, lost_measures, lost_location, now_point
    global electricity, http_status
    if init_flag is False:
        return make_return(_code=500, _message=init_error_message)
    if request.method == 'GET':
        lost_measures = request.args.get('lost_measures', None)
        lost_location = request.args.get('lost_location', None)
        # print(http_status)
        return make_return(_code=200, _message='OK', _data=http_status)

    elif request.method == 'PUT':
        try:
            post_data = json.loads(request.data)
            log.info('PUT /api/v1/status/  :{}'.format(post_data))
            tmp_obstacle_onoff = post_data.get('obstacle_onoff')
            if tmp_obstacle_onoff is not None:
                obstacle_onoff = str(tmp_obstacle_onoff)
                return make_return(_code=200, _message='obstacle_onoff changed to {}'.format(tmp_obstacle_onoff))

            tmp_online_status = post_data.get('online_status')
            if tmp_online_status is not None:
                online_status = tmp_online_status
                return make_return(_code=200, _message='online_status changed to {}'.format(tmp_online_status))

            return make_return(_code=400, _message='Request parameter is empty; nothing update')
        except Exception as e:
            return make_return(_code=400, _message=str(e))
    else:
        return make_return(_code=400, _message="The request method must in [PUT, GET]")

@app.route('/api/v1/tasks/open_charge', methods=['POST'])
def open_charge():
    post_data = json.loads(request.data)
    log.info("POST /api/v1/tasks/open_charge, {}".format(post_data))
    target_degree = post_data.get("target_degree")
    tcpServer.openCharge({}, target_degree)     # 修改锁
    data = tcpServer.task["openCharge"]["data"]
    if target_degree and data == 1:     # 自动充满
        executor.submit(exec_work, {}, target_degree)
    return make_return(_code=200, _message="", _data=data)

@app.route('/api/v1/tasks/close_charge', methods=['GET'])
def close_charge():
    log.info("GET /api/v1/tasks/close_charge")
    tcpServer.closeCharge()
    return make_return(_code=200, _message="OK")

@app.route('/api/v1/chargePileStatus/', methods=['GET'])
def chargePileStatus(payload={}):
    data = dict()
    data["rechargeShot"] = tcpServer.status
    data["bms"] = bms.status
    data["car"]= http_status
    return make_return(_code=200, _message="", _data=data)
    
@app.route('/api/v1/set_params', methods=['POST'])
def set_params_route():
    data = json.loads(request.data)
    fristx, fristy = data.get("fristx", 1.7), data.get("fristy", 1.1)
    secondx, secondy = data.get("secondx", -1.7), data.get("secondy", 1.1)
    thirdx, thirdy = data.get("thirdx", -1.7), data.get("thirdy", -1.1)
    fourthx, fourthy = data.get("fourthx", 1.7), data.get("fourthy", -1.1)
    config = {
        "fristx": fristx, 
        "fristy": fristy, 
        "secondx": secondx, 
        "secondy": secondy,
        "thirdx": thirdx, 
        "thirdy": thirdy, 
        "fourthx": fourthx, 
        "fourthy": fourthy
    }
    try:
        local_filepath = os.path.join(
            get_package_share_directory('camera_detect'),
                'configuration_files', "avoid_area.json")
        with open(local_filepath, 'w', encoding='UTF-8') as f:
            json.dump(config, f, ensure_ascii=False, indent=4)
    except Exception as e:
        log.info("write data to avoid_area.json: {}".format(e))
        return make_return(_code=400, _message="")
    log.info('GET /api/v1/set_params/')   
    return make_return(_code=200, _message="OK")

@app.route('/api/v1/set_camera_params', methods=['POST'])
def camera_params():
    request_data = json.loads(request.data)
    pkg = get_package_share_directory('camera_detect')
    config_file = os.path.join(pkg, 'configuration_files', "dabai_dcw.json")
    try:
        with open(config_file, "r+", encoding="utf-8") as f:
            jfile = json.load(f)
            for item, data in request_data.items():
                if item in jfile.keys():
                    jfile[item]["offset_x"] = data.get("offset_x",jfile[item]["offset_x"])
                    jfile[item]["offset_y"] = data.get("offset_y",jfile[item]["offset_y"])
                    jfile[item]["offset_z"] = data.get("offset_z",jfile[item]["offset_y"])
                else:
                    return make_return(_code=400, _message="receive error data, {}".format(request_data))
            f.seek(0)
            json.dump(jfile, f, ensure_ascii=False, indent=4)
            f.truncate()
    except Exception as e:
        log.info("write data to dabai_dcw.json: {}".format(e))
        return make_return(_code=400, _message="")
    return make_return(_code=200, _message="OK")

@app.route('/api/v1/get_params/', methods=['GET'])
def get_params():
    try:
        pkg = get_package_share_directory('camera_detect')
        config_file = os.path.join(pkg, 'configuration_files', "dabai_dcw.json")
        params = {}
        with open(config_file, "r", encoding="utf-8") as f:
            jfile = json.load(f)
            for item, data in jfile.items():
                params[item] = {
                    "offset_x": jfile[item]["offset_x"],
                    "offset_y": jfile[item]["offset_y"],
                    "offset_z": jfile[item]["offset_z"]
                    }
        
        local_filepath = os.path.join(
            get_package_share_directory('camera_detect'),
                'configuration_files', "avoid_area.json")
        with open(local_filepath, 'r', encoding='UTF-8') as file:
            params["obstacle"] = json.load(file)
    except Exception as e:
        log.info("write data to dabai_dcw.json: {}".format(e))
        return make_return(_code=400, _message='read file error!' + str(e))
    return make_return(_code=200, _message='OK', _data=params)

@app.route('/site_line', methods=['POST'])
def site_line():
    data = json.loads(request.data)
    return_lite(data)
    log.info('GET /site_line')   
    return make_return(_code=200, _message="OK")

@app.route('/api/v1/reboot/', methods=['Get'])
def reboot():
    os.system("reboot")
    return make_return(_code=200, _message="OK")

def start_app():
    app.run(host='0.0.0.0', port='8080')

def exec_work(_parms={}, target_degree=None):
    global mission_status, cancelCharge
    currentChargingTime = tcpServer.status["rechargeShot"]["currentChargingTime"]
    while True:
        time.sleep(1)
        if target_degree:   # hand
            if target_degree == -1:
                if tcpServer.status["rechargeShot"]["soc"] > agvserver_setting["power"]["max_charge_power"]:
                    log.info("charge over!")
                    tcpServer.closeCharge()
                    return
            else:
                if target_degree < tcpServer.status["rechargeShot"]["currentChargingPower"]:
                    log.info("manual operation stop charge!")
                    tcpServer.closeCharge()
                    return
        elif _parms:
            if _parms["chargeModel"] == 1:
                if (tcpServer.status["rechargeShot"]["Electricityfreee"] + \
                tcpServer.status["rechargeShot"]["serviceFree"]) >= _parms["data"]:
                    tcpServer.closeCharge()
                    log.info("chargeModel = 1, charge over!")
            elif _parms["chargeModel"] == 2:
                if tcpServer.status["rechargeShot"]["currentChargingPower"] > _parms["data"]:
                    log.info("chargeModel = 2, charge over!")
                    tcpServer.closeCharge()
            elif _parms["chargeModel"] == 3:
                if (tcpServer.status["rechargeShot"]["currentChargingTime"] - currentChargingTime) >  _parms["data"]:
                    tcpServer.closeCharge()
                    log.info("chargeModel = 3, charge over!")
            time.sleep(2)
            if tcpServer.endBilld == True:
                mission_status = 2
                return
            elif cancelCharge:
                cancelCharge = False
                return
        else:
            log.info("exec work error!")
            return
        
        if target_degree or _parms:
            if bms.status["baterryState"]["soc"] < agvserver_setting["power"]["lowest_soc"] or \
                tcpServer.status["rechargeShot"]["soc"] >= agvserver_setting["power"]["max_charge_power"]:
                log.info("bms soc lowest or charge over!")
                tcpServer.closeCharge()
                if str(mission_code) == "200":
                    mission_status = 2
                return
        

def exec_th_blocking():
    while True:
        try:
            time.sleep(0.5)
            emergency_stop()
        except Exception as e:
            print(e)

def tasks_list(msg, topic):
    global return_date
    try:
        _topic = "mrclient/" + str(_agv_id) + "/return"
        # print("tasks_list msg:{}, type: {}".format(msg, type(msg)))
        log.info(msg)
        msg = json.loads(msg)
        id = msg.get('id')
        cmd = msg.get('cmd')
        tick = msg.get('tick')
        msgId = msg.get('msgId')
        payload = msg.get('payload')
        function_list = ["pause", "resume", "cancel", "post_tasks", "map_mapping", 
                         "maps", "change_map", "change_map_status", "location", "confirm_location", 
                         "reboot", "update_advancedAreaList", "statusCfg", "/api/v1/music/play/", 
                         "/api/v1/music/cancel/", "download_files", "settleBilld", "configBillingModel", "chargePileStatus",
                        "setOrderInfo", "orderFinished"]
        
        for i in function_list:
            if i in cmd or cmd == "/api/v1/tasks/":
                if cmd == "/api/v1/tasks/":
                    i = "post_tasks"
                elif cmd == "/api/v1/music/play/": 
                    i = "load_play"
                elif cmd == "/api/v1/music/cancel/": 
                    i = "music_cancel"
                fun = eval(i)
                res = fun(payload)
                return_date = {
                    "payload": res
                }
                # return_date['payload'] = res
                return_date['cmd'] = cmd
                return_date['tick'] = time.time()
                return_date['msgId'] = msgId
                return_date['id'] = id
                return_date=json.dumps(return_date)
                mqtt_client.publish(_topic, return_date)
                break
        time.sleep(0.1)
    except Exception as e:
        log.info("executing tasks error! {}".format(e))

def download_files(payload):
    map_file = payload.get("map_name")
    json_file = payload.get("regulation")
    ftp_host = "192.168.192.233"
    ftp_port = 8000
    ftp_username = "info"
    ftp_password = "654321"
    if map_file:
        remote_filepath = "img/" + map_file
        local_filepath = os.path.join(
                            get_package_share_directory('robot_server'),
                            'maps'
                        ) + map_file
        download_file_from_ftp(ftp_host, ftp_port, ftp_username, ftp_password, remote_filepath, local_filepath)
        image_base64 = base64.b64encode(local_filepath).decode('utf-8')
        return image_base64
    if json_file:
        remote_filepath = "points/" + json_file
        local_filepath = os.path.join(
                            get_package_share_directory('robot_server'),
                            'maps'
                        ) + json_file
        download_file_from_ftp(ftp_host, ftp_port, ftp_username, ftp_password, remote_filepath, local_filepath)
        with open(local_filepath, "r") as f:
            data = str(f.read())
            return data

def load_play(payload):
    global play_param
    if play_param['loop'] == 0:
        play_param['loop'] = payload.get("loop")
        play_param['fileName'] = payload.get("fileName")
        t = threading.Thread(target=music_play, name=music_play)
        t.start()
        log.info('GET /api/v1/music/play/')
        return make_return(_code=200, _message="OK")
    else:
        return make_return(_code=400, _message="music is playing")
        
def music_cancel(payload):
    global play_param
    play_param['loop'] = 0
    play_param['fileName'] = ""
    log.info('GET /api/v1/music/stop/')
    return make_return(_code=200, _message="OK")

def music_play():
    global play_param
    try:
        while play_param.get("loop") and play_param['fileName']:
            node_server.music_play(str(play_param['fileName']) + ".wav")
            time.sleep(5)
            if play_param["loop"] != 0:
                play_param["loop"] -= 1
        print("music play over!")
    except Exception as e:
        print(e)

def register():
    _topic = "mrclient/regist/" + str(_agv_id) 
    _payload = agvserver_setting.get("register")
    date = {
        "cmd": "/api/v1/regist",
        "id": _agv_id,
        "tick": time.time(),
        "payload": _payload
    }
    log.info(str(_agv_id)+" registe to mqtt server")
    date = json.dumps(date)
    mqtt_client.publish(_topic, date)
    time.sleep(0.1)

def return_lite(payload):
    _topic = "mrclient/return_lite/" + str(_agv_id) 
    date = {
        "cmd": "/api/v1/return_lite",
        "id": _agv_id,
        "tick": time.time(),
        "payload": payload
    }
    log.info(str(_agv_id)+" return_lite line to mqtt server")
    date = json.dumps(date)
    mqtt_client.publish(_topic, date)
    time.sleep(0.1)

def statusCfg(payload):
    global post_period, lastSendStatusTime
    post_period = payload.get("period")
    lastSendStatusTime = 0
    log.info("POST /api/v1/statusCfg, period = {}".format(post_period))
    return make_return(_code=200, _message="OK")
    
def pause(payload):
    global _stop_task
    if init_flag is False:
        return make_return(_code=500, _message=init_error_message)
   
    node_server.paused() # 暂停任务
    _stop_task = True
    log.info('GET /api/v1/tasks/pause/')
    return make_return(_code=200, _message="OK")

def cancel_charge(payload):
    tcpServer.closeCharge()
    return make_return(_code=200, _message="OK")
    
def resume(payload):
    global _stop_task
    if init_flag is False:
        return make_return(_code=500, _message=init_error_message)
    _stop_task = False
    node_server.resume()
    log.info('GET /api/v1/tasks/resume/')
    return make_return(_code=200, _message="OK")
    
def cancel(payload):
    global exec_flag, mission_id, mission_code, mission_status, _stop_task, cancelCharge
    if init_flag is False:
        return make_return(_code=500, _message=init_error_message)
    if tcpServer.registed and tcpServer.status["rechargeShot"]["chargingStatus"]:    
        tcpServer.closeCharge()
        cancelCharge = True
    mission_id = ''
    mission_code = ''
    mission_status = '0'
    exec_flag = datetime.now()
    _stop_task = False
    node_server.cancel_goal()
    log.info('GET /api/v1/tasks/cancel/')
    return make_return(_code=200, _message="OK")

def post_tasks(payload):
    """
    下发任务
    {
        "location": "22",
        "mission_id": "任务编号",
        "mission_code": "操作码-楼层",
        "map_name": "目标地点地图名称"
    }
    """
    print("\n********任务下发********")
    global mission_id, mission_code, target_map, exec_flag, mission_status, update_charge_status, tasks_code
    if init_flag is False:
        return make_return(_code=500, _message=init_error_message)
    try:
        post_data = payload
        log.info('POST /api/v1/tasks/  {}'.format(str(post_data)))
        location = post_data.get('location')
        mission_id = post_data.get('mission_id')
        mission_code = post_data.get('mission_code')
        target_map = post_data.get('map_name')
        mission_status = 1
        node_server.send_location(location)
        time.sleep(1)
        update_charge_status = True     # 可能需要加锁
        exec_flag = datetime.now()
        if str(mission_code) == "200":
            param = {
                "serialNumbers": payload["param"].get("serialNumbers"),
                "chargeModel": payload["param"].get("chargeModel"),
                "data": payload["param"].get("data"),
                "balance": payload["param"].get("balance"),
                "electricityfreeeDiscount": payload["param"].get("electricityfreeeDiscount"),
                "serviceFreeeDiscount": payload["param"].get("serviceFreeeDiscount")
            }
            if tcpServer.registed:
                tcpServer.openCharge(param)
                data = tcpServer.task["openCharge"]["data"]
                if data == 1:     # 自动充满
                    executor.submit(exec_work, param)
                    return make_return(_code=200, _message="", _data = data)
                else:
                    return make_return(_code=400, _message="", _data = data)
            else:
                return make_return(_code=401, _message="not registing")
        return make_return(_code=200, _message="")
    except Exception as e:
        log.error(e)
        return make_return(_code=400, _message='Parse json error')
    
def configBillingModel(payload):
    if tcpServer.registed:
        temp = payload.copy()
        tcpServer.configBillingModel(payload)
        data = tcpServer.task["configBillingModel"]["data"]
        if data == 0:
            config_file = os.path.join(
                get_package_share_directory('robot_server'),
                'billingModel.json')
            with open(config_file, 'w', encoding='UTF-8') as f:
                json.dump(temp, f, ensure_ascii=False, indent=4)
        return make_return(_code=200, _message='', _data=data)
    else:
        return make_return(_code=401, _message="not registing")

def settleBilld(payload):
    data = tcpServer.status["settleBilld"]
    dic = {
        "serialNumbers": data["serialNumbers"],
        "power": data["power"],
        "electricityfreee":  data["electricityfreee"],
        "serviceFree": data["serviceFree"],
        "startTime": data["startTime"],
        "stopTime": data["stopTime"],
        "endSOC": data["endSOC"],
        "stopReason": data["stopReason"],
        "timeBilling": data["timeBilling"]
    }
    return make_return(_code=200, _message='', _data=dic)
    
def setOrderInfo(payload):
    tcpServer.status["displayBilling"] = {
        "serialNumbers": (22 - len(payload.get("serialNumbers"))) * "0" + payload.get("serialNumbers"),
        "parkingNumber": payload.get("parkingNumber"),
        "carNumber": payload.get("carNumber"),
        "chargeModel": payload.get("chargeModel"),
        "chargeData": payload.get("chargeData")
    }
    return make_return(_code=200, _message="ok")

def orderFinished(payload):
    tcpServer.status["displayBilling"] = {
        "serialNumbers": "",
        "parkingNumber": "",
        "carNumber": "",
        "chargeModel": None,
        "chargeData": None
    }
    return make_return(_code=200, _message="ok")

def status():
    global post_period, http_status, lastSendStatusTime, node_server, run_status, xbox_status, mission_status, charge_stop
    register()      
    _topic = "mrclient/" + str(_agv_id) + "/status"
    f = lambda x: 0 if x == True else 1
    startTime = 0
    playTime = 0
    while True:
        try:
            _obstacle = "1" if node_server.front_camera_stop or node_server.back_camera_stop else "0"
            data_dict = {}
            data  = {
                "1": run_status,
                "2": str(target_id),
                "3": str(now_point),
                "4": str(mission_code),
                "5": str(mission_id),
                "6": str(mission_status),
                "7": [f(bms.has_usbToCan), f(bms.has_readBuf), f(xbox_status), 
                      f(mqtt_client.is_connected), int(emergency_button), f(tcpServer.connected), f(tcpServer.registed)],
                "8": node_server.pose.get("x"),
                "9": node_server.pose.get("y"),
                "10": node_server.pose.get("angle"),
                "11": node_server.pose.get("confidence"),
                "12": str(obstacle),
                "13": _obstacle,
                "15": str(map_name),
                "16": {
                    "1": bms.status["baterryState"]["soc"],
                    "2": bms.status["baterryState"]["alarm"],
                    "3": bms.status["baterryState"]["state"],
                    "4": bms.status["relay"]["ChrOFFCC2"],
                    "5": bms.status["relay"]["ChrCommunication"],
                    "6": bms.status["relay"]["ChrState"],
                    "7": tcpServer.status["rechargeShot"]["chargingStatus"],
                    "8": tcpServer.status["rechargeShot"]["rechargeShotStatus"],
                    "9": tcpServer.status["heartBeat"]["status"]
                },
                "17": play_param.get("fileName"),
                "18": play_param.get("loop"),
                "20": tcpServer.status["displayBilling"]["serialNumbers"]
            }
            http_status = data
            http_status["19"] = str(_agv_id)
            data_dict["id"] = str(_agv_id)
            data_dict["tick"] = time.time()  
            data_dict["cmd"] = "/api/v1/status/"
            data_dict["payload"] = data
            data_dict=json.dumps(data_dict)
            _time = time.time()
            if(post_period > 0 and ((_time - lastSendStatusTime) > post_period/1000)):
                lastSendStatusTime = _time
            for name, info in tcpServer.status["chargeError"].items():
                if info >= 1:
                    param = {"fileName": '19', "loop": 1}
                    load_play(param)
            for name, info in bms.status["alarm"].items():
                if info >= 1:
                    param = {"fileName": '18', "loop": 1}
                    load_play(param)
            if (time.time() - startTime) > 20:
                # print(bms.status["relay"]["ChrOFFCC2"], tcpServer.status["rechargeShot"]["rechargeShotStatus"], type(tcpServer.status["rechargeShot"]["rechargeShotStatus"]))
                if bms.status["relay"]["ChrOFFCC2"] or tcpServer.status["rechargeShot"]["rechargeShotStatus"]:
                    startTime = time.time()
                    node_server.paused() # 暂停任务
                    log.info("充电枪未在充电座上，暂停任务")
                    charge_stop = True
                    # if tcpServer.status["rechargeShot"]["rechargeShotStatus"] == 2:
                    #     param = {"fileName": '20', "loop": 1}
                    if tcpServer.status["rechargeShot"]["rechargeShotStatus"] == 1:
                        param = {"fileName": '26', "loop": 1}
                    else:
                        param = {"fileName": '21', "loop": 1}
                    load_play(param)
            if bms.status["relay"]["ChrOFFCC2"] == 0 and tcpServer.status["rechargeShot"]["rechargeShotStatus"] == 0 and charge_stop:
                time.sleep(3)
                node_server.resume()
                charge_stop = False
                log.info("恢复任务!")
            # if tcpServer.status["displayBilling"]["serialNumbers"] and str(run_status) == 2 and (time.time() - playTime) > 36:
            #    playTime = time.time()
            #    param = {"fileName": '25', "loop": 1}
            #    load_play(param)
            if post_period == 0:
                time.sleep(1)
                continue
            mqttLock.acquire()
            mqtt_client.publish(_topic, data_dict)
            mqttLock.release()
            time.sleep(0.1)
        except Exception as e:
            log.info(e)


def baterryPub():
    _topic = "mrclient/" + str(_agv_id) + "/baterry/status"
    while True:
        try:
            if tcpServer.status["rechargeShot"]["soc"] > 0 and str(mission_code) == "200":
                data_dict = dict()
                data_dict["id"] = str(_agv_id)
                data_dict["tick"] = time.time()  
                data_dict["cmd"] = "/api/v1/baterry/status/"
                data_dict["payload"] = {
                    "baterryState": {
                        "voltage": bms.status["baterryState"]["voltage"],
                        "electric": bms.status["baterryState"]["electric"],
                        "SOH": bms.status["baterryState"]["SOH"]
                    },
                    "rechargeShot":{
                        "chargingStatus": tcpServer.status["rechargeShot"]["chargingStatus"],
                        "rechargeShotStatus": tcpServer.status["rechargeShot"]["rechargeShotStatus"],
                        "floorLock": tcpServer.status["rechargeShot"]["floorLock"],
                        "chargeMode": tcpServer.status["rechargeShot"]["chargeMode"],
                        "chargeVlotage": tcpServer.status["rechargeShot"]["chargeVlotage"],
                        "chargeElectric": tcpServer.status["rechargeShot"]["chargeElectric"],
                        "currentChargingPower": tcpServer.status["rechargeShot"]["currentChargingPower"],
                        "currentChargingTime": tcpServer.status["rechargeShot"]["currentChargingTime"],
                        "residualTime": tcpServer.status["rechargeShot"]["residualTime"],
                        "soc": tcpServer.status["rechargeShot"]["soc"],
                        "Electricityfreee": tcpServer.status["rechargeShot"]["Electricityfreee"],
                        "serviceFree": tcpServer.status["rechargeShot"]["serviceFree"],
                        "serialNumbers": tcpServer.status["rechargeShot"]["serialNumbers"]
                    }
                }
                data_dict=json.dumps(data_dict)
                mqttLock.acquire()
                mqtt_client.publish(_topic, data_dict)
                mqttLock.release()
        except Exception as e:
            log.info("baterryPub error: {}".format(e))
        time.sleep(1)

def emergency_stop():
    global emergency_button
    try:
        _emergency_stop_parameter = agvserver_setting['tasks']['Emergency_stop']
        if _emergency_stop_parameter.get("on_or_off") == 'off':
            return

        _moudle_name = _emergency_stop_parameter.get('module_name')
        _slave = _emergency_stop_parameter.get('slave')
        _mode_instance = mode_name_2_instance.get(_moudle_name)

        read_data = _mode_instance.single_read(int(_slave), 1, 1)
        x1 = read_data[0] & 1
        if x1 == 1:
            param = {"fileName": '2', "loop": 1}
            load_play(param)
            emergency_button = 1
        else:
            emergency_button = 0
        x2 = (read_data[0] >> 1) & 1
        if tcpServer.rechargeShotStatus == 0:    
            if x2 == 1:
                tcpServer.status["rechargeShot"]["rechargeShotStatus"] = 0
            else:
                tcpServer.status["rechargeShot"]["rechargeShotStatus"] = 1
        else:
            tcpServer.status["rechargeShot"]["rechargeShotStatus"] = 2
        # print(x1, x2, tcpServer.status["rechargeShot"]["rechargeShotStatus"])
    except Exception as e:
        print('emergency_stop:' + str(e))



def main():
    global mqtt_client, node_server
    mqtt_client = MQTT(_broker, _port, _agv_id, tasks_sub_topic, tasks_list)
    rclpy.init()
    node_server = mr_node()
    init_moduls()
    init_tasks()
    rclpy.spin(node_server)
    node_server.destroy_node()
    rclpy.shutdown()
