from .msg_process import MsgProcess
from .my_socket import Socket
from .log import log
from functools import wraps
import json
import traceback


class Robot(object):
    def __init__(self) -> None:
        pass

    def log_decorator(func):
        @wraps(func)
        def record(self, *args, **kwargs):
            try:
                return func(self, *args, **kwargs)
            except Exception as e:
                log.error(f"{func.__name__} is error,here are details:{traceback.format_exc()}")

        return record

    @log_decorator
    def goloc_by_robotip(self, robot_ip, location_name, angle=None):
        """
        goloc_by_robotip：启动站点任务，AGV运行到指定目标点
        输入参数:
            robot_ip目标机器人ip
            location_name指定站点目标名称
            angle:angle为到点朝向角度，缺省则会使用站点上的数据
        返回数据：
            {
                ret_code : 'API错误码',
                err_msg, '错误信息'
            }
        """
        json_msg = {"id": location_name}
        if angle:
            json_msg.update["angle"] = angle

        with Socket(robot_ip, port=19206) as sock:
            msg = MsgProcess.packMsg(req_id=1, msg_type=3051, msg=json_msg)
            sock.send(msg)

            recived_data = self.recive_data(sock, robot_ip)
            # log.info('机器[{}] goloc_by_robotip,收到返回json如下:{}'.format(robot_ip, recived_data))
            return recived_data

    @log_decorator
    def cancel_goloc(self, robot_ip):
        """
        cancel_goloc:取消站点任务
        输入
            robot_ip 目标机器人ip
        返回数据：
            {
                ret_code : 'API错误码',
                err_msg, '错误信息'
            }
        """
        with Socket(robot_ip, port=19206) as sock:
            msg = MsgProcess.packMsg(req_id=1, msg_type=3003)
            sock.send(msg)

            recived_data = self.recive_data(sock, robot_ip)
            # log.info('机器[{}] cancel_goloc,收到返回json如下:{}'.format(robot_ip, recived_data))
            return recived_data

    @log_decorator
    def pause_goloc(self, robot_ip):
        """
        pause_goloc: 暂停站点任务
        输入：
            robot_ip:目标机器人ip
        返回数据：
            {
                ret_code : 'API错误码',
                err_msg, '错误信息'
            }
        """
        with Socket(robot_ip, port=19206) as sock:
            msg = MsgProcess.packMsg(req_id=1, msg_type=3001)
            sock.send(msg)

            recived_data = self.recive_data(sock, robot_ip)
            # log.info('机器[{}] pause_goloc,收到返回json如下:{}'.format(robot_ip, recived_data))
            return recived_data

    @log_decorator
    def resume_goloc(self, robot_ip):
        """
        resume_goloc: 恢复站点任务
        输入：
            robot_ip:目标机器人ip
        返回数据：
            {
                ret_code : 'API错误码',
                err_msg, '错误信息'
            }
        """
        with Socket(robot_ip, port=19206) as sock:
            msg = MsgProcess.packMsg(req_id=1, msg_type=3002)
            sock.send(msg)

            recived_data = self.recive_data(sock, robot_ip)
            # log.info('机器[{}] resume_goloc,收到返回json如下:{}'.format(robot_ip, recived_data))
            return recived_data

    @log_decorator
    def exec_task_list(self, robot_ip, task_list):
        """
        exec_task:执行任务组（链）
        输入：
            robot_ip: 目标机器人ip
            task_list：站点数组
        输出：
            {
                ret_code : 'API错误码',
                err_msg, '错误信息'
            }
        """
        pass

    @log_decorator
    def cancel_task_list(self, robot_ip):
        """
        cancel_task_list:取消任务组（链）
        输入：
            robot_ip: 目标机器人ip
        返回数据：
            {
                ret_code : 'API错误码',
                err_msg, '错误信息'
            }
        """
        with Socket(robot_ip, port=19206) as sock:
            msg = MsgProcess.packMsg(req_id=1, msg_type=3104)
            sock.send(msg)

            recived_data = self.recive_data(sock, robot_ip)
            # log.info('机器[{}] cancel_task_list,收到返回json如下:{}'.format(robot_ip, recived_data))
            return recived_data

    @log_decorator
    def pause_task_list(self, robot_ip):
        """
        pause_task_list:暂停任务组（链）
        输入：
            robot_ip: 目标机器人ip
        返回数据：
            {
                ret_code : 'API错误码',
                err_msg, '错误信息'
            }
        """
        with Socket(robot_ip, port=19206) as sock:
            msg = MsgProcess.packMsg(req_id=1, msg_type=3102)
            sock.send(msg)

            recived_data = self.recive_data(sock, robot_ip)
            # log.info('机器[{}] pause_task_list,收到返回json如下:{}'.format(robot_ip, recived_data))
            return recived_data

    @log_decorator
    def resume_task_list(self, robot_ip):
        """
        resume_task_list:恢复任务组（链）
        输入：
            robot_ip: 目标机器人ip
        返回数据：
            {
                ret_code : 'API错误码',
                err_msg, '错误信息'
            }
        """

        with Socket(robot_ip, port=19206) as sock:
            msg = MsgProcess.packMsg(req_id=1, msg_type=3103)
            sock.send(msg)

            recived_data = self.recive_data(sock, robot_ip)
            # log.info('机器[{}] resume_task_list,收到返回json如下:{}'.format(robot_ip, recived_data))
            return recived_data

    # noinspection PyArgumentList
    @log_decorator
    def query_AGV_location(self, robot_ip):
        """
        query_AGV_location:查询AGV位置
        输入：
            robot_ip: 目标机器人ip
        返回：
            位置信息{
                        x : 'x坐标, 单位米',
                        y : 'y坐标' 单位米,
                        angle : 'angle坐标，单位rad',
                        confidence, '定位置信度，范围[0,1]',
                        ret_code, 'API错误码'，
                        err_msg, '错误信息'
                    }
        """
        with Socket(robot_ip, port=19204) as sock:
            msg = MsgProcess.packMsg(req_id=1, msg_type=1004)
            sock.send(msg)
            recived_data = self.recive_data(sock, robot_ip)
            # log.info('机器[{}] query_AGV_location,收到返回json如下:{}'.format(robot_ip, recived_data))
            return recived_data

    @log_decorator
    def readzhangai(self, robot_ip):
        """
        readzhangai 查询AGV是否有激光障碍阻挡
        输入：
                robot_ip: 目标机器人ip
        返回：
                1：被阻挡
                0：未被阻挡
        """
        with Socket(robot_ip, port=19204) as sock:
            msg = MsgProcess.packMsg(req_id=1, msg_type=1006)
            sock.send(msg)
            recived_data = self.recive_data(sock, robot_ip)
            if "block_reason" in recived_data:
                t = recived_data["block_reason"]
                if t == 1:
                    return True
                else:
                    return False
            else:
                return False

    @log_decorator
    def query_AGV_taskStatus(self, robot_ip):
        """
        query_AGV_taskStatus:查询AGV当前任务状态
        输入：
             robot_ip: 目标机器人ip
        输出：
            {
                task_status:'0=None/1=WAITING/2=RUNING/3=SUSPENDED/
                        4=COMPLETED/5=FAILED/6=CANCELED', |
                task_type:,'1=没有任务/2=自由导航到站点/3=固定路径导航到站点/
                        4=巡检/100=其他' |
                target_id:'当前任务要去的站点，仅当task_type为2或3时有效', |
                target_point:'当前任务要去的站点世界坐标[x,y,r]', |
                finished_path:'已经经过的站点，形式为list', |
                unfinished_path:'还没去的站点，形式为list', |
                ret_code:'API错误码', |
                err_msg:'错误信息'
            }

        """
        with Socket(robot_ip, port=19204) as sock:
            msg = MsgProcess.packMsg(req_id=1, msg_type=1020)
            sock.send(msg)
            recived_data = self.recive_data(sock, robot_ip)
            # log.info('机器[{}] query_AGV_taskStatus,收到返回json如下:{}'.format(robot_ip, recived_data))
            return (recived_data)

    @log_decorator
    def query_AGV_speed(self, robot_ip):
        """
        query_AGV_speed
        输入：
             robot_ip: 目标机器人ip
        输出：
            {
                'vx': 前进后退速度,
                'vy': 横向速度，
                'w'：角速度
            }
        """
        with Socket(robot_ip, port=19204) as sock:
            msg = MsgProcess.packMsg(req_id=1, msg_type=1005)
            sock.send(msg)
            recived_data = self.recive_data(sock, robot_ip)
            return recived_data

    @log_decorator
    def control_mode(self, robot_ip, mode):
        """
        control_mode:切换手动和自动模式
        输入：
            robot_ip：目标机器人ip
            mode : 0手动  1自动
        返回：

        """
        with Socket(robot_ip, port=19207) as sock:
            msg = MsgProcess.packMsg(req_id=1, msg_type=4000, msg={"mode": int(mode)})
            # log.info(msg)
            sock.send(msg)
            recived_data = self.recive_data(sock, robot_ip)
            # log.info('机器[{}] control_mode,收到返回json如下:{}'.format(robot_ip, recived_data))
            return recived_data

    @log_decorator
    def move(self, robot_ip, v, w):
        """
        move: 手动操控移动
        输入：
            robot_ip：目标机器人ip
            v: 前进后退速度 m/s
            w: 角速度 弧度每秒
        返回：
        """
        with Socket(robot_ip, port=19205) as sock:
            msg = MsgProcess.packMsg(req_id=1, msg_type=2010, msg={"vx": float(v), "w": float(w)})
            log.info(msg)
            sock.send(msg)
            # time.sleep(1)
            recived_data = self.recive_data(sock, robot_ip)
            # log.info('机器[{}] move,收到返回json如下:{}'.format(robot_ip, recived_data))
            return recived_data

    @log_decorator
    def set_move_factory(self, robot_ip, _max_speed=None, _max_acc=None, _max_rot=None, _max_rot_acc=None, _block_x_dist=None, _block_y_dist=None):
        """
        #                  | x方向     右手直角坐标系
        #                  |
        # y方向            |
        # <----------------|-------------------->
        #                  |
        #                  |
        #                  |
        set_move_factory: 设置运行参数
        输入
            robot_ip：目标机器人ip
            可以缺省
            _max_speed : 最大速度 米每秒
            _max_acc：最大加速度 米每平方秒
            _max_rot: 最大角速度 度每秒
            _max_rot_acc： 最大角加速度 度没平方秒
            _block_x_dist x方向避障距离 米
            _block_y_dist y方向避障距离 米

        返回：
        """
        with Socket(robot_ip, port=19207) as sock:
            factory = {}
            if _max_speed is not None:
                factory['MaxSpeed'] = float(_max_speed)
            if _max_acc is not None:
                factory['MaxAcc'] = float(_max_acc)
            if _max_rot is not None:
                factory['MaxRot'] = float(_max_rot)
            if _max_rot_acc is not None:
                factory['MaxRotAcc'] = float(_max_rot_acc)
            if _block_x_dist is not None:
                factory['BlockXDist'] = float(_block_x_dist)
            if _block_y_dist is not None:
                factory['BlockYDist'] = float(_block_y_dist)
            msg = MsgProcess.packMsg(req_id=1, msg_type=4101, msg={"MoveFactory": factory})
            sock.send(msg)
            recived_data = self.recive_data(sock, robot_ip)
            # log.info('机器[{}] set_move_factory,收到返回json如下:{}'.format(robot_ip, recived_data))
            return recived_data

    @log_decorator
    def get_move_factory(self, robot_ip):
        """
        #                  | x方向     右手直角坐标系
        #                  |
        # y方向            |
        # <----------------|-------------------->
        #                  |
        #                  |
        #                  |
        get_move_factory: 查询运行参数
        返回
            robot_ip：目标机器人ip
            可以缺省
            _max_speed : 最大速度 米每秒
            _max_acc：最大加速度 米每平方秒
            _max_rot: 最大角速度 度每秒
            _max_rot_acc： 最大角加速度 度没平方秒
            _block_x_dist x方向避障距离 米
            _block_y_dist y方向避障距离 米
        """
        with Socket(robot_ip, port=19204) as sock:
            msg = MsgProcess.packMsg(req_id=1, msg_type=1400)
            sock.send(msg)
            recived_data = self.recive_data(sock, robot_ip)
            # log.info('机器[{}] get_move_factory,收到返回json如下:{}'.format(robot_ip, recived_data))
            return recived_data

    @log_decorator
    def reset_location(self, robot_ip, x, y, angle, home_flag:bool):
        """
        reset_location: 重定位
        输入：
            robot_ip：目标机器人ip
            x: x坐标
            y: y坐标
            angle: 角度
            home_flag: 是否是五个默认重启点之一， True/ False
                        True: x, y, angle 不生效 寻找默认重启点，进行重定位
                        False: x, y, angle 按照x y angle 重定位
        返回：
        """
        with Socket(robot_ip, port=19205) as sock:
            msg = MsgProcess.packMsg(req_id=1, msg_type=2002, msg={"x": float(x), "y": float(y), "angle": float(angle), "home": home_flag})
            # log.info(msg)
            sock.send(msg)
            # time.sleep(1)
            recived_data = self.recive_data(sock, robot_ip)
            # log.info('机器[{}] reset_location,收到返回json如下:{}'.format(robot_ip, recived_data))
            return recived_data

    @log_decorator
    def confirm_location(self, robot_ip):
        """
                confirm_location: 重定位后确认位置正确
                输入：
                    robot_ip：目标机器人ip
                返回：
                """
        with Socket(robot_ip, port=19205) as sock:
            msg = MsgProcess.packMsg(req_id=1, msg_type=2003)
            # log.info(msg)
            sock.send(msg)
            # time.sleep(1)
            recived_data = self.recive_data(sock, robot_ip)
            # log.info('机器[{}] confirm_location,收到返回json如下:{}'.format(robot_ip, recived_data))
            return recived_data

    @log_decorator
    def change_map(self, robot_ip, map_name):
        """
        reset_location: 更换地图
        输入：
            robot_ip：目标机器人ip
            map_name: 要切换的地图名称
        返回：
        """
        with Socket(robot_ip, port=19205) as sock:
            msg = MsgProcess.packMsg(req_id=1, msg_type=2022, msg={"map_name": map_name})
            # log.info(msg)
            sock.send(msg)
            # time.sleep(1)
            recived_data = self.recive_data(sock, robot_ip)
            # log.info('机器[{}] change_map,收到返回json如下:{}'.format(robot_ip, recived_data))
            return recived_data

    @log_decorator
    def get_change_map_status(self, robot_ip):
        """
        get_change_map_status: 更换地图后的更换状态
        输入：
            robot_ip：目标机器人ip
        返回：
        """
        with Socket(robot_ip, port=19204) as sock:
            msg = MsgProcess.packMsg(req_id=1, msg_type=1022)
            # log.info(msg)
            sock.send(msg)
            # time.sleep(1)
            recived_data = self.recive_data(sock, robot_ip)
            # log.info('机器[{}] change_map,收到返回json如下:{}'.format(robot_ip, recived_data))
            return recived_data

    @log_decorator
    def get_maps(self, robot_ip):
        """
        get_maps: 查询载入地图 以及存储的地图
        输入：
            robot_ip：目标机器人ip
        返回：
        """
        with Socket(robot_ip, port=19204) as sock:
            msg = MsgProcess.packMsg(req_id=1, msg_type=1300)
            sock.send(msg)
            recived_data = self.recive_data(sock, robot_ip)
            # log.info('机器[{}] get_maps,收到返回json如下:{}'.format(robot_ip, recived_data))
            return recived_data

    @log_decorator
    def arrived_location(self, robot_ip, target):
        """
        arrived_location判断目标机器人是否到达目标地点
        输入：
            robot_ip：目标机器人ip
        返回：
            True/False
        """
        return_msg = self.query_AGV_taskStatus(robot_ip)
        if return_msg is not None:
            task_status = return_msg["task_status"]

            if task_status == 4 and target == return_msg['target_id']:
                return True
            else:
                return False
        else:
            return False

    @log_decorator
    def recive_data(self, socket_fp, robot_ip):
        """
        传入socket句柄，接收返回的数据， 返回解析后的json数据，并记录日志
        输入：
            socket_fp 需要接收返回数据的socket的句柄
            robot_ip 接收的机器人的ip, 记录日志用
        返回：
            解析后的json数据区， 如果无则返回None
        """
        jsonDataLen = 0
        backReqNum = 0
        returned_msg = None
        data = socket_fp.recv(16)
        if len(data) < 16:
            log.error(
                '调度机器人{}时，报文返回头长度小于16, 报文头如下:{}'.format(robot_ip, data))
        else:
            jsonDataLen, backReqNum = MsgProcess.unpackHead(data)
            if jsonDataLen > 0:
                data = socket_fp.recv(jsonDataLen)
                returned_msg = json.loads(data)
        return returned_msg
