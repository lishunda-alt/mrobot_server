import serial
import requests
import modbus_tk.defines as cst
from modbus_tk import modbus_rtu

"""
写寄存器
GET /api/v1/tasks/pause/
GET /api/v1/tasks/resume/
GET /api/v1/tasks/cancel/
× GET /api/v1/map_mapping/
× GET /api/v1/maps/?map_name=001.smap 
GET /api/v1/confirm_location/
GET /api/v1/reboot/?key=INFO-ROBOT
POST /api/v1/tasks/
PUT /api/v1/status/
PUT /api/v1/location/

读寄存器
GET /api/v1/status/

线圈寄存器：实际上就可以类比为开关量，每个bit都对应一个信号的开关状态。
离散输入寄存器：离散输入寄存器就相当于线圈寄存器的只读模式，他也是每个bit表示一个开关量
保持寄存器：这个寄存器的单位不再是bit而是两个byte，也就是可以存放具体的数据量的，并且是可读写的
输入寄存器：只剩下这最后一个了，这个和保持寄存器类似，但是也是只支持读而不能写
"""


def fun_float2(num):
    if num < 0:
        return 1, abs(num)
    else:
        return 0, abs(num)


fun_int = lambda s: 0 if (s == 'None' or s is None) else int(s)
fun_float = lambda s: 0 if (s == 'None' or s is None) else int(float(s) * 1000)
fun_LM = lambda s: int(s.strip('LM')) if s.startswith('LM') else fun_int(s)


class MyPDT(object):
    def __init__(self, com=None, baudrate=None, bytesize=None, parity=None, stopbits=None):
        """
        block0 是模拟输入寄存器， 一个寄存器占两个bytes, 只能读
        block1 是保持寄存器可以读写
        """
        self.ser = serial.Serial(com, baudrate, bytesize, parity, stopbits)
        self.server = modbus_rtu.RtuServer(self.ser)
        self.server.start()
        self.slave_1 = self.server.add_slave(17)
        _zero_data = [0] * 100
        # add_block(name, block_type, starting_address, length)
        self.slave_1.add_block('block0', cst.HOLDING_REGISTERS, 0, 100)
        self.slave_1.set_values('block0', 0, _zero_data)

        self.slave_1.add_block('block1', cst.HOLDING_REGISTERS, 100, 100)
        self.slave_1.set_values('block1', 100, _zero_data)

        self.agv_status = dict()

    def agv_status_to_block0(self):
        _status = self.agv_status

        if self.agv_status:
            self.slave_1.set_values('block0', 0, fun_int(_status.get('run_status')))
            self.slave_1.set_values('block0', 1, fun_int(_status.get('task_type')))
            self.slave_1.set_values('block0', 2, int(fun_LM(_status.get('target_id'))))

            if _status.get('target_point') is None or _status.get('target_point') == 'None':
                self.slave_1.set_values('block0', 3, 0)
                self.slave_1.set_values('block0', 4, 0)
                self.slave_1.set_values('block0', 5, 0)
            else:
                self.slave_1.set_values('block0', 3, int(float(_status.get('target_point')[0]) * 1000))
                self.slave_1.set_values('block0', 4, int(float(_status.get('target_point')[1]) * 1000))
                self.slave_1.set_values('block0', 5, int(float(_status.get('target_point')[2]) * 1000))

            # target_map
            if _status.get('target_map') is None or _status.get('target_map') == 'None':
                self.slave_1.set_values('block0', 6, 0)
                self.slave_1.set_values('block0', 7, 0)
                self.slave_1.set_values('block0', 8, 0)
                self.slave_1.set_values('block0', 9, 0)
            else:
                self.slave_1.set_values('block0', 6, ord(_status.get('target_map')[0]))
                self.slave_1.set_values('block0', 7, ord(_status.get('target_map')[1]))
                self.slave_1.set_values('block0', 8, ord(_status.get('target_map')[2]))
                self.slave_1.set_values('block0', 9, ord(_status.get('target_map')[3]))

            # finished_path
            if not _status.get('finished_path'):
                self.slave_1.set_values('block0', 10, 0)
                self.slave_1.set_values('block0', 11, 0)
                self.slave_1.set_values('block0', 12, 0)
            else:
                if 1 <= len(_status.get('finished_path')):
                    self.slave_1.set_values('block0', 10, fun_LM(_status.get('finished_path')[0]))
                if 2 <= len(_status.get('finished_path')):
                    self.slave_1.set_values('block0', 11, fun_LM(_status.get('finished_path')[1]))
                if 3 <= len(_status.get('finished_path')):
                    self.slave_1.set_values('block0', 12, fun_LM(_status.get('finished_path')[2]))

            self.slave_1.set_values('block0', 13, fun_LM(_status.get('now_point')))
            self.slave_1.set_values('block0', 14, fun_LM(_status.get('next_point')))
            self.slave_1.set_values('block0', 15, fun_int(_status.get('mission_code')))
            self.slave_1.set_values('block0', 16, fun_int(_status.get('mission_id')))
            self.slave_1.set_values('block0', 17, fun_int(_status.get('mission_status')))
            self.slave_1.set_values('block0', 18, fun_int(_status.get('mechanical_failure_status')))

            bit, num = fun_float2(fun_float(_status.get('vx')))
            self.slave_1.set_values('block0', 19, bit)
            self.slave_1.set_values('block0', 20, num)

            bit, num = fun_float2(fun_float(_status.get('vy')))
            self.slave_1.set_values('block0', 21, bit)
            self.slave_1.set_values('block0', 22, num)

            bit, num = fun_float2(fun_float(_status.get('w')))
            self.slave_1.set_values('block0', 23, bit)
            self.slave_1.set_values('block0', 24, num)

            bit, num = fun_float2(fun_float(_status.get('x')))
            self.slave_1.set_values('block0', 25, bit)
            self.slave_1.set_values('block0', 26, num)

            bit, num = fun_float2(fun_float(_status.get('y')))
            self.slave_1.set_values('block0', 27, bit)
            self.slave_1.set_values('block0', 28, num)

            bit, num = fun_float2(fun_float(_status.get('angle')))
            self.slave_1.set_values('block0', 29, bit)
            self.slave_1.set_values('block0', 30, num)

            self.slave_1.set_values('block0', 31, fun_float(_status.get('confidence')))
            self.slave_1.set_values('block0', 32, fun_int(_status.get('obstacle')))
            self.slave_1.set_values('block0', 33, fun_int(_status.get('obstacle_onoff')))
            self.slave_1.set_values('block0', 34, fun_int(_status.get('online_status')))
            self.slave_1.set_values('block0', 35, fun_int(_status.get('emergency_button')))

            # map_name
            if _status.get('map_name') is None:
                self.slave_1.set_values('block0', 36, 0)
                self.slave_1.set_values('block0', 37, 0)
                self.slave_1.set_values('block0', 38, 0)
                self.slave_1.set_values('block0', 39, 0)
            else:
                self.slave_1.set_values('block0', 36, ord(_status.get('map_name')[0]))
                self.slave_1.set_values('block0', 37, ord(_status.get('map_name')[1]))
                self.slave_1.set_values('block0', 38, ord(_status.get('map_name')[2]))
                self.slave_1.set_values('block0', 39, ord(_status.get('map_name')[3]))

            self.slave_1.set_values('block0', 40, fun_int(_status.get('floor')))
            self.slave_1.set_values('block0', 41, fun_float(_status.get('voltage')))
            self.slave_1.set_values('block0', 42, fun_float(_status.get('mah')))
            self.slave_1.set_values('block0', 43, fun_int(_status.get('power')))

    def set_status(self, _status_data_dict):
        self.agv_status = _status_data_dict
        self.agv_status_to_block0()
        self.block_to_requests()

    def block_to_requests(self):
        """
        GET /api/v1/tasks/pause/ 100
        GET /api/v1/tasks/resume/ 101
        GET /api/v1/tasks/cancel/ 102
        PUT /api/v1/location/
        x 103 y 104 angle 105
        GET /api/v1/confirm_location/ 106
        GET /api/v1/reboot/?key=INFO-ROBOT 107
        POST /api/v1/tasks/
        location 108    mission_id 109      mission_code 110        map_name 111 112 113 114
        PUT /api/v1/status/   change_flag 115  obstacle_onoff 116      online_status 117
        """
        data = self.slave_1.get_values('block1', 100, 100)
        _pause = data[0]
        if _pause != 0:
            requests.get('http://127.0.0.1:8080/api/v1/tasks/pause/')
        _resume = data[1]
        if _resume != 0:
            requests.get('http://127.0.0.1:8080/api/v1/tasks/resume/')
        _cancel = data[2]
        if _cancel != 0:
            requests.get('http://127.0.0.1:8080/api/v1/tasks/cancel/')
        _location_x = data[3] / 1000
        _loaction_y = data[4] / 1000
        _location_angle = data[5] / 1000
        if _location_x != 0 or _loaction_y != 0 or _location_angle != 0:
            requests.put('http://127.0.0.1/api/v1/location/',
                         json={'x': _location_x, 'y': _loaction_y, 'angle': _location_angle})
        _confirm_location = data[6]
        if _confirm_location != 0:
            requests.get('http://127.0.0.1:8080/api/v1/confirm_location/')
        _robot = data[7]
        if _robot != 0:
            requests.get('http://127.0.0.1:8080/api/v1/reboot/?key=INFO-ROBOT')
        _tasks_location = data[8]
        _tasks_mission_id = data[9]
        if data[10] == 0:
            _tasks_mission_code = None
        else:
            _tasks_mission_code = data[10]
        if data[11] == 0:
            _tasks_map_name = None
        else:
            _tasks_map_name = chr(data[11]) + chr(data[12]) + chr(data[12]) + chr(data[14]) + '.smap'
        if _tasks_location != 0:
            requests.post('http://127.0.0.1:8080/api/v1/tasks/',
                          json={'location': 'LM'+str(_tasks_location), 'mission_id': _tasks_mission_id,
                                'mission_code': _tasks_mission_code, 'map_name': _tasks_map_name})
        _change_flag = data[15]
        if _change_flag != 0:
            requests.put('http://127.0.0.1:8080/api/v1/status/',
                         json={'obstacle_onoff': data[16], 'online_status': data[17]})
        self.slave_1.set_values('block1', 100, [0] * 100)
