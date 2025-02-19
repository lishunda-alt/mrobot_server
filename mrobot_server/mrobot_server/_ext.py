import json
import uuid
from functools import reduce
from pathlib import Path
import time
import psutil
from ftplib import FTP

def make_return(_code, _message, _data=None):
    """
    生成返回内容的函数
    """
    if _data is None:
        _data = {}
    return_dict = {
        "code": str(_code),
        "message": str(_message),
        "data": _data
    }
    # return json.dumps(return_dict)
    return return_dict


def read_setting_file(file_name):
    try:
        with open(file_name, 'r', encoding='UTF-8') as f:
            agvserver_setting = json.load(f)
            return agvserver_setting
        
    except Exception as e:
        print(e)

def get_agv_id(data):
    try:
        head = data.get("head")
        epoch = hex(data.get("epoch"))[2:]
        capacity = hex(data.get("capacity"))[2:]
        year = hex(data.get("year"))[2:]
        month = hex(data.get("month"))[2:]
        dic = psutil.net_if_addrs()
        for adapter in dic:
            snicList = dic[adapter]
            for snic in snicList:
                if snic.family.name in {'AF_LINK', 'AF_PACKET'}:
                    if("00:00:00:00:00:00" == snic.address):
                        continue
                    mac = snic.address.hex[-6:]
        return head + epoch + capacity + year + month + mac
    except Exception as e:
        print("get mac error", e)
        return head + epoch + capacity + year + month


def system_sleep(_wait_time):
    for i in range(_wait_time):
        time.sleep(1)
        print('\r 开机后等待{}秒，确保程序运行环境正常'.format((_wait_time - i)), end="")


def return_xor(a, b):
    return a ^ b


def add_checker(_some_bytes):
    _checker = reduce(return_xor, _some_bytes)
    return _some_bytes + bytes.fromhex(hex(_checker)[2:4])


def get_mac_address():
    mac = uuid.UUID(int=uuid.getnode()).hex[-12:]
    return ":".join([mac[e:e + 2] for e in range(0, 11, 2)])


def read_map_date_file():
    try:
        with open('map_date.json', 'r', encoding='UTF-8') as f:
            map_date = json.load(f)
            return map_date
    except Exception as e:
        print(e)
    pass

def read_map_file(path):
    try:
        _path = "C:/Windows/System32/maps/" + path
        with open(_path, 'r', encoding='UTF-8') as f:
            map = json.load(f)
            return map
    except Exception as e:
        print(e)
    pass

def download_file_from_ftp(host, port, username, password, remote_filepath, local_filepath):
    try:
        # 连接到FTP服务器
        ftp = FTP()
        ftp.connect(host, port)
        ftp.login(username, password)

        # 下载文件
        with open(local_filepath, 'wb') as local_file:
            ftp.retrbinary(f"RETR {remote_filepath}", local_file.write)

        print(f"文件 '{remote_filepath}' 下载成功到本地 '{local_filepath}'")
    except Exception as e:
        print(f"下载文件失败: {e}")
    finally:
        # 关闭FTP连接
        if ftp:
            ftp.quit()
