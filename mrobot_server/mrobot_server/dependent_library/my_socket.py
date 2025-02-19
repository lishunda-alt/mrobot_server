import socket
from .log import log
import time

class Socket(object):
    """
    使用__enter__和__exit__函数，管理socket的连接

    使用方法：
    with Socket(ip) as sock:
        pass
    """

    def __init__(self, ip, port) -> None:
        """
        api_type = 1[默认]为建立机器人状态控制socket, 端口为19206
        api_type = 2      为建立机器人状态查询socket, 端口为19204
        """
        self.ip = ip
        self.port = port
        """if api_type == 1:
            self.port = 19206
        elif api_type == 2:
            self.port = 19204
        elif api_type == 3:
            self.port = 19205
        elif api_type ==
        else:
            raise Exception('api_type错误，api_type必须为1或2')"""

    def __enter__(self):
        try:
            # print("建立连接", self.ip, self.port)
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            # self.sock.bind(("127.0.0.1", 9685))
            self.sock.connect((self.ip, self.port))
            self.sock.settimeout(5)
            return self.sock
        except Exception as e:
            pass
            # self.sock.close()
            # log.error('机器{}建立连接失败'.format(self.ip))

    def __exit__(self, exc_type, exc_val, traceback):
        if exc_type is not None:
            log.error('机器{}建立连接失败，问题:[{}| {}]'.format(self.ip, exc_type, exc_val))
        # print("断开连接", self.ip, self.port)
        self.sock.close()
        return True




    
