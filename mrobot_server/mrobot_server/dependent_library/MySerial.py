import serial
import json
import time

def hexstr2hex(hexstr):
    return hex(int(hexstr, 16))


def hex2hexstring(hexnum):
    return hex(hexnum)


def bytes2hexstr(bytes_num):
    return bytes_num.hex()


class MySerial(object):
    def __init__(self, com, baudrate, bytesize, parity, stopbits):
        try:
            self.ser = serial.Serial(com, baudrate, bytesize, parity, stopbits, timeout=0.1)
        except Exception:
            print('打开串口设备异常')

    def open_dtu_tools(self):
        self.ser.open()
        return self.ser.is_open

    def close_dtu_tools(self):
        self.ser.close()
        return self.ser.closed

    def read_size(self, size) -> bytes:
        return self.ser.read(size)

    def read_line(self):
        received_bytes_data = self.ser.readline()
        received_str_data = bytes.decode(received_bytes_data)
        return json.loads(received_str_data)

    def my_read_line(self):
        return self.ser.readline()

    def send(self, data):
        self.ser.write(data)

