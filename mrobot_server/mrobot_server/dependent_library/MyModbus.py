import serial
from modbus_tk import modbus_rtu
import modbus_tk.defines as cst


class MyModbus(object):
    def __init__(self, com=None, baudrate=None, bytesize=None, parity=None, stopbits=None):
        try:
            self.ser = serial.Serial(com, baudrate, bytesize, parity, stopbits)
            self.master = modbus_rtu.RtuMaster(self.ser)
            self.master.set_timeout(5.0)
            self.master.set_verbose(True)
        except Exception as e:
            print(e)

    def write(self, slave, star_address, output_value):
        try:
            print("\n********串口写命令********")
            self.master.execute(
                slave, cst.WRITE_MULTIPLE_REGISTERS, star_address, output_value=output_value)
        except Exception as e:
            print(e)

    def single_write(self, slave, star_address, output_value):
        try:
            self.master.execute(
                slave, cst.WRITE_SINGLE_REGISTER, star_address, output_value=output_value)
        except Exception as e:
            print(e)

    def single_read(self, slave, star_address, quantity):
        try:
            return self.master.execute(slave, cst.READ_HOLDING_REGISTERS, star_address, quantity)
        except Exception as e:
            print(e)
        
    def write_single_register(self, slave, star_address, output_value):
        try:
            print((slave, star_address, output_value))
            self.master.execute(
                slave, cst.WRITE_SINGLE_REGISTER, star_address, output_value=output_value)
        except Exception as e:
            print(e)

    def my_read_line(self):
        return self.ser.readline()


if __name__ == '__main__':
    '''
    ser = serial.Serial("com1", 9600, 8, 'N', 1)
    pow = MyModbus(ser=ser)
    read_data = pow.single_read(
            int(240), int(1), 8)
    print(read_data)
    '''

    try:
        # 设定串口为从站
        INFO_IO = modbus_rtu.RtuMaster(serial.Serial(port="com1",
                                                     baudrate=9600, bytesize=8, parity='N', stopbits=1))
        INFO_IO.set_timeout(0.1)
        INFO_IO.set_verbose(True)
    except Exception as exc:
        print(str(exc))
    read = INFO_IO.execute(240, cst.READ_HOLDING_REGISTERS, 1, 1)
    print(read)
