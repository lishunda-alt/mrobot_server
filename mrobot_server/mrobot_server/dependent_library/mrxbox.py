import socket
import time
import threading
import struct

class mrxbox:
    def __init__(self):
        self.left_y = 0
        self.right_x = 0
        self.A, self.B, self.X, self.Y = 0, 0, 0, 0
        self.enable = 0
        self.power = 0
        
        # t = threading.Thread(target=self.get_xbox)
        # t.start()

    def get_status(self):
        return {
            "LY": self.left_y,
            "RX": self.right_x,
            "A": self.A,
            "B": self.B,
            "X": self.X,
            "Y": self.Y,
            "enable": self.enable,
            "power": self.power
        }
    
    def get_xbox(self):
        while True:
            try:
                client1 = socket.socket()
                client1.connect(('192.168.192.170', 8888))
                client1.settimeout(2)
                while True:
                    try:
                        date = list(client1.recv(8))
                        self.left_y = date[0] if date[0] < 50 else (date[0] - 255)
                        self.right_x = date[1] if date[1] < 50 else (date[1] - 255)
                        self.enable = date[2]
                        self.A = date[3]
                        self.B = date[4]
                        self.X = date[5]
                        self.Y = date[6]
                        self.power = date[7]
                        time.sleep(0.02)
                        
                    except Exception as e:
                        client1.close()
                        break
            except Exception as e:
                print(e)

#mrxbox = mrxbox()