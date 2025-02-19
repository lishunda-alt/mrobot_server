import pygame
from pathlib import Path
import threading
import os
from natsort import ns, natsorted
import time
import json
class Music_play(object):
    def __init__(self):
        pygame.mixer.pre_init()
        pygame.mixer.init()
        pygame.init()
        project_path = Path.cwd()
        self.mp3s_path = Path(project_path, "mp3s")
        self.name= None     # 需要播放的音乐名
        self.number = 0     
        self.loop = None    # 播放次数
        self.files = os.listdir(str(self.mp3s_path))    # file list
        self.natfiles = natsorted(self.files,alg=ns.PATH)   # file nature list
        self.play_time = 2500
        self.t = threading.Thread(target=self.play)
        self.t.start()

    def input(self, args, _time = 2500):
        if isinstance(args, str) == True:
            self.name = args
            self.loop = 1
            self.play_time = _time
        else:
            if self.name != str(args.get("fileName")) + ".wav":
                self.name = str(args.get("fileName")) + ".wav"
            self.loop = int(args.get('loop'))
            self.play_time = _time
        self.number = self.loop

    def get_sound(self):
        if not pygame.mixer.Channel(0).get_busy():
            return ""
        return self.name

    def get_number(self):
        return self.number

    def stop(self):
        self.number = 0
        pygame.mixer.Channel(0).stop()

    def play(self, _maxtime=2500):
        while True:
            try:   
                _maxtime = self.play_time
                if not pygame.mixer.Channel(0).get_busy() and self.number != 0:

                    for i in self.natfiles:
                        if i == self.name:
                            _all_path = f"{self.mp3s_path}\{i}"
                            pygame.mixer.Channel(0).play(pygame.mixer.Sound(_all_path), maxtime=_maxtime)
                            self.number -= 1
                    # print(self.natfiles, self.name, self.number)
                time.sleep(0.5)

            except Exception as e:
                print(e)


if __name__ == '__main__':
    m = Music_play()
