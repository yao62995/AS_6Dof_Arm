# --coding=utf8--
import serial
import json
import time
import sys
# from jiont_test import trans_matrix, random_config
import copy

'''
# 舵机0，0～180，初始位置90
# 舵机1，从右到左，10～180，初始95
# 舵机2，从上到下，180～30，初始150
# 舵机3，0~180,初始90
# 舵机4，0~90,初始90
# 舵机5，30～90，初始90
# 将参数类型有dict改为list，去掉write_joint的model参数，只传入绝对位置
'''


class Control(object):
    # 初始化参数，传入com口和波特率
    def __init__(self, port="com6", baudR=9600):
        self.readFlag = "*"
        self.moveFlag = "#"
        self.resetFlag = "!"
        self.speed = 60
        self.receive_data = ""
        self.initPosition = [90, 95, 90, 90, 90, 90]
        self.direction = [1, -1, 1, 1, 1]
        self.curPosition = [90, 95, 90, 90, 90, 90]
        self.RelPosition = [90, 180, 180, 180, 90]
        self.initRelPosition = [90, 180, 180, 180, 90]
        self.min_ang = [0, 10, 30, 0, 0]
        self.max_ang = [180, 180, 180, 180, 90]
        try:
            self.ser = serial.Serial(port, baudR, timeout=1)
            self.ser.flush()
            time.sleep(2)
        except Exception, e:
            print Exception, ":", e
            exit()

    # 发送串口信息
    def send_data(self, command):
        self.ser.write(command)
        time.sleep(0.1)

    # 读取串口信息
    def read_data(self):
        self.ser.flush()
        self.receive_data = self.ser.readline()
        return self.receive_data

    # 读取舵机角度
    # 默认获取所有
    # 传入list获取list对应舵机角度
    def read_joint(self, nums="*"):
        com = ""
        if nums == "*":
            com += self.readFlag + nums[0]
        else:
            for num in nums:
                number = self._toHex(str(num))
                com += self.readFlag + number
        print com
        self.send_data(com)
        angles = self.read_data()
        angle_list = angles.split(" ")
        angle_list.pop()
        print angle_list
        if len(angle_list):
            return angle_list
        else:
            print "Error,list is empty"
            return 0

    # 控制舵机移动
    # model是模式，0代表绝对位置，1代表增量位置
    # dic是各舵机和对应的角度
    def write_joint(self, l):
        l = self._position(l)
        if l is None:
            return
        com = self._get_command(l)
        if not com:
            return
        print com
        # self._get_rel_position(l)
        self.send_data(com)

    def _position(self, l):
        if len(l) > 5:
            l = l[:5]
        for i in range(0, len(l)):
            if l[i] > self.max_ang[i] or l[i] < self.min_ang[i]:
                print "{0}:{1} out of range,the range is {2} ~ {3}".format(i, l[i], self.min_ang[i], self.max_ang[i])
                return
            else:
                l[i] = int(l[i]) - self.curPosition[i]
        return l

    def _get_command(self, l):
        com = ""
        for i in range(0, len(l)):
            a = self.curPosition[i] + int(l[i])
            self.curPosition[i] = a
            number = self._toHex(i)
            angle = self._toHex(a)
            speed = self._toHex(self.speed)
            com += self.moveFlag + number + angle + speed
        return com

    def read_head_status(self, num=5):
        com = ""
        number = self._toHex(str(num))
        com = self.readFlag + number
        self.send_data(com)
        s = self.read_data()
        ang = 90 - int(s)
        return float(ang) / float(60)

    def write_head_status(self, stas, num=5):
        com = ""
        number = self._toHex(str(num))
        angle = 90 - int(float(stas) * 60)
        sangle = self._toHex(angle)
        speed = self._toHex(self.speed)
        com = self.moveFlag + number + sangle + speed
        self.curPosition[5] = angle
        self.send_data(com)

    def init(self):
        self.send_data(self.resetFlag)
        self.curPosition = copy.deepcopy(self.initPosition)
        self.RelPosition = self.initRelPosition
        return 1

    def read_initial_angles(self):
        print self.RelPosition
        return self.RelPosition

    def _get_rel_position(self, l):
        if l[1] > 0:
            l[1] = int(l[1]) - 5
        else:
            l[1] = int(l[1]) + 5
        for key in range(0, len(l)):
            if key == 0:
                pass
            elif key == 1:
                self.RelPosition[0] = self.RelPosition[0] - int(l[1])
                self.RelPosition[1] = self.RelPosition[1] + int(l[1])
            elif key == 2:
                self.RelPosition[1] = self.RelPosition[1] + int(l[2])
                self.RelPosition[2] = self.RelPosition[2] - int(l[2])
            elif key == 3:
                self.RelPosition[2] = self.RelPosition[2] + int(l[3])
            elif key == 4:
                self.RelPosition[4] = 90 + int(l[4])

    def _toHex(self, angle):
        angle = hex(int(angle))[2:]
        if len(angle) < 2:
            angle = "0" + angle
        sangle = str(angle).upper()
        return sangle

    def set_speed(self, ms):
        ms = int(ms)
        if ms > 255:
            s = ms % 255
            print "Speed can not be greater than 255,speed is set to {0}".format(s)
            self.speed = s
        elif ms < 0:
            a = abs(ms) % 255
            print "Speed can not be less than 0,speed is set to {0}".format(a)
            self.speed = a
        else:
            self.speed = ms

    def get_speed(self):
        print self.speed


def test1():
    con = Control(port="com4")
    while True:
        com = raw_input("enter mode:")
        if com == "1":
            print "get positon model"
            a = raw_input("enter command:")
            if a == "":
                con.read_joint()
            elif a == "Q":
                continue
            else:
                coml = a.split(',')
                con.read_joint(coml)
        elif com == "2":
            dic = []
            print "move mode"
            c = raw_input("enter command:")
            com_l = c.split(",")
            for i in range(0, len(com_l)):
                dic.append(int(com_l[i]))
            con.write_joint(dic)
            print con.curPosition
        elif com == "3":
            print "head model"
            b = raw_input("get or write:")
            if b == "1":
                print con.read_head_status()
            else:
                c = raw_input("Input 0~1:")
                con.write_head_status(c)
        elif com == "4":
            print "init model"
            con.init()
        elif com == "5":
            con.read_initial_angles()
        elif com == "6":
            s = raw_input("Input speed:")
            con.set_speed(s)


def test2():
    con = Control('com6')
    dic =dict()
    while True:
        pos = [0, 0, 0, 0, 0]
        config = raw_input("enter a config:").strip()
        if not config:
            con.init()
            continue
        if config == 'o':
            con.write_head_status(1)
            continue
        if config == 'c':
            con.write_head_status(0.3)
            continue
        if config == 'g':
            config = [0, -20, -70, -65]
        else:
            config = config.split(',')
        for i in range(len(config)):
            pos[i] = int(config[i])
        for i in range(0, len(pos)):
            dic[i] = pos[i]*con.direction[i] + con.initPosition[i]
        print pos, trans_matrix(pos)[:3, -1], con.initPosition
        print dic
        con.write_head_status(1)
        con.write_joint(0, dic)
        con.write_head_status(0.2)
        con.init()
        time.sleep(1)
        tar_pos = random_config()
        dic = dict()
        for i in range(0, len(tar_pos)):
            dic[i] = tar_pos[i]*con.direction[i] + con.initPosition[i]
        con.write_joint(0, dic)
        con.write_head_status(1)
        con.init()
        dic = dict()
    pass

if __name__ == '__main__':
    test1()
