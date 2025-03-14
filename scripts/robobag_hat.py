#!/usr/bin/env python3
import serial
import json
import serial.tools.list_ports

import rospy
from std_msgs.msg import Float64
from std_msgs.msg import Int16
from std_msgs.msg import String
import yaml
import os


class Board:
    # CONFIG_FILE_PATH = '/home/nick/trionix/src/trionix_new/config/thrusters2.yaml'
    # port_num = 0
    CONFIG_FILE_PATH = '/home/orangepi/trionix_ws/src/trionix_new/config/thrusters_bag.yaml'
    SERIAL_PORT = "/dev/ttyUSB"
    # SERIAL_PORT = '/dev/ttyS1'

    CONFIG_DATA = 0

    port_num = 0
    led = 0.0
    depth = 0.0
    temp = 0.0
    led_indicate = 1  #   0 - off, 1 - green, 2 - polis, 3 - orange_blink, 4 - blue_water
    thrusters = [0, 0]
    thrusters_n = [0, 0]
    thrusters_num = {}
    thr_list = ""


    def __init__(self):
        self.scan_for_hash()
        self.ser = serial.Serial(self.SERIAL_PORT + str(self.port_num), 115200, timeout=0.03)
        # self.depth_publisher_ = rospy.Publisher('/depth', Float64, queue_size=1)
        # self.pitch_publisher_ = rospy.Publisher('/pitch', Float64, queue_size=1)
        self.heading_publisher_ = rospy.Publisher('/heading', Float64, queue_size=1)
        self.thr_list_publisher_ = rospy.Publisher('/thrusters_list', String, queue_size=1)

        self.update_thrusters_config_data(1)

        self.cmd_timer_ = rospy.Timer(rospy.Duration(0.1), self.send_cmd)
        rospy.Subscriber('thrusters_0', Float64, self.thrusters_callback, 0)
        rospy.Subscriber('thrusters_1', Float64, self.thrusters_callback, 1)
        # rospy.Subscriber('thrusters_2', Float64, self.thrusters_callback, 2)
        # rospy.Subscriber('thrusters_3', Float64, self.thrusters_callback, 3)
        rospy.Subscriber('thrusters_update', Int16, self.update_thrusters_config_data)
        rospy.Subscriber('thrusters_data_pub', Int16, self.pub_thrusters_config_data)
        rospy.Subscriber('thrusters_save', String, self.save_thrusters_config_data)

        rospy.Subscriber('light', Float64, self.led_callback)
        rospy.Subscriber('light_signal', Int16, self.led_signal_callback)
        # rospy.Subscriber('manipulator', Float64, self.manip_callback)
        # print('Get current working directory : ', os.getcwd())

        rospy.loginfo('Cpu Board started')
        

    def __del__(self):
        self.ser.write('$3 0 0 0 0;'.encode('utf_8'))
        if self.ser:
            self.ser.close()


    def scan_for_hash(self, timeout=None):
        ports = serial.tools.list_ports.comports()
        for port, desc, hwid in sorted(ports):
            print(f"Проверка порта: {port}  Описание: {desc}")
            try:
                with serial.Serial(port, baudrate=115200, timeout=timeout) as ser: #  Настройте baudrate>
                    data = ser.read(2048)  # Читаем только один байт
                    data = data.decode("UTF-8")
                    if '#' in data and ';' in data:
                        print(f"Символ '#' найден на порту: {port[-1]}")
                        self.port_num = port[-1]
                    else:
                        print(f"Символ '#' не найден на порту: {port}")
            except serial.SerialException as e:
                print(f"Ошибка при подключении к порту {port}: {e}")
            except Exception as e:
                print(f"Произошла непредвиденная ошибка на порту {port}: {e}")


    def update_thrusters_config_data(self, msg):
        self.thrusters_num = {}

        with open(self.CONFIG_FILE_PATH, 'r') as file:
            prime_service = yaml.safe_load(file)

        listt = [key for key in prime_service["thrusters"]]

        for i in listt:
            self.thr_list += i + " " 
            thr_num = (prime_service["thrusters"][str(i)]["thruster_number"])
            thr_k = (prime_service["thrusters"][str(i)]["k_forward"])
            self.thrusters_num.update({i: {"thruster_number":int(thr_num), "k_forward":float(thr_k)}})

        self.thr_list = self.thr_list.rstrip(self.thr_list[-1])
        if self.CONFIG_DATA == 0:
            self.CONFIG_DATA = self.thrusters_num.copy()
        file.close()


    def pub_thrusters_config_data(self, msg):
        # rospy.loginfo(self.CONFIG_DATA)
        # rospy.loginfo(self.thrusters_num)
        self.thr_list_publisher_.publish(str(json.dumps(self.thrusters_num)))


    def save_thrusters_config_data(self, msg):
        data = json.loads(msg.data)
        
        with open(self.CONFIG_FILE_PATH, 'r') as file:
            prime_service = yaml.safe_load(file)

        file.close()
        
        for i in data.keys():
            prime_service["thrusters"][str(i)]["thruster_number"] = int(data[i]["thruster_number"])
            prime_service["thrusters"][str(i)]["k_forward"] = float(data[i]["k_forward"])
            prime_service["thrusters"][str(i)]["k_backward"] = float(data[i]["k_forward"])
        

        with open(self.CONFIG_FILE_PATH, 'w') as file2:
            yaml.dump(prime_service, file2)
        
        file2.close()

        self.update_thrusters_config_data(1)


    def invert_k(self, data_1, data_2):
        if int(data_1) != int(data_2):
            return -1
        else: return 1


    def read_data(self):
        resp = self.ser.readline()
        try:
            resp = resp.decode('UTF-8')
            data = resp[3:-2].split(' ')
            # p = data[1]
            # h = data[2]

            h = data[4]
            # rospy.loginfo(data[4])
            self.heading_publisher_.publish(float(h))
            # self.pitch_publisher_.publish(float(p))
            # self.depth_publisher_.publish(float(depth))
        except Exception:
            pass


    def chec_sum(self, cmd):
        data = cmd
        bitC1 = 0
        bitC2 = 0
        for i in data:
            bitC1 += i
            bitC2 += bitC1
        cmd = f'$3 {cmd[0]} {cmd[1]} {cmd[2]} {cmd[3]} {bitC1} {bitC2};'
        return cmd
    

    def send_cmd(self, _):
        try:
            # thr_1 = int(self.thrusters[self.thrusters_num["front"]["thruster_number"]]) * self.invert_k(self.thrusters_num["front"]["k_forward"], self.CONFIG_DATA["front"]["k_forward"]) 
            # thr_2 = int(self.thrusters[self.thrusters_num["back"]["thruster_number"]]) * self.invert_k(self.thrusters_num["back"]["k_forward"], self.CONFIG_DATA["back"]["k_forward"]) 
            # thr_3 = int(self.thrusters[self.thrusters_num["left"]["thruster_number"]]) * self.invert_k(self.thrusters_num["left"]["k_forward"], self.CONFIG_DATA["left"]["k_forward"])
            # thr_4 = int(self.thrusters[self.thrusters_num["right"]["thruster_number"]]) * self.invert_k(self.thrusters_num["right"]["k_forward"], self.CONFIG_DATA["right"]["k_forward"] )
            # cmd = f'$3 {thr_1} {thr_2} {thr_3} {thr_4} {self.led};'.encode('utf-8')
            
            #cmd = str('$3' + ' ' + str(self.thrusters[0]) + ' ' + str(self.thrusters[1]) + ' ' + str(self.thrusters[2]) + ' ' + str(self.thrusters[3]) + ' '  + str(self.led) + ' ' + ';').encode('utf-8')
            
            # self.thrusters_n[self.thrusters_num["front"]["thruster_number"]] = int(self.thrusters[3]) * self.invert_k(self.thrusters_num["front"]["k_forward"], self.CONFIG_DATA["front"]["k_forward"]) 
            # self.thrusters_n[self.thrusters_num["back"]["thruster_number"]] = int(self.thrusters[0]) * self.invert_k(self.thrusters_num["back"]["k_forward"], self.CONFIG_DATA["back"]["k_forward"]) 
            self.thrusters_n[self.thrusters_num["left"]["thruster_number"]] = int(self.thrusters[0]) * self.invert_k(self.thrusters_num["left"]["k_forward"], self.CONFIG_DATA["left"]["k_forward"]) 
            self.thrusters_n[self.thrusters_num["right"]["thruster_number"]] = int(self.thrusters[1]) * self.invert_k(self.thrusters_num["right"]["k_forward"], self.CONFIG_DATA["right"]["k_forward"]) 
            

            # cmd = f'$3 {self.thrusters_n[0]} {self.thrusters_n[1]} {self.led} {self.led_indicate};'.encode('utf-8')
            cmd = [self.thrusters_n[0], self.thrusters_n[1], self.led, self.led_indicate]
            cmd = self.chec_sum(cmd)
            self.ser.write(cmd.encode('utf-8'))

            #rospy.loginfo(cmd)
        except KeyError:
            pass


    def thrusters_callback(self, msg, i):
        self.thrusters[i] = int(min(max(msg.data * 100, -100), 100)) * 5
        # if i == 2:
        #     rospy.loginfo(self.thrusters[2])


    def led_callback(self, msg):
        # self.led = int(min(max(msg.data * 255, 0), 255))
        self.led = int(msg.data)
        

    def led_signal_callback(self, msg):
        self.led_indicate = int(msg.data)

    
    def manip_callback(self, msg):
        self.manip = int(msg.data)


if __name__ == '__main__':
    rospy.init_node('hat_node')

    board = Board()
    while not rospy.is_shutdown():
        board.read_data()
