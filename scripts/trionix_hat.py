#!/usr/bin/env python3
import serial

import rospy
from std_msgs.msg import Float64


class Board:
    # orientation = [0.0, 0.0, 0.0]  # y p r
    # depth = 0.0
    # temp = 0.0
    # thrusters = [0, 0, 0, 0, 0, 0]
    # led = 0
    # servo = 0
    # claw = 0
    # roll_shift = 60.1
    # pitch_shift = -7.8

    led = 0.0
    depth = 0.0
    temp = 0.0
    thrusters = [0, 0, 0, 0]

    def __init__(self):
        self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.03)
        self.depth_publisher_ = rospy.Publisher('/depth', Float64, queue_size=1)
        self.pitch_publisher_ = rospy.Publisher('/pitch', Float64, queue_size=1)

        self.cmd_timer_ = rospy.Timer(rospy.Duration(0.1), self.send_cmd)
        rospy.Subscriber('thrusters_0', Float64, self.thrusters_callback, 0)
        rospy.Subscriber('thrusters_1', Float64, self.thrusters_callback, 1)
        rospy.Subscriber('thrusters_2', Float64, self.thrusters_callback, 2)
        rospy.Subscriber('thrusters_3', Float64, self.thrusters_callback, 3)
        
        rospy.Subscriber('light', Float64, self.led_callback)

        rospy.loginfo('Cpu Board started')

    def __del__(self):
        self.ser.write('$3 0 0 0 0 0;'.encode('utf_8'))
        if self.ser:
            self.ser.close()


    def read_data(self):
        resp = self.ser.readline()
        # rospy.loginfo(resp)
        try:
            resp = resp.decode('UTF-8')
            data = resp[3:-2].split(' ')
            # rospy.loginfo("Get" + str(data))
            # r = data[0]
            p = data[1]
            depth = data[3]
            self.pitch_publisher_.publish(float(p))
            self.depth_publisher_.publish(float(depth))
        except Exception:
            pass
    

    def send_cmd(self, _):
        cmd = f'$3 {self.thrusters[0]} {self.thrusters[1]} {self.thrusters[2]} {self.thrusters[3]} {self.led};'.encode('utf-8')
        #cmd = str('$3' + ' ' + str(self.thrusters[0]) + ' ' + str(self.thrusters[1]) + ' ' + str(self.thrusters[2]) + ' ' + str(self.thrusters[3]) + ' '  + str(self.led) + ' ' + ';').encode('utf-8')
        self.ser.write(cmd)
        # rospy.loginfo("Send:" + str(cmd))

    def thrusters_callback(self, msg, i):
        self.thrusters[i] = int(min(max(msg.data * 100, -100), 100))

    def led_callback(self, msg):
        # self.led = int(min(max(msg.data * 255, 0), 255))
        self.led = int(msg.data)



if __name__ == '__main__':
    rospy.init_node('hat_node')

    board = Board()
    while not rospy.is_shutdown():
        board.read_data()
