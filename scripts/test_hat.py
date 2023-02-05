#!/usr/bin/env python
import serial

import rospy
from std_msgs.msg import Float64

class trionix_board:
    led = 0.0
    depth = 0.0
    temp = 0.0
    thrusters = [0, 0, 0, 0]

    def __init__(self):
        self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.03)
        self.pitch_publisher_ = rospy.Publisher('/pitch', Float64, queue_size=1)

        self.cmd_timer_ = rospy.Timer(rospy.Duration(0.1), self.send_cmd)
        for i in range(4):
            rospy.Subscriber(f'thruster_{i+1}', Float64, self.thrusters_callback, i)

        rospy.Subscriber('light', Float64, self.led_callback)
        rospy.loginfo('Cpu Board started')


    def __del__(self):
        self.ser.write(f'$3 0 0 0 0 0 0 0;'.encode('utf_8'))
        if self.ser:
            self.ser.close()


    def read_data(self):
        resp = self.ser.readline()
        rospy.loginfo(resp)
        try:
            resp = resp.decode('UTF-8')
            data = resp[3:-2].split(' ')
            p, r, depth, temp = data
            self.pitch_publisher_.publish(float(p))
        except Exception:
            pass
    
    
    def send_cmd(self, _):
        cmd = f'$3 {self.thrusters[0]} {self.thrusters[1]} {self.thrusters[2]} {self.thrusters[3]} {self.led};'.encode('utf-8')
        self.ser.write(cmd)


    def thrusters_callback(self, msg, i):
        self.thrusters[i] = int(min(max(msg.data * 100, -100), 100))
    

    def led_callback(self, msg):
       self.led = int(min(max(msg.data * 255, 0), 255))


if __name__ == '__main__':
    rospy.init_node('trionix_board')

    trionix = trionix_board()
    while not rospy.is_shutdown():
        trionix.read_data()
