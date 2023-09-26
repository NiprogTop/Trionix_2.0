# #!/usr/bin/env python3
# import serial
# import time

# import rospy
# from std_msgs.msg import Float64

# class trionix_board:
#     led = 0.0
#     depth = 0.0
#     temp = 0.0
#     thrusters = [0, 0, 0, 0]

#     def __init__(self):
#         self.ser = serial.Serial('/dev/ttyUSB0', 115200)#, timeout=0.03)
#         self.pitch_publisher_ = rospy.Publisher('/pitch', Float64, queue_size=1)

#         self.cmd_timer_ = rospy.Timer(rospy.Duration(0.1), self.send_cmd)
# #        for i in range(4):
#  #           rospy.Subscriber(f'thruster_{i+1}', Float64, self.thrusters_callback, i)
#         rospy.Subscriber('thrusters_0', Float64, self.thrusters_callback, 0)
#         rospy.Subscriber('thrusters_1', Float64, self.thrusters_callback, 1)
#         rospy.Subscriber('thrusters_2', Float64, self.thrusters_callback, 2)
#         rospy.Subscriber('thrusters_3', Float64, self.thrusters_callback, 3)

#         rospy.Subscriber('light', Float64, self.led_callback)
#         rospy.loginfo('Cpu Board started')


#     def __del__(self):
#         self.ser.write('$3 0 0 0 0 0 0 0;'.encode('utf_8'))
#         if self.ser:
#             self.ser.close()


#     def read_data(self):
#         resp = self.ser.readline()
#         rospy.loginfo(resp)
#         try:
#             resp = resp.decode('UTF-8')
#             data = resp[3:-2].split(' ')
#             p, r, depth, temp = data
#             self.pitch_publisher_.publish(float(0))
#         except Exception as er:
#             pass
            
    
    
#     def send_cmd(self, _):
#         cmd = str('$3' + ' ' + str(self.thrusters[0]) + ' ' + str(self.thrusters[1]) + ' ' + str(self.thrusters[2]) + ' ' + str(self.thrusters[3]) + ' '  + str(self.led) + ' ' + ';').encode('utf-8')
#         self.ser.write(cmd)
#         #rospy.loginfo(cmd)



#     def thrusters_callback(self, msg, i):
#         self.thrusters[i] = int(min(max(msg.data * 100, -100), 100))
    
#     def thrusters_callback_2(self, msg):
#         self.thrusters[1] = int(min(max(msg.data * 100, -100), 100))


#     def led_callback(self, msg):
#        self.led = int(min(max(msg.data * 255, 0), 255))


# if __name__ == '__main__':
#     rospy.init_node('trionix_board')
#     rospy.loginfo(22)
#     #trionix = trionix_board()
#     while not rospy.is_shutdown():
#         trionix.read_data()
#         time.sleep(1)
        
#!/usr/bin/env python3

import yaml



# print(prime_service["thrusters"]["left"])

# for i in prime_service.values():
#     for j in i.keys():
#         print(j + ":" + str(i))
# listt = []
# for i in prime_service.values():
    
#     for key in dict.keys():
#         listt.append(key)

with open('config/thrusters.yaml', 'r') as file:
    prime_service = yaml.safe_load(file)

listt = [key for key in prime_service["thrusters"]]

thrusters_num = {}
for i in listt:
    n = (prime_service["thrusters"][str(i)]["thruster_number"])
    thrusters_num.update({i: n})

file.close()

print(thrusters_num)
# for i in prime_service.values():
#     print(str(list.keys()) + ":" + str(prime_service["thrusters"]))

