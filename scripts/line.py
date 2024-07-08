#!/usr/bin/env python3
# coding: utf-8

import time
from math import sin, cos
import numpy as np

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_msgs.msg import Bool

# from hector_uav_msgs.srv import EnableMotors

import cv2
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import CompressedImage

# from nav_msgs.msg import Odometry
import tf2_ros
# import tf.transformations as ttt

PLANING_HORIZON = 50

TIME_LIFTOFF = 3

RING_AVOIDANCE_TIME = 6 # [seconds]
DEFAULT_ALTITUDE = 1.5    # [meters]

V_MAX = 2.05
W_MAX = 0.35

Kp_z = 0.05

Kp_y =  0.015
Kd_y =  0.000045
Ki_y =  0.0000825

Kp_w =  0.01555
Kd_w =  0.000095
Ki_w =  0.000165


class SimpleMover():

    def __init__(self):
        rospy.init_node('line_follower', anonymous=True)

        self.cmd_vel_pub = rospy.Publisher('/teleop_command', Twist, queue_size=1)
        
        self.cv_bridge = CvBridge()

        self.start_flag = False

        rospy.on_shutdown(self.shutdown)

        rospy.Subscriber("/image_raw", Image, self.camera_callback)
        rospy.Subscriber("/line_go", Bool, self.start)
        # rospy.Subscriber("cam_2/camera/image", Image, self.camera_rings_callback)
        
        # rospy.Subscriber('/ground_truth/state', Odometry, self.obom_callback)
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        self.drone_state = [0] * 6  # position vector
        self.e_y = 0
        self.e_omega_z = 0
        self.rate = rospy.Rate(30)

        self.z = 0
        self.z_prev = 0
        self.z_des = DEFAULT_ALTITUDE
        
        self.zz = 0
        self.zz_prev = 0
        self.omega_error = 0

        self.y = 0
        self.y_error = 0
        self.y_error_prev = 0
        self.y_prev = 0

        self.image_1 = []
        # self.image_2 = [] 

        self.state = "free_flight"
        self.red_ring_detected = False
        self.blue_ring_detected = False
        self.time_start_up = 0
        self.avoidance_time = 0
        self.e_x_blue, self.e_y_blue = 0, 0

        # cv2.namedWindow('marking')

        # cv2.createTrackbar('H Lower','marking',0,179,self.nothing)
        # cv2.createTrackbar('H Higher','marking',179,179,self.nothing)
        # cv2.createTrackbar('S Lower','marking',0,255,self.nothing)
        # cv2.createTrackbar('S Higher','marking',255,255,self.nothing)
        # cv2.createTrackbar('V Lower','marking',0,255, self.nothing)
        # cv2.createTrackbar('V Higher','marking',255,255,self.nothing)


    def nothing(self, x):
        pass

    def start(self, msg):
        self.start_flag = msg


    def obom_callback(self, msg):
        """ Pose of a robot extraction"""
        # transform = self.tfBuffer.lookup_transform('world', 'base_stabilized', rospy.Time()).transform
        # x, y, z = transform.translation.x, transform.translation.y, transform.translation.z
        # quat = transform.rotation
        # r, p, y = ttt.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])

        # self.drone_state = [x, y, z, r, p, y]
        # rospy.loginfo(self.drone_state)
        self.z = msg.pose.pose.position.z
        self.y = msg.pose.pose.position.y
        self.zz = msg.pose.pose.orientation.z

    
    def ring_detector(self, image, lower, upper, color):
       color_mask = cv2.inRange(image, lower, upper)
       color_contours, _ = cv2.findContours(color_mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
       if color_contours:
           max_len_c = 0
           c = color_contours[0]
           for i in range(0, len(color_contours)):
               if len(color_contours[i]) > max_len_c:
                   c = color_contours[i]
                   max_len_c = len(color_contours[i])
           self.color_distance = max_len_c
           M = cv2.moments(c)
           if M['m00'] != 0:
               cx = int(M['m10']/M['m00'])
               cy = int(M['m01']/M['m00'])
           else:
               cx = 0
               cy = 0
           (x1,y1), color_r = cv2.minEnclosingCircle(c)
           if color_r > 10:
               image = cv2.circle(image, (cx, cy), radius=5, color=color, thickness=-1)
               cv2.drawContours(color_r, c, -1, (0,255,0), 1)
               color_r = cv2.circle(color_r, (int(x1), int(y1)), radius=int(color_r), color=color, thickness=4)       
               return image, (x1,y1), color_r[0]
       return image, (0,0), 0
        

    def camera_callback(self, msg):
        """ Computer vision stuff"""


        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))

        # self.show_image(cv_image, title='Line_0')

        # hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # hL = cv2.getTrackbarPos('H Lower','marking')
        # hH = cv2.getTrackbarPos('H Higher','marking')
        # sL = cv2.getTrackbarPos('S Lower','marking')
        # sH = cv2.getTrackbarPos('S Higher','marking')
        # vL = cv2.getTrackbarPos('V Lower','marking')
        # vH = cv2.getTrackbarPos('V Higher','marking')

        # LowerRegion = np.array([hL,sL,vL],np.uint8)
        # upperRegion = np.array([hH,sH,vH],np.uint8)

        # redObject = cv2.inRange(hsv,LowerRegion,upperRegion)

        # kernal = np.ones((1,1),"uint8")

        # red = cv2.morphologyEx(redObject,cv2.MORPH_OPEN,kernal)
        # red = cv2.dilate(red,kernal,iterations=1)

        # res1=cv2.bitwise_and(cv_image, cv_image, mask = red)


        # cv2.imshow("Masking ",res1)

        grey_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(grey_image, (10, 146, 170), (25, 255, 255))
        # mask = cv2.inRange(grey_image, (0, 97, 0), (29, 147, 255))

        # self.show_image(mask, title='Line_2')
        # cv_image = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)

        # grey_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        # _, mask = cv2.threshold(grey_image, 8, 255, cv2.THRESH_BINARY_INV)
        # cv_image = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)

        cv2.line(cv_image, (320, 0), (320, 480), (0, 320, 0), 1)
        cv2.line(cv_image, (0, 240), (640, 240), (0, 320, 0), 1)

        # "steering" conrol
        top_points = np.where(mask[10] >= 10)
        mid_points = np.where(mask[50] >= 10)
        # mid_points = np.where(mask[msg.height / 2] >= 10)
        if  (not np.isnan(np.average(top_points)) and not np.isnan(np.average(mid_points))):
            top_line_point = int(np.average(top_points))
            mid_line_point = int(np.average(mid_points))
            self.omega_error = top_line_point - mid_line_point
            
            cv2.circle(cv_image, (top_line_point, 10), 5, (0,0,255), 1)
            cv2.circle(cv_image, (mid_line_point, int(msg.height/2)), 5, (0,0,255), 1)
            cv2.line(cv_image, (mid_line_point, int(msg.height/2)), (top_line_point, 10), (0, 0, 255), 3)

        # y-offset control
        __, cy_list = np.where(mask >= 10)
        if not np.isnan(np.average(cy_list)):
            cy = int(np.average(cy_list))
            self.y_error = msg.width / 2 - cy
            
            cv2.circle(cv_image, (cy, int(msg.height/2)), 7, (0,255,0), 1)
            cv2.line(cv_image, (320, 240), (cy, int(msg.height/2)), (0, 255, 0), 3)

        self.image_1 = cv_image
        # self.show_image(cv_image)




    # def show_image(self, img, title='Camera'):
    #     cv2.imshow(title, img)
        # cv2.waitKey(3)
        
    
    def fsm_update(self):
        if self.red_ring_detected:
            self.state = "drone_up"
            # rospy.loginfo(self.avoidance_time)
        elif RING_AVOIDANCE_TIME < self.avoidance_time < RING_AVOIDANCE_TIME + 5:
            self.state = "drone_down"
            # rospy.loginfo(str(self.avoidance_time) + " --- END! --- ")
            self.time_start_up = 0
            # self.z_des = DEFAULT_ALTITUDE
        elif self.blue_ring_detected:
           self.state = "drone_blue_ring"
            # self.state = "drone_up"
        else:
        #    self.state = "free_flight"
        # elif self.state == "drone_down":
            pass
        #    self.state = "free_flight"
            # self.z_des = DEFAULT_ALTITUDE


    def spin(self):
        # self.enable_motors()
        
        # Initialisations
        altitude_prev = 0
        y_error_prev = 0
        omega_error_prev = 0
        

        # self.zz = self.drone_state[5]
        # self.z = self.drone_state[2]
        # rospy.loginfo(self.z)
        # self.y = self.drone_state[1]

        time_start = rospy.get_time()
        time_prev = time_start
        while not rospy.is_shutdown():
            if self.start_flag:
                try:
                    # Time stuff
                    t = rospy.get_time() - time_start
                    dt = t - time_prev
                    time_prev = t
                    # rospy.loginfo(self.z)

                    self.fsm_update()
                    if self.state == "drone_up":
                        self.z_des = 4
                        if self.time_start_up == 0:
                            self.time_start_up = rospy.get_time()
                    elif self.state == "drone_down":
                        # pass
                        self.z_des = DEFAULT_ALTITUDE
                    elif self.state == "drone_blue_ring":
                        self.z_des += 0.001 * self.e_y_blue 
                        rospy.loginfo(self.z_des)                    
                        # pass
                    elif self.state == "free_flight":
                        self.z_des = DEFAULT_ALTITUDE
                        # pass
                    else:
                        rospy.logerr("Error: state name error!")

                    if dt == 0:
                        dt = 1 / 30.

                    k = 10.0
                    kd = 70.0
                    # self.z_des = 2.5
                    if (self.z_des < 1.5):
                        self.z_des = 1.5

                    # u_z = k * (self.z_des - self.z) + kd * (self.z_prev - self.z)
                    # self.z_prev = self.z
                    
                    # altitude_prev = self.drone_state[2]

                    zk = 0.0015
                    zkd = 0.0008

                    u_zz = zk * (self.omega_error - self.zz) + zkd * (self.zz_prev - self.zz)
                    # # zz_prev = self.zz
                    # # u_omega_zz = 0
                    self.zz_prev= self.omega_error

                    yk = 0.0001
                    ykd = 0.0001
                    # y_des = 2.5

                    u_y = yk * (self.y_error - self.y) + ykd * (self.y_error_prev - self.y)
                    self.y_error_prev = self.y_error
                    # u_y = 0
                    # y_error_prev = self.y_error

                    twist_msg = Twist()
                    twist_msg.linear.x = 0.25
                    
                    twist_msg.linear.y = 0.0
                    # twist_msg.linear.z = u_z
                    twist_msg.angular.z = u_zz * -1
                    self.cmd_vel_pub.publish(twist_msg)
                    
                    # print(self.state, self.z_des)

                    self.avoidance_time = rospy.get_time() - self.time_start_up
                    
                    # if len(self.image_1) > 0:
                    #     self.show_image(self.image_1, title='Line')
                        # self.show_image(self.image_2, title='Rings')


                except KeyboardInterrupt:
                    break

            self.rate.sleep()

    def shutdown(self):
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)


if __name__=="__main__":
    simple_mover = SimpleMover()
    simple_mover.spin()
