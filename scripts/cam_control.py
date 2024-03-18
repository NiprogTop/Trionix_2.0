#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Int16
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from datetime import datetime as dt
import os

class ImageWriter:
    def __init__(self):
        self.cv_bridge = CvBridge()
        rospy.Subscriber("/image_raw/compressed", CompressedImage, self.cam_cal)
        rospy.Subscriber("/cam_writer_command", Int16, self.cam_comm)
        rospy.set_param("video_status", 0)
        rec_param = rospy.get_param("format")
        rospy.loginfo(rec_param)
        rec_param = rec_param.split(" ")

        ##### Video file param ######
        self.fourcc = cv2.VideoWriter_fourcc(*"MJPG")
        self.fps = int(rec_param[4])
        self.resolution = (int(rec_param[1]),int(rec_param[3]))
        self.video_path = "/video/"
        self.photo_path = "/photo/"
        self.frame_num = 0
        self.cam_comm_data = 0

        self.video_writer_status = 0

        self.filename = " "
        # dd = self.dt_get()

        # print(str(dt.now().strftime("%d%m%Y_%H%M%S")) + ".avi")
        # print(self.video_name_get())

        # self.video_writer = cv2.VideoWriter(self.video_name_get(), self.fourcc, self.fps, self.resolution)

    def __del__(self):
        if self.video_writer_status > 0:
            rospy.set_param("video_status", 0)
            self.video_writer.release()


    def show_image(self, img, title='Camera'):
        cv2.imshow(title, img)
        cv2.waitKey(3) 


    def video_name_get(self):
        name = os.path.expanduser('~') + self.video_path + str(dt.now().strftime("%d%m%Y_%H%M%S")) + ".avi"
        return(name)


    def photo_name_get(self):
        name = os.path.expanduser('~') + self.photo_path + str(dt.now().strftime("%d%m%Y_%H%M%S")) + ".jpg"
        return(name)

    def convert_ros_compressed_to_cv2(self, compressed_msg):
        np_arr = np.frombuffer(compressed_msg.data, np.uint8)
        return cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    
    def cam_comm(self, msg):
        self.cam_comm_data = msg.data
        if msg.data == 2:
            rospy.set_param("video_status", 2)
        # rospy.loginfo(msg.data)



    def cam_cal(self, msg):
        if self.video_writer_status > 0 or self.cam_comm_data == 5 or self.cam_comm_data == 1:
            # if self.cam_comm_data != 2:
            try:
                cv_image = self.convert_ros_compressed_to_cv2(msg)
            except CvBridgeError as e:
                rospy.logerr("CvBridge Error: {0}".format(e))

            if self.cam_comm_data == 5: # photo
                # rospy.loginfo("photo: ")
                self.photo_writer(cv_image)
                self.cam_comm_data = 0

            if self.cam_comm_data == 1 or self.video_writer_status == 1:
                if self.cam_comm_data != 2:
                    self.video_write(cv_image)
                    # rospy.loginfo("go -")
                # self.video_writer.release()
                # rospy.loginfo("pause")
            
            if self.cam_comm_data == 3: # end recording
                if self.video_writer_status > 0:
                    self.video_writer.release()
                self.cam_comm_data, self.video_writer_status = 0, 0
                rospy.set_param("video_status", 0)

        ######### Writing video ########
        # self.video_writer.write(cv_image)
        # self.video_write(cv_image)

        ######### Writing img #########
        # filename = 'savedImage.jpg'
        # cv2.imwrite(filename, cv_image)
        # self.photo_writer(cv_image)



    def video_write(self, frame):
        if self.video_writer_status == 0:            
            rospy.loginfo("Create")
            self.video_writer = cv2.VideoWriter(self.video_name_get(), self.fourcc, self.fps, self.resolution)
            self.video_writer_status = 1
            rospy.set_param("video_status", 1)
        self.video_writer.write(frame)
        self.frame_num += 1
        # rospy.loginfo(self.frame_num)


    def photo_writer(self, img):
        # if self.frame_num % self.fps == 0:
        filename = self.photo_name_get()
        # rospy.loginfo(filename)
        cv2.imwrite(filename, img)
        os.chmod(filename, 0o666)


    def run(self):
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('cam_record_')
    image_writer = ImageWriter()
    image_writer.run()
