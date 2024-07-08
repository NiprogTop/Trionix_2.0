#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Int16
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from datetime import datetime as dt
import os
import subprocess
import signal

class ImageWriter:
    def __init__(self):
        self.cv_bridge = CvBridge()
        # rospy.Subscriber("/image_raw/compressed", CompressedImage, self.cam_cal)
        rospy.Subscriber("/cam_writer_command", Int16, self.cam_cmd)
        rospy.set_param("video_status", 0)
        rec_param = rospy.get_param("format")
        rospy.loginfo(rec_param)
        rec_param = rec_param.split(" ")

        ##### Video file param ######
        self.vidcap = cv2.VideoCapture('http://192.168.1.100:8090/?action=stream')
        self.fourcc = cv2.VideoWriter_fourcc(*'XVID')
        self.file_out = None

        self.file_counter = 1
        self.frame_counter = 0
        self.frame_rate = int(self.vidcap.get(cv2.CAP_PROP_FPS))
        self.minuts = 3
        
        # self.resolution = (int(rec_param[1]),int(rec_param[3]))
        self.video_path = "/video/"
        self.photo_path = "/photo/"
        self.frame_num = 0
        self.cam_comm_data = 0

        self.ffmpeg_process = None
        
        self.seconds_per_file = self.frame_rate * self.minuts * 60 # 5 minutes 30 fps

        self.video_writer_status = 0

        rospy.loginfo("FPS: " + str(self.frame_rate))
        self.loop()

        # dd = self.dt_get()

        # print(str(dt.now().strftime("%d%m%Y_%H%M%S")) + ".avi")
        # print(self.video_name_get())

        # self.video_writer = cv2.VideoWriter(self.video_name_get(), self.fourcc, self.fps, self.resolution)

    def __del__(self):
        self.vidcap.release()
        if self.video_writer_status > 0:
            rospy.set_param("video_status", 0)
            # self.video_writer.release()
        if self.file_out is not None:
            self.file_out.release()
        cv2.destroyAllWindows()

    def cam_cmd(self, msg):
        self.cam_comm_data = msg.data
        # rospy.loginfo(self.cam_comm_data)


    def loop(self):        
        while not rospy.is_shutdown():
            if self.video_writer_status > 0 or self.cam_comm_data == 5 or self.cam_comm_data == 1:
                wait_mil_sec = 25
                i = 0
                while i <= wait_mil_sec:
                    if self.cam_comm_data == 5: # check photo cmd
                        ret, self.frame = self.vidcap.read()
                        if not ret:
                            break                    
                        self.show_image(self.frame)
                        self.photo_writer(self.frame)
                        self.cam_comm_data = 0

                    if self.cam_comm_data == 1 or self.video_writer_status == 1: # video writing
                        ret, self.frame = self.vidcap.read()
                        if not ret:
                            break                     
                        self.video_write_url(self.frame)
                        self.cam_comm_data == 0

                    if self.cam_comm_data == 3: # end recording
                        if self.video_writer_status > 0:
                            self.file_out.release()
                            self.frame_counter = 0
                        self.cam_comm_data, self.video_writer_status = 0, 0
                        rospy.loginfo("-- End file writing! --")
                        rospy.set_param("video_status", 0)
                


    def show_image(self, img, title='Camera'):
        cv2.imshow(title, img)
        cv2.waitKey(3)


    def video_write_url(self, frame):
        if self.video_writer_status == 0 or self.frame_counter == self.seconds_per_file:
            if self.file_out is not None:   
                self.file_out.release()        
            rospy.loginfo("-- File created --")
            self.file_out = cv2.VideoWriter(self.video_name_get(), self.fourcc, self.frame_rate, (self.frame.shape[1], self.frame.shape[0]))
            self.video_writer_status = 1
            rospy.set_param("video_status", 1)
            self.frame_counter = 0

        self.file_out.write(frame)
        self.frame_counter += 1
        rospy.loginfo(self.seconds_per_file - self.frame_counter)


    def video_name_get(self):
        timestamp = dt.now().strftime("%Y%m%d_%H%M%S")
        name = os.path.expanduser('~') + self.video_path + timestamp + ".mp4"
        return(name)


    def start_recording(self, output_dir, input_url):

        if not os.path.exists(output_dir):
            os.makedirs(output_dir)

        # timestamp = dt.now().strftime("%Y%m%d_%H%M%S")
        # output_path = os.path.expanduser('~') + self.video_path + timestamp + ".mp4"
        # output_path = os.path.join(output_dir, f"trionix_{timestamp}_%03d.mp4")
        output_path = self.video_name_get()
        

        ffmpeg_command = [
            'ffmpeg',
            '-i', input_url,
            # '-r', '15',
            '-codec:v', 'libx264',
            '-codec:v', 'copy',
            '-threads', '0',
            '-crf', '38',
            '-x264-params', 'opencl=true',
            '-preset', 'medium',
            # '-pix_fmt', 'yuv420p',
            '-segment_time', '20',
            '-f', 'segment',
            '-reset_timestamps', '1',
            '-movflags', '+faststart',
            output_path
        ]

        self.ffmpeg_process = subprocess.Popen(ffmpeg_command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

        print("Recording started.")


    def stop_recording(self):
        if self.ffmpeg_process is not None:
            self.ffmpeg_process.send_signal(signal.SIGTERM)
            self.ffmpeg_process.wait()  # Подождите завершения процесса
            self.ffmpeg_process = None
            print("Recording stopped.")
        else:
            print("No recording process found.")


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
        if msg.data == 5:
            self.photo_writer(self.vidcap)
        # rospy.loginfo(msg.data)


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
        filename = self.photo_name_get()
        # rospy.loginfo(filename)
        # image = cv2.rotate(img, cv2.ROTATE_180)
        # img = image        
        cv2.imwrite(filename, img)
        os.chmod(filename, 0o666)


    def run(self):
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('cam_record_')
    image_writer = ImageWriter()
    # image_writer.run()
