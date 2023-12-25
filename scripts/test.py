#!/usr/bin/env python3

import json
import os
from pathlib import Path

from math import pi, copysign, atan2, sin, cos, radians, degrees

import rospy
import smach

from std_srvs.srv import SetBool
from std_srvs.srv import Empty, EmptyResponse
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist


PATH = os.path.dirname(__file__)
FILE_PATH = "../missions"
FILE_NAME = "/test_mision.json"

os.chdir(PATH)
os.chdir(FILE_PATH)
MISSION_PATH = os.getcwd()

print(MISSION_PATH)


def mission_file_parser():
    os.chdir(PATH)
    os.chdir(FILE_PATH)
    MISSION_PATH = os.getcwd()

    with open(MISSION_PATH + FILE_NAME) as data_file:
        data = data_file.read()
        data = json.loads(data)
        for index, block_name in enumerate(data):
            print(index)
            for block_type in data[block_name].keys():
                print(data[block_name][block_type])
    return data

mission_file_parser()