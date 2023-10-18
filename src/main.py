#!/usr/bin/env python
"""
__author__ = "Zezhou Wang"
__license__ = "BSD"
__version__ = "0.0.1"
__maintainer__ = "Zezhou Wang"
__email__ = "zezhou.wang@rutgers.edu"
"""

import subprocess
import os
import time
from datetime import datetime, timezone

from constants import get_topics, VEHICLE_LENGTH, MINIMUM_DISTANCE
from utils.utils import kill_gazebo, shutdown_ros_node, kill_ros_master

REPETITION = 5
MAX_RANGE_UNIT = 50  # meters
MAX_RANGES = [MAX_RANGE_UNIT * i for i in range(1, 5)]  # meters

if __name__ == '__main__':
    # set up logging to file
    robot_num = 2
    movement_direction = -1
    kill_on_end = 1
    for uwb_range in MAX_RANGES:
        if uwb_range == 50:
            continue
        robot_spacing = uwb_range + 20
        robot_spacing = max(robot_spacing, VEHICLE_LENGTH + MINIMUM_DISTANCE)
        speeds = [i for i in range(80, 0, -1)]  # m/s
        cnt = 1
        while cnt < REPETITION + 1:
            for speed in speeds:
                sub_dir_name = f'varied_speed_{uwb_range}' + f'/{speed}'
                dt_string = datetime.now(timezone.utc).strftime("%Y-%m-%dT%H-%M-%SZ")
                print(f"------------[START] REPETITION: {cnt} for speed: {speed} range: {uwb_range}-----------------")
                expected_eta_sec = (robot_spacing / speed)
                os.makedirs(os.path.expanduser('~/') + 'catkin_ws/src/uwb_v2v_ros_sim/bagdata/' + sub_dir_name,
                            exist_ok=True)
                bag_file_name = os.path.expanduser(
                    '~/') + 'catkin_ws/src/uwb_v2v_ros_sim/bagdata/' + sub_dir_name + '/' + f'{dt_string}_{cnt}.bag'
                topics = get_topics(robot_num=robot_num)
                topics_arg = ' '.join(topics)
                try:
                    subprocess.call(
                        f"roslaunch uwb_v2v_ros_sim multi_robot.launch "
                        f"'robot_num:={robot_num}' "
                        f"'robot_spacing:={robot_spacing}' "
                        f"'movement_direction:={movement_direction}' "
                        f"'speed:={speed}' "
                        f"'kill_on_end:={kill_on_end}' "
                        f"'bag_file_path:={bag_file_name}' "
                        f"'max_range:={uwb_range}' "
                        f"'topics:={topics_arg}' ",
                        shell=True,
                        timeout=expected_eta_sec / 0.3 + 10
                    )
                    kill_gazebo()
                    shutdown_ros_node()
                    time.sleep(1)
                    print(
                        f"------------[FINISHED] REPETITION: {cnt} for speed: {speed} range: {uwb_range}-----------------")
                except subprocess.TimeoutExpired:
                    print(f"Timeout expired for speed: {speed} at repetition: {cnt}")
                    kill_gazebo()
                    shutdown_ros_node()
                    print(
                        f"------------[TIMED OUT] REPETITION: {cnt} for speed: {speed} range: {uwb_range}-----------------")
                    continue
            cnt += 1
    kill_ros_master()
