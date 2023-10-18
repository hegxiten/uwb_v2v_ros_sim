import os
import signal
import subprocess
import time
from collections import defaultdict

import numpy as np

import rospy
from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from constants import MINIMUM_DISTANCE, MAXIMUM_DISTANCE
from utils.utils import kill_gazebo, shutdown_ros_node, kill_ros_master


class Simulator:

    def __init__(self, robot_idx, movement_direction, speed, kill_on_end=1):
        self.robot_idx = robot_idx
        self.kill_on_end = kill_on_end
        self.A_pose, self.B_pose = None, None
        # Wait for all the services
        rospy.wait_for_service('/gazebo/get_link_state')
        rospy.wait_for_service('/gazebo/set_link_state')
        # Wait for all the topics
        rospy.wait_for_message('/gazebo/link_states', LinkStates)
        rospy.wait_for_message('/uwb_aloha_networking', String)
        rospy.sleep(1)

        self.movement_direction = movement_direction
        self.speed = speed

        self.vel_pub = rospy.Publisher(f'/robot{robot_idx}/cmd_vel', Twist, queue_size=10)
        self.link_state_sub = rospy.Subscriber('/gazebo/link_states', LinkStates, self.link_states_callback)

        self.ground_truth_robot_dist_end_map = defaultdict(lambda: {'A': float('inf'), 'B': float('inf')})

        rospy.Timer(rospy.Duration(0.1), self.timer_callback_pub_vel_msg)

    def timer_callback_pub_vel_msg(self, event):
        vel_msg = Twist()
        vel_msg.linear.y = self.speed * self.movement_direction
        self.vel_pub.publish(vel_msg)

    def link_states_callback(self, msg):
        for name, pose, twist in zip(msg.name, msg.pose, msg.twist):
            if f'robot{self.robot_idx}' in name and 'A_end_virtual_frame' in name:
                self.A_pose = pose
                continue
            if f'robot{self.robot_idx}' in name and 'B_end_virtual_frame' in name:
                self.B_pose = pose
                continue
            if not self.A_pose or not self.B_pose:
                continue
            if f'robot' in name and 'end_virtual_frame' in name:
                peer_id = name.split('::')[0]
                peer_end = name.split('::')[1][0]
                peer_pose = pose
                self.ground_truth_robot_dist_end_map[peer_id][peer_end] = self.get_dist(peer_pose)
        if self.kill_on_end:
            if self.ground_truth_robot_dist_map and float('inf') not in list(self.ground_truth_robot_dist_map.values()):
                if any(np.array(list(self.ground_truth_robot_dist_map.values())) < MINIMUM_DISTANCE) or \
                   all(np.array(list(self.ground_truth_robot_dist_map.values())) > MAXIMUM_DISTANCE):
                    rospy.loginfo(f"Robot {self.robot_idx} is too close to another robot")
                    kill_gazebo()
                    shutdown_ros_node()
                    time.sleep(2)


    def get_dist(self, peer_pose):
        if not self.A_pose or not self.B_pose:
            return float('inf')
        A_x, A_y, A_z = self.A_pose.position.x, self.A_pose.position.y, self.A_pose.position.z
        B_x, B_y, B_z = self.B_pose.position.x, self.B_pose.position.y, self.B_pose.position.z
        peer_x, peer_y, peer_z = peer_pose.position.x, peer_pose.position.y, peer_pose.position.z
        return min(abs(A_y - peer_y), abs(B_y - peer_y),)

    @property
    def ground_truth_robot_dist_map(self):
        return {
            k: min(v.values()) for k, v in self.ground_truth_robot_dist_end_map.items()
        }

    def stop_simulation(self):
        # Get the PID of roscore or roslaunch
        ps = subprocess.Popen(["ps", "ax"], stdout=subprocess.PIPE)
        out, err = ps.communicate()
        for line in out.splitlines():
            if b"roscore" in line or b"roslaunch" in line:
                pid = int(line.split(None, 1)[0])
                os.kill(pid, signal.SIGINT)
                time.sleep(1)

