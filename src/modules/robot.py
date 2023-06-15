import json
from collections import defaultdict

import rospy
from std_msgs.msg import String


class Robot:
    def __init__(self, id,
                 uwb_slave_A_info_pos, uwb_slave_B_info_pos, uwb_master_A_info_pos, uwb_master_B_info_pos,
                 vehicle_length, yaw_idx):
        self.id = id
        self.name = f"robot{self.id}"
        self.namespace = f"/{self.name}"
        self.uwb_master_A = self.name + "::" + "uwb_master_A"
        self.uwb_master_B = self.name + "::" + "uwb_master_B"
        self.uwb_slave_A = self.name + "::" + "uwb_slave_A"
        self.uwb_slave_B = self.name + "::" + "uwb_slave_B"
        self.uwb_slave_A_info_pos = uwb_slave_A_info_pos
        self.uwb_slave_B_info_pos = uwb_slave_B_info_pos
        self.uwb_master_A_info_pos = uwb_master_A_info_pos
        self.uwb_master_B_info_pos = uwb_master_B_info_pos
        self.vehicle_length = vehicle_length
        self.yaw_idx = yaw_idx

        self.twr_sub = rospy.Subscriber(f"{self.namespace}/twr_ranges", String, self.robot_twr_range_sub_callback)
        self.robot_dist_pub = rospy.Publisher(f"{self.namespace}/robot_dist", String, queue_size=10)
        self.A_end_robot_range_map = defaultdict(lambda: {'A': float('inf'), 'B': float('inf')})
        self.B_end_robot_range_map = defaultdict(lambda: {'A': float('inf'), 'B': float('inf')})
        self.A_end_nearest_robot_end_map = defaultdict(dict)
        self.B_end_nearest_robot_end_map = defaultdict(dict)

    @property
    def robot_distance_map(self):
        robot_distance_map = defaultdict(dict)
        for robot_id, nearest_end in self.A_end_nearest_robot_end_map.items():
            robot_distance_map[robot_id]['A'] = self.A_end_robot_range_map[robot_id][nearest_end]
        for robot_id, nearest_end in self.B_end_nearest_robot_end_map.items():
            robot_distance_map[robot_id]['B'] = self.B_end_robot_range_map[robot_id][nearest_end]
        return {
            robot_id: min(v.values()) for robot_id, v in robot_distance_map.items()
        }

    def robot_twr_range_sub_callback(self, msg):
        rospy.loginfo(f"{self.name} sees {self.robot_distance_map}")
        data = json.loads(msg.data)
        network_id = data['network_id']
        twr_range_map = data['twr_range_map']
        if network_id == self.uwb_master_A:
            self.A_end_robot_range_map = twr_range_map
            self.A_end_nearest_robot_end_map = {
                robot_id: 'A' if v['A'] <= v['B'] else 'B' for robot_id, v in self.A_end_robot_range_map.items()
            }
            for robot_id, end_dist_map in twr_range_map.items():
                for end, dist in end_dist_map.items():
                    msg_pub = String()
                    msg_pub.data = json.dumps(self.robot_distance_map)
                    self.robot_dist_pub.publish(msg_pub)
        elif network_id == self.uwb_master_B:
            self.B_end_robot_range_map = twr_range_map
            self.B_end_nearest_robot_end_map = {
                robot_id: 'A' if v['A'] <= v['B'] else 'B' for robot_id, v in self.B_end_robot_range_map.items()
            }
            for robot_id, end_dist_map in twr_range_map.items():
                for end, dist in end_dist_map.items():
                    msg_pub = String()
                    msg_pub.data = json.dumps(self.robot_distance_map)
                    self.robot_dist_pub.publish(msg_pub)



