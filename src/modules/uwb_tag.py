import json
import random
from collections import defaultdict

import numpy as np

import rospy
import tf
from gazebo_msgs.msg import LinkStates
from std_msgs.msg import String
from std_srvs.srv import Trigger
from utils.utils import packet_loss_probability


class UwbTag:
    def __init__(self, network_id: str, uwb_range: int, backoff_scale: int = 1):
        self.network_id = network_id
        self.backoff_scale = backoff_scale
        self.max_range = uwb_range
        self.link_states_sub = rospy.Subscriber('/gazebo/link_states', LinkStates, self.link_states_callback)
        self.ground_truth_anchor_pose_map = {}
        self.ground_truth_tag_pose_map = {}
        self.pose = None
        self.paired_slave_id = self.network_id.replace('master', 'slave')
        self.service_proxy_name = f"get_in_sync_{self.paired_slave_id.replace('::', '__')}"
        self.persist_twr_range_map = defaultdict(lambda: {'A': float('inf'), 'B': float('inf')})
        self.robot_id = self.network_id.split('::')[0]

        self.twr_pub = rospy.Publisher('twr_ranges', String, queue_size=10)

        self.listener = tf.TransformListener()

    def get_in_sync_peer_map(self):
        try:
            get_json = rospy.ServiceProxy(self.service_proxy_name, Trigger)
            response = get_json()
            data = json.loads(response.message)
            return data
        except rospy.ServiceException as e:
            rospy.logwarn("Service call failed: %s" % e)

    def update_twr_ranges(self):
        in_sync_peer_map = self.get_in_sync_peer_map()
        in_sync_peer_cnt = 0
        instant_twr_range_map = defaultdict(lambda: {'A': float('inf'), 'B': float('inf')})
        for peer_id, timestamp in in_sync_peer_map.items():
            if peer_id == self.network_id:
                continue
            if peer_id.split('::')[1].startswith('uwb_anchor') or peer_id.split('::')[1].startswith('uwb_slave'):
                robot_id = peer_id.split('::')[0]
                end = peer_id.split('::')[1][-1]
                if robot_id != self.robot_id:
                    ground_truth_twr = self.ground_truth_anchor_dist_map.get(peer_id)
                    if random.random() < packet_loss_probability(ground_truth_twr, self.max_range):
                        continue
                    robot_dist = self.twr_dist_to_robot_dist(ground_truth_twr, peer_id)
                    self.persist_twr_range_map[robot_id][end] = robot_dist
                    instant_twr_range_map[robot_id][end] = robot_dist
                    in_sync_peer_cnt += 1
        if in_sync_peer_cnt == 0:
            return
        msg = String()
        msg.data = json.dumps({
            'network_id': self.network_id,
            'twr_range_map': instant_twr_range_map,
        })
        self.twr_pub.publish(msg)

    def twr_dist_to_robot_dist(self, twr_dist, peer_id):
        # TODO: use tf to make use of info_pos
        return twr_dist

    def link_states_callback(self, msg):
        for name, pose, twist in zip(msg.name, msg.pose, msg.twist):
            if name == self.network_id:
                self.pose = pose
                continue
            if name.split('::')[1].startswith('uwb_anchor') or name.split('::')[1].startswith('uwb_slave'):
                self.ground_truth_anchor_pose_map[name] = pose
                continue
            if name.split('::')[1].startswith('uwb_tag') or name.split('::')[1].startswith('uwb_master'):
                self.ground_truth_tag_pose_map[name] = pose
                continue

    @property
    def ground_truth_anchor_dist_map(self):
        return {
            k: np.linalg.norm(
                np.array([v.position.x, v.position.y, v.position.z]) -
                np.array([self.pose.position.x, self.pose.position.y, self.pose.position.z])
            ) for k, v in self.ground_truth_anchor_pose_map.items()
        } if (len(self.ground_truth_anchor_pose_map) != 0 and self.pose is not None) else {}

    @property
    def ground_truth_tag_dist_map(self):
        return {
            k: np.linalg.norm(
                np.array([v.position.x, v.position.y, v.position.z]) -
                np.array([self.pose.position.x, self.pose.position.y, self.pose.position.z])
            ) for k, v in self.ground_truth_tag_pose_map.items()
        } if (len(self.ground_truth_tag_pose_map) != 0 and self.pose is not None) else {}

    @property
    def nearest_end_map(self):
        return {
            k: 'A' if v['A'] <= v['B'] else 'B' for k, v in self.persist_twr_range_map.items()
        } if len(self.persist_twr_range_map) != 0 else {}
