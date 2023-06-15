import json
import random
from collections import defaultdict

import numpy as np

import rospy
from constants import STALE_THRESHOLD
from gazebo_msgs.msg import LinkStates
from std_msgs.msg import String
from utils.utils import packet_loss_probability


class UwbAnchor:
    def __init__(self, network_id: str, max_seats: int, uwb_range: int, backoff_scale: int = 1):
        self.network_id = network_id
        self.max_seats = max_seats
        self.backoff_scale = backoff_scale
        self.max_range = uwb_range
        self.seat = -1
        self.timestamped_seat_map = defaultdict(lambda: {
            'seat': -1,
            'last_seen_timestamp': rospy.Time.now().to_sec()
        })
        self.aloha_attempt = 0
        self.aloha_pub = rospy.Publisher('/uwb_aloha_networking', String, queue_size=10)
        self.aloha_sub = rospy.Subscriber('/uwb_aloha_networking', String, self.aloha_callback)

        self.link_states_sub = rospy.Subscriber('/gazebo/link_states', LinkStates, self.link_states_callback)
        self.ground_truth_anchor_pose_map = {}
        self.ground_truth_tag_pose_map = {}
        self.pose = None

    @property
    def in_sync_peer_map(self):
        return {k: v['last_seen_timestamp'] for k, v in self.timestamped_seat_map.items()
                if rospy.Time.now().to_sec() - v['last_seen_timestamp'] < STALE_THRESHOLD and k != self.network_id}

    @property
    def seat_map(self):
        return {k: v['seat'] for k, v in self.timestamped_seat_map.items()}

    def aloha_callback(self, msg):
        try:
            msg_dict = json.loads(msg.data)
        except json.decoder.JSONDecodeError:
            rospy.logerr(f"UWB Anchor {self.network_id} received invalid aloha_seat_map_dict: [{msg.data}]")
            return
        publisher = msg_dict['publisher']
        aloha_seat_map_dict = msg_dict['seat_map']
        ground_truth_dist = self.ground_truth_anchor_dist_map.get(publisher)
        if ground_truth_dist is None:
            return
        if publisher == self.network_id:
            return
        if random.random() < packet_loss_probability(ground_truth_dist, self.max_range):
            return
        self.process_received_aloha_msg(msg)

    def process_received_aloha_msg(self, msg):
        msg_dict = json.loads(msg.data)
        publisher = msg_dict['publisher']
        aloha_seat_map_dict = msg_dict['seat_map']
        if self.network_id in aloha_seat_map_dict:
            agreed = False
            if self.seat == -1:
                # Previously joined the network already, take that seat again
                self.seat = aloha_seat_map_dict[self.network_id]
            elif aloha_seat_map_dict[self.network_id]['seat'] == self.seat:
                # Joined the network and self.seat is agreed by publisher - update the timestamps informed by publisher
                agreed = True
            else:
                # Repetitive network joining attempts happened before and self-assigned some other seat.
                # Grab that chosen seat and join the network
                self.seat = aloha_seat_map_dict[self.network_id]['seat']
            self.update_timestamped_seat_map(publisher, aloha_seat_map_dict)
            self.aloha_attempt = 0
            if not agreed:
                self.seat_map_publish()
        else:
            if self.seat == -1:
                # Not joined the network - attempt to join the network
                exit_code = self.attempt_network_join(aloha_seat_map_dict)
                if exit_code == 0:
                    self.seat_map_publish()
                    self.aloha_attempt = 0
                else:
                    self.aloha_attempt += 1
                    rospy.logwarn("No seats available, wait to try again")
                    rospy.sleep(self.aloha_attempt * self.backoff_scale)
            else:
                # Joined the network but self.seat is not agreed by publisher
                # (someone else is not aware of self.seat)
                if self.seat not in [v['seat'] for v in aloha_seat_map_dict.values()]:
                    # self.seat is not taken by anyone else
                    assert self.timestamped_seat_map[self.network_id]['seat'] == self.seat
                    self.update_timestamped_seat_map(publisher, aloha_seat_map_dict)
                    self.seat_map_publish()
                    self.aloha_attempt = 0
                else:
                    # self.seat is taken by someone else, reset (kick self out of the network)
                    self.seat = -1
                    self.update_timestamped_seat_map(publisher, aloha_seat_map_dict)

    def update_timestamped_seat_map(self, publisher, aloha_seat_map_dict):
        try:
            timestamp = rospy.Time.now().to_sec()
            self.timestamped_seat_map[publisher]['seat'] = aloha_seat_map_dict[publisher]['seat']
            self.timestamped_seat_map[publisher]['last_seen_timestamp'] = timestamp
            for k, v in aloha_seat_map_dict.items():
                if k != publisher and k != self.network_id:
                    self.timestamped_seat_map[k]['seat'] = v['seat']
                    if timestamp - v['last_seen_timestamp'] < STALE_THRESHOLD:
                        self.timestamped_seat_map[k]['last_seen_timestamp'] = max(
                            self.timestamped_seat_map[k]['last_seen_timestamp'],
                            v['last_seen_timestamp']
                        )
            self.timestamped_seat_map.update({
                self.network_id: {
                    'seat': self.seat,
                    'last_seen_timestamp': timestamp,
                }
            })

        except KeyError:
            rospy.logwarn(f"UWB Anchor {self.network_id} received invalid aloha_seat_map_dict: [{aloha_seat_map_dict}]")
            rospy.logerr(f"UWB Anchor {self.network_id} self.timestamped_seat_map: [{self.timestamped_seat_map}]")

    def attempt_network_join(self, aloha_seat_map_dict):
        # Attempting joining the network
        rospy.loginfo(f"Anchor #{self.network_id} attempting joining the network")
        seats_available = [i for i in range(self.max_seats) if i not in aloha_seat_map_dict.values()]
        if len(seats_available) != 0:
            chosen_seat = random.choice(seats_available)
            self.seat = chosen_seat
            self.timestamped_seat_map.update(aloha_seat_map_dict)
            self.timestamped_seat_map.update({
                self.network_id: {
                    'seat': chosen_seat,
                    'last_seen_timestamp': rospy.Time.now().to_sec(),
                }
            })
            return 0
        else:
            self.aloha_attempt += 1
            return 1

    def seat_map_publish(self):
        payload = {
            'publisher': self.network_id,
            'seat_map': self.timestamped_seat_map,
        }
        msg = String()
        msg.data = json.dumps(payload)
        self.aloha_pub.publish(msg)

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
