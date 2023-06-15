#!/usr/bin/env python
import json

import roslaunch
import rospy
from std_srvs.srv import Trigger, TriggerResponse
from modules.uwb_anchor import UwbAnchor

global slave


def handle_master_in_sync_request(req):
    global slave
    response = TriggerResponse()
    response.success = True
    response.message = json.dumps(slave.in_sync_peer_map)
    return response


def timer_callback(event):
    global slave
    rospy.sleep(0.1 + slave.aloha_attempt * slave.backoff_scale)
    slave.seat_map_publish()


if __name__ == "__main__":
    rospy.init_node('spawned_uwb_slave', anonymous=True)
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    launch = roslaunch.scriptapi.ROSLaunch()
    launch.start()

    network_id = rospy.get_param('~network_id')
    uwb_range = int(rospy.get_param('~uwb_range'))
    rospy.loginfo(f"Starting Node UWB Slave #{network_id}")
    slave = UwbAnchor(network_id, 16, uwb_range=uwb_range, backoff_scale=1)
    s = rospy.Service(f"get_in_sync_{network_id.replace('::', '__')}", Trigger, handle_master_in_sync_request)
    rospy.Timer(rospy.Duration(0.1), timer_callback)
    rospy.spin()
