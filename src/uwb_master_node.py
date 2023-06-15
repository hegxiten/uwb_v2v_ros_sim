#!/usr/bin/env python
import roslaunch
import rospy
from modules.uwb_tag import UwbTag

global master

def timer_callback(event):
    global master
    master.update_twr_ranges()


if __name__ == "__main__":
    rospy.init_node('spawned_uwb_master', anonymous=True)
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    launch = roslaunch.scriptapi.ROSLaunch()
    launch.start()

    network_id = rospy.get_param('~network_id')
    uwb_range = int(rospy.get_param('~uwb_range'))
    rospy.loginfo(f"Starting Node UWB Master #{network_id}")
    master = UwbTag(network_id, uwb_range=uwb_range, backoff_scale=1)
    rospy.Timer(rospy.Duration(0.1), timer_callback)
    rospy.spin()
