#!/usr/bin/env python
import roslaunch
import rospy
from modules.simulator import Simulator

global simulator

def move_straight():
    rospy.init_node('move_robot', anonymous=True)
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    launch = roslaunch.scriptapi.ROSLaunch()
    launch.start()

    movement_direction = int(rospy.get_param('~movement_direction', default=1))
    speed = int(rospy.get_param('~speed', default=1))
    kill_on_end = int(rospy.get_param('~kill_on_end', default=1))
    rospy.loginfo(f"MOVEMENT DIRECTION: {movement_direction}")
    simulator = Simulator(robot_idx=1,
                          movement_direction=movement_direction,
                          speed=speed,
                          kill_on_end=kill_on_end)
    rospy.spin()


if __name__ == '__main__':
    try:
        move_straight()
    except rospy.ROSInterruptException:
        pass

