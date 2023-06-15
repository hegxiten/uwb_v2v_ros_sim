#!/usr/bin/env python
import os
import roslaunch
import rospy
import sys

def main(args):
    rospy.init_node('multi_robot_spawner', anonymous=True)
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    launch = roslaunch.scriptapi.ROSLaunch()
    launch.start()

    robot_num = int(rospy.get_param('~robot_num', default=1))
    robot_spacing = int(rospy.get_param('~robot_spacing', default=20))
    max_range = int(rospy.get_param('~max_range'))
    for i in range(robot_num):
        node = roslaunch.core.Node(package='uwb_v2v_ros_sim',
                                   node_type='robot_node.py',
                                   name=f'robot{i}',
                                   args=f'_robot_index:={i} _yaw_idx:={i} _robot_spacing:={robot_spacing} _max_range:={max_range}',
                                   output='screen')
        launch.launch(node)
        rospy.sleep(0.5)


if __name__ == '__main__':
    main(sys.argv)
    rospy.spin()
