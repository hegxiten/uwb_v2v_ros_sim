#!/usr/bin/env python
import os.path
import tempfile

import roslaunch
import rospy
from constants import VEHICLE_LENGTH, UWB_SLAVE_A_DEFAULT_INFO_POS, UWB_SLAVE_B_DEFAULT_INFO_POS, \
    UWB_MASTER_A_DEFAULT_INFO_POS, UWB_MASTER_B_DEFAULT_INFO_POS
from modules.robot import Robot
from xacro import process_file
from lxml import etree

global robot


def spawn_robot(i, launch, xacro_path,
                uwb_slave_A_info_pos, uwb_slave_B_info_pos,
                uwb_master_A_info_pos, uwb_master_B_info_pos,
                uwb_range, vehicle_length, spacing, yaw=0):
    # Load robot_description parameter
    global robot
    robot_name = 'robot{}'.format(i)
    x = 0
    y = i * spacing  # stagger each robot $spacing meters apart

    urdf_content = generate_robot_urdf(yaw,
                                       uwb_slave_A_info_pos, uwb_slave_B_info_pos,
                                       uwb_master_A_info_pos, uwb_master_B_info_pos,
                                       vehicle_length, xacro_path)
    rospy.set_param("/{}/robot_description".format(robot_name), urdf_content.toxml())

    # Spawn the robot in Gazebo
    robot_node = roslaunch.core.Node(
        package='gazebo_ros',
        node_type='spawn_model',
        name='spawn_urdf{}'.format(i),
        args='-urdf -model {0} -x {1} -y {2} -z 0 -param /{0}/robot_description'.format(robot_name, x, y),
        namespace=robot_name,
        respawn=False,
        output='screen'
    )
    launch.launch(robot_node)
    # Init robot state publisher
    robot_state_publisher_node = roslaunch.core.Node(
        package='robot_state_publisher',
        node_type='robot_state_publisher',
        name='robot_state_publisher'.format(i),
        args='robot_description',
        namespace=robot_name,
        respawn=False,
        output='screen'
    )
    launch.launch(robot_state_publisher_node)
    # Init joint state publisher
    joint_state_publisher_node = roslaunch.core.Node(
        package='joint_state_publisher',
        node_type='joint_state_publisher',
        name='joint_state_publisher'.format(i),
        args='robot_description',
        namespace=robot_name,
        respawn=False,
        output='screen'
    )
    launch.launch(joint_state_publisher_node)

    rospy.sleep(0.5)

    # Spawn UWB Slave nodes
    uwb_slave_node_A = roslaunch.core.Node(
        package='uwb_v2v_ros_sim',
        node_type='uwb_slave_node.py',
        name='robot{}_uwb_slave_node_A'.format(i),
        args=f'_network_id:=robot{i}::uwb_slave_A _uwb_range:={uwb_range}',
        namespace=robot_name,
        respawn=False,
        output='screen',
    )
    uwb_slave_node_B = roslaunch.core.Node(
        package='uwb_v2v_ros_sim',
        node_type='uwb_slave_node.py',
        name='robot{}_uwb_slave_node_B'.format(i),
        args=f'_network_id:=robot{i}::uwb_slave_B _uwb_range:={uwb_range}',
        namespace=robot_name,
        respawn=False,
        output='screen',
    )

    launch.launch(uwb_slave_node_A)
    launch.launch(uwb_slave_node_B)

    rospy.sleep(0.5)

    uwb_master_node_A = roslaunch.core.Node(
        package='uwb_v2v_ros_sim',
        node_type='uwb_master_node.py',
        name='robot{}_uwb_master_node_A'.format(i),
        args=f'_network_id:=robot{i}::uwb_master_A _uwb_range:={uwb_range}',
        namespace=robot_name,
        respawn=False,
        output='screen',
    )
    uwb_master_node_B = roslaunch.core.Node(
        package='uwb_v2v_ros_sim',
        node_type='uwb_master_node.py',
        name='robot{}_uwb_master_node_B'.format(i),
        args=f'_network_id:=robot{i}::uwb_master_B _uwb_range:={uwb_range}',
        namespace=robot_name,
        respawn=False,
        output='screen',
    )

    launch.launch(uwb_master_node_A)
    launch.launch(uwb_master_node_B)


def generate_robot_urdf(yaw,
                        uwb_slave_A_info_pos, uwb_slave_B_info_pos,
                        uwb_master_A_info_pos, uwb_master_B_info_pos,
                        vehicle_length, xacro_path):
    # Process the xacro file to generate the URDF content
    temp_file = tempfile.NamedTemporaryFile(delete=True)  # Create a temporary file
    parser = etree.XMLParser(remove_blank_text=True)  # Parse the xacro file
    tree = etree.parse(xacro_path, parser)
    root = tree.getroot()

    for joint in root.iter('joint'):
        if 'name' in joint.attrib and joint.attrib['name'] == 'base_joint':
            for origin in joint.iter('{*}origin'):
                if 'rpy' in origin.attrib:
                    origin.attrib['rpy'] = f'0 0 {yaw}'

    for element in root.iter('{*}A_end_virtual_frame', '{*}B_end_virtual_frame'):
        if 'half_vehicle_length' in element.attrib:
            element.attrib['half_vehicle_length'] = f'{vehicle_length / 2}'

    for uwb_link in root.iter('{*}uwb_link'):
        if 'link_name' in uwb_link.attrib and uwb_link.attrib['link_name'] == 'uwb_slave_A':
            if 'joint_origin' in uwb_link.attrib:
                uwb_link.attrib['joint_origin'] = " ".join([str(x) for x in uwb_slave_A_info_pos])
        if 'link_name' in uwb_link.attrib and uwb_link.attrib['link_name'] == 'uwb_slave_B':
            if 'joint_origin' in uwb_link.attrib:
                uwb_link.attrib['joint_origin'] = " ".join([str(x) for x in uwb_slave_B_info_pos])
        if 'link_name' in uwb_link.attrib and uwb_link.attrib['link_name'] == 'uwb_master_A':
            if 'joint_origin' in uwb_link.attrib:
                uwb_link.attrib['joint_origin'] = " ".join([str(x) for x in uwb_master_A_info_pos])
        if 'link_name' in uwb_link.attrib and uwb_link.attrib['link_name'] == 'uwb_master_B':
            if 'joint_origin' in uwb_link.attrib:
                uwb_link.attrib['joint_origin'] = " ".join([str(x) for x in uwb_master_B_info_pos])

    tree.write(temp_file.name, xml_declaration=True)  # Write the new XML content to the temp file
    urdf_content = process_file(temp_file.name)
    temp_file.close()  # Close and remove the temporary file
    return urdf_content


if __name__ == '__main__':
    rospy.init_node('spawned_robot', anonymous=True)
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    launch = roslaunch.scriptapi.ROSLaunch()
    launch.start()

    i = rospy.get_param('~robot_index')
    yaw_idx = rospy.get_param('~yaw_idx')
    init_spacing = rospy.get_param('~robot_spacing')
    uwb_range = rospy.get_param('~max_range')
    robot = Robot(i,
                  uwb_slave_A_info_pos=UWB_SLAVE_A_DEFAULT_INFO_POS,
                  uwb_slave_B_info_pos=UWB_SLAVE_B_DEFAULT_INFO_POS,
                  uwb_master_A_info_pos=UWB_MASTER_A_DEFAULT_INFO_POS,
                  uwb_master_B_info_pos=UWB_MASTER_B_DEFAULT_INFO_POS,
                  vehicle_length=VEHICLE_LENGTH,
                  yaw_idx=yaw_idx)
    spawn_robot(i, launch,
                xacro_path=os.path.expanduser(
                    '~/catkin_ws/src/uwb_v2v_ros_sim/rail_vehicle_description/model.urdf.xacro'
                ),
                uwb_slave_A_info_pos=UWB_SLAVE_A_DEFAULT_INFO_POS,
                uwb_slave_B_info_pos=UWB_SLAVE_B_DEFAULT_INFO_POS,
                uwb_master_A_info_pos=UWB_MASTER_A_DEFAULT_INFO_POS,
                uwb_master_B_info_pos=UWB_MASTER_B_DEFAULT_INFO_POS,
                uwb_range=uwb_range,
                vehicle_length=VEHICLE_LENGTH,
                spacing=init_spacing,
                yaw=yaw_idx * 3.14159)
    # rospy.Timer(rospy.Duration(0.1), timer_callback)
    rospy.spin()
