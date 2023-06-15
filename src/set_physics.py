#!/usr/bin/env python
from gazebo_msgs.srv import SetPhysicsProperties, GetPhysicsProperties, SetPhysicsPropertiesRequest
import rospy

rospy.init_node('set_physics')

rospy.wait_for_service('/gazebo/get_physics_properties')
get_physics_properties = rospy.ServiceProxy('/gazebo/get_physics_properties', GetPhysicsProperties)

physics_properties = get_physics_properties()

# To slow down/speed up the simulation to control the speed of real time
physics_properties.time_step *= 1
physics_properties.max_update_rate /= 1

rospy.wait_for_service('/gazebo/set_physics_properties')
set_physics_properties = rospy.ServiceProxy('/gazebo/set_physics_properties', SetPhysicsProperties)

resp = set_physics_properties(physics_properties.time_step, physics_properties.max_update_rate, physics_properties.gravity, physics_properties.ode_config)
if resp.success:
    print("Physics properties set successfully")
else:
    print("Failed to set physics properties")