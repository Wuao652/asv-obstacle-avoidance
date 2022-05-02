#! /usr/bin/env python3
import rospy
import numpy as np
from gazebo_msgs.srv import SetLinkState, SetLinkStateRequest

rospy.init_node('p_robot_env')
rospy.wait_for_service('/gazebo/set_link_state')

set_state_service = rospy.ServiceProxy('/gazebo/set_link_state', SetLinkState)

obstacle_position = np.array([[-19.0, -48.0, 0.0],
                              [-5.0,  -44.0, 0.0],
                              [-19.0, -10.0, 0.0],
                              [-3.0,  -52.0, 0.0],
                              [13.0,  -49.0, 0.0],
                              [28.0,  -30.0, 0.0],
                              [15.0,  -16.0, 0.0],
                              [5.0,   -54.0, 0.0],
                              [-15.0, -37.0, 0.0],
                              [-18.0, -60.0, 0.0],
                              [-7.0,  -12.0, 0.0],
                              [-28.0, -26.0, 0.0], 
                              [29.0,  -52.0, 0.0], 
                              [7.0,   -26.0, 0.0],
                              [17.0,  -35.0, 0.0]])





for i in range(15):
    obstacle_link = SetLinkStateRequest()
    obstacle_link.link_state.link_name = "polyform_a7_" + str(i) + "::base_link"
    obstacle_link.link_state.pose.position.x = obstacle_position[i][0]
    obstacle_link.link_state.pose.position.y = obstacle_position[i][1]
    obstacle_link.link_state.pose.position.z = obstacle_position[i][2]
    obstacle_link.link_state.reference_frame = "world"
    result = set_state_service(obstacle_link)




# objstate = SetLinkStateRequest()
# # objstate = SetModelStateRequest()  # Create an object of type SetModelStateRequest

# # set red cube pose
# objstate.link_state.link_name = "polyform_a7_0::base_link"
# objstate.link_state.pose.position.x = 0
# objstate.link_state.pose.position.y = 0
# objstate.link_state.pose.position.z = 0
# # objstate.model_state.pose.orientation.w = 1
# # objstate.model_state.pose.orientation.x = 0
# # objstate.model_state.pose.orientation.y = 0
# # objstate.model_state.pose.orientation.z = 0
# # objstate.model_state.twist.linear.x = 0.0
# # objstate.model_state.twist.linear.y = 0.0
# # objstate.model_state.twist.linear.z = 0.0
# # objstate.model_state.twist.angular.x = 0.0
# # objstate.model_state.twist.angular.y = 0.0
# # objstate.model_state.twist.angular.z = 0.0
# objstate.link_state.reference_frame = "world"

# result = set_state_service(objstate)

# objstate1 = SetLinkStateRequest()
# # objstate = SetModelStateRequest()  # Create an object of type SetModelStateRequest

# # set red cube pose
# objstate1.link_state.link_name = "polyform_a7_1::base_link"
# objstate1.link_state.pose.position.x = 0
# objstate1.link_state.pose.position.y = 0
# objstate1.link_state.pose.position.z = 0
# # objstate.model_state.pose.orientation.w = 1
# # objstate.model_state.pose.orientation.x = 0
# # objstate.model_state.pose.orientation.y = 0
# # objstate.model_state.pose.orientation.z = 0
# # objstate.model_state.twist.linear.x = 0.0
# # objstate.model_state.twist.linear.y = 0.0
# # objstate.model_state.twist.linear.z = 0.0
# # objstate.model_state.twist.angular.x = 0.0
# # objstate.model_state.twist.angular.y = 0.0
# # objstate.model_state.twist.angular.z = 0.0
# objstate1.link_state.reference_frame = "world"

# result = set_state_service(objstate1)