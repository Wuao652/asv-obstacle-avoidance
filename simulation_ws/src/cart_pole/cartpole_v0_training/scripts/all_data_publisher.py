#! /usr/bin/env python

import rospy

from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState #/cartpole_v0/joint_states
from sensor_msgs.msg import Imu #/cartpole_v0/imu/data
from nav_msgs.msg import Odometry #/odom
from tf.transformations import euler_from_quaternion

#some variables
base_data = [0,0]
roll = pitch = yaw = 0.0

def joint_state_cb(data):
    global base_data
    base_data[0] = data.position[0]
    base_data[1] = data.velocity[0]

def imu_cb(data):
    global roll, pitch, yaw
    orientation_q = data.orientation
    (roll, pitch, yaw) = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
    
    

if __name__ == '__main__':
    rospy.init_node('allData_node', log_level= rospy.INFO)
    rospy.loginfo('ALL DATA RUNNING ...')
    
    #create publisher
    all_data_pub = rospy.Publisher('/cartpole_v0/all_data', Float64MultiArray, queue_size=1000)
    
    #create subscribers 
    joint_state_sub = rospy.Subscriber('/cartpole_v0/joint_states', JointState, joint_state_cb)
    imu_sub = rospy.Subscriber('/cartpole_v0/imu/data', Imu, imu_cb)
    
    rate = rospy.Rate(1)
    try:
        while not rospy.is_shutdown():
            
            #assemble and publish all relevant data pieces
            global all_data_pub, base_data, pitch
            #prepare message
            msg = Float64MultiArray()
            msg.data = [base_data[0], base_data[1], pitch]
            all_data_pub.publish(msg)
            
            rate.sleep()
    except rospy.ROSInterruptException:
        rospy.loginfo("ALL DATA TERMINATED!")
    
    
    