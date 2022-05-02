#! /usr/bin/env python3
import rospy
import numpy as np
# from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
# from gazebo_msgs.srv import GetModelState, GetModelStateRequest



class GazeboModel(object):
    def __init__(self, robot_model_name, rate_hz=50.0):
        
        self._name = robot_model_name
        self.rate = rospy.Rate(hz=rate_hz)
        self.model_states_sub = rospy.Subscriber("/gazebo/model_states", ModelStates, self.model_states_callback)
        # self.pose_pub = rospy.Publisher("/gazebo/" + self.link_name_rectified, Pose, queue_size = 10)
        
        # self.get_model_srv = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState, self.odo_callback)


    def model_states_callback(self, data):
        try:
            # print('This is the whole link_states!')
            # print(data.name)
            print("get model from gazebo!")
            ind = data.name.index(self._name)
            # pose = data.pose[ind]
            # print(pose)

            self.odom = Odometry()
            header = Header()
            header.stamp = rospy.Time.now()
            header.frame_id = 'world'
            self.odom.header = header
            self.odom.pose.pose = data.pose[ind]
            self.odom.twist.twist = data.twist[ind]
            print(self.odom)
            # for i in range(self.links_num):    
            #     ind = data.name.index(self.links_name[i])
            #     pose = data.pose[ind]
            #     print('ID ', ind)
            #     print(pose)
            #     self.links_list[:, i] = np.array([pose.position.x, pose.position.y, pose.position.z])
            #     pass
            # print(self.links_list)
            # print(self.links_list.shape)
            
        except ValueError:
            pass    
        
    def get_odometry_loop(self):
        while not rospy.is_shutdown():
            self.rate.sleep()



if __name__ == "__main__":
    try:
        rospy.init_node('get_gazebo_model', anonymous=True)
        gb = GazeboModel('wamv')
        publish_rate = 50.0
        gb.get_odometry_loop()

    except rospy.ROSInterruptException:
        pass