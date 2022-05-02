#! /usr/bin/env python3
import rospy
import numpy as np
# from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import Pose
# from gazebo_msgs.srv import GetModelState, GetModelStateRequest


class GazeboBuoys(object):
    def __init__(self, robot_model_name, rate_hz=50.0):
        
        self.link_name = robot_model_name
        self.rate = rospy.Rate(hz=rate_hz)
        self.link_states_sub = rospy.Subscriber("/gazebo/link_states", LinkStates, self.link_states_callback)
        # self.pose_pub = rospy.Publisher("/gazebo/" + self.link_name_rectified, Pose, queue_size = 10)
    def link_states_callback(self, data):
        try:
            print('This is the whole link_states!')
            # print(data.name)
            self.links_name = [n for n in data.name if "buoys::polyform_" in n]
            self.links_num = len(self.links_name)
            self.links_list = np.zeros((3, self.links_num))
            print('------------------------------------------------------')

                
            for i in range(self.links_num):    
                ind = data.name.index(self.links_name[i])
                pose = data.pose[ind]
                print('ID ', ind)
                print(pose)
                self.links_list[:, i] = np.array([pose.position.x, pose.position.y, pose.position.z])
                pass
            print(self.links_list)
            print(self.links_list.shape)
            # ind = data.name.index('buoys::polyform_a7_0::base_link')
            # print('--------------------------------------------------------------------------------')
            # print(ind)
            #self.link_pose = data.pose[ind]
        except ValueError:
            pass    
        
        # # Get model name: rosservice call /gazebo/get_world_properties "{}"
        # odom_topic_name = "/"+str(robot_model_name)+"/odom"
        # self.odom_pub = rospy.Publisher(odom_topic_name, Odometry, queue_size=1)
        
        # ropy.wait_for_service('/gazebo/get_model_properties')
        # self.buoys = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)  
        # rospy.loginfo("GazeboModelOdom is Waiting for service.../gazebo/get_model_state")
        # rospy.wait_for_service('/gazebo/get_model_state')
        # self.get_model_srv = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

        # self.odom = Odometry()
        # header = Header()
        # header.stamp = rospy.Time.now()
        # header.frame_id = 'world'
        # self.odom.header = header

        # self.model = GetModelStateRequest()
        # self.model.model_name = 'wamv'

        # Reset Value to True to reset counter when Time moves backwards ( reset sim )
        # http://docs.ros.org/api/rospy/html/rospy.timer.Rate-class.html#sleep
        # self.rate_object = rospy.Rate(hz=rate_hz, reset=True)

    def get_odometry_loop(self):
        while not rospy.is_shutdown():
            self.rate.sleep()
        # while not rospy.is_shutdown():
        #     result = self.get_model_srv(self.model)
        #     # print(result)
        #     self.odom.pose.pose = result.pose
        #     self.odom.twist.twist = result.twist

        #     header = Header()
        #     header.stamp = rospy.Time.now()
        #     self.odom.header = header

        #     self.odom_pub.publish(self.odom)

        #     self.rate_object.sleep()

if __name__ == "__main__":
    try:
        rospy.init_node('get_buoys_link_pose', anonymous=True)
        gb = GazeboBuoys('buoys')
        publish_rate = 50.0
        gb.get_odometry_loop()

    except rospy.ROSInterruptException:
        pass