#!/usr/bin/env python
# ROS packages required
import rospy
import numpy
from std_msgs.msg import Float64
import time
from gazebo_connection import GazeboConnection
from sensor_msgs.msg import JointState
from rosgraph_msgs.msg import Clock
from controllers_connection import ControllersConnection

class DebugClass(object):

    def __init__(self):
        self._base_pub = rospy.Publisher('/cartpole_v0/foot_joint_position_controller/command', Float64, queue_size=1)
        self._pole_pub = rospy.Publisher('/cartpole_v0/pole_joint_position_controller/command', Float64, queue_size=1)
        self.init_pos = [0.0, 0.0]
        self.publishers_array = []
        self.publishers_array.append(self._base_pub)
        self.publishers_array.append(self._pole_pub)

    def check_publishers_connection(self):
        """
        Checks that all the publishers are working
        :return:
        """
        rate = rospy.Rate(10)  # 10hz
        while self._base_pub.get_num_connections() == 0 and not rospy.is_shutdown():
            rospy.loginfo("No susbribers to _base_pub yet so we wait and try again")
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                # This is to avoid error when world is rested, time when backwards.
                pass
        rospy.loginfo("_base_pub Publisher Connected")

        while self._pole_pub.get_num_connections() == 0 and not rospy.is_shutdown():
            rospy.loginfo("No susbribers to _pole_pub yet so we wait and try again")
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                # This is to avoid error when world is rested, time when backwards.
                pass
        rospy.loginfo("_pole_pub Publisher Connected")

        rospy.loginfo("All Publishers READY")

    def check_all_systems_ready(self, init=True):
        self.base_position = None
        while self.base_position is None and not rospy.is_shutdown():
            try:
                self.base_position = rospy.wait_for_message("/cartpole_v0/joint_states", JointState, timeout=1.0)
                rospy.loginfo("Current cartpole_v0/joint_states READY=>"+str(self.base_position))
                if init:
                    # We Check all the sensors are in their initial values
                    positions_ok = all(abs(i) <= 1.0e-02 for i in self.base_position.position)
                    velocity_ok = all(abs(i) <= 1.0e-02 for i in self.base_position.velocity)
                    efforts_ok = all(abs(i) <= 1.0e-01 for i in self.base_position.effort)
                    base_data_ok = positions_ok and velocity_ok and efforts_ok
                    rospy.loginfo("Checking Init Values Ok=>" + str(base_data_ok))
            except:
                rospy.loginfo("Current cartpole_v0/joint_states not ready yet, retrying for getting joint_states")
        rospy.logdebug("ALL SYSTEMS READY")

    def move_joints(self, joints_array):
        rospy.loginfo("Move Joint Value=" + str(joints_array))
        joint_value = Float64()
        joint_value.data = joints_array[0]
        rospy.loginfo("Single Base JointsPos>>"+str(joint_value))
        self._base_pub.publish(joint_value)

    def set_init_pose(self):
        """
        Sets joints to initial position [0,0,0]
        :return:
        """
        self.check_publishers_connection()
        self.move_joints(self.init_pos)

    def get_clock_time(self):
        self.clock_time = None
        while self.clock_time is None and not rospy.is_shutdown():
            try:
                self.clock_time = rospy.wait_for_message("/clock", Clock, timeout=1.0)
                rospy.loginfo("Current clock_time READY=>" + str(self.clock_time))
            except:
                rospy.loginfo("Current clock_time not ready yet, retrying for getting Current clock_time")
        return self.clock_time

if __name__ == '__main__':
    rospy.init_node('debug_test_node', anonymous=True)

    controllers_object = ControllersConnection(namespace="cartpole_v0")

    debug_object = DebugClass()
    wait_time = 0.1
    gazebo = GazeboConnection()
    rospy.loginfo("RESETING SIMULATION")
    gazebo.pauseSim()
    gazebo.resetSim()
    gazebo.unpauseSim()
    rospy.loginfo("CLOCK AFTER RESET")
    debug_object.get_clock_time()
    rospy.loginfo("RESETING CONTROLLERS SO THAT IT DOESNT WAIT FOR THE CLOCK")
    controllers_object.reset_cartpole_joint_controllers()
    rospy.loginfo("AFTER RESET CHECKING SENSOR DATA")
    debug_object.check_all_systems_ready()
    rospy.loginfo("CLOCK AFTER SENSORS WORKING AGAIN")
    debug_object.get_clock_time()

    while not rospy.is_shutdown():
        raw_input("Start Loop...PRESS KEY")
        rospy.loginfo("START CHECKING SENSOR DATA")
        debug_object.check_all_systems_ready()
        rospy.loginfo("SET init pose...")
        debug_object.set_init_pose()
        rospy.loginfo("WAIT FOR GOING TO INIT POSE")
        time.sleep(wait_time)

        pos_x = [0.0]
        #INCREMENT_FORCE = 1000.0 # EFFORT VALUE
        INCREMENT_FORCE = 0.1
        NUM_REPETITIONS = 1000
        r_object = rospy.Rate(100.0)
        for i in range(NUM_REPETITIONS):
            random_int = numpy.random.randint(2, size=1)[0]
            rospy.loginfo("random_int=" + str(random_int))

            if random_int == 0:
                rospy.logwarn("INCREMENT")
                pos_x[0] += INCREMENT_FORCE
            else:
                rospy.logerr("DECREMENT")
                pos_x[0] -= INCREMENT_FORCE

            rospy.loginfo("REPETITION="+str(i))
            rospy.loginfo("GO TO P-X=="+str(pos_x))
            debug_object.move_joints(pos_x)
            rospy.loginfo("WAIT TIME="+str(wait_time))
            #time.sleep(wait_time)
            r_object.sleep()

        rospy.logwarn("STOP SPEED")
        debug_object.move_joints([0.0])
        time.sleep(wait_time*10)

        rospy.loginfo("END CHECKING SENSOR DATA")
        debug_object.check_all_systems_ready()
        rospy.loginfo("CLOCK BEFORE RESET")
        debug_object.get_clock_time()

        rospy.loginfo("SETTING INITIAL POSE TO AVOID")
        debug_object.set_init_pose()
        time.sleep(wait_time*2.0)
        rospy.loginfo("AFTER INITPOSE CHECKING SENSOR DATA")
        debug_object.check_all_systems_ready()
        rospy.loginfo("We deactivate gravity to check any reasidual effect of reseting the simulation")
        gazebo.change_gravity(0.0, 0.0, 0.0)

        rospy.loginfo("RESETING SIMULATION")
        gazebo.pauseSim()
        gazebo.resetSim()
        gazebo.unpauseSim()
        rospy.loginfo("CLOCK AFTER RESET")
        debug_object.get_clock_time()

        rospy.loginfo("RESETING CONTROLLERS SO THAT IT DOESNT WAIT FOR THE CLOCK")
        controllers_object.reset_cartpole_joint_controllers()
        rospy.loginfo("AFTER RESET CHECKING SENSOR DATA")
        debug_object.check_all_systems_ready()
        rospy.loginfo("CLOCK AFTER SENSORS WORKING AGAIN")
        debug_object.get_clock_time()
        rospy.loginfo("We reactivating gravity...")
        gazebo.change_gravity(0.0, 0.0, -9.81)
        rospy.loginfo("END")



