import numpy
import rospy
import time
import copy
from openai_ros import robot_gazebo_env
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates, LinkStates
from gazebo_msgs.srv import SetLinkState, SetLinkStateRequest
from std_msgs.msg import Header
# from gazebo_msgs.srv import GetLinkState
from openai_ros.openai_ros_common import ROSLauncher
import subprocess
from std_msgs.msg import Float32
from openai_ros.msg import Course
class WamvEnv(robot_gazebo_env.RobotGazeboEnv):
    """Superclass for all WamvEnv environments.
    """

    def __init__(self, ros_ws_abspath):
        """
        Initializes a new WamvEnv environment.

        To check any topic we need to have the simulations running, we need to do two things:
        1) Unpause the simulation: without that th stream of data doesnt flow. This is for simulations
        that are pause for whatever the reason
        2) If the simulation was running already for some reason, we need to reset the controlers.
        This has to do with the fact that some plugins with tf, dont understand the reset of the simulation
        and need to be reseted to work properly.

        The Sensors: The sensors accesible are the ones considered usefull for AI learning.

        Sensor Topic List:
        * /wamv/odom: Odometry of the Base of Wamv

        Actuators Topic List:
        * /cmd_drive: You publish the speed of the left and right propellers.

        Args:
        """
        rospy.logdebug("Start WamvEnv INIT...")
        # Variables that we give through the constructor.
        # None in this case

        # We launch the ROSlaunch that spawns the robot into the world
        print('Start to put the robot into world!')
        ROSLauncher(rospackage_name="vrx_gazebo",
                    launch_file_name="wuao_obstacle_put_wamv_in_world.launch",
                    ros_ws_abspath=ros_ws_abspath)
        
        # hard code to roslaunch the odometry node 
        # rospy.loginfo(
        #         ">>>>>>>>>>Launch the odometry in workspace-->")
        # odom_rospackage_name = 'vrx_gazebo'
        # odom_launch_file_name = 'wuao_start_odom_publisher.launch'
        # roslaunch_command = "roslaunch  {0} {1}".format(odom_rospackage_name, odom_launch_file_name)
        # command = roslaunch_command
        # rospy.logwarn("Launching command="+str(command))
        # p = subprocess.Popen(command, shell=True)
        # state = p.poll()
        
        # hard code to roslaunch the controller node
        
        print(f"Start to load the controller!")
        rospy.loginfo(
                ">>>>>>>>>>Launch the odometry in workspace-->")
        odom_rospackage_name = 'vrx_control'
        odom_launch_file_name = 'course_controller.launch'
        roslaunch_command = "roslaunch  {0} {1}".format(odom_rospackage_name, odom_launch_file_name)
        command = roslaunch_command
        rospy.logwarn("Launching command="+str(command))
        p = subprocess.Popen(command, shell=True)
        state = p.poll()
        print(f"finish loading the controller!")
        
        
        # from robotx_gazebo.msg import UsvDrive

        # Internal Vars
        # Doesnt have any accesibles
        self.controllers_list = []

        # It doesnt use namespace
        self.robot_name_space = ""

        # We launch the init function of the Parent Class robot_gazebo_env.RobotGazeboEnv
        super(WamvEnv, self).__init__(controllers_list=self.controllers_list,
                                            robot_name_space=self.robot_name_space,
                                            reset_controls=False,
                                            start_init_physics_parameters=False,
                                            reset_world_or_sim="WORLD")



        rospy.logdebug("WamvEnv unpause1...")
        self.gazebo.unpauseSim()
        #self.controllers_object.reset_controllers()

        self._check_all_systems_ready()


        # We Start all the ROS related Subscribers and publishers
        # rospy.Subscriber("/wamv/odom", Odometry, self._odom_callback)
        rospy.Subscriber("/gazebo/model_states", ModelStates, self._odom_callback)
        #self.link_states_sub = rospy.Subscriber("/gazebo/link_states", LinkStates, self.link_states_callback)
        rospy.Subscriber("/gazebo/link_states", LinkStates, self._linkstates_callback)

        # service used for reset the obstacles positions
        rospy.wait_for_service('/gazebo/set_link_state')
        self.set_state_service = rospy.ServiceProxy('/gazebo/set_link_state', SetLinkState)
        
        self.publishers_array = []
        # We Start all the ROS related publishers

        # self._cmd_drive_pub = rospy.Publisher('/cmd_drive', UsvDrive, queue_size=1)
        # self.publishers_array.append(self._cmd_drive_pub)
        print('import Course message')
        self._cmd_drive_pub = rospy.Publisher('/course_cmd', Course, queue_size=1)
        print("import finished")
        # self.left_pub = rospy.Publisher('/wamv/thrusters/left_thrust_cmd', Float32, queue_size=1)
        # self.right_pub = rospy.Publisher('/wamv/thrusters/right_thrust_cmd', Float32, queue_size=1)

        # self.publishers_array.append(self.left_pub)
        # self.publishers_array.append(self.right_pub)
        self.publishers_array.append(self._cmd_drive_pub)
        
        self._check_all_publishers_ready()

        self.gazebo.pauseSim()

        rospy.logdebug("Finished WamvEnv INIT...")

    # Methods needed by the RobotGazeboEnv
    # ----------------------------


    def _check_all_systems_ready(self):
        """
        Checks that all the sensors, publishers and other simulation systems are
        operational.
        """
        rospy.logdebug("WamvEnv check_all_systems_ready...")
        self._check_all_sensors_ready()
        rospy.logdebug("END WamvEnv _check_all_systems_ready...")
        return True


    # CubeSingleDiskEnv virtual methods
    # ----------------------------

    def _check_all_sensors_ready(self):
        rospy.logdebug("START ALL SENSORS READY")
        self._check_odom_ready()
        self._check_obs_list_ready()
        rospy.logdebug("ALL SENSORS READY")


    def _check_odom_ready(self):
        self.odom_data = None
        rospy.logdebug("Waiting for /wamv/odom to be READY...")
        while self.odom_data is None and not rospy.is_shutdown():
            try:
                data = rospy.wait_for_message("/gazebo/model_states", ModelStates, timeout=2.0)
                rospy.logdebug("Current /gazebo/modelstates READY=>")
                self.odom_data = data
            except:
                rospy.logerr("Current /wamv/odom not ready yet, retrying for getting odom")
        return self.odom_data
    
    def _check_obs_list_ready(self):
        self.obs_data = None
        rospy.logdebug("Waiting for /gazebo/linkstates to be READY...")
        while self.obs_data is None and not rospy.is_shutdown():
            try:
                data = rospy.wait_for_message("/gazebo/link_states", LinkStates, timeout=2.0)
                rospy.logdebug("Current /gazebo/linkstates READY=>")
                self.obs_data = data
            except:
                rospy.logerr("Current /gazebo/linkstates not ready yet, retrying for getting obs_list")
        return self.obs_data
        pass

    # def _odom_callback(self, data):
    #     self.odom = data
    def _odom_callback(self, data):
        try:
            self.odom_data = data
        except ValueError:
            pass

    def _linkstates_callback(self, data):
        try:
            # print('This is the whole link_states!')
                # print(data.name)
            self.obs_data = data
        except ValueError:
            pass
    
    
    
    
    def _check_all_publishers_ready(self):
        """
        Checks that all the publishers are working
        :return:
        """
        rospy.logdebug("START ALL SENSORS READY")
        for publisher_object in self.publishers_array:
            self._check_pub_connection(publisher_object)
        rospy.logdebug("ALL SENSORS READY")

    def _check_pub_connection(self, publisher_object):

        rate = rospy.Rate(10)  # 10hz
        while publisher_object.get_num_connections() == 0 and not rospy.is_shutdown():
            rospy.logdebug("No susbribers to publisher_object yet so we wait and try again")
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                # This is to avoid error when world is rested, time when backwards.
                pass
        rospy.logdebug("publisher_object Publisher Connected")

        rospy.logdebug("All Publishers READY")


    # Methods that the TrainingEnvironment will need to define here as virtual
    # because they will be used in RobotGazeboEnv GrandParentClass and defined in the
    # TrainingEnvironment.
    # ----------------------------
    def _set_init_pose(self):
        """Sets the Robot in its init pose
        """
        raise NotImplementedError()

    def _init_env_variables(self):
        """Inits variables needed to be initialised each time we reset at the start
        of an episode.
        """
        raise NotImplementedError()

    def _compute_reward(self, observations, done):
        """Calculates the reward to give based on the observations given.
        """
        raise NotImplementedError()

    def _set_action(self, action):
        """Applies the given action to the simulation.
        """
        raise NotImplementedError()

    def _get_obs(self):
        raise NotImplementedError()

    def _is_done(self, observations):
        """Checks if episode done based on observations given.
        """
        raise NotImplementedError()

    # Methods that the TrainingEnvironment will need.
    # ----------------------------
    
    # def set_propellers_speed(self, right_propeller_speed, left_propeller_speed, time_sleep=1.0):
    #     """
    #     It will set the speed of each of the two proppelers of wamv.
    #     """
    #     # i = 0
    #     # for publisher_object in self.publishers_array:
    #     #     from robotx_gazebo.msg import UsvDrive
    #     #     usv_drive_obj = UsvDrive()
    #     #     usv_drive_obj.right = right_propeller_speed
    #     #     usv_drive_obj.left = left_propeller_speed

    #     #     rospy.logdebug("usv_drive_obj>>"+str(usv_drive_obj))
    #     #     publisher_object.publish(usv_drive_obj)
    #     #     i += 1
   
    #     left_msg, right_msg = Float32(), Float32()
    #     right_msg.data = right_propeller_speed
    #     left_msg.data = left_propeller_speed
    #     left_pub, right_pub = self.publishers_array[0], self.publishers_array[1]    
    #     left_pub.publish(left_msg)
    #     right_pub.publish(right_msg)
    #     self.wait_time_for_execute_movement(time_sleep)
        
    def set_propellers_speed(self, linear_speed, angular_speed, time_sleep=1.0):
        """
        It will set the speed of each of the two proppelers of wamv.
        """
        cmd_pub = self.publishers_array[0]
        course_cmd = Course()
        course_cmd.yaw = angular_speed
        course_cmd.speed = linear_speed
        course_cmd.keep_station = False
        course_cmd.station_yaw = 0.0
        course_cmd.station_dist_x = 0.0
        course_cmd.station_dist_y = 0.0
        cmd_pub.publish(course_cmd)
        self.wait_time_for_execute_movement(time_sleep)

    def wait_time_for_execute_movement(self, time_sleep):
        """
        Because this Wamv position is global, we really dont have
        a way to know if its moving in the direction desired, because it would need
        to evaluate the difference in position and speed on the local reference.
        """
        time.sleep(time_sleep)

    def get_odom(self):
        data = copy.deepcopy(self.odom_data)
        ind = data.name.index('wamv')
        odom = Odometry() 
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'world'
        odom.header = header
        odom.pose.pose = data.pose[ind]
        odom.twist.twist = data.twist[ind]
        self.odom = odom
        return self.odom
    
    def get_obs_list(self):
        data = copy.deepcopy(self.obs_data)
        obs_name = [n for n in data.name if "buoys::polyform_" in n]
        obs_num = len(obs_name)
        obs_list = numpy.zeros((3, obs_num))
        # print('------------------------------------------------------')
        for i in range(obs_num):    
            ind = data.name.index(obs_name[i])
            pose = data.pose[ind]
            # print('ID ', ind)
            # print(pose)
            obs_list[:, i] = numpy.array([pose.position.x, pose.position.y, pose.position.z]) 
        self.obs_list = obs_list
        return self.obs_list

    def reset_obstacle_course(self):
        # obstacle_position = numpy.array([[-19.0, -48.0, 0.0],
        #                                  [-5.0,  -44.0, 0.0],
        #                                  [-19.0, -10.0, 0.0],
        #                                  [-3.0,  -52.0, 0.0],
        #                                  [13.0,  -49.0, 0.0],
        #                                  [28.0,  -30.0, 0.0],
        #                                  [15.0,  -16.0, 0.0],
        #                                  [5.0,   -54.0, 0.0],
        #                                  [-15.0, -37.0, 0.0],
        #                                  [-18.0, -60.0, 0.0],
        #                                  [-7.0,  -12.0, 0.0],
        #                                  [-28.0, -26.0, 0.0], 
        #                                  [29.0,  -52.0, 0.0], 
        #                                  [7.0,   -26.0, 0.0],
        #                                  [17.0,  -35.0, 0.0]])

        obstacle_position = numpy.array([[-2.0, -16.9, 0.0],
                                         [1.0,  -15.3, 0.0],
                                         [22.0, -16.8, 0.0],
                                         [19.0, -16.2, 0.0],
                                         [16.0, -15.1, 0.0],
                                         [-2.0, -33.5, 0.0],
                                         [1.0,  -32.7, 0.0],
                                         [4.0,  -32.3, 0.0],
                                         [7.0,  -32.4, 0.0],
                                         [10.0, -33.8, 0.0],
                                         [13.0, -49.5, 0.0],
                                         [16.0, -49.6, 0.0], 
                                         [19.0, -48.8, 0.0], 
                                         [22.0, -49.3, 0.0],
                                         [-2.0, -48.7, 0.0]])
        
        # obstacle_position = numpy.array([[-1.0, -16.9, 0.0],
        #                                  [3.0,  -15.3, 0.0],
        #                                  [21.0, -16.8, 0.0],
        #                                  [17.0, -16.2, 0.0],
        #                                  [13.0, -15.1, 0.0],
        #                                  [-1.0, -33.5, 0.0],
        #                                  [3.0,  -32.7, 0.0],
        #                                  [7.0,  -32.3, 0.0],
        #                                  [11.0, -32.4, 0.0],
        #                                  [15.0, -33.8, 0.0],
        #                                  [9.0,  -49.5, 0.0],
        #                                  [13.0, -49.6, 0.0], 
        #                                  [17.0, -48.8, 0.0], 
        #                                  [21.0, -49.3, 0.0],
        #                                  [-1.0, -48.7, 0.0]])
        for i in range(15):
            obstacle_link = SetLinkStateRequest()
            obstacle_link.link_state.link_name = "polyform_a7_" + str(i) + "::base_link"
            obstacle_link.link_state.pose.position.x = obstacle_position[i][0]
            obstacle_link.link_state.pose.position.y = obstacle_position[i][1]
            obstacle_link.link_state.pose.position.z = obstacle_position[i][2]
            obstacle_link.link_state.reference_frame = "world"
            result = self.set_state_service(obstacle_link)
        
