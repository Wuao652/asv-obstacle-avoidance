import rospy
import numpy
from gym import spaces
from openai_ros.robot_envs import wamv_robot_env
from gym.envs.registration import register
from geometry_msgs.msg import Point
from geometry_msgs.msg import Vector3
from tf.transformations import euler_from_quaternion
from openai_ros.task_envs.task_commons import LoadYamlFileParamsTest
from openai_ros.openai_ros_common import ROSLauncher
from scipy.spatial.transform import Rotation as R
import copy
import os

class ObstacleAvoidanceEnv(wamv_robot_env.WamvEnv):
    def __init__(self):
        """
        Make Wamv learn how to move straight from The starting point
        to a desired point inside the designed corridor.
        Demonstrate Navigation Control
        """
        print('Start to loading the Wamv task Environment!')
        # This is the path where the simulation files, the Task and the Robot gits will be downloaded if not there
        ros_ws_abspath = rospy.get_param("/wamv/ros_ws_abspath", None)
       
        assert ros_ws_abspath is not None, "You forgot to set ros_ws_abspath in your yaml file of your main RL script. Set ros_ws_abspath: \'YOUR/SIM_WS/PATH\'"
        assert os.path.exists(ros_ws_abspath), "The Simulation ROS Workspace path " + ros_ws_abspath + \
                                               " DOESNT exist, execute: mkdir -p " + ros_ws_abspath + \
                                               "/src;cd " + ros_ws_abspath + ";catkin_make"
        
        ROSLauncher(rospackage_name="vrx_gazebo",
                    launch_file_name="wuao_obstacle_avoidance.launch",
                    ros_ws_abspath=ros_ws_abspath)
        print('Successfully load the world!!')
        
        # Load Params from the desired Yaml file
        LoadYamlFileParamsTest(rospackage_name="openai_ros",
                               rel_path_from_package_to_file="src/openai_ros/task_envs/wamv2/config",
                               yaml_file_name="wamv_nav_obstacles.yaml")

        print('Good After Load YamlFile!')
        # Here I temporarily comment the RobotEnvironment!
        
        # Here we will add any init functions prior to starting the MyRobotEnv
        print('Start to load the robot model!')
        print('path: ', ros_ws_abspath)
        super(ObstacleAvoidanceEnv, self).__init__(ros_ws_abspath)

        # Only variable needed to be set here

        rospy.loginfo("Start WamvNavEnv INIT...")
        # We set the action space as continuous space!
        # number_actions = rospy.get_param('/wamv/n_actions')
        # self.action_space = spaces.Discrete(number_actions)

    


        # Test if we can get the param from the original launch files
        self.control_type = rospy.get_param('/wamv/control_type')
        print('The current control type is ', self.control_type)
        
        # Actions and Observations
        self.propeller_high_speed = rospy.get_param('/wamv/propeller_high_speed')
        self.propeller_low_speed = rospy.get_param('/wamv/propeller_low_speed')
        self.max_angular_speed = rospy.get_param('/wamv/max_angular_speed')
        self.max_distance_from_des_point = rospy.get_param('/wamv/max_distance_from_des_point')
        
        self.angular_high_speed = rospy.get_param('/wamv/angular_high_speed')
        self.angular_low_speed = rospy.get_param('/wamv/angular_low_speed')

        self.desired_point_epsilon = rospy.get_param("/wamv/desired_point_epsilon")
        self.desired_collison_epsilon = rospy.get_param("/wamv/desired_collison_epsilon")
        
        # Get Desired Point to Get
        self.desired_point = Point()
        self.desired_point.x = rospy.get_param("/wamv/desired_point/x")
        self.desired_point.y = rospy.get_param("/wamv/desired_point/y")
        self.desired_point.z = rospy.get_param("/wamv/desired_point/z")
        

        self.work_space_x_max = rospy.get_param("/wamv/work_space/x_max")
        self.work_space_x_min = rospy.get_param("/wamv/work_space/x_min")
        self.work_space_y_max = rospy.get_param("/wamv/work_space/y_max")
        self.work_space_y_min = rospy.get_param("/wamv/work_space/y_min")

        self.dec_obs = rospy.get_param("/wamv/number_decimals_precision_obs")

        # We place the Maximum and minimum values of actions
        self.action_space = spaces.Box(numpy.array([self.propeller_low_speed, self.angular_low_speed]), 
                                    numpy.array([self.propeller_high_speed, self.angular_high_speed]))
        # We place the Maximum and minimum values of observations

        high = numpy.array([self.work_space_x_max,
                            self.work_space_y_max,
                            1.57,
                            1.57,
                            3.14,
                            self.propeller_high_speed,
                            self.propeller_high_speed,
                            self.max_angular_speed,
                            self.max_distance_from_des_point
                            ])
        high = numpy.hstack([high, 
                             numpy.full((30, ), 60.)
                            ])

        low = numpy.array([ self.work_space_x_min,
                            self.work_space_y_min,
                            -1*1.57,
                            -1*1.57,
                            -1*3.14,
                            -1*self.propeller_high_speed,
                            -1*self.propeller_high_speed,
                            -1*self.max_angular_speed,
                            0.0
                            ])
        low = numpy.hstack([low, 
                            numpy.full((30, ), -60.)
                            ])

        self.observation_space = spaces.Box(low, high)

        rospy.loginfo("ACTION SPACES TYPE===>"+str(self.action_space))
        rospy.loginfo("OBSERVATION SPACES TYPE===>"+str(self.observation_space))

        # Rewards
        # We set the reward range, which is not compulsory but here we do it.
        # reward consist of 3 parts, R_done, R_collision and R_close_to_target
        self.reward_range = (-numpy.inf, numpy.inf)
        
        self.done_reward =rospy.get_param("/wamv/done_reward")
        self.collision_reward = rospy.get_param("/wamv/collision_reward")
        self.coeff= rospy.get_param("/wamv/closer_to_point_coeff")
        self.closer_to_point_reward = rospy.get_param("/wamv/closer_to_point_reward")
        self.cumulated_reward = 0.0
        self.cumulated_steps = 0.0

        rospy.logdebug("END WamvNavobstaclesEnv INIT...")

    def _set_init_pose(self):
        """
        Sets the two proppelers speed to 0.0 and waits for the time_sleep
        to allow the action to be executed
        """

        right_propeller_speed = 0.0
        left_propeller_speed = 0.0
        self.set_propellers_speed(  right_propeller_speed,
                                    left_propeller_speed,
                                    time_sleep=1.0)

        return True
    
    def set_target(self, x, y, z):
        self.desired_point = Point()
        self.desired_point.x = x
        self.desired_point.y = y
        self.desired_point.z = z

    def _init_env_variables(self):
        """
        Inits variables needed to be initialised each time we reset at the start
        of an episode.
        :return:
        """
        self.reset_obstacle_course()
        # For Info Purposes
        self.cumulated_reward = 0.0
        self.cumulated_steps = 0.0
        self.has_collision = False
        # We get the initial pose to mesure the distance from the desired point.
        odom = self.get_odom()
        current_position = Vector3()
        current_position.x = odom.pose.pose.position.x
        current_position.y = odom.pose.pose.position.y
        self.previous_distance_from_des_point = self.get_distance_from_desired_point(current_position)



    def _set_action(self, action):
        """
        It sets the cmds of wamv based on the action given.
        :param action: tuple(2, ) the action movement to do next [left, right]
        """
        
        rospy.logwarn("Start Set Action ==>"+str(action))
        angular_speed = action[1]
        linear_speed = action[0]
        self.set_propellers_speed(  linear_speed,
                                    angular_speed,
                                    time_sleep=1.0)

        rospy.logdebug("END Set Action ==>"+str(action))
        # rospy.logdebug("Start Set Action ==>"+str(action))
        # right_propeller_speed = action[1]
        # left_propeller_speed = action[0]
        # self.set_propellers_speed(  right_propeller_speed,
        #                             left_propeller_speed,
        #                             time_sleep=1.0)

        # rospy.logdebug("END Set Action ==>"+str(action))
    
    
    # def _set_action(self, action):
    #     """
    #     It sets the joints of wamv based on the action integer given
    #     based on the action number given.
    #     :param action: The action integer that sets what movement to do next.
    #     """

    #     rospy.logdebug("Start Set Action ==>"+str(action))
    #     print(f'the current action is {action}')

    #     right_propeller_speed = 0.0
    #     left_propeller_speed = 0.0

    #     if action == 0: # Go Forwards
    #         right_propeller_speed = self.propeller_high_speed
    #         left_propeller_speed = self.propeller_high_speed
    #     elif action == 1: # Go BackWards
    #         right_propeller_speed = -1*self.propeller_high_speed
    #         left_propeller_speed = -1*self.propeller_high_speed
    #     elif action == 2: # Turn Left
    #         right_propeller_speed = self.propeller_high_speed
    #         left_propeller_speed = -1*self.propeller_high_speed
    #     elif action == 3: # Turn Right
    #         right_propeller_speed = -1*self.propeller_high_speed
    #         left_propeller_speed = self.propeller_high_speed


    #     # We tell wamv the propeller speeds
    #     self.set_propellers_speed(  right_propeller_speed,
    #                                 left_propeller_speed,
    #                                 time_sleep=1.0)

    #     rospy.logdebug("END Set Action ==>"+str(action))

    def _get_obs(self):
        """
        Here we define what sensor data defines our robots observations
        To know which Variables we have access to, we need to read the
        WamvEnv API DOCS.
        :return: observation: tuple(9, ) [x, y, roll, pitch, yaw, Vx, Vy, w, d]
        + tuple(30, ) [Ox1, Oy1, Ox2, Oy2 ..., Ox15, Oy15]
        """
        rospy.logdebug("Start Get Observation ==>")
        self.cumulated_steps += 1
        odom = self.get_odom()
        obs_list = self.get_obs_list()
        # print('self.ob_list: ', obs_list)
        
        base_position = odom.pose.pose.position
        base_orientation_quat = odom.pose.pose.orientation
        base_roll, base_pitch, base_yaw = self.get_orientation_euler(base_orientation_quat)
        base_speed_linear = odom.twist.twist.linear
        base_speed_angular_yaw = odom.twist.twist.angular.z

        distance_from_desired_point = self.get_distance_from_desired_point(base_position)
        # TODO
        obs_list_robot = self.get_obstacles_for_robot(base_position, base_orientation_quat, obs_list)
        # distance_from_obstacles = self.get_distances_from_obstacle(base_position, obs_list)
        # distance_from_obstacles = numpy.round_(distance_from_obstacles, self.dec_obs)
        
        observation = []
        observation.append(round(base_position.x,self.dec_obs))
        observation.append(round(base_position.y,self.dec_obs))
        observation.append(round(base_roll,self.dec_obs))
        observation.append(round(base_pitch,self.dec_obs))
        observation.append(round(base_yaw,self.dec_obs))
        observation.append(round(base_speed_linear.x,self.dec_obs))
        observation.append(round(base_speed_linear.y,self.dec_obs))
        observation.append(round(base_speed_angular_yaw,self.dec_obs))
        observation.append(round(distance_from_desired_point,self.dec_obs))
        
        # change the observation to numpy array
        observation = numpy.array(observation)
        # observation = numpy.hstack((observation, distance_from_obstacles))
        observation = numpy.hstack((observation, obs_list_robot))
        # print("robot pos: ", base_position.x, base_position.y, base_position.z)
        # print("obstacle list:")
        # print(obs_list)
        # print('obstacles in robot frame:')
        # print(obs_list_robot)
        rospy.logdebug("END get Observation ==>"+str(observation))
        return observation


    def _is_done(self, observations):
        """
        We consider the episode done if:
        1) The wamvs is ouside the workspace
        2) It got to the desired point
        3) The robot collide with the obstacles
        """
        distance_from_desired_point = observations[8]

        current_position = Vector3()
        current_position.x = observations[0]
        current_position.y = observations[1]

        # print(f"The obstacles:", observations[9:])
        
        # print(f'The possible collision ({observations[25]}, {observations[26]})')
        is_inside_corridor = self.is_inside_workspace(current_position)
        has_reached_des_point = self.is_in_desired_position(current_position, self.desired_point_epsilon)
        has_collision = self.is_collide_with_obstacles(observations, self.desired_collison_epsilon)
        # oversteplimit = self.cumulated_steps >= 200
        
        done = not(is_inside_corridor) or has_reached_des_point or has_collision

        return done

    def _compute_reward(self, observations, done):
        """
        We Base the rewards in if its done or not and we base it on
        if the distance to the desired point has increased or not
        :return:
        """

        # We only consider the plane, the fluctuation in z is due mainly to wave
        current_position = Point()
        current_position.x = observations[0]
        current_position.y = observations[1]

        distance_from_des_point = self.get_distance_from_desired_point(current_position)
        distance_difference =  distance_from_des_point - self.previous_distance_from_des_point
        
        heading_from_des_point = numpy.arctan2(self.desired_point.y - current_position.y, self.desired_point.x - current_position.x)
        heading_difference = numpy.abs(self.wrap2Pi(heading_from_des_point - observations[4]))
        
        if not done:
            if distance_difference < 0.0:
                rospy.logwarn("DECREASE IN DISTANCE GOOD")
                pass
            else:
                rospy.logerr("ENCREASE IN DISTANCE BAD")
                pass
            reward = -1.0 * self.coeff * distance_difference - 10.0 * heading_difference
            print(f"distance reward :{reward}, heading difference :{heading_difference}")
        else:
            # is done
            has_collision = self.is_collide_with_obstacles(observations, self.desired_collison_epsilon)
            if has_collision:
                rospy.logwarn("ROBOT HAS COLLISION")
                reward = self.collision_reward - self.done_reward
            elif self.is_in_desired_position(current_position, self.desired_point_epsilon):
                rospy.logwarn("ROBOT ARRIVED AT THE TARGET")
                reward = self.done_reward
            else:
                reward = -1.0 * self.done_reward
                
        # if not done:

        #     # If there has been a decrease in the distance to the desired point, we reward it
        #     if distance_difference < 0.0:
        #         rospy.logwarn("DECREASE IN DISTANCE GOOD")
        #         reward = self.closer_to_point_reward
        #     else:
        #         rospy.logerr("ENCREASE IN DISTANCE BAD")
        #         reward = -1*self.closer_to_point_reward

        # else:

        #     if self.is_in_desired_position(current_position, self.desired_point_epsilon):
        #         reward = self.done_reward
        #     else:
        #         reward = -1*self.done_reward


        self.previous_distance_from_des_point = distance_from_des_point


        rospy.logdebug("reward=" + str(reward))
        self.cumulated_reward += reward
        rospy.logdebug("Cumulated_reward=" + str(self.cumulated_reward))
        # self.cumulated_steps += 1
        rospy.logdebug("Cumulated_steps=" + str(self.cumulated_steps))

        return reward


    # Internal TaskEnv Methods

    def is_in_desired_position(self,current_position, epsilon=0.05):
        """
        It return True if the current position is similar to the desired poistion
        """
        distance = self.get_distance_from_desired_point(current_position)
        return distance <= epsilon
    # Previous check collision
    # def is_collide_with_obstacles(self, observations, epsilon=0.05):
    #     distances = observations[9:]
    #     return numpy.any(distances <= epsilon)
    #     pass
    
    # Here check collision with geometric constraint!
    def is_collide_with_obstacles(self, observations, epsilon=0.05):
        '''
        Robot.bottom > Ob.top
        -Robot.top > -Ob.bottom
        -Robot.left > -Ob.right
        Robot.right > Ob.left
        '''
        RobotTop = 3.0
        RobotBottom = -3.0
        RobotRight = -1.4
        RobotLeft = 1.4
        ob_radius = 0.5 + epsilon
        
        ob_pos = observations[9:].reshape((-1, 2)) #15*2
        ob_num = ob_pos.shape[0]
        robotconfig = numpy.array([[RobotBottom],[-RobotTop], [-RobotLeft],[RobotRight]])
        robotboundary = numpy.tile(robotconfig, (1, ob_num))
        ob_pos = ob_pos.T # 2*15
        ob_x, ob_y = ob_pos[0, :], ob_pos[1, :]
        obboundary = numpy.zeros_like(robotboundary) #4*15
        obboundary[0, :] = ob_x + ob_radius
        obboundary[1, :] = -1.0 * (ob_x - ob_radius)
        obboundary[2, :] = -1.0 * (ob_y - ob_radius)
        obboundary[3, :] = ob_y + ob_radius    
        cond = robotboundary > obboundary
        cond = numpy.sum(cond, axis=0)  
        self.has_collision = numpy.any(cond == 0)
        return numpy.any(cond == 0)
        pass

    def get_distance_from_desired_point(self, current_position):
        """
        Calculates the distance from the current position to the desired point
        :param start_point:
        :return:
        """
        distance = self.get_distance_from_point(current_position,
                                                self.desired_point)

        return distance

    def get_distance_from_point(self, pstart, p_end):
        """
        Given a Vector3 Object, get distance from current position
        :param p_end:
        :return:
        """
        a = numpy.array((pstart.x, pstart.y, pstart.z))
        b = numpy.array((p_end.x, p_end.y, p_end.z))

        distance = numpy.linalg.norm(a - b)

        return distance
    def get_distances_from_obstacle(self, pos, obs_list):
        pos_ = numpy.array((pos.x, pos.y, pos.z)) # (3, )
        #obs_list (3, 15)
        distances = numpy.sum(numpy.square(obs_list - pos_.reshape((-1, 1))), axis=0)
        return numpy.sqrt(distances)
        pass

    def get_obstacles_for_robot(self, base_position, base_orientation_quat, obs_list):
        orientation_list = [base_orientation_quat.x,
                            base_orientation_quat.y,
                            base_orientation_quat.z,
                            base_orientation_quat.w]
        roll, pitch, yaw = euler_from_quaternion(orientation_list)
        rot = R.from_quat(numpy.array(orientation_list))
        # 3*3 rotation matrix
        trans = numpy.array([base_position.x, base_position.y])
        T = numpy.eye(3)
        T[0:2, 0:2] = R.as_dcm(rot)[0:2, 0:2]
        T[0:2, 2] = trans
        
        # robot coordinate to the world coordinate
        obstacle_pos_world = copy.deepcopy(obs_list)
        obstacle_pos_world[-1, :] = 1.0
        
        obstacle_pos_robot = numpy.linalg.inv(T) @ obstacle_pos_world
        obstacle_pos_robot_ = obstacle_pos_robot[0:2, :].T 
        # print('compute get ob in loop')
        # print(obstacle_pos_world_)
        return obstacle_pos_robot_.reshape(-1)
        pass


    def get_orientation_euler(self, quaternion_vector):
        # We convert from quaternions to euler
        orientation_list = [quaternion_vector.x,
                            quaternion_vector.y,
                            quaternion_vector.z,
                            quaternion_vector.w]

        roll, pitch, yaw = euler_from_quaternion(orientation_list)
        return roll, pitch, yaw

    def is_inside_workspace(self,current_position):
        """
        Check if the Wamv is inside the Workspace defined
        """
        is_inside = False

        # rospy.logwarn("##### INSIDE WORK SPACE? #######")
        rospy.logwarn(f'XYZ current_position: ({current_position.x}, {current_position.y}, {current_position.z})')
        # rospy.logwarn("XYZ current_position: (", current_position.x, )
        # rospy.logwarn(str(current_position))
        
        # rospy.logwarn("work_space_x_max"+str(self.work_space_x_max)+",work_space_x_min="+str(self.work_space_x_min))
        # rospy.logwarn("work_space_y_max"+str(self.work_space_y_max)+",work_space_y_min="+str(self.work_space_y_min))
        # rospy.logwarn("############")

        
        # if current_position.x > self.work_space_x_min and current_position.x <= self.work_space_x_max:
        #     if current_position.y > self.work_space_y_min and current_position.y <= self.work_space_y_max:
        #             is_inside = True

        # return is_inside
        
        min_x_, max_x_ = self.work_space_x_min - current_position.x, self.work_space_x_max - current_position.x
        min_y_, max_y_ = self.work_space_y_min - current_position.y, self.work_space_y_max - current_position.y
        
        return min_x_ <= 0 and max_x_ >=0 and min_y_ <= 0 and max_y_ >= 0
    
    def wrap2Pi(self, input):
        phases =  (( -input + numpy.pi) % (2.0 * numpy.pi ) - numpy.pi) * -1.0
        return phases



