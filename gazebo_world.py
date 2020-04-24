# coding=utf-8
import time
import rospy
import copy
import tf
import numpy as np

from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from rosgraph_msgs.msg import Clock
from std_srvs.srv import Empty
from std_msgs.msg import Int8
from model.utils import get_init_pose, get_goal_point
from kobuki_msgs.msg import BumperEvent
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import GetModelState


class StageWorld():
    def __init__(self, beam_num, index, num_env):
        self.index = index
        self.num_env = num_env
        node_name = 'StageEnv_' + str(index)
        rospy.init_node(node_name, anonymous=None)

        self.beam_mum = beam_num
        self.laser_cb_num = 0
        self.scan = None

        # used in reset_world
        self.self_speed = [0.0, 0.0]
        self.step_goal = [0., 0.]
        self.step_r_cnt = 0.

        # used in generate goal point
        self.map_size = np.array([8., 8.], dtype=np.float32)  # 20x20m
        self.goal_size = 0.1
        # self.goal_size = 0.5

        self.robot_value = 10.
        self.goal_value = 0.


        # # -----------Publisher and Subscriber-------------
        # cmd_vel_topic = 'robot_' + str(index) + '/cmd_vel'
        # self.cmd_vel = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)

        # cmd_pose_topic = 'robot_' + str(index) + '/cmd_pose'
        # self.cmd_pose = rospy.Publisher(cmd_pose_topic, Pose, queue_size=10)

        # object_state_topic = 'robot_' + str(index) + '/base_pose_ground_truth'
        # self.object_state_sub = rospy.Subscriber(object_state_topic, Odometry, self.ground_truth_callback)

        # laser_topic = 'robot_' + str(index) + '/base_scan'

        # self.laser_sub = rospy.Subscriber(laser_topic, LaserScan, self.laser_scan_callback)

        # odom_topic = 'robot_' + str(index) + '/odom'
        # self.odom_sub = rospy.Subscriber(odom_topic, Odometry, self.odometry_callback)

        # crash_topic = 'robot_' + str(index) + '/is_crashed'
        # self.check_crash = rospy.Subscriber(crash_topic, Int8, self.crash_callback)


        # self.sim_clock = rospy.Subscriber('clock', Clock, self.sim_clock_callback)

        # -----------Publisher and Subscriber 2-------------
        cmd_vel_topic = "/mobile_base/commands/velocity"
        self.cmd_vel = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)

        cmd_pose_topic = 'robot_' + str(index) + '/cmd_pose' # @llx 用于 reset pose
        self.cmd_pose = rospy.Publisher(cmd_pose_topic, Pose, queue_size=10)

        object_state_topic = "/odom"
        self.object_state_sub = rospy.Subscriber(object_state_topic, Odometry, self.ground_truth_callback)

        laser_topic = '/scan'
        self.laser_sub = rospy.Subscriber(laser_topic, LaserScan, self.laser_scan_callback)

        odom_topic = "/odom"
        self.odom_sub = rospy.Subscriber(odom_topic, Odometry, self.odometry_callback)

        # 不准，只会检测前侧的碰撞，替换为 scan
        # crash_topic = '/mobile_base/events/bumper'
        # self.check_crash = rospy.Subscriber(crash_topic, BumperEvent, self.crash_callback)


        self.sim_clock = rospy.Subscriber('clock', Clock, self.sim_clock_callback)

        # -----------Service-------------------
        self.pause_stage = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        self.unpause_stage = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        self.reset_stage = rospy.ServiceProxy('/gazebo/reset_world', Empty)
        # self.get_model_states = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        # import pdb; pdb.set_trace()

        # # Wait until the first callback
        self.speed = None
        self.state = None
        self.speed_GT = None
        self.state_GT = None
        self.is_crashed = False
        # while self.scan is None or self.speed is None or self.state is None\
        #         or self.speed_GT is None or self.state_GT is None or self.is_crashed is None:
        #     pass

        rospy.sleep(1.)
        # # What function to call when you ctrl + c
        # rospy.on_shutdown(self.shutdown)

    def ground_truth_callback(self, GT_odometry):
        Quaternious = GT_odometry.pose.pose.orientation
        Euler = tf.transformations.euler_from_quaternion([Quaternious.x, Quaternious.y, Quaternious.z, Quaternious.w])
        self.state_GT = [GT_odometry.pose.pose.position.x, GT_odometry.pose.pose.position.y, Euler[2]]
        v_x = GT_odometry.twist.twist.linear.x
        v_y = GT_odometry.twist.twist.linear.y
        v = np.sqrt(v_x**2 + v_y**2)
        self.speed_GT = [v, GT_odometry.twist.twist.angular.z]

    # @llx
    def transfer_state(self, pose):
        Quaternious = pose.orientation
        Euler = tf.transformations.euler_from_quaternion([Quaternious.x, Quaternious.y, Quaternious.z, Quaternious.w])
        return [pose.position.x, pose.position.y, Euler[2]]

    # @llx
    def transfer_v(self, twist):
        v_x = twist.linear.x
        v_y = twist.linear.y
        v = np.sqrt(v_x**2 + v_y**2)
        return [v, twist.angular.z]

    def laser_scan_callback(self, scan):
        # import pdb; pdb.set_trace()
        self.scan_param = [scan.angle_min, scan.angle_max, scan.angle_increment, scan.time_increment,
                           scan.scan_time, scan.range_min, scan.range_max]
        self.scan = np.array(scan.ranges)
        self.laser_cb_num += 1

        # @llx detect collision by scan
        if min(self.scan)<0.2:
            print("bumper： ", min(self.scan))
            self.is_crashed = True


    def odometry_callback(self, odometry):
        Quaternions = odometry.pose.pose.orientation
        Euler = tf.transformations.euler_from_quaternion([Quaternions.x, Quaternions.y, Quaternions.z, Quaternions.w])
        self.state = [odometry.pose.pose.position.x, odometry.pose.pose.position.y, Euler[2]]
        self.speed = [odometry.twist.twist.linear.x, odometry.twist.twist.angular.z]

    def sim_clock_callback(self, clock):
        self.sim_time = clock.clock.secs + clock.clock.nsecs / 1000000000.

    def crash_callback(self, flag):
        print("bumper, state: ", flag.bumper, flag.state)
        self.is_crashed = flag.state
        # self.is_crashed = ( flag.state == BumperEvent.PRESSED )

    def get_self_stateGT(self):
        return self.state_GT

    def get_self_speedGT(self):
        return self.speed_GT

    def get_laser_observation(self):
        scan = copy.deepcopy(self.scan)
        # print(scan[:10])
        # import pdb; pdb.set_trace()
        scan[np.isnan(scan)] = 6.0
        scan[np.isinf(scan)] = 6.0
        raw_beam_num = len(scan)
        sparse_beam_num = self.beam_mum
        step = float(raw_beam_num) / sparse_beam_num
        sparse_scan_left = []
        index = 0.
        for x in xrange(int(sparse_beam_num / 2)):
            sparse_scan_left.append(scan[int(index)])
            index += step
        sparse_scan_right = []
        index = raw_beam_num - 1.
        for x in xrange(int(sparse_beam_num / 2)):
            sparse_scan_right.append(scan[int(index)])
            index -= step
        scan_sparse = np.concatenate((sparse_scan_left, sparse_scan_right[::-1]), axis=0)
        return scan_sparse / 6.0 - 0.5


    def get_self_speed(self):
        return self.speed

    def get_self_state(self):
        return self.state

    def get_crash_state(self):
        return self.is_crashed

    def get_sim_time(self):
        return self.sim_time

    def get_local_goal(self):
        # [x, y, theta] = self.get_self_stateGT()
        # [goal_x, goal_y] = self.goal_point
        # local_x = (goal_x - x) * np.cos(theta) + (goal_y - y) * np.sin(theta)
        # local_y = -(goal_x - x) * np.sin(theta) + (goal_y - y) * np.cos(theta)
        # return [local_x, local_y]
        return (40,-1) 

    def reset_world(self):
        self.reset_stage()
        self.self_speed = [0.0, 0.0]
        self.step_goal = [0., 0.]
        self.step_r_cnt = 0.
        self.start_time = time.time()
        self.is_crashed = False
        rospy.sleep(0.5)

    def pause_world(self):
        self.pause_stage()
    
    def unpause_world(self):
        self.unpause_stage()

    def generate_goal_point(self):
        # if self.index > 33 and self.index < 44:
        #     self.goal_point = self.generate_random_goal()
        # else:
        #     self.goal_point = get_goal_point(self.index)
        
        self.goal_point = (40,-1)

        self.pre_distance = 0
        self.distance = copy.deepcopy(self.pre_distance) # @llx why



    def get_reward_and_terminate(self, t):
        terminate = False
        laser_scan = self.get_laser_observation()
        laser_min = np.amin(laser_scan)
        [x, y, theta] = self.get_self_stateGT()
        # print('goal: ', self.goal_point)
        print('x y: ', x,y)
        [v, w] = self.get_self_speedGT()
        # rob_state = self.get_model_states('mobile_base','')
        # position = rob_state.pose
        # print(self.transfer_state(position))
        # twist = rob_state.twist
        # print(self.transfer_v(twist))
        # import pdb;pdb.set_trace()
        
        self.pre_distance = copy.deepcopy(self.distance)
        self.distance = np.sqrt((self.goal_point[0] - x) ** 2 + (self.goal_point[1] - y) ** 2)
        print('distance:', self.distance)

        reward_g = (self.pre_distance - self.distance) * 2.5
        reward_c = 0
        reward_w = 0
        result = 0

        is_crash = self.get_crash_state()

        if self.distance < self.goal_size:
            terminate = True
            reward_g = 15
            result = 'Reach Goal'
        else:
            # reward_g = self.pre_distance - self.distance
            reward_g = self.distance

        if is_crash == 1:
            print('is_crash: ', is_crash)
            terminate = True
            reward_c = -15.
            result = 'Crashed'

        if np.abs(w) >  0.7:
        # if np.abs(w) >  1.05:
            reward_w = np.abs(w)
            print("reward_w: ", reward_w)

        # if t > 2000:
        #     terminate = True
        #     result = 'Time out'
        reward = -0.1* reward_g + reward_c + -0.1 * reward_w

        return reward, terminate, result

    def reset_pose(self):
        # import pdb; pdb.set_trace()
        if self.index > 33 and self.index < 44:
            reset_pose = self.generate_random_pose()
        else:
            reset_pose = get_init_pose(self.index)
        rospy.sleep(0.05)
        self.control_pose(reset_pose)
        [x_robot, y_robot, theta] = self.get_self_stateGT()

        while np.abs(reset_pose[0] - x_robot) > 0.2 or np.abs(reset_pose[1] - y_robot) > 0.2:
            [x_robot, y_robot, theta] = self.get_self_stateGT()
        # rospy.sleep(0.05)


    def control_vel(self, action):
        move_cmd = Twist()
        move_cmd.linear.x = action[0]
        move_cmd.linear.y = 0.
        move_cmd.linear.z = 0.
        move_cmd.angular.x = 0.
        move_cmd.angular.y = 0.
        move_cmd.angular.z = action[1]
        # print(move_cmd)
        self.cmd_vel.publish(move_cmd)


    def control_pose(self, pose):
        pose_cmd = Pose()
        assert len(pose)==3
        pose_cmd.position.x = pose[0]
        pose_cmd.position.y = pose[1]
        pose_cmd.position.z = 0

        qtn = tf.transformations.quaternion_from_euler(0, 0, pose[2], 'rxyz')
        pose_cmd.orientation.x = qtn[0]
        pose_cmd.orientation.y = qtn[1]
        pose_cmd.orientation.z = qtn[2]
        pose_cmd.orientation.w = qtn[3]
        self.cmd_pose.publish(pose_cmd)


    def generate_random_pose(self):
        [x_robot, y_robot, theta] = self.get_self_stateGT()
        x = np.random.uniform(9, 19)
        y = np.random.uniform(0, 1)
        if y <= 0.4:
            y = -(y * 10 + 1)
        else:
            y = -(y * 10 + 9)
        dis_goal = np.sqrt((x - x_robot) ** 2 + (y - y_robot) ** 2)
        while (dis_goal < 7) and not rospy.is_shutdown():
            x = np.random.uniform(9, 19)
            y = np.random.uniform(0, 1)
            if y <= 0.4:
                y = -(y * 10 + 1)
            else:
                y = -(y * 10 + 9)
            dis_goal = np.sqrt((x - x_robot) ** 2 + (y - y_robot) ** 2)
        theta = np.random.uniform(0, 2*np.pi)
        return [x, y, theta]

    def generate_random_goal(self):
        [x_robot, y_robot, theta] = self.get_self_stateGT()
        x = np.random.uniform(9, 19)
        y = np.random.uniform(0, 1)
        if y <= 0.4:
            y = -(y*10 + 1)
        else:
            y = -(y*10 + 9)
        dis_goal = np.sqrt((x - x_robot) ** 2 + (y - y_robot) ** 2)
        while (dis_goal < 7) and not rospy.is_shutdown():
            x = np.random.uniform(9, 19)
            y = np.random.uniform(0, 1)
            if y <= 0.4:
                y = -(y * 10 + 1)
            else:
                y = -(y * 10 + 9)
            dis_goal = np.sqrt((x - x_robot) ** 2 + (y - y_robot) ** 2)
        return [x, y]





