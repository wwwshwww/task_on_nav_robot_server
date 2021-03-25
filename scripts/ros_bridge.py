#!/usr/bin/env python

import rospy
import actionlib
import PyKDL
import tf2_ros
from tf_conversions import posemath

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Twist, Pose, Pose2D, PoseStamped
from nav_msgs.msg import Odometry, Path, OccupancyGrid
from visualization_msgs.msg import MarkerArray, Marker
from gazebo_msgs.srv import GetModelState, SetModelState
from gazebo_msgs.msg import ModelState, ContactsState
from std_msgs.msg import String
from std_srvs.srv import Empty

from threading import Event
import copy
import trimesh
import numpy as np
from skimage.measure import block_reduce
from skimage.transform import resize

from robo_gym_server_modules.robot_server.grpc_msgs.python import robot_server_pb2

from roomor.generator import CubeRoomGenerator

PARAM_RESOLUTION = 0.05

class RosBridge:
    
    def __init__(self, real_robot=False, wait_moved=True, action_time=3, slam_map_size=512, agent_size=0.8, wall_threshold=0.01):
        self.gazebo_proxy = self.get_gazebo_proxy()
        
        # Event is clear while initialization or set_state is going on
        self.reset = Event()
        self.reset.clear()
        self.get_state_event = Event()
        self.get_state_event.set()
        
        self.real_robot = real_robot
        self.wait_moved = wait_moved
        self.action_time = action_time
        
        self.move_base_client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        self.mir_exec_path = rospy.Publisher('mir_exec_path', Path, queue_size=10)
        self.target_pub = rospy.Publisher('target_markers', MarkerArray, queue_size=10)
        
        self.slam_reset_pub = rospy.Publisher('/syscommand', String, latch=True, queue_size=1)
        
        if self.real_robot:
            rospy.Subscriber("odom", Odometry, self.callback_odometry, queue_size=1)
        else:
            rospy.Subscriber("odom_comb", Odometry, self.callback_odometry, queue_size=1)
            
        rospy.Subscriber("mir_collision", ContactsState, self.callback_collision)
        rospy.Subscriber("robot_pose", Pose, self.callback_state, queue_size=1)
        rospy.Subscriber("map", OccupancyGrid, self.callback_map, queue_size=1)
        
        self.mir_start_state = ModelState()
        
        # Map info
        self.map_resolution = 0.0
        self.map_origin = Pose()
        
        self.slam_map_size = slam_map_size
        self.map_size = 256
        self.map_data = [0] * (self.map_size**2)
        self.map_data_trueth = [0] * (self.map_size**2)
        self.mir_pose = [0.0] * 3
        self.mir_twist = [0.0] * 2
        self.collision = False
        
        self.room_generator = None
        self.room_config = None
        
        self.room_generator_params = dict()
        
        self.rgp_tags = ['obstacle_count', 'obstacle_size', 'target_size', 'room_length_max', 
                         'room_mass_min', 'room_mass_max', 'room_wall_height', 'room_wall_thickness']
        
        # Room generator parameter
        self.room_generator_params['obstacle_count'] = 10
        self.room_generator_params['obstacle_size'] = 0.7
        self.room_generator_params['target_size'] = 0.2
        self.room_generator_params['room_length_max'] = 9.0
        self.room_generator_params['room_mass_min'] = 20
        self.room_generator_params['room_mass_max'] = 36
        self.room_generator_params['room_wall_height'] = 0.8
        self.room_generator_params['room_wall_thickness'] = 0.05
        
        # parameter as not state
        self.room_generator_params['agent_size'] = agent_size # default 0.35
#         self.room_generator_params['agent_size'] = 0.35
        self.room_generator_params['wall_threshold'] = wall_threshold # default 0.01 
#         self.room_generator_params['wall_threshold'] = 0.01
        
        # Room's infomation
        self.obstacles = [] # [3,n]
        self.targets = [] # [3,n]
        
        self.path_frame = 'map'
        
        if self.real_robot:
            self.path_frame = 'world'

            # Apply transform to center the robot, with real_robot we use World frame,
            # World is the Map frame translated in XY to center robot
            tfBuffer = tf2_ros.Buffer()
            listener = tf2_ros.TransformListener(tfBuffer)

            trans = tfBuffer.lookup_transform("world", "map", rospy.Time(), rospy.Duration(1.0))
            v = PyKDL.Vector(trans.transform.translation.x,trans.transform.translation.y,trans.transform.translation.z)
            r = PyKDL.Rotation.Quaternion(
                trans.transform.rotation.x,
                trans.transform.rotation.y,
                trans.transform.rotation.z,
                trans.transform.rotation.w
            )
            self.world_to_map = PyKDL.Frame(r,v)
        
        # Initialize Path
        self.mir_path = Path()
        self.mir_path.header.stamp = rospy.Time.now()
        self.mir_path.header.frame_id = self.path_frame
        
        self.rate = rospy.Rate(10) #30Hz
        self.reset.set()
        
    def get_state(self):
        self.get_state_event.clear()
        # Get environment state
        state = []
        mir_pose = copy.deepcopy(self.mir_pose)
        mir_twist = copy.deepcopy(self.mir_twist)
        is_collision = copy.deepcopy(self.collision)
        is_change_room = 0
        is_change_mir_pose = 0

        targets = copy.deepcopy(self.targets)
        if len(targets) != 0:
            targets = targets.flatten()
        
        map_size = copy.deepcopy(self.map_size)
        map_data = copy.deepcopy(self.map_data)
        
        map_data_trueth = copy.deepcopy(self.map_data_trueth)
        
        rgp = [copy.deepcopy(self.room_generator_params[tag]) for tag in self.rgp_tags]
        
        self.get_state_event.set()
        
        # Create and fill State message
        msg = robot_server_pb2.State()
        msg.state.extend([map_size])
        msg.state.extend(map_data)
        msg.state.extend(map_data_trueth)
        msg.state.extend(mir_pose)
        msg.state.extend(mir_twist)
        msg.state.extend([is_collision])
        msg.state.extend([is_change_room])
        msg.state.extend([is_change_mir_pose])
        msg.state.extend(rgp)
        msg.state.extend(targets)
        msg.success = 1
        
        return msg
        
    def set_state(self, state_msg):
        state = state_msg.state
        
        self.reset.clear()
        
        self.mir_path = Path()
        self.mir_path.header.stamp = rospy.Time.now()
        self.mir_path.header.frame_id = self.path_frame
        
        self.map_size = int(copy.deepcopy(state[0]))
        
        ignore_index = (self.map_size**2)*2 + 7
        is_change_room = int(state[ignore_index])
        is_change_robot = int(state[ignore_index+1])
        state[ignore_index+2] = int(state[ignore_index+2])
        
        if not self.real_robot:
            if is_change_room:
                same_all = all(
                    [state[ignore_index+i+2] == self.room_generator_params[tag] for i, tag in enumerate(self.rgp_tags)]
                )
                
                if not same_all:
                    # Update parameter what use when generate room
                    for i, tag in enumerate(self.rgp_tags):
                        self.room_generator_params[tag] = copy.deepcopy(state[ignore_index+i+2])
                
                self.room_generator_params['obstacle_count'] = int(copy.deepcopy(state[ignore_index+2]))
                        
                self.room_config = self.gen_simulation_room((self.room_generator is None) or (not same_all))
                self.targets = self._spawnconfig_to_xyr(self.room_config.target_pose)
                self.obstacles = self._spawnconfig_to_xyr(self.room_config.obstacle_pose)
            
            if is_change_robot:
                pos_x, pos_y, ori_z = self.gen_agent_state(
                    self.room_config, 
                    self.room_generator_params['agent_size'],
                    self.room_generator_params['wall_threshold']
                )
                self.mir_pose = [pos_x, pos_y, ori_z]
                self.mir_start_state = self._xyr_to_modelstate(pos_x, pos_y, ori_z)
            
            #### Must use map->odom_comb frame transform to get truethly occupancy grid map
            
            import sys
            try:
                if is_change_room or is_change_robot:
                    px, py, oz = self._modelstate_to_xyr(self.mir_start_state)
                    map_trueth = self.room_config.get_occupancy_grid(
                        freespace_poly=self.room_config.get_freespace_poly(),
                        resolution=self.map_resolution,
                        map_size=int(self.map_size)
                    )
#                     map_trueth_2d = map_trueth.reshape((int(self.map_size),int(self.map_size))).T
#                     map_trueth_2d = np.flipud(map_trueth_2d)
#                     map_trueth_2d = np.fliplr(map_trueth_2d)
#                     self.map_data_trueth = map_trueth_2d.flatten()
                    self.map_data_trueth = map_trueth
            
                self.set_env_state(self.room_config, self.mir_start_state, spawn=is_change_room)

            except Exception as e:
                rospy.loginfo("\n\n{}:{}\n\n".format(type(e),sys.exc_info()))
            
        else:
            target_space = state[ignore_index+2+len(self.rgp_tags):]
            self.targets = np.reshape(target_space, (len(target_space)//3, 3))
            
        if len(self.targets) > 0:
            self.publish_target_markers(self.targets)
            
        rospy.sleep(0.5)
        self.reset_navigation()
        self.reset.set()
        
        return 1
    
    def _spawnconfig_to_xyr(self, spawn_config):
        return np.vstack([
            spawn_config['positions'][:,0], 
            spawn_config['positions'][:,1], 
            spawn_config['orientations'][:,2]
        ]).T
    
    def call_move_base(self, pos_x, pos_y, ori_z):
        import sys
        try:
            self.move_base_client.wait_for_server()
        except:
            rospy.loginfo("\n\n{}\n\n".format(sys.exc_info()))
            
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = pos_x
        goal.target_pose.pose.position.y = pos_y
        ori = PyKDL.Rotation.RPY(0,0, ori_z)
        goal.target_pose.pose.orientation.x, \
            goal.target_pose.pose.orientation.y, \
            goal.target_pose.pose.orientation.z, \
            goal.target_pose.pose.orientation.w = ori.GetQuaternion()
        
        self.move_base_client.send_goal(goal)
        if self.wait_moved:
            wait = self.move_base_client.wait_for_result()
            if not wait:
                rospy.logerr("Action server not available!")
                rospy.signal_shutdown("Action server not available!")
            else:
                return self.move_base_client.get_result()
        else:
            rospy.sleep(self.action_time)
            return self.move_base_client.get_state()
            
    def gen_simulation_room(self, new_generator=False):
        try:
            if new_generator:
                d = copy.deepcopy(self.room_generator_params)
                d.update(self.gazebo_proxy)
                rospy.loginfo("CubeRoomGenerator paramater: {}".format(str(d)))
                self.room_generator = CubeRoomGenerator(**d)
                
            room = self.room_generator.generate_new()
        except Exception as e:
            rospy.loginfo("{} Error occured when generating room: {}".format(type(e), e))
        
        return room
    
    def gen_agent_state(self, room, agent_size, wall_threshold):
        thresh = wall_threshold + agent_size
        obs = room.obstacle_pose['positions'][:,:2]
        freezone = room.get_freezone_poly().buffer(-thresh, cap_style=2, join_style=2)
        ## generate pose that have nothing possibility to touch any obstacle
        poses = trimesh.path.polygons.sample(freezone, 500)
        pos = [0,0]
        for p in poses:
            pp = np.full([len(obs),2], p)
            min_norm = np.min(np.linalg.norm(pp-obs, axis=1))
            if min_norm > agent_size:
                pos = p
                rospy.loginfo("\n generated: {}\n".format(p))
                break
            
        ori_z = np.random.rand()*np.pi*2 - np.pi
        return pos[0], pos[1], ori_z
        
    def _xyr_to_modelstate(self, pos_x, pos_y, ori_z):
        start_state = ModelState()
        start_state.model_name = 'mir'
        start_state.pose.position.x = pos_x
        start_state.pose.position.y = pos_y
        start_state.pose.position.z = self.room_generator_params['obstacle_size']/2
        orientation = PyKDL.Rotation.RPY(0,0, ori_z)
        start_state.pose.orientation.x, \
            start_state.pose.orientation.y, \
            start_state.pose.orientation.z, \
            start_state.pose.orientation.w = orientation.GetQuaternion()
        
        start_state.twist.linear.x = 0.0
        start_state.twist.linear.y = 0.0
        start_state.twist.linear.z = 0.0
        start_state.twist.angular.x = 0.0
        start_state.twist.angular.y = 0.0
        start_state.twist.angular.z = 0.0
        
        return start_state
    
    def _modelstate_to_xyr(self, modelstate):
        return modelstate.pose.position.x, modelstate.pose.position.y, modelstate.pose.orientation.z
    
    def set_env_state(self, room, agent_state, spawn=True):
        # Spawn models to Gazebo world and sleep until it done
        if spawn:
            room.spawn_all()
        
        rospy.wait_for_service('/gazebo/set_model_state')
        try:
            set_model_state_client = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            set_model_state_client(agent_state)
        except rospy.ServiceException as e:
            print("Service call failed:" + e)
            
    def reset_navigation(self, slam_method='hector'):
        if slam_method == 'hector':
            self.slam_reset_pub.unregister()
            self.slam_reset_pub = rospy.Publisher('/syscommand', String, latch=True, queue_size=1)
            self.slam_reset_pub.publish("reset")
        else:
            pass
        
        rospy.wait_for_service('/move_base_node/clear_costmaps')
        try:
            clear_costmap_client = rospy.ServiceProxy('/move_base_node/clear_costmaps', Empty)
            res = clear_costmap_client()
        except rospy.ServiceException as e:
            rospy.loginfo("Service call failed:" + e)
            
        rospy.sleep(2.5)
    
    def publish_target_markers(self, target_poses):
        marker_array = MarkerArray()
        
        for i in range(len(target_poses)):
            m = Marker()
            m.type = 2 # sphere
            m.scale.x = self.room_generator_params['target_size']
            m.scale.y = self.room_generator_params['target_size']
            m.scale.z = self.room_generator_params['target_size']
            m.action = 0
            m.frame_locked = 1
            m.pose.position.x = target_poses[i][0]
            m.pose.position.y = target_poses[i][1]
            m.pose.position.z = 0.0
            rpy_orientation = PyKDL.Rotation.RPY(0,0,target_poses[i][2])
            q_orientation = rpy_orientation.GetQuaternion()
            m.pose.orientation.x = q_orientation[0]
            m.pose.orientation.y = q_orientation[1]
            m.pose.orientation.z = q_orientation[2]
            m.pose.orientation.w = q_orientation[3]
            m.id = i
            m.header.stamp = rospy.Time.now()
            m.header.frame_id = self.path_frame
            m.color.a = 1.0
            m.color.r = 0.0
            m.color.g = 1.0
            m.color.b = 1.0
            
            marker_array.markers.append(m)
            
        self.target_pub.publish(marker_array)
        
    def callback_state(self, data):
        # If state is not being reset proceed otherwise skip callback
        if self.reset.isSet():
            if self.real_robot:
                # Convert Pose from relative to Map to relative to World frame
                f_r_in_map = posemath.fromMsg(data)
                f_r_in_world = self.world_to_map * f_r_in_map
                data = posemath.toMsg(f_r_in_world)

            x = data.position.x
            y = data.position.y

            orientation = PyKDL.Rotation.Quaternion(data.orientation.x,
                                                 data.orientation.y,
                                                 data.orientation.z,
                                                 data.orientation.w)

            euler_orientation = orientation.GetRPY()
            yaw = euler_orientation[2]

            # Append Pose to Path
            stamped_mir_pose = PoseStamped()
            stamped_mir_pose.pose = data
            stamped_mir_pose.header.stamp = rospy.Time.now()
            stamped_mir_pose.header.frame_id = self.path_frame
            self.mir_path.poses.append(stamped_mir_pose)
            self.mir_exec_path.publish(self.mir_path)

            # Update internal Pose variable
            self.mir_pose = copy.deepcopy([x,y,yaw])
        else:
            pass
    
    def callback_odometry(self, data):
        lin_vel = data.twist.twist.linear.x
        ang_vel = data.twist.twist.angular.z

        # Update internal Twist variable
        self.mir_twist = copy.deepcopy([lin_vel, ang_vel])
        
    def callback_map(self, data):
        if self.get_state_event.isSet():
            info = data.info
            dif = self.slam_map_size // self.map_size
            self.map_resolution = info.resolution * dif + PARAM_RESOLUTION
            shape = (self.slam_map_size, self.slam_map_size)
            if self.slam_map_size == self.map_size:
                self.map_data = np.array(data.data)
            else:
#                 self.map_data = np.array(data.data).reshape(shape)[::dif, ::dif].flatten()

#                 tmp = np.array(data.data).reshape(shape)
#                 tmp[tmp<0] = 50
#                 self.map_data = resize(tmp, (self.map_size,self.map_size)).flatten()
            
                self.map_data = block_reduce(np.array(data.data).reshape(shape), (dif,dif), np.max).flatten()
            
            self.map_origin = info.origin
        else:
            pass
        
    def callback_collision(self, data):
        if data.states == []:
            self.collision = False
        else:
            self.collision = True
            
    def get_gazebo_proxy(self):
        ros = rospy.get_master()
        ros_host, ros_port = ros.target._ServerProxy__host.split(":")
#         import os
#         gazebo = os.getenv('GAZEBO_MASTER_URI')
        gazebo = rospy.get_param("/gazebo_uri")
        gazebo_host, gazebo_port = gazebo.split("//")[1].split(":")
        
        rospy.loginfo("ros_host:{}, ros_port:{}, gazebo_host:{}, gazebo_port:{}".format(ros_host, ros_port, gazebo_host, gazebo_port))
        
        return {
            'ros_host': ros_host, 
            'ros_port': int(ros_port), 
            'gazebo_host': gazebo_host,
            'gazebo_port': int(gazebo_port)
        }
        