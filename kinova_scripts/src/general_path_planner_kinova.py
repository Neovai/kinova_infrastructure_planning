#! /usr/bin/env python

# Author: Nuha Nishat
# Date: 1/30/20

# Edited by Akshaya Agrawal
# Date: 05/25/21

# Edited By: Ryan Roberts 
# Email: roberyan@oregonstate.edu
# Date: 09/21

import rospy
import yaml
import sys, os
import math
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
import tf, math
import tf.transformations
import pdb
import csv
import rosnode
import functools
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String

from moveit_commander.conversions import pose_to_list
from moveit_msgs.msg import RobotState, PlanningScene, PlanningSceneComponents, AllowedCollisionEntry, \
    AllowedCollisionMatrix
from moveit_msgs.srv import GetPlanningScene, ApplyPlanningScene
import time
import numpy as np
import copy
from tf.transformations import quaternion_from_euler

# move_group_python_interface_tutorial was used as reference

class MoveRobot():
    def __init__(self, mode, second_arg, csv_out=None):
        
        try:
            self.mode = int(mode)
        except Exception:
            raise IOError("invalid first argument (must be 0 or 1)")
        #read joint angles
        if(self.mode == 0):
            self.csv_name = second_arg
            self.joint_poses = []
            try:
                f = open(self.csv_name, "r")
                f.close()
            except Exception:
                raise IOError("invalid csv file name. Be sure the name includes the correct directory relative to this script")
        #write joint angles
        elif(self.mode == 1):
            self.write_out_dir = "joint_angles/"
            self.csv_name = self.write_out_dir + csv_out
            try:
                self.run_custom = int(second_arg)
            except Exception:
                raise IOError("invalid second argument (must be 0 or 1)")
            if(self.run_custom < 0 or self.run_custom > 1):
                raise IOError("invalid second argument (must be 0 or 1)")
        else:
            raise IOError("invalid first argument (must be 0 or 1)")
        
        # Initialize moveit commander and ros node for moveit
        
        # To read from redirected ROS Topic (Gazebo launch use)
#        joint_state_topic = ['joint_states:=/j2s7s300/joint_states']
#        moveit_commander.roscpp_initialize(joint_state_topic)
#        rospy.init_node('move_kinova', anonymous=False)
#        moveit_commander.roscpp_initialize(sys.argv)
        
        # For real robot launch use
#        joint_state_topic = ['joint_states:=/j2s7s300_driver/out/joint_state']
#        moveit_commander.roscpp_initialize(joint_state_topic)
#        rospy.init_node('move_kinova', anonymous=False)
#        moveit_commander.roscpp_initialize(sys.argv)
        
        # for virtual robot launch use
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_kinova', anonymous=True)

        # Define robot using RobotCommander. Provided robot info such as
        # kinematic model and current joint state
        self.robot = moveit_commander.RobotCommander()

        # Setting the world
        self.scene = moveit_commander.PlanningSceneInterface()

        # Define the planning group for the arm you are using
        # You can easily look it up on rviz under the MotionPlanning tab
        self.move_group = moveit_commander.MoveGroupCommander("arm")
        self.move_gripper = moveit_commander.MoveGroupCommander("gripper")

        # Set the precision of the robot. Doesn't work!
        rospy.set_param('/move_group/trajectory_execution/allowed_start_tolerance', 0.05)

        rospy.wait_for_service("/apply_planning_scene", 10.0)
        rospy.wait_for_service("/get_planning_scene", 10.0)

        self.apply_scene = rospy.ServiceProxy('/apply_planning_scene', ApplyPlanningScene)
        self.get_scene = rospy.ServiceProxy('/get_planning_scene', GetPlanningScene)
        rospy.sleep(2)

        # To see the trajectory
        self.disp = moveit_msgs.msg.DisplayTrajectory()

        self.disp.trajectory_start = self.robot.get_current_state()

        self.rate = rospy.Rate(10)

        self.move_group.allow_replanning(1)
        
        #how much to round joint angles
        self.joint_angle_rounded = 2 

#        #check which controller is running (sim or real)
#        if(rosnode.rosnode_ping("/move_group/fake_controller_joint_states", max_count=10)):
#            self.real_robot = False
#        elif(rosnode.rosnode_ping("/move_group/controller_joint_states", max_count=10)): #guessed on name!
#            self.real_robot = True
#        else:
#            raise Exception("Can't find move_group controller node")
        
        #run main
        self.main()

    def set_planner_type(self, planner_name):
        if planner_name == "RRT":
            self.move_group.set_planner_id("RRTConnectkConfigDefault")
        if planner_name == "RRT*":
            self.move_group.set_planner_id("RRTstarkConfigDefault")
        if planner_name == "PRM*":
            self.move_group.set_planner_id("PRMstarkConfigDefault")

    def go_to_goal(self, ee_pose):
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.position.x = ee_pose[0]
        pose_goal.position.y = ee_pose[1]
        pose_goal.position.z = ee_pose[2]

        if len(ee_pose) == 6:
            quat = tf.transformations.quaternion_from_euler(math.radians(ee_pose[3]), math.radians(ee_pose[4]),
                                                            math.radians(ee_pose[5]))
            pose_goal.orientation.x = quat[0]
            pose_goal.orientation.y = quat[1]
            pose_goal.orientation.z = quat[2]
            pose_goal.orientation.w = quat[3]

        else:
            pose_goal.orientation.x = ee_pose[3]
            pose_goal.orientation.y = ee_pose[4]
            pose_goal.orientation.z = ee_pose[5]
            pose_goal.orientation.w = ee_pose[6]

        self.move_group.set_pose_target(pose_goal)
        self.move_group.set_planning_time(20)
        rospy.sleep(2)
        self.move_group.go(wait=True)
        self.move_group.stop()

        self.move_group.clear_pose_targets()
        rospy.sleep(2)

    def display_trajectory(self):
        self.disp_pub = rospy.Publisher("/move_group/display_planned_path", moveit_msgs.msg.DisplayTrajectory,
                                        queue_size=20)
        self.disp.trajectory.append(self.plan)
        print(self.disp.trajectory)
        self.disp_pub.publish(self.disp)

    def go_to_finger_joint_state(self, joint_values):
        try:
            gripper_states = JointState()
            gripper_states.position = joint_values
            self.move_gripper.set_joint_value_target(gripper_states.position)
            self.move_gripper.go(wait=True)
            self.move_gripper.stop()
            self.move_gripper.clear_pose_targets()
            rospy.sleep(2)
            return True
        except:
            return False

    def go_to_arm_joint_state(self, joint_values):
        try:
            if(joint_values == "Home"):
                self.move_group.set_named_target("Home")
            elif(joint_values == "Vertical"):
                self.move_group.set_named_target("Vertical")
            else:
                arm_states = JointState()
                arm_states.position = joint_values
                self.move_group.set_joint_value_target(arm_states.position)
            self.move_group.go(wait=True)
            self.move_group.stop()
            self.move_group.clear_pose_targets()
            rospy.sleep(2)
            return True
        except:
            return False

    def make_yaml(self, f_name, group = 0):
        if(group == 0):
            current_plan = self.move_group.plan()
        elif(group == 1):
            current_plan = self.move_gripper.plan()
        else:
            raise IOError("invalid group state")
        with open(self.yaml_folder + f_name, "w") as f:
            yaml.dump(current_plan, f, default_flow_style=True)

    def run_yaml(self, f_name, group = 0):
        with open(self.yaml_folder + f_name, "r") as f_open:
            loaded_plan = yaml.load(f_open, Loader=yaml.Loader)
        if(group == 0):
            self.move_group.execute(loaded_plan)
        elif(group == 1):
            self.move_gripper.execute(loaded_plan)
        else:
            raise IOError("invalid group state")
    
    def build_env(self, path):
        if path == 0:
            handle_mesh = "/home/roberyan/kinova_ws/src/kinova-ros/kinova_description/meshes/drawer_Link.STL"
            handle_pose = PoseStamped()
            handle_pose.header.frame_id = self.robot.get_planning_frame()
            handle_pose.pose.position.x = 0.66
            handle_pose.pose.position.y = -0.18
            handle_pose.pose.position.z = 0.67
            self.scene.add_mesh('handle', handle_pose, handle_mesh)
            wall_pose = PoseStamped()
            wall_pose.header.frame_id = self.robot.get_planning_frame()
            wall_pose.pose.position.x = 0.82
            wall_pose.pose.position.y = 0.0
            wall_pose.pose.position.z = 0.85
            self.scene.add_box('drawer_face', wall_pose, (.01, 1.0, 1.0))

        elif path == 1:
            handle_mesh = "/home/roberyan/kinova_ws/src/kinova-ros/kinova_description/meshes/drawer_Link.STL"
            handle_pose = PoseStamped()
            handle_pose.header.frame_id = self.robot.get_planning_frame()
            handle_pose.pose.position.x = 0.66 - 0.12
            handle_pose.pose.position.y = -0.18
            handle_pose.pose.position.z = 0.67
            self.scene.add_mesh('handle', handle_pose, handle_mesh)

        elif path == 2:
            wall_pose = PoseStamped()
            wall_pose.header.frame_id = self.robot.get_planning_frame()
            wall_pose.pose.position.x = 0.77
            wall_pose.pose.position.y = 0.0
            wall_pose.pose.position.z = 0.85
            self.scene.add_box('drawer_face', wall_pose, (.01, 1.0, 1.0))
            wall_pose = PoseStamped()
            wall_pose.header.frame_id = self.robot.get_planning_frame()
            wall_pose.pose.position.x = 0.60
            wall_pose.pose.position.y = 0.0
            wall_pose.pose.position.z = 0.86
            self.scene.add_box('wall_b', wall_pose, (0.8, 0.5, 0.01))
            wall_pose = PoseStamped()
            wall_pose.header.frame_id = self.robot.get_planning_frame()
            wall_pose.pose.position.x = 0.61
            wall_pose.pose.position.y = -0.03
            wall_pose.pose.position.z = 0.92
            self.scene.add_box('wall_l', wall_pose, (0.3, 0.01, 0.2))
            wall_pose = PoseStamped()
            wall_pose.header.frame_id = self.robot.get_planning_frame()
            wall_pose.pose.position.x = 0.61
            wall_pose.pose.position.y = 0.08
            wall_pose.pose.position.z = 0.92
            self.scene.add_box('wall_r', wall_pose, (0.3, 0.01, 0.2))
            wall_pose = PoseStamped()
            wall_pose.header.frame_id = self.robot.get_planning_frame()
            wall_pose.pose.position.x = 0.70
            wall_pose.pose.position.y = 0.00
            wall_pose.pose.position.z = 1.24
            self.scene.add_box('wall_t', wall_pose, (0.1, 0.5, 0.01))

        #update of path 1
        elif path == 3:
            self.scene.remove_world_object('wall_t')
            wall_pose = PoseStamped()
            wall_pose.header.frame_id = self.robot.get_planning_frame()
            wall_pose.pose.position.x = 0.70 - .03
            wall_pose.pose.position.y = 0.00
            wall_pose.pose.position.z = 1.24
            self.scene.add_box('wall_t', wall_pose, (0.1, 0.5, 0.01))
            
        #update of path 1
        elif path == 4:
            self.scene.remove_world_object('wall_t')
            wall_pose = PoseStamped()
            wall_pose.header.frame_id = self.robot.get_planning_frame()
            wall_pose.pose.position.x = 0.70 - .05
            wall_pose.pose.position.y = 0.00
            wall_pose.pose.position.z = 1.24
            self.scene.add_box('wall_t', wall_pose, (0.1, 0.5, 0.01))

        #build safety walls
        elif path == 5:
            wall_pose = PoseStamped()
            wall_pose.header.frame_id = self.robot.get_planning_frame()
            wall_pose.pose.position.x = 0.43
            wall_pose.pose.position.y = -0.53
            wall_pose.pose.position.z = 1.21
            self.scene.add_box('safety_l', wall_pose, (1.50, 0.01, 1.50)) 
            wall_pose = PoseStamped()
            wall_pose.header.frame_id = self.robot.get_planning_frame()
            wall_pose.pose.position.x = 0.43
            wall_pose.pose.position.y = 0.53
            wall_pose.pose.position.z = 1.21
            #self.scene.add_box('safety_r', wall_pose, (1.50, 0.01, 1.50)) 
            wall_pose = PoseStamped()
            wall_pose.header.frame_id = self.robot.get_planning_frame()
            wall_pose.pose.position.x = -0.33
            wall_pose.pose.position.y = 0.22
            wall_pose.pose.position.z = 1.21
            self.scene.add_box('safety_b', wall_pose, (0.01, 1.50, 1.50)) 

        else:
            raise IOError("invalid path")

    def teardown_env(self, path):
        if path == 0:
            self.scene.remove_world_object('drawer_face')
            self.scene.remove_world_object('handle')
        
        elif path == 1:
            self.scene.remove_world_object('handle')
        
        elif path == 2:
            self.scene.remove_world_object('drawer_face')
            self.scene.remove_world_object('wall_l')
            self.scene.remove_world_object('wall_r')
            self.scene.remove_world_object('wall_b')
            self.scene.remove_world_object('wall_t')

        #remove safety walls
        elif path == 3:
            self.scene.remove_world_object('safety_l')
            self.scene.remove_world_object('safety_r')
            self.scene.remove_world_object('safety_b')
        
        else:
            raise IOError("invalid path")

    #captures all current joint values and writes to csv file specified by user
    def capture_joint_pose(self):
        self.current_joint_values = self.move_group.get_current_joint_values()
        gripper_values = self.move_gripper.get_current_joint_values()
        for i in range(len(gripper_values)):
            self.current_joint_values.append(gripper_values[i])
        #round joint values
        for i in range(len(self.current_joint_values)):
            self.current_joint_values[i] = round(float(self.current_joint_values[i]), self.joint_angle_rounded)

    def write_joint_pose(self):
        self.capture_joint_pose()
        with open(self.csv_name, mode="a") as f:
            writer = csv.writer(f, delimiter=",", quotechar="|")
            writer.writerow(self.current_joint_values)

    def read_joint_poses(self):
        with open(self.csv_name, mode="r") as f:
            reader = csv.reader(f, delimiter=" ", quotechar="|")
            for row in reader:
                parsed_row = row[0].split(",")
                parsed_row = [float(v) for v in parsed_row]
                self.joint_poses.append(copy.deepcopy(parsed_row))

    def execute_joint_poses(self):
        #01:18 old
        #00:56 new
        self.read_joint_poses()
        num_poses = len(self.joint_poses)
        for i in range(num_poses):
            rospy.loginfo("Moving to joint pose {} out of {}".format((i + 1), num_poses))
            next_arm_joint_angles = self.joint_poses[i][:7]
            next_gripper_joint_angles = self.joint_poses[i][7:10]
            self.capture_joint_pose()
            current_arm_joint_angles = self.current_joint_values[:7]
            current_gripper_joint_angles = self.current_joint_values[7:10]
            #compare current and next joint lists. If same, don't set new goal, otherwise set and go to new joint goals
            if(not (functools.reduce(lambda x,y : x and y, map(lambda p,q : p == q, current_arm_joint_angles,next_arm_joint_angles), True))):
                self.go_to_arm_joint_state(next_arm_joint_angles)
            if(not (functools.reduce(lambda x,y : x and y, map(lambda p,q : p == q, current_gripper_joint_angles,next_gripper_joint_angles), True))):
                self.go_to_finger_joint_state(next_gripper_joint_angles)

    def main(self):
        self.set_planner_type("RRT")

        #run loaded joint angles
        if(self.mode == 0):
            self.execute_joint_poses()
        
        #record joint angles
        elif(self.mode == 1):
            if(not os.path.exists(self.write_out_dir)):
                os.makedirs(self.write_out_dir)

            if(self.run_custom):
                #write custom path here
                
                rospy.loginfo('opening the gripper')
                self.go_to_finger_joint_state('Open')
                rospy.loginfo("going to home state")
                self.go_to_arm_joint_state("Home")
                rospy.loginfo("finished going to home state")
                self.write_joint_pose()

                self.build_env(5) #safety walls
                self.build_env(0)
                rospy.loginfo("putting palm to handle [point 1 of 4]")
                current_point = [0.594897268928, (-0.00552424651151 + 0.0275), 1.08080196315, -0.0552241400824, 0.998162456525, -0.0237767228365, -0.0075280931829]
                self.go_to_goal(current_point)
                self.write_joint_pose()
                
                rospy.loginfo("putting palm to handle [point 3 of 4]")
                self.go_to_finger_joint_state([0.4, 0.4, 0.4]) #close fingers a little
                current_point[0] = current_point[0] + .055
                current_point[2] = current_point[2] - .08 #was .081
                self.go_to_goal(current_point)
                self.write_joint_pose()
                
                rospy.loginfo("putting palm to handle [point 4 of 4]")
                current_point[0] = current_point[0] + .055
                current_point[2] = current_point[2] - .08 #was .081
                self.go_to_goal(current_point)
                self.teardown_env(0)
                self.write_joint_pose()

                rospy.loginfo("closing gripper")
                self.go_to_finger_joint_state([1, 0.9, 0.9]) #try [1.1, 0.9, 0.9] in real world
                self.write_joint_pose()
               
                self.build_env(2)
                rospy.loginfo("pulling drawer out [point 1 of 3]")
                current_point[0] = current_point[0] - 0.04 #distance to pull drawer (<=20cm)
                self.go_to_goal(current_point)
                self.write_joint_pose()
                
                self.build_env(3)
                rospy.loginfo("pulling drawer out [point 2 of 3]")
                current_point[0] = current_point[0] - 0.04 #distance to pull drawer (<=20cm)
                self.go_to_goal(current_point)
                self.write_joint_pose()
                
                self.build_env(4)
                rospy.loginfo("pulling drawer out [point 3 of 3]")
                current_point[0] = current_point[0] - 0.04 #distance to pull drawer (<=20cm)
                self.go_to_goal(current_point)
                self.teardown_env(2)
                self.write_joint_pose()

                rospy.loginfo("releasing handle")
                self.go_to_finger_joint_state([0.3, 0.3, 0.3])
                self.write_joint_pose()
               
                self.build_env(1)
                rospy.loginfo("moving gripper from handle (test_point2)")
                test_point = [ 0.55645217799, 0.0265633405959, 1.02602250364, -0.0552636649865, 0.998142031898, -0.0243654184542, -0.00804598493432]
                self.go_to_goal(test_point)
                self.write_joint_pose()
               
                rospy.loginfo("going to home state")
                self.go_to_arm_joint_state("Home")
                rospy.loginfo("finished")
                self.teardown_env(1)
                self.teardown_env(3) #safety walls
                self.write_joint_pose()

            #capture joint poses on command
            #todo: make sure this works with capturing real robot joint angles
            else:
                while(True):
                    user_in = raw_input("Enter 0 to record current joint angles. Enter 1 to exit (saves automatically): ")
                    if(user_in == "1"):
                        break
                    elif(user_in == "0"):
                        self.write_joint_pose()
                    else:
                        print("Invalid user input")
                pass


"""
First arg: mode 
    - read joint poses (0)
    - capture joint poses (1)

read joint poses:
    Second arg: 
        - name of csv file containing joint poses

capture joint poses:
    Second arg: 
        - move end effector in rviz and capture joint angles on command (0)
        - run custom code (1)
    Third arg:
        - name of csv file to be written to

Notes:
    - gripper joint values will return as 0.0 until they are actually moved for the first time
"""
if __name__ == '__main__':
    if(len(sys.argv) == 3):
        MoveRobot(sys.argv[1], sys.argv[2])
    elif(len(sys.argv) == 4):
        MoveRobot(sys.argv[1], sys.argv[2], sys.argv[3])
    else:
        raise IOError("Invalid number of arguments")
