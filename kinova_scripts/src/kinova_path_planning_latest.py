#! /usr/bin/env python

# Author: Nuha Nishat
# Date: 1/30/20

# Edited by Akshaya Agrawal
# Date: 05/25/21

# Modified By: Ryan Roberts (roberyan@oregonstate.edu)
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
import math
import copy
from tf.transformations import quaternion_from_euler

# move_group_python_interface_tutorial was used as reference

class MoveRobot():
    def __init__(self, planning):
        # Initialize moveit commander and ros node for moveit

        # To read from redirected ROS Topic
        joint_state_topic = ['joint_states:=/j2s7s300/joint_states']
        moveit_commander.roscpp_initialize(joint_state_topic)
        rospy.init_node('move-kinova', anonymous=False)
        moveit_commander.roscpp_initialize(sys.argv)

        # Define robot using RobotCommander. Provided robot info such as
        # kinematic model and current joint state
        self.robot = moveit_commander.RobotCommander()

        # Setting the world
        self.scene = moveit_commander.PlanningSceneInterface()

        # Define the planning group for the arm you are using
        # You can easily look it up on rviz under the MotionPlanning tab
        self.move_group = moveit_commander.MoveGroupCommander("arm")
        self.move_gripper = moveit_commander.MoveGroupCommander("gripper")

        # Set the precision of the robot
        rospy.set_param('/move_group/trajectory_execution/allowed_start_tolerance', 0.01)

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

        self.planning = planning
        self.main()

    def set_planner_type(self, planner_name):
        if planner_name == "RRT":
            self.move_group.set_planner_id("RRTConnectkConfigDefault")
        if planner_name == "RRT*":
            self.move_group.set_planner_id("RRTstarkConfigDefault")
        if planner_name == "PRM*":
            self.move_group.set_planner_id("PRMstarkConfigDefault")

    def go_to_joint_state(self, joint_state):
        joint_goal = JointState()
        joint_goal.position = joint_state
        self.move_group.set_joint_value_target(joint_goal.position)

        self.plan = self.move_group.plan()
        self.move_group.go(wait=True)
        self.move_group.execute(self.plan, wait=True)

        self.move_group.stop()
        self.move_group.clear_pose_targets()
        rospy.sleep(2)

    def go_to_goal(self, ee_pose, f_name = None):
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
        if(self.planning == "1"):
            self.make_yaml(f_name)
        self.move_group.go(wait=True)
        self.move_group.stop()

        self.move_group.clear_pose_targets()
        rospy.sleep(2)

    def move_gripper(self, cmd):
        if cmd == "Close":
            self.move_gripper.set_named_target("Close")
        elif cmd == "Open":
            self.move_gripper.set_named_target("Open")
        else:
            self.move_gripper.set_joint_value_target(cmd)
        self.move_gripper.go(wait=True)
        rospy.sleep(2)

    def display_trajectory(self):
        self.disp_pub = rospy.Publisher("/move_group/display_planned_path", moveit_msgs.msg.DisplayTrajectory,
                                        queue_size=20)
        self.disp.trajectory.append(self.plan)
        print(self.disp.trajectory)
        self.disp_pub.publish(self.disp)

    def go_to_finger_state(self, cmd):
        if cmd == "Close":
            self.move_gripper.set_named_target("Close")
        elif cmd == "Open":
            self.move_gripper.set_named_target("Open")
        else:
            self.move_gripper.set_joint_value_target(cmd)
        self.move_gripper.go(wait=True)
        self.move_gripper.stop()
        self.move_gripper.clear_pose_targets()
        rospy.sleep(2)

    def go_to_finger_joint_state(self, joint_values, f_name = None):
        try:
            gripper_states = JointState()
            gripper_states.position = joint_values
            self.move_gripper.set_joint_value_target(gripper_states.position)
            if(self.planning == "1"):
                self.make_yaml(f_name, 1)
            self.move_gripper.go(wait=True)
            self.move_gripper.stop()
            self.move_gripper.clear_pose_targets()
            rospy.sleep(2)
            return True
        except:
            return False

    def go_to_arm_joint_state(self, joint_values, f_name=None):
        try:
            if(joint_values == "Home"):
                self.move_group.set_named_target("Home")
            elif(joint_values == "Vertical"):
                self.move_group.set_named_target("Vertical")
            else:
                arm_states = JointState()
                arm_states.position = joint_values
                self.move_group.set_joint_value_target(arm_states.position)
            if(self.planning == "1"):
                self.make_yaml(f_name)
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
            handle_mesh = "/home/sogol/kinova_Ws/src/kinova-ros/kinova_description/meshes/drawer_Link.STL"
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
            handle_mesh = "/home/sogol/kinova_Ws/src/kinova-ros/kinova_description/meshes/drawer_Link.STL"
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
            self.scene.add_box('wall_l', wall_pose, (0.3, 0.01, 0.1))
            wall_pose = PoseStamped()
            wall_pose.header.frame_id = self.robot.get_planning_frame()
            wall_pose.pose.position.x = 0.61
            wall_pose.pose.position.y = 0.08
            wall_pose.pose.position.z = 0.92
            self.scene.add_box('wall_r', wall_pose, (0.3, 0.01, 0.1))
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

    def main(self):
        #try:
        #    loaded_plan = open("drawer_planned_paths/point1.txt", "r")
        #    print(loaded_plan.read())
        #except IOError:

        #pick planner
        self.set_planner_type("RRT")
        #open gripper and go to home position
        rospy.loginfo('opening the gripper')
        self.go_to_finger_state('Open')
        rospy.loginfo("going to home state")
        self.go_to_arm_joint_state("Home")
       #home_pose = self.move_group.get_current_pose().pose
        rospy.loginfo("finished going to home state")
        self.yaml_folder = "drawer_planned_paths/" #make user defined
        if(not os.path.exists(self.yaml_folder)):
            os.makedirs(self.yaml_folder)

        if(self.planning == "1"):
            #default height: .855m
            self.build_env(5) #safety walls
            self.build_env(0)
            rospy.loginfo("putting palm to handle [point 1 of 4]")
            current_point = [0.594897268928, (-0.00552424651151 + 0.0275), 1.08080196315, -0.0552241400824, 0.998162456525, -0.0237767228365, -0.0075280931829]
            self.go_to_goal(current_point, "point1.yaml")
            
            #rospy.loginfo("putting palm to handle [point 2 of 4]")
            #current_point[1] = current_point[1] + .0275
            #self.go_to_goal(current_point, "point2.yaml")
            
            rospy.loginfo("putting palm to handle [point 3 of 4]")
            self.go_to_finger_joint_state([0.4, 0.4, 0.4], "point2_gclose.yaml") #close fingers a little
            current_point[0] = current_point[0] + .055
            current_point[2] = current_point[2] - .08 #was .081
            self.go_to_goal(current_point, "point3.yaml")
            
            rospy.loginfo("putting palm to handle [point 4 of 4]")
            current_point[0] = current_point[0] + .055
            current_point[2] = current_point[2] - .08 #was .081
            self.go_to_goal(current_point, "point4.yaml")
            self.teardown_env(0)

            rospy.loginfo("closing gripper")
            self.go_to_finger_joint_state([1, 0.9, 0.9], "close_gripper.yaml") #try [1.1, 0.9, 0.9] in real world
           
            self.build_env(2)
            rospy.loginfo("pulling drawer out [point 1 of 3]")
            current_point[0] = current_point[0] - 0.04 #distance to pull drawer (<=20cm)
            self.go_to_goal(current_point, "pull_drawer1.yaml")
            
            self.build_env(3)
            rospy.loginfo("pulling drawer out [point 2 of 3]")
            current_point[0] = current_point[0] - 0.04 #distance to pull drawer (<=20cm)
            self.go_to_goal(current_point, "pull_drawer2.yaml")
            
            self.build_env(4)
            rospy.loginfo("pulling drawer out [point 3 of 3]")
            current_point[0] = current_point[0] - 0.04 #distance to pull drawer (<=20cm)
            self.go_to_goal(current_point, "pull_drawer3.yaml")
            self.teardown_env(2)

            rospy.loginfo("releasing handle")
            self.go_to_finger_joint_state([0.3, 0.3, 0.3], "release_handle.yaml")
            #self.go_to_finger_state('Open')
           
            self.build_env(1)
            rospy.loginfo("moving gripper from handle (test_point2)")
            test_point = [ 0.55645217799, 0.0265633405959, 1.02602250364, -0.0552636649865, 0.998142031898, -0.0243654184542, -0.00804598493432]
            self.go_to_goal(test_point, "distance_handle.yaml")
           
            rospy.loginfo("going to home state")
            self.go_to_arm_joint_state("Home", "return_home.yaml")
            rospy.loginfo("finished")
            self.teardown_env(1)
            self.teardown_env(3) #safety walls

        elif(self.planning == "0"):
            rospy.loginfo("putting palm to handle [point 1 of 4] (yaml version)")
            self.run_yaml("point1.yaml")
            rospy.loginfo("going to end goal")
            self.run_yaml("point2.yaml")
            rospy.loginfo("putting palm to handle [point 3 of 4]")
            self.run_yaml("point2_gclose.yaml", 1)
            self.run_yaml("point3.yaml")
            rospy.loginfo("putting palm to handle [point 4 of 4]")
            self.run_yaml("point4.yaml")
            rospy.loginfo("closing gripper")
            self.run_yaml("close_gripper.yaml", 1)
            rospy.loginfo("pulling drawer out")
            self.run_yaml("pull_drawer1.yaml")
            self.run_yaml("pull_drawer2.yaml")
            self.run_yaml("pull_drawer3.yaml")
            rospy.loginfo("releasing handle")
            self.run_yaml("release_handle.yaml", 1)
            rospy.loginfo("moving gripper from handle")
            self.run_yaml("distance_handle.yaml")
            rospy.loginfo("going to home state")
            self.run_yaml("return_home.yaml")
            rospy.loginfo("finished")

        else:
            raise IOError("Invalid first argument")
        
if __name__ == '__main__':
    MoveRobot(sys.argv[1])
