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
import shape_msgs.msg
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
        rospy.set_param('/move_group/trajectory_execution/allowed_start_tolerance', "0.05")
        print("startT: {}".format(rospy.get_param("/move_group/trajectory_execution/allowed_start_tolerance")))

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
            self.move_group.execute(loaded_plan, wait=True)
        elif(group == 1):
            self.move_gripper.execute(loaded_plan, wait=True)
        else:
            raise IOError("invalid group state")

    def main(self):
        #try:
        #    loaded_plan = open("drawer_planned_paths/point1.txt", "r")
        #    print(loaded_plan.read())
        #except IOError:

        #pick planner
        self.set_planner_type("RRT")
        self.yaml_folder = "drawer_planned_paths/" #make user defined
        if(not os.path.exists(self.yaml_folder)):
            os.makedirs(self.yaml_folder)

        print("jointT: {}".format(self.move_group.get_goal_joint_tolerance()))
        print("poseT: {}".format(self.move_group.get_goal_position_tolerance()))
        print("oreT: {}".format(self.move_group.get_goal_orientation_tolerance()))
        print("pathC: {}".format(self.move_group.get_path_constraints()))

        #open gripper and go to home position
        rospy.loginfo('opening the gripper')
        self.go_to_finger_state('Open')
        rospy.loginfo("going to home state")
        self.go_to_arm_joint_state("Home", "home_init.yaml")
        home_pose = self.move_group.get_current_pose().pose
        rospy.loginfo("finished going to home state")

        #rospy.loginfo("running test movement")
        #self.go_to_arm_joint_state([3.1415, 3.1415, 3.1415, 3.1415, 3.1415, 3.1415, 3.1415, 1.2, 1.2, 1.2], "DEL.yaml")
        #rospy.loginfo("finished test")

        if(self.planning == "1"):
            waypoints = []
            pose = geometry_msgs.msg.Pose()
            waypoints.append(self.move_group.get_current_pose().pose)
            pose.position.x = 0.594897268928
            pose.position.y = (-0.00552424651151 + 0.0275)
            pose.position.z = 1.08080196315 
            pose.orientation.x = -0.0552241400824 
            pose.orientation.y = 0.998162456525 
            pose.orientation.z = -0.0237767228365 
            pose.orientation.w = -0.0075280931829
            waypoints.append(copy.deepcopy(pose))
            pose.position.x = pose.position.x + .055
            pose.position.z = pose.position.z - .08 #was .081
            waypoints.append(copy.deepcopy(pose))
            pose.position.x = pose.position.x + .055
            pose.position.z = pose.position.z - .08 #was .081
            waypoints.append(copy.deepcopy(pose))
            (plan, fraction) = self.move_group.compute_cartesian_path(waypoints, 0.01, 0.0)
            print("fraction of cartesian path: {}".format(fraction))
            self.move_group.execute(plan, wait=True)
            self.move_group.stop()
            #self.make_yaml("big_test.yaml")
            self.move_group.clear_pose_targets()
            rospy.sleep(2)
            #close gripper
            self.go_to_finger_joint_state([1, 0.9, 0.9], "close_gripper.yaml") #try [1.1, 0.9, 0.9] in real world
            #open drawer
            waypoints = []
            pose.position.x = pose.position.x - 0.04
            waypoints.append(copy.deepcopy(pose))
            pose.position.x = pose.position.x - 0.04
            waypoints.append(copy.deepcopy(pose))
            pose.position.x = pose.position.x - 0.04
            waypoints.append(copy.deepcopy(pose))
            (plan, fraction) = self.move_group.compute_cartesian_path(waypoints, 0.01, 0.0)
            print("fraction of cartesian path: {}".format(fraction))

            #add path constraint
            pose_constraint = moveit_msgs.msg.PositionConstraint()
            pose_constraint.link_name = self.move_group.get_end_effector_link()
            pose_constraint.target_point_offset.x = pose.position.x
            pose_constraint.target_point_offset.y = pose.position.y
            pose_constraint.target_point_offset.z = pose.position.z
            bound = shape_msgs.msg.SolidPrimitive()
            bound.type = bound.BOX
            bound.dimensions.append(0.2) #x
            bound.dimensions.append(0.1) #y
            bound.dimensions.append(0.0) #z
            pose_constraint.constraint_region = bound
            pose_constraint.weight = 1.0
            constraints = moveit_msgs.msg.Constraints()
            constraints.position_constraints.append(pose_constraint)
            self.move_group.set_path_constraints(constraints)

            self.move_group.execute(plan, wait=True)
            self.move_group.stop()
            self.move_group.clear_pose_targets()
            rospy.sleep(2)
            #release drawer
            self.go_to_finger_joint_state([0.3, 0.3, 0.3], "release_handle.yaml")
            #going to home state 
            pose.position.x = 0.55645217799
            pose.position.y =0.0265633405959
            pose.position.z =1.02602250364
            pose.orientation.x =-0.0552636649865
            pose.orientation.y =0.998142031898
            pose.orientation.z =-0.0243654184542
            pose.orientation.w =-0.00804598493432
            waypoints = []
            waypoints.append(copy.deepcopy(pose))
            #waypoints.append(copy.deepcopy(home_pose))
            (plan, fraction) = self.move_group.compute_cartesian_path(waypoints, 0.01, 0.0)
            print("fraction of cartesian path: {}".format(fraction))
            self.move_group.execute(plan, wait=True)
            self.move_group.stop()
            self.move_group.clear_pose_targets()
            rospy.sleep(2)
            self.go_to_arm_joint_state("Home", "return_home.yaml")
            
            

#            #default height: .855m
#            rospy.loginfo("putting palm to handle [point 1 of 4]")
#            current_point = [0.594897268928, (-0.00552424651151 + 0.0275), 1.08080196315, -0.0552241400824, 0.998162456525, -0.0237767228365, -0.0075280931829]
#            self.go_to_goal(current_point, "point1.yaml")
#            
#            #rospy.loginfo("putting palm to handle [point 2 of 4]")
#            #current_point[1] = current_point[1] + .0275
#            #self.go_to_goal(current_point, "point2.yaml")
#            
#            rospy.loginfo("putting palm to handle [point 3 of 4]")
#            self.go_to_finger_joint_state([0.4, 0.4, 0.4], "point2_gclose.yaml") #close fingers a little
#            current_point[0] = current_point[0] + .055
#            current_point[2] = current_point[2] - .08 #was .081
#            self.go_to_goal(current_point, "point3.yaml")
#            
#            rospy.loginfo("putting palm to handle [point 4 of 4]")
#            current_point[0] = current_point[0] + .055
#            current_point[2] = current_point[2] - .08 #was .081
#            self.go_to_goal(current_point, "point4.yaml")
#            #self.go_to_goal(current_point, "rep.yaml") #repetitive
#           
#            rospy.loginfo("closing gripper")
#            self.go_to_finger_joint_state([1, 0.9, 0.9], "close_gripper.yaml") #try [1.1, 0.9, 0.9] in real world
#           
#            rospy.loginfo("pulling drawer out [point 1 of 3]")
#            current_point[0] = current_point[0] - 0.04 #distance to pull drawer (<=20cm)
#            self.go_to_goal(current_point, "pull_drawer1.yaml")
#            
#            rospy.loginfo("pulling drawer out [point 2 of 3]")
#            current_point[0] = current_point[0] - 0.04 #distance to pull drawer (<=20cm)
#            self.go_to_goal(current_point, "pull_drawer2.yaml")
#            
#            rospy.loginfo("pulling drawer out [point 3 of 3]")
#            current_point[0] = current_point[0] - 0.04 #distance to pull drawer (<=20cm)
#            self.go_to_goal(current_point, "pull_drawer3.yaml")
#           
#            rospy.loginfo("releasing handle")
#            self.go_to_finger_joint_state([0.3, 0.3, 0.3], "release_handle.yaml")
#            #self.go_to_finger_state('Open')
#           
#            rospy.loginfo("moving gripper from handle (test_point2)")
#            test_point = [ 0.55645217799, 0.0265633405959, 1.02602250364, -0.0552636649865, 0.998142031898, -0.0243654184542, -0.00804598493432]
#            self.go_to_goal(test_point, "distance_handle.yaml")
#           
#            rospy.loginfo("going to home state")
#            self.go_to_arm_joint_state("Home", "return_home.yaml")
#            rospy.loginfo("finished")

        elif(self.planning == "0"):
            #self.run_yaml("big_test.yaml")
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
