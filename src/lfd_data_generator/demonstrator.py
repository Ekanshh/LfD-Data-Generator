#!/usr/bin/env python

import random
from lfd_data_generator.simulator import Simulator

import rospy
import geometry_msgs.msg


class Demonstrator(Simulator):
    """Generate demonstrations using Moveit and Panda
    """

    def __init__(self):
        """Initialize the node
        """
        super(Demonstrator, self).__init__()
                
        # Initialize the variables
        self.demon_numb = 0
        self.randomize = False
        self.start_pose = geometry_msgs.msg.Pose()
        self.start_pose.orientation.x = 1.0
        self.start_pose.position.x = -0.21
        self.start_pose.position.y = 0.21
        self.start_pose.position.z = 0.59
        self.target_pose = geometry_msgs.msg.Pose()
        self.target_pose.orientation.x = 1.0
        self.target_pose.position.x = 0.7 
        self.target_pose.position.y = 0.0 
        self.target_pose.position.z = 0.59
        
        
    def go_to_start(self,):
        """Go to the start pose
        """
        
        self.move_group.set_pose_target(self.start_pose)
        success = self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()        
        return success

    def generate_motion_plan_to_target_pose(self):
        """Generate a motion plan to the target pose
        """
        
        self.move_group.set_start_state_to_current_state()
        
        if self.randomize:
            self.target_pose.position.x += 0.05 * random.uniform(-1, 1)
            self.target_pose.position.y += 0.05 * random.uniform(-1, 1)
            self.target_pose.position.z += 0.05 * random.uniform(-1, 1)
            
        self.move_group.set_pose_target(self.target_pose)
        success,plan,_,_ = self.move_group.plan()
        
        return success, plan
                
    def execute_motion_plan_to_target_pose(self, plan):
        """Execute the plan
        """
        
        is_executed = self.move_group.execute(plan, wait=True)
        
        return is_executed
