#!/usr/bin/env python

import random
from lfd_data_generator.simulator import Simulator

import rospy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import RobotTrajectory

class Demonstrator(Simulator):
    """Generate demonstrations using Moveit and Panda
    """

    def __init__(self):
        """Initialize the node
        """
        super(Demonstrator, self).__init__()
            
    # def go_to_start(self,):
    #     """Go to the start pose
    #     """
        
    #     self.move_group.set_pose_target(self.start_pose)
    #     success = self.move_group.go(wait=True)
    #     self.move_group.stop()
    #     self.move_group.clear_pose_targets()        
    #     return success

    def generate_motion_plan(self, 
                             target_pose: Pose = None,
                             randomize: bool = False):
        """Generate a motion plan to the target pose
        """
        
        self.move_group.set_start_state_to_current_state()
        
        # Randomize the target pose if needed
        if randomize:
            target_pose.position.x += 0.05 * random.uniform(-1, 1)
            target_pose.position.y += 0.05 * random.uniform(-1, 1)
            target_pose.position.z += 0.05 * random.uniform(-1, 1)
        
        # Plan to the target pose
        self.move_group.set_pose_target(target_pose)
        success, plan, _, _ = self.move_group.plan()
        
        return success, plan
                
    def execute_motion_plan(self, 
                            plan: RobotTrajectory = None):
        """Execute the motion plan to the target pose
        """
        success = self.move_group.execute(plan, wait=True)
        
        return success