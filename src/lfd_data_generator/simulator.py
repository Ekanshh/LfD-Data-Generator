#!/usr/bin/env python

import sys

import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import shape_msgs.msg


class Simulator(object):
    """Panda Moveit Simulation"""
    
    def __init__(self):
        
        super(Simulator, self).__init__()
        
        moveit_commander.roscpp_initialize(sys.argv)
        
        # Initialize the move group
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group_name = "panda_arm"
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name)
        
    
    def start_simulation(self):
        """Start the simulation"""   
        rospy.loginfo(f"Adding objects to the scene") 
        # Add objects to the scene
        self.add_wall()
        self.add_peg_object()
        self.attach_obj_to_eff("peg")
        rospy.loginfo(f"Objects added to the scene")
    
        
    def add_collision_object(self, 
                             obj_id, 
                             primitive_type, 
                             dimensions, 
                             pose, 
                             frame_id, 
                             operation):
        """Add a collision object to the scene
        """
        rospy.logdebug(f"Add {obj_id} to the scene")
        obj = moveit_msgs.msg.CollisionObject()
        obj.header.frame_id = frame_id
        obj.id = obj_id
        primitive = shape_msgs.msg.SolidPrimitive()
        primitive.type = primitive_type
        primitive.dimensions = dimensions
        obj.primitives.append(primitive)
        obj.primitive_poses.append(pose)
        obj.operation = operation
        return self.scene.add_object(obj)

    def add_wall(self):
        """Add a wall to the scene
        """
        
        dimensions = [0.1, 1.5, 0.5]
        pose = geometry_msgs.msg.Pose()
        pose.orientation.w = 1.0
        pose.position.x = 0.5
        pose.position.y = 0.0
        pose.position.z = 0.25
        return self.add_collision_object("wall",
                                         shape_msgs.msg.SolidPrimitive().BOX, 
                                         dimensions, pose,
                                         self.move_group.get_planning_frame(),
                                         moveit_msgs.msg.CollisionObject().ADD)

    def add_peg_object(self):
        """Add a peg to the scene
        """
        
        dimensions = [0.20, 0.04]
        pose = geometry_msgs.msg.Pose()
        pose.orientation.w = 1.0
        pose.position.z = 0.2
        return self.add_collision_object("peg",
                                         shape_msgs.msg.SolidPrimitive().CYLINDER, 
                                         dimensions, 
                                         pose, 
                                         self.move_group.get_end_effector_link(), 
                                         moveit_msgs.msg.CollisionObject().ADD)
    
    def wait_for_state_update(self, 
                              object_id, 
                              object_is_known=False, 
                              object_is_attached=False, 
                              timeout=4):
        
        """Wait for the state update
        """
        
        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            attached_objects = self.scene.get_attached_objects([object_id])
            is_attached = len(attached_objects.keys()) > 0
            
            is_known = object_id in self.scene.get_known_object_names()

            if (object_is_attached==is_attached) and (object_is_known==is_known):
                return True

            # Sleep so that we give other threads time on the processor
            rospy.sleep(0.1)
            seconds = rospy.get_time()

        return False
        
    def attach_obj_to_eff(self, 
                          object_id):
        """Attach an object to the robot's end effector
        """
        
        eef_link = self.move_group.get_end_effector_link()
        grasping_group = "panda_hand"
        touch_links = self.robot.get_link_names(group=grasping_group)
        self.scene.attach_box(eef_link, object_id, touch_links=touch_links)
        return self.wait_for_state_update(object_id=object_id, 
                                          object_is_known=True, 
                                          object_is_attached=False, 
                                          timeout=4)