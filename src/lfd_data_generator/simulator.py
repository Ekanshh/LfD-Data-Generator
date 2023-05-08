#!/usr/bin/env python

import rospy
from moveit_commander import RobotCommander, PlanningSceneInterface, MoveGroupCommander
from moveit_msgs.msg import CollisionObject
from geometry_msgs.msg import Pose, Point
from shape_msgs.msg import SolidPrimitive


class Simulator(object):
    """Panda Moveit Simulation"""
    
    def __init__(self):
        """Initialize the node"""
        
        super(Simulator, self).__init__()
        
        # Initialize moveit 
        self.robot = RobotCommander()
        self.scene = PlanningSceneInterface()
        self.group_name = "panda_arm"
        self.move_group = MoveGroupCommander(self.group_name)
        
        # Start the simulation
        self.start_simulation()
        rospy.sleep(0.1)
        
    def start_simulation(self):
        """Start the simulation"""   
        rospy.logdebug(f"Adding objects to the scene...") 
        
        # Add wall to the scene
        self.add_object(obj_id="wall",
                        primitive_type=SolidPrimitive.BOX,
                        dimensions=[0.1, 1.5, 0.5],
                        pose=Pose(position=Point(x=0.5, y=0.0, z=0.25)),
                        frame_id=self.move_group.get_planning_frame(),
                        operation= CollisionObject().ADD)

        # Attach peg to the robot end effector
        peg_obj_id = "peg"
        self.add_object(obj_id=peg_obj_id,
                        primitive_type=SolidPrimitive.CYLINDER,
                        dimensions=[0.20, 0.04],
                        pose=Pose(position=Point(x=0., y=0., z=0.2)),
                        frame_id=self.move_group.get_end_effector_link(),
                        operation=CollisionObject().ADD)
        self.attach_obj_to_eef(obj_id=peg_obj_id)
        
        rospy.logdebug(f"Objects added to the scene.")
    
        
    def add_collision_object(self, 
                             obj_id, 
                             primitive_type, 
                             dimensions, 
                             pose, 
                             frame_id, 
                             operation):
        """Add a collision object to the scene
        """
        rospy.logdebug(f"Adding {obj_id} to the scene...")
        
        obj = CollisionObject()
        obj.header.frame_id = frame_id
        obj.id = obj_id
        primitive = SolidPrimitive()
        primitive.type = primitive_type
        primitive.dimensions = dimensions
        obj.primitives.append(primitive)
        obj.primitive_poses.append(pose)
        obj.operation = operation
        
        return self.scene.add_object(obj)

    def add_object(self, 
                   obj_id: str,
                   primitive_type: SolidPrimitive,
                   dimensions: list,
                   pose: Pose,
                   frame_id: str,
                   operation: CollisionObject):
        
        """Add object to the scene
        """
        return self.add_collision_object(obj_id=obj_id,
                                         primitive_type=primitive_type, 
                                         dimensions=dimensions, 
                                         pose=pose,
                                         frame_id=frame_id,
                                         operation=operation)

    def wait_for_state_update(self, 
                              obj_id, 
                              obj_is_known=False, 
                              obj_is_attached=False, 
                              timeout=4):
        
        """Wait for the state update
        """
        
        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            attached_objects = self.scene.get_attached_objects([obj_id])
            is_attached = len(attached_objects.keys()) > 0
            
            is_known = obj_id in self.scene.get_known_object_names()

            if (obj_is_attached==is_attached) and \
                (obj_is_known==is_known):
                return True

            # Sleep so that we give other threads time on the processor
            rospy.sleep(0.1)
            seconds = rospy.get_time()

        return False
        
    def attach_obj_to_eef(self, 
                          obj_id: str):
        """Attach an object to the robot's end effector
        """
        
        # Get the end effector link
        eef_link = self.move_group.get_end_effector_link()
        # Get the grasping group
        grasping_group = "panda_hand"
        touch_links = self.robot.get_link_names(group=grasping_group)
        # Attach the object
        self.scene.attach_box(eef_link, obj_id, touch_links=touch_links)
        
        return self.wait_for_state_update(obj_id=obj_id, 
                                          obj_is_known=True, 
                                          obj_is_attached=False, 
                                          timeout=10)