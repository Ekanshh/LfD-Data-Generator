
#!/usr/bin/env python

from lfd_data_generator.simulator import Simulator
from lfd_data_generator.demonstrator import Demonstrator
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Pose

class LfdDataGenerator(Demonstrator):
    """Generate demonstrations using Moveit and Panda in a simulated 
       environment"""
    
    def __init__(self):
        
        # Initialize the node
        rospy.init_node("lfd_data_generator", anonymous=True)
        
        super(LfdDataGenerator, self).__init__()
        
        # Define the start and target poses for the demonstration
        self.start_pose = self.create_pose_msg(pos_x=float("-0.21"), pos_y=0.21, pos_z=0.59)
        self.target_pose = self.create_pose_msg(pos_x=0.7, pos_y=0.0, pos_z=0.59)
        
        # Trigger recording
        self.trigger_pub = rospy.Publisher('/trigger', 
                                           String, 
                                           queue_size=10)
        
        self.total_demonstrations = 10
        
    def create_pose_msg(self,
                        pos_x: float,
                        pos_y: float,
                        pos_z: float):
        """Create a Pose message"""
        pose_msg = Pose()
        pose_msg.position.x = pos_x
        pose_msg.position.y = pos_y
        pose_msg.position.z = pos_z
        pose_msg.orientation.x = 1.0
        return pose_msg 
    
    def trigger(self, 
                is_recording: bool = False,
                is_shutting_down: bool = False):
        """Send a trigger to start/stop recording the end effector pose
        """
        
        trigger_msg = String()
        
        if is_recording == True:
            trigger_msg.data = "e_start"
        elif is_recording == False:
            trigger_msg.data = "e_stop"
        elif is_shutting_down == True:
            trigger_msg.data = "e_shutdown"
        else:
            trigger_msg.data = "e_unknown"
            
        rospy.loginfo(f"Sending {trigger_msg.data} trigger...")
        self.trigger_pub.publish(trigger_msg)
        rospy.sleep(0.1)
            
    
    def run(self):
        """Run the demonstration
        """
        print(f"Generating {self.total_demonstrations} demonstrations...")
        
        for i in range(0, self.total_demonstrations):
            
            print(f"Generating demonstration {i+1}...")
            
            # Go to the starting pose
            # Plan
            has_start_plan, start_plan = \
                self.generate_motion_plan(target_pose=self.start_pose)
            rospy.sleep(0.1)
            # Execute
            if has_start_plan:
                _ = self.execute_motion_plan(plan=start_plan)
                rospy.sleep(0.1)
                
                # Go to the target pose
                # Plan
                has_target_plan, plan = \
                    self.generate_motion_plan(target_pose=self.target_pose)
                rospy.sleep(0.1)
                if has_target_plan:
                    # Start recording
                    self.trigger(is_recording=True)
                    # Execute motion plan to target pose
                    _ = self.execute_motion_plan(plan)
                    # Stop recording
                    self.trigger(is_recording=False)
        
        # Shut down the node
        self.trigger(is_shutting_down=True)
        rospy.signal_shutdown("Done")
        
if __name__ == "__main__":
    try:
        node = LfdDataGenerator()
        node.run()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
