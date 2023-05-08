
#!/usr/bin/env python

from lfd_data_generator.simulator import Simulator
from lfd_data_generator.demonstrator import Demonstrator
import rospy
from std_msgs.msg import String

class LfdDataGenerator(object):
    def __init__(self):
        rospy.init_node("lfd_data_generator", anonymous=True)
        self.simulator = Simulator()
        self.demo_generator = Demonstrator()
        
        self.trigger_pub = rospy.Publisher('/trigger', 
                                       String, 
                                       queue_size=10)

    def trigger(self, 
                is_recording: bool,
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
        # Start the simulation
        self.simulator.start_simulation()

        for i in range(0, 10):
            
            # Go to the starting pose
            start_success = self.demo_generator.go_to_start()
            rospy.sleep(3.0)
            
            if start_success: 
                # Generate motion plan to target pose
                plan_success, plan = self.demo_generator.generate_motion_plan_to_target_pose()

                if plan_success:
                    
                    # Start recording
                    self.trigger(is_recording=True)
                        
                    # Execute motion plan to target pose
                    self.demo_generator.execute_motion_plan_to_target_pose(plan)

                    # Stop recording
                    self.trigger(is_recording=False)
        
        self.trigger(is_shutting_down=True)
        rospy.signal_shutdown("Done")
        
if __name__ == "__main__":
    try:
        node = LfdDataGenerator()
        node.run()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
