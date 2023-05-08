#!/usr/bin/env python

import os
import numpy as np
import pickle
import rospy
import tf
import geometry_msgs.msg
from std_msgs.msg import String

class Recorder():
    """Record end-effector (eef) data
    """

    def __init__(self):
        
        super(Recorder, self).__init__()
        
        # Initialize the node
        rospy.init_node("lfd_data_recorder", anonymous=True)
        
        # Subscribe to the trigger topic
        rospy.Subscriber("/trigger", 
                         String, 
                         self.trigger_callback)
        
        # Initialize the transform listener
        self.tflistener = tf.TransformListener()
        
        # End effector pose publisher and subscriber
        rospy.Subscriber("/eef_pose", geometry_msgs.msg.PoseStamped, self.eef_pose_callback)
        self.eef_pose_pub = rospy.Publisher("/eef_pose", geometry_msgs.msg.PoseStamped, queue_size=10)
        
        # Initialize the variables
        self.is_recording = False
        self.demo_numb = 0
        self.record_eef_poses = []
        self.save_eef_poses = []
        self.parent_link = 'world'
        self.link = 'panda_link8'
        

    def trigger_callback(self, trigger_msg):
        """Callback function for trigger topic
        """
        rospy.loginfo(f"Trigger received: {trigger_msg.data}")
        
        if trigger_msg.data == "e_start":
            self.is_recording = True
            self.demo_numb += 1
        elif trigger_msg.data == "e_stop":
            self.is_recording = False
            self.save_data()
            self.record_eef_poses = []
        elif trigger_msg.data == "e_shutdown":
            rospy.signal_shutdown("Done")
        else:
            pass
        
    def eef_pose_callback(self, eef_pose_msg):
        """Callback function for eef pose topic
        """
        current_pose = [eef_pose_msg.pose.position.x,
                        eef_pose_msg.pose.position.y,
                        eef_pose_msg.pose.position.z,
                        eef_pose_msg.pose.orientation.x,
                        eef_pose_msg.pose.orientation.y,
                        eef_pose_msg.pose.orientation.z,
                        eef_pose_msg.pose.orientation.w]
        
        if self.is_recording:
            rospy.logdebug(f"Recording Demostration {self.demo_numb}...")
            rospy.logdebug(f"Current eef pose: {current_pose}")
            self.record_eef_poses.append([current_pose])
            rospy.sleep(0.1)
            
    def save_data(self):
        """Save recorded data
        """
        rospy.loginfo(f"Saving data for demonstration {self.demo_numb}...")
        demo_folder = f"demostrations"
        if not os.path.exists(demo_folder):
            os.makedirs(demo_folder)
        file_name = f"eef_poses_demo_{self.demo_numb}.pickle"
        file_path = os.path.join(demo_folder, file_name)
        with open(file_path, "wb") as f:
            pickle.dump(self.record_eef_poses, f)
        rospy.loginfo(f"Recorded eef poses saved to file {file_name}")
        
    def run(self):
        """Run the node
        """
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            try:
                (trans, rot) = self.tflistener.lookupTransform(self.link, self.parent_link, rospy.Time(0))
                pose_msg = geometry_msgs.msg.PoseStamped()
                pose_msg.header.stamp = rospy.Time.now()
                pose_msg.header.frame_id = self.link
                pose_msg.pose.position.x = trans[0]
                pose_msg.pose.position.y = trans[1]
                pose_msg.pose.position.z = trans[2]
                pose_msg.pose.orientation.x = rot[0]
                pose_msg.pose.orientation.y = rot[1]
                pose_msg.pose.orientation.z = rot[2]
                pose_msg.pose.orientation.w = rot[3]
                self.eef_pose_pub.publish(pose_msg)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            rate.sleep()

if __name__ == '__main__':
    try:
        recorder = Recorder()
        recorder.run()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass