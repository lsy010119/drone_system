#!/usr/bin/env python3
import numpy as np
import rospy

from drone_system.msg import Status
from std_msgs.msg import String,Float32MultiArray

class TrajectoryGenerator:

    def __init__(self):
        
        rospy.init_node("trajectory_generator")
        rospy.Subscriber("action_msgs", String, self.action_update)
        self.pub2motion_motion = rospy.Publisher("motion_msgs", String,queue_size=1)
        self.pub2motion_trajec = rospy.Publisher("trajec_msgs", Float32MultiArray,queue_size=1)

        self.action = None


    def action_update(self,action_msgs):

        self.action = action_msgs.data



    def run(self):

        while not rospy.is_shutdown():
            if self.action != None:
                if self.action != "park" or "search":
                    
                    print(self.action)
                    self.pub2motion_motion.publish(self.action) 
                    # if the mission is simple, pass the mission to motion controller
                    self.action = None


                elif self.action == "park":

                    pass

                elif self.action == "search":

                    pass


if __name__ == "__main__":

    T = TrajectoryGenerator()
    T.run()