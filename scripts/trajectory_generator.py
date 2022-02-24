#!/usr/bin/python3
import numpy as np
import rospy

from drone_system.msg import Status
from std_msgs.msg import Bool,String,Float32MultiArray
from path_generator import Path_Generator


class TrajectoryGenerator:

    def __init__(self):
        
        self.path = Path_Generator()        

        rospy.init_node("trajectory_generator")
        rospy.Subscriber("/action_msgs", String, self.action_update)
        rospy.Subscriber("/sensor_msgs", Status, self.action_update)
        rospy.Subscriber("/trjec_request", Bool, self.action_update)
        self.pub2motion_motion = rospy.Publisher("/motion_msgs", String,queue_size=1)
        self.pub2motion_trajec = rospy.Publisher("/trajec_msgs", Float32MultiArray,queue_size=1)
        
        self.current_status = None
        self.action = None



    def current_status(self,status):

        self.current_status = np.array([status.data.pos_n,
                                        status.data.pos_e,
                                        status.data.pos_d,
                                        status.data.vel_n,
                                        status.data.vel_e,
                                        status.data.vel_d])



    def action_update(self,action_msgs):

        self.action = action_msgs.data



    def run(self):


        while not rospy.is_shutdown():

            if self.action != None:

                if self.action in ["arm","disarm","land"]:
                    
                    print(self.action)
                    self.pub2motion_motion.publish(self.action) 
                    # if the mission is simple, pass the mission to motion controller
                    self.action = None

                elif self.action == "take_off":

                    print(self.action)



                    

                elif self.action == "park":

                    pass

                elif self.action == "search":

                    pass


if __name__ == "__main__":

    T = TrajectoryGenerator()
    T.run()