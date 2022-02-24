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
        rospy.Subscriber("/action_msgs", Float32MultiArray, self.action_update)
        rospy.Subscriber("/sensor_msgs", Status, self.action_update)
        rospy.Subscriber("/trajec_request", Bool, self.action_update)
        self.pub2motion = rospy.Publisher("/motion_msgs", Float32MultiArray,queue_size=1)
        
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

                if self.action[0] in [0,1,3]:
                    
                    print(self.action)
                    self.pub2motion_motion.publish(self.action) 
                    # if the mission is simple, pass the mission to motion controller
                    self.action = None

                elif self.action[0] == 2: # take_off mission

                    print(self.action)

                    self.path.generate(self.current_status,self.action)

                    self.action = None
                    

                elif self.action == "park":

                    pass

                elif self.action == "search":

                    pass


if __name__ == "__main__":

    T = TrajectoryGenerator()
    T.run()