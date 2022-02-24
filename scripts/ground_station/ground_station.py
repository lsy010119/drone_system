#!/usr/bin/python3
import numpy as np
import rospy

from drone_system.msg import Status
from std_msgs.msg     import Float32MultiArray,Bool

class GroundStation:


    def __init__(self):

        rospy.init_node("ground_station")
        rospy.Subscriber("/input_permission", Bool, self.input_handler)
        self.pub2datahub = rospy.Publisher("/mission_msgs", Float32MultiArray, queue_size=1)
        self.input_avaliable = False
        self.gs_rate = rospy.Rate(10)



    def user_input(self):

        while not rospy.is_shutdown():
            
            if self.input_avaliable == True:
                
                self.input_avaliable = False
                
                mission_msgs = Float32MultiArray()

                mission_no = int(input("===============Mission Input================\n\
0 : disarm\n1 : arm\n2 : take_off\n3 : land\n4 : park\n5 : search\nInput Mission No : "))

                mission_data = np.array([mission_no])

                if mission_no == 2: # input the target altitude for take_off mission

                    target_altitude = input("Mission : take off\nInput the target altitude : ")

                    mission_data = mission_data.append(target_altitude)

                elif mission_no == 5:
                    
                    # drag the area to search
                    # and attach the coordinates of area to the mission data
                    pass

                mission_msgs.data = mission_data

                self.pub2datahub.publish(mission_msgs)

            self.gs_rate.sleep()
    

    def input_handler(self, permission):

        # permission will be sended from motion_controller 
        # when the mission recieved is done 
        self.input_avaliable = permission.data



if __name__ == "__main__":

    GS = GroundStation()
    GS.user_input()