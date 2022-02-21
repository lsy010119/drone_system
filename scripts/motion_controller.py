import asyncio
import rospy

from mavsdk             import System
from drone_system.msg   import Status
from std_msgs.msg       import String,Bool,Float32MultiArray


class MotionController:

    def __init__(self):
        
        rospy.init_node("motion_controller")
        rospy.Subscriber("/motion_msgs",Float32MultiArray,self.motion_update)
        self.pub2trajec = rospy.Publisher("/trajectory_request",Bool,queue_size=1)
        self.pub2fsm = rospy.Publisher("/is_done",Bool,queue_size=1)
        
        self.motion_command = None


    def motion_update(self,motion_command):

       self.motion_command = motion_command 


    async def connect_and_arm(self):
        
        print("connected")
        print("armed")



    async def arm(self):
        # perform the action recieved
        print("armed")

        # send the signal to fsm that the drone completed the mission 
        self.pub2fsm.publish(True)

    async def main(self):

        while not rospy.is_shutdown():

            if type(self.motion_command) == str: # if its the simple mission

                if self.motion_command == "arm":
                    
                    self.motion_command = None
                    await self.connect_and_arm()
                    