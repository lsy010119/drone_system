import asyncio
import rospy

from mavsdk             import System
from drone_system.msg   import Status
from std_msgs.msg       import String,Bool,Float32MultiArray


class MotionController:

    def __init__(self):
        
        rospy.init_node("motion_controller")
        rospy.Subscriber("/motion_msgs",String,self.motion_update)
        rospy.Subscriber("/trajec_msgs",Float32MultiArray,self.motion_update)
        self.pub2trajec = rospy.Publisher("/trajectory_request",Bool,queue_size=1)
        self.pub2fsm = rospy.Publisher("/is_done",Bool,queue_size=1)
        
        self.motion_command = None


    def motion_update(self,motion_command):

       self.motion_command = motion_command.data


    async def connect_and_arm(self):
        
        print("armed")


    async def disarm(self):
        
        print("disarmed")


    async def take_off(self):

        print("take_off")


    async def hold(self):

        print("hold")

    
    async def land(self):

        print("land")


    async def trajectory_tracking(self,trajectory):

        print("tracking")


    async def main(self):

        while not rospy.is_shutdown():

            if self.motion_command != None:

                if type(self.motion_command) == str: # if its the simple mission


                    if self.motion_command == "arm":
                        
                        self.motion_command = None
                        await self.connect_and_arm()
                        
                        self.pub2fsm.publish(True)


                    elif self.motion_command == "disarm":

                        self.motion_command = None
                        await self.disarm()

                        self.pub2fsm.publish(True)


                    elif self.motion_command == "take_off":

                        self.motion_command = None
                        await self.take_off()

                        self.pub2fsm.publish(True)

                    elif self.motion_command == "hold":

                        self.motion_command = None
                        await self.hold()

                        self.pub2fsm.publish(True)


                else: # if it is the mission requirs planning a trajectory

                    print(self.motion_command)
                    print(type(self.motion_command))
                    # "0" means that it didnt reached to the destination
                    if self.motion_command[0] == 0: 
                        
                        self.trajectory_tracking(self.motion_command[1:])

                        # send a signal if the drone had done
                        # tracking the trajectory recieved
                        self.pub2trajec.publish(True) 

                    # "1" means that its a final trajectory for the destination
                    elif self.motion_command[0] == 1:

                        self.trajectory_tracking(self.motion_command[1:])

                        # send a signal if the drone had done
                        # the mission recieved
                        self.pub2fsm.publish(True)


if __name__ == "__main__":

    M = MotionController()
    
    loop = asyncio.get_event_loop()
    loop.run_until_complete(M.main())