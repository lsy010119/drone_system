import numpy as np
from std_msgs.msg import String,Bool,Float32MultiArray


class FSM:
    """
    Mission Code ===========================================
    
    Code Number         Trajectory Requirements  

    0 : disarm                  X
    1 : arm                     X
    2 : take_off                O
    3 : land                    X
    4 : park                    O
    5 : search                  O
    6 : hold                    X (translates autometically)

    Avaliable translations of the state ====================

    0 ===> 1 (user)             2 ===> 6 (Auto)
                                3 ===> 0 (Auto)
    1 ===> 0 (user)             4 ===> 3 (Auto)
    1 ===> 2 (user)             5 ===> 6 (Auto)

    6 ===> 3 (user)
    6 ===> 4 (user)
    6 ===> 5 (user)
    """
    def __init__(self):

        pass


    def transform_disarm(self,data_hub):
        """
        State : 'disarm'
        
        0 disarm =(/mission)====> arm    1
        """
        avaliable_mission = [1]

        ''' disarm ===> arm (by /mission_msgs from ground_station) '''  
        if data_hub.mission[0] == 1 and data_hub.transform_trigger:

            print("disarm ---> arm")
            data_hub.transform_trigger = False # turn off the trigger

            mission_data = Float32MultiArray() # encoding mission into message
            mission_data.data = data_hub.mission

            data_hub.pub2trajec.publish(mission_data) # send a mission to trajectory
            data_hub.cur_state = "arm" # update the current state

            # inputing mission is not avaliable until the mission "arm" is done
            data_hub.pub2ground.publish(False)
            

        elif not data_hub.mission[0] in avaliable_mission: 

            data_hub.transform_trigger = False # turn off the trigger
            
            # give the input permission to input the valid mission
            data_hub.pub2ground.publish(True)



    def transform_arm(self,data_hub):

        """
        State : 'arm'
        
        1 arm =(/mission)====> disarm     0
        1 arm =(auto_disarm)=> disarm     0
        1 arm =(/mission)====> take_off   2
        """
        available_mission = [0,2]

        ''' arm ===> disarm (by /mission_msgs from ground_station) '''  
        if data_hub.mission[0] == 0 and data_hub.transform_trigger:

            print("arm ---> disarm")
            data_hub.transform_trigger = False # turn off the trigger
            
            mission_data = Float32MultiArray() # encoding mission into message
            mission_data.data = data_hub.mission

            data_hub.pub2trajec.publish(mission_data) # send a mission to trajectory
            data_hub.cur_state = "disarm" # update the current state
            
            # inputing mission is not avaliable until the mission "disarm" is done
            data_hub.pub2ground.publish(False)

        ''' arm ===> take_off (by /mission_msgs from ground_station) '''  
        if data_hub.mission[0] == 2 and data_hub.transform_trigger:

            print("arm ---> take_off")
            data_hub.transform_trigger = False # turn off the trigger

            mission_data = Float32MultiArray() # encoding mission into message
            mission_data.data = data_hub.mission
            # mission msg indicating take_off includes the data of the target altitude 

            data_hub.pub2trajec.publish(mission_data) # send a mission to trajectory
            data_hub.cur_state = "take_off" # update the current state

            data_hub.is_performing_action = True # the drone starts to performing an action
            
            # inputing mission is not avaliable until the mission "take_off" is done
            data_hub.pub2ground.publish(False)

        elif not data_hub.mission[0] in available_mission:

            data_hub.transform_trigger = False # turn off the trigger
            
            # give the input permission to input the valid mission
            data_hub.pub2ground.publish(True)



    def transform_take_off(self,data_hub):

        """
        State : 'take_off'
        
        2 take_off =(/is_done)====> hold   6
        """

        ''' take_off ===> hold (by /is_done from motion_controller) '''  
        if data_hub.is_performing_action == False: # if take_off mission is done

            print("take_off ---> hold")
            # data_hub.transform_trigger = False # turn off the trigger

            mission_data = Float32MultiArray() # encoding mission into message
            mission_data.data = np.array([6]) # 6 : hold

            data_hub.pub2trajec.publish(mission_data) # send a mission to trajectory
            data_hub.cur_state = "hold" # update the current state



    def transform_hold(self,data_hub):

        """
        State : 'hold'
        
        6 hold =(/mission)====> land   3
        6 hold =(/mission)====> park   4
        6 hold =(/mission)====> search 5
        """

        available_mission = [3,4,5]

        ''' hold ===> land (by /mission_msgs from ground station) '''  
        if data_hub.mission[0] == 3 and data_hub.transform_trigger:

            print("hold ---> land")
            data_hub.transform_trigger = False # turn off the trigger
            
            mission_data = Float32MultiArray() # encoding mission into message
            mission_data.data = data_hub.mission

            data_hub.pub2trajec.publish(mission_data) # send a mission to trajectory
            data_hub.cur_state = "land" # update the current state
            data_hub.is_performing_action = True # the drone starts to performing an action
            
            # inputing mission is not avaliable until the mission "land" is done
            data_hub.pub2ground.publish(False)

        ''' arm ===> park (by /mission_msgs from ground station) '''  
        if data_hub.mission[0] == 4 and data_hub.transform_trigger:

            print("hold ---> park")
            data_hub.transform_trigger = False # turn off the trigger

            mission_data = Float32MultiArray() # encoding mission into message
            mission_data.data = data_hub.mission

            data_hub.pub2trajec.publish(mission_data) # send a mission to trajectory
            # msg indicates park mission includes the data of the relative position of the marker 

            data_hub.cur_state = "park" # update the current state
            data_hub.is_performing_action = True # the drone starts to performing an action

            # inputing mission is not avaliable until the mission "park" is done
            data_hub.pub2ground.publish(False)

        ''' arm ===> search (by /mission_msgs from ground station) '''  
        if data_hub.mission[0] == 5 and data_hub.transform_trigger:

            print("hold ---> search")
            data_hub.transform_trigger = False # turn off the trigger
            
            mission_data = Float32MultiArray() # encoding mission into message
            mission_data.data = data_hub.mission

            data_hub.pub2trajec.publish(mission_data) # send a mission to trajectory
            # msg indicates search mission includes the data of the waypoints 

            data_hub.cur_state = "search" # update the current state
            data_hub.is_performing_action = True # the drone starts to performing an action
            
            # inputing mission is not avaliable until the mission "search" is done
            data_hub.pub2ground.publish(False)

        elif not data_hub.mission[0] in available_mission:

            # print("Invalid mission recieved. please input the avaliable mission")
            data_hub.transform_trigger = False # turn off the trigger
            
            # give the input permission to input the valid mission
            data_hub.pub2ground.publish(True)



    def transform_land(self,data_hub):

        """
        State : 'land'
        
        1. land =(/is_done)====> disarm
        """

        ''' land ===> disarm (by /is_done from motion_controller) '''  
        if data_hub.is_performing_action == False:

            print("land ---> disarm")
            data_hub.cur_state = "disarm" # update the current state

            # inputing mission is not avaliable until the mission "land" is done
            data_hub.pub2ground.publish(False)





    def transform_state(self,data_hub):

        #FSM Algorithm here

        if data_hub.cur_state == "arm":
            
            self.transform_arm(data_hub)


        elif data_hub.cur_state == "disarm":
        
            self.transform_disarm(data_hub)

        
        elif data_hub.cur_state == "take_off":
        
            self.transform_take_off(data_hub)

        
        elif data_hub.cur_state == "hold":
        
            self.transform_hold(data_hub)

        
        elif data_hub.cur_state == "land":
        
            self.transform_land(data_hub)

        
        elif data_hub.cur_state == "park":
        
            pass


        elif data_hub.cur_state == "search":
        
            pass
