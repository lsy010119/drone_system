import numpy as np
import scipy.sparse as ssp
import scipy.sparse.linalg as sla


class Path_Generator:

    def __init__(self):
        
        pass

    def generate(self,x_0,x_des):             ## input : x_0 by state_update / output : velocity list ,4 front

        

        delt = 
        gamma = 0.05

        A = np.array([[1,   0,   0,(1-gamma*delt/2)*delt,  0,   0],
                      [0,   1,   0,   0,(1-gamma*delt/2)*delt,  0],
                      [0,   0,   1,   0,  0,(1-gamma*delt/2)*delt],
                      [0,   0,   0, 1-gamma*delt,          0,   0],
                      [0,   0,   0,   0,  0,    1-gamma*delt,   0],
                      [0,   0,   0,   0,  0,  0,     1-gamma*delt]])

        B = np.array([[delt**2/2, 0, 0],
                      [0, delt**2/2, 0],
                      [0, 0, delt**2/2],
                      [delt,      0, 0],
                      [0, delt,      0]
                      [0, 0,      delt]])




        G = np.zeros((6,3*n))                                                              # Gu = x_des - (A^n)*(x_0)

        for i in range(n):                                                                 ## set G matrix
            G[:,3*i:3*(i+1)] = np.linalg.matrix_power(A,max(0,n-i-1))@B
        u_hat = sla.lsqr(G,x_des - np.linalg.matrix_power(A,n)@x_0)[0]                     # Optimize

        u_vec = u_hat                                                                           # save
        u_opt = u_vec.reshape(n,3).T                                                       # reshpae
        x = np.zeros((6,n+1))                                                        # saver
        x[:,0] = x_0                                                                            # set x_0
        for t in range(n):                                                           # save [x,y,z,Vx,Vy,Vz]
            x[:,t+1] = A.dot(x[:,t]) + B.dot(u_opt[:,t])
        x = x.T                                                                                 # x : [[x0],[x1],[x2]...]
                                                                                                # xn : [Pos_x,Pox_y,Pos_z,Vel_x,Vel_y,Vel_z]

        

        # if Time <= 1:                                          # last trajectory
        #     Time = 1
        #     gen_number = 0                                     # finish and reset
        #     action = None
        # else :
        #     Time -= 0.4
        #     n -= 4
        #     gen_number += 1
        #     # action = None

        # velocity_4_front = np.reshape(x[0:4,3:6],(1,12))[0]               # 4 front of velocity list
        
        # print('velo data number {}= '.format(gen_number),velocity_4_front.reshape((4,3))[:,2])


        return velocity_4_front
