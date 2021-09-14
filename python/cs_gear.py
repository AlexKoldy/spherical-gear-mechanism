import numpy as np
import pinocchio

class CS_Gear():
    def __init__(self):
        '''Euler Angles'''
        self.phi = 0 # rad; roll
        self.theta = 0 # rad; pitch
        self.psi = 0 # rad; yaw
        self.q = np.array([[self.phi],
                          [self.theta],
                          [self.psi]])

        '''Desired Euler Angles'''
        '''
        self.phi_des = 0 # rad; roll
        self.theta_des = 0 # rad; pitch
        self.psi_des = 0 # rad; yaw
        self.q_des = np.array([[self.phi_des],
                              [self.theta_des],
                              [self.psi_des]])
        '''

        '''Unit vectors extending from the origin of CS gear in the CS gear's frame'''
        self.j_A = np.array([[1],
                             [0],
                             [0]])
        self.j_B = np.array([[0],
                             [1],
                             [0]])

        '''Pose History'''
        self.phi_history = [] # rad
        self.theta_history = [] # rad
        self.psi_history = [] # rad

    def R_B_H(self, q_des):
        return pinocchio.rpy.rpyToMatrix(q_des[0], q_des[1], q_des[2])

    def update(self, q, q_des):
        '''Update the Euler angles'''
        self.phi = q[0]
        self.theta = q[1]
        self.psi = q[2]
        self.q = q

        '''Target Orientation of CS Gear'''
        self.q_B_H = self.R_B_H(q_des)

        '''Append history list for graphing'''
        self.phi_history.append(q[0])
        self.theta_history.append(q[1])
        self.psi_history.append(q[2])









        '''
        theta_A1 = np.arctan((np.cos(phi_d)*np.sin(psi_d) + np.cos(psi_d)*np.sin(theta_d)*np.sin(phi_d)) / (np.cos(phi_d)*np.cos(psi_d)*np.sin(theta_d) - np.sin(phi_d)*np.sin(psi_d)))
        theta_A2 = np.arccos(np.cos(theta_d)*np.cos(psi_d))
        theta_A3 = -np.arctan(np.cos(theta_d)*np.sin(psi_d) / np.sin(theta_d))
        theta_B1 = -np.arctan((np.cos(psi_d)*np.cos(beta)*np.cos(phi_d) + np.sin(psi_d)*(np.sin(beta)*np.cos(theta_d) - np.cos(beta)*np.sin(theta_d)*np.sin(phi_d)))/(-np.cos(beta)*np.sin(theta_d)*np.sin(psi_d) + np.cos(psi_d)*np.sin(psi_d))) 
        theta_B2 = np.arccos(np.cos(psi_d)*np.cos(phi_d)*np.sin(beta) - np.sin(psi_d)*(np.cos(beta)*np.cos(theta_d) + np.sin(beta)*np.sin(theta_d)*np.sin(phi_d))) 
        theta_B3 = np.arctan((np.cos(psi_d)*(np.cos(beta)*np.cos(theta_d) + np.sin(beta)*np.sin(theta_d)*np.sin(phi_d)) + np.sin(psi_d)*np.sin(beta)*np.cos(phi_d)) / (-np.cos(beta)*np.sin(theta_d) + (np.sin(beta)*np.cos(theta_d)*np.sin(phi_d)))) 
        
        R = np.array([[np.cos(theta_A1), np.sin(theta_A2)*np.sin(theta_A3), np.cos(theta_A3)*np.sin(theta_A2)],
                      [np.sin(theta_A1)*np.sin(theta_A2), np.cos(theta_A1)*np.cos(theta_A3) - np.cos(theta_A2)*np.sin(theta_A1)*np.sin(theta_A3), -np.cos(theta_A1)*np.sin(theta_A3) - np.cos(theta_A2)*np.cos(theta_A3)*np.sin(theta_A1)],
                      [-np.cos(theta_A1)*np.sin(theta_A2), np.cos(theta_A1)*np.cos(theta_A2)*np.sin(theta_A3) + np.cos(theta_A3)*np.sin(theta_A1), np.cos(theta_A1)*np.cos(theta_A2)*np.cos(theta_A3) - np.sin(theta_A1)*np.sin(theta_A3)]])
        '''


    #def generate_angles(self, phi_mp, theta_mp):
        #self.phi = phi_mp
        #self.theta = theta_mp/-2



        


        

