from differential import Differential
from cs_gear import CS_Gear
import numpy as np


class Mechanism():
    def __init__(self, beta):
        '''Mechanical Constants'''
        self.beta = beta

        '''Establish Parts of the Mechanism'''
        self.cs_gear = CS_Gear()
        self.differential_A = Differential(self.cs_gear, np.pi/2, 0, 0, 'A')
        self.differential_B = Differential(self.cs_gear, -np.pi/2, 0, np.pi, 'B')

        '''Desired Final Euler Angles of CS Gear'''
        self.phi_cs_des = 0 # rad; roll
        self.theta_cs_des = np.pi/2 # rad; pitch
        self.psi_cs_des = 0 # rad; yaw
        self.q_cs_des = np.array([[self.phi_cs_des],
                                 [self.theta_cs_des],
                                 [self.psi_cs_des]])

        '''Generate Trajectory'''
        self.q_cs_des_generated = self.generate_trajectory(self.q_cs_des)

        print("q_cs (before update): ")
        print(self.cs_gear.q)

        self.cs_gear.update(self.cs_gear.q, self.q_cs_des_generated[:, 0].flatten())

        print("q_cs (after update): ")
        print(self.cs_gear.q)

        '''Inverse Kinematics'''
        self.differential_A.update()
        self.differential_B.update()

        print("diff_A (after update): ")
        print(self.differential_A.q)

        print("diff_B (after update): ")
        print(self.differential_B.q)

    def update(self, iterator):
        '''Forward Kinematics'''
        theta_A1 = self.differential_A.theta_1
        theta_A2 = self.differential_A.theta_2
        theta_B1 = self.differential_B.theta_1
        theta_B2 = self.differential_B.theta_2
        
        U = (np.sin(self.beta)*np.cos(theta_A2)*np.sin(theta_A1)*np.sin(theta_A2) - np.cos(self.beta)*((np.sin(theta_A1)**2)*(np.sin(theta_A2)**2)+1))
        V = ((np.sin(theta_A1)**2*np.sin(theta_A2)**2) - (np.sin(theta_A2)**2 + 1)) 
        phi_cs = -np.arctan2((np.sin(theta_B1)*np.cos(theta_A1)*np.sin(theta_A1)*np.sin(theta_A2)**2 + np.cos(theta_B1)*U), ((np.cos(theta_B1)*np.cos(theta_A1)*np.sin(theta_A2)*(np.cos(self.beta)*np.sin(theta_A1)*np.sin(theta_A2) - np.sin(self.beta)*np.cos(theta_A2)) + np.sin(theta_B1)*V))) 
        theta_cs = np.arcsin(np.sin(theta_A2)*(-np.cos(theta_A1)*np.cos(theta_A2)*np.sin(theta_B1) + np.cos(theta_A2)*np.cos(theta_B1)*np.sin(theta_A1)*np.cos(self.beta) + np.sin(theta_A2)*np.cos(theta_B1)*np.sin(self.beta))) 
        psi_cs = -np.arctan2((np.sin(theta_A2) / np.cos(theta_A2)*(np.cos(theta_A1)*np.cos(theta_B1)*np.cos(self.beta) + np.sin(theta_A1)*np.sin(theta_B1))), 1) 
        q_cs = np.array([[phi_cs],
                         [theta_cs],
                         [psi_cs]])

        '''Update CS Gear'''
        self.cs_gear.update(q_cs, self.q_cs_des_generated[:, iterator])
        
        '''Inverse Kinematics'''
        self.differential_A.update()
        self.differential_B.update()

    def generate_trajectory(self, q_cs_des):
        self.phi_cs_des_generated = np.linspace(self.cs_gear.q[0], q_cs_des[0], 1000).flatten()
        self.theta_cs_des_generated = np.linspace(self.cs_gear.q[1], q_cs_des[1], 1000).flatten()
        self.psi_cs_des_generated = np.linspace(self.cs_gear.q[2], q_cs_des[2], 1000).flatten()
        
        q_cs_des_generated = np.vstack((self.phi_cs_des_generated, self.theta_cs_des_generated, self.psi_cs_des_generated))
        return q_cs_des_generated

        
        
        
        
        

        
