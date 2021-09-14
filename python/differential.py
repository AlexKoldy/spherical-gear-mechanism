import numpy as np
import pinocchio

class Differential():
    def __init__(self, cs_gear, alpha, beta, gamma, name):
        '''Determine whether differential A or B'''
        self.name = name

        '''Integrate CS Gear'''
        self.cs_gear = cs_gear
        
        '''Mechanical Constants'''
        self.alpha = alpha
        self.beta = beta
        self.gamma = gamma

        '''Roll and pitch of MP Gear'''
        self.phi = 0 # rad; roll
        self.theta = 0 # rad; pitch
        self.q = np.array([[self.phi],
                          [self.theta]])

        self.theta_1 = self.phi # rad
        self.theta_2 = self.theta/-2 # rad

        '''Pose History'''
        self.phi_history = [] # rad
        self.theta_history = [] # rad
        self.phi_history.append(self.phi)
        self.theta_history.append(self.theta)

    def R_B_M(self, alpha, beta, gamma):
        return pinocchio.rpy.rpyToMatrix(alpha, beta, gamma)

    def update(self):
        '''Inverse kinematics'''
        self.q_B_M = self.R_B_M(self.alpha, self.beta, self.gamma) # Orientation of driving module with respect to holder's frame

        if (self.name == 'A'):
            self.j = self.cs_gear.j_A
        elif (self.name == 'B'):
            self.j = self.cs_gear.j_B
        else:
            print("ERROR: Differential named improperly")
        
        self.J = np.linalg.inv(self.q_B_M) @ self.cs_gear.q_B_H @ self.j

        yz = np.array([[1],
                       [0],
                       [0]])

        J = self.J - (np.dot(self.J.flatten(), yz.flatten()) / np.linalg.norm(yz)**2)*yz

        self.phi = np.arctan2(J[1], J[2]) # rad; roll angle
        self.theta = 2*np.arccos(J[0]) # rad; pitch angle
        self.q = np.array([[self.phi],
                          [self.theta]])

        '''For forward kinematics'''
        self.theta_1 = self.phi
        self.theta_2 = self.theta/-2

        '''Append history list for graphing'''
        self.phi_history.append(self.phi)
        self.theta_history.append(self.theta)


