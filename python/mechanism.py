from matplotlib.pyplot import csd
import numpy as np

class Mechanism():
	class Differential():
		def __init__(self, mechanical_constants, q_0=np.zeros((2, 1))):
			'''Monopole Gear state variables'''
			self.q = q_0
								
			'''Mechanical constants'''
			self.alpha = mechanical_constants[0]
			self.beta = mechanical_constants[1]
			self.gamma = mechanical_constants[2]

			'''Pose history'''
			self.q_history = []
			self.q_history.append(self.q)

		def update(self, q):
			self.q = q
			self.q_history.append(self.q)

	class CrossSphericalGear():
		def __init__(self, q_0=np.zeros((3, 1))):
			'''Cross Spherial Gear state variables'''
			self.q = q_0
			
			'''Pose history'''
			self.q_history = []
			self.q_history.append(self.q)

		def update(self, q):
			self.q = q
			self.q_history.append(self.q)
		
	def __init__(self, steps):
		'''Build differentials'''
		self.differential_A = self.Differential(np.array([np.pi/2, 0, 0]))
		self.differential_B = self.Differential(np.array([-np.pi/2, 0, np.pi]))

		'''Build cross spherial gear'''
		q_cs_0 = self.forward_kinematics(self.differential_A.q, self.differential_B.q)
		self.cs_gear = self.CrossSphericalGear(q_cs_0)

		'''Generate trajectory of cross spherical gear'''
		q_cs_des = np.array([[0], # desired roll [rad]
							 [np.pi/2], # desired pitch [rad]
							 [0]]) # desired yaw [rad]
		self.q_cs_trajectory = self.generate_trajectory(self.cs_gear.q, q_cs_des, steps)

	def forward_kinematics(self, q_differential_A, q_differential_B):
		beta = np.pi/2

		'''Convert true angles to joint angles'''
		theta_A1 = q_differential_A.flatten()[0]
		theta_A2 = q_differential_A.flatten()[1]/-2
		theta_B1 = q_differential_B.flatten()[0]
		theta_B2 = q_differential_B.flatten()[1]/-2
		
		'''Forward kinematics'''
		U = (np.around(np.sin(beta), decimals=5)*np.cos(theta_A2)*np.sin(theta_A1)*np.sin(theta_A2) - np.around(np.cos(beta), decimals=5)*((np.sin(theta_A1)**2)*(np.sin(theta_A2)**2)+1))
		V = ((np.sin(theta_A1)**2*np.sin(theta_A2)**2) - (np.sin(theta_A2)**2 + 1)) 
		phi_cs = -np.arctan2((np.sin(theta_B1)*np.cos(theta_A1)*np.sin(theta_A1)*np.sin(theta_A2)**2 + np.cos(theta_B1)*U), ((np.cos(theta_B1)*np.cos(theta_A1)*np.sin(theta_A2)*(np.around(np.cos(beta), decimals=5)*np.sin(theta_A1)*np.sin(theta_A2) - np.around(np.around(np.sin(beta), decimals=5), decimals=5)*np.cos(theta_A2)) + np.sin(theta_B1)*V))) 
		theta_cs = np.arcsin(np.sin(theta_A2)*(-np.cos(theta_A1)*np.cos(theta_A2)*np.sin(theta_B1) + np.cos(theta_A2)*np.cos(theta_B1)*np.sin(theta_A1)*np.around(np.cos(beta), decimals=5) + np.sin(theta_A2)*np.cos(theta_B1)*np.around(np.sin(beta), decimals=5))) 
		psi_cs = -np.arctan2((np.sin(theta_A2) / np.cos(theta_A2)*(np.cos(theta_A1)*np.cos(theta_B1)*np.around(np.cos(beta), decimals=5) + np.sin(theta_A1)*np.sin(theta_B1))), 1) 
		q_cs = np.array([[phi_cs], # roll [rad]
                         [theta_cs], # pitch [rad]
                         [psi_cs]]) # yaw [rad]

		return q_cs
	
	def generate_trajectory(self, q_cs_0, q_cs_des, steps):
		phi_cs_trajectory = np.linspace(q_cs_0[0], q_cs_des[0], steps).flatten()
		theta_cs_trajectory = np.linspace(q_cs_0[1], q_cs_des[1], steps).flatten()
		psi_cs_trajectory = np.linspace(q_cs_0[2], q_cs_des[2], steps).flatten()
		q_cs_trajectory = np.vstack((phi_cs_trajectory, theta_cs_trajectory, psi_cs_trajectory))

		return q_cs_trajectory

	def inverse_kinematics(self, q_cs_des):
		beta = np.pi/2

		'''Inverse kinematics'''
		theta_A1 = np.arctan2((np.cos(q_cs_des[0])*np.sin(q_cs_des[2]) + np.cos(q_cs_des[2])*np.sin(q_cs_des[1])*np.sin(q_cs_des[0])), (np.cos(q_cs_des[0])*np.cos(q_cs_des[2])*np.sin(q_cs_des[1]) - np.sin(q_cs_des[0])*np.sin(q_cs_des[2])))
		theta_A2 = np.arccos(np.cos(q_cs_des[1])*np.cos(q_cs_des[2]))
		theta_A3 = -np.arctan2(np.cos(q_cs_des[1])*np.sin(q_cs_des[2]), np.sin(q_cs_des[1]))
		theta_B1 = -np.arctan2((np.cos(q_cs_des[2])*np.around(np.around(np.cos(beta), decimals=5), decimals=5)*np.cos(q_cs_des[0]) + np.sin(q_cs_des[2])*(np.around(np.sin(beta), decimals=5)*np.cos(q_cs_des[1]) - np.around(np.cos(beta), decimals=5)*np.sin(q_cs_des[1])*np.sin(q_cs_des[0]))), (-np.cos(q_cs_des[0])*np.sin(q_cs_des[1])*np.sin(q_cs_des[2]) + np.cos(q_cs_des[2])*np.sin(q_cs_des[0]))) 
		theta_B2 = np.arccos(np.cos(q_cs_des[2])*np.cos(q_cs_des[0])*np.around(np.sin(beta), decimals=5) - np.sin(q_cs_des[2])*(np.around(np.cos(beta), decimals=5)*np.cos(q_cs_des[1]) + np.around(np.sin(beta), decimals=5)*np.sin(q_cs_des[1])*np.sin(q_cs_des[0]))) 
		theta_B3 = np.arctan2((np.cos(q_cs_des[2])*(np.around(np.cos(beta), decimals=5)*np.cos(q_cs_des[1]) + np.around(np.sin(beta), decimals=5)*np.sin(q_cs_des[1])*np.sin(q_cs_des[0])) + np.sin(q_cs_des[2])*np.around(np.sin(beta), decimals=5)*np.cos(q_cs_des[0])), (-np.around(np.cos(beta), decimals=5)*np.sin(q_cs_des[1]) + (np.around(np.sin(beta), decimals=5)*np.cos(q_cs_des[1])*np.sin(q_cs_des[0])))) 

		q_differential_A = np.array([[theta_A1], # roll [rad]
									 [-2*theta_A2]]) # pitch [rad]
		q_differential_B = np.array([[theta_B1], # roll [rad]
									 [-2*theta_B2]]) # pitch [rad]
		
		return [q_differential_A, q_differential_B]
	
	def update(self, i):
		'''
		Inverse kinematics: map desired next cross spherical 
		gear position to new differential positions
		'''
		q_differential_A, q_differential_B = self.inverse_kinematics(self.q_cs_trajectory[:, i])
		self.differential_A.update(q_differential_A)
		self.differential_B.update(q_differential_B)

		'''
		Update cross spherical gear
		'''
		q_cs = self.q_cs_trajectory[:, i].reshape((3, 1))
		self.cs_gear.update(q_cs)



		
