import numpy as np
import matplotlib.pyplot as plt

class Robot():
	def __init__(self):
		mp_A_x0 = np.zeros([2])
		mp_B_x0 = np.zeros([2])

	
	def genTraj(self, cs_orien0, cs_orienf, T):
		traj = np.linspace(cs_orien0, cs_orienf, T)
		return traj

	def invKin(self, cs_orien):
		beta = np.pi/2
		q_cs_des = cs_orien
		theta_A1 = np.arctan2((np.cos(q_cs_des[0])*np.sin(q_cs_des[2]) + np.cos(q_cs_des[2])*np.sin(q_cs_des[1])*np.sin(q_cs_des[0])), (np.cos(q_cs_des[0])*np.cos(q_cs_des[2])*np.sin(q_cs_des[1]) - np.sin(q_cs_des[0])*np.sin(q_cs_des[2])))
		theta_A2 = np.arccos(np.cos(q_cs_des[1])*np.cos(q_cs_des[2]))
		# theta_A3 = -np.arctan2(np.cos(q_cs_des[1])*np.sin(q_cs_des[2]), np.sin(q_cs_des[1]))
		B1_num = (np.cos(q_cs_des[2])*np.cos(beta)*np.cos(q_cs_des[0]) + np.sin(q_cs_des[2])*(np.sin(beta)*np.cos(q_cs_des[1]) - np.cos(beta)*np.sin(q_cs_des[1])*np.sin(q_cs_des[0])))
		if (B1_num<1.e-12):
			B1_num =0
		theta_B1 = -np.arctan2(B1_num, (np.cos(q_cs_des[0])*np.sin(q_cs_des[1])*np.sin(q_cs_des[2]) + np.cos(q_cs_des[2])*np.sin(q_cs_des[0]))) 
		theta_B2 = np.arccos(np.cos(q_cs_des[2])*np.cos(q_cs_des[0])*np.sin(beta) - np.sin(q_cs_des[2])*(np.cos(beta)*np.cos(q_cs_des[1]) + np.sin(beta)*np.sin(q_cs_des[1])*np.sin(q_cs_des[0]))) 
		# theta_B3 = np.arctan2((np.cos(q_cs_des[2])*(np.cos(beta)*np.cos(q_cs_des[1]) + np.sin(beta)*np.sin(q_cs_des[1])*np.sin(q_cs_des[0])) + np.sin(q_cs_des[2])*np.sin(beta)*np.cos(q_cs_des[0])), (-np.cos(beta)*np.sin(q_cs_des[1]) + (np.sin(beta)*np.cos(q_cs_des[1])*np.sin(q_cs_des[0])))) 

		mp_A = [theta_A1, theta_A2]# roll, pitch
		mp_B = [theta_B1, theta_B2]# roll, pitch
		
		return [mp_A, mp_B]
		
cs_orien0 = np.zeros([3])
cs_orien0[0] = 0#np.pi/10.
cs_orien0[1] = 0#np.pi/12.
cs_orien0[2] = 0#np.pi/10.
cs_orienf = np.zeros([3])
cs_orienf[0] = 0#np.pi/3.
cs_orienf[1] = np.pi/4.
cs_orienf[2] = 0#np.pi/3.
T = 10000
robot = Robot()
cs_traj = robot.genTraj(cs_orien0, cs_orienf, T)
mp_log = []
for i in range(T):
	mp_log.append(robot.invKin(cs_traj[i]))




plt.figure()
plt.plot([dp[0]/np.pi*180. for dp in cs_traj], label= 'CS roll')
plt.plot([dp[1]/np.pi*180. for dp in cs_traj], label= 'CS pitch')
plt.plot([dp[2]/np.pi*180. for dp in cs_traj], label= 'CS yaw')
plt.legend()

plt.figure()
plt.plot([ dp[0][0]/np.pi*180. for dp in mp_log], 'k-', label='MP A roll')
plt.plot([ dp[0][1]/np.pi*180. for dp in mp_log], 'r-', label='MP A pitch')
plt.legend()

plt.figure()
plt.plot([ dp[1][0]/np.pi*180. for dp in mp_log], 'k-', label='MP B roll')
plt.plot([ dp[1][1]/np.pi*180. for dp in mp_log], 'r-', label='MP B pitch')
plt.legend()

plt.show()
