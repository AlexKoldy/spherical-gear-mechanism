import numpy as np
from scipy.integrate import odeint
import matplotlib.pyplot as plt

from integrators import get_integrator, Integrator

class Admittance_Controller:
	def __init__(self, m, b, k, r, traj):
		'''System parameters'''
		self.m = m # virtual mass [kg]
		self.b = b # virtual damping coefficient [Ns/m]
		self.k = k # virtual stiffness [N/m]
		self.r = r # arm length [m]

		'''Trajectory to follow'''
		self.traj = traj # [rad]

		'''Initial condition'''
		self.q_0 = np.zeros((2, 1)) # [rad]

		'''Time'''
		start_time = 0 # [s]
		end_time = 10 # [s]
		num_timesteps = 100
		self.dt = (end_time - start_time)/num_timesteps # timestep [s]
		self.t = np.linspace(start_time, end_time, num_timesteps) # time [s]

		'''Iterator'''
		self.p = 0

		'''State'''
		self.q = self.q_0 # [rad]

		'''Integrator'''
		self.integrator = get_integrator(self.dt, self.q_dot)

	'''Update'''
	def __call__(self, F_ext):
		self.F_ext = F_ext
		self.q = self.integrator.step(self.q, self.t[self.p+1])
		#self.q = odeint(self.q_dot, self.q_0.flatten(), self.t[self.p+1])
		self.p += 1

		'''Send theta_d to position controller'''
		return self.q[0]

	'''Derivative of state'''
	def q_dot(self, q, t):
		p = self.p
		
		'''Discrete trajectory'''
		theta_0 = self.traj[p]
		if self.p == 0:
			theta_0_dot = (self.traj[p+1] - self.traj[p])/(2*self.dt)
			theta_0_ddot = (self.traj[p+1] - 2*self.traj[p] + self.traj[p])/self.dt**2
		else:
			theta_0_dot = (self.traj[p+1] - self.traj[p - 1])/(2*self.dt)
			theta_0_ddot = (self.traj[p+1] - 2*self.traj[p] + self.traj[p-1])/self.dt**2

		theta_d = q[0]
		theta_d_dot = q[1]

		'''
		Admittance control and error equations:
		e'' = (1/M)(F_ext/r - b*e' - k*e)
		e = theta_d - theta_0
		'''
		theta_d_ddot = (1/self.m)*((self.F_ext/self.r) - self.b*(theta_d_dot - theta_0_dot) - self.k*(theta_d - theta_0)) + theta_0_ddot
		
		q_dot = np.array([[theta_d_dot],
						  [theta_d_ddot]])
						  
		return q_dot.flatten()

'''Testing purposes only. Not clean and will be deleted'''
traj = np.linspace(0, 20, 100)
admittance_controller = Admittance_Controller(m=1, b=1, k=1, r=1, traj=traj)
t_history = admittance_controller.t
theta_d_history = []
while admittance_controller.p != 99:
	theta_d = admittance_controller(F_ext=1)
	theta_d_history.append(theta_d)

# Plot the Results
plt.plot(t_history[:99], theta_d_history[:], label='theta_d')
plt.plot(t_history[:99], traj[:99], label='theta_0')
plt.title('Simulation of Mass-Spring-Damper System')
plt.xlabel('t')
plt.ylabel('x(t)')
plt.legend()
plt.grid()
plt.show()