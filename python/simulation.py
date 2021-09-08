from mechanism import Mechanism
from differential import Differential
from cs_gear import CS_Gear
import numpy as np
import matplotlib.pyplot as plt

class Simulation():
    def __init__(self):
        '''Time variables'''
        self.dt = 0.1 # s; time increment
        self.t_history = [] # s
        self.t_history.append(0)
       
        self.mechanism = Mechanism(np.pi)

        '''Generate Trajectory'''
        self.mechanism.generate_trajectory(np.array([[self.mechanism.phi_cs_des], [self.mechanism.theta_cs_des], [self.mechanism.psi_cs_des]]))
        

    def simulate(self):
        t = 0 # s
        iterator = 0
        for _ in range(1000):
            self.mechanism.update(iterator)

            '''Increment time'''
            t += self.dt
            self.t_history.append(t)
            iterator += 1


        plt.figure()
        plt.plot(self.t_history[:1000], self.mechanism.cs_gear.phi_history, label="Roll", color="blue")
        plt.plot(self.t_history[:1000], self.mechanism.cs_gear.theta_history, label="Pitch", color="green")
        plt.plot(self.t_history[:1000], self.mechanism.cs_gear.psi_history, label="Yaw", color="red")
        
        plt.plot(self.t_history[:1000], self.mechanism.q_cs_des_generated[0, :], '--', label="Desired Roll", color="blue")
        plt.plot(self.t_history[:1000], self.mechanism.q_cs_des_generated[1, :], '--', label="Desired Pitch", color="green")
        plt.plot(self.t_history[:1000], self.mechanism.q_cs_des_generated[2, :], '--', label="Desired Yaw", color="red")       
        
        plt.legend()
        plt.show()


simulator = Simulation()
simulator.simulate()

