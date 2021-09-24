from mechanism import Mechanism
import numpy as np
import matplotlib.pyplot as plt

class Simulation():
    def __init__(self):
        '''Time variables'''
        self.dt = 0.1 # time increment [s]
        self.t_history = [] # history of time [s]
        self.t_history.append(0)
        
        '''Establish 3-degree-of-freedom mechanism'''
        self.steps = 1000
        self.mechanism = Mechanism(self.steps)

    def simulate(self):
        t = 0 # time [s]
        for i in range(1, self.steps):
            self.mechanism.update(i)

            '''Increment time'''
            t += self.dt
            self.t_history.append(t)

        '''Cross Spherial Gear Position'''
        plt.figure(1)
        plt.plot(self.t_history, np.asarray(self.mechanism.cs_gear.q_history)[:, 0].flatten() * (180/np.pi), label="Roll", color="blue")
        plt.plot(self.t_history, np.asarray(self.mechanism.cs_gear.q_history)[:, 1].flatten() * (180/np.pi), label="Pitch", color="green")
        plt.plot(self.t_history, np.asarray(self.mechanism.cs_gear.q_history)[:, 2].flatten() * (180/np.pi), label="Yaw", color="red")
        plt.plot(self.t_history, self.mechanism.q_cs_trajectory[0, :] * (180/np.pi), '--', label="Desired Roll", color="blue")
        plt.plot(self.t_history, self.mechanism.q_cs_trajectory[1, :] * (180/np.pi), '--', label="Desired Pitch", color="green")
        plt.plot(self.t_history, self.mechanism.q_cs_trajectory[2, :] * (180/np.pi), '--', label="Desired Yaw", color="red")       
        plt.legend()
        plt.show()

        '''Differential A & B Position'''
        plt.figure(2)
        plt.plot(self.t_history, np.asarray(self.mechanism.differential_A.q_history)[:, 0].flatten(), label="A Roll")
        plt.plot(self.t_history, np.asarray(self.mechanism.differential_A.q_history)[:, 1].flatten(), label="A Pitch")
        plt.plot(self.t_history, np.asarray(self.mechanism.differential_B.q_history)[:, 0].flatten(), label="B Roll")
        plt.plot(self.t_history, np.asarray(self.mechanism.differential_B.q_history)[:, 1].flatten(), label="B Pitch")
        plt.legend()
        plt.show()

simulator = Simulation()
simulator.simulate()

