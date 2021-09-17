from mechanism2 import Mechanism
#from differential import Differential
#from cs_gear import CS_Gear
import numpy as np
import matplotlib.pyplot as plt

class Simulation():
    def __init__(self):
        '''Time variables'''
        self.dt = 0.1 # s; time increment
        self.t_history = [] # s
        self.t_history.append(0)
       
        self.mechanism = Mechanism()

        '''Generate Trajectory'''
        #self.mechanism.generate_trajectory(np.array([[self.mechanism.phi_cs_des], [self.mechanism.theta_cs_des], [self.mechanism.psi_cs_des]]))
        

    def simulate(self):
        t = 0 # s
        steps = 1000
        for i in range(1, steps):
            self.mechanism.update(i)

            '''Increment time'''
            t += self.dt
            self.t_history.append(t)

        #print(np.asarray(self.mechanism.cs_gear.q_history)[:, 1].flatten())

        plt.figure()
        
        #plt.plot(self.t_history, np.asarray(self.mechanism.cs_gear.q_history)[:, 0].flatten(), label="Roll", color="blue")
        #plt.plot(self.t_history, np.asarray(self.mechanism.cs_gear.q_history)[:, 1].flatten(), label="Pitch", color="green")
        #plt.plot(self.t_history, np.asarray(self.mechanism.cs_gear.q_history)[:, 2].flatten(), label="Yaw", color="red")
        
        #plt.plot(self.t_history, self.mechanism.q_cs_trajectory[0, :], '--', label="Desired Roll", color="blue")
        #plt.plot(self.t_history, self.mechanism.q_cs_trajectory[1, :], '--', label="Desired Pitch", color="green")
        #plt.plot(self.t_history, self.mechanism.q_cs_trajectory[2, :], '--', label="Desired Yaw", color="red")       
        

        plt.plot(self.t_history, np.asarray(self.mechanism.differential_A.q_history)[:, 0].flatten(), label="A Roll")
        plt.plot(self.t_history, np.asarray(self.mechanism.differential_A.q_history)[:, 1].flatten(), label="A Pitch")

        plt.plot(self.t_history, np.asarray(self.mechanism.differential_B.q_history)[:, 0].flatten(), label="B Roll")
        plt.plot(self.t_history, np.asarray(self.mechanism.differential_B.q_history)[:, 1].flatten(), label="B Pitch")
        plt.xlim(0, 1)
        

        #plt.xlim(0, 0.01)
        plt.legend()
        plt.show()


simulator = Simulation()
simulator.simulate()

