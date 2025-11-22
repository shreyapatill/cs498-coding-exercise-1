import numpy as np
import matplotlib.pyplot as plt
import math

class problem_set3:

       
    def __init__(self):
        np.random.seed(1)
        self.dt = 0.01
        self.time = np.arange(0,700,self.dt)
        
        self.sigma_b = 1E-6 # bias covariance for acceleration
        self.sigma_z = 2.5E-3 # noise covariance for acceleration
        self.x = 0.0 #True position
        self.v = 0.0 #True velocity
        self.a = 0.0 # True acceleration
        self.b = 0.0 # Bias for acceleration measurements

        self.a_ = 0.0 # Acceleration measurement
        # STATE VECTOR
        self.xhat = np.array([[0.0],[0.0]]) # state vector (position, velocity)

        self.results = [[], [],[],[],[],[],[],[],[],[]] #do not change this variable. You can use it to plot some data.
        for t in self.time: # control loop
            self.true_states(t)
            self.measurements_model()
            self.estimated_states()
            self.save_results(t)

    
    def save_results(self,t):
        self.results[0].append(t)
        self.results[1].append(self.x) # True position
        self.results[2].append(self.xhat[0,0]) # Position estimate
        self.results[3].append(self.a) # True acceleration
        self.results[4].append(self.a_) # Acceleration measurements
        self.results[5].append(self.v) # True velocity
        self.results[6].append(self.xhat[1,0]) # Velocity estimate

    def true_states(self, t):
        #self.dt is already defined
        self.a = math.sin(0.1*t)*0.01 # True acceleration
        xdot =  self.v
        vdot = self.a # .. Your code here 
        self.x = self.x + xdot * self.dt # .. True position
        self.v = self.v + vdot * self.dt # .. True velocity

    def measurements_model(self):
        noise = np.random.randn(1,1).item()*self.sigma_z # Acceleration noise
        bdot = np.random.randn(1,1).item()*self.sigma_b
        self.b = self.b + bdot * self.dt # .. Integrate bdot

        self.a_ = self.a + self.b + noise # Acceleration measurements
        
    def estimated_states(self):
        # Estimated position and velocity
        xdot = self.xhat[1,0]
        vdot = self.a_ # ..
        self.xhat[0,0] = self.xhat[0,0] + xdot * self.dt # ..
        self.xhat[1,0] = self.xhat[1,0] + vdot * self.dt # ..

      
if __name__ == '__main__':
    problem = problem_set3()
    plt.figure (1)
    plt.plot(problem.results[0], problem.results[2],'b')
    plt.plot(problem.results[0], problem.results[1],'r')
    plt.legend(["Estimated position", "True position"])
    
    plt.figure (2)
    plt.plot(problem.results[0], problem.results[4],'b')
    plt.plot(problem.results[0], problem.results[3],'r')
    plt.legend(["Acceleration measurement", "True acceleration"])
    
    plt.figure (3)
    plt.plot(problem.results[0], problem.results[6],'b')
    plt.plot(problem.results[0], problem.results[5],'r')
    plt.legend(["Estimated velocity", "True velocity"])

    plt.show()
