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
        self.sigma_p = 3 # GPS noise covariance 
        self.x = 0.0 #True position
        self.v = 0.0 #True velocity
        self.a = 0.0 # True acceleration
        self.b = 0.0 # Bias for acceleration measurements
        self.a_ = 0.0 # Acceleration measurement
        
        self.x_gps = 0.0 # GPS position

        # STATE VECTOR
        self.xhat = np.array([[0.0],[0.0],[0.0]]) # state vector (position, velocity, bias)

        self.C = np.array([[1, 0, 0]]) # Measurement matrix
        self.A = np.array([[0, 1, 0],[0, 0, -1],[0, 0, 0]]) #A matrix for the point-mass system model

        self.R = np.eye(1)*self.sigma_p # Measurement covariance matrix
        self.P = np.eye(3) # State covariance matrix
        self.Q = np.eye(3)*0.1*self.sigma_z # Process noise

        self.results = [[], [],[],[],[],[],[],[],[],[]] 
        for t in self.time: # control loop
            self.true_states(t)
            self.measurements_model()
            self.estimated_states_prediction()
            self.estimated_states_correction()

            self.save_results(t)

    
    def save_results(self,t):
        self.results[0].append(t)
        self.results[1].append(self.x)
        self.results[2].append(self.xhat[0,0])
        self.results[3].append(self.a)
        self.results[4].append(self.a_)
        self.results[5].append(self.v)
        self.results[6].append(self.xhat[1,0])
        Pdiag = self.P.diagonal()
        self.results[7].append(Pdiag[0])
        self.results[8].append(Pdiag[1])
        self.results[9].append(Pdiag[2])
   
    def true_states(self, t):
        self.a = math.sin(0.1*t)*0.01 # True acceleration
        
        xdot =  self.v
        vdot = self.a
        self.x = self.x + xdot*self.dt #True position
        self.v = self.v + vdot*self.dt #True velocity
   
    def measurements_model(self): # In practice, you don't need to do this, as you measure your variables with sensors
        
        # Acceleration measurements
        noise = np.random.randn(1,1).item()*self.sigma_z # noise
        bdot = np.random.randn(1,1).item()*self.sigma_b
        self.b = self.b + bdot*self.dt 
        self.a_ = self.a + self.b + noise #Acceleration measurements
        
        # GPS measurements
        self.x_gps = self.x + np.random.randn(1,1).item()*self.sigma_p # noise
        
    def estimated_states_prediction(self):
        #Prediction step
        # Your dot variables here. Remember your state vector xhat (position, velocity, bias)
        
        xdot = self.xhat[1,0] # ..
        vdot = self.a_ - self.xhat[2,0] # ..
        bdot = 0 # ..
        # Euler integration to compute your state vector

        self.xhat[0,0] = self.xhat[0,0] + xdot * self.dt # ..
        self.xhat[1,0] = self.xhat[1,0] + vdot * self.dt # .. 
        self.xhat[2,0] = self.xhat[2,0] + bdot * self.dt # ..
    
        # Integrate state covariance matrix
        pdot = np.matmul(self.A,self.P) + np.matmul(self.P,self.A.T) + self.Q
        self.P = self.P + pdot * self.dt # ..
    
    def estimated_states_correction(self):
        # Correction step
        # These variables are already defined: self.C, self.P, self.R
        y = np.array([[self.x_gps]]) # measurements for correction
        
        # Compute Kalman filter gain
        # Your code here
        L = np.matmul(self.P, self.C.T) @ np.linalg.inv(np.matmul(self.C, np.matmul(self.P, self.C.T)) + self.R)
        
        self.xhat = self.xhat + np.matmul(L, (y - np.matmul(self.C, self.xhat))) # ..
        self.P = np.matmul((np.eye(3) - np.matmul(L, self.C)), self.P) # ..

     
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

    plt.figure (4)
    plt.plot(problem.results[0], problem.results[7],'b')
    plt.plot(problem.results[0], problem.results[8],'r')
    plt.plot(problem.results[0], problem.results[9],'g')
    plt.legend(["Position covariance", "Velocity covariance", "bias covariance"])

    plt.show()
