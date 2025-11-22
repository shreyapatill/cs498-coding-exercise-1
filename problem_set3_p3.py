import numpy as np
import matplotlib.pyplot as plt
import math

class problem_set3:

       
    def __init__(self):
        np.random.seed(1)
        self.dt = 0.01
        self.time = np.arange(0,200,self.dt)
        
        self.sigma_b = 1E-6 # bias covariance for acceleration
        self.sigma_z = 2.5E-3 # noise covariance for acceleration
        self.sigma_p = 3 # GPS noise covariance 
        self.sigma_w = 2.5E-3 # noise covariance for gyroscope
        self.sigma_enc = 1E-2 #noise covariane for linear velocity (encoder)

        self.x = 0.0 #True x position
        self.y = 0.0 #True y position
        self.v = 0.0 #True forward velocity
        self.w = 0.0 # True angular velocity
        self.a = 0.0 # True acceleration
        self.b = 0.0 # Bias for acceleration measurements
        self.a_ = 0.0 # Acceleration measurement
        
        self.x_gps = 0.0 # GPS measurements
        self.y_gps = 0.0 
        self.v_enc = 0.0 # Wheel encoder measurements
        self.w_gyro = 0.0 # Gyroscope measurements

        self.theta = 0.0 # Heading angle
        # STATE VECTOR
        self.xhat = np.array([[0.0],[0.0],[0.0],[0.0],[0.0]])# state vector (x, y, theta, velocity, bias)
        
        self.P = np.eye(5) #state covariance matrix
        self.Q = np.eye(5)*0.1*self.sigma_z # Process noise matrix
        self.C = np.array([[1, 0, 0, 0, 0],
                    [0, 1, 0, 0, 0],
                    [0, 0, 0, 1, 0]]) # Measurement matrix
        self.R = np.diag([self.sigma_p, self.sigma_p, self.sigma_enc]) # Measurement covariance matrix
        
        self.v_last = 0.0

        self.results = [[],[],[],[],[],[],[],[],[],[],[],[],[],[],[],[]] #do not change this variable. You can use it to plot some data.
        
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
        self.results[3].append(self.y)
        self.results[4].append(self.xhat[1,0])
        self.results[5].append(self.theta)
        self.results[6].append(self.xhat[2,0])
        self.results[7].append(self.v)
        self.results[8].append(self.xhat[3,0])
        self.results[9].append(self.b)
        self.results[10].append(self.xhat[4,0])
        
        Pdiag = self.P.diagonal() # state covariance
        self.results[11].append(Pdiag[0])
        self.results[12].append(Pdiag[1])
        self.results[13].append(Pdiag[2])
        self.results[14].append(Pdiag[3])
        self.results[15].append(Pdiag[4])
        
    
    def true_states(self, t):
        v_left = 0.5 + 0.1*np.sin(0.1*t) # True left-wheels speed
        v_right = 0.6 + 0.2*np.sin(0.1*t) # True right-wheels speed

        self.v = (v_right+v_left)/2 # True forward velocity
        self.w = (v_right-v_left)/(2) # True angular velocity
        self.a = (self.v-self.v_last)/self.dt # True acceleration
        self.theta = self.theta + self.dt*self.w # True heading angle
    
        R_bi = np.array([[math.cos(self.theta), -math.sin(self.theta)],
                        [math.sin(self.theta), math.cos(self.theta)]])
    
        v_i = np.matmul(R_bi, np.array([[self.v],[0]])) # Velocity in the inertial frame
        vx = v_i[0,0]
        vy = v_i[1,0]
        self.x = self.x + self.dt*vx # True position x
        self.y = self.y + self.dt*vy # True position y
        self.v_last = self.v
    
                  
    def measurements_model(self): # In practice, you don't need to do this, as you measure your variables with sensors
        # Acceleration measurements
        noise = np.random.randn(1,1).item()*self.sigma_z # noise
        bdot = np.random.randn(1,1).item()*self.sigma_b
        self.b = self.b + bdot*self.dt 
        self.a_ = self.a + self.b + noise # Acceleration measurements
        # GPS measurements
        self.x_gps = self.x + np.random.randn(1,1).item()*self.sigma_p
        self.y_gps = self.y + np.random.randn(1,1).item()*self.sigma_p
        # Gyro measurements
        self.w_gyro = self.w + np.random.randn(1,1).item()*self.sigma_w
   
        # Encoder measurements (forward velocity)
        self.v_enc = self.v + np.random.randn(1,1).item()*self.sigma_enc

    def estimated_states_prediction(self):
        
        # EKF Prediction step
        # Your dot variables here. Remember your state vector xhat (x, y, theta, v, bias)
        xdot = self.xhat[3,0] * math.cos(self.xhat[2,0]) # ..
        ydot = self.xhat[3,0] * math.sin(self.xhat[2,0]) # ..
        theta_dot = self.w_gyro # .. 
        vdot = self.a_ - self.xhat[4,0] # ..
        bdot = 0 # ..
        
        # Euler integration to compute your state vector xhat
        # Your code here
        self.xhat[0,0] = self.xhat[0,0] + xdot * self.dt
        self.xhat[1,0] = self.xhat[1,0] + ydot * self.dt
        self.xhat[2,0] = self.xhat[2,0] + theta_dot * self.dt
        self.xhat[3,0] = self.xhat[3,0] + vdot * self.dt
        self.xhat[4,0] = self.xhat[4,0] + bdot * self.dt
        
        # Transition matrix
        A = np.array([[0, 0, -self.xhat[3,0]*math.sin(self.xhat[2,0]), math.cos(self.xhat[2,0]), 0],
                      [0, 0, self.xhat[3,0]*math.cos(self.xhat[2,0]), math.sin(self.xhat[2,0]), 0],
                      [0, 0, 0, 0, 0],
                      [0, 0, 0, 0, 0],
                      [0, 0, 0, 0, 0]]) # ..
        
        # Integrate state covariance matrix
        pdot = np.matmul(A, self.P) + np.matmul(self.P, A.T) + self.Q # ..
        self.P = self.P + pdot * self.dt # ..
        
    def estimated_states_correction(self):
        # correction step
        
        # These variables are already defined: self.C, self.P, self.R
        z = np.array([[self.x_gps],[self.y_gps],[self.v_enc]]) # measurements for correction
        
        # Compute Kalman filter gain
        # Your code here
        L = np.matmul(self.P, self.C.T) @ np.linalg.inv(np.matmul(self.C, np.matmul(self.P, self.C.T)) + self.R)
        
        self.xhat = self.xhat + np.matmul(L, (z - np.matmul(self.C, self.xhat))) # ..
        self.P = np.matmul((np.eye(5) - np.matmul(L, self.C)), self.P) # ..


    
if __name__ == '__main__':
    problem = problem_set3()
    plt.figure (1)
    plt.plot(problem.results[0], problem.results[2],'b')
    plt.plot(problem.results[0], problem.results[1],'r')
    plt.legend(["Estimated x position", "True x position"])
    
    plt.figure (2)
    plt.plot(problem.results[0], problem.results[4],'b')
    plt.plot(problem.results[0], problem.results[3],'r')
    plt.legend(["Estimated y position", "True y position"])
    
    plt.figure (3)
    plt.plot(problem.results[0], problem.results[6],'b')
    plt.plot(problem.results[0], problem.results[5],'r')
    plt.legend(["Estimated theta", "True theta"])

    plt.figure (4)
    plt.plot(problem.results[0], problem.results[8],'b')
    plt.plot(problem.results[0], problem.results[7],'r')
    plt.legend(["Estimated velocity", "True velocity"])

    plt.figure (5)
    plt.plot(problem.results[0], problem.results[10],'b')
    plt.plot(problem.results[0], problem.results[9],'r')
    plt.legend(["Estimated bias", "True bias"])

    plt.figure (6)
    plt.plot(problem.results[2], problem.results[4],'b')
    plt.plot(problem.results[1], problem.results[3],'r')
    plt.legend(["Estimated trajectory", "True trajectory"])


    plt.figure (7)
    plt.plot(problem.results[0], problem.results[11],'b')
    plt.plot(problem.results[0], problem.results[12],'r')
    plt.plot(problem.results[0], problem.results[13],'g')
    plt.plot(problem.results[0], problem.results[14],'k')
    plt.plot(problem.results[0], problem.results[15],'c')
    plt.legend(["x covariance", "y covariance", "theta covariance", "v covariance", "bias covariance"])

    plt.show()
