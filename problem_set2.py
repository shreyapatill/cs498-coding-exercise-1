import numpy as np
from simple_pid import PID # >>pip3 install simple-pid
import matplotlib.pyplot as plt
import math
from scipy.spatial.transform import Rotation as R # >>pip3 install scipy

# PID further info
# https://github.com/m-lundberg/simple-pid

def pi_clip(angle):
    '''Function to map angle error values between [-pi, pi)'''
    if angle > 0:
        if angle > math.pi:
            return angle - 2*math.pi
    else:
        if angle < -math.pi:
            return angle + 2*math.pi
    return angle

def transformations(Rab, Rbc, Tab, Tbc):
    '''
    Arguments:
        Rab: Rotation matrix from coordinate reference frame b to reference frame a (numpy.ndarray (3,3))
        Rbc: Rotation matrix from coordinate reference frame c to reference frame b (numpy.ndarray (3,3))
        Tab: Translation of b with respect to a (numpy.ndarray(3,))
        Tbc: Translation of c with respect to b (numpy.ndarray(3,))
    Return:
        Rac: Rotation matrix from coordinate reference frame c to reference frame a (numpy.ndarray (3,3))
        quat_ac: quaternion (in order: qx, qy, qz, qw) from coordinate frame c to a (numpy.ndarray (4,))
        euler_ac: Euler angles (in rads and 'xyz' order) from reference frame c to a (numpy.ndarray(3,))
        Tac: Translation of c with respect to a (numpy.ndarray(3,))
    '''
    # compute rotation matrix from c to a: Rac = Rab * Rbc
    Rac = np.dot(Rab, Rbc)
    
    # compute translation from c to a: Tac = Rab * Tbc + Tab
    Tac = np.dot(Rab, Tbc) + Tab
    
    # convert rotation matrix to scipy rotation object
    rotation = R.from_matrix(Rac)
    
    # get quaternion (scipy returns [x, y, z, w] format)
    quat_ac = rotation.as_quat()
    
    # get euler angles in 'xyz' order (scipy default)
    euler_ac = rotation.as_euler('xyz')

    return Rac, quat_ac, euler_ac, Tac


class problem_set2:

    pid_w = PID(-2.0, -0.1, -0.5, setpoint=0.0, output_limits=(-5, 5))
    pid_w.error_map = pi_clip #Function to map angle error values between -pi and pi.
    
    pid_v = PID(-1.5, -0.05, -0.2, setpoint=0.0, output_limits=(0, 2))
    
    def __init__(self):
        
        self.x = 0.0 # (x,y) Robot's position
        self.y = 0.0
        self.xd = 0.0 # (xd, yd) is the desired goal
        self.yd = 0.0
        self.time = np.arange(0,40,0.1)
        self.dt = 0.1
        
        self.v = 0 # Forward velocity
        self.w = 0 # Angular velocity
        self.theta = 0 # Heading angle
        self.results = [[],[],[],[],[],[],[]] #do not change this variable. You can use it to plot some data.
        for t in self.time: # control loop
            self.desired_trajectory(t)
            self.update_robot_state()
            angle_error, distance_error = self.compute_error()
            self.w, self.v = self.compute_control(angle_error, distance_error)
            self.save_results()

    def desired_trajectory(self, t):
        self.xd = 10*np.cos(2*np.pi*0.03*t)
        self.yd = 10*np.sin(2*np.pi*0.03*t) 
        
    
    def update_robot_state(self):
        # euler integration for differential drive robot kinematics
        # ẋ = v * cos(θ), ẏ = v * sin(θ), θ̇ = w
        self.x = self.x + self.v * np.cos(self.theta) * self.dt
        self.y = self.y + self.v * np.sin(self.theta) * self.dt
        self.theta = self.theta + self.w * self.dt
        
    
    def compute_error(self):
        # compute distance error (rho) - euclidean distance to goal
        distance_error = np.sqrt((self.xd - self.x)**2 + (self.yd - self.y)**2)
        
        # compute angle error (alpha) - angle between robot x-axis and vector to goal
        angle_to_goal = np.arctan2(self.yd - self.y, self.xd - self.x)
        angle_error = angle_to_goal - self.theta
        
        # wrap angle error to [-pi, pi] using pi_clip function
        angle_error = pi_clip(angle_error)
        
        return angle_error, distance_error
    
    def compute_control(self, angle_error, distance_error): # It computes the control commands
        control_w = self.pid_w(angle_error, dt = self.dt)
        control_v = self.pid_v(distance_error, dt = self.dt)
        return control_w, control_v

    def save_results(self):
        self.results[0].append(self.x)
        self.results[1].append(self.y)
        self.results[2].append(self.xd)
        self.results[3].append(self.yd)
        self.results[4].append(self.theta)
        self.results[5].append(self.w)
        self.results[6].append(self.v)


if __name__ == '__main__':
    problem = problem_set2()
    plt.figure (1)
    plt.plot(problem.results[0], problem.results[1],'b')
    plt.plot(problem.results[2], problem.results[3],'r')
    plt.xlabel("x")
    plt.ylabel("y")
    plt.legend(["Actual trajectory", "Desired trajectory"])

    plt.figure (2)
    plt.plot(problem.results[5],'b')
    plt.plot(problem.results[6],'r')
    plt.legend(["Angular velocity w", "Forward velocity v"])
    plt.show()
