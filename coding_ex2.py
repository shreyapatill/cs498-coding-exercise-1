# Student name: 

import math
import numpy as np
from numpy import linalg as LA
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, Accel, PoseStamped
from tf2_ros import TransformBroadcaster

from std_msgs.msg import String, Float32
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import Imu
import matplotlib.pyplot as plt
import time
from mobile_robotics.utils import quaternion_from_euler, lonlat2xyz, quat2euler


class ExtendedKalmanFilter(Node):

    
    def __init__(self):
        super().__init__('ExtendedKalmanFilter')
        
        # Declare parameters for Q and R scaling
        self.declare_parameter('Q_scale', 1.0)
        self.declare_parameter('R_scale', 1.0)
        self.declare_parameter('config_name', 'original')
        self.declare_parameter('rgps_zero', False)  # Set to True to assume GPS at CG
        
        Q_scale = self.get_parameter('Q_scale').get_parameter_value().double_value
        R_scale = self.get_parameter('R_scale').get_parameter_value().double_value
        self.config_name = self.get_parameter('config_name').get_parameter_value().string_value
        rgps_zero = self.get_parameter('rgps_zero').get_parameter_value().bool_value
        
        self.get_logger().info(f'Configuration: {self.config_name}, Q_scale: {Q_scale}, R_scale: {R_scale}, R_GPS_zero: {rgps_zero}')
        
        #array to save the sensor measurements from the rosbag file
        #measure = [p, q, r, fx, fy, fz, x, y, z, vx, vy, vz] 
        self.measure = np.zeros(12)
        
        #Initialization of the variables used to generate the plots.
        self.PHI = []  
        self.PSI = []
        self.THETA = []
        self.P_R = []
        self.P_R1 = []
        self.P_R2 = []
        self.Pos = []
        self.Vel = []
        self.Quater = []
        self.measure_PosX = []
        self.measure_PosY = []
        self.measure_PosZ = []
        self.P_angular = []
        self.Q_angular = []
        self.R_angular = []
        self.P_raw_angular = []
        self.Q_raw_angular = []
        self.R_raw_angular = []
        self.Bias =[]
        
        self.POS_X = []
        self.POS_Y = []
        
        
        #Initialization of the variables used in the EKF
        
        # initial bias values, these are gyroscope and accelerometer biases
        self.bp= 0.0
        self.bq= 0.0
        self.br= 0.0
        self.bfx = 0.0
        self.bfy = 0.0
        self.bfz = 0.0
        # initial rotation
        self.q2, self.q3, self.q4, self.q1 = quaternion_from_euler(0.0, 0.0, np.pi/2) #[qx,qy,qz,qw]

        #initialize the state vector [x y z vx vy vz          quat    gyro-bias accl-bias]
        self.xhat = np.array([[0, 0, 0, 0, 0, 0, self.q1, self.q2, self.q3, self.q4, self.bp, self.bq, self.br, self.bfx, self.bfy, self.bfz]]).T

        # Set R_GPS based on parameter
        if rgps_zero:
            self.rgps = np.array([0.0, 0.0, 0.0])  # GPS at CG
            self.get_logger().info('R_GPS set to zero - assuming GPS at CG')
        else:
            self.rgps = np.array([-0.15, 0.0, 0.0])  # GPS offset from CG
        # This is the location of the GPS wrt CG, this is very important
        
        #noise params process noise (my gift to you :))
        Q_original = np.diag([0.1, 0.1, 0.1, 0.01, 0.01, 0.01, 0.5, 0.5, 0.5, 0.5, 0.01, 0.01, 0.01, 0.001, 0.001, 0.001])
        #measurement noise
        #GPS position and velocity
        R_original = np.diag([10, 10, 10, 2, 2, 2])
        
        # Apply scaling factors
        self.Q = Q_scale * Q_original
        self.R = R_scale * R_original
        
       
        #Initialize P, the covariance matrix
        self.P = np.diag([30, 30, 30, 3, 3, 3, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1])
        self.Pdot=self.P*0.0
        
        self.time = []
        self.loop_t = 0

        # You might find these blocks useful when assembling the transition matrices
        self.Z = np.zeros((3,3))
        self.I = np.eye(3,3)
        self.Z34 = np.zeros((3,4))
        self.Z43 = np.zeros((4,3))
        self.Z36 = np.zeros((3,6))

        self.lat = 0
        self.lon = 0
        self.lat0 = 0
        self.lon0 = 0
        self.flag_lat = False
        self.flag_lon = False
        self.dt = 0.0125 #set sample time

        # Ros subscribers and publishers
        self.subscription_imu = self.create_subscription(Imu, 'terrasentia/imu', self.callback_imu, 10)
        self.subscription_gps_lat = self.create_subscription(Float32, 'gps_latitude', self.callback_gps_lat, 10)
        self.subscription_gps_lon = self.create_subscription(Float32, 'gps_longitude', self.callback_gps_lon, 10)
        self.subscription_gps_speed_north = self.create_subscription(Float32, 'gps_speed_north', self.callback_gps_speed_north, 10)
        self.subscription_gps_speend_east = self.create_subscription(Float32, 'gps_speed_east', self.callback_gps_speed_east, 10)
        
        # Odometry publisher
        self.odom_publisher = self.create_publisher(Odometry, 'odom', 10)
        
        # Path publisher for trajectory visualization
        self.path_publisher = self.create_publisher(Path, 'odom/path', 10)
        self.path = Path()
        self.path.header.frame_id = "odom"
        
        # Transform broadcaster for tf2
        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.timer_ekf = self.create_timer(self.dt, self.ekf_callback)
        self.timer_plot = self.create_timer(1, self.plot_data_callback)

    
    def callback_imu(self,msg):
        #measurement vector = [p, q, r, fx, fy, fz, x, y, z, vx, vy, vz]
        # In practice, the IMU measurements should be filtered. In this coding exercise, we are just going to clip
        # the values of velocity and acceleration to keep them in physically possible intervals.
        self.measure[0] = np.clip(msg.angular_velocity.x,-5,5) #(-5,5)
        self.measure[1] = np.clip(msg.angular_velocity.y,-5,5) #(-5,5)
        self.measure[2] = np.clip(msg.angular_velocity.z,-5,5) #(-5,5)
        self.measure[3] = np.clip(msg.linear_acceleration.x,-6,6) #(-6,6)
        self.measure[4] = np.clip(msg.linear_acceleration.y,-6,6) #(-6,6)
        self.measure[5] = np.clip(msg.linear_acceleration.z,-16,-4) #(-16,-4) 
 
    def callback_gps_lat(self, msg):
        self.lat = msg.data
        if (self.flag_lat == False): #just a trick to recover the initial value of latitude
            self.lat0 = msg.data
            self.flag_lat = True
        
        if (self.flag_lat and self.flag_lon): 
            x, y = lonlat2xyz(self.lat, self.lon, self.lat0, self.lon0) # convert latitude and longitude to x and y coordinates
            self.measure[6] = x
            self.measure[7] = y
            self.measure[8] = 0.0 

    
    def callback_gps_lon(self, msg):
        self.lon = msg.data
        if (self.flag_lon == False): #just a trick to recover the initial value of longitude
            self.lon0 = msg.data
            self.flag_lon = True    
    
    def callback_gps_speed_east(self, msg): 
        self.measure[9] = msg.data # vx (east speed)
        self.measure[11] = 0.0 # vz

    def callback_gps_speed_north(self, msg):
        self.measure[10] = msg.data # vy (north speed)

   
    def ekf_callback(self):
        #print("iteration:  ",self.loop_t)
        if (self.flag_lat and self.flag_lon):  #Trick  to sincronize rosbag with EKF
            self.ekf_function()
        else:
            print("Play the rosbag file...")

    
    
    def ekf_function(self):
        
        # Adjusting angular velocities and acceleration with the corresponding bias
        self.p = (self.measure[0]-self.xhat[10,0])
        self.q = (self.measure[1]-self.xhat[11,0])
        self.r = self.measure[2]-self.xhat[12,0]
        self.fx = (self.measure[3]-self.xhat[13,0])
        self.fy = (self.measure[4]-self.xhat[14,0])
        self.fz = self.measure[5]-self.xhat[15,0]
        
        # Get the current quaternion values from the state vector
        # Remember again the state vector [x y z vx vy vz q1 q2 q3 q4 bp bq br bx by bz]
        self.quat = np.array([[self.xhat[6,0], self.xhat[7,0], self.xhat[8,0], self.xhat[9,0]]]).T
    
        self.q1 = self.quat[0,0]
        self.q2 = self.quat[1,0]
        self.q3 = self.quat[2,0]
        self.q4 = self.quat[3,0]
                
        # Rotation matrix: body to inertial frame
        self.R_bi = np.array([[pow(self.q1,2)+pow(self.q2,2)-pow(self.q3,2)-pow(self.q4,2), 2*(self.q2*self.q3-self.q1*self.q4), 2*(self.q2*self.q4+self.q1*self.q3)],
                          [2*(self.q2*self.q3+self.q1*self.q4), pow(self.q1,2)-pow(self.q2,2)+pow(self.q3,2)-pow(self.q4,2), 2*(self.q3*self.q4-self.q1*self.q2)],
                          [2*(self.q2*self.q4-self.q1*self.q3), 2*(self.q3*self.q4+self.q1*self.q2), pow(self.q1,2)-pow(self.q2,2)-pow(self.q3,2)+pow(self.q4,2)]])
        
            
        #Prediction step
        #First write out all the dots for all the states, e.g. pxdot, pydot, q1dot etc
       
        pxdot = self.xhat[3,0]
        pydot = self.xhat[4,0]
        pzdot = self.xhat[5,0]
        
        # acceleration in body frame
        a_body = np.array([[self.fx], [self.fy], [self.fz]])
        # transform to inertial frame
        a_inertial = self.R_bi @ a_body
        # gravity vector in inertial frame [0, 0, 9.801]
        g_inertial = np.array([[0.0], [0.0], [9.801]])
        
        vxdot = a_inertial[0,0]
        vydot = a_inertial[1,0]
        vzdot = a_inertial[2,0] + g_inertial[2,0]
        
        # quaternion derivative: q_dot = -0.5 * Omega(omega) * q
        # Omega matrix for angular velocity [p, q, r]
        Omega = np.array([[0.0, self.p, self.q, self.r],
                          [-self.p, 0.0, -self.r, self.q],
                          [-self.q, self.r, 0.0, -self.p],
                          [-self.r, -self.q, self.p, 0.0]])
        qdot = -0.5 * Omega @ self.quat
        
        # bias derivatives (constant bias model)
        bpdot = 0.0
        bqdot = 0.0
        brdot = 0.0
        bfxdot = 0.0
        bfydot = 0.0
        bfzdot = 0.0
        
        #Now integrate Euler Integration for Process Updates and Covariance Updates
        # Euler works fine
        # Remember again the state vector [x y z vx vy vz q1 q2 q3 q4 bp bq br bx by bz]
        self.xhat[0,0] = self.xhat[0,0] + self.dt*pxdot
        self.xhat[1,0] = self.xhat[1,0] + self.dt*pydot
        self.xhat[2,0] = self.xhat[2,0] + self.dt*pzdot
        self.xhat[3,0] = self.xhat[3,0] + self.dt*vxdot
        self.xhat[4,0] = self.xhat[4,0] + self.dt*vydot
        self.xhat[5,0] = self.xhat[5,0] + self.dt*vzdot
        self.xhat[6,0] = self.xhat[6,0] + self.dt*qdot[0,0]
        self.xhat[7,0] = self.xhat[7,0] + self.dt*qdot[1,0]
        self.xhat[8,0] = self.xhat[8,0] + self.dt*qdot[2,0]
        self.xhat[9,0] = self.xhat[9,0] + self.dt*qdot[3,0]

        print("x ekf: ", self.xhat[0,0])
        print("y ekf: ", self.xhat[1,0])
        print("z ekf: ", self.xhat[2,0])
        
        # Extract and normalize the quat    
        self.quat = np.array([[self.xhat[6,0], self.xhat[7,0], self.xhat[8,0], self.xhat[9,0]]]).T
        # normalize quat
        quat_norm = LA.norm(self.quat)
        if quat_norm > 1e-10:
            self.quat = self.quat / quat_norm
        
        #re-assign quat
        self.xhat[6,0] = self.quat[0,0]
        self.xhat[7,0] = self.quat[1,0]
        self.xhat[8,0] = self.quat[2,0]
        self.xhat[9,0] = self.quat[3,0]
        
        # update quaternion components after normalization
        self.q1 = self.quat[0,0]
        self.q2 = self.quat[1,0]
        self.q3 = self.quat[2,0]
        self.q4 = self.quat[3,0]
        
        # update rotation matrix after quaternion update
        self.R_bi = np.array([[pow(self.q1,2)+pow(self.q2,2)-pow(self.q3,2)-pow(self.q4,2), 2*(self.q2*self.q3-self.q1*self.q4), 2*(self.q2*self.q4+self.q1*self.q3)],
                          [2*(self.q2*self.q3+self.q1*self.q4), pow(self.q1,2)-pow(self.q2,2)+pow(self.q3,2)-pow(self.q4,2), 2*(self.q3*self.q4-self.q1*self.q2)],
                          [2*(self.q2*self.q4-self.q1*self.q3), 2*(self.q3*self.q4+self.q1*self.q2), pow(self.q1,2)-pow(self.q2,2)-pow(self.q3,2)+pow(self.q4,2)]])
        
                
        # Now write out all the partials to compute the transition matrix Phi
        #delV/delQ - partial derivative of (R_b→I @ a_body) w.r.t. quaternion
        # From lecture equation 118: Fvq = ∂(R_b→I * a)/∂q
        ax, ay, az = self.fx, self.fy, self.fz
        q1, q2, q3, q4 = self.q1, self.q2, self.q3, self.q4
        
        Fvq = np.array([[2*(q1*ax + q4*ay - q3*az), 2*(q2*ax + q3*ay + q4*az), 2*(-q3*ax + q2*ay + q1*az), 2*(-q4*ax - q1*ay + q2*az)],
                        [2*(q4*ax + q1*ay - q2*az), 2*(q3*ax - q2*ay - q1*az), 2*(q2*ax + q3*ay + q4*az), 2*(q1*ax - q4*ay + q3*az)],
                        [2*(-q3*ax + q2*ay + q1*az), 2*(q4*ax + q1*ay - q2*az), 2*(-q1*ax + q4*ay - q3*az), 2*(q2*ax - q3*ay + q4*az)]])
        
        #delV/del_abias - partial derivative of (R_b→I @ a_body) w.r.t. acceleration bias
        # From lecture equation 114: Fvba = -R_b→I
        Fvb = -self.R_bi  # 3x3
        
        #delQ/delQ - partial derivative of quaternion dynamics w.r.t. quaternion
        # From lecture equation 115: Fqq = -0.5 * Ω(ω)
        Fqq = -0.5 * Omega  # 4x4
     
        #delQ/del_gyrobias - partial derivative of quaternion dynamics w.r.t. gyro bias
        # From lecture equation 120: Fqbω = 0.5 * [q2 q3 q4; -q1 q4 -q3; -q4 -q1 q2; q3 -q2 -q1]
        Fqb = 0.5 * np.array([[self.q2, self.q3, self.q4],
                              [-self.q1, self.q4, -self.q3],
                              [-self.q4, -self.q1, self.q2],
                              [self.q3, -self.q2, -self.q1]])  # 4x3
        
        # Now assemble the Transition matrix A (16x16)
        # State vector: [x, y, z, vx, vy, vz, q1, q2, q3, q4, bp, bq, br, bfx, bfy, bfz]
        # From lecture equation 121
        A = np.zeros((16, 16))
        
        # Position derivatives: p_dot = v
        A[0:3, 3:6] = self.I  # Fpv = I (3x3)
        
        # Velocity derivatives: v_dot = R_b→I @ a + g
        A[3:6, 6:10] = Fvq  # Fvq (3x4)
        A[3:6, 13:16] = Fvb  # Fvba (3x3)
        
        # Quaternion derivatives: q_dot = -0.5 * Omega @ q
        A[6:10, 6:10] = Fqq  # Fqq (4x4)
        A[6:10, 10:13] = Fqb  # Fqbω (4x3)
        
        # Bias derivatives: b_dot = 0 (all zeros, already initialized)
        
        #Propagate the error covariance matrix, I suggest using the continuous integration since Q, R are not discretized 
        #Pdot = A@P+P@A.transpose() + Q
        #P = P +Pdot*dt
        Pdot = A @ self.P + self.P @ A.T + self.Q
        self.P = self.P + Pdot * self.dt
        
        #Correction step
        #Get measurements 3 positions and 3 velocities from GPS
        # Note: GPS measurements are at GPS location, state is at CG
        # If R_GPS is zero, GPS is at CG, so measurements are direct
        # If R_GPS is non-zero, need to account for offset:
        # GPS position = CG position + R_b→I @ r_GPS
        # GPS velocity = CG velocity + R_b→I @ (ω × r_GPS)
        
        # Compute expected measurement from state estimate
        if np.linalg.norm(self.rgps) < 1e-6:  # R_GPS is zero (GPS at CG)
            # GPS position = CG position directly
            gps_pos_expected = self.xhat[0:3]
            # GPS velocity = CG velocity directly
            gps_vel_expected = self.xhat[3:6]
        else:
            # Position: p_GPS = p_CG + R_b→I @ r_GPS
            gps_pos_expected = self.xhat[0:3] + self.R_bi @ self.rgps.reshape(3,1)
            
            # Velocity: v_GPS = v_CG + R_b→I @ (ω × r_GPS)
            # ω × r_GPS cross product
            omega_cross = np.array([[0.0, -self.r, self.q],
                                    [self.r, 0.0, -self.p],
                                    [-self.q, self.p, 0.0]])
            omega_cross_rgps = omega_cross @ self.rgps.reshape(3,1)
            gps_vel_expected = self.xhat[3:6] + self.R_bi @ omega_cross_rgps
        
        z_hat = np.vstack([gps_pos_expected, gps_vel_expected])  # 6x1
        
        # Actual measurement
        self.z = np.array([[self.measure[6], self.measure[7], self.measure[8], self.measure[9], self.measure[10], self.measure[11]]]).T #x y z vx vy vz
    
        #Write out the measurement matrix linearization to get H
        # If R_GPS is zero, GPS measurements don't depend on quaternion (GPS at CG)
        # If R_GPS is non-zero, measurements depend on quaternion through offset transformation
        
        if np.linalg.norm(self.rgps) < 1e-6:  # R_GPS is zero (GPS at CG)
            # When GPS is at CG, measurements are direct:
            # GPS position = CG position (no quaternion dependence)
            # GPS velocity = CG velocity (no quaternion dependence)
            Hxq = np.zeros((3, 4))  # No quaternion dependence for position
            Hvq = np.zeros((3, 4))  # No quaternion dependence for velocity
            
            # Assemble H matrix (6x16)
            # H = [I(3x3) Z(3x3) Z(3x4) Z(3x6);
            #      Z(3x3) I(3x3) Z(3x4) Z(3x6)]
            H = np.zeros((6, 16))
            H[0:3, 0:3] = self.I      # position w.r.t. position
            H[3:6, 3:6] = self.I      # velocity w.r.t. velocity
            # All quaternion terms are zero (no dependence)
        else:
            # From lecture equations 124 and 125
            # r_GPS = [-0.15, 0, 0], so r_GPS(1) = -0.15
            rgps1 = self.rgps[0]  # -0.15
            
            #del P/del q - partial derivative of (p + R @ r_GPS) w.r.t. quaternion
            # From lecture equation 124
            Hxq = np.array([[-rgps1*2*q1, -rgps1*2*q2, rgps1*2*q3, rgps1*2*q4],
                            [-rgps1*2*q4, -rgps1*2*q3, -rgps1*2*q2, -rgps1*2*q1],
                            [rgps1*2*q3, -rgps1*2*q4, rgps1*2*q1, -rgps1*2*q2]])  # 3x4
            
            # del v/del q - partial derivative of (v + R @ (ω × r_GPS)) w.r.t. quaternion
            # From lecture equation 125
            # Note: Q = q (angular velocity y), R = r (angular velocity z)
            Q = self.q
            R = self.r
            Hvq = np.array([[rgps1*2*q3*Q + rgps1*2*q4*R, rgps1*2*q4*Q - rgps1*2*q3*R, rgps1*2*q1*Q - rgps1*2*q2*R, rgps1*2*q2*Q + rgps1*2*q1*R],
                            [-rgps1*2*q2*Q - rgps1*2*q1*R, rgps1*2*q2*R - rgps1*2*q1*Q, rgps1*2*q4*Q - rgps1*2*q3*R, rgps1*2*q3*Q + rgps1*2*q4*R],
                            [rgps1*2*q1*Q - rgps1*2*q2*R, -rgps1*2*q2*Q - rgps1*2*q1*R, -rgps1*2*q3*Q - rgps1*2*q4*R, rgps1*2*q4*Q - rgps1*2*q3*R]])  # 3x4
            
            # Assemble H matrix (6x16)
            # From lecture equation 126
            # H = [I(3x3) Z(3x3) Hxq Z(3x6);
            #      Z(3x3) I(3x3) Hvq Z(3x6)]
            H = np.zeros((6, 16))
            H[0:3, 0:3] = self.I      # position w.r.t. position
            H[0:3, 6:10] = Hxq        # position w.r.t. quaternion
            H[3:6, 3:6] = self.I      # velocity w.r.t. velocity
            H[3:6, 6:10] = Hvq        # velocity w.r.t. quaternion
            # All other entries are zero (already initialized)

        #Compute Kalman gain
        # L = P @ H^T @ (H @ P @ H^T + R)^(-1)
        S = H @ self.P @ H.T + self.R  # innovation covariance
        L = self.P @ H.T @ LA.inv(S)  # Kalman gain
        
        #Perform xhat correction    xhat = xhat + L@(z-H@xhat)
        innovation = self.z - z_hat
        self.xhat = self.xhat + L @ innovation
        
        #propagate error covariance approximation P = (np.eye(16,16)-L@H)@P
        self.P = (np.eye(16,16) - L @ H) @ self.P

        #Now let us do some book-keeping 
        # Get some Euler angles
        
        phi, theta, psi = quat2euler(self.quat.T)

        self.PHI.append(phi*180/math.pi)
        self.THETA.append(theta*180/math.pi)
        self.PSI.append(psi*180/math.pi)
    
          
        # Saving data for the plots. Uncomment the 4 lines below once you have finished the ekf function

        DP = np.diag(self.P)
        self.P_R.append(DP[0:3])
        self.P_R1.append(DP[3:6])
        self.P_R2.append(DP[6:10])
        self.Pos.append(self.xhat[0:3].T[0])
        self.POS_X.append(self.xhat[0,0])
        self.POS_Y.append(self.xhat[1,0])
        self.Vel.append(self.xhat[3:6].T[0])
        self.Quater.append(self.xhat[6:10].T[0])
        self.Bias.append(self.xhat[10:16].T[0])
        B = self.measure[6:9].T
        self.measure_PosX.append(B[0])
        self.measure_PosY.append(B[1])
        self.measure_PosZ.append(B[2])

        self.P_angular.append(self.p)
        self.Q_angular.append(self.q)
        self.R_angular.append(self.r)

        self.loop_t += 1
        self.time.append(self.loop_t*self.dt)
        
        # Publish odometry and transform
        self.publish_odometry()

    def publish_odometry(self):
        """Publish odometry message and tf2 transform"""
        # Create odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"
        
        # Set position (x, y, z)
        odom_msg.pose.pose.position.x = float(self.xhat[0,0])
        odom_msg.pose.pose.position.y = float(self.xhat[1,0])
        odom_msg.pose.pose.position.z = float(self.xhat[2,0])
        
        # Set orientation (quaternion)
        # State has [q1, q2, q3, q4] = [qw, qx, qy, qz]
        odom_msg.pose.pose.orientation.w = float(self.xhat[6,0])  # q1 = qw
        odom_msg.pose.pose.orientation.x = float(self.xhat[7,0])  # q2 = qx
        odom_msg.pose.pose.orientation.y = float(self.xhat[8,0])  # q3 = qy
        odom_msg.pose.pose.orientation.z = float(self.xhat[9,0])  # q4 = qz
        
        # Set velocity (linear and angular)
        odom_msg.twist.twist.linear.x = float(self.xhat[3,0])
        odom_msg.twist.twist.linear.y = float(self.xhat[4,0])
        odom_msg.twist.twist.linear.z = float(self.xhat[5,0])
        odom_msg.twist.twist.angular.x = float(self.p)
        odom_msg.twist.twist.angular.y = float(self.q)
        odom_msg.twist.twist.angular.z = float(self.r)
        
        # Set pose covariance (6x6 matrix: x, y, z, roll, pitch, yaw)
        # Extract position and orientation covariances from P
        pose_cov = np.zeros(36)
        pose_cov[0] = self.P[0,0]   # x-x
        pose_cov[7] = self.P[1,1]   # y-y
        pose_cov[14] = self.P[2,2]  # z-z
        # Orientation covariance (simplified - using quaternion covariance diagonal)
        pose_cov[21] = self.P[6,6]  # qw-qw
        pose_cov[28] = self.P[7,7]  # qx-qx
        pose_cov[35] = self.P[8,8]  # qy-qy
        odom_msg.pose.covariance = pose_cov.tolist()
        
        # Set twist covariance (6x6 matrix: vx, vy, vz, wx, wy, wz)
        twist_cov = np.zeros(36)
        twist_cov[0] = self.P[3,3]   # vx-vx
        twist_cov[7] = self.P[4,4]   # vy-vy
        twist_cov[14] = self.P[5,5]  # vz-vz
        twist_cov[21] = 0.01  # wx-wx (angular velocity uncertainty)
        twist_cov[28] = 0.01  # wy-wy
        twist_cov[35] = 0.01  # wz-wz
        odom_msg.twist.covariance = twist_cov.tolist()
        
        # Publish odometry
        self.odom_publisher.publish(odom_msg)
        
        # Create and publish transform
        t = TransformStamped()
        t.header.stamp = odom_msg.header.stamp
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        
        # Set translation
        t.transform.translation.x = odom_msg.pose.pose.position.x
        t.transform.translation.y = odom_msg.pose.pose.position.y
        t.transform.translation.z = odom_msg.pose.pose.position.z
        
        # Set rotation
        t.transform.rotation = odom_msg.pose.pose.orientation
        
        # Broadcast transform
        self.tf_broadcaster.sendTransform(t)
        
        # Update and publish path
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = odom_msg.header.stamp
        pose_stamped.header.frame_id = "odom"
        pose_stamped.pose = odom_msg.pose.pose
        
        self.path.header.stamp = odom_msg.header.stamp
        self.path.poses.append(pose_stamped)
        
        # Keep only last 10000 poses to limit memory
        if len(self.path.poses) > 10000:
            self.path.poses.pop(0)
        
        # Publish path
        self.path_publisher.publish(self.path)

    def plot_data_callback(self):

        # Figure 1: Euler angles
        plt.figure(1)
        plt.clf()
        plt.plot(self.time,self.PHI,'b', self.time, self.THETA, 'g', self.time,self.PSI, 'r')
        plt.legend(['phi','theta','psi'])
        plt.title('Phi, Theta, Psi [deg]')
        plt.xlabel('Time [s]')
        plt.ylabel('Angle [deg]')
        plt.grid(True)

        # Figure 2: xy trajectory
        plt.figure(2)
        plt.clf()
        plt.plot(self.measure_PosX, self.measure_PosY, 'r--', label='GPS', linewidth=1.5)
        plt.plot(self.POS_X, self.POS_Y, 'b-', label='EKF', linewidth=1.5)
        plt.title('xy trajectory')
        plt.legend(['GPS','EKF'])
        plt.xlabel('X [m]')
        plt.ylabel('Y [m]')
        plt.grid(True)
        plt.axis('equal')

        # Figure 3: Covariance of Position
        plt.figure(3)
        plt.clf()
        if len(self.P_R) > 0:
            P_R_array = np.array(self.P_R)
            plt.plot(self.time, P_R_array[:,0], 'b-', label='px', linewidth=1.5)
            plt.plot(self.time, P_R_array[:,1], 'g-', label='py', linewidth=1.5)
            plt.plot(self.time, P_R_array[:,2], 'r-', label='pz', linewidth=1.5)
        plt.title('Covariance of Position')
        plt.legend(['px','py','pz'])
        plt.xlabel('Time [s]')
        plt.ylabel('Covariance [m²]')
        plt.grid(True)

        # Figure 4: Gyroscope Bias
        plt.figure(4)
        plt.clf()
        if len(self.Bias) > 0:
            Bias_array = np.array(self.Bias)
            plt.plot(self.time, Bias_array[:,0], 'b-', label='bp', linewidth=1.5)
            plt.plot(self.time, Bias_array[:,1], 'g-', label='bq', linewidth=1.5)
            plt.plot(self.time, Bias_array[:,2], 'r-', label='br', linewidth=1.5)
        plt.title('Gyroscope Bias')
        plt.legend(['bp','bq','br'])
        plt.xlabel('Time [s]')
        plt.ylabel('Bias [rad/s]')
        plt.grid(True)

        # Figure 5: Accelerometer Bias
        plt.figure(5)
        plt.clf()
        if len(self.Bias) > 0:
            Bias_array = np.array(self.Bias)
            plt.plot(self.time, Bias_array[:,3], 'b-', label='bfx', linewidth=1.5)
            plt.plot(self.time, Bias_array[:,4], 'g-', label='bfy', linewidth=1.5)
            plt.plot(self.time, Bias_array[:,5], 'r-', label='bfz', linewidth=1.5)
        plt.title('Accelerometer Bias')
        plt.legend(['bfx','bfy','bfz'])
        plt.xlabel('Time [s]')
        plt.ylabel('Bias [m/s²]')
        plt.grid(True)
                
        plt.ion()
        plt.show()
        plt.pause(0.0001)
    
    def save_plots(self):
        """Save all plots to files for report"""
        try:
            prefix = f"{self.config_name}_"
            
            # Save trajectory data for comparison
            trajectory_data = {
                'time': np.array(self.time),
                'ekf_x': np.array(self.POS_X),
                'ekf_y': np.array(self.POS_Y),
                'gps_x': np.array(self.measure_PosX),
                'gps_y': np.array(self.measure_PosY),
                'config_name': self.config_name
            }
            np.save(f"{prefix}trajectory_data.npy", trajectory_data)
            print(f"Saved: {prefix}trajectory_data.npy")
            
            # Figure 1: Euler angles
            plt.figure(1)
            plt.clf()
            plt.plot(self.time,self.PHI,'b', self.time, self.THETA, 'g', self.time,self.PSI, 'r')
            plt.legend(['phi','theta','psi'])
            plt.title(f'Phi, Theta, Psi [deg] - {self.config_name}')
            plt.xlabel('Time [s]')
            plt.ylabel('Angle [deg]')
            plt.grid(True)
            plt.savefig(f'{prefix}euler_angles.png', dpi=300, bbox_inches='tight')
            print(f"Saved: {prefix}euler_angles.png")

            # Figure 2: xy trajectory
            plt.figure(2)
            plt.clf()
            plt.plot(self.measure_PosX, self.measure_PosY, 'r--', label='GPS', linewidth=1.5)
            plt.plot(self.POS_X, self.POS_Y, 'b-', label='EKF', linewidth=1.5)
            plt.title(f'xy trajectory - {self.config_name}')
            plt.legend(['GPS','EKF'])
            plt.xlabel('X [m]')
            plt.ylabel('Y [m]')
            plt.grid(True)
            plt.axis('equal')
            plt.savefig(f'{prefix}xy_trajectory.png', dpi=300, bbox_inches='tight')
            print(f"Saved: {prefix}xy_trajectory.png")

            # Figure 3: Covariance of Position
            plt.figure(3)
            plt.clf()
            if len(self.P_R) > 0:
                P_R_array = np.array(self.P_R)
                plt.plot(self.time, P_R_array[:,0], 'b-', label='px', linewidth=1.5)
                plt.plot(self.time, P_R_array[:,1], 'g-', label='py', linewidth=1.5)
                plt.plot(self.time, P_R_array[:,2], 'r-', label='pz', linewidth=1.5)
            plt.title(f'Covariance of Position - {self.config_name}')
            plt.legend(['px','py','pz'])
            plt.xlabel('Time [s]')
            plt.ylabel('Covariance [m²]')
            plt.grid(True)
            plt.savefig(f'{prefix}covariance_position.png', dpi=300, bbox_inches='tight')
            print(f"Saved: {prefix}covariance_position.png")

            # Figure 4: Gyroscope Bias
            plt.figure(4)
            plt.clf()
            if len(self.Bias) > 0:
                Bias_array = np.array(self.Bias)
                plt.plot(self.time, Bias_array[:,0], 'b-', label='bp', linewidth=1.5)
                plt.plot(self.time, Bias_array[:,1], 'g-', label='bq', linewidth=1.5)
                plt.plot(self.time, Bias_array[:,2], 'r-', label='br', linewidth=1.5)
            plt.title(f'Gyroscope Bias - {self.config_name}')
            plt.legend(['bp','bq','br'])
            plt.xlabel('Time [s]')
            plt.ylabel('Bias [rad/s]')
            plt.grid(True)
            plt.savefig(f'{prefix}gyroscope_bias.png', dpi=300, bbox_inches='tight')
            print(f"Saved: {prefix}gyroscope_bias.png")

            # Figure 5: Accelerometer Bias
            plt.figure(5)
            plt.clf()
            if len(self.Bias) > 0:
                Bias_array = np.array(self.Bias)
                plt.plot(self.time, Bias_array[:,3], 'b-', label='bfx', linewidth=1.5)
                plt.plot(self.time, Bias_array[:,4], 'g-', label='bfy', linewidth=1.5)
                plt.plot(self.time, Bias_array[:,5], 'r-', label='bfz', linewidth=1.5)
            plt.title(f'Accelerometer Bias - {self.config_name}')
            plt.legend(['bfx','bfy','bfz'])
            plt.xlabel('Time [s]')
            plt.ylabel('Bias [m/s²]')
            plt.grid(True)
            plt.savefig(f'{prefix}accelerometer_bias.png', dpi=300, bbox_inches='tight')
            print(f"Saved: {prefix}accelerometer_bias.png")
        except Exception as e:
            print(f"Error saving plots: {e}")

def main(args=None):
    rclpy.init(args=args)

    ekf_node = ExtendedKalmanFilter()
    
    try:
        rclpy.spin(ekf_node)
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        # Save plots before shutting down
        ekf_node.save_plots()
        ekf_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
