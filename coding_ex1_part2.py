# version 0.0
# Jose Cuaran


import math
import numpy as np
import rclpy
from rclpy.node import Node
#from rclpy.clock import Clock

from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

from std_msgs.msg import String, Float32
from nav_msgs.msg import Odometry
from mobile_robotics.utils import quaternion_from_euler, lonlat2xyz #edit according to your package's name


class OdometryNode(Node):
    # Initialize some variables
    
    gyro_yaw = 0.0
    latitude = 0.0
    longitude = 0.0
    
    # GPS reference coordinates (initial position)
    lat0 = 0.0
    lon0 = 0.0
    gps_initialized = False
    gps_lat_received = False
    gps_lon_received = False

    x = 0.0 # x robot's position
    y = 0.0 # y robot's position
    theta = 0.0 # heading angle

    last_time = 0.0
    current_time = 0.0
    
    last_x = 0.0
    last_y = 0.0
    last_theta = 0.0
    
    vx = 0.0
    vy = 0.0
    last_vx = 0.0
    last_vy = 0.0

    def __init__(self):
        super().__init__('minimal_subscriber')
        
        # Declare subscribers to GPS and gyro topics
        self.subscription_Gyro_yaw = self.create_subscription(Float32, 'Gyro_yaw', self.callback_Gy, 10)
        self.subscription_latitude = self.create_subscription(Float32, 'latitude', self.callback_lat, 10)
        self.subscription_longitude = self.create_subscription(Float32, 'longitude', self.callback_lon, 10)

        self.last_time = self.get_clock().now().nanoseconds/1e9
        
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10) #keep in mind how to declare publishers for next assignments
        self.timer = self.create_timer(0.1, self.timer_callback_odom) #It creates a timer to periodically publish the odometry.
        
        self.tf_broadcaster = TransformBroadcaster(self) # To broadcast the transformation between coordinate frames.


        self.file_object_results  = open("results_part2.txt", "w+")
        self.timer2 = self.create_timer(0.1, self.callback_write_txt_file) #Another timer to record some results in a .txt file
        


    def callback_Gy(self, msg):
        self.gyro_yaw = msg.data
        
    def callback_lat(self, msg):
        self.latitude = msg.data
        self.gps_lat_received = True
        # Initialize reference coordinates when both lat and lon are received
        if not self.gps_initialized and self.gps_lat_received and self.gps_lon_received:
            self.lat0 = self.latitude
            self.lon0 = self.longitude
            self.gps_initialized = True
            print(f"GPS initialized with reference: lat0={self.lat0}, lon0={self.lon0}")
        
    def callback_lon(self, msg):
        self.longitude = msg.data
        self.gps_lon_received = True
        # Initialize reference coordinates when both lat and lon are received
        if not self.gps_initialized and self.gps_lat_received and self.gps_lon_received:
            self.lat0 = self.latitude
            self.lon0 = self.longitude
            self.gps_initialized = True
            print(f"GPS initialized with reference: lat0={self.lat0}, lon0={self.lon0}")
        

    def timer_callback_odom(self):
        '''
        Compute the position using GPS measurements
        Use lonlat2xyz to convert GPS coordinates to x,y displacement
        '''

        self.current_time = self.get_clock().now().nanoseconds/1e9
        dt = 0.1  # Fixed timestep as suggested in debugging hints
        
        # Only proceed if GPS is initialized
        if not self.gps_initialized:
            return
            
        # Calculate position from GPS
        x_gps, y_gps = lonlat2xyz(self.latitude, self.longitude, self.lat0, self.lon0)
        
        # Update position from GPS
        self.x = x_gps
        self.y = y_gps
        
        # Calculate heading from gyro
        self.theta = self.last_theta + self.gyro_yaw * dt
        
        # Calculate velocities from position differences
        if dt > 0:
            self.vx = (self.x - self.last_x) / dt
            self.vy = (self.y - self.last_y) / dt
        else:
            self.vx = 0.0
            self.vy = 0.0

        position = [self.x, self.y, 0.0]
        quater = quaternion_from_euler(0.0, 0.0, self.theta)
        print("position: ", position)
        print("orientation: ", quater)


        # We need to set an odometry message and publish the transformation between two coordinate frames
        # Further info about odometry message: https://docs.ros2.org/foxy/api/nav_msgs/msg/Odometry.html
        # Further info about tf2: https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Introduction-To-Tf2.html
        # Further info about coordinate frames in ROS: https://www.ros.org/reps/rep-0105.html

        frame_id = 'odom'
        child_frame_id = 'base_link'
        
        
        self.broadcast_tf(position, quater, frame_id, child_frame_id)  # Before creating the odometry message, go to the broadcast_tf function and complete it.
        
        odom = Odometry()
        odom.header.frame_id = frame_id
        odom.header.stamp = self.get_clock().now().to_msg()

        # set the pose
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = quater[0]
        odom.pose.pose.orientation.y = quater[1]
        odom.pose.pose.orientation.z = quater[2]
        odom.pose.pose.orientation.w = quater[3]

        # set the velocities
        odom.child_frame_id = child_frame_id
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = self.vy
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = math.radians(self.gyro_yaw)

        self.odom_pub.publish(odom)

        self.last_time = self.current_time
        self.last_x = self.x
        self.last_y = self.y
        self.last_vx = self.vx
        self.last_vy = self.vy
        self.last_theta = self.theta
        
    def broadcast_tf(self, pos, quater, frame_id, child_frame_id):
        '''
        It continuously publishes the transformation between two reference frames.
        Complete the translation and the rotation of this transformation
        '''
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = frame_id
        t.child_frame_id = child_frame_id

        # Set translation (position)
        t.transform.translation.x = pos[0]
        t.transform.translation.y = pos[1]
        t.transform.translation.z = pos[2]

        # Set rotation (quaternion)
        t.transform.rotation.x = quater[0]
        t.transform.rotation.y = quater[1]
        t.transform.rotation.z = quater[2]
        t.transform.rotation.w = quater[3]

        # Send the transformation
        self.tf_broadcaster.sendTransform(t)
    
    def callback_write_txt_file(self):
        if (self.x != 0 or self.y != 0 or self.theta != 0):
            self.file_object_results.write(str(self.current_time) + " " + str(self.x)+" "+str(self.y)+" "+str(self.theta)+"\n")

    
def main(args=None):
    rclpy.init(args=args)

    odom_node = OdometryNode()

    rclpy.spin(odom_node)
    odom_node.file_object_results.close()
    odom_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()