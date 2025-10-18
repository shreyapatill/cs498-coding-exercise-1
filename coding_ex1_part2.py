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

    # GPS variables
    lat = 0.0  # Current latitude
    lon = 0.0  # Current longitude
    lat0 = 0.0  # Initial latitude (origin)
    lon0 = 0.0  # Initial longitude (origin)
    flag_lat = False  # Flag to store first latitude reading
    flag_lon = False  # Flag to store first longitude reading

    x = 0.0 # x robot's position
    y = 0.0 # y robot's position
    x_prev = 0.0  # Previous x position (for velocity calculation)
    y_prev = 0.0  # Previous y position (for velocity calculation)
    theta = 0.0 # heading angle

    last_time = 0.0
    current_time = 0.0

    def __init__(self):
        super().__init__('minimal_subscriber')
        
        # Subscribe to gyroscope for heading
        self.subscription_Gyro_yaw = self.create_subscription(Float32, 'Gyro_yaw', self.callback_Gy, 10)

        # Subscribe to GPS topics
        self.subscription_lat = self.create_subscription(Float32, 'latitude', self.callback_lat, 10)
        self.subscription_lon = self.create_subscription(Float32, 'longitude', self.callback_lon, 10)

        self.last_time = self.get_clock().now().nanoseconds/1e9

        self.odom_pub = self.create_publisher(Odometry, 'odom', 10) #keep in mind how to declare publishers for next assignments
        self.timer = self.create_timer(0.1, self.timer_callback_odom) #It creates a timer to periodically publish the odometry.

        self.tf_broadcaster = TransformBroadcaster(self) # To broadcast the transformation between coordinate frames.


        self.file_object_results  = open("results_part2.txt", "w+")
        self.timer2 = self.create_timer(0.1, self.callback_write_txt_file) #Another timer to record some results in a .txt file
        


    def callback_Gy(self, msg):
        self.gyro_yaw = msg.data

    def callback_lat(self, msg):
        self.lat = msg.data
        if not self.flag_lat:  # Store first reading as origin
            self.lat0 = msg.data
            self.flag_lat = True

    def callback_lon(self, msg):
        self.lon = msg.data
        if not self.flag_lon:  # Store first reading as origin
            self.lon0 = msg.data
            self.flag_lon = True

    def timer_callback_odom(self):
        '''
        Use GPS measurements for position and gyroscope for heading
        '''

        self.current_time = self.get_clock().now().nanoseconds/1e9
        dt = self.current_time - self.last_time  # Calculate actual dt

        # Get position from GPS (replaces wheel odometry)
        if self.flag_lat and self.flag_lon:
            self.x, self.y = lonlat2xyz(self.lat, self.lon, self.lat0, self.lon0)

        # Calculate velocity from position change
        if dt > 0:
            vx = (self.x - self.x_prev) / dt
            vy = (self.y - self.y_prev) / dt
            v = math.sqrt(vx**2 + vy**2)
        else:
            v = 0.0

        # Update theta using gyroscope measurement
        self.theta += dt * self.gyro_yaw

        # Store current position for next iteration
        self.x_prev = self.x
        self.y_prev = self.y

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
        odom.twist.twist.linear.x = v
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = self.gyro_yaw  # Use gyroscope measurement

        self.odom_pub.publish(odom)

        self.last_time = self.current_time
        
    def broadcast_tf(self, pos, quater, frame_id, child_frame_id):
        '''
        It continuously publishes the transformation between two reference frames.
        Complete the translation and the rotation of this transformation
        '''
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = frame_id
        t.child_frame_id = child_frame_id

        # Set the translation
        t.transform.translation.x = pos[0]
        t.transform.translation.y = pos[1]
        t.transform.translation.z = pos[2]

        # Set the rotation
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
