import rclpy
import math
import numpy as np
import sys
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf_transformations import euler_from_quaternion

class TBug(Node):
    def __init__(self, args):
        super().__init__('go_to_goal')
        
        # initialization Variable
        self.goal_coordinates = [float(args[0]),float(args[1])]
        self.odom = [0,0,0] #x,y,yaw
        self.w = 0
        self.v = 0 

        #Subscription
        self.odom_subscription = self.create_subscription(Odometry,'/odom',self.odom_callback,10)

        #publisher
        self.vel_publisher = self.create_publisher(Twist, '/motion', 10)

        #threshold Variable
        self.rotation_threshold = 2*math.pi/180 # 5 degrees
        self.rotation_Kp = 40
        self.max_rotation_w = 35
        self.drive_threshold = 1.0
        self.drive_Kp = 40
        self.max_drive_v = 35

        # Flag Variable
        self.rotation_complete = False

        #parameter Variiable

        self.timer = self.create_timer(0.1, self.run)

## CALLBACKs

    def odom_callback(self,msg):
        self.odom[0] = msg.pose.pose.position.x
        self.odom[1] = msg.pose.pose.position.y
        roll, pitch, yaw = euler_from_quaternion([msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w])
        self.odom[2] = yaw

#helper functionws
    def distance(self, c1, c2):
        return math.sqrt(sum((a-b)**2 for a, b in zip(c1, c2)))
    
#logic functions
      
    def go_to_position(self, position):
        if not self.rotation_complete:
            req_yaw = math.atan2((position[1] - self.odom[1]),(position[0] - self.odom[0]))
            self.rotate(req_yaw)
        else:
            self.drive(position)

    def drive(self,position):
        error = self.distance([self.odom[0],self.odom[1]],position)
        if error < self.drive_threshold:
            self.get_logger().warn(f'drive done :: Error = {error:.3f} m')
            self.v = 0.0
        else:
            self.get_logger().info(f"Error :: {error:.3f} m")
            self.v = self.drive_Kp*error
            self.v = np.clip(self.v,0,self.max_drive_v)
        msg = Twist ()
        msg.linear.x = self.v
        self.vel_publisher.publish(msg)

    def rotate(self,req_yaw):
        error = self.odom[2] -req_yaw
        if abs(error) < self.rotation_threshold:
            self.get_logger().warn(f'rotation done :: Error = {(abs(error))*180/(math.pi):.3f} degrees')
            self.w = 0.0
            self.rotation_complete = True
        else:
            self.get_logger().info(f"Error :: {(abs(error))*180/(math.pi):.3f} degrees")
            self.w = self.rotation_Kp*error
            print(self.w)
            self.w = np.clip(self.w,-1*(self.max_rotation_w),self.max_rotation_w)
        msg = Twist ()
        msg.angular.z = self.w
        self.vel_publisher.publish(msg) 

    def run(self):
        self.go_to_position(self.goal_coordinates)

        
def main(args=None):
    rclpy.init(args=args)

    cli_args = sys.argv[1:]  # sys.argv[0] is the script itself
    node=TBug(cli_args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()  