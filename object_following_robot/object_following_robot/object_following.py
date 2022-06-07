import math
import rclpy

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry


class RobotController(Node):

    def __init__(self):
        super().__init__('robot_controller')

        # Define publishers and subscribers
        self.publisher = self.create_publisher(PoseStamped, '/robot1/goal_pose', 10)
        self.subscriber = self.create_subscription(Odometry, '/robot2/wheel/odometry', self.odom_callback, 10)
        self.timer = self.create_timer(1, self.control_cycle)

        
    # Callback executed to store current turtle state
    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
   	    self.y = msg.pose.pose.position.y
   	    self.z = msg.pose.pose.position.z
   	    self.q0 = msg.pose.pose.orientation.w
        self.q1 = msg.pose.pose.orientation.x
   	    self.q2 = msg.pose.pose.orientation.y
   	    self.q3 = msg.pose.pose.orientation.z


    # Timer callback - executed every 0.01 seconds
    def control_cycle(self):
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = "map"
        goal_msg.header.stamp = pose.header.stamp
        goal_msg.pose.position.x = self.x
        goal_msg.pose.position.y = self.y
        goal_msg.pose.position.z = self.z
        goal_msg.pose.orientation.w = self.q0
        goal_msg.pose.orientation.x = self.q1
        goal_msg.pose.orientation.y = self.q2
        goal_msg.pose.orientation.z = self.q3
        self.publisher.publish(goal_msg)
            

def main(args=None):
    rclpy.init(args=args)

    controller = RobotController()

    rclpy.spin(controller)

    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
