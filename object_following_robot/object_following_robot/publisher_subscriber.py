# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry


class PublisherSubscriber(Node):

    def __init__(self):
        super().__init__('publisher_subscriber')

        # def publisher
        self.publisher_ = self.create_publisher(PoseStamped, '/robot1/goal_pose', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.x = 0.0
        self.y = 0.0 
        self.z = 0.0
        self.q0 = 0.0
        self.q1 = 0.0
        self.q2 = 0.0
        self.q3 = 0.0

        # def subscriber
        self.subscription = self.create_subscription(
            Odometry,
            '/robot2/wheel/odometry',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y   
        self.z = msg.pose.pose.position.z
        self.q0 = msg.pose.pose.orientation.w
        self.q1 = msg.pose.pose.orientation.x
        self.q2 = msg.pose.pose.orientation.y
        self.q3 = msg.pose.pose.orientation.z
        # self.get_logger().info('I heard: "%f"' % self.x)
        # self.get_logger().info('I heard: "%f"' % self.y)
        # self.get_logger().info('I heard: "%f"' % self.z)
        # self.get_logger().info('I heard: "%f"' % self.q0)
        # self.get_logger().info('I heard: "%f"' % self.q1)
        # self.get_logger().info('I heard: "%f"' % self.q2)
        # self.get_logger().info('I heard: "%f"' % self.q3)


    def timer_callback(self):
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = "map"
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.position.x = self.x
        goal_msg.pose.position.y = self.y
        goal_msg.pose.position.z = self.z
        goal_msg.pose.orientation.w = self.q0
        goal_msg.pose.orientation.x = self.q1
        goal_msg.pose.orientation.y = self.q2
        goal_msg.pose.orientation.z = self.q3
        self.publisher_.publish(goal_msg)
        self.get_logger().info('\033[91m' + 'Publishing:'  + '\033[0m')
        self.get_logger().info('--- position - x: "%f"' % goal_msg.pose.position.x)
        self.get_logger().info('--- position - y: "%f"' % goal_msg.pose.position.y)
        self.get_logger().info('--- position - z: "%f"' % goal_msg.pose.position.z)
        self.get_logger().info('--- orientation - w: "%f"' % goal_msg.pose.orientation.w)
        self.get_logger().info('--- orientation - x: "%f"' % goal_msg.pose.orientation.x)
        self.get_logger().info('--- orientation - y: "%f"' % goal_msg.pose.orientation.y)
        self.get_logger().info('--- orientation - z: "%f"' % goal_msg.pose.orientation.z)


def main(args=None):
    rclpy.init(args=args)

    pub_sub = PublisherSubscriber()

    rclpy.spin(pub_sub)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    pub_sub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()