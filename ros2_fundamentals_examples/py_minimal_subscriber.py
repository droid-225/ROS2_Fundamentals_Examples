#! /usr/bin/env python3

"""
Description:
  This ROS 2 node subscribes to "Hello World" messages.

------
Publishing Topics:
  None

Subscription Topics:
  The channel containing the "Hello World" messages.
  /py_example_topic - std_msgs/String
------
Author: Rishit Shah
Date: February 20, 2026
"""

import rclpy
from rclpy.node import Node

from std_msgs.msg import String

class MinimalPySubscriber(Node):
  def __init__(self):
    super().__init__('minimal_py_subscriber')

    self.subscriber_1 = self.create_subscription(
      String, # data type expected by subscriber
      'py_example_topic', # topic this subscriber is subscribed to
      self.listener_callback, # callback function to activate when a messages comes across the subscribed topic
      10 # queue size, here we only want to keep a max of 10 messages in the queue
    )

  def listener_callback(self, msg):
    self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
  rclpy.init(args=args)

  minimal_py_subscriber = MinimalPySubscriber()

  rclpy.spin(minimal_py_subscriber)

  minimal_py_subscriber.destroy_node()

  rclpy.shutdown()


if __name__ == '__main__':
  main()