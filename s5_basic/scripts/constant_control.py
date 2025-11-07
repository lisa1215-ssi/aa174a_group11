#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

# import the message type to use
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist


class Heartbeat(Node):
    def __init__(self) -> None:
        # initialize base class (must happen before everything else)
        super().__init__("cmd_vel")

        # create publisher with: self.create_publisher(<msg type>, <topic>, <qos>)
        self.hb_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        
        # create a timer with: self.create_timer(<second>, <callback>)
        self.hb_timer = self.create_timer(1.0, self.hb_callback)

        self.kill = self.create_subscription(Bool, "/kill", self.kill_callback, 10)


    def hb_callback(self) -> None:
        """
        Heartbeat callback triggered by the timer
        """
        # construct heartbeat message
		
        msg = Twist()
        msg.linear.x = 5.0
        msg.angular.z = 1.0

        # publish heartbeat counter
        self.hb_pub.publish(msg)

    def kill_callback(self, msg: Bool) -> None:
        """
        Kill callback triggered by /kill subscription
        """
        if msg.data:
            self.get_logger().fatal("Kill signal received. Stopping heartbeat.")
            self.hb_timer.cancel()

            self.hb_pub.publish(Twist())  # publish zero velocity to stop the robot

if __name__ == "__main__":
    rclpy.init()        # initialize ROS2 context (must run before any other rclpy call)
    node = Heartbeat()  # instantiate the heartbeat node
    rclpy.spin(node)    # Use ROS2 built-in schedular for executing the node
    rclpy.shutdown()    # cleanly shutdown ROS2 context