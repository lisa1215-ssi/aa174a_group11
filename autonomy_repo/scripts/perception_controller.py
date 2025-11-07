#!/usr/bin/env python3

import numpy, rclpy

from asl_tb3_lib.control import BaseHeadingController
from asl_tb3_lib.math_utils import wrap_angle
from asl_tb3_msgs.msg import TurtleBotControl, TurtleBotState

from std_msgs.msg import Int64, Bool


class PerceptionController(BaseHeadingController):

    def __init__(self, node_name: str = "perception_control") -> None:
        super().__init__(node_name)

        self.declare_parameter("active", True)



    @property
    def active(self) -> bool:
        return self.get_parameter("active").value


    #def active(self, value: bool):
    #    self.set_parameter(rclpy.Parameter("active", rclpy.Parameter.Type.Bool, value))

    def compute_control_with_goal(
        self,
        state: TurtleBotState,
        goal: TurtleBotState
    ) -> TurtleBotControl:

        command = TurtleBotControl()
        '''
        if(self.active):
            self.set_parameters([rclpy.Parameter("command.omega", value = 0.5)])
        else:
            start_time = self.get_clock().now().nanoseconds / 1e9
            self.set_parameters([rclpy.Parameter("command.omega", value = 0)])
            while((self.get_clock().now().nanoseconds / 1e9) < start_time+5):
                pass
            self.active = true;
            self.set_parameters([rclpy.Parameter("command.omega", value = 0.5)])
        '''

        command.omega = 0.5

        return command


if __name__ == "__main__":
    rclpy.init()        # initialize ROS2 context (must run before any other rclpy call)
    node = PerceptionController()  # instantiate the heading controller node
    rclpy.spin(node)    # Use ROS2 built-in schedular for executing the node
    rclpy.shutdown()    # cleanly shutdown ROS2 context
