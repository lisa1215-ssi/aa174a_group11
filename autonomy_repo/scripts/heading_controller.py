#!/usr/bin/env python3

import numpy, rclpy

from asl_tb3_lib.control import BaseHeadingController
from asl_tb3_lib.math_utils import wrap_angle
from asl_tb3_msgs.msg import TurtleBotControl, TurtleBotState

from std_msgs.msg import Int64, Bool


class HeadingController(BaseHeadingController):
    """ Student can inherit from this class to build a heading controller node

    This node takes target pose from /cmd_pose, and control the robot's orientation
    towards the target pose orientation using a heading controller
    """

    def __init__(self, node_name: str = "heading_control") -> None:
        super().__init__(node_name)

        # Added class variable for proportional control
        self.declare_parameter("active", True)
        self.prev_time = 0

    @property
    def active(self) -> bool:
        """ Get real-time parameter value of maximum velocity

        Returns:
            float: latest parameter value of maximum velocity
        """

        # Retrieve the relevant controller parameter
        return self.get_parameter("active").value


    def compute_control_with_goal(
        self,
        state: TurtleBotState,
        goal: TurtleBotState
    ) -> TurtleBotControl:
        """ Compute control given current robot state and goal state

        Args:
            state (TurtleBotState): current robot state
            goal (TurtleBotState): current goal state

        Returns:
            TurtleBotControl: control command
        """
        omega = 0.5

        command = TurtleBotControl()
        # TODO: Compute the control command based on the current and goal states
        # You may need to handle angle wrapping and apply a control law
        # that uses the parameter(s) defined above.

        # Computer the heading error
        #heading_error = wrap_angle(goal.theta - state.theta)

        # Apply proportional control law
        command.v = 0.0

        if(self.active):
            command.omega = omega
        else:
            if(self.prev_time != 0):
                print('')
                self.prev_time = self.get_clock().now().nanoseconds/1e9

            cur_time = self.get_clock().now().nanoseconds / 1e9
            if(cur_time > self.prev_time + 5):
                print('Waiting for 5 seconds')
                command.omega = 0.5
                self.set_parameters([rclpy.Parameter("active", value = True)])
                self.prev_time = 0
            else:
                raise Exception("Bob")
                print('5 seconds has passed')
                command.omega = 0

        return command


if __name__ == "__main__":
    rclpy.init()        # initialize ROS2 context (must run before any other rclpy call)
    node = HeadingController()  # instantiate the heading controller node
    rclpy.spin(node)    # Use ROS2 built-in schedular for executing the node
    rclpy.shutdown()    # cleanly shutdown ROS2 context
