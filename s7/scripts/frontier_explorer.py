#!/usr/bin/env python3
import rclpy                    # ROS2 client library
from rclpy.node import Node     # ROS2 node baseclass
from nav_msgs.msg import OccupancyGrid
from asl_tb3_msgs.msg import TurtleBotState
from std_msgs.msg import Bool
from asl_tb3_lib.grids import StochOccupancyGrid2D

import numpy as np
import typing as T
from scipy.signal import convolve2d

class Frontier_Explorer(Node):
    def __init__(self):
        super().__init__("Explorer")
        self.occupancy: T.Optional[StochOccupancyGrid2D] = None
        self.nav_success = None
        self.state = None
        self.state_flag = 0 
        self.map_flag = 0
        self.stop_flag = 0 # 0 means no stop sign detected
        self.start_explore_flag = 0
        self.prev_time = 0
        self.current_time = 0
        self.declare_parameter("active", True)

        self.state_sub = # TODO make subscriber node
        self.map_sub = # TODO make subscriber node
        self.nav_success_sub = # TODO make subscriber node
        self.cmd_nav_pub = self.create_publisher(TurtleBotState, "/cmd_nav", 10)
        self.detector_sub = self.create_subscription(Bool, "/detector_bool", self.detector_cb, 10)
    
        
    def explore(self):
        """ returns potential states to explore
        Args:
            occupancy (StochasticOccupancyGrid2D): Represents the known, unknown, occupied, and unoccupied states. See class in first section of notebook.

        Returns:
            frontier_states (np.ndarray): state-vectors in (x, y) coordinates of potential states to explore. Shape is (N, 2), where N is the number of possible states to explore.
        """
        print("Running Explore\n")
        window_size = 13    # defines the window side-length for neighborhood of cells to consider for heuristics
        current_state = np.array([self.state.x, self.state.y])
        
        occupied_mask = np.where(self.occupancy.probs >= 0.5, 1, 0)
		unknown_mask = # TODO use np.where to find unknown mask which is indicated by the value -1
		unoccupied_mask = # TODO use np.where to find unknown mask which is indicated by values in range [0, 0.5)

        kernel = # TODO define the kernel
        occupied=# TODO 2d convolution of mask (use convolve2d) using kernel and mode='same'
        unoccupied=# TODO 2d convolution of mask (use convolve2d) using kernel and mode='same'
        unknown= # TODO 2d convolution of mask (use convolve2d) using kernel and mode='same'

        frontier_mask = # TODO use np.where to make frontier mask based on the 3 conditions of Exploration Heuristics
        frontier_states = np.transpose(np.nonzero(np.transpose(frontier_mask)))
        frontier_states = self.occupancy.grid2state(frontier_states)
        
        if len(frontier_states) == 0:
            print("Finished exploring")

        frontier_state  = np.argmin(np.linalg.norm(frontier_states-current_state,axis=1))
        frontier_state = frontier_states[frontier_state,:]

       
        msg = # TODO
        msg.x, msg.y = #TODO
	    # TODO below publish frontier state into cmd_nav topic
		

    
    def nav_success_cb(self, msg: Bool) -> None:
        """ Callback triggered when nav_success is updated

        Args:
            msg (Bool): updated nav_success message
        """
        # current_time = self.get_clock().now().nanoseconds/1e9
        if self.active: 
            self.explore()
        # else:           
        #     if (current_time-self.prev_time)<=5:
        #         pass
        #     else:
        #         self.explore()
        #         self.prev_time = 0
        #         self.set_parameters([rclpy.Parameter("active",value = True)])

        # self.explore()       
        print("i got a message")
        self.nav_success = msg

    def state_callback(self, msg: TurtleBotState) -> None:
        """ Callback triggered when nav_success is updated

        Args:
            msg (Bool): updated nav_success message
        """
        self.state_flag = 1
        self.state=msg

    def map_callback(self, msg: OccupancyGrid) -> None:
        """ Callback triggered when the map is updated

        Args:
            msg (OccupancyGrid): updated map message
        """
        print("Obtained Map")
        if self.occupancy is None:
            self.map_flag=1
        self.occupancy = StochOccupancyGrid2D(
            resolution=msg.info.resolution,
            size_xy=np.array([msg.info.width, msg.info.height]),
            origin_xy=np.array([msg.info.origin.position.x, msg.info.origin.position.y]),
            window_size=9,
            probs=msg.data,
        )
        if self.map_flag==1 and self.state_flag==1:
            print("Exploring")
            self.explore()
            self.map_flag=0

    @property  
    def active(self)-> bool:
        return self.get_parameter("active").value
    
         
    def detector_cb(self, msg:Bool):
        if msg.data and (self.stop_flag == 0):
            self.set_parameters([rclpy.Parameter("active",value = False)])
            if self.prev_time == 0: # guards in case another stop sign is detected
                self.prev_time = self.get_clock().now().nanoseconds/1e9
            self.stop_sign_wait()# call function to deal with stop sign detection
        elif (self.current_time - self.prev_time) <= 13:
            self.stop_sign_wait()

    def stop_sign_wait(self):
        # this if check may be redundant
        # if self.active: # if stop sign has been detected and robot has not been stopped
        #     self.active = self.set_parameters([rclpy.Parameter("active",value = False)])
        #     msg = self.state
        #     self.cmd_nav_pub.publish(self.state)
            
        # else: # if it's not detected
        self.current_time = self.get_clock().now().nanoseconds/1e9
        if (self.current_time - self.prev_time) <= 5:
            # msg = self.state
            self.cmd_nav_pub.publish(self.state)
            print('stop sign seen within 5 seconds')
        elif (self.current_time - self.prev_time) <= 10: # some buffer time in between
            if (self.start_explore_flag == 0):
                self.explore()
                self.start_explore_flag = 1
            self.set_parameters([rclpy.Parameter("active",value = True)])
            self.stop_flag = 1 # stop sign has been detected and being dealt with (end process)
            print('stop sign seen within 10 seconds')

        else:
            # self.explore()
            self.stop_flag = 0
            self.prev_time = 0
        
        self.start_explore_flag = 0
        
            
        # return msg

    
            

def main():
    rclpy.init(args=None)
    print("main")
    node = Frontier_Explorer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
