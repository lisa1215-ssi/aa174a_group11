#!/usr/bin/env python3

# Import ROS2 Python client library
import rclpy                    
from rclpy.node import Node     # ROS2 node baseclass
# Import navigation and math utilities from ASL TurtleBot3 library
from asl_tb3_lib.navigation import BaseNavigator, TrajectoryPlan, StochOccupancyGrid2D
from asl_tb3_lib.math_utils import wrap_angle
from asl_tb3_lib.tf_utils import quaternion_to_yaw
from asl_tb3_msgs.msg import TurtleBotControl, TurtleBotState
# Import standard Python libraries
import numpy as np
from numpy import linalg
import typing as T
from scipy.interpolate import splev, splrep   # spline interpolation utilities
import matplotlib.pyplot as plt
import matplotlib.patches as patches


# ---------------------------
# A* PATH PLANNING ALGORITHM
# ---------------------------
class AStar(object): 

    def __init__(self, statespace_lo, statespace_hi, x_init, x_goal, occupancy, resolution=0.1):
        # Define search space bounds
        self.statespace_lo = statespace_lo         # lower bound of state space (e.g., [-5, -5])
        self.statespace_hi = statespace_hi         # upper bound of state space (e.g., [5, 5])
        self.occupancy = occupancy                 # occupancy grid (typically DetOccupancyGrid2D)
        self.resolution = resolution               # grid resolution (meters per cell)

        # Reference offset for grid snapping
        self.x_offset = x_init                     
        # Snap the start and goal states to the nearest grid cells
        self.x_init = self.snap_to_grid(x_init)    
        self.x_goal = self.snap_to_grid(x_goal)    

        # Initialize data structures for the A* search
        self.closed_set = set()    # already explored states
        self.open_set = set()      # frontier states to explore next

        # Dictionaries for path cost tracking
        self.est_total_cost = {}  # f(x): estimated total cost (start → x → goal)
        self.cost_to_arrive = {}    # g(x): cost to reach state x from start
        self.came_from = {}         # predecessor mapping to reconstruct final path

        # Initialize start node
        self.open_set.add(self.x_init)
        self.cost_to_arrive[self.x_init] = 0
        self.est_total_cost[self.x_init] = self.distance(self.x_init, self.x_goal)

        # Will hold the final reconstructed path
        self.path = None        


    def is_free(self, x):
        """Check whether a given state x is in free space (not occupied)."""
        return self.occupancy.is_free(np.array(x))

    def distance(self, x1, x2):
        """Compute Euclidean distance between two 2D points."""
        return np.linalg.norm(np.array(x1) - np.array(x2))

    def snap_to_grid(self, x):
        """Snap continuous coordinates to the nearest grid cell based on resolution."""
        return (
            self.resolution * round((x[0] - self.x_offset[0]) / self.resolution) + self.x_offset[0],
            self.resolution * round((x[1] - self.x_offset[1]) / self.resolution) + self.x_offset[1],
        )

    def get_neighbors(self, x):
        """Return all valid (free) neighboring cells of the current grid state."""
        neighbors = []
        # Explore the 8-connected neighborhood (including diagonals)
        for i in range(-1,2):
            for j in range(-1,2):
                if i == j == 0:
                    continue  # Skip the current cell itself
                # TODO: Compute neighbor coordinates
                # HINT: The neighbor’s coordinates should be offset from x by i * resolution in x and j * resolution in y.
                #       For example, if resolution = 0.1, the neighbors are at ±0.1 or diagonal ±(0.1,0.1) away.
                x_neigh = x[0] + i * self.resolution
                y_neigh = x[1] + j * self.resolution

                # Snap to grid to ensure discretization consistency
                neighs = self.snap_to_grid((x_neigh,y_neigh))
                # Only consider neighbor if it lies in free space
                if self.is_free(neighs):
                    neighbors.append(neighs)
        return neighbors

    def find_node_min_toal_cost(self):
        # TODO: Return the node in the open set with the lowest estimated total cost f(x).
        # HINT 1: Look through all nodes in self.open_set.
        # HINT 2: Use self.est_total_cost[...] to get the f(x) value for each node.
        # HINT 3: You can use Python's min() function with a key=... argument 
        #         to find the node with the smallest estimated cost.
        # Example: min(collection, key=lambda node: ...)
        # If open set is empty, return None
        if not self.open_set:
            return None
        # Use est_total_cost, defaulting to +inf for nodes not present
        return min(self.open_set, key=lambda node: self.est_total_cost.get(node, float("inf")))


    def reconstruct_path(self):
        """Reconstruct full path from start to goal using the came_from dictionary."""
        path = [self.x_goal]
        current = path[-1]
        # Follow predecessors backward until reaching the start
        while current != self.x_init:
            path.append(self.came_from[current])
            current = path[-1]
        # Return path in forward order
        return list(reversed(path))
        # plt.show()

    def solve(self):
        """Perform the A* search algorithm to find a path from start to goal."""
        while len(self.open_set) > 0:
            # select node with minimum estimated total cost
            x_current = self.find_node_min_toal_cost()

            # Check if the goal has been reached
            if x_current == self.x_goal:
                self.path = self.reconstruct_path()
                print("DONE")
                return True
            
            # Move current node from open to closed set
            self.open_set.remove(x_current)
            self.closed_set.add(x_current)

            # Explore each neighboring state
            for x_neigh in self.get_neighbors(x_current):
                if x_neigh not in self.closed_set:
                    # TODO: Compute tentative cost to reach the neighbor
                    # HINT: g(x_neigh) = g(x_current) + distance(x_current, x_neigh)
                    cost_to_neighbor = self.cost_to_arrive[x_current] + self.distance(x_current, x_neigh)

                    # Add neighbor to open set if it is a new path
                    if x_neigh not in self.open_set:
                        self.open_set.add(x_neigh)
                    else:
                        # Skip if the new path is not better
                        if cost_to_neighbor > self.cost_to_arrive[x_neigh]:
                            continue

                    # Record the best path so far
                    self.came_from[x_neigh] = x_current
                    self.cost_to_arrive[x_neigh] = cost_to_neighbor

                    # TODO: Record total cost of neighbor nodes
                    # HINT: f = g + heuristic. Use Euclidean distance to goal as heuristic h(x).
                    self.est_total_cost[x_neigh] = cost_to_neighbor + self.distance(x_neigh, self.x_goal)

        # If the open set is empty and goal not reached, no path exists
        return False
        ########## Code ends here ##########


# ---------------------------
# NAVIGATOR CONTROL CLASS
# ---------------------------
class Navigator(BaseNavigator):
    """Implements a ROS2 Navigator node that plans and follows trajectories using A* and spline tracking."""

    def __init__(self, 
        kpx = 0.2, 
        kpy = 0.2, 
        kdx = 0, 
        kdy = 0.
    ):
        # Initialize the BaseNavigator node
        super().__init__("navigator")
        # PID gains for position and derivative control
        self.kp = 5.0
        self.kpx = kpx
        self.kpy = kpy
        self.kdx = kdx
        self.kdy = kdy

        # Control parameters and thresholds
        self.V_PREV_THRESH = 0.001   # small velocity threshold to avoid singularities
        self.coeffs = np.zeros(8)     # placeholder for spline coefficients
        self.v_desired = 0.15         # desired forward velocity (m/s)
        self.spline_alpha = 0.02      # smoothing factor for spline interpolation


    def reset(self) -> None:
        """Reset stored control and timing variables."""
        self.V_prev = 0.
        self.om_prev = 0.
        self.t_prev = 0.

    def compute_heading_control(self, current_state: TurtleBotState, goal_state: TurtleBotState) -> TurtleBotControl:
        """
        Align robot heading with the desired orientation using proportional control.
        """
        print("In compute_heading_control")
        self.get_logger().info("In compute_heading_control")

        # Compute heading error wrapped between [-pi, pi]
        heading_error = wrap_angle(goal_state.theta - current_state.theta)

        # Proportional control on angular velocity
        omega_needed = self.kp * heading_error
        
        # Generate control message
        output_control = TurtleBotControl()
        output_control.omega = omega_needed
        return output_control
    
    def compute_trajectory_tracking_control(self, 
        state: TurtleBotState,
        plan: TrajectoryPlan,
        t: float
    ) -> TurtleBotControl:
        """Compute control commands to track a smooth spline trajectory."""
        print("In compute_trajectory_tracking_control")
        self.get_logger().info("In compute_trajectory_tracking_control")

        # Compute time step since last control update
        dt = t - self.t_prev

        # Desired trajectory from the spline representation
        x_d = splev(t, plan.path_x_spline, der=0)
        y_d = splev(t, plan.path_y_spline, der=0)
        xd_d = splev(t, plan.path_x_spline, der=1)
        yd_d = splev(t, plan.path_y_spline, der=1)
        xdd_d = splev(t, plan.path_x_spline, der=2)
        ydd_d = splev(t, plan.path_y_spline, der=2)

        ########## Code starts here ##########
        # Avoid division by zero when computing acceleration-based controls
        if abs(self.V_prev) < self.V_PREV_THRESH:
            self.V_prev = self.V_PREV_THRESH

        # Compute current velocity components in world frame
        xd = self.V_prev * np.cos(state.theta)
        yd = self.V_prev * np.sin(state.theta)

        # Virtual control law for desired accelerations (PD control)
        u = np.array([
            xdd_d + self.kpx*(x_d - state.x) + self.kdx*(xd_d - xd),
            ydd_d + self.kpy*(y_d - state.y) + self.kdy*(yd_d - yd)
        ])

        # Jacobian matrix maps control accelerations to linear/angular velocities
        J = np.array([
            [np.cos(state.theta), -self.V_prev*np.sin(state.theta)],
            [np.sin(state.theta),  self.V_prev*np.cos(state.theta)]
        ])

        # Solve for actual control inputs: linear acceleration (a) and angular velocity (om)
        a, om = linalg.solve(J, u)

        # Integrate acceleration to get new forward velocity
        V = self.V_prev + a*dt
        ########## Code ends here ##########

        # Update internal state for next iteration
        self.t_prev = t
        self.V_prev = V
        self.om_prev = om

        # Construct and return control command
        output_control = TurtleBotControl()
        output_control.omega = om
        output_control.v = V
        return output_control

    def compute_trajectory_plan(self,
        state: TurtleBotState,
        goal: TurtleBotState,
        occupancy: StochOccupancyGrid2D,
        resolution: float,
        horizon: float,
    ) -> T.Optional[TrajectoryPlan]:
        """Plan a smooth trajectory from current state to goal using A* and cubic spline smoothing."""
        print("In compute_trajectory_plan")
        self.get_logger().info("In compute_trajectory_plan")
        
        # Define start and goal states
        x_init = (state.x, state.y)
        x_goal = (goal.x, goal.y)

        # Define search space bounds
        statespace_lo = (-1 * horizon + state.x, -1 * horizon + state.y)
        statespace_hi = (     horizon + state.x,      horizon + state.y)

        self.get_logger().info("Initialize A* planner")

        # Initialize A* planner
        astar = AStar(statespace_lo, statespace_hi, x_init, x_goal, occupancy, resolution)

        self.get_logger().info("Solving A* planner")

        # Execute A* search and handle failure
        if not astar.solve() or len(astar.path) < 4:
            self.get_logger().info("No path found! (This is normal, try re-running the block above)")
            return None
        
        self.get_logger().info("ASTAR Done")
        
        # Reset previous velocity and time values
        self.reset()
        ts = None
        path_x_spline = None
        path_y_spline = None

        # Convert found path to array form
        path = np.array(astar.path)
        x = path[:, 0]
        y = path[:, 1]

        # Compute distances and parameterize by time assuming constant speed
        dx = np.diff(x)
        dy = np.diff(y)
        distances = np.sqrt(dx**2 + dy**2)
        dt = distances / self.v_desired
        ts = np.concatenate(([0], np.cumsum(dt)))
        
        # Fit smoothing splines for x(t) and y(t)
        path_x_spline = splrep(ts, x, s=self.spline_alpha)
        path_y_spline = splrep(ts, y, s=self.spline_alpha)
        
        ###### YOUR CODE END HERE ######
        
        # Return trajectory plan object
        return TrajectoryPlan(
            path=path,
            path_x_spline=path_x_spline,
            path_y_spline=path_y_spline,
            duration=ts[-1],
        )


# ---------------------------
# MAIN EXECUTION ENTRY POINT
# ---------------------------
if __name__ == "__main__":
    rclpy.init()            # Initialize ROS2 client library
    node = Navigator()      # Create Navigator node instance
    rclpy.spin(node)        # Keep node alive and responsive to callbacks
    rclpy.shutdown()        # Graceful shutdown after node exits