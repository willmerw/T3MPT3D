import do_mpc # Importing the MPC library
import casadi as ca
from casadi import vertcat, horzcat, DM # Importing specific functions from casadi for symbolic mathematics
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import TwistStamped, Pose, PoseStamped
from nav_msgs.msg import Odometry, Path, OccupancyGrid
import numpy as np
import math

# Node that runs two separate MPC:s
class PathMPC(Node):
    def __init__(self):
        super().__init__('turtle_mpc_node')
        # Name-spaced parameters
        self.declare_parameter('tb1_cmd_vel', '/turtlebot1/cmd_vel')
        self.declare_parameter('tb2_cmd_vel', '/turtlebot2/cmd_vel')
        self.declare_parameter('tb1_odom', '/turtlebot1/odom_world')
        self.declare_parameter('tb2_odom', '/turtlebot2/odom_world')
        self.declare_parameter('rivstart', '/rivstart')
        self.declare_parameter('tb1_path', '/turtlebot1/path')
        self.declare_parameter('tb2_path', '/turtlebot2/path')
        self.declare_parameter('tb1_mpc_traj', '/turtlebot1/mpc_traj')
        self.declare_parameter('tb2_mpc_traj', '/turtlebot2/mpc_traj')
        self.declare_parameter('tb1_traj', '/turtlebot1/traj')
        self.declare_parameter('tb2_traj', '/turtlebot2/traj')

        tb1_cmd_vel:str = self.get_parameter('tb1_cmd_vel').get_parameter_value().string_value
        tb2_cmd_vel:str = self.get_parameter('tb2_cmd_vel').get_parameter_value().string_value
        tb1_odom:str = self.get_parameter('tb1_odom').get_parameter_value().string_value
        tb2_odom:str = self.get_parameter('tb2_odom').get_parameter_value().string_value
        rivstart:str = self.get_parameter('rivstart').get_parameter_value().string_value
        tb1_path:str = self.get_parameter('tb1_path').get_parameter_value().string_value
        tb2_path:str = self.get_parameter('tb2_path').get_parameter_value().string_value
        tb1_mpc_traj:str = self.get_parameter('tb1_mpc_traj').get_parameter_value().string_value
        tb2_mpc_traj:str = self.get_parameter('tb2_mpc_traj').get_parameter_value().string_value
        tb1_traj:str = self.get_parameter('tb1_traj').get_parameter_value().string_value
        tb2_traj:str = self.get_parameter('tb2_traj').get_parameter_value().string_value


        # Publishes one velocity command per robot
        self.cmd_vel_publisher1 = self.create_publisher(TwistStamped, tb1_cmd_vel, 10)
        self.cmd_vel_publisher2 = self.create_publisher(TwistStamped, tb2_cmd_vel, 10)
        self.mpc_traj_publisher1 = self.create_publisher(Path, tb1_mpc_traj, 10)
        self.traj_publisher1 = self.create_publisher(Path, tb1_traj, 10)
        self.mpc_traj_publisher2 = self.create_publisher(Path, tb2_mpc_traj, 10)
        self.traj_publisher2 = self.create_publisher(Path, tb2_traj, 10)

        # Runs every 0.1 seconds, executes the code
        self.timer = self.create_timer(0.1, self.timer_callback)

        # Odom subscribers for both robots
        self.create_subscription(Odometry, tb1_odom, self.odom_callback1, 10)
        self.create_subscription(Odometry, tb2_odom, self.odom_callback2, 10)
        # Rivstart is for shutting down the node and starting plank MPC
        self.create_subscription(Bool, rivstart, self.turn_off_callback, 10)
        # The goal paths for tb1 and tb2
        self.create_subscription(Path, tb1_path, self.path_callback1, 10)
        self.create_subscription(Path, tb2_path, self.path_callback2, 10)
        # The map
        self.create_subscription(OccupancyGrid, 'map', self.map_callback, 10)

        # Params to keep track of poses
        self.x1 = 0.0
        self.y1 = 0.0
        self.yaw1 = 0.0
        self.x2 = 0.0
        self.y2 = 0.0
        self.yaw2 = 0.0

        # To store the paths
        self.path1 = []
        self.path_header1=""
        self.path2 = []
        self.path_header2=""

        # Store travelled paths
        self.traj1 = []
        self.traj2 = []

        self.latest_map = None

        # Defing both models
        self.model1 = self.defineTBotModel()
        self.model2 = self.defineTBotModel()
        self.model1.setup()
        self.model2.setup()

        # Gets switched when rivstart arrives
        self.turn_off = False

        # If either robot has stopped, turn of placing constraint at nearest obstacle
        self.tb1_stopped = False
        self.tb2_stopped = False

        ts = 0.1 # Sampling time
        self.N = 40 # Horizon

        # Lists to store the bots predictions
        self.pred1 = np.zeros((self.N + 1, 2))
        self.pred2 = np.zeros((self.N + 1, 2))

        # Defining both MPC:s
        self.mpc1 = self.defineTBotMPC(self.model1, ts, self.N)
        self.mpc2 = self.defineTBotMPC(self.model2, ts, self.N)
        self.mpc1.setup()
        self.mpc2.setup()

        # Setting initial geuss once, allows warm-up
        self.mpc1.set_initial_guess()
        self.mpc2.set_initial_guess()

    # Define a tbot model
    def defineTBotModel(self):
        model_type = 'continuous'
        model = do_mpc.model.Model(model_type)

        # States
        model.set_variable(var_type='_x', var_name='x', shape=(1,1))
        model.set_variable(var_type='_x', var_name='y', shape=(1,1))
        th = model.set_variable(var_type='_x', var_name='th', shape=(1,1))

        # Inputs
        vx = model.set_variable(var_type='_u', var_name='vx')
        vt = model.set_variable(var_type='_u', var_name='vt')

        # Dynamics
        model.set_rhs('x', vx*ca.cos(th))
        model.set_rhs('y', vx*ca.sin(th))
        model.set_rhs('th', vt)

        # Keeping track of other robot
        model.set_variable(var_type='_tvp', var_name='other_x')
        model.set_variable(var_type='_tvp', var_name='other_y')

        # Closest obstacle
        model.set_variable(var_type='_tvp', var_name='obs_x')
        model.set_variable(var_type='_tvp', var_name='obs_y')

        # Desired path
        model.set_variable('_tvp', 'ref_x')
        model.set_variable('_tvp', 'ref_y')

        return model

    # Define a tbot MPC
    def defineTBotMPC(self, model, ts, N):
        # Creating the MPC
        mpc = do_mpc.controller.MPC(model)
        setup_mpc = {
            'n_horizon': N,
            't_step': ts,
            'n_robust': 1,
            'store_full_solution': True
        }
        mpc.set_param(**setup_mpc)
        # Removing output
        mpc.settings.nlpsol_opts = {
        'ipopt.print_level': 0,   # no iteration table
        'ipopt.sb': 'yes',        # no IPOPT banner
        'print_time': 0,          # no CasADi timing summary
        }

        # Defining cost function
        mterm = (
            (model.x['x'] - model.tvp['ref_x'])**2 +
            (model.x['y'] - model.tvp['ref_y'])**2
        )
        lterm = (
            (model.x['x'] - model.tvp['ref_x'])**2 +
            (model.x['y'] - model.tvp['ref_y'])**2
        )
        mpc.set_objective(mterm=mterm, lterm=lterm)
        mpc.set_rterm(vx=1e-2, vt=1e-2)

        # Input Constraints
        mpc.bounds['lower','_u','vx'] = 0.0
        mpc.bounds['upper','_u','vx'] =  0.2 # 0.2
        mpc.bounds['lower','_u','vt'] = -0.5 # -0.5
        mpc.bounds['upper','_u','vt'] =  0.5 # 0.5

        # Distance to be kept between tbs
        safe_distance = 0.3

        # Dynamic constraint for the other robot
        mpc.set_nl_cons(
            'other_robot',
            safe_distance**2 - (
                (model.x['x'] - model.tvp['other_x'])**2 +
                (model.x['y'] - model.tvp['other_y'])**2
            ),
            ub=0.0
        )

        # Dynamic constraint at the closest occupied cell in the occupancy mao
        mpc.set_nl_cons(
            'closest_obstacle',
            0.2 ** 2 - ((model.x['x'] - model.tvp['obs_x']) ** 2
                        + (model.x['y'] - model.tvp['obs_y']) ** 2),
            ub=0.0,
            soft_constraint=False
        )

        # Template for time-varying parameters
        tvp_template = mpc.get_tvp_template()

        def tvp_fun(t_now):
            # Selecting model
            if model is self.model1:
                pred = self.pred2
                path = self.path1
                x = self.x1
                y = self.y1
            else:
                pred = self.pred1
                path = self.path2
                x = self.x2
                y = self.y2

            # Don't use whole path every time
            start_idx = self.closest_path_index(path, x, y)

            # Compute closest occupied once per call, not per horizon step
            obs_x, obs_y = self.closest_occupied(x, y, self.latest_map)
            # "Removing" constraint by setting far away
            if obs_x == None or self.tb1_stopped or self.tb2_stopped:
                obs_x, obs_y = x + 50.0, y + 50.0
            for k in range(N + 1):
                # Moving other robot constraint along horizon
                tvp_template['_tvp', k, 'other_x'] = pred[k, 0]
                tvp_template['_tvp', k, 'other_y'] = pred[k, 1]

                # Keeping obstacle constraint constant along horizon
                tvp_template['_tvp', k, 'obs_x'] = obs_x
                tvp_template['_tvp', k, 'obs_y'] = obs_y

                # Updating goal
                if len(path) > 0:
                    idx = min(start_idx + k, len(path) - 1)
                    tvp_template['_tvp', k, 'ref_x'] = path[idx][0]
                    tvp_template['_tvp', k, 'ref_y'] = path[idx][1]

            return tvp_template
        mpc.set_tvp_fun(tvp_fun)
        return mpc

    # Finding closest path index
    def closest_path_index(self, path, x, y):
        if not path:
            return 0
        distances = [
            (px - x)**2 + (py - y)**2
            for px, py in path
        ]
        return int(np.argmin(distances))

    # Finding closest occupied cell
    def closest_occupied(self, x, y, grid):
        if grid == None:
            return None, None
        # Converting to shape (h, w)
        grid_array = np.array(grid.data).reshape(grid.info.height, grid.info.width)
        check_distance = 20
        mini_step = 1
        closest_cell = (None, None)
        closest = float('inf')
        # Converting x, y to cell indices
        column = int((x - grid.info.origin.position.x) / grid.info.resolution)
        row = int((y -grid.info.origin.position.y) / grid.info.resolution)
        # Checking in a box around robot
        for i in np.arange(-check_distance, check_distance, mini_step):
            for k in np.arange(-check_distance, check_distance, mini_step):
                r = row + i
                c = column + k
                if (r < 0 or r >= grid.info.height or c < 0 or c >= grid.info.width):
                    continue
                # If occupied
                if grid_array[r][c] == 100:
                    dist = np.sqrt(i**2 + k**2)
                    # If closest obstacle
                    if dist < closest:
                        closest = dist
                        closest_cell = (r, c)
        if closest_cell[0] == None:
            return None, None
        # Convert to robot frame and return
        occ_x = grid.info.origin.position.x + closest_cell[1] * grid.info.resolution
        occ_y = grid.info.origin.position.y + closest_cell[0] * grid.info.resolution
        return occ_x, occ_y

    # Callback for tb1 odom
    def odom_callback1(self, msg):
        self.x1 = msg.pose.pose.position.x
        self.y1 = msg.pose.pose.position.y
        q = msg.pose.pose.orientation

        self.yaw1 = np.arctan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

    # Callback for tb2 odom
    def odom_callback2(self, msg):
        self.x2 = msg.pose.pose.position.x
        self.y2 = msg.pose.pose.position.y
        q = msg.pose.pose.orientation

        self.yaw2 = np.arctan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

    # Turn of if received rivstart
    def turn_off_callback(self, msg):
        if msg.data:
            self.turn_off = True

    # Callback for goal path 1
    def path_callback1(self, msg):
        self.path_header1=msg.header.frame_id
        self.path1=[]
        for point in msg.poses:
            self.path1.append((point.pose.position.x, point.pose.position.y))

    # Callback for goal path 2
    def path_callback2(self, msg):
        self.path_header2=msg.header.frame_id
        self.path2=[]
        for point in msg.poses:
            self.path2.append((point.pose.position.x, point.pose.position.y))

    # Callback to get latest map
    def map_callback(self, msg):
        self.latest_map = msg

    # Runs every 0.1 s. Executes the code
    def timer_callback(self):
        # Stops publishing cmd_vel if rivstart has been sent
        if self.turn_off:
            return
        # Set to true if len(path) == 0, meaning that the bot has found its goal (plank)
        tb3_1_stop = False
        tb3_2_stop = False
        if len(self.path1) == 0:
            # If bot has found plank, keep constraint for other robot constant at this pose
            self.pred1 = np.tile([self.x1, self.y1], (self.N + 1, 1))
            # Publish 0 velocities
            new_cmd = TwistStamped()
            new_cmd.twist.linear.x = 0.0
            new_cmd.twist.angular.z = 0.0
            self.cmd_vel_publisher1.publish(new_cmd)
            tb3_1_stop = True
            self.tb1_stopped = True
        else:
            self.tb1_stopped = False
        if len(self.path2) == 0:
            # If bot has found plank, keep constraint for other robot constant at this pose
            self.pred2 = np.tile([self.x2, self.y2], (self.N + 1, 1))
            # Publish 0 velocities
            new_cmd = TwistStamped()
            new_cmd.twist.linear.x = 0.0
            new_cmd.twist.angular.z = 0.0
            self.cmd_vel_publisher2.publish(new_cmd)
            tb3_2_stop = True
            self.tb2_stopped = True
        else:
            self.tb2_stopped = False
        # only if it hasn't found goal
        if not tb3_1_stop:
            p0 = np.array([self.x1, self.y1, self.yaw1])
            self.mpc1.x0 = p0
            p = p0.copy()

            # For publishing mpc trajectory
            xp, yp = p0[:2].copy()
            self.traj1.append([xp, yp])
            traj_msg = Path()
            traj_msg.header.frame_id = 'world'
            for x,y in self.traj1:
                pose = PoseStamped()
                pose.header = traj_msg.header
                pose.pose.position.x = float(x)
                pose.pose.position.y = float(y)
                pose.pose.position.z = 0.0
                traj_msg.poses.append(pose)

            self.traj_publisher1.publish(traj_msg)

            # Calculate prediction
            u = self.mpc1.make_step(p)
            x_pred = self.mpc1.data.prediction(('_x', 'x')).flatten()
            y_pred = self.mpc1.data.prediction(('_x', 'y')).flatten()

            path_msg = Path()
            path_msg.header.stamp = self.get_clock().now().to_msg()
            path_msg.header.frame_id = 'world'
            for x, y in zip(x_pred, y_pred):
                pose = PoseStamped()
                pose.header = path_msg.header
                pose.pose.position.x = float(x)
                pose.pose.position.y = float(y)
                path_msg.poses.append(pose)
            self.mpc_traj_publisher1.publish(path_msg)
            # Save prediction for other bot constraint
            self.pred1 = np.column_stack((x_pred, y_pred))

            # Publish velocity command
            new_cmd = TwistStamped()
            new_cmd.twist.linear.x = u[0][0]
            new_cmd.twist.angular.z = u[1][0]
            self.cmd_vel_publisher1.publish(new_cmd)

        # Only if it hasn't found goal
        if not tb3_2_stop:
            p0 = np.array([self.x2, self.y2, self.yaw2])
            self.mpc2.x0 = p0
            p = p0.copy()

            # For publishing mpc trajectory
            xp, yp = p0[:2].copy()
            self.traj2.append([xp, yp])
            traj_msg = Path()
            traj_msg.header.frame_id = 'world'
            for x,y in self.traj2:
                pose = PoseStamped()
                pose.header = traj_msg.header
                pose.pose.position.x = float(x)
                pose.pose.position.y = float(y)
                pose.pose.position.z = 0.0
                traj_msg.poses.append(pose)

            self.traj_publisher2.publish(traj_msg)

            # Calculate prediction
            u = self.mpc2.make_step(p)
            x_pred = self.mpc2.data.prediction(('_x', 'x')).flatten()
            y_pred = self.mpc2.data.prediction(('_x', 'y')).flatten()

            # For publishing mpc trajectory
            path_msg = Path()
            path_msg.header.stamp = self.get_clock().now().to_msg()
            path_msg.header.frame_id = 'world'
            for x, y in zip(x_pred, y_pred):
                pose = PoseStamped()
                pose.header = path_msg.header
                pose.pose.position.x = float(x)
                pose.pose.position.y = float(y)
                path_msg.poses.append(pose)
            self.mpc_traj_publisher2.publish(path_msg)
            # Save prediction for other bot constraint
            self.pred2 = np.column_stack((x_pred, y_pred))

            # Publish velocity command
            new_cmd = TwistStamped()
            new_cmd.twist.linear.x = u[0][0]
            new_cmd.twist.angular.z = u[1][0]
            self.cmd_vel_publisher2.publish(new_cmd)

def main(): # Main Function
    rclpy.init()
    node = PathMPC()
    rclpy.spin(node) # Keep the node alive and runnings.
    node.destroy_node() # Destroy the node after completion (typically when exited).
    rclpy.shutdown() # Shutdown the ros client on interruption.


if __name__=='__main__': # Entry point
    main()