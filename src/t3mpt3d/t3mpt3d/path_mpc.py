import do_mpc # Importing the MPC library
import casadi as ca
from casadi import vertcat, horzcat, DM # Importing specific functions from casadi for symbolic mathematics
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import String, Bool
from geometry_msgs.msg import TwistStamped, Pose, PoseStamped
from nav_msgs.msg import Odometry, Path, OccupancyGrid
import numpy as np
import math

class PathMPC(Node):
    def __init__(self):
        super().__init__('turtle_mpc_node')
        self.declare_parameter('tb1_cmd_vel', '/turtlebot1/cmd_vel')
        self.declare_parameter('tb2_cmd_vel', '/turtlebot2/cmd_vel')
        self.declare_parameter('tb1_odom', '/turtlebot1/odom_world')
        self.declare_parameter('tb2_odom', '/turtlebot2/odom_world')
        self.declare_parameter('rivstart', '/rivstart')      
        self.declare_parameter('tb1_path', '/turtlebot1/path')
        self.declare_parameter('tb2_path', '/turtlebot2/path')

        tb1_cmd_vel:str = self.get_parameter('tb1_cmd_vel').get_parameter_value().string_value
        tb2_cmd_vel:str = self.get_parameter('tb2_cmd_vel').get_parameter_value().string_value
        tb1_odom:str = self.get_parameter('tb1_odom').get_parameter_value().string_value
        tb2_odom:str = self.get_parameter('tb2_odom').get_parameter_value().string_value
        rivstart:str = self.get_parameter('rivstart').get_parameter_value().string_value
        tb1_path:str = self.get_parameter('tb1_path').get_parameter_value().string_value
        tb2_path:str = self.get_parameter('tb2_path').get_parameter_value().string_value

        plank_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                               reliability=ReliabilityPolicy.RELIABLE)

        self.cmd_vel_publisher1 = self.create_publisher(TwistStamped, tb1_cmd_vel, 10)
        self.cmd_vel_publisher2 = self.create_publisher(TwistStamped, tb2_cmd_vel, 10)


        self.timer = self.create_timer(0.1, self.timer_callback)

        self.create_subscription(Odometry, tb1_odom, self.odom_callback1, 10)
        self.create_subscription(Odometry, tb2_odom, self.odom_callback2, 10)
        self.create_subscription(Bool, rivstart, self.turn_off_callback, plank_qos)
        self.create_subscription(Path, tb1_path, self.path_callback1, 10)
        self.create_subscription(Path, tb2_path, self.path_callback2, 10)
        self.create_subscription(OccupancyGrid, 'map', self.map_callback, 10)

        self.x1 = 0.0
        self.y1 = 0.0
        self.yaw1 = 0.0
        self.x2 = 0.0
        self.y2 = 0.0
        self.yaw2 = 0.0

        self.path1 = []
        self.path_header1=""
        self.path2 = []
        self.path_header2=""

        self.latest_map = None

        self.model1 = self.defineTBotModel()
        self.model2 = self.defineTBotModel()
        self.model1.setup()
        self.model2.setup()

        self.turn_off = False

        self.tb1_stopped = False
        self.tb2_stopped = False

        ts = 0.1
        self.N = 40 # Horizon

        self.pred1 = np.zeros((self.N + 1, 2))
        self.pred2 = np.zeros((self.N + 1, 2))

        self.mpc1 = self.defineTBotMPC(self.model1, ts, self.N)
        self.mpc2 = self.defineTBotMPC(self.model2, ts, self.N)
        self.mpc1.setup()
        self.mpc2.setup()
        self.mpc1.set_initial_guess()
        self.mpc2.set_initial_guess()


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

    def defineTBotMPC(self, model, ts, N):
        mpc = do_mpc.controller.MPC(model)
        setup_mpc = {
            'n_horizon': N,
            't_step': ts,
            'n_robust': 1,
            'store_full_solution': True
        }
        mpc.set_param(**setup_mpc)
        mpc.settings.nlpsol_opts = {
        'ipopt.print_level': 0,   # no iteration table
        'ipopt.sb': 'yes',        # no IPOPT banner
        'print_time': 0,          # no CasADi timing summary
        }

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
        mpc.bounds['upper','_u','vx'] =  0.1
        mpc.bounds['lower','_u','vt'] = -0.25
        mpc.bounds['upper','_u','vt'] =  0.25


        safe_distance = 0.3

        mpc.set_nl_cons(
            'other_robot',
            safe_distance**2 - (
                (model.x['x'] - model.tvp['other_x'])**2 +
                (model.x['y'] - model.tvp['other_y'])**2
            ),
            ub=0.0
        )

        mpc.set_nl_cons(
            'closest_obstacle',
            0.2 ** 2 - ((model.x['x'] - model.tvp['obs_x']) ** 2
                        + (model.x['y'] - model.tvp['obs_y']) ** 2),
            ub=0.0,
            soft_constraint=False
        )

        tvp_template = mpc.get_tvp_template()

        def tvp_fun(t_now):
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

            start_idx = self.closest_path_index(path, x, y)

            # Compute once per call, not per horizon step
            obs_x, obs_y = self.closest_occupied(x, y, self.latest_map)
            if obs_x == None or self.tb1_stopped or self.tb2_stopped:
                obs_x, obs_y = x + 50.0, y + 50.0
            for k in range(N + 1):
                tvp_template['_tvp', k, 'other_x'] = pred[k, 0]
                tvp_template['_tvp', k, 'other_y'] = pred[k, 1]

                tvp_template['_tvp', k, 'obs_x'] = obs_x
                tvp_template['_tvp', k, 'obs_y'] = obs_y

                if len(path) > 0:
                    idx = min(start_idx + k, len(path) - 1)
                    tvp_template['_tvp', k, 'ref_x'] = path[idx][0]
                    tvp_template['_tvp', k, 'ref_y'] = path[idx][1]

            return tvp_template

        mpc.set_tvp_fun(tvp_fun)


        return mpc

    def closest_path_index(self, path, x, y):
        if not path:
            return 0
        distances = [
            (px - x)**2 + (py - y)**2
            for px, py in path
        ]
        return int(np.argmin(distances))

    def closest_occupied(self, x, y, grid):
        if grid == None:
            return None, None
        grid_array = np.array(grid.data).reshape(grid.info.height, grid.info.width)
        check_distance = 20
        mini_step = 1
        closest_cell = (None, None)
        closest = float('inf')
        column = int((x - grid.info.origin.position.x) / grid.info.resolution)
        row = int((y -grid.info.origin.position.y) / grid.info.resolution)
        for i in np.arange(-check_distance, check_distance, mini_step):
            for k in np.arange(-check_distance, check_distance, mini_step):
                r = row + i
                c = column + k
                if (r < 0 or r >= grid.info.height or c < 0 or c >= grid.info.width):
                    continue
                if grid_array[r][c] == 100:
                    dist = np.sqrt(i**2 + k**2)
                    if dist < closest:
                        closest = dist
                        closest_cell = (r, c)
        if closest_cell[0] == None:
            return None, None
        occ_x = grid.info.origin.position.x + closest_cell[1] * grid.info.resolution
        occ_y = grid.info.origin.position.y + closest_cell[0] * grid.info.resolution
        return occ_x, occ_y

    def odom_callback1(self, msg):
        self.x1 = msg.pose.pose.position.x
        self.y1 = msg.pose.pose.position.y
        q = msg.pose.pose.orientation

        self.yaw1 = np.arctan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

    def odom_callback2(self, msg):
        self.x2 = msg.pose.pose.position.x
        self.y2 = msg.pose.pose.position.y
        q = msg.pose.pose.orientation

        self.yaw2 = np.arctan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

    def turn_off_callback(self, msg):
        if msg.data:
            self.turn_off = True

    def path_callback1(self, msg):
        self.path_header1=msg.header.frame_id
        self.path1=[]
        for point in msg.poses:
            self.path1.append((point.pose.position.x, point.pose.position.y))

    def path_callback2(self, msg):
        self.path_header2=msg.header.frame_id
        self.path2=[]
        for point in msg.poses:
            self.path2.append((point.pose.position.x, point.pose.position.y))

    def map_callback(self, msg):
        self.latest_map = msg

    def timer_callback(self):
        if self.turn_off:
            new_cmd = TwistStamped()
            new_cmd.twist.linear.x = 0.0
            new_cmd.twist.angular.z = 0.0
            self.cmd_vel_publisher1.publish(new_cmd)
            self.cmd_vel_publisher2.publish(new_cmd)
            return
        tb3_1_stop = False
        tb3_2_stop = False
        if len(self.path1) == 0:
            self.pred1 = np.tile([self.x1, self.y1], (self.N + 1, 1))
            new_cmd = TwistStamped()
            new_cmd.twist.linear.x = 0.0
            new_cmd.twist.angular.z = 0.0
            self.cmd_vel_publisher1.publish(new_cmd)
            tb3_1_stop = True
            self.tb1_stopped = True
        if len(self.path2) == 0:
            self.pred2 = np.tile([self.x2, self.y2], (self.N + 1, 1))
            new_cmd = TwistStamped()
            new_cmd.twist.linear.x = 0.0
            new_cmd.twist.angular.z = 0.0
            self.cmd_vel_publisher2.publish(new_cmd)
            tb3_2_stop = True
            self.tb2_stopped = True
        if self.turn_off:
            return None

        if not tb3_1_stop:
            p0 = np.array([self.x1, self.y1, self.yaw1])
            self.mpc1.x0 = p0
            p = p0.copy()

            u = self.mpc1.make_step(p)
            x_pred = self.mpc1.data.prediction(('_x', 'x')).flatten()
            y_pred = self.mpc1.data.prediction(('_x', 'y')).flatten()
            self.pred1 = np.column_stack((x_pred, y_pred))

            new_cmd = TwistStamped()
            new_cmd.twist.linear.x = u[0][0]
            new_cmd.twist.angular.z = u[1][0]
            self.cmd_vel_publisher1.publish(new_cmd)

        if not tb3_2_stop:
            p0 = np.array([self.x2, self.y2, self.yaw2])
            self.mpc2.x0 = p0
            p = p0.copy()

            u = self.mpc2.make_step(p)
            x_pred = self.mpc2.data.prediction(('_x', 'x')).flatten()
            y_pred = self.mpc2.data.prediction(('_x', 'y')).flatten()
            self.pred2 = np.column_stack((x_pred, y_pred))

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