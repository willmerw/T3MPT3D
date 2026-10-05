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

class PointMPC(Node):
    def __init__(self):
        super().__init__('turtle_mpc_node')

        self.cmd_vel_publisher1 = self.create_publisher(TwistStamped, '/tb3_1/cmd_vel', 10)
        self.cmd_vel_publisher2 = self.create_publisher(TwistStamped, '/tb3_2/cmd_vel', 10)
        self.mpc_traj_publisher1 = self.create_publisher(Path, '/tb3_1/mpc_trajectory', 10)
        self.mpc_traj_publisher2 = self.create_publisher(Path, '/tb3_2/mpc_trajectory', 10)
        self.traj_publisher = self.create_publisher(Path, '/trajectory', 10)

        self.timer = self.create_timer(0.1, self.timer_callback)

        self.create_subscription(Odometry, '/tb3_1/odom', self.odom_callback1, 10)
        self.create_subscription(Odometry, '/tb3_2/odom', self.odom_callback2, 10)
        self.create_subscription(Bool, '/tb3_1/found_goal', self.found_goal_callback1, 10)
        self.create_subscription(Bool, '/tb3_2/found_goal', self.found_goal_callback2, 10)
        self.create_subscription(Bool, '/model', self.turn_off_callback, 10)
        self.create_subscription(Path, '/tb3_1/goal_trajectory', self.path_callback1, 10)
        self.create_subscription(Path, '/tb3_2/goal_trajectory', self.path_callback2, 10)
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

        self.traj1 = []
        self.traj2 = []

        self.latest_map = None

        self.model1 = self.defineTBotModel()
        self.model2 = self.defineTBotModel()
        self.model1.setup()
        self.model2.setup()

        self.found_goal1 = False
        self.found_goal2 = False
        self.stopped1 = False
        self.stopped2 = False
        self.turn_off = False

        ts = 0.1
        N = 20 # Horizon

        self.pred1 = np.zeros((N + 1, 2))
        self.pred2 = np.zeros((N + 1, 2))

        self.mpc1 = self.defineTBotMPC(self.model1, ts, N)
        self.mpc2 = self.defineTBotMPC(self.model2, ts, N)
        self.mpc1.settings.supress_ipopt_output = True
        self.mpc2.settings.supress_ipopt_output = True
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

        # State Bounds
        mpc.bounds['lower','_x','x'] = -10.0
        mpc.bounds['upper','_x','x'] =  10.0
        mpc.bounds['lower','_x','y'] = -10.0
        mpc.bounds['upper','_x','y'] =  10.0

        # Input Constraints
        mpc.bounds['lower','_u','vx'] = 0.0
        mpc.bounds['upper','_u','vx'] =  0.5
        mpc.bounds['lower','_u','vt'] = -0.8
        mpc.bounds['upper','_u','vt'] =  0.8


        safe_distance = 0.3

        mpc.set_nl_cons(
            'other_robot',
            safe_distance**2 - (
                (model.x['x'] - model.tvp['other_x'])**2 +
                (model.x['y'] - model.tvp['other_y'])**2
            ),
            ub=0.0
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

            for k in range(N + 1):

                # Other robot
                tvp_template['_tvp', k, 'other_x'] = pred[k, 0]
                tvp_template['_tvp', k, 'other_y'] = pred[k, 1]

                # Reference trajectory
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

    def closest_occupied(self):
        pass

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

    def found_goal_callback1(self, msg):
        if msg.data:
            self.found_goal1 = True

    def found_goal_callback2(self, msg):
        if msg.data:
            new_cmd = TwistStamped()
            new_cmd.twist.linear.x = 0.0
            new_cmd.twist.angular.z = 0.0
            self.cmd_vel_publisher2.publish(new_cmd)
            self.found_goal2 = True

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
            return None

        if self.found_goal1:
            if self.stopped1:
                pass
            else:
                new_cmd = TwistStamped()
                new_cmd.twist.linear.x = 0.0
                new_cmd.twist.angular.z = 0.0
                self.cmd_vel_publisher1.publish(new_cmd)
                self.stopped1 = True
        else:
            p0 = np.array([self.x1, self.y1, self.yaw1])
            self.mpc1.x0 = p0
            p = p0.copy()

            self.traj1.append([self.x1, self.y1])
            traj_msg = Path()
            traj_msg.header.frame_id = 'odom'
            for x,y in self.traj1:
                pose = PoseStamped()
                pose.header = traj_msg.header
                pose.pose.position.x = float(x)
                pose.pose.position.y = float(y)
                pose.pose.position.z = 0.0
                traj_msg.poses.append(pose)

            self.mpc_traj_publisher1.publish(traj_msg)

            u = self.mpc1.make_step(p)

            x_pred = self.mpc1.data.prediction(('_x', 'x')).flatten()
            y_pred = self.mpc1.data.prediction(('_x', 'y')).flatten()
            self.pred1 = np.column_stack((x_pred, y_pred))

            path_msg = Path()
            path_msg.header.stamp = self.get_clock().now().to_msg()
            path_msg.header.frame_id = 'odom'

            for x, y in zip(x_pred, y_pred):
                pose = PoseStamped()
                pose.header = path_msg.header
                pose.pose.position.x = float(x)
                pose.pose.position.y = float(y)
                path_msg.poses.append(pose)
            self.mpc_traj_publisher1.publish(path_msg)

            new_cmd = TwistStamped()
            new_cmd.twist.linear.x = u[0][0]
            new_cmd.twist.angular.z = u[1][0]
            self.cmd_vel_publisher1.publish(new_cmd)

        if self.found_goal2 and not self.stopped2:
            if self.stopped2:
                pass
            else:
                new_cmd = TwistStamped()
                new_cmd.twist.linear.x = 0.0
                new_cmd.twist.angular.z = 0.0
                self.cmd_vel_publisher2.publish(new_cmd)
                self.stopped2 = True
        else:
            p0 = np.array([self.x2, self.y2, self.yaw2])
            self.mpc2.x0 = p0
            p = p0.copy()

            self.traj2.append([self.x2, self.y2])
            traj_msg = Path()
            traj_msg.header.frame_id = 'odom'
            for x,y in self.traj2:
                pose = PoseStamped()
                pose.header = traj_msg.header
                pose.pose.position.x = float(x)
                pose.pose.position.y = float(y)
                pose.pose.position.z = 0.0
                traj_msg.poses.append(pose)

            self.mpc_traj_publisher2.publish(traj_msg)

            u = self.mpc2.make_step(p)

            x_pred = self.mpc2.data.prediction(('_x', 'x')).flatten()
            y_pred = self.mpc2.data.prediction(('_x', 'y')).flatten()
            self.pred2 = np.column_stack((x_pred, y_pred))

            path_msg = Path()
            path_msg.header.stamp = self.get_clock().now().to_msg()
            path_msg.header.frame_id = 'odom'

            for x, y in zip(x_pred, y_pred):
                pose = PoseStamped()
                pose.header = path_msg.header
                pose.pose.position.x = float(x)
                pose.pose.position.y = float(y)
                path_msg.poses.append(pose)
            self.mpc_traj_publisher2.publish(path_msg)

            new_cmd = TwistStamped()
            new_cmd.twist.linear.x = u[0][0]
            new_cmd.twist.angular.z = u[1][0]
            self.cmd_vel_publisher2.publish(new_cmd)

def main(): # Main Function
    rclpy.init()
    node = PointMPC()
    rclpy.spin(node) # Keep the node alive and runnings.
    node.destroy_node() # Destroy the node after completion (typically when exited).
    rclpy.shutdown() # Shutdown the ros client on interruption.


if __name__=='__main__': # Entry point
    main()