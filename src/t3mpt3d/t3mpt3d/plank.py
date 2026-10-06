import do_mpc # Importing the MPC library
import message_filters
import casadi as ca
from casadi import vertcat, horzcat, DM # Importing specific functions from casadi for symbolic mathematics
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import TwistStamped, Pose, PoseStamped, Twist
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import Odometry, Path
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import numpy as np
import math

class PlankMPC(Node):
    def __init__(self):
        super().__init__('turtle_mpc_node')

        vis_qos = QoSProfile(
                    depth=1,
                    durability=DurabilityPolicy.TRANSIENT_LOCAL,
                    reliability=ReliabilityPolicy.RELIABLE
                )


        self.r1_pub = self.create_publisher(Twist, '/tb3_1/cmd_vel', 10)
        self.r2_pub = self.create_publisher(Twist, '/tb3_2/cmd_vel', 10)
        self.mpc_traj_publisher = self.create_publisher(Path, '/mpc_trajectory', 10)
        self.traj_publisher = self.create_publisher(Path, '/trajectory', 10)
        self.plank_vis_pub = self.create_publisher(Marker, '/plank_vis', vis_qos)
        self.plank_odom_pub = self.create_publisher(Odometry, '/plank_odom', 10)
        self.marker_pub = self.create_publisher(Marker, '/goal', vis_qos)
        self.path_pub = self.create_publisher(Path, '/path', vis_qos)
        self.plank_path_pub = self.create_publisher(MarkerArray, '/plank_path_vis', vis_qos) #Planks on the path

        self.timer = self.create_timer(0.1, self.timer_callback)



        self.odom1_sub = message_filters.Subscriber(
            self, Odometry, '/tb3_1/odom',10
        )
        self.odom2_sub = message_filters.Subscriber(
            self, Odometry, '/tb3_2/odom',10
        )
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.odom1_sub, self.odom2_sub], queue_size=10, slop=0.05
        )
        self.sync.registerCallback(self.odom_callback)

        self.p1 = [0.0, 0.0, 0.0]
        self.p2 = [0.0, 0.0, 0.0]
        self.spawn_offsets = ((-2.0, -0.5), (-2.0, 0.5))

        self.plank = [0.0, 0.0, 0.0, 0.0, -np.pi/2] #x,y,th1,th2,thp

        self.r = 0.5
        self.traj = []

        self.model = definePlankModel()
        self.model.setup()

        self.x_desired = 2.0
        self.y_desired = 2.0
        #self.yaw_desired = -np.pi/2
        self.yaw_desired = 0.0

        self.path = [[-2.0,0.0,-np.pi/2],[1.0,1.0,0.0],[2.0,1.0,-np.pi/2],[2.0,3.0,-np.pi/2]]
        self.path_i=0
        ts = 0.1
        N = 20 # Horizon

        self.mpc = definePlankMPC(self.model, ts, N)

        tvp_template = self.mpc.get_tvp_template()
        def tvp_fun(t_now):
            for k in range(N + 1):
                tvp_template['_tvp', k, 'x_goal'] = self.x_desired
                tvp_template['_tvp', k, 'y_goal'] = self.y_desired
                tvp_template['_tvp', k, 'yaw_goal'] = self.yaw_desired
            return tvp_template

        self.mpc.set_tvp_fun(tvp_fun)
        self.mpc.setup()


    def odom_callback(self, odom1_msg: Odometry, odom2_msg: Odometry):
        offset1_x, offset1_y = self.spawn_offsets[0]
        offset2_x, offset2_y = self.spawn_offsets[1]

        self.p1[0] = odom1_msg.pose.pose.position.x + offset1_x
        self.p1[1] = odom1_msg.pose.pose.position.y + offset1_y
        q1 = odom1_msg.pose.pose.orientation
        self.p1[2] = np.arctan2(
            2.0 * (q1.w * q1.z + q1.x * q1.y),
            1.0 - 2.0 * (q1.y * q1.y + q1.z * q1.z)
        )

        self.p2[0] = odom2_msg.pose.pose.position.x + offset2_x
        self.p2[1] = odom2_msg.pose.pose.position.y + offset2_y
        q2 = odom2_msg.pose.pose.orientation
        self.p2[2] = np.arctan2(
            2.0 * (q2.w * q2.z + q2.x * q2.y),
            1.0 - 2.0 * (q2.y * q2.y + q2.z * q2.z)
        )

        xy1 = np.array([self.p1[0], self.p1[1]])
        xy2 = np.array([self.p2[0], self.p2[1]])
        d = xy2 - xy1
        pxy = xy1 + d/2
        th1 = self.p1[2]
        th2 = self.p2[2]

        thp = np.atan2(d[1],d[0])

        px,py = pxy

        self.plank = px,py,th1,th2,thp
        marker = self.plank_marker(px,py,thp,1337)
        self.plank_vis_pub.publish(marker)
        self.publish_plank_odom()

    def timer_callback(self):
        p0 = np.array(self.plank)

        if self.path_i >= len(self.path):
            stop_cmd = Twist()
            self.r1_pub.publish(stop_cmd)
            self.r2_pub.publish(stop_cmd)
            return
        self.publish_plank_path()
        self.x_desired, self.y_desired, self.yaw_desired = self.path[self.path_i]
        d = np.linalg.norm(np.array([self.x_desired, self.y_desired]) - p0[:2])

        if d < 0.1:
            self.path_i += 1
            if self.path_i >= len(self.path):
                stop_cmd = Twist()
                self.r1_pub.publish(stop_cmd)
                self.r2_pub.publish(stop_cmd)
                return

            self.x_desired, self.y_desired, self.yaw_desired = self.path[self.path_i]

        self.publish_marker(self.x_desired, self.y_desired)

        self.mpc.x0 = p0
        self.mpc.set_initial_guess()

        p = p0.copy()
        xp, yp = p0[:2].copy()
        self.traj.append([xp, yp])
        traj_msg = Path()
        traj_msg.header.frame_id = 'world'
        for x,y in self.traj:
            pose = PoseStamped()
            pose.header = traj_msg.header
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.position.z = 0.0
            traj_msg.poses.append(pose)

        self.traj_publisher.publish(traj_msg)


        u = self.mpc.make_step(p)

        x_pred = self.mpc.data.prediction(('_x', 'x')).flatten()
        y_pred = self.mpc.data.prediction(('_x', 'y')).flatten()
        pred_trajectory = np.hstack([x_pred, y_pred])

        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'world'

        for x, y in zip(x_pred, y_pred):
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            path_msg.poses.append(pose)
        self.mpc_traj_publisher.publish(path_msg)

        r1_cmd = Twist()
        r2_cmd = Twist()

        v1 = u[0][0]
        v2 = u[1][0]
        vth1 = u[2][0]
        vth2 = u[3][0]
        r1_cmd.linear.x = v1
        r1_cmd.angular.z = vth1

        r2_cmd.linear.x = v2
        r2_cmd.angular.z = vth2


        self.r1_pub.publish(r1_cmd)
        self.r2_pub.publish(r2_cmd)

    def plank_marker(self,x,y,thp,id):
        marker = Marker()

        # Frame and timestamp configuration
        marker.header.frame_id = "world"
        marker.header.stamp = self.get_clock().now().to_msg()

        # Namespace and ID to identify the marker
        marker.ns = "lines"
        marker.id = id

        # Type: LINE_STRIP connects points sequentially (0-1, 1-2, 2-3...)
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        # Scale: scale.x controls the line thickness (in meters)
        marker.scale.x = 0.05

        # Color and Alpha (opacity) - Alpha must be > 0 to be visible
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        shift = np.array([np.cos(thp),np.sin(thp)]) * self.r
        p1 = np.array([x,y]) + shift
        p2 = np.array([x,y]) - shift
        pp1 = Point(x=p1[0], y=p1[1], z=0.0)
        pp2 = Point(x=p2[0], y=p2[1], z=0.0)

        marker.points = [pp1, pp2]

        return marker

    def publish_plank_odom(self):
        odom = Odometry()
        x,y,th1,th2,thp = self.plank
        odom.header.frame_id = "map"
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        qx = 0.0
        qy = 0.0
        qz = math.sin(thp / 2.0)
        qw = math.cos(thp / 2.0)
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        self.plank_odom_pub.publish(odom)

    def publish_marker(self, x,y):
        marker = Marker()

        # Frame and timestamp configuration
        marker.header.frame_id = 'world'
        marker.header.stamp = self.get_clock().now().to_msg()

        # Unique namespace/ID pair (use a different ID or ns from the line marker)
        marker.ns = "shapes"
        marker.id = 1

        # Marker types: Marker.SPHERE, Marker.CUBE, Marker.CYLINDER, Marker.ARROW, etc.
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        # Position and Orientation
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0  # Valid quaternion required

        # Scale: Dimensions in meters (x, y, z represent diameter/size)
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.01

        # Color (RGBA)
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        self.marker_pub.publish(marker)

    def publish_plank_path(self):
        path_msg = Path()
        planks = MarkerArray()
        path_msg.header.frame_id = 'world'
        for i,(x,y,thp) in enumerate(self.path):
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.position.z = 0.0
            path_msg.poses.append(pose)
            plank = self.plank_marker(x,y,thp,i)
            plank.ns = "path_planks"
            planks.markers.append(plank)

        self.plank_path_pub.publish(planks)
        self.path_pub.publish(path_msg)


def draw_eight(self, x, y, diameter):
    pts = []
    fid = 10
    for i in range(-180,180,fid):
        rad = math.radians(i)
        x_pos = (x + diameter) + diameter*math.cos(rad)
        y_pos = y + diameter*math.sin(rad)
        pts.append((x_pos,y_pos))
    for i in range(360,0,-fid):
        rad = math.radians(i)
        x_pos = (x - diameter) + diameter*math.cos(rad)
        y_pos = y + diameter*math.sin(rad)
        pts.append((x_pos,y_pos))
    return pts


def definePlankModel():

  model_type = 'continuous'
  model = do_mpc.model.Model(model_type)

  # States
  x = model.set_variable(var_type='_x', var_name='x', shape=(1,1))
  y = model.set_variable(var_type='_x', var_name='y', shape=(1,1))
  th1 = model.set_variable(var_type='_x', var_name='th1', shape=(1,1))
  th2 = model.set_variable(var_type='_x', var_name='th2', shape=(1,1))
  thp = model.set_variable(var_type='_x', var_name='thp', shape=(1,1))
  r = 1.0

  # Inputs
  v1 = model.set_variable(var_type='_u', var_name='v1')
  v2 = model.set_variable(var_type='_u', var_name='v2')
  vth1 = model.set_variable(var_type='_u', var_name='vth1')
  vth2 = model.set_variable(var_type='_u', var_name='vth2')

  model.set_variable(var_type='_tvp', var_name='x_goal')
  model.set_variable(var_type='_tvp', var_name='y_goal')
  model.set_variable(var_type='_tvp', var_name='yaw_goal')

  # Dynamics
  model.set_rhs('x', ((v1*ca.cos(th1)+v2*ca.cos(th2))/2))
  model.set_rhs('y', ((v1*ca.sin(th1)+v2*ca.sin(th2))/2))
  model.set_rhs('th1', vth1)
  model.set_rhs('th2', vth2)
  model.set_rhs('thp', ((v2*ca.cos(th2)-v1*ca.cos(th1)) * -ca.sin(thp) + ((v2*ca.sin(th2) - v1*ca.sin(th1)) * ca.cos(thp)))/(2*r))

  return model

def definePlankMPC(model, ts, N):
  mpc = do_mpc.controller.MPC(model)
  setup_mpc = {
      'n_horizon': N,
      't_step': ts,
      'n_robust': 1,
      'store_full_solution': True
  }
  mpc.set_param(**setup_mpc)

  mterm = (model.x['x'] - model.tvp['x_goal'])**2 + (model.x['y'] - model.tvp['y_goal'])**2
  lterm = (model.x['x'] - model.tvp['x_goal'])**2 + (model.x['y'] - model.tvp['y_goal'])**2 + (model.x['thp'] - model.tvp['yaw_goal'])**2
  mpc.set_objective(mterm=mterm, lterm=lterm)
  mpc.set_rterm(v1=1e-2, v2=1e-2, vth1=1e-2,vth2=1e-2)

  # State Bounds
  #mpc.bounds['lower','_x','x'] = -10.0
  #mpc.bounds['upper','_x','x'] =  10.0
  #mpc.bounds['lower','_x','y'] = -10.0
  #mpc.bounds['upper','_x','y'] =  10.0

  # Input Constraints
  mpc.bounds['lower','_u','v1'] = 0.0
  mpc.bounds['upper','_u','v1'] =  0.2
  mpc.bounds['lower','_u','v2'] = 0.0
  mpc.bounds['upper','_u','v2'] =  0.2
  mpc.bounds['lower','_u','vth1'] = -0.5
  mpc.bounds['upper','_u','vth1'] =  0.5
  mpc.bounds['lower','_u','vth2'] = -0.5
  mpc.bounds['upper','_u','vth2'] =  0.5


  v1_par = model.u['v1'] * ca.cos(model.x['th1'] - model.x['thp'])
  v2_par = model.u['v2'] * ca.cos(model.x['th2'] - model.x['thp'])

  v_diff = v1_par - v2_par

  mpc.set_nl_cons(
    'parallel_movement_ub',
    v_diff,
    ub=0.0,
    soft_constraint=True,
    penalty_term_cons=1e2
  )

  mpc.set_nl_cons(
    'parallel_movement_lb',
    -v_diff,
    ub=0.0,
    soft_constraint=True,
    penalty_term_cons=1e2
  )


  return mpc

def main(): # Main Function
    rclpy.init()
    node = PlankMPC()
    rclpy.spin(node) # Keep the node alive and runnings.
    node.destroy_node() # Destroy the node after completion (typically when exited).
    rclpy.shutdown() # Shutdown the ros client on interruption.


if __name__=='__main__': # Entry point
    main()