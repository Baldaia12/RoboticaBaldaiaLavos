"""Labwork 3 - velocity control of the UFACTORY Lite 6 along a circle, as a ROS 2 node.

THIS IS THE ONLY FILE YOU EDIT IN LABWORK 3.

Implement the following, replacing every ``raise NotImplementedError``:

    Part A - the task
        circle_reference(t, c, r, alpha_dot)      desired pose and velocity at time t

    Part B - the control
        jacobian(q)                               the 6x6 Jacobian of {6} in {0}
        open_loop_step(q, V_d)                    joint velocities, open loop
        closed_loop_step(q, T_d, V_d, kp)         joint velocities, closed loop
        VelocityNode                              the node that moves the arm

    The constants KP, DT, QDOT_MAX and SIGMA_MIN below are chosen by the group
    (Part B and analysis E.2).

Run the node with ``ros2 run lite6_velocity vel_node`` (in Gazebo add
``--ros-args -p use_sim_time:=true``).

``pose_error``, ``rotation_vector``, ``load_circle``, ``JOINT_NAMES``,
``TRAJECTORY_TOPIC`` and ``main`` below are PROVIDED: already implemented, do
not edit them.

Keep the function and class names, arguments and return types exactly as they
are: the instructor's grader uses them by name.

Conventions - those of Labworks 1 and 2:

    * Frame {0} coincides with the ROS frame ``link_base``; frame {6} with
      ``link_eef``.
    * Joint angles in RADIANS, in the controller's convention, as published on
      /joint_states; joint velocities in rad/s.
    * Lengths in METRES, linear velocities in m/s, angular velocities in rad/s,
      time in seconds.

Allowed imports: ``numpy`` and ``math`` for the maths; ``forward_kinematics``
from your own Labwork 1 and ``inverse_kinematics`` and ``select_solution``
from your own Labwork 2 (already imported below); and, for the node,
``rclpy``, ``sensor_msgs``, ``geometry_msgs``, ``std_msgs``,
``trajectory_msgs`` and ``builtin_interfaces`` (already imported below).
Nothing else - no MoveIt, no kinematics library, no URDF parser, no tf.
"""

import math
import os

import numpy as np

from lite6_kinematics.forward_kinematics import forward_kinematics
from lite6_ik.inverse_kinematics import inverse_kinematics, select_solution

try:
    import rclpy
    from builtin_interfaces.msg import Duration
    from geometry_msgs.msg import PoseStamped
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
except ImportError:         # the maths also runs without ROS
    rclpy = None
    Node = object

JOINT_NAMES = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
BASE_FRAME = 'link_base'
TRAJECTORY_TOPIC = '/lite6_traj_controller/joint_trajectory'

# --- chosen by the group (Part B, analysis E.2) -------------------------------
# Values backed by sampling C1, C2 and C3 (361 points each): on C1/C2 the
# smallest singular value of J stays ~0.13 and the joint speed needed stays
# ~0.43 rad/s; C3 passes near the base singularity (sigma down to ~0.03, joint
# speed up to ~1.27 rad/s and points with no in-range solution).
# DT = 0.02 s (50 Hz): the circle (alpha_dot = pi/2) lasts 4 s per lap, so a lap
#   is 200 control steps - smooth, and well within what the controller accepts.
# KP = 5.0 1/s: closed-loop error decays as e_{k+1} ~ (1 - KP*DT) e_k; with
#   DT = 0.02, KP*DT = 0.1 << 2, so the loop is stable and well damped, and the
#   error time constant 1/KP = 0.2 s corrects drift in a few steps. (E.2 sweeps
#   KP and DT and confirms the stable region.)
# QDOT_MAX = 0.8 rad/s: ~1.9x the 0.43 rad/s that C1/C2 need (they run), well
#   below the 1.27 rad/s of C3 (it trips 'too_fast'). Keep it below the robot's
#   reduced-mode joint speed limit; lower it if that limit is under 0.8 rad/s.
# SIGMA_MIN = 0.05: below the ~0.13 seen on C1/C2 (they run) and above the ~0.03
#   reached on C3, so C3 trips 'singular' before the Jacobian becomes unusable.
KP = 5.0                # 1/s   gain of the closed loop
DT = 0.02               # s     control period
QDOT_MAX = 0.8          # rad/s largest joint velocity the node may command
SIGMA_MIN = 0.05        # smallest singular value of J below which the node stops


# =============================================================================
# PROVIDED - do not edit
# =============================================================================
def rotation_vector(R) -> np.ndarray:
    """Rotation vector phi(R) = theta * u of a 3x3 rotation matrix.

    u is the unit axis of the rotation and theta in [0, pi] its angle, so the
    result has norm theta; the zero vector for the identity. Expressed in the
    frame in which R is expressed.
    """
    R = np.asarray(R, dtype=float)
    v = 0.5 * np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    s = np.linalg.norm(v)                   # sin(theta)
    c = 0.5 * (np.trace(R) - 1.0)           # cos(theta)
    theta = math.atan2(s, c)
    if s > 1e-6 or (s > 0.0 and c > 0.0):
        return (theta / s) * v
    if c > 0.0:                             # R is the identity
        return v
    M = 0.5 * (R + np.eye(3))               # theta = pi: R = 2 u u^T - I
    i = int(np.argmax(np.diag(M)))
    u = M[:, i] / math.sqrt(M[i, i])
    return math.pi * u


def pose_error(T_c, T_d) -> np.ndarray:
    """Error between a current pose T_c and a desired pose T_d, both (4, 4) in {0}.

    Returns the (6,) array [p_d - p_c ; phi(R_d R_c^T)]: the vector from the
    current to the desired position (m), then the rotation vector (rad) of the
    rotation that takes the current orientation to the desired one, expressed
    in {0}.
    """
    T_c = np.asarray(T_c, dtype=float)
    T_d = np.asarray(T_d, dtype=float)
    e = np.zeros(6)
    e[:3] = T_d[:3, 3] - T_c[:3, 3]
    e[3:] = rotation_vector(T_d[:3, :3] @ T_c[:3, :3].T)
    return e


def load_circle(name):
    """(c, r) of circle ``name`` in config/lw3_circles.yaml: centre (3,) in m, radius in m."""
    import yaml
    try:
        from ament_index_python.packages import get_package_share_directory
        path = os.path.join(get_package_share_directory('lite6_velocity'), 'config', 'lw3_circles.yaml')
    except Exception:       # outside ROS: the source tree
        path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                            'config', 'lw3_circles.yaml')
    with open(path) as f:
        circles = yaml.safe_load(f)['circles']
    if name not in circles:
        raise KeyError(f'circle {name!r} is not in {path}: {sorted(circles)}')
    return np.asarray(circles[name]['center'], dtype=float), float(circles[name]['radius'])


# =============================================================================
# Part A - the task
# =============================================================================
def circle_reference(t, c, r, alpha_dot):
    """Desired pose and velocity of {6} at time t on the circle of Part A.

    Args:
        t:         time since the start of the circle (s).
        c:         (3,) centre of the circle in {0} (m).
        r:         radius (m).
        alpha_dot: angular rate of the point on the circle (rad/s).

    Returns:
        (T_d, V_d): T_d the (4, 4) desired pose of {6} in {0}; V_d the (6,)
        desired velocity [v; w] in {0}: v (m/s) the linear velocity of the
        origin of {6}, w (rad/s) its angular velocity.
    """
    c = np.asarray(c, dtype=float).reshape(3)
    alpha = alpha_dot * t                       # angle travelled on the circle
    ca, sa = math.cos(alpha), math.sin(alpha)

    # Desired pose: a point on the horizontal circle, with a fixed orientation
    # (tool pointing down: roll pi, pitch 0, yaw 0 -> R = diag(1, -1, -1)).
    T_d = np.eye(4)
    T_d[:3, :3] = np.array([[1.0,  0.0,  0.0],
                            [0.0, -1.0,  0.0],
                            [0.0,  0.0, -1.0]])
    T_d[:3, 3] = c + r * np.array([ca, sa, 0.0])

    # Desired velocity: tangent to the circle (derivative of p_d), no rotation.
    v = r * alpha_dot * np.array([-sa, ca, 0.0])
    w = np.zeros(3)
    V_d = np.concatenate([v, w])
    return T_d, V_d


# =============================================================================
# Part B - the control
# =============================================================================
def jacobian(q) -> np.ndarray:
    """The Jacobian J(q) of {6}, expressed in {0}.

    Args:
        q: six joint angles (rad), order joint1..joint6.

    Returns:
        (6, 6) array J such that, for the joint velocities dq (rad/s),
        [v; w] = J(q) @ dq, where v (rows 0-2) is the linear velocity (m/s) of
        the origin of {6} and w (rows 3-5) its angular velocity (rad/s), both
        expressed in {0}. How J is obtained is the group's choice.
    """
    # Geometric Jacobian from the Labwork 1 frames. forward_kinematics with
    # all_frames gives [^0_0 T, ^0_1 T, ..., ^0_6 T], every frame in {0}.
    # Every joint of the Lite 6 is revolute, so joint i (i = 1..6), whose axis
    # is the z-axis of frame {i-1}, contributes the column
    #     [ z_{i-1} x (p_6 - o_{i-1}) ;  z_{i-1} ],
    # with z_{i-1} the joint axis, o_{i-1} its frame origin and p_6 the
    # end-effector origin, all expressed in {0}.
    frames = forward_kinematics(q, all_frames=True)
    p_e = frames[6][:3, 3]
    J = np.zeros((6, 6))
    for i in range(6):
        z = frames[i][:3, 2]                    # axis of joint i+1, in {0}
        o = frames[i][:3, 3]                    # origin of frame {i}, in {0}
        J[:3, i] = np.cross(z, p_e - o)         # linear velocity part
        J[3:, i] = z                            # angular velocity part
    return J


def open_loop_step(q, V_d) -> np.ndarray:
    """Joint velocities of the open-loop law of Part B.

    Args:
        q:   six joint angles (rad), the current configuration.
        V_d: (6,) desired velocity [v; w] in {0} (m/s, rad/s).

    Returns:
        (6,) joint velocities (rad/s).
    """
    # Open loop: invert the velocity map [v; w] = J(q) dq, so dq = J(q)^-1 V_d.
    # np.linalg.solve is used instead of forming the inverse explicitly.
    J = jacobian(q)
    return np.linalg.solve(J, np.asarray(V_d, dtype=float))


def closed_loop_step(q, T_d, V_d, kp) -> np.ndarray:
    """Joint velocities of the closed-loop law of Part B.

    Args:
        q:   six joint angles (rad), the current configuration.
        T_d: (4, 4) desired pose of {6} in {0}.
        V_d: (6,) desired velocity [v; w] in {0} (m/s, rad/s).
        kp:  gain (1/s).

    Returns:
        (6,) joint velocities (rad/s). The error is pose_error(), which is
        provided.
    """
    # Closed loop: add a correction proportional to the pose error so that
    # drift (from discretisation, model error or disturbances) is pulled back
    # instead of accumulating. With T_c = ^0_6 T(q) the current pose,
    #     dq = J(q)^-1 ( V_d + kp * pose_error(T_c, T_d) ).
    J = jacobian(q)
    T_c = forward_kinematics(q)
    e = pose_error(T_c, T_d)
    return np.linalg.solve(J, np.asarray(V_d, dtype=float) + kp * e)


# =============================================================================
# Part B - the node
# =============================================================================
def _quaternion_xyzw(R):
    """Unit quaternion (x, y, z, w) of a 3x3 rotation matrix, for the reference pose."""
    R = np.asarray(R, dtype=float)
    t = np.trace(R)
    if t > 0.0:
        s = math.sqrt(t + 1.0) * 2.0
        w, x, y, z = 0.25 * s, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        w, x, y, z = (R[2, 1] - R[1, 2]) / s, 0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        w, x, y, z = (R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        w, x, y, z = (R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s
    n = math.sqrt(x * x + y * y + z * z + w * w)
    return float(x / n), float(y / n), float(z / n), float(w / n)


class VelocityNode(Node):
    """Moves the end-effector along a circle by velocity control.

    ROS parameters (name, default):
        circle      'C1'        a circle of config/lw3_circles.yaml (load_circle())
        center      [0, 0, 0]   centre (m) of a circle of the group's own; used,
        radius      0.0         with radius (m), when radius > 0
        alpha_dot   pi/2        rad/s
        law         'closed'    'open' or 'closed'
        kp          KP          1/s
        dt          DT          s, the control period
        laps        3
        qdot_max    QDOT_MAX    rad/s
        sigma_min   SIGMA_MIN
        start_time  5.0         s, duration of the motion to the start of the circle

    Subscribes   /joint_states         sensor_msgs/JointState (joints picked by name)
    Publishes    TRAJECTORY_TOPIC      trajectory_msgs/JointTrajectory
                 /lite6_vel/reference  geometry_msgs/PoseStamped, frame BASE_FRAME
                 /lite6_vel/status     std_msgs/String, durability TRANSIENT_LOCAL

    Behaviour, with every time taken from the node's clock:
        1. When the first joint angles are received, it selects, with the
           group's Labwork 2 functions, the solution for the start of the
           circle (t = 0) nearest to the current angles, sends it as a
           trajectory of one point reached in start_time, and publishes the
           status 'moving_to_start'. With no executable solution it publishes
           'unreachable' and does nothing more.
        2. Once start_time has elapsed and every joint is within 1e-2 rad of
           that solution, it publishes 'running'; t = 0 is that instant.
        3. Every dt seconds, with q the latest angles received on
           /joint_states: it publishes T_d(t) on /lite6_vel/reference;
           if the smallest singular value of jacobian(q) is below sigma_min it
           publishes 'singular' and stops; it computes the joint velocities
           dq with the law chosen; if any |dq_i| exceeds qdot_max it publishes
           'too_fast' and stops; otherwise it sends q + dt * dq as a trajectory
           of one point reached in dt.
        4. After laps full turns of the circle it publishes 'done' and stops.
    Stopping means sending nothing more: the controller holds the last target.
    """

    def __init__(self):
        super().__init__('vel_node')

        # --- parameters (handout defaults) ---------------------------------
        self.declare_parameter('circle', 'C1')
        self.declare_parameter('center', [0.0, 0.0, 0.0])
        self.declare_parameter('radius', 0.0)
        self.declare_parameter('alpha_dot', math.pi / 2)
        self.declare_parameter('law', 'closed')
        self.declare_parameter('kp', float(KP))
        self.declare_parameter('dt', float(DT))
        self.declare_parameter('laps', 3)
        self.declare_parameter('qdot_max', float(QDOT_MAX))
        self.declare_parameter('sigma_min', float(SIGMA_MIN))
        self.declare_parameter('start_time', 5.0)

        value = lambda name: self.get_parameter(name).value
        radius = float(value('radius'))
        if radius > 0.0:            # a circle of the group's own
            self.c = np.asarray(value('center'), dtype=float).reshape(3)
            self.r = radius
        else:                       # a named circle of config/lw3_circles.yaml
            self.c, self.r = load_circle(str(value('circle')))
        self.alpha_dot = float(value('alpha_dot'))
        self.law = str(value('law'))
        self.kp = float(value('kp'))
        self.dt = float(value('dt'))
        self.laps = int(value('laps'))
        self.qdot_max = float(value('qdot_max'))
        self.sigma_min = float(value('sigma_min'))
        self.start_time = float(value('start_time'))
        self.total_time = self.laps * 2.0 * math.pi / self.alpha_dot

        # --- state ---------------------------------------------------------
        self.q = None               # latest joint angles from /joint_states
        self.q_start = None         # target for the start of the circle
        self.phase = 'wait_first'   # wait_first -> moving_to_start -> running -> done
        self.status = None          # last status published (publish only on change)
        self.t_cmd = None           # clock time the start trajectory was sent
        self.t0 = None              # clock time the circle started (t = 0)

        # --- ROS I/O -------------------------------------------------------
        status_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.traj_pub = self.create_publisher(JointTrajectory, TRAJECTORY_TOPIC, 10)
        self.ref_pub = self.create_publisher(PoseStamped, '/lite6_vel/reference', 10)
        self.status_pub = self.create_publisher(String, '/lite6_vel/status', status_qos)
        self.create_subscription(JointState, '/joint_states', self.on_joint_state, 10)
        self.timer = self.create_timer(self.dt, self.on_timer)

    # -----------------------------------------------------------------------
    # helpers
    # -----------------------------------------------------------------------
    def clock_s(self) -> float:
        """Current time in seconds from the node's clock (sim or real)."""
        return self.get_clock().now().nanoseconds * 1e-9

    def set_status(self, text) -> None:
        """Publish the status, only when it changes."""
        if text != self.status:
            self.status = text
            msg = String()
            msg.data = text
            self.status_pub.publish(msg)
            self.get_logger().info(f'status: {text}')

    def send_target(self, q_target, duration) -> None:
        """Send one trajectory point, to be reached in `duration` seconds."""
        point = JointTrajectoryPoint()
        point.positions = [float(v) for v in q_target]
        sec = int(duration)
        point.time_from_start = Duration(sec=sec, nanosec=int((duration - sec) * 1e9))
        traj = JointTrajectory()
        traj.joint_names = JOINT_NAMES
        traj.points = [point]
        self.traj_pub.publish(traj)

    def publish_reference(self, T_d) -> None:
        """Publish the desired pose on /lite6_vel/reference."""
        msg = PoseStamped()
        msg.header.frame_id = BASE_FRAME
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = float(T_d[0, 3])
        msg.pose.position.y = float(T_d[1, 3])
        msg.pose.position.z = float(T_d[2, 3])
        qx, qy, qz, qw = _quaternion_xyzw(T_d[:3, :3])
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        self.ref_pub.publish(msg)

    # -----------------------------------------------------------------------
    # callbacks
    # -----------------------------------------------------------------------
    def on_joint_state(self, msg) -> None:
        """Store the six joint angles, picked by name; start on the first one."""
        positions = dict(zip(msg.name, msg.position))
        if not all(name in positions for name in JOINT_NAMES):
            return                                  # not a state of this arm
        self.q = np.array([positions[name] for name in JOINT_NAMES], dtype=float)
        if self.phase == 'wait_first':
            self.move_to_start()

    def move_to_start(self) -> None:
        """Send the arm to the start of the circle, nearest to its pose now."""
        T_d0, _ = circle_reference(0.0, self.c, self.r, self.alpha_dot)
        solutions = inverse_kinematics(T_d0)
        q_start = select_solution(solutions, self.q) if solutions else None
        if q_start is None:
            self.phase = 'unreachable'
            self.set_status('unreachable')
            return
        self.q_start = q_start
        self.send_target(q_start, self.start_time)
        self.t_cmd = self.clock_s()
        self.phase = 'moving_to_start'
        self.set_status('moving_to_start')

    def on_timer(self) -> None:
        """The control loop, every dt seconds."""
        if self.q is None or self.phase in (
                'wait_first', 'unreachable', 'done', 'singular', 'too_fast'):
            return                                  # nothing to do / already stopped

        if self.phase == 'moving_to_start':
            reached = np.max(np.abs(self.q - self.q_start)) < 1e-2
            if self.clock_s() - self.t_cmd >= self.start_time and reached:
                self.t0 = self.clock_s()            # t = 0 is now
                self.phase = 'running'
                self.set_status('running')
            return

        # phase == 'running'
        t = self.clock_s() - self.t0
        if t >= self.total_time:                    # laps full turns done
            self.set_status('done')
            self.phase = 'done'
            return

        T_d, V_d = circle_reference(t, self.c, self.r, self.alpha_dot)
        self.publish_reference(T_d)

        sigma = np.linalg.svd(jacobian(self.q), compute_uv=False).min()
        if sigma < self.sigma_min:                  # too close to a singularity
            self.set_status('singular')
            self.phase = 'singular'
            return

        if self.law == 'open':
            dq = open_loop_step(self.q, V_d)
        else:
            dq = closed_loop_step(self.q, T_d, V_d, self.kp)

        if np.max(np.abs(dq)) > self.qdot_max:      # would drive a joint too fast
            self.set_status('too_fast')
            self.phase = 'too_fast'
            return

        self.send_target(self.q + self.dt * dq, self.dt)


def main(args=None):
    """PROVIDED. Entry point of ``ros2 run lite6_velocity vel_node``."""
    if rclpy is None:
        raise RuntimeError('ROS 2 (rclpy) is not available')
    rclpy.init(args=args)
    node = VelocityNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
