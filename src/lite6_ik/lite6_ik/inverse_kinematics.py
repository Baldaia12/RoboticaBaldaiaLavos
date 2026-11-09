"""Labwork 2 - algebraic inverse kinematics of the UFACTORY Lite 6, as a ROS 2 node.

THIS IS THE ONLY FILE YOU EDIT IN LABWORK 2.

Implement the following, replacing every ``raise NotImplementedError``:

    Part B - the maths (no ROS needed)
        inverse_kinematics(T_d)                    every distinct solution for a pose
        select_solution(solutions, q_current)      the solution that is executed

    Part B - the node
        IkNode                                     reads a desired pose, publishes
                                                   the selected joint angles

Run the node with ``ros2 run lite6_ik ik_node``.

``transform_from_pose``, ``JOINT_LIMITS``, the constants ``TOL_SAME`` and
``TOL_WRIST`` and ``main`` below are PROVIDED: already implemented, do not edit
them.

Keep the function and class names, arguments and return types exactly as they
are: the checker node (``ik_check``), the study program (``ik_study``), the
tests in ``test/`` and the instructor's grader all use them by name.

Conventions - those of Labwork 1:

    * Frame {0} coincides with the ROS frame ``link_base``; frame {6} with
      ``link_eef``. The desired pose T_d is the pose of {6} in {0}.
    * Joint angles in RADIANS, in the controller's convention, as published on
      /joint_states (q = 0 is the controller's zero configuration).
    * Lengths in METRES.

Allowed imports: ``numpy`` and ``math`` for the maths, ``forward_kinematics``
from your own ``lite6_kinematics.forward_kinematics`` (already imported below),
and ``rclpy``, ``sensor_msgs`` and ``geometry_msgs`` for the node (already
imported below). Nothing else - no MoveIt, no kinematics library, no URDF
parser, no tf. ``test/test_inverse_kinematics.py`` checks the imports.
"""

import math

import numpy as np

from lite6_kinematics.forward_kinematics import forward_kinematics

try:
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
except ImportError:         # the maths and its unit tests also run without ROS
    rclpy = None
    Node = object

JOINT_NAMES = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
BASE_FRAME = 'link_base'


# =============================================================================
# Part B - the maths
# =============================================================================
def inverse_kinematics(T_d) -> list:
    """Every distinct joint vector that puts the end-effector at T_d.

    The solution is closed-form, from the expressions of Part A: no iterative
    or optimisation method.

    Args:
        T_d: (4, 4) desired pose of {6} in {0}; translation in metres.

    Returns:
        A list of (6,) float arrays, joint angles in radians, order
        joint1..joint6, one array per distinct solution (in any order). The
        empty list if T_d is out of the robot's reach.

    Conventions of Part A:
        * every angle is in (-pi, pi]; the joint ranges JOINT_LIMITS are NOT
          applied here: a solution outside them is still returned;
        * two solutions in which no angle differs by more than TOL_SAME
          (angles compared modulo 2 pi) are the same solution: only one of
          them is returned;
        * if the angle between the axes of joints 4 and 6 is smaller than
          TOL_WRIST, those axes are aligned and every solution returned has
          q4 = 0.
    """
    """Calcula todas as configurações distintas que atingem a pose T_d."""
    T_d = np.asarray(T_d, dtype=float)
    R = T_d[:3, :3]

    # Na Lab 1, a ferramenta está a 0.0615 m do centro do punho
    # ao longo do eixo z do referencial final.
    x, y, z = T_d[:3, 3] - 0.0615 * R[:, 2]
    z -= 0.2433  # Altura do ombro na Lab 1.

    upper_arm = 0.200
    forearm = math.hypot(0.087, 0.2276)
    forearm_offset = math.atan2(0.087, -0.2276)
    radius = math.hypot(x, y)

    # Lei dos cossenos no triângulo ombro-cotovelo-punho.
    cosine = ((radius**2 + z**2 - upper_arm**2 - forearm**2)
              / (2.0 * upper_arm * forearm))

    # Uma pequena margem evita rejeitar uma pose exatamente
    # na fronteira do alcance por erro de arredondamento.
    if cosine < -1.0 - 1e-12 or cosine > 1.0 + 1e-12:
        return []

    elbow = math.acos(max(-1.0, min(1.0, cosine)))
    azimuth = math.atan2(y, x)
    two_pi = 2.0 * math.pi
    solutions = []

    def wrap(angle):
        """Coloca o ângulo no intervalo (-pi, pi]."""
        value = (angle + math.pi) % two_pi - math.pi
        return value + two_pi if value <= -math.pi else value

    # Duas direções possíveis para a base.
    for q1, signed_radius in ((azimuth, radius),
                              (azimuth + math.pi, -radius)):

        # Em cada direção, há duas posições possíveis do cotovelo.
        for delta in (elbow, -elbow):
            q3 = forearm_offset - delta
            q2 = (math.atan2(signed_radius, z)
                  - math.atan2(forearm * math.sin(delta),
                               upper_arm + forearm * math.cos(delta)))

            # Orientação resultante das três primeiras juntas,
            # de acordo com a cinemática direta da Lab 1.
            c1, s1 = math.cos(q1), math.sin(q1)
            cv, sv = math.cos(q2 - q3), math.sin(q2 - q3)
            R03 = np.array([
                [c1 * cv,  s1, -c1 * sv],
                [s1 * cv, -c1, -s1 * sv],
                [-sv,       0.0, -cv]
            ])

            # Orientação que as juntas do punho ainda têm de produzir.
            R36 = R03.T @ R
            sine_abs = math.hypot(R36[0, 2], R36[1, 2])

            if sine_abs < TOL_WRIST:
                # Na singularidade, q4 e q6 não ficam determinados
                # individualmente. A convenção do trabalho é q4 = 0.
                if R36[2, 2] > 0.0:
                    wrist_branches = [(
                        0.0, 0.0,
                        math.atan2(R36[1, 0], R36[0, 0])
                    )]
                else:
                    wrist_branches = [(
                        0.0, math.pi,
                        math.atan2(R36[1, 0], -R36[0, 0])
                    )]
            else:
                # Fora da singularidade, o punho tem duas soluções.
                wrist_branches = []
                for sine in (sine_abs, -sine_abs):
                    q4 = math.atan2(-R36[1, 2] / sine,
                                    -R36[0, 2] / sine)
                    q5 = math.atan2(sine, R36[2, 2])
                    q6 = math.atan2(-R36[2, 1] / sine,
                                    R36[2, 0] / sine)
                    wrist_branches.append((q4, q5, q6))

            for q4, q5, q6 in wrist_branches:
                candidate = np.array([
                    wrap(q1), wrap(q2), wrap(q3),
                    wrap(q4), wrap(q5), wrap(q6)
                ])

                # Ângulos que diferem em 2*pi representam
                # a mesma configuração.
                repeated = any(
                    np.all(np.abs((candidate - previous + math.pi)
                                  % two_pi - math.pi) <= TOL_SAME)
                    for previous in solutions
                )
                if not repeated:
                    solutions.append(candidate)

    return solutions


def select_solution(solutions, q_current):
    """The solution that is executed, chosen among `solutions`.

    Args:
        solutions: list of (6,) joint vectors (rad), as inverse_kinematics()
                   returns them. Their angles may be any real numbers: this
                   function must not assume they are in (-pi, pi].
        q_current: six joint angles (rad), the current configuration of the arm.

    Returns:
        A (6,) float array, the selected joint vector, or None if no solution
        is executable (no solutions at all, or all discarded). The inputs are
        not modified.

    The rule is the one of Part A: in each solution, every angle q_i is
    replaced by the value q_i + 2 pi k (k an integer) that lies inside the
    range of joint i (JOINT_LIMITS) and is closest to q_current[i]; a solution
    for which no such value exists is discarded. Among the solutions that
    remain, the one at the smallest euclidean distance from q_current is
    selected, and the returned vector is that solution with the replaced angles.
    """
    """Escolhe a solução permitida mais próxima da configuração atual."""
    q_current = np.asarray(q_current, dtype=float)
    best = None
    best_distance = math.inf
    two_pi = 2.0 * math.pi

    for solution in solutions:
        candidate = np.empty(6, dtype=float)
        valid = True

        for i in range(6):
            angle = float(solution[i])
            lower, upper = JOINT_LIMITS[i]

            # Encontrar os inteiros k que colocam
            # angle + 2*pi*k dentro dos limites da junta.
            k_min = math.ceil((lower - angle) / two_pi)
            k_max = math.floor((upper - angle) / two_pi)

            if k_min > k_max:
                valid = False
                break

            # Escolher o equivalente mais próximo da posição atual.
            k = min(
                range(k_min, k_max + 1),
                key=lambda value: abs(angle + two_pi * value - q_current[i])
            )
            candidate[i] = angle + two_pi * k

        if valid:
            distance = np.linalg.norm(candidate - q_current)
            if distance < best_distance:
                best = candidate
                best_distance = distance

    return best  # None se nenhuma solução estiver dentro dos limites.


# =============================================================================
# Part B - the node
# =============================================================================
class IkNode(Node):
    """Solves the inverse kinematics of every desired pose it receives.

    Subscribes   target_topic         geometry_msgs/PoseStamped  (/lite6_ik/target)
                 joint_states_topic   sensor_msgs/JointState     (/joint_states)
    Publishes    solution_topic       sensor_msgs/JointState     (/lite6_ik/solution)

    The target is the pose of link_eef in link_base (header.frame_id is
    BASE_FRAME). For each target received, the node computes the solutions and
    selects one from the latest joint angles received. If there is a selected
    solution it publishes a JointState:
        name             JOINT_NAMES
        position         the selected solution, in the order of JOINT_NAMES
        header.stamp     the stamp of the TARGET it answers (copy it:
                         ik_check matches the solution to the target by it)
    Otherwise it publishes nothing and logs why:
        unreachable      the pose has no solution at all
        out_of_range     it has solutions, but all of them were discarded

    The topic names are constructor arguments so that the tests can run the
    node on private topics: use them, do not write the names again.
    """

    def __init__(self, target_topic='/lite6_ik/target',
                 joint_states_topic='/joint_states',
                 solution_topic='/lite6_ik/solution'):
        super().__init__('ik_node')

        # Ainda não recebemos os ângulos atuais do robô.
        self.q_current = None

        self.publisher = self.create_publisher(
            JointState, solution_topic, 10
        )
        self.create_subscription(
            JointState, joint_states_topic, self.on_joint_state, 10
        )
        self.create_subscription(
            PoseStamped, target_topic, self.on_target, 10
        )

    def on_joint_state(self, msg):
        """Guarda os últimos ângulos das seis juntas."""
        # A mensagem pode trazer as juntas noutra ordem
        # e também pode conter juntas de outros dispositivos.
        positions = dict(zip(msg.name, msg.position))

        if not all(name in positions for name in JOINT_NAMES):
            return

        self.q_current = np.array(
            [positions[name] for name in JOINT_NAMES], dtype=float
        )

    def on_target(self, msg):
        """Calcula e publica a solução da pose pedida."""
        if self.q_current is None:
            self.get_logger().warning('Ainda não recebi /joint_states')
            return

        T_d = transform_from_pose(msg)
        solutions = inverse_kinematics(T_d)

        if not solutions:
            self.get_logger().warning('unreachable')
            return

        selected = select_solution(solutions, self.q_current)
        if selected is None:
            self.get_logger().warning('out_of_range')
            return

        answer = JointState()
        answer.name = JOINT_NAMES
        answer.position = selected.tolist()

        # O verificador associa a resposta ao pedido por esta data.
        answer.header.stamp = msg.header.stamp
        self.publisher.publish(answer)


# =============================================================================
# PROVIDED - DO NOT EDIT. Already implemented; nothing below is yours to change.
# =============================================================================
TOL_SAME = 1e-6     # rad - two solutions closer than this in every angle are the same
TOL_WRIST = 1e-6    # rad - angle between the axes of joints 4 and 6 below which they are aligned

# Joint ranges (rad) of the xarm_description model of the Lite 6, one row per
# joint, [lower, upper].
JOINT_LIMITS = np.array([
    [-math.pi * 0.99, math.pi * 0.99],
    [-2.61799, 2.61799],
    [-0.061087, math.pi * 0.99],
    [-math.pi * 0.99, math.pi * 0.99],
    [-2.1642, 2.1642],
    [-math.pi * 0.99, math.pi * 0.99],
])


def transform_from_pose(msg) -> np.ndarray:
    """(4, 4) transform of a geometry_msgs/PoseStamped.

    The quaternion is normalised first, so a slightly off unit quaternion still
    gives a rotation matrix.
    """
    p, o = msg.pose.position, msg.pose.orientation
    n = math.sqrt(o.x * o.x + o.y * o.y + o.z * o.z + o.w * o.w)
    x, y, z, w = o.x / n, o.y / n, o.z / n, o.w / n
    T = np.eye(4)
    T[:3, :3] = [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ]
    T[:3, 3] = (p.x, p.y, p.z)
    return T


def main(args=None):
    """Entry point of ``ros2 run lite6_ik ik_node``."""
    if rclpy is None:
        raise SystemExit('ROS 2 is not available here: run this inside the container.')
    rclpy.init(args=args)
    node = None
    status = 0
    try:
        node = IkNode()
        node.get_logger().info('ik_node running: desired poses in, joint angles out')
        rclpy.spin(node)
    except NotImplementedError as e:
        print(f'\n{e}\nImplement it in lite6_ik/inverse_kinematics.py first.')
        status = 1
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return status


if __name__ == '__main__':
    raise SystemExit(main())
