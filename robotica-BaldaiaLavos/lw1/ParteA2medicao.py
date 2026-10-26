#!/usr/bin/env python3
"""LW1 Parte A.2 - medicao dos parametros do Lite 6 no Gazebo.

Roda UMA junta de cada vez para +90 graus (as outras a 0) e regista, para cada
configuracao, so o que a ficha permite usar:
    - os angulos de junta efetivamente atingidos   (/joint_states)
    - a pose de link_eef relativamente a link_base  (tf)

Correr DENTRO do contentor, com o Gazebo ligado noutro terminal (lab sim-lite):

    python3 ~/dev_ws/src/lw1_medicao.py

Resultado: ~/shared/lw1_medicoes.csv  (= pasta shared no Windows).
Este ficheiro nao faz parte do pacote lite6_kinematics; e so uma ferramenta de medicao.
"""
import csv
import math
import os
import time

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectoryPoint

JOINTS = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
ANGULO = math.radians(90.0)
CONFIGS = [('zero', [0.0] * 6)] + [
    (f'q{j + 1}=+90', [ANGULO if i == j else 0.0 for i in range(6)]) for j in range(6)]
SAIDA = os.path.expanduser('~/shared/lw1_medicoes.csv')


def quat_para_matriz(x, y, z, w):
    return [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]


class Medidor(Node):
    def __init__(self):
        super().__init__('lw1_medicao',
                         parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.cliente = ActionClient(self, FollowJointTrajectory,
                                    '/lite6_traj_controller/follow_joint_trajectory')
        self.tf = Buffer()
        self.ouvinte = TransformListener(self.tf, self)
        self.js = None
        self.create_subscription(JointState, '/joint_states', self._js, 10)

    def _js(self, msg):
        self.js = msg

    def esperar(self, segundos):
        fim = time.monotonic() + segundos
        while time.monotonic() < fim:
            rclpy.spin_once(self, timeout_sec=0.05)

    def mover(self, q, tempo=4):
        ponto = JointTrajectoryPoint()
        ponto.positions = list(q)
        ponto.velocities = [0.0] * 6
        ponto.time_from_start = Duration(sec=tempo)
        objetivo = FollowJointTrajectory.Goal()
        objetivo.trajectory.joint_names = JOINTS
        objetivo.trajectory.points = [ponto]
        f = self.cliente.send_goal_async(objetivo)
        rclpy.spin_until_future_complete(self, f)
        if not f.result().accepted:
            raise RuntimeError('o controlador rejeitou o movimento')
        f = f.result().get_result_async()
        rclpy.spin_until_future_complete(self, f)
        return f.result().result.error_code

    def medir(self):
        self.esperar(2.0)                     # deixa o braco assentar
        inicio = time.monotonic()
        while True:
            tf_ok = self.tf.can_transform('link_base', 'link_eef', rclpy.time.Time())
            if tf_ok and self.js is not None:
                break
            if time.monotonic() - inicio > 10.0:
                raise RuntimeError(
                    f'sem dados apos 10 s: tf link_base->link_eef={tf_ok}, '
                    f'/joint_states recebido={self.js is not None}. '
                    f'Frames conhecidos: {self.tf.all_frames_as_string()}')
            self.esperar(0.2)
        t = self.tf.lookup_transform('link_base', 'link_eef', rclpy.time.Time()).transform
        q = {n: p for n, p in zip(self.js.name, self.js.position)}
        r = t.rotation
        R = quat_para_matriz(r.x, r.y, r.z, r.w)
        return ([math.degrees(q[n]) for n in JOINTS],
                [1000 * t.translation.x, 1000 * t.translation.y, 1000 * t.translation.z],
                [r.x, r.y, r.z, r.w],
                [R[0][2], R[1][2], R[2][2]])      # direcao do Z de link_eef, em link_base


def main():
    rclpy.init()
    no = Medidor()
    print('a espera do controlador (o Gazebo tem de estar ligado: lab sim-lite)...')
    no.cliente.wait_for_server()
    linhas = []
    for nome, q in CONFIGS:
        codigo = no.mover(q)
        qm, p, quat, z = no.medir()
        linhas.append([nome, *[round(v, 3) for v in qm], *[round(v, 2) for v in p],
                       *[round(v, 5) for v in quat], *[round(v, 3) for v in z], codigo])
        print(f'{nome:8s} q(graus)={[round(v, 2) for v in qm]}  '
              f'p(mm)=[{p[0]:8.2f} {p[1]:8.2f} {p[2]:8.2f}]  Z_eef={[round(v, 2) for v in z]}')
    no.mover([0.0] * 6)
    with open(SAIDA, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['config', 'q1_deg', 'q2_deg', 'q3_deg', 'q4_deg', 'q5_deg', 'q6_deg',
                    'x_mm', 'y_mm', 'z_mm', 'qx', 'qy', 'qz', 'qw', 'zx', 'zy', 'zz', 'error_code'])
        w.writerows(linhas)
    print(f'\nguardado em {SAIDA}')
    no.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()