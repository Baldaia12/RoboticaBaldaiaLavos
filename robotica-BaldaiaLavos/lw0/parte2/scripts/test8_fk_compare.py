"""Compara a cinemática direta do SDK com a posição obtida por tf2.

Exemplos, a partir da pasta lw0/parte2:

    python scripts/test8_fk_compare.py --pose A --tf2-xyz 0.087 0.000 0.154
    python scripts/test8_fk_compare.py --pose B --tf2-xyz 0.087 -0.249 0.154

Os três valores de --tf2-xyz vêm de "Translation" no Teste 5 e estão em
metros. O SDK devolve a posição em milímetros e a orientação em graus.

Este script calcula uma pose; não comanda um movimento. No entanto,
safe_session prepara e ativa o controlador ao estabelecer a ligação.
"""

import argparse

import numpy as np
from scipy.spatial.transform import Rotation

from lite6.config import load_config
from lite6.session import check, safe_session


def pose_to_T(pose_mm_deg) -> np.ndarray:
    """Converte a pose do SDK numa matriz homogénea 4×4 em metros.

    Entrada: [x, y, z] em mm e [roll, pitch, yaw] em graus.
    Para ângulos fixos X-Y-Z, a rotação é Rz(yaw) Ry(pitch) Rx(roll).
    """
    if pose_mm_deg is None or len(pose_mm_deg) < 6:
        raise ValueError("A pose do SDK deve conter posição e orientação")

    x, y, z, roll, pitch, yaw = [float(value) for value in pose_mm_deg[:6]]

    transform = np.eye(4)
    transform[:3, :3] = Rotation.from_euler(
        "xyz", [roll, pitch, yaw], degrees=True
    ).as_matrix()
    transform[:3, 3] = np.array([x, y, z]) / 1000.0

    return transform


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--pose",
        choices=("A", "B"),
        default="A",
        help="pose cujos ângulos serão lidos de config/robot.yaml",
    )
    parser.add_argument(
        "--tf2-xyz",
        type=float,
        nargs=3,
        required=True,
        metavar=("X", "Y", "Z"),
        help="translação do tf2_echo para a mesma pose, em metros",
    )
    args = parser.parse_args()

    cfg = load_config()

    # safe_session ativa o controlador, mesmo sem este script comandar
    # movimento. Confirmar antes de o usar com o braço físico.
    if cfg["target"] != "sim":
        answer = input(
            "Robô real: o docente autorizou e há alguém junto da "
            "paragem de emergência? [s/N] "
        )
        if answer.strip().lower() != "s":
            print("Execução cancelada.")
            return

    joints_deg = cfg["test_poses"][f"pose_{args.pose}_joints"]

    with safe_session(cfg) as (arm, _, log):
        # Esta chamada calcula a FK para os ângulos indicados. Não leva
        # fisicamente o braço à Pose A ou B.
        code, sdk_pose = arm.get_forward_kinematics(
            joints_deg,
            input_is_radian=False,
            return_is_radian=False,
        )
        check(code, "get_forward_kinematics", log)

        if sdk_pose is None or len(sdk_pose) < 6:
            raise RuntimeError("O SDK não devolveu uma pose completa")

    sdk_transform = pose_to_T(sdk_pose)
    tf2_xyz_m = np.array(args.tf2_xyz, dtype=float)

    # tf2 e a matriz convertida do SDK estão agora na mesma unidade: metros.
    difference_mm = (tf2_xyz_m - sdk_transform[:3, 3]) * 1000.0
    distance_mm = np.linalg.norm(difference_mm)

    np.set_printoptions(precision=5, suppress=True)
    print(f"alvo         : {cfg['target']}")
    print(f"Pose {args.pose}       : juntas = {joints_deg} graus")
    print(f"pose SDK     : {[round(float(v), 3) for v in sdk_pose[:6]]}")
    print(f"matriz SDK   :\n{sdk_transform}")
    print(f"tf2 xyz      : {tf2_xyz_m} m")
    print(f"tf2 - SDK    : {np.round(difference_mm, 4)} mm")
    print(f"diferença    : {distance_mm:.4f} mm")

    # A translação do tf2_echo foi impressa com três casas decimais
    # em metros; por isso, a diferença calculada tem precisão limitada
    # pelo arredondamento desses valores.


if __name__ == "__main__":
    main()