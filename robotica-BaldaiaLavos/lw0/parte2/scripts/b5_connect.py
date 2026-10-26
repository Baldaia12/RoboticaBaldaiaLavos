"""Verifica a ligação ao Lite 6 através da sessão da parte 2.

Sem --move: lê juntas e pose, mas safe_session também prepara e ativa
o controlador. Com --move: vai a start_joints e regressa a home_joints.
O alvo é escolhido em config/robot.yaml; por omissão, é o simulador.
"""

import argparse

from lite6.config import load_config
from lite6.session import check, go_home, move_joints, safe_session


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--move",
        action="store_true",
        help="mover para start_joints e regressar a home_joints",
    )
    args = parser.parse_args()

    cfg = load_config()

    # Pedir confirmação antes de abrir uma sessão que pode ativar o braço real,
    # mesmo quando --move não foi indicado.
    if cfg["target"] != "sim":
        answer = input(
            "Robô real: o docente autorizou e há alguém junto da "
            "paragem de emergência? [s/N] "
        )
        if answer.strip().lower() != "s":
            print("Execução cancelada.")
            return

    with safe_session(cfg) as (arm, _, log):
        # O SDK só garante que os valores lidos são válidos quando code == 0.
        angles_code, angles = arm.get_servo_angle(is_radian=False)
        check(angles_code, "get_servo_angle", log)

        pose_code, pose = arm.get_position(is_radian=False)
        check(pose_code, "get_position", log)

        if angles is None or len(angles) < 6:
            raise RuntimeError("O SDK não devolveu os seis ângulos das juntas")
        if pose is None or len(pose) < 6:
            raise RuntimeError("O SDK não devolveu a pose completa do TCP")

        print(f"alvo     : {cfg['target']} ({cfg['robot']['ip']})")
        print(f"firmware : {arm.version}")
        print(f"juntas   : {[round(a, 3) for a in angles[:6]]} graus")
        print(f"pose TCP : {[round(p, 3) for p in pose[:6]]} [mm, mm, mm, graus, graus, graus]")

        if args.move:
            # move_joints e go_home usam os valores definidos em robot.yaml.
            # Começar por verificar este percurso no simulador de firmware.
            move_joints(arm, cfg["test_poses"]["start_joints"], cfg, log)
            result = go_home(arm, cfg, log)

            print(
                f"home     : code={result['code']} "
                f"t={result['time_s']:.2f}s "
                f"erro_max={result['max_err_deg']:.3f}°"
            )

            tolerance = cfg["motion"]["home_tolerance_deg"]
            if result["code"] != 0 or result["max_err_deg"] > tolerance:
                raise RuntimeError(
                    "Regresso a home fora do critério: "
                    f"code={result['code']}, "
                    f"erro_max={result['max_err_deg']:.3f}°, "
                    f"tolerância={tolerance}°"
                )


if __name__ == "__main__":
    main()