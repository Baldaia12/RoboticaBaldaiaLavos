"""Testes de ciclo de vida de safe_session e go_home (LW0, parte 2).

Os testes usam o simulador de firmware por omissão.
Os resultados são mostrados no terminal e guardados em logs/*.csv.

O teste T-S1 inicia um movimento e provoca uma exceção durante o movimento
para verificar se o controlador para corretamente.
"""

import copy
import csv
import statistics
import subprocess
import sys
import time
from datetime import datetime

from xarm.wrapper import XArmAPI

from lite6.config import ROOT, load_config
from lite6.session import go_home, move_joints, safe_session


# Limiares dos requisitos de desempenho.
P1_LIMIAR, P1_ALVO = 5.0, 2.0
P2_LIMIAR, P2_ALVO = 15.0, 8.0

# Deriva máxima admissível durante a observação da paragem.
IMOVEL_DEG = 0.05

RESULTS: list[dict] = []


class InducedFault(Exception):
    """Exceção usada para provocar uma paragem controlada no teste T-S1."""


def record(test_id, requirement, passed, measured, criterion):
    """Regista o resultado no terminal e na lista que será escrita no CSV."""
    result = "PASSA" if passed else "FALHA"

    RESULTS.append({
        "teste": test_id,
        "requisito": requirement,
        "resultado": result,
        "medido": measured,
        "criterio": criterion,
    })

    print(
        f"[{result}] {test_id} ({requirement}): "
        f"{measured} | critério: {criterion}"
    )


def read_angles(arm):
    """Lê as seis juntas em graus."""
    code, angles = arm.get_servo_angle(is_radian=False)

    if code != 0:
        raise RuntimeError(
            f"get_servo_angle falhou (code={code})"
        )

    if angles is None or len(angles) < 6:
        raise RuntimeError(
            "get_servo_angle não devolveu as seis juntas"
        )

    return [float(angle) for angle in angles[:6]]


def read_state(arm):
    """Lê o estado atual do controlador sem o alterar."""
    code, state = arm.get_state()

    if code != 0:
        raise RuntimeError(
            f"get_state falhou (code={code})"
        )

    return state


def max_diff(a, b):
    """Calcula a maior diferença absoluta entre duas posições articulares."""
    if len(a) < 6 or len(b) < 6:
        raise ValueError(
            "são necessários seis ângulos em cada leitura"
        )

    return max(
        abs(float(x) - float(y))
        for x, y in zip(a[:6], b[:6])
    )


def t_f1(cfg):
    """R1: a sessão deve estabelecer ligação e deixar o controlador pronto."""
    with safe_session(cfg) as (arm, _, _):
        passed = (
            arm.connected
            and arm.mode == 0
            and arm.state != 4
            and arm.error_code == 0
        )

        measured = (
            f"connected={arm.connected} "
            f"mode={arm.mode} "
            f"state={arm.state} "
            f"error={arm.error_code}"
        )

    record(
        "T-F1",
        "R1",
        passed,
        measured,
        "ligado, mode=0, state≠4, error_code=0",
    )


def t_f2(cfg):
    """R2: a ligação deve fechar mesmo quando ocorre uma exceção."""
    arm_ref = None

    try:
        with safe_session(cfg) as (arm, _, _):
            arm_ref = arm
            raise InducedFault("falha induzida no T-F2")

    except InducedFault:
        pass

    if arm_ref is None:
        raise RuntimeError(
            "T-F2 não chegou a abrir a sessão"
        )

    connected = arm_ref.connected

    record(
        "T-F2",
        "R2",
        not connected,
        f"connected após exceção={connected}",
        "connected=False",
    )


def t_f3(cfg):
    """R3: partir de start_joints e regressar à posição home."""
    tolerance = cfg["motion"]["home_tolerance_deg"]

    with safe_session(cfg) as (arm, _, log):
        move_joints(
            arm,
            cfg["test_poses"]["start_joints"],
            cfg,
            log,
        )

        result = go_home(arm, cfg, log)

    passed = (
        result["code"] == 0
        and result["max_err_deg"] <= tolerance
    )

    record(
        "T-F3",
        "R3",
        passed,
        (
            f"code={result['code']} "
            f"erro_max={result['max_err_deg']:.3f}°"
        ),
        f"code=0 e erro_max ≤ {tolerance}°",
    )


def t_p1(cfg):
    """P1: medir o tempo de abertura da safe_session."""
    repetitions = cfg["test_poses"]["n_connect"]
    times = []

    for _ in range(repetitions):
        start = time.monotonic()

        with safe_session(cfg):
            times.append(time.monotonic() - start)

    worst = max(times)
    average = statistics.mean(times)
    target_met = worst < P1_ALVO

    record(
        "T-P1",
        "P1",
        worst < P1_LIMIAR,
        (
            f"pior={worst:.2f}s "
            f"média={average:.2f}s "
            f"n={repetitions} "
            f"(alvo {'cumprido' if target_met else 'não cumprido'})"
        ),
        f"pior < {P1_LIMIAR}s (alvo < {P1_ALVO}s)",
    )


def t_p2(cfg):
    """P2: repetir o percurso start_joints → home."""
    repetitions = cfg["test_poses"]["n_home"]
    tolerance = cfg["motion"]["home_tolerance_deg"]

    times = []
    successful_homes = 0

    with safe_session(cfg) as (arm, _, log):
        for _ in range(repetitions):
            move_joints(
                arm,
                cfg["test_poses"]["start_joints"],
                cfg,
                log,
            )

            result = go_home(arm, cfg, log)
            times.append(result["time_s"])

            if (
                result["code"] == 0
                and result["max_err_deg"] <= tolerance
            ):
                successful_homes += 1

    worst = max(times)
    average = statistics.mean(times)
    all_homes_ok = successful_homes == repetitions
    target_met = all_homes_ok and worst < P2_ALVO
    passed = all_homes_ok and worst < P2_LIMIAR

    record(
        "T-P2",
        "P2",
        passed,
        (
            f"pior={worst:.2f}s "
            f"média={average:.2f}s "
            f"home_ok={successful_homes}/{repetitions} "
            f"(alvo {'cumprido' if target_met else 'não cumprido'})"
        ),
        (
            f"todos os regressos válidos e "
            f"pior < {P2_LIMIAR}s "
            f"(alvo < {P2_ALVO}s)"
        ),
    )


def t_s1(cfg):
    """S1: uma exceção durante o movimento deve provocar uma paragem.

    O movimento é acompanhado através das leituras das juntas.
    A exceção só é provocada depois de se confirmar que o braço começou
    a mover-se e ainda não chegou ao alvo.
    """
    start_joints = cfg["test_poses"]["start_joints"]
    far_joints = cfg["test_poses"]["far_joints"]

    progress_limit = 0.5
    remaining_limit = 1.0
    progress_seen = 0.0

    try:
        with safe_session(cfg) as (arm, _, log):
            # Coloca o braço na posição inicial do teste.
            move_joints(
                arm,
                start_joints,
                cfg,
                log,
            )

            initial_angles = read_angles(arm)

            # Inicia o movimento sem esperar que termine.
            move_joints(
                arm,
                far_joints,
                cfg,
                log,
                wait=False,
            )

            deadline = (
                time.monotonic()
                + cfg["motion"]["home_timeout_s"]
            )

            while time.monotonic() < deadline:
                current_angles = read_angles(arm)

                progress_seen = max_diff(
                    initial_angles,
                    current_angles,
                )

                remaining = max_diff(
                    current_angles,
                    far_joints,
                )

                # O braço já começou a mover-se e ainda não chegou
                # ao alvo: este é o momento correto para provocar a falha.
                if (
                    progress_seen > progress_limit
                    and remaining > remaining_limit
                ):
                    raise InducedFault(
                        "falha induzida durante o movimento"
                    )

                # Se chegar ao alvo antes da interrupção, o teste falha.
                if remaining <= remaining_limit:
                    raise RuntimeError(
                        "o braço chegou ao alvo antes da interrupção"
                    )

                time.sleep(0.05)

            raise TimeoutError(
                "o movimento não começou dentro do tempo limite"
            )

    except InducedFault:
        # A safe_session já enviou set_state(4).
        pass

    # Observação separada, sem limpar erros nem reativar o movimento.
    observer = XArmAPI(
        cfg["robot"]["ip"],
        is_radian=False,
        check_joint_limit=cfg["robot"].get(
            "check_joint_limit",
            True,
        ),
    )

    try:
        if not observer.connected:
            raise ConnectionError(
                "T-S1: não foi possível observar o controlador"
            )

        state_before = read_state(observer)
        angles_before = read_angles(observer)

        time.sleep(1.0)

        angles_after = read_angles(observer)
        state_after = read_state(observer)

    finally:
        observer.disconnect()

    drift = max_diff(
        angles_before,
        angles_after,
    )

    distance_to_target = max_diff(
        angles_after,
        far_joints,
    )

    passed = (
        state_before == 4
        and state_after == 4
        and drift < IMOVEL_DEG
        and distance_to_target > 1.0
    )

    record(
        "T-S1",
        "S1",
        passed,
        (
            f"state={state_before}→{state_after} "
            f"progresso antes da paragem={progress_seen:.1f}° "
            f"deriva em 1s={drift:.3f}° "
            f"distância ao alvo={distance_to_target:.1f}°"
        ),
        (
            f"state=4, deriva < {IMOVEL_DEG}° "
            "e alvo não atingido"
        ),
    )


def t_s2(cfg):
    """S2: uma velocidade excessiva deve ser recusada sem movimento."""
    bad_cfg = copy.deepcopy(cfg)

    bad_cfg["motion"]["joint_speed"] = (
        cfg["safety"]["max_joint_speed"] * 2
    )

    with safe_session(cfg) as (arm, _, log):
        move_joints(
            arm,
            cfg["test_poses"]["start_joints"],
            cfg,
            log,
        )

        before = read_angles(arm)

        try:
            go_home(arm, bad_cfg, log)
            refused = False

        except ValueError:
            refused = True

        time.sleep(0.5)

        after = read_angles(arm)

    movement = max_diff(before, after)

    record(
        "T-S2",
        "S2",
        refused and movement < IMOVEL_DEG,
        (
            f"recusou={refused} "
            f"movimento={movement:.3f}°"
        ),
        (
            f"ValueError e movimento < {IMOVEL_DEG}°"
        ),
    )


def t_i1(cfg):
    """I1: verifica automaticamente a ausência de IP fixo no código."""
    process = subprocess.run(
        [
            sys.executable,
            "-m",
            "pytest",
            "-q",
            "tests/test_config.py::test_no_hardcoded_ip",
        ],
        cwd=ROOT,
        capture_output=True,
        text=True,
    )

    output = (
        process.stdout
        or process.stderr
    ).strip()

    summary = (
        output.splitlines()[-1]
        if output
        else "(sem saída)"
    )

    record(
        "T-I1",
        "I1",
        process.returncode == 0,
        summary,
        (
            "teste automático sem IP fixo; "
            "unidades e parâmetros verificados por inspeção"
        ),
    )


# T-S1 fica no fim porque deixa o controlador deliberadamente parado.
TESTS = [
    ("T-F1", "R1", t_f1),
    ("T-F2", "R2", t_f2),
    ("T-F3", "R3", t_f3),
    ("T-P1", "P1", t_p1),
    ("T-P2", "P2", t_p2),
    ("T-I1", "I1", t_i1),
    ("T-S2", "S2", t_s2),
    ("T-S1", "S1", t_s1),
]


def main():
    """Executa todos os testes e guarda os resultados num ficheiro CSV."""
    cfg = load_config()
    RESULTS.clear()

    print(
        f"alvo: {cfg['target']} "
        f"({cfg['robot']['ip']})\n"
    )

    if cfg["target"] != "sim":
        answer = input(
            "ATENÇÃO: robô real. "
            "O docente autorizou e existe alguém junto "
            "da paragem de emergência? [s/N] "
        )

        if answer.strip().lower() != "s":
            sys.exit("abortado")

    # Uma falha num teste não impede a execução dos restantes.
    for test_id, requirement, test_fn in TESTS:
        try:
            test_fn(cfg)

        except Exception as exc:
            record(
                test_id,
                requirement,
                False,
                (
                    f"exceção: {type(exc).__name__}: "
                    f"{exc}"
                ),
                "ver a primeira mensagem de erro no terminal",
            )

    output_dir = ROOT / cfg["logging"]["dir"]
    output_dir.mkdir(
        parents=True,
        exist_ok=True,
    )

    output_path = (
        output_dir
        / (
            f"lifecycle_{cfg['target']}_"
            f"{datetime.now():%Y%m%d_%H%M%S}.csv"
        )
    )

    with open(
        output_path,
        "w",
        newline="",
        encoding="utf-8",
    ) as file:
        writer = csv.DictWriter(
            file,
            fieldnames=list(RESULTS[0]),
        )

        writer.writeheader()
        writer.writerows(RESULTS)

    passed_count = sum(
        row["resultado"] == "PASSA"
        for row in RESULTS
    )

    print(
        f"\n{passed_count}/{len(RESULTS)} passam. "
        f"Resultados em {output_path.relative_to(ROOT)}"
    )


if __name__ == "__main__":
    main()