"""Testes unitários que NÃO precisam de robô (§A.3, degrau 1). Correm a cada alteração."""
import re

from lite6.config import ROOT, load_config

IP_RE = re.compile(r"\b\d{1,3}(?:\.\d{1,3}){3}\b")


def test_config_loads():
    cfg = load_config()
    assert cfg["target"] in cfg["targets"]
    assert "ip" in cfg["robot"]
    assert len(cfg["motion"]["home_joints"]) == 6


def test_target_switch_is_one_parameter():
    """Simulador <-> robô é um parâmetro, nunca uma edição (§A.3)."""
    for name in load_config()["targets"]:
        assert load_config(target=name)["robot"]["ip"]


def test_no_hardcoded_ip():
    """T-I1 / §B.5.3: nenhum endereço IP escrito no código Python."""
    offenders = []
    for folder in ("lite6", "scripts"):
        for py in (ROOT / folder).rglob("*.py"):
            for n, line in enumerate(py.read_text(encoding="utf-8").splitlines(), 1):
                if IP_RE.search(line):
                    offenders.append(f"{py.relative_to(ROOT)}:{n}: {line.strip()}")
    assert not offenders, "IP no código:\n" + "\n".join(offenders)


def test_speed_within_cap():
    cfg = load_config()
    assert cfg["motion"]["joint_speed"] <= cfg["safety"]["max_joint_speed"]


def test_domain_id_reflects_course_instruction():
    """O docente não atribuirá números de grupo; usamos o domínio 0."""
    assert load_config()["ros"]["domain_id"] == 0