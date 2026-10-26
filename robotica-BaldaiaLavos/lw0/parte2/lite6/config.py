"""Leitura de config/robot.yaml: o único sítio onde o código sabe onde está a configuração."""
from __future__ import annotations

import os
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG = ROOT / "config" / "robot.yaml"


def load_config(path: str | Path | None = None, target: str | None = None) -> dict:
    """Lê o YAML e resolve o alvo (sim/lab).

    Prioridade do alvo: argumento > variável de ambiente LITE6_TARGET > campo 'target'.
    O dicionário devolvido tem cfg["robot"] = parâmetros do alvo escolhido.
    """
    path = Path(path) if path else DEFAULT_CONFIG
    with open(path, encoding="utf-8") as f:
        cfg = yaml.safe_load(f)
    name = target or os.environ.get("LITE6_TARGET") or cfg["target"]
    if name not in cfg["targets"]:
        raise KeyError(f"alvo '{name}' não existe em {path} (opções: {list(cfg['targets'])})")
    cfg["target"] = name
    cfg["robot"] = cfg["targets"][name]
    return cfg
