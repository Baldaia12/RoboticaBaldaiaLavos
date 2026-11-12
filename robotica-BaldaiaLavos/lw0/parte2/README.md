# LW0 Parte 2 — ambiente e ciclo de vida

Grupo Baldaia–Lavos, PL2. A secção da Parte 2 está no [relatório conjunto](../../relatorio/main.tex): contém os testes B.5, C.5 (1–8), as duas matrizes `tf2`, a comparação SDK/TF, os requisitos de `safe_session` e `home`, resultados, rastreabilidade e modelo em V. Os ensaios desta parte foram feitos em simulação de firmware e Gazebo.

## Ambiente

- Nível intermédio: Windows, Python 3.12.10, ambiente virtual e xArm Python SDK; projeto Python nesta pasta.
- Nível avançado: Docker, ROS 2 Jazzy, MoveIt 2 e Gazebo Harmonic pela via Windows do repositório da disciplina.
- Simulador de firmware UFACTORY com perfil Lite 6 (`6 9`), IP `127.0.0.1`.
- `ROS_DOMAIN_ID=0`, valor usado pelo grupo no ambiente documentado. Confirmar que todos os terminais do mesmo ensaio usam o mesmo valor.

O IP, a home e os limites da aplicação SDK ficam em [`config/robot.yaml`](config/robot.yaml); a estrutura do pacote está em `lite6/`. O ficheiro [`requirements.txt`](requirements.txt) fixa as dependências Python.

## Reproduzir os testes Python

No PowerShell, a partir desta pasta:

```powershell
python -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install -r requirements.txt
python -m pip install -e .
pytest -q
```

Para o simulador de firmware, iniciar o contentor `uf_software`, executar `screen -wipe` e `/xarm_scripts/xarm_start.sh 6 9` dentro dele, confirmar o Studio em `http://127.0.0.1:18333` e usar os scripts `scripts/b5_connect.py`, `scripts/test8_fk_compare.py` e `scripts/lifecycle_tests.py`. Consultar o [relatório conjunto](../../relatorio/main.tex) antes de repetir o teste de paragem intencional T-S1: o simulador tem de ser reiniciado após esse teste.

Para ROS 2, abrir PowerShell na pasta exterior `robotica`, correr `.\lab.ps1 up`, `.\lab.ps1 doctor`, `.\lab.ps1 shell`; no contentor, `lab test 1` e `lab test 1b` (segundo terminal), depois `lab test 2` a `lab test 7` pela ordem do enunciado. O teste 8 usa o script Python do grupo contra o simulador de firmware. Para uma repetição limpa de B.5 e dos testes 1, 3 e 8, usar esta sequência e os passos pormenorizados no relatório.

## Resultados e evidências

Os cinco testes Python, B.5 e os testes avançados 1–8 foram registados como aprovados. No teste 8, a diferença de posição SDK–TF na pose A foi 0,200 mm. O ciclo de vida cobriu 3 requisitos funcionais, 2 de desempenho, 1 de interface e 2 de segurança; os oito testes associados passaram no simulador. A comparação refere-se aos dados arredondados do teste 5 e do SDK, sem alegar precisão física superior.

- [`logs/lifecycle_sim_20261026_170724.csv`](logs/lifecycle_sim_20261026_170724.csv): resultados dos testes do ciclo de vida.
- [`docs/evidencias/`](docs/evidencias/): ângulos, matrizes e árvores de referenciais dos testes 4 e 5.
- [`docs/modelo_V.svg`](docs/modelo_V.svg): desenho do modelo em V.
- [`../../relatorio/main.tex`](../../relatorio/main.tex): relatório conjunto com resultados e matriz de rastreabilidade.
