# Robotica Baldaia Lavos

Repositório do grupo Bernardo Pereira (2022219048) e João Lavos (2021222529)
para a unidade curricular Robotics, 2026/2027.

## Estrutura

```text
robotica/                         # ambiente ROS 2 fornecido pelo docente
├── lab.ps1                       # comandos Docker/ROS no Windows
├── shared/                       # ficheiros exportados pelo ambiente ROS
└── robotica-BaldaiaLavos/        # trabalho do grupo
    ├── README.md
    └── lw0/
        ├── parte1/               # Parte 1 do LW0
        ├── parte2/               # projeto Python e ciclo de vida
        └── relatorio_LW0.tex     # relatório final em LaTeX
```

O `lab.ps1` pertence à pasta exterior `robotica`. Os scripts Python da Parte 2
correm a partir de `robotica-BaldaiaLavos\lw0\parte2`.

## Identificação ROS

O docente informou que não seriam atribuídos números aos grupos. Por isso,
usamos `ROS_DOMAIN_ID=0` no `.env` da pasta exterior `robotica`. Todos os
terminais ROS devem usar o mesmo valor.

## Parte 2 em Python

No PowerShell:

```powershell
cd C:\Users\balda\Documents\Mestrado\Robotica\P\robotica\robotica-BaldaiaLavos\lw0\parte2
& C:\Users\balda\venvs\robotica\Scripts\Activate.ps1
pytest -q
```

Para criar o ambiente do zero:

```powershell
python -m venv C:\Users\balda\venvs\robotica
& C:\Users\balda\venvs\robotica\Scripts\Activate.ps1
python -m pip install --upgrade pip
pip install -r requirements.txt
pip install -e .
```

## Simulador de firmware UFACTORY

Numa janela do PowerShell do Windows:

```powershell
docker start -ai uf_software
```

Dentro do contentor, iniciar o perfil Lite 6:

```bash
screen -wipe
/xarm_scripts/xarm_start.sh 6 9
```

Deixar esta janela aberta. O Studio do simulador fica disponível em
`http://127.0.0.1:18333`.

Noutra janela PowerShell, a partir de `lw0\parte2`:

```powershell
python scripts\b5_connect.py --move
python scripts\test8_fk_compare.py --pose A --tf2-xyz 0.087 0.000 0.154
python scripts\lifecycle_tests.py
```

O teste de ciclo de vida termina com `T-S1`, que provoca uma paragem
intencional. Para repetir o ciclo completo, reiniciar primeiro o simulador.

## Ambiente ROS 2, MoveIt 2 e Gazebo

Estes comandos são executados a partir da pasta exterior `robotica`, não da
pasta `robotica-BaldaiaLavos`:

```powershell
cd C:\Users\balda\Documents\Mestrado\Robotica\P\robotica
.\lab.ps1 up
.\lab.ps1 doctor
.\lab.ps1 shell
```

Dentro do contentor, os testes avançados são:

```bash
lab test 1
lab test 1b       # segundo terminal, para o listener
lab test 2
lab test 3
lab test 4 link_base link_eef
lab test 5 link_base link_eef
lab test 6
lab test 7
```

O Teste 8 é executado pelo script Python do grupo. O PDF do Teste 4 é criado
em `shared/`. Para terminar o ambiente ROS:

```powershell
.\lab.ps1 down
```

## Evidências e relatório

- resultados do ciclo de vida: `lw0\parte2\logs\lifecycle_sim_*.csv`;
- logs da sessão: `lw0\parte2\logs\session_*.log`;
- evidências dos Testes 4 e 5: `lw0\parte2\docs\evidencias\`;
- modelo em V: `lw0\parte2\docs\modelo_V.svg`;
- relatório LaTeX combinado: `lw0\relatorio_LW0.tex`.

Não executar comandos contra o braço real sem autorização do docente e sem
um membro do grupo junto da paragem de emergência.
