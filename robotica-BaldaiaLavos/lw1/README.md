# LW1 — cinemática direta do UFACTORY Lite 6

Grupo Baldaia–Lavos, PL2. Implementação em [`forward_kinematics.py`](../../src/lite6_kinematics/lite6_kinematics/forward_kinematics.py), dados nesta pasta e [relatório LaTeX conjunto](../relatorio/main.tex).

## Modelo e convenções

O código usa a **convenção DH standard**, embora o enunciado peça DH modificada. A função conserva os nomes `dh_table` e `mdh_transform` do template; a expressão matricial implementada é standard: `Rz(theta) Tz(d) Tx(a) Rx(alpha)`. Os parâmetros coincidem com a tabela *Standard D-H Parameters* da folha UFACTORY, depois de converter mm para m e graus para radianos. Esta distinção está explicada no relatório; não se apresenta a tabela standard como modificada.

Os referenciais de entrada e saída são `link_base` e `link_eef`. As juntas vêm de `/joint_states` em radianos; as distâncias estão em metros. [`lw1_medicoes.csv`](lw1_medicoes.csv) contém sete configurações de medição, sem usar o URDF para estimar os parâmetros. O [esboço original dos referenciais](../relatorio/images/lw1_esboco.png) foi fornecido pelo grupo.

## Ficheiros

| Ficheiro | Conteúdo |
|---|---|
| [`ParteA2medicao.py`](ParteA2medicao.py) | Script de apoio à medição da Parte A.2. |
| [`lw1_medicoes.csv`](lw1_medicoes.csv) | Juntas, posição, orientação e código de retorno de sete configurações. |
| [`lw1_fk_sim.csv`](lw1_fk_sim.csv) | Saída íntegra do `fk_check` em Gazebo, dez poses. |
| [`lw1_fk_real.csv`](lw1_fk_real.csv) | Saída íntegra do `fk_check` no braço, dez poses. |
| [`lw1_fk_real.mp4`](lw1_fk_real.mp4) | Vídeo da execução no braço; percorre P0–P9 com confirmação entre movimentos. |

## Resultados

| Comparação | Máximo de posição | Máximo de orientação | Requisito |
|---|---:|---:|---|
| Gazebo, FK do grupo vs. TF | 0,982 mm | 0,0014° | P1.3 cumpre (< 2 mm / < 0,05°). |
| Braço, FK do grupo vs. TF | 0,982 mm | 0,0014° | Comparação adicional. |
| Braço, FK do grupo vs. controlador | 4,111 mm | 0,7393° | **P1.4 não cumpre** (< 3 mm / < 0,1°). |

A execução real terminou nas dez posições sem trajetórias rejeitadas. A boa concordância com TF mostra consistência com o modelo ROS; não elimina a discrepância com a pose reportada pelo controlador. O CSV inclui o campo `tcp_offset` reportado pelo verificador. P1.2 é avaliado com configurações não divulgadas pelo docente; estes CSV não provam esse requisito.

## Repetir o verificador

Dentro do contentor ROS 2, após `lab build` e numa shell nova:

| Terminal | Gazebo | Braço real |
|---|---|---|
| 1 | `lab sim-lite` | `lab real <IP_DO_ROBO>` |
| 2 | `ros2 run lite6_kinematics fk_node` | igual |
| 3 | `ros2 run lite6_kinematics fk_check --ros-args -p robot:=sim` | `ros2 run lite6_kinematics fk_check --ros-args -p robot:=real` |

Os resultados surgem em `~/shared/lw1_fk_sim.csv` ou `~/shared/lw1_fk_real.csv`; os ficheiros nesta pasta são as cópias entregues. O teste real só se repete com as condições de segurança do enunciado da LW0 Parte 1 §3. O [vídeo](lw1_fk_real.mp4) tem 90,24 s.
