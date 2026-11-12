# LW2 — cinemática inversa algébrica do UFACTORY Lite 6

Grupo Baldaia–Lavos, PL2. A implementação avaliada é [`inverse_kinematics.py`](../../src/lite6_ik/lite6_ik/inverse_kinematics.py), no pacote **`lite6_ik`**. Usa a cinemática direta da LW1 e uma solução fechada; os ficheiros antigos de Newton em `lite6_kinematics` não pertencem a esta entrega. [Relatório LaTeX conjunto](../relatorio/main.tex).

## Algoritmo

A partir da pose desejada, subtrai-se o avanço de 0,0615 m da ferramenta para obter o centro do punho. A lei dos cossenos dá dois ramos para o cotovelo; há duas orientações para a base e duas soluções para o punho regular, até oito soluções distintas. Na singularidade do punho (`q5=0`), a convenção é `q4=0` e a pose fixa `q4+q6`. As soluções são devolvidas em `(−π,π]` e deduplicadas módulo `2π`.

Para executar uma solução, cada ângulo é ajustado por `2πk` dentro dos limites de junta e escolhe-se o vetor de menor distância euclidiana aos ângulos atuais. O nó subscreve `/lite6_ik/target` e `/joint_states` e publica a solução em `/lite6_ik/solution`, ou regista `unreachable`/`out_of_range` sem publicar.

## Ficheiros de evidência

| Ficheiro | Conteúdo |
|---|---|
| [`lw2_ik_sim.csv`](lw2_ik_sim.csv) e [`lw2_ik_real.csv`](lw2_ik_real.csv) | Teste principal A–D e U em Gazebo e no braço. |
| [`lw2_ik_sim_A_all.csv`](lw2_ik_sim_A_all.csv) | E.2: oito soluções da pose A; quatro executadas e quatro rejeitadas por limites. |
| [`E2_A_solucao_1.png`](E2_A_solucao_1.png), [`2`](E2_A_solucao_2.png), [`5`](E2_A_solucao_5.png), [`6`](E2_A_solucao_6.png) | Capturas do Gazebo das quatro soluções executadas. |
| [`lw2_ik_sim_A_S1.csv`](lw2_ik_sim_A_S1.csv) a [`S4`](lw2_ik_sim_A_S4.csv) | E.3: solução selecionada desde cada configuração inicial. |
| [`lw2_study_count.csv`](lw2_study_count.csv) | E.1: 500 poses, número total e dentro dos limites. |
| [`lw2_study_wrist.csv`](lw2_study_wrist.csv) | E.4: 100 poses singulares, sete soluções por pose. |
| [`lw2_ik_real.mp4`](lw2_ik_real.mp4) | Vídeo da execução real A–D e tentativa U. |

## Resultados principais

- Gazebo: A–D `reached`, U `refused`, nenhum `rejected`; erro máximo vs. TF **1,105 mm / 0,00155°**. P2.4 cumpre.
- Braço real: A–D `reached`, U `refused`, nenhum `rejected`; erro máximo vs. TF **1,188 mm / 0,0399°**.
- Braço real vs. controlador: máximo **3,788 mm / 0,5686°**. **P2.5 não cumpre** o limiar < 3 mm / < 0,1°; a comparação adicional com TF não substitui esta métrica.
- P2.6: a pose U foi recusada em Gazebo e no braço, sem executar movimento para U: **2/2**.
- E.1: 500/500 poses com oito soluções, 500/500 vetores de origem encontrados; 1 a 8 soluções dentro dos limites, com distribuição detalhada no relatório.
- E.2: soluções 1, 2, 5 e 6 executadas; 3, 4, 7 e 8 rejeitadas por limites.
- E.4: 700 linhas (100 poses × sete soluções), uma solução singular canónica com `q4=0` por pose.

P2.2 e P2.3 usam casos e referência não divulgados pelo docente; estes estudos internos não substituem essa avaliação.

## Repetir

Depois de `lab build`, abrir nova shell no contentor. Para Gazebo: terminal 1 `lab sim-lite`, terminal 2 `ros2 run lite6_ik ik_node`, terminal 3 `ros2 run lite6_ik ik_check --ros-args -p robot:=sim`. Para o braço, trocar apenas o terminal 1 por `lab real <IP_DO_ROBO>` e `robot:=real` no terminal 3, depois de completar a simulação e verificar as condições de segurança da LW0 Parte 1 §3.

Os estudos sem movimento usam `ros2 run lite6_ik ik_study count` e `ros2 run lite6_ik ik_study wrist`. Para E.2, com Gazebo: `ros2 run lite6_ik ik_check --ros-args -p robot:=sim -p target:=A -p all:=true`. Para E.3, repetir com `-p target:=A -p start:=S1`, `S2`, `S3` e `S4`, mantendo `ik_node` ativo. As saídas nascem em `~/shared/` e foram copiadas para esta pasta sem edição. O [vídeo](lw2_ik_real.mp4) tem 66,32 s.
