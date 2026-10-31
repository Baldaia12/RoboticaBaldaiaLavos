# LW2 - Cinemática inversa (Inverse Kinematics)

Entregáveis do grupo para o LabWork 2. O código avaliado é o bloco `TODO` de
`src/lite6_kinematics/lite6_kinematics/inverse_kinematics.py` (Newton com
Jacobiano numérico); esta pasta guarda os CSVs e capturas produzidos pelo
`ik_check` e pelo `ik_study`.

## Ficheiros

| Ficheiro | O que é |
|----------|---------|
| `lw2_ik_sim.csv` | Resultado do `ik_check` em **simulação (Gazebo)**: poses A-D atingidas e U recusada. Por linha: pose, arranque `S0`, `result`, alvo, solução de juntas e erro de verificação `ep_tf_mm` / `eo_tf_deg`. |
| `lw2_ik_sim_A_S1.csv` ... `_S4.csv` | **E.2**: as 4 soluções distintas para a pose A, uma por arranque (`S1`-`S4`). Cada ficheiro tem a linha da respetiva solução. |
| `E2_S1.png` ... `E2_S4.png` | **E.2**: capturas no Gazebo de cada uma das 4 soluções (S1-S4). |
| `lw2_study_convergence.csv` | **E.1**: `ik_study convergence` - 500 poses (seed fixa 2627), arranque `S0`, passo `h = H`. Colunas: `h`, `index`, `q_true1..6`, `status`, `n_iter`, `q1..6`, `ep_mm`, `eo_deg`. |
| `lw2_study_nudge.csv` | Estudo complementar `ik_study nudge` - as mesmas 500 poses a partir de `S0` para vários passos do Jacobiano numérico `h` (1e-1 ... 1e-14), 3000 linhas. |
| `lw2_study_zero.csv` | Estudo complementar `ik_study zero` - as mesmas 500 poses com arranque em `Theta = 0` e `h = H`, 500 linhas. |
| `lw2_ik_real.csv` | *(por preencher)* — `ik_check` no **robô real** (Parte D, no laboratório). |

> Nota: `convergence` corresponde ao estudo E.1. Os estudos `nudge` e `zero`
> cobrem as restantes alíneas (E.3 e E.4) — confirmar a correspondência exata
> com o enunciado antes da entrega.

## Resultados

### Parte C - `ik_check` em simulação (Gazebo)
- Poses **A, B, C, D atingidas** (`reached`); pose **U recusada** (`refused`),
  por estar fora do espaço de trabalho.
- Erro máximo das atingidas: **1.107 mm** / **0.0017°**.
- Meta **P2.4**: **cumprida**. ✅

### E.1 - estudo de convergência (`lw2_study_convergence.csv`)
- **87.2% converged**.
- **62 out_of_range** (50 devidos apenas à junta 5).
- **2 max_iterations**.

### E.2 - múltiplas soluções para a pose A
- **4 soluções distintas** (S1-S4):
  - **S1 / S2**: braço pela frente;
  - **S3 / S4**: braço por cima e por trás;
  - dentro de cada par, o **pulso inverte-se**.

### Parte D - robô real
*(por preencher)*
- `ik_check` no robô real: poses atingidas ______ / recusadas ______.
- Erro máximo: ______ mm / ______°.
- Meta P2.4: ______.
