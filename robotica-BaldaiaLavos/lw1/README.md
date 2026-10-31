# LW1 - Cinemática direta (Forward Kinematics)

Entregáveis do grupo para o LabWork 1. O código avaliado é o bloco `TODO` de
`src/lite6_kinematics/lite6_kinematics/forward_kinematics.py` (tabela DH
standard); esta pasta guarda os dados e scripts de apoio do grupo.

## Ficheiros

| Ficheiro | O que é |
|----------|---------|
| `ParteA2medicao.py` | Script do grupo para a Parte A.2: lê as poses do robô/simulador e escreve as medições das várias configurações. |
| `lw1_medicoes.csv` | Medições por configuração (7 configs: `zero`, `q1=+90`, ...). Juntas em graus, posição do efetor em mm, orientação em quaternião, eixo `z` e `error_code`. Serve de apoio à tabela DH e ao esboço dos referenciais. |
| `lw1_fk_sim.csv` | Resultado do `fk_check` em **simulação (Gazebo)**: 10 poses, com as juntas, a FK do grupo (`student_*`), a FK de referência do TF2 (`tf_*`) e os erros `ep_tf_mm` (posição) e `eo_tf_deg` (orientação). |
| `lw1_fk_real.csv` | *(por preencher)* — `fk_check` no **robô real** (Parte D, no laboratório). |

## Resultados

### Parte C - `fk_check` em simulação (Gazebo)
- **10/10 poses** validadas.
- Erro máximo: **0.982 mm** (posição) / **0.0014°** (orientação).
- Meta **P1.3** (< 2 mm / < 0.05°): **cumprida**. ✅

### Parte D - robô real
*(por preencher)*
- `fk_check` no robô real: __/__ poses.
- Erro máximo: ______ mm / ______°.
- Meta P1.3 (< 2 mm / < 0.05°): ______.
