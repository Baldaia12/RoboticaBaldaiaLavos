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
| `lw1_fk_real.csv` | Resultado do `fk_check` no **robô real** (Parte D): 10 poses, com a FK do grupo (`student_*`), a referência do TF2 (`tf_*`) e a pose reportada pelo **controlador** do robô (`ctrl_*`, com `tcp_offset`), e os erros vs tf e vs controlador. |

## Resultados

### Parte C - `fk_check` em simulação (Gazebo)
- **10/10 poses** validadas.
- Erro máximo: **0.982 mm** (posição) / **0.0014°** (orientação).
- Meta **P1.3** (< 2 mm / < 0.05°): **cumprida**. ✅

### Parte D - robô real
- **10/10 poses** executadas no robô real (modo reduzido).
- **vs TF (modelo):** erro máximo **0.982 mm** / **0.0016°** (média 0.734 mm) — a FK do grupo
  mantém-se validada no robô, tal como em simulação.
- **vs controlador (P1.4):** erro máximo **4.130 mm** / **0.7589°** (média 2.883 mm).
  Meta **P1.4** (< 3 mm / < 0.1°): **não cumprida**.

**Análise (P1.4 não cumprida):** o `tcp_offset` registado é 0, logo o erro não vem de uma
ferramenta mal configurada. O erro vs controlador cresce com a configuração (≈1.3 mm no `home`
até ≈4.1 mm nas poses mais esticadas), o que é a assinatura da **exatidão absoluta de um robô
não calibrado** (desvios de zero das juntas que se acumulam). O modelo cinemático do grupo está
validado de forma independente contra o TF a **< 1 mm**, pelo que o desvio vs controlador é uma
limitação do hardware (calibração do robô físico), não da cinemática direta implementada.
