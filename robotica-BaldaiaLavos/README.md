# Robótica 2026/2027 — grupo Baldaia–Lavos (PL2)

Bernardo Pereira (2022219048) e João Lavos (2021222529). Repositório: [RoboticaBaldaiaLavos](https://github.com/Baldaia12/RoboticaBaldaiaLavos), ramo `windows`.

## Entregas e evidências

| Parte | Código e evidências | Relatório |
|---|---|---|
| LW0 Parte 1 — operação e segurança | [README](lw0/parte1/README.md) | [LaTeX conjunto](relatorio/main.tex) |
| LW0 Parte 2 — ambiente e ciclo de vida | [README](lw0/parte2/README.md) | [LaTeX conjunto](relatorio/main.tex) |
| LW1 — cinemática direta | [README](lw1/README.md) | [LaTeX conjunto](relatorio/main.tex) |
| LW2 — cinemática inversa algébrica | [README](lw2/README.md) | [LaTeX conjunto](relatorio/main.tex) |

O relatório LaTeX reúne as quatro partes e inclui o esboço da LW1, o gráfico da LW0 e as quatro capturas E.2 da LW2. As tabelas numéricas do relatório resultam dos CSV identificados nos README de cada parte; os CSV produzidos pelos verificadores não foram editados.

## Organização do repositório

- `lw0/parte1/`: scripts de registo e sessão segura, dados e relatório da primeira sessão.
- `lw0/parte2/`: projeto Python/SDK, configuração, testes, logs e relatório do ambiente e ciclo de vida.
- `lw1/`: medições e CSV do `fk_check` em Gazebo e no braço, mais [vídeo da execução real](lw1/lw1_fk_real.mp4).
- `lw2/`: CSV do `ik_check` e `ik_study`, capturas E.2 e [vídeo da execução real](lw2/lw2_ik_real.mp4).
- `relatorio/`: fonte LaTeX e imagens para a Entrega 1.
- `../src/lite6_kinematics/lite6_kinematics/forward_kinematics.py`: implementação da LW1.
- `../src/lite6_ik/lite6_ik/inverse_kinematics.py`: implementação algébrica da LW2. Os ficheiros antigos de Newton em `lite6_kinematics` não são a entrega da LW2 algébrica.

Os links para os vídeos são relativos: funcionam no repositório local e, depois de os ficheiros serem publicados, na página do GitHub, sem prender a documentação ao nome de uma branch. Os vídeos têm 90,24 s (LW1) e 66,32 s (LW2).

## Resultados medidos

| Verificação | Resultado principal | Referência e limiar do enunciado |
|---|---|---|
| LW1 Gazebo, 10 poses | máximo 0,982 mm / 0,0014° | FK vs. TF; P1.3: < 2 mm / < 0,05° — cumpre |
| LW1 braço, 10 poses | máximo 4,111 mm / 0,7393° | FK vs. controlador; P1.4: < 3 mm / < 0,1° — **não cumpre** |
| LW2 Gazebo, A–D | máximo 1,105 mm / 0,00155°; U recusada | pose atingida vs. alvo, TF; P2.4 e P2.6 — cumprem |
| LW2 braço, A–D | máximo 3,788 mm / 0,5686°; U recusada | pose atingida vs. alvo, controlador; P2.5 — **não cumpre**; P2.6 cumpre |

As comparações com TF no robô real também foram registadas: LW1, 0,982 mm / 0,0014°; LW2, 1,188 mm / 0,0399°. Não substituem a comparação com o controlador exigida em P1.4 e P2.5. Os valores devem ser discutidos com o docente; não foram arredondados para declarar esses requisitos cumpridos.

## Reproduzir LW1 e LW2

No Windows, a partir da pasta exterior `robotica`, iniciar Docker Desktop e executar `./lab.ps1 up` (em PowerShell: `.\lab.ps1 up`), depois `.\lab.ps1 shell` em cada terminal. Dentro do contentor, usar `lab build` após alterações no código e abrir uma shell nova. Os comandos exatos, a ordem dos nós, os testes e os ficheiros de saída estão nos README da [LW1](lw1/README.md) e [LW2](lw2/README.md). A pasta `shared/` da raiz exterior corresponde a `~/shared` no contentor.

Para ensaios no braço: primeiro simulação, modo reduzido, espaço de trabalho livre e uma pessoa na paragem de emergência, conforme a LW0 Parte 1. `Ctrl+C` não interrompe um movimento já em curso.
