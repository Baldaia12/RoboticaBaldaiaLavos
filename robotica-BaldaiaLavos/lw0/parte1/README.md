# LW0 Parte 1 — operar o UFACTORY Lite 6

Grupo Baldaia–Lavos, PL2. A secção da Parte 1 está no [relatório conjunto](../../relatorio/main.tex); reúne as observações da sessão no braço e as evidências desta pasta.

## Trabalho e evidências

- Ligação à rede `Robot-1`; controlador `192.168.1.182`, portátil `192.168.1.12/24`, Studio no porto `18333`. Estes são os valores da sessão documentada, não endereços universais para outras bancadas.
- Live Control: gamas observadas das seis juntas, movimentação por juntas e por coordenadas cartesianas, leitura do estado, ensino manual e garra. Não foram registadas medições com paquímetro da garra.
- Primeiro contacto com o SDK: leitura de juntas e pose, regresso à configuração home a partir de três posições, deslocação de +50 mm em Z e comandos da garra.
- Exemplos `0002-get_property` e `2001-move_joint` adaptados ao Lite 6 e descritos no relatório.
- Registo manual de 30,00 s: 1712 amostras, cadência média 57,03 amostras/s. [Dados](juntas_30s.csv) e [gráfico](juntas_30s.png).
- Colisão registada pelo controlador (`C31`, junta 2) e recuperação descrita no relatório.

## Ficheiros

| Ficheiro | Conteúdo |
|---|---|
| [`../../relatorio/main.tex`](../../relatorio/main.tex) | Relatório conjunto da entrega, incluindo a LW0 Parte 1. |
| [`logs_joints.py`](logs_joints.py) | Leitura dos seis ângulos durante 30 s e registo para o gráfico. |
| [`safe_session.py`](safe_session.py) | Ligação, limpeza de erros, ativação, modo/estado e parâmetros de modo reduzido. |
| [`juntas_30s.csv`](juntas_30s.csv) e [`juntas_30s.png`](juntas_30s.png) | Dados originais e evolução temporal de J1–J6. |

O enunciado da LW0 Parte 1 §3 define o procedimento de segurança que continua a aplicar-se nas LW1 e LW2 no braço real.
