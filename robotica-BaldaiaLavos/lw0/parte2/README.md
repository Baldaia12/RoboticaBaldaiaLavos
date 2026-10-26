# LW0 Parte 2 — projeto Python e validação

Bernardo Pereira (2022219048) e João Lavos (2021222529)

Este diretório é robotica-BaldaiaLavos\lw0\parte2. O trabalho do grupo está
dentro de robotica-BaldaiaLavos, que por sua vez está dentro da pasta exterior
robotica usada pelo ambiente ROS 2 do docente.

Repositório do grupo: https://github.com/Baldaia12/RoboticaBaldaiaLavos

## Identificação ROS

O docente informou que não seriam atribuídos números aos grupos. O valor usado
é, portanto, ROS_DOMAIN_ID=0, definido no ficheiro .env da pasta exterior
robotica. Não alterar este valor para um número inventado.

## 1. Ambiente Python

Todos os comandos seguintes começam nesta pasta:

~~~powershell
cd C:\Users\balda\Documents\Mestrado\Robotica\P\robotica\robotica-BaldaiaLavos\lw0\parte2
~~~

Para recriar o ambiente no Windows PowerShell:

~~~powershell
python -m venv C:\Users\balda\venvs\robotica
& C:\Users\balda\venvs\robotica\Scripts\Activate.ps1
python -m pip install --upgrade pip
pip install -r requirements.txt
pip install -e .
~~~

O último comando instala o pacote local lite6, permitindo os imports dos
scripts. No IDE, selecionar o interpretador de
C:\Users\balda\venvs\robotica.

## 2. Testes sem robô

Com o ambiente virtual ativo:

~~~powershell
pytest -q
~~~

Resultado esperado: os cinco testes passam. O teste de domain_id verifica o
valor 0, que é o valor adotado para este grupo sem número atribuído.

## 3. Simulador de firmware UFACTORY

Na primeira utilização, criar o contentor numa única linha:

~~~powershell
docker run -it --name uf_software -p 18333:18333 -p 502:502 -p 503:503 -p 504:504 -p 30000:30000 -p 30001:30001 -p 30002:30002 -p 30003:30003 danielwang123321/uf-ubuntu-docker
~~~

Nas utilizações seguintes, numa janela do PowerShell do Windows:

~~~powershell
docker start -ai uf_software
~~~

Dentro do contentor, iniciar o perfil Lite 6:

~~~bash
screen -wipe
/xarm_scripts/xarm_start.sh 6 9
~~~

Deixar essa janela aberta. O Studio fica em http://127.0.0.1:18333.

Depois, noutra janela PowerShell, com o ambiente virtual ativo:

~~~powershell
python scripts\b5_connect.py
python scripts\b5_connect.py --move
python scripts\test8_fk_compare.py --pose A --tf2-xyz 0.087 0.000 0.154
python scripts\lifecycle_tests.py
~~~

O script b5_connect.py valida a ligação, a configuração de segurança e o
movimento start → home. O script test8_fk_compare.py compara a cinemática
direta do SDK com a transformação tf2, recebida em metros. O script
lifecycle_tests.py executa os requisitos R1–R3, P1–P2, I1 e S1–S2.

O último teste do ciclo de vida, T-S1, provoca uma paragem intencional e deixa
o controlador em state=4. Para repetir o ciclo completo, reiniciar primeiro
o contentor e o firmware.

## 4. Configuração e segurança

O alvo e todos os parâmetros estão em config/robot.yaml. O alvo usado na
entrega é sim, com IP 127.0.0.1. O modo real lab só pode ser usado com
autorização do docente, alguém junto da paragem de emergência e todos os
testes prévios concluídos no simulador.

Os limites usados são os da Parte 1: juntas reduzidas a 54 graus/s, TCP a
150 mm/s e fronteira [300, 0, 200, -200, 400, 10] mm. O código recusa ainda
velocidades articulares acima de max_joint_speed=30 graus/s.

## 5. Ambiente ROS 2, MoveIt 2 e Gazebo

O lab.ps1 está na pasta exterior robotica, não nesta pasta. Para iniciar o
ambiente avançado:

~~~powershell
cd C:\Users\balda\Documents\Mestrado\Robotica\P\robotica
.\lab.ps1 up
.\lab.ps1 doctor
.\lab.ps1 shell
~~~

Dentro do contentor, repetir os testes da ficha:

~~~bash
lab test 1
lab test 1b       # segundo terminal, para o listener
lab test 2
lab test 3
lab test 4 link_base link_eef
lab test 5 link_base link_eef
lab test 6
lab test 7
~~~

O Teste 8 é executado pelo script Python da secção 3. No Teste 1, os dois
terminais devem usar o mesmo ROS_DOMAIN_ID=0. No Teste 3, deixar o RViz
aberto para os Testes 4 e 5. O Teste 4 cria o PDF da árvore de frames em
shared/.

Para parar o ambiente ROS:

~~~powershell
.\lab.ps1 down
~~~

## 6. Evidências

- CSV do ciclo de vida: logs\lifecycle_sim_*.csv;
- logs da sessão: logs\session_*.log;
- matrizes e juntas do Teste 5: docs\evidencias\;
- figura do modelo em V: docs\modelo_V.svg;
- relatório combinado: ..\relatorio_LW0.tex.

O README deve ser suficiente para uma pessoa recriar o ambiente e repetir
pytest, B.5 e os Testes 1, 3 e 8 a partir de uma shell limpa.
