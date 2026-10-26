"""Regista durante 30 s os ângulos das seis juntas do Lite 6."""

import csv
import time

import matplotlib.pyplot as plt
from xarm.wrapper import XArmAPI

IP = "192.168.1.182"
DURACAO = 30.0  # segundos

arm = XArmAPI(IP, is_radian=False)
amostras = []

try:
    print("A iniciar 30 segundos de registo. Guiem o braço em modo manual.")
    inicio = time.perf_counter()

    # Sem pausa entre leituras: medir a cadência máxima sustentada.
    while time.perf_counter() - inicio < DURACAO:
        codigo, angulos = arm.get_servo_angle(is_radian=False)

        if codigo != 0:
            print(f"Leitura interrompida: código {codigo}")
            break

        tempo = time.perf_counter() - inicio
        amostras.append([tempo] + angulos[:6])

finally:
    arm.disconnect()

if len(amostras) < 2:
    raise RuntimeError("Não há amostras suficientes para criar o gráfico.")

# Guardar os dados para análise e para a entrega.
with open("juntas_30s.csv", "w", newline="", encoding="utf-8") as ficheiro:
    escritor = csv.writer(ficheiro)
    escritor.writerow(["tempo_s", "J1_graus", "J2_graus", "J3_graus",
                       "J4_graus", "J5_graus", "J6_graus"])
    escritor.writerows(amostras)

tempo_observado = amostras[-1][0] - amostras[0][0]
cadencia = (len(amostras) - 1) / tempo_observado

# Traçar as seis juntas em função do tempo.
for junta in range(6):
    plt.plot(
        [linha[0] for linha in amostras],
        [linha[junta + 1] for linha in amostras],
        label=f"J{junta + 1}"
    )

plt.xlabel("Tempo (s)")
plt.ylabel("Ângulo (graus)")
plt.title("Juntas do Lite 6 durante a movimentação manual")
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.savefig("juntas_30s.png", dpi=200)
plt.close()

print(f"Amostras: {len(amostras)}")
print(f"Duração observada: {tempo_observado:.2f} s")
print(f"Cadência obtida: {cadencia:.2f} amostras/s")
print("Guardados: juntas_30s.csv e juntas_30s.png")