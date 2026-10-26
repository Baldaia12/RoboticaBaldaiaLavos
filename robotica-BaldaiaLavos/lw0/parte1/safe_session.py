"""Prepara uma sessão de trabalho com o Lite 6."""

from xarm.wrapper import XArmAPI

IP = "192.168.1.182"

# Confirmar estes limites com o docente antes de executar (valores em mm).
FRONTEIRA = [300, 0, 200, -200, 400, 10]
#             Xmax Xmin Ymax Ymin Zmax Zmin


def verificar(codigo, operacao):
    """Confirma o código devolvido pelo SDK."""
    if isinstance(codigo, (list, tuple)) and len(codigo) == 1:
        codigo = codigo[0]

    if codigo != 0:
        raise RuntimeError(f"{operacao} falhou: código {codigo}")


def safe_session(ip=IP, fronteira=FRONTEIRA):
    """Liga e prepara o robô; devolve o objeto XArmAPI."""
    arm = XArmAPI(ip, is_radian=False)

    try:
        # Confirmar que a posição atual cabe na fronteira escolhida.
        codigo, pose = arm.get_position()
        verificar(codigo, "Ler a pose")
        x, y, z = pose[:3]
        xmax, xmin, ymax, ymin, zmax, zmin = fronteira

        if not (xmin <= x <= xmax and
                ymin <= y <= ymax and
                zmin <= z <= zmax):
            raise RuntimeError(
                f"TCP fora da fronteira: x={x:.1f}, y={y:.1f}, z={z:.1f} mm"
            )

        # Sequência de preparação indicada na ficha.
        verificar(arm.clean_warn(), "Limpar avisos")
        verificar(arm.clean_error(), "Limpar erros")

        # Aplicar os limites antes de ativar os servos.
        verificar(arm.set_reduced_max_joint_speed(54),
                  "Limitar velocidade das juntas")
        verificar(arm.set_reduced_max_tcp_speed(150),
                  "Limitar velocidade do TCP")
        verificar(arm.set_reduced_tcp_boundary(fronteira),
                  "Definir fronteira do TCP")
        verificar(arm.set_fence_mode(True), "Ativar fronteira")
        verificar(arm.set_reduced_mode(True), "Ativar modo reduzido")

        verificar(arm.motion_enable(enable=True), "Ativar movimento")
        verificar(arm.set_mode(0), "Definir modo Position")
        verificar(arm.set_state(0), "Definir estado pronto")

        print("Sessão preparada.")
        print("Fronteira TCP [Xmax, Xmin, Ymax, Ymin, Zmax, Zmin]:",
              fronteira)
        print("Limites: juntas 54 graus/s; TCP 150 mm/s")
        print("Modo reduzido:", arm.is_reduced_mode)
        print("Fronteira ativa:", arm.is_fence_mode)
        return arm

    except Exception:
        arm.disconnect()
        raise


# Permite testar este ficheiro sozinho, sem comandar uma deslocação.
if __name__ == "__main__":
    robo = safe_session()
    robo.disconnect()