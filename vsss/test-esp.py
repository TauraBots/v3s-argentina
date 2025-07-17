import requests
import time
import math

RAIO_RODA = 0.0595

def mps_para_rpm(vel_mps):
    return (vel_mps * 60) / (2 * math.pi * RAIO_RODA)

def enviar_velocidade(ip, porta, vl_mps, vr_mps):
    rpm_l = mps_para_rpm(vl_mps)
    rpm_r = mps_para_rpm(vr_mps)

    url = f"http://{ip}:{porta}/velocity"
    payload = {
        "vl": rpm_l,
        "vr": rpm_r
    }

    try:
        response = requests.post(url, data=payload, timeout=0.2)
        print(f"📤 Enviado → RPM_L={rpm_l:.2f}, RPM_R={rpm_r:.2f}")
        print(f"  → Status: {response.status_code}")
        print(f"  → Resposta: {response.text}")
    except requests.exceptions.RequestException as e:
        print(f"❌ Erro ao enviar dados: {e}")

if __name__ == "__main__":
    ip = "192.168.53.145"
    porta = 8080

    vl = 2.5  # m/s
    vr = 2.5  # m/s

    enviar_velocidade(ip, porta, vl, vr)
    time.sleep(0.1) 
    enviar_velocidade(ip, porta, 0, 0) 
