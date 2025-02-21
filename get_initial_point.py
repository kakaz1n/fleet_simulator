import requests
import json
import re
import time
import threading

# Configurações da API
base_url = "http://10.83.131.155/api/v2.0.0"
auth_header = {
    "accept": "application/json",
    "Authorization": "Basic ZGlzdHJpYnV0b3I6NjJmMmYwZjFlZmYxMGQzMTUyYzk1ZjZmMDU5NjU3NmU0ODJiYjhlNDQ4MDY0MzNmNGNmOTI5NzkyODM0YjAxNA==",
    "Accept-Language": "en_US"
}

ultima_missao = None  # Armazena a última missão detectada
ultima_posicao_inicial = None  # Última posição inicial do robô registrada
ultima_posicao_destino = None  # Última posição de destino registrada
mutex = threading.Lock()  # Lock para evitar concorrência


def detectar_mudanca_missao():
    """Thread que monitora mudanças na missão e captura a posição inicial"""
    global ultima_missao, ultima_posicao_inicial

    while True:
        try:
            # 1. Obter o status do robô
            status_response = requests.get(f"{base_url}/status", headers=auth_header)
            if status_response.status_code != 200:
                time.sleep(1)
                continue

            status_data = status_response.json()

            # 2. Capturar a missão atual
            mission_text = status_data.get("mission_text", "")
            match = re.search(r"Moving to '(.*?)'", mission_text)
            destino_atual = match.group(1) if match else None

            if not destino_atual:
                time.sleep(1)
                continue

            # Captura a posição atual do robô
            posicao_atual = (status_data["position"]["x"], status_data["position"]["y"])

            # Se a missão mudou, armazena a posição inicial
            if destino_atual != ultima_missao:
                with mutex:
                    ultima_missao = destino_atual
                    # Somente imprime se a posição inicial mudou
                    if posicao_atual != ultima_posicao_inicial:
                        ultima_posicao_inicial = posicao_atual
                        print(f"\nNova missão detectada: {destino_atual}")
                        print(f"Posição inicial do robô: X={ultima_posicao_inicial[0]}, Y={ultima_posicao_inicial[1]}")

        except Exception as e:
            print(f"Erro na thread de detecção de missão: {e}")

        time.sleep(0.5)  # Verifica mudanças de missão rapidamente


def obter_posicao_destino():
    """Obtém a posição de destino com base no nome da missão"""
    global ultima_posicao_destino

    while True:
        if ultima_missao:
            try:
                # 3. Obter o ID do mapa
                status_response = requests.get(f"{base_url}/status", headers=auth_header)
                if status_response.status_code != 200:
                    time.sleep(2)
                    continue

                status_data = status_response.json()
                map_id = status_data.get("map_id")
                if not map_id:
                    time.sleep(2)
                    continue

                # 4. Obter as posições no mapa
                positions_response = requests.get(f"{base_url}/maps/{map_id}/positions", headers=auth_header)
                if positions_response.status_code != 200:
                    time.sleep(2)
                    continue

                positions_data = positions_response.json()

                # 5. Encontrar o GUID correspondente ao nome do destino
                guid_destino = None
                with mutex:
                    destino_atual = ultima_missao  # Usa a missão detectada pela outra thread

                for position in positions_data:
                    if position.get("name") == destino_atual:
                        guid_destino = position.get("guid")
                        break

                if guid_destino:
                    # 6. Obter os detalhes completos da posição pelo GUID
                    position_response = requests.get(f"{base_url}/positions/{guid_destino}", headers=auth_header)
                    if position_response.status_code == 200:
                        position_data = position_response.json()
                        posicao_destino = (position_data.get("pos_x"), position_data.get("pos_y"))

                        # Somente imprime se a posição de destino mudou
                        if posicao_destino != ultima_posicao_destino:
                            ultima_posicao_destino = posicao_destino
                            print(f"Posição de destino do robô: X={ultima_posicao_destino[0]}, Y={ultima_posicao_destino[1]}")

            except Exception as e:
                print(f"Erro ao obter posição de destino: {e}")

        time.sleep(5)  # Atualiza a posição de destino periodicamente


# Criar e iniciar threads
thread_missao = threading.Thread(target=detectar_mudanca_missao, daemon=True)
thread_destino = threading.Thread(target=obter_posicao_destino, daemon=True)

thread_missao.start()
thread_destino.start()

# Mantém o script rodando
while True:
    time.sleep(1)


