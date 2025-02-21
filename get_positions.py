import requests
import json
import re

# Configurações da API
base_url = "http://10.83.131.155/api/v2.0.0"
auth_header = {
    "accept": "application/json",
    "Authorization": "Basic ZGlzdHJpYnV0b3I6NjJmMmYwZjFlZmYxMGQzMTUyYzk1ZjZmMDU5NjU3NmU0ODJiYjhlNDQ4MDY0MzNmNGNmOTI5NzkyODM0YjAxNA==",
    "Accept-Language": "en_US"
}

# 1. Obter o status do robô
status_response = requests.get(f"{base_url}/status", headers=auth_header)
status_data = status_response.json()

# 2. Capturar o nome do destino a partir do "mission_text"
mission_text = status_data.get("mission_text", "")
match = re.search(r"Moving to '(.*?)'", mission_text)
destino_atual = match.group(1) if match else None

if not destino_atual:
    print("Destino não encontrado.")
    exit()

print(f"Destino atual do robô: {destino_atual}")

# 3. Obter o ID do mapa
map_id = status_data.get("map_id")
if not map_id:
    print("ID do mapa não encontrado.")
    exit()
print(map_id)
# 4. Obter as posições no mapa
positions_response = requests.get(f"{base_url}/maps/{map_id}/positions", headers=auth_header)
positions_data = positions_response.json()

# 5. Encontrar o guid correspondente ao nome do destino
guid_destino = None
print(positions_data)
for position in positions_data:
    print(position)
    if position.get("name") == destino_atual:
        guid_destino = position.get("guid")
        break
print(guid_destino)
if guid_destino:
    print(f"GUID do destino '{destino_atual}': {guid_destino}")
    
    # 6. Obter os detalhes completos da posição pelo GUID
    position_response = requests.get(f"{base_url}/positions/{guid_destino}", headers=auth_header)
    
    if position_response.status_code == 200:
        position_data = position_response.json()
        print("Detalhes da posição:")
        print(json.dumps(position_data, indent=4))
        print(position_data.get("pos_x"))
        print(position_data.get("pos_y"))
    else:
        print(f"Erro ao obter detalhes da posição. Código: {position_response.status_code}")

else:
    print(f"Destino '{destino_atual}' não encontrado no mapa.")
