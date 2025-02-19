import requests
import json

# Configuração do MiR
MIR_IP = "10.83.131.110"
AUTH_TOKEN = "ZGlzdHJpYnV0b3I6NjJmMmYwZjFlZmYxMGQzMTUyYzk1ZjZmMDU5NjU3NmU0ODJiYjhlNDQ4MDY0MzNmNGNmOTI5NzkyODM0YjAxNA=="
HEADERS = {
    "Authorization": f"Basic {AUTH_TOKEN}",
    "Accept-Language": "en_US",
    "accept": "application/json"
}

def get_mission_id():
    url = f"http://{MIR_IP}/api/v2.0.0/status"
    response = requests.get(url, headers=HEADERS)
    print(f"[DEBUG] Status request response: {response.status_code}, {response.text}")
    if response.status_code == 200:
        return response.json().get("mission_queue_id")
    return None

def get_mission_positions(mission_id):
    url = f"http://{MIR_IP}/api/v2.0.0/mission_queue/{mission_id}/actions"
    response = requests.get(url, headers=HEADERS)
    print(f"[DEBUG] Mission actions response: {response.status_code}, {response.text}")
    positions = []
    if response.status_code == 200:
        actions = response.json()
        for action in actions:
            print(f"[DEBUG] Action found: {action}")
            if "action_type" in action and action["action_type"] == "move":
                parameters = action.get("parameters", {})
                position = parameters.get("position")
                if position:
                    positions.append(position)
                    print(f"[DEBUG] Move position added: {position}")
    return positions

def get_map_id():
    url = f"http://{MIR_IP}/api/v2.0.0/status"
    response = requests.get(url, headers=HEADERS)
    print(f"[DEBUG] Map ID response: {response.status_code}, {response.text}")
    if response.status_code == 200:
        return response.json().get("map_id")
    return None

def get_paths(map_id):
    url = f"http://{MIR_IP}/api/v2.0.0/maps/{map_id}/paths"
    response = requests.get(url, headers=HEADERS)
    print(f"[DEBUG] Paths response: {response.status_code}, {response.text}")
    if response.status_code == 200:
        return response.json()
    return []

def get_position_coordinates(position_id):
    url = f"http://{MIR_IP}/api/v2.0.0/positions/{position_id}"
    response = requests.get(url, headers=HEADERS)
    print(f"[DEBUG] Position {position_id} response: {response.status_code}, {response.text}")
    if response.status_code == 200:
        data = response.json()
        return data.get("pos_x"), data.get("pos_y"), data.get("orientation")
    return None, None, None

def find_path_sequence(positions, paths):
    path_sequence = []
    for pos_id in positions:
        for path in paths:
            if path["start_pos"].endswith(pos_id):
                path_sequence.append((path["start_pos"], path["goal_pos"]))
                print(f"[DEBUG] Path added: {path['start_pos']} -> {path['goal_pos']}")
                break
    return path_sequence

def main():
    mission_id = get_mission_id()
    if not mission_id:
        print("Erro ao obter missão atual.")
        return
    
    print(f"[DEBUG] Missão ID: {mission_id}")
    positions = get_mission_positions(mission_id)
    if not positions:
        print("Nenhuma posição de movimento encontrada na missão.")
        return
    
    print(f"[DEBUG] Posições da missão: {positions}")
    map_id = get_map_id()
    if not map_id:
        print("Erro ao obter o ID do mapa.")
        return
    
    print(f"[DEBUG] Map ID: {map_id}")
    paths = get_paths(map_id)
    if not paths:
        print("Erro ao obter os caminhos do mapa.")
        return
    
    print(f"[DEBUG] Caminhos disponíveis: {paths}")
    path_sequence = find_path_sequence(positions, paths)
    
    print("Caminho que o MiR irá percorrer:")
    for start, goal in path_sequence:
        start_x, start_y, _ = get_position_coordinates(start.split("/")[-1])
        goal_x, goal_y, _ = get_position_coordinates(goal.split("/")[-1])
        print(f"De ({start_x}, {start_y}) para ({goal_x}, {goal_y})")
    
if __name__ == "__main__":
    main()
