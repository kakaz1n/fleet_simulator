import requests
import asyncio
import websockets
import json

class MirRobot: 
    def __init__(self, 
                 base_url="http://10.83.131.155/api/v2.0.0", 
                 ws_url="ws://10.83.131.155:9090",
                 auth_token="Basic ZGlzdHJpYnV0b3I6NjJmMmYwZjFlZmYxMGQzMTUyYzk1ZjZmMDU5NjU3NmU0ODJiYjhlNDQ4MDY0MzNmNGNmOTI5NzkyODM0YjAxNA==",
                 language="en_US"):
        """
        Inicializa a classe do MIR com a URL base da API, a URL do WebSocket e os cabeçalhos necessários.
        
        :param base_url: URL base da API do MIR.
        :param ws_url: URL do WebSocket do ROS para comunicação em tempo real.
        :param auth_token: Token de autorização.
        :param language: Cabeçalho de idioma.
        """
        self.base_url = base_url 
        self.ws_url = ws_url
        self.headers = {
            "Authorization": auth_token,
            "Accept-Language": language,
            "accept": "application/json"
        }

    async def get_status(self): #####QUANDO NAO TEM ELE TRAVA O SISTEMA
        url = f"{self.base_url}/status"
        response = requests.get(url, headers=self.headers)
        response.raise_for_status()
        status_data = response.json()

        # Obtém o caminho da missão de forma assíncrona
        status_data["path"] = await self.get_mission_path_ws()

        return status_data

    async def get_mission_path_ws(self, timeout: float = 5.0) -> list:
        """
        Obtém o caminho da missão via WebSocket, ouvindo o tópico '/mirwebapp/web_path'.
        Retorna uma lista de coordenadas (x, y).
        Se não receber nenhuma mensagem em 'timeout' segundos, retorna lista vazia.
        """
        uri = self.ws_url
        path_data = []

        try:
            async with websockets.connect(uri) as websocket:
                # Monta a mensagem de subscrição
                subscribe_msg = {
                    "op": "subscribe",
                    "topic": "/mirwebapp/web_path"
                }
                await websocket.send(json.dumps(subscribe_msg))

                # Usa asyncio.wait_for para evitar bloqueio indefinido:
                mensagem = await asyncio.wait_for(websocket.recv(), timeout=timeout)

                data = json.loads(mensagem)
                if "msg" in data:
                    caminho = data["msg"]
                    # Cria lista de dicionários [{x:..., y:...}, ...]
                    path_data = [
                        {"x": x, "y": y}
                        for x, y in zip(caminho["x"], caminho["y"])
                    ]

                # Opcional: Fazer 'unsubscribe' antes de encerrar,
                # se desejarmos interromper o recebimento
                unsubscribe_msg = {
                    "op": "unsubscribe",
                    "topic": "/mirwebapp/web_path"
                }
                await websocket.send(json.dumps(unsubscribe_msg))

        except asyncio.TimeoutError:
            print(f"⚠️ Timeout de {timeout}s ao aguardar dados do WebSocket.")
        except Exception as e:
            print(f"🚨 Erro ao conectar ou receber do WebSocket: {e}")

        return path_data
    
    def move_to(self, goal_x, goal_y):
        url = f"{self.base_url}/status"
        payload = {
            "position": {"x": goal_x, "y": goal_y},
            "mission_text": "Mover para nova posição"
        }
        response = requests.put(url, json=payload, headers=self.headers)
        response.raise_for_status()
        return response.json()

    def send_robot_command(self, command, params=None):
        url = f"{self.base_url}/robots"
        payload = {"command": command}
        if params:
            payload.update(params)
        response = requests.post(url, json=payload, headers=self.headers)
        response.raise_for_status()
        return response.json()

    def get_map_data(self, map_id):
        url = f"{self.base_url}/maps/{map_id}/positions"
        response = requests.get(url, headers=self.headers)
        response.raise_for_status()
        return response.json()

    def update_on_map(self):
        status = self.get_status()
        position = status.get("position", {})
        return {
            "x": position.get("x"),
            "y": position.get("y"),
            "robot_name": status.get("robot_name"),
            "battery": status.get("battery_percentage"),
            "state": status.get("state_text"),
            'path': self.get_mission_path_ws()
        }
    
    def get_position_details(self, position_guid):
        """
        Recupera os detalhes de uma posição (incluindo coordenadas x e y)
        a partir do seu GUID.
        
        :param position_guid: GUID da posição.
        :return: Dicionário com os detalhes da posição.
        """
        url = f"{self.base_url}/positions/{position_guid}"
        response = requests.get(url, headers=self.headers)
        response.raise_for_status()
        return response.json()

    def get_current_mission_guid(self):
        """
        Retorna o identificador da missão que o robô está executando,
        extraído do campo 'mission_queue_url' (ou 'mission_queue_id' se não houver URL).
        
        :return: ID da missão em execução.
        """
        status = self.get_status()
        mission_url = status.get("mission_queue_url", "")
        if mission_url:
            # Supondo que mission_url esteja no formato "/v2.0.0/mission_queue/{mission_id}"
            mission_id = mission_url.strip("/").split("/")[-1]
            return mission_id
        return status.get("mission_queue_id")
    
    def get_mission_actions(self, mission_id):
        """
        Recupera a lista de ações da missão atual.
        
        :param mission_id: ID da missão.
        :return: Lista de ações (cada ação contém 'url' e 'id').
        """
        url = f"{self.base_url}/mission_queue/{mission_id}/actions"
        response = requests.get(url, headers=self.headers)
        response.raise_for_status()
        return response.json()
    
    def get_action_details(self, mission_id, action_id):
        """
        Recupera os detalhes de uma ação específica da missão.
        
        :param mission_id: ID da missão.
        :param action_id: ID da ação.
        :return: Dicionário com os detalhes da ação.
        """
        url = f"{self.base_url}/mission_queue/{mission_id}/actions/{action_id}"
        response = requests.get(url, headers=self.headers)
        response.raise_for_status()
        return response.json()

    def get_mission_move_positions(self):
        """
        Automatiza o fluxo para obter as posições (coordenadas x,y) que o robô
        irá percorrer na missão atual:
        
          1. Obtém o ID da missão a partir do /status.
          2. Recupera a lista de ações da missão.
          3. Para cada ação, obtém seus detalhes e filtra aquelas do tipo "move".
          4. Para cada ação "move", extrai o GUID da posição (no campo 'parameters')
             e recupera seus detalhes (incluindo coordenadas).
        
        :return: Lista de dicionários com os detalhes (incluindo x e y) de cada posição.
        """
        mission_id = self.get_current_mission_guid()
        print(mission_id)
        if not mission_id:
            print("Missão atual não encontrada no status.")
            return None

        try:
            actions = self.get_mission_actions(mission_id)
        except requests.HTTPError as e:
            print("Erro ao obter a lista de ações da missão:", e)
            return None

        move_positions = []
        for action in actions:
            action_id = action.get("id")
            try:
                action_detail = self.get_action_details(mission_id, action_id)
            except requests.HTTPError as e:
                print(f"Erro ao obter detalhes da ação {action_id}: {e}")
                continue

            # Filtra ações do tipo "move" (ou ajuste conforme o action_type esperado)
            if action_detail.get("action_type") == "move":
                parameters = action_detail.get("parameters", {})
                position_guid = parameters.get("position")
                if position_guid:
                    try:
                        pos_detail = self.get_position_details(position_guid)
                        move_positions.append(pos_detail)
                    except requests.HTTPError as e:
                        print(f"Erro ao obter detalhes da posição {position_guid}: {e}")
        return move_positions


# Exemplo de uso:
if __name__ == "__main__":
    robot = MirRobot()
    
    # positions = robot.get_mission_move_positions()
    # if positions:
    #     print("Posições do caminho planejado para a missão atual:")
    #     for pos in positions:
    #         # Ajuste os campos conforme o formato da resposta da API para a posição
    #         x = pos.get("x") or pos.get("position", {}).get("x")
    #         y = pos.get("y") or pos.get("position", {}).get("y")
    #         print(f"Posição: x={x}, y={y}")
    # else:
    #     print("Não foi possível recuperar as posições do caminho planejado.")
