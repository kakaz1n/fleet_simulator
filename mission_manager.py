import threading
import json
import subprocess
import time
import random
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from fastapi import FastAPI
import uvicorn
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from typing import Optional
import tempfile
import requests  # Adicione essa importação no início do arquivo
from mir import MirRobot

class UnassignedDeliveryMissionRequest(BaseModel):
    pickup_area: str
    delivery_area: str
    items: list[str]  # Ex.: ["peca5", "peca6"]


class DeliveryMissionRequest(BaseModel):
    robot_id: str
    pickup_area: str
    delivery_area: str
    items: list[str]  # peças necessárias
    tractor_id: Optional[str] = None  # Novo campo para associar a produção do trator

# 🔹 Modelos de dados para API
class RobotRequest(BaseModel):
    robot_id: str
    x: int
    y: int

class MissionRequest(BaseModel):
    robot_id: str
    goal_x: Optional[float] = None  # Alterado de int para float
    goal_y: Optional[float] = None  # Alterado de int para float
    area_name: Optional[str] = None  # Campo opcional para nome da área

    # ───────── Modelo para criação de ordens de produção de tratores ─────────
class ProductionStep(BaseModel):
    cell: str           # Nome da célula (ex.: "Montagem 2-3")
    kits: list[str]     # Lista de kits (peças) necessários para essa célula

class TractorProductionRequest(BaseModel):
    tractor_id: str
    steps: list[ProductionStep]

MAP_SIZE = 200
GRID_ROWS = 6
GRID_COLS = 6
CELL_WIDTH = 22
CELL_HEIGHT = 22
CORRIDOR_WIDTH = 12
DOOR_SIZE = 8

SECTORS = ["Montagem", "Produção", "Armazenamento", "Expedição"]

AREA_MAP = {}  # Dicionário que mapeia nomes de áreas para coordenadas

def is_inside_any_cell(x, y, cells):
    """Retorna True se (x, y) estiver DENTRO de qualquer célula."""
    for (x1, y1, x2, y2, cell_name) in cells:
        if x1 < x < x2 and y1 < y < y2:
            return True
    return False

def find_center(x1, y1, x2, y2):
    """Retorna a coordenada central (x, y) de um retângulo."""
    return ((x1 + x2) // 2, (y1 + y2) // 2)

def generate_factory_layout():
    """Gera um mapa baseado em células, garantindo que todas tenham pelo menos uma porta."""
    walls = set()
    doors = set()
    cells = []  # lista de (x1, y1, x2, y2, nome)
    
    cell_has_door = {}  # Rastreamento de portas por célula

    # Criar células e paredes ao redor
    for row in range(GRID_ROWS):
        for col in range(GRID_COLS):
            x1 = col * (CELL_WIDTH + CORRIDOR_WIDTH) - MAP_SIZE // 2
            y1 = row * (CELL_HEIGHT + CORRIDOR_WIDTH) - MAP_SIZE // 2
            x2 = x1 + CELL_WIDTH
            y2 = y1 + CELL_HEIGHT

            sector_name = SECTORS[row % len(SECTORS)]
            cell_name = f"{sector_name} {row+1}-{col+1}"
            cells.append((x1, y1, x2, y2, cell_name))
            AREA_MAP[cell_name] = find_center(x1, y1, x2, y2)
            cell_has_door[cell_name] = False  # Inicialmente, nenhuma porta

            # Criar paredes ao redor da célula
            for x in range(x1, x2 + 1):
                walls.add((x, y1))
                walls.add((x, y2))
            for y in range(y1, y2 + 1):
                walls.add((x1, y))
                walls.add((x2, y))

    # Criar portas entre células
    for row in range(GRID_ROWS):
        for col in range(GRID_COLS):
            x1, y1, x2, y2, cell_name = cells[row * GRID_COLS + col]

            # Porta horizontal (entre colunas)
            if col < GRID_COLS - 1:
                px = x2
                py_center = (y1 + y2) // 2
                for i in range(DOOR_SIZE):
                    door_pos = (px, py_center + i)
                    walls.discard(door_pos)
                    doors.add(door_pos)
                cell_has_door[cell_name] = True
                cell_has_door[cells[row * GRID_COLS + col + 1][4]] = True

            # Porta vertical (entre linhas)
            if row < GRID_ROWS - 1:
                py = y2
                px_center = (x1 + x2) // 2
                for i in range(DOOR_SIZE):
                    door_pos = (px_center + i, py)
                    walls.discard(door_pos)
                    doors.add(door_pos)
                cell_has_door[cell_name] = True
                cell_has_door[cells[(row + 1) * GRID_COLS + col][4]] = True

    # Garante que **todas** as células tenham pelo menos uma porta
    for (x1, y1, x2, y2, cell_name) in cells:
        if not cell_has_door[cell_name]:  # Se uma célula não tem porta, adiciona uma
            px_center = (x1 + x2) // 2
            py_center = (y1 + y2) // 2
            # Tenta adicionar uma porta na parte superior ou esquerda
            if row > 0:  # Se não for a primeira linha
                doors.add((px_center, y1))
                walls.discard((px_center, y1))
            elif col > 0:  # Se não for a primeira coluna
                doors.add((x1, py_center))
                walls.discard((x1, py_center))
            else:  # Se for a primeira célula, adiciona uma porta forçada na borda direita
                doors.add((x2, py_center))
                walls.discard((x2, py_center))
            cell_has_door[cell_name] = True

    # Criar corredores
    corridors = set()
    half = MAP_SIZE // 2
    for x in range(-half, half + 1):
        for y in range(-half, half + 1):
            if (x, y) in walls or (x, y) in doors or is_inside_any_cell(x, y, cells):
                continue
            corridors.add((x, y))

    return walls, corridors, cells, doors

# Executar a função
WALLS, CORRIDORS, CELLS, DOORS = generate_factory_layout()



TRACTORS = {}

PIECES_STATUS = {}  # Ex.: { "peca1": "Armazenamento 3-1", ... }

# 🔹 Exibir quais portas estão sendo consideradas
# print(f"🚪 Portas registradas: {DOORS}")
print(f"🚧 Paredes registradas (após remoção de portas): {WALLS}")
# 🔹 Removendo todas as portas das paredes
# WALLS -= set(DOORS)

# 🔹 Filtrando qualquer dado inválido
# WALLS = {wall for wall in WALLS if isinstance(wall[0], (int, float)) and isinstance(wall[1], (int, float))}

#kits de peças
class KitRequest(BaseModel):
    robot_id: str
    items: list[str]  # Lista de identificadores das peças (ex.: códigos, nomes, etc.)

# 🔹 Gerenciador de Missões (ROS2)
class MissionManager(Node):
    def __init__(self):
        super().__init__('mission_manager')
        self.robots = {}
        self.robot_processes = {}

        self.create_subscription(String, 'robot_status', self.robot_status_callback, 10)
        self.mission_pub = self.create_publisher(String, 'robot_mission', 10)
        self.create_subscription(String, '/add_robot', self.add_robot_callback, 10)
        self.create_subscription(String, '/add_mission', self.add_mission_callback, 10)
        self.create_subscription(String, '/add_delivery', self.add_delivery_mission, 10)
        self.create_subscription(String, '/add_tractor', self.create_tractor_production, 10)

        self.timer = self.create_timer(1.0, self.publish_missions)

    def add_robot(self, robot_id, x, y):
        """Cria um novo robô se ele ainda não existir"""
        if robot_id in self.robot_processes:
            self.get_logger().warn(f"⚠️ Robô {robot_id} já existe!")
            return

        # Função para criar arquivos temporários para as listas
        def write_to_tempfile(data):
            temp_file = tempfile.NamedTemporaryFile(delete=False, mode='w', encoding='utf-8')
            json.dump(data, temp_file)
            temp_file.close()
            return temp_file.name
        self.robots[robot_id] = {'position': (x, y), 'goal': None, 'trajectory': [], 'kit': [],'current_delivery': None }
        walls_json = json.dumps(list(WALLS))
        doors_json = json.dumps(list(DOORS))
        corridors_json = json.dumps(list(CORRIDORS))
        cells_json = json.dumps(list(CELLS))
        walls_file = write_to_tempfile(walls_json)
        doors_file = write_to_tempfile(doors_json)
        corridors_file = write_to_tempfile(corridors_json)
        cells_file = write_to_tempfile(str(cells_json))

        process = subprocess.Popen(["python3", "robot.py", robot_id, str(x), str(y),     walls_file, doors_file, corridors_file,cells_file])

        # ✅ Passando argumentos corretamente para o subprocesso
        # process = subprocess.Popen(["python3", "robot.py", robot_id, str(x), str(y), json.dumps(list(WALLS))])
        self.robot_processes[robot_id] = process

        self.get_logger().info(f"✅ Novo robô {robot_id} criado na posição ({x}, {y})")
    def add_mission_callback(self, msg):
        """Recebe uma missão e atribui ao robô"""
        try:
            data = json.loads(msg.data)
            robot_id = data.get("robot_id")
            area_name = data.get("area_name")
            goal = data.get("goal")  # Adicionado para verificar coordenadas diretas

            if not robot_id:
                self.get_logger().error("❌ Erro: Robot ID ausente!")
                return
            
            if robot_id not in self.robots:
                self.get_logger().error(f"❌ Erro: Robô {robot_id} não encontrado!")
                return

            if goal is not None:
                goal_x, goal_y = goal
            elif area_name and area_name in AREA_MAP:
                goal_x, goal_y = AREA_MAP[area_name]
            else:
                self.get_logger().error("❌ Erro: Nenhuma meta válida encontrada!")
                return

            self.get_logger().info(f"✅ Missão recebida para {robot_id}: ({goal_x}, {goal_y})")

            # Atualiza a missão do robô
            self.robots[robot_id]['goal'] = (goal_x, goal_y)

            if self.robots[robot_id]['position'] == self.robots[robot_id]['goal']:
                self.get_logger().info(f"❌ {robot_id} já está no destino.")
                self.robots[robot_id]['goal'] = None
                return

            # Publica a missão no tópico correto
            mission_msg = String()
            mission_msg.data = json.dumps({"robot_id": robot_id, "goal": [goal_x, goal_y]})
            self.mission_pub.publish(mission_msg)

            self.get_logger().info(f"🚀 Missão enviada para {robot_id}: {goal_x}, {goal_y}")

        except json.JSONDecodeError:
            self.get_logger().error("❌ Erro ao decodificar JSON na missão!")

    def add_robot_callback(self, msg):
        """Recebe um comando para adicionar um robô via ROS2."""
        try:
            data = json.loads(msg.data)  # Converte JSON para dicionário
            robot_id = data.get("robot_id")
            x = data.get("x")
            y = data.get("y")

            if robot_id is None or x is None or y is None:
                self.get_logger().error("❌ Erro: Parâmetros incompletos para adicionar robô!")
                return

            self.add_robot(robot_id, x, y)

        except json.JSONDecodeError as e:
            self.get_logger().error(f"❌ Erro ao decodificar JSON: {e}")

        # Modificações na robot_status_callback para atualizar o status das peças:
    def robot_status_callback(self, msg):
        try:
            data = json.loads(msg.data)
            robot_id = data['robot_id']
            position = tuple(data['position'])
            position_name = data['area']
            estimated_time = data.get('estimated_time', None)
            planned_path = data.get('planned_path', [])
            if robot_id in self.robots:
                self.robots[robot_id]['position'] = position
                self.robots[robot_id]['area'] = position_name
                self.robots[robot_id]['estimated_time'] = estimated_time
                self.robots[robot_id]['planned_path'] = planned_path

                delivery = self.robots[robot_id].get('current_delivery')
                if delivery is not None:
                    goal = self.robots[robot_id]['goal']
                    if goal and position == goal:
                        stage = delivery['stage']
                        if stage == "pickup":
                            items = delivery['items']
                            self.robots[robot_id]['kit'].extend(items)
                            self.get_logger().info(
                                f"🔹 {robot_id} fez pickup das peças {items} em {delivery['pickup_area']}"
                            )
                            delivery['stage'] = "delivery"
                            new_goal = delivery['delivery_coords']
                            self.robots[robot_id]['goal'] = new_goal
                            mission_msg = String()
                            mission_msg.data = json.dumps({
                                "robot_id": robot_id,
                                "goal": [new_goal[0], new_goal[1]]
                            })
                            self.mission_pub.publish(mission_msg)
                            self.get_logger().info(
                                f"🚀 Próxima etapa: entregar peças em {delivery['delivery_area']}"
                            )
                        elif stage == "delivery":
                            items = delivery['items']
                            for item in items:
                                if item in self.robots[robot_id]['kit']:
                                    self.robots[robot_id]['kit'].remove(item)
                                    # Atualiza o status da peça para a célula de entrega
                                    PIECES_STATUS[item] = delivery['delivery_area']
                            self.get_logger().info(
                                f"🔸 {robot_id} entregou as peças {items} em {delivery['delivery_area']}"
                            )
                            # Agora, para cada trator, verifica se este delivery satisfaz alguma etapa:
                            for tractor_id, production in TRACTORS.items():
                                for step in production['steps']:
                                    if step['cell'] == delivery['delivery_area']:
                                        # Adiciona as peças entregues no dicionário 'delivered'
                                        delivered_set = production['delivered'].get(step['cell'], set())
                                        delivered_set.update(items)
                                        production['delivered'][step['cell']] = delivered_set
                                        # Se todas as peças necessárias foram entregues, marca a etapa como concluída
                                        if set(step['kits']).issubset(delivered_set):
                                            if step['cell'] not in production['completed_steps']:
                                                production['completed_steps'].append(step['cell'])
                                                self.get_logger().info(
                                                    f"✅ Etapa '{step['cell']}' concluída na produção do trator {tractor_id}"
                                                )
                            self.robots[robot_id]['current_delivery'] = None
                            self.robots[robot_id]['goal'] = None
                            self.get_logger().info(
                                f"✅ Entrega finalizada para {robot_id}"
                            )
                self.get_logger().info(f"📡 {robot_id} atualizado para {position}")
        except json.JSONDecodeError:
            self.get_logger().error("❌ Erro ao processar status do robô!")


    def publish_missions(self):
        """Envia missões ativas para os robôs, apenas se ainda não chegaram ao destino"""
        for robot_id, info in self.robots.items():
            if info['goal'] is None:
                continue  # Se não há um objetivo, não envia missão

            # Se a posição do robô já for igual ao objetivo, remove a missão
            if info['position'] == info['goal']:
                self.get_logger().info(f"✅ {robot_id} já chegou ao destino {info['goal']}, missão cancelada.")
                info['goal'] = None  # Remove a missão
                continue  # Não publica a missão novamente

            mission_msg = String()
            mission_msg.data = json.dumps({"robot_id": robot_id, "goal": info['goal']})
            self.mission_pub.publish(mission_msg)
            self.get_logger().info(f"📜 Missão enviada para {robot_id}: {info['goal']}")

    def add_delivery_mission(self, msg):
        """Recebe uma missão de entrega via ROS2 e a adiciona ao sistema."""
        try:
            data = json.loads(msg.data)  # Converte a string JSON em um dicionário

            if data["robot_id"] not in self.robots:
                self.get_logger().error(f"❌ Erro: Robô '{data['robot_id']}' não encontrado!")
                return

            robot_info = self.robots[data["robot_id"]]

            if data["pickup_area"] not in AREA_MAP:
                self.get_logger().error(f"❌ Erro: Área de pickup '{data['pickup_area']}' não encontrada!")
                return

            if data["delivery_area"] not in AREA_MAP:
                self.get_logger().error(f"❌ Erro: Área de entrega '{data['delivery_area']}' não encontrada!")
                return

            pickup_coords = AREA_MAP[data["pickup_area"]]
            delivery_coords = AREA_MAP[data["delivery_area"]]

            # Atualiza a missão do robô
            robot_info["current_delivery"] = {
                "stage": "pickup",
                "items": data["items"],
                "pickup_area": data["pickup_area"],
                "pickup_coords": pickup_coords,
                "delivery_area": data["delivery_area"],
                "delivery_coords": delivery_coords,
                "tractor_id": data.get("tractor_id", None)  # Pode ser None se não informado
            }

            robot_info["goal"] = pickup_coords

            # Publica a missão no tópico `/robot_mission`
            mission_msg = String()
            mission_msg.data = json.dumps({
                "robot_id": data["robot_id"],
                "goal": [pickup_coords[0], pickup_coords[1]]
            })
            self.mission_pub.publish(mission_msg)

            self.get_logger().info(f"🚀 Missão de entrega iniciada para {data['robot_id']}! Buscar peças em '{data['pickup_area']}'.")

        except json.JSONDecodeError as e:
            self.get_logger().error(f"❌ Erro ao decodificar JSON: {e}")


    def create_tractor_production(self, msg):
        """Cria um trator a partir da mensagem recebida pelo tópico ROS2"""
        global TRACTORS, PIECES_STATUS

        try:
            data = json.loads(msg.data)  # Converte string JSON para dicionário

            if data["tractor_id"] in TRACTORS:
                self.get_logger().error(f"🚨 Trator {data['tractor_id']} já existe!")
                return

            TRACTORS[data["tractor_id"]] = {
                "steps": data["steps"],
                "completed_steps": [],
                "delivered": {step["cell"]: set() for step in data["steps"]}
            }

            # Inicializa o status das peças nos depósitos
            for step in data["steps"]:
                for piece in step["kits"]:
                    if piece not in PIECES_STATUS:
                        PIECES_STATUS[piece] = f"Armazenamento 3-{random.randint(1,6)}"

            self.get_logger().info(f"✅ Trator {data['tractor_id']} criado com sucesso!")
        
        except json.JSONDecodeError as e:
            self.get_logger().error(f"❌ Erro ao decodificar JSON: {e}")

# 🔹 Inicializa API FastAPI
app = FastAPI()

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

mission_manager = None  # Será inicializado no main()

@app.get("/status")
def get_status():
    return {
        "robots": mission_manager.robots,
        "walls": list(WALLS),
        "areas": [{"x1": x1, "y1": y1, "x2": x2, "y2": y2, "name": name} for x1, y1, x2, y2, name in CELLS],
        "doors": list(DOORS)
    }

@app.post("/add_robot")
def add_robot(data: RobotRequest):
    """Adiciona um novo robô"""
    if mission_manager is None:
        return {"error": "Mission Manager não foi inicializado"}
    
    if data.robot_id in mission_manager.robots:
        return {"error": f"Robô {data.robot_id} já existe"}

    mission_manager.add_robot(data.robot_id, data.x, data.y)
    return {"message": f"🤖 Robô {data.robot_id} adicionado na posição ({data.x}, {data.y})"}

@app.post("/add_mission")
def add_mission(data: MissionRequest):
    """Atribui uma missão a um robô"""
    if mission_manager is None:
        return {"error": "Mission Manager não foi inicializado"}
    
    if data.robot_id not in mission_manager.robots:
        return {"error": f"Robô '{data.robot_id}' não encontrado"}

    print(f"🔹 Recebido: robot_id='{data.robot_id}', area_name='{data.area_name}', goal=({data.goal_x}, {data.goal_y})")

    goal_x, goal_y = None, None

    # Se um nome de área foi fornecido, converte para coordenadas
    if data.area_name and data.area_name in AREA_MAP:
        goal_x, goal_y = AREA_MAP[data.area_name]
        print(f"📍 Área '{data.area_name}' encontrada, atribuindo coordenadas ({goal_x}, {goal_y})")
    elif data.goal_x is not None and data.goal_y is not None:
        goal_x, goal_y = round(data.goal_x), round(data.goal_y)  # 🔹 Arredondar para inteiro
        print(f"📍 Nenhuma área fornecida, usando coordenadas clicadas ({goal_x}, {goal_y})")
    else:
        print("❌ Nenhuma coordenada válida fornecida!")
        return {"error": "Nenhuma coordenada válida ou área fornecida!"}

    # Confirma que as coordenadas foram corretamente definidas
    if goal_x is None or goal_y is None:
        print("❌ Erro: goal_x ou goal_y ainda são None após processamento.")
        return {"error": "Erro ao determinar a missão!"}

    # Atribui a missão ao robô
    mission_manager.robots[data.robot_id]['goal'] = (goal_x, goal_y)

    print(f"✅ Missão atribuída a {data.robot_id}: ({goal_x}, {goal_y})")
    return {"message": f"✅ Missão atribuída a {data.robot_id}: ({goal_x}, {goal_y})"}

@app.post("/add_kit")
def add_kit(data: KitRequest):
    """
    Adiciona (carrega) peças ao kit do robô indicado.
    Exemplo de JSON de entrada:
        {
            "robot_id": "R1",
            "items": ["peca_a", "peca_b"]
        }
    """
    if data.robot_id not in mission_manager.robots:
        return {"error": f"Robô {data.robot_id} não encontrado"}

    # Adiciona as peças ao kit atual do robô
    mission_manager.robots[data.robot_id]['kit'].extend(data.items)

    return {
        "message": f"Peças {data.items} adicionadas ao kit do robô {data.robot_id}",
        "kit_atual": mission_manager.robots[data.robot_id]['kit']
    }

@app.post("/remove_kit")
def remove_kit(data: KitRequest):
    """
    Remove (descarrega) as peças informadas do kit do robô.
    Exemplo de JSON de entrada:
        {
            "robot_id": "R1",
            "items": ["peca_a"]
        }
    """
    if data.robot_id not in mission_manager.robots:
        return {"error": f"Robô {data.robot_id} não encontrado"}

    current_kit = mission_manager.robots[data.robot_id]['kit']
    for item in data.items:
        if item in current_kit:
            current_kit.remove(item)

    return {
        "message": f"Peças {data.items} removidas do kit do robô {data.robot_id}",
        "kit_atual": mission_manager.robots[data.robot_id]['kit']
    }

@app.post("/add_delivery_mission")
def add_delivery_mission(data: DeliveryMissionRequest):
    if data.robot_id not in mission_manager.robots:
        return {"error": f"Robô '{data.robot_id}' não encontrado"}

    robot_info = mission_manager.robots[data.robot_id]

    if data.pickup_area not in AREA_MAP:
        return {"error": f"Área de pickup '{data.pickup_area}' não encontrada no mapa"}

    if data.delivery_area not in AREA_MAP:
        return {"error": f"Área de entrega '{data.delivery_area}' não encontrada no mapa"}

    pickup_coords = AREA_MAP[data.pickup_area]
    delivery_coords = AREA_MAP[data.delivery_area]

    # Armazena também o tractor_id, se fornecido
    robot_info['current_delivery'] = {
        "stage": "pickup",
        "items": data.items,
        "pickup_area": data.pickup_area,
        "pickup_coords": pickup_coords,
        "delivery_area": data.delivery_area,
        "delivery_coords": delivery_coords,
        "tractor_id": data.tractor_id
    }

    robot_info['goal'] = pickup_coords

    mission_msg = String()
    mission_msg.data = json.dumps({
        "robot_id": data.robot_id,
        "goal": [pickup_coords[0], pickup_coords[1]]
    })
    mission_manager.mission_pub.publish(mission_msg)

    return {
        "message": f"Missão de entrega iniciada para {data.robot_id}. Primeiro objetivo: buscar peças em '{data.pickup_area}'."
    }


@app.post("/request_delivery_mission")
def request_delivery_mission(data: UnassignedDeliveryMissionRequest):
    """
    Cria uma missão de entrega sem especificar o robô.
    O sistema escolhe automaticamente o robô livre mais próximo do pickup.
    """
    # 1) Obter coordenadas do pickup e delivery
    if data.pickup_area not in AREA_MAP:
        return {"error": f"Pickup '{data.pickup_area}' não encontrada no mapa."}
    if data.delivery_area not in AREA_MAP:
        return {"error": f"Delivery '{data.delivery_area}' não encontrada no mapa."}

    pickup_coords = AREA_MAP[data.pickup_area]     # (px, py)
    delivery_coords = AREA_MAP[data.delivery_area] # (dx, dy)

    # 2) Encontrar robô livre mais próximo do pickup
    best_robot_id = None
    best_distance = float("inf")

    for r_id, r_info in mission_manager.robots.items():
        # Verifica se o robô está ocupado:
        #   - se tiver 'current_delivery' != None, está em missão;
        #   - ou se tiver 'goal' != None e a 'current_delivery' for algo custom.
        if r_info.get("current_delivery") is not None:
            continue  # já está ocupado com entrega

        # (Opcional) Se quiser considerar qualquer robô com 'goal' == None como livre:
        # if r_info["goal"] is not None:
        #     continue

        # Calcula distância simples (Manhattan ou Euclidiana) ao local de pickup
        (rx, ry) = r_info['position']
        (px, py) = pickup_coords
        dist = abs(rx - px) + abs(ry - py)  # Manhattan
        # Ou dist = math.hypot(rx - px, ry - py)  # Euclidiana

        if dist < best_distance:
            best_distance = dist
            best_robot_id = r_id

    if best_robot_id is None:
        return {"error": "Não há robô livre para assumir esta missão."}

    # 3) Tendo o robô selecionado, define a missão
    robot_info = mission_manager.robots[best_robot_id]
    robot_info['current_delivery'] = {
        "stage": "pickup",
        "items": data.items,
        "pickup_area": data.pickup_area,
        "pickup_coords": pickup_coords,
        "delivery_area": data.delivery_area,
        "delivery_coords": delivery_coords
    }
    # Ajusta o goal para o pickup
    robot_info['goal'] = pickup_coords

    # Publicar no tópico para o robô
    mission_msg = String()
    mission_msg.data = json.dumps({
        "robot_id": best_robot_id,
        "goal": [pickup_coords[0], pickup_coords[1]]
    })
    mission_manager.mission_pub.publish(mission_msg)

    return {
        "message": f"Missão atribuída ao robô '{best_robot_id}'."
                   f" (Pickup: {data.pickup_area}, Delivery: {data.delivery_area}, Itens: {data.items})"
    }

# ───────── NOVOS ENDPOINTS PARA A PRODUÇÃO DE TRATORES ─────────
@app.get("/tractor_production")
def get_tractor_production():
    return TRACTORS

@app.get("/trator_status")
def get_trator_status():
    # Você pode incluir também a produção do trator no status geral
    return {       
        "tractors": TRACTORS
    }
# Atualize o endpoint de criação do trator para inicializar o status das peças:
@app.post("/tractor_production")
def create_tractor_production(data: TractorProductionRequest):
    global TRACTORS, PIECES_STATUS
    if data.tractor_id in TRACTORS:
        return {"error": "Trator já existe"}
    # Inicializa a ordem de produção com:
    # - steps: as etapas necessárias;
    # - completed_steps: lista inicialmente vazia;
    # - delivered: dicionário que associará cada célula a um conjunto de peças entregues.
    TRACTORS[data.tractor_id] = {
        "steps": [step.dict() for step in data.steps],
        "completed_steps": [],
        "delivered": {step.cell: set() for step in data.steps}
    }
    # Inicializa cada peça com um local aleatório entre Armazenamento 3-1 e Armazenamento 3-6
    for step in data.steps:
        for piece in step.kits:
            if piece not in PIECES_STATUS:
                PIECES_STATUS[piece] = f"Armazenamento 3-{random.randint(1,6)}"
    return {"message": f"Trator {data.tractor_id} criado com sucesso!"}


@app.post("/trigger_tractor_missions")
def trigger_tractor_missions(data: dict):
    tractor_id = data.get("tractor_id")
    if not tractor_id or tractor_id not in TRACTORS:
        return {"error": "Trator não encontrado"}
    
    production = TRACTORS[tractor_id]
    missions_triggered = []

    # Para cada etapa da produção que ainda não foi completada:
    for step in production["steps"]:
        cell = step["cell"]
        for piece in step["kits"]:
            # Se a peça ainda não estiver na célula requerida:
            if PIECES_STATUS.get(piece) != cell:
                # Local atual da peça
                current_location = PIECES_STATUS.get(piece, "Armazenamento 3-1")
                pickup_coords = AREA_MAP.get(current_location)
                if not pickup_coords:
                    continue  # Pula se não houver coordenadas definidas

                # Procura um robô livre para realizar a missão (usando distância Manhattan)
                best_robot_id = None
                best_distance = float("inf")
                for r_id, r_info in mission_manager.robots.items():
                    if r_info.get("current_delivery") is not None:
                        continue
                    (rx, ry) = r_info['position']
                    (px, py) = pickup_coords
                    dist = abs(rx - px) + abs(ry - py)
                    if dist < best_distance:
                        best_distance = dist
                        best_robot_id = r_id

                if best_robot_id:
                    delivery_coords = AREA_MAP.get(cell)
                    if not delivery_coords:
                        continue
                    # Cria os dados da missão para essa peça
                    mission_data = {
                        "pickup_area": current_location,
                        "delivery_area": cell,
                        "items": [piece]
                    }
                    # Dispara a missão usando requests para chamar o endpoint /request_delivery_mission
                    try:
                        res = requests.post("http://127.0.0.1:8000/request_delivery_mission", json=mission_data)
                        res.raise_for_status()
                    except Exception as e:
                        mission_manager.get_logger().error(f"Erro ao disparar missão: {e}")
                        continue

                    missions_triggered.append({
                        "robot": best_robot_id,
                        "piece": piece,
                        "from": current_location,
                        "to": cell
                    })
                    # Atualiza o status da peça para "em trânsito"
                    PIECES_STATUS[piece] = "em trânsito"

    return {"message": "Missões acionadas", "missions": missions_triggered}

# Novo endpoint para consultar o status das peças
@app.get("/pieces_status")
def get_pieces_status():
    return PIECES_STATUS

@app.put("/tractor_production/{tractor_id}")
def update_tractor_production(tractor_id: str, data: TractorProductionRequest):
    global TRACTORS
    if tractor_id not in TRACTORS:
        return {"error": "Trator não encontrado"}
    
    current_production = TRACTORS[tractor_id]
    # Converta as novas etapas para dicionários
    new_steps = [step.dict() for step in data.steps]
    new_delivered = {}
    new_completed = []
    # Para cada nova etapa, verifique se ela já existia e preserve o delivered correspondente.
    for step in new_steps:
        cell = step["cell"]
        # Se já existia, preserva o conjunto; caso contrário, inicia vazio.
        if "delivered" in current_production and cell in current_production["delivered"]:
            delivered_set = current_production["delivered"][cell]
        else:
            delivered_set = set()
        new_delivered[cell] = delivered_set
        # Verifica se o conjunto entregue já satisfaz as peças necessárias.
        if set(step["kits"]).issubset(delivered_set):
            new_completed.append(cell)
    
    # Atualiza a ordem de produção preservando as informações de entregas anteriores
    current_production["steps"] = new_steps
    current_production["delivered"] = new_delivered
    current_production["completed_steps"] = new_completed

    return {"message": f"Trator {tractor_id} atualizado com sucesso!", "tractor": current_production}

@app.delete("/tractor_production/{tractor_id}")
def delete_tractor_production(tractor_id: str):
    if tractor_id not in TRACTORS:
        return {"error": "Trator não encontrado"}
    del TRACTORS[tractor_id]
    return {"message": f"Trator {tractor_id} removido com sucesso!"}


mir = MirRobot()

@app.get("/mir_status") 
def get_mir_status(): 
    """ Endpoint para consultar o status do MIR. """ 
    try: 
        status = mir.get_status() 
        return status 
    except Exception as e: 
        return {"error": str(e)}


@app.put("/mir_move")
def move_mir(goal_x: int, goal_y: int): 
    """ Endpoint para mover o MIR para as coordenadas informadas. Exemplo de chamada: PUT /mir_move?goal_x=50&goal_y=100 """ 
    try: 
        response = mir.move_to(goal_x, goal_y) 
        return response 
    except Exception as e: 
        return {"error": str(e)}

def main():
    global mission_manager
    rclpy.init()
    mission_manager = MissionManager()
    
    threading.Thread(target=rclpy.spin, args=(mission_manager,), daemon=True).start()
    uvicorn.run(app, host="0.0.0.0", port=8000, log_level="info")

if __name__ == "__main__":
    main()
