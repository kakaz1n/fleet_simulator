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
import os
from fastapi.responses import JSONResponse
from typing import List
import traceback
import ast
from fastapi import WebSocket, WebSocketDisconnect
import asyncio

class UnassignedDeliveryMissionRequest(BaseModel):
    pickup_area: str
    delivery_area: str
    items: List[str]
    tractor_id: str  # ✅ Adicionado para suportar identificação do trator


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

CELLS_STATUS = {cell[4]: "noPiece" for cell in CELLS}  # Inicializa todas as células como "noPiece"


TRACTORS = {}

PIECES_STATUS = {}  # Agora será {(piece, tractor_id): "Armazenamento 3-1"}
# 🔹 Histórico de missões realizadas
MISSION_HISTORY = {
    "active": {},   # Missões em andamento {piece_id: {"robot": r_id, "status": "pickup" | "delivery"}}
    "completed": set()  # Peças já entregues
}
# 🔹 Exibir quais portas estão sendo consideradas
# print(f"🚪 Portas registradas: {DOORS}")
# print(f"🚧 Paredes registradas (após remoção de portas): {WALLS}")
HISTORICO_FILE = "historico.json"

historico_estados = []

mir_x = mir_y = mir_orientation = mir_battery = 0
mir_mission = mir_state = mir_mode = mir_nome = ""

def load_partial_json_array(filename):
    """
    Tenta ler o arquivo como uma lista JSON de objetos
    e retorna apenas os itens válidos até encontrar algo corrompido.

    Exemplo:
    [
      {"timestamp":..., ...}, 
      {"timestamp":..., ...}, 
      (AQUI TRUNCADO)...
    ]

    Então retornará uma lista apenas com os objetos válidos anteriores
    ao ponto em que falha.
    """
    with open(filename, "r", encoding="utf-8") as f:
        data = f.read()

    decoder = json.JSONDecoder()
    arr = []
    idx = 0
    length = len(data)

    # Pular espaços iniciais:
    while idx < length and data[idx].isspace():
        idx += 1

    # Verifica se o arquivo começa com '[' (lista)
    if idx < length and data[idx] == '[':
        idx += 1  # avança o '['

        # Ler objetos, um por um
        while idx < length:
            # Remove espaços
            while idx < length and data[idx].isspace():
                idx += 1
            if idx >= length:
                break

            # Se encontrar ']', acabou a lista
            if data[idx] == ']':
                idx += 1
                break

            # Tentar decodificar um objeto
            try:
                obj, new_pos = decoder.raw_decode(data, idx)
                arr.append(obj)
                idx = new_pos
            except json.JSONDecodeError:
                # Ao falhar, interrompemos e retornamos o que deu para ler
                break

            # Pular espaços
            while idx < length and data[idx].isspace():
                idx += 1

            # Se houver uma vírgula, pula
            if idx < length and data[idx] == ',':
                idx += 1
    else:
        # Se não começa com '[', talvez seja um JSON diferente (objeto único ou corrompido)
        # Tentamos carregar diretamente
        try:
            single = json.loads(data)
            # Se for um objeto único, retornamos em lista
            return [single]
        except:
            # Corrompido - não deu para ler nada
            return []

    return arr

def corrigir_json_corrompido(filename):
    if not os.path.exists(filename):
        print(f"📂 Arquivo '{filename}' não existe. Criando um novo...")
        with open(filename, "w", encoding="utf-8") as f:
            json.dump([], f, indent=2, ensure_ascii=False)
        return  # Sai da função se o arquivo não existia

    with open(filename, "r", encoding="utf-8") as f:
        data = f.read()

    try:
        json.loads(data)  # Testa se o JSON está válido
        print("✅ JSON está válido!")
    except json.JSONDecodeError:
        print("⚠️ JSON corrompido! Tentando recuperar...")
        # Tentativa de truncar até o último JSON válido
        pos = data.rfind("}")
        if pos != -1:
            data = data[:pos+1] + "]"
            try:
                historico_valido = json.loads(data)
                with open(filename, "w", encoding="utf-8") as f:
                    json.dump(historico_valido, f, indent=2, ensure_ascii=False)
                print("✅ JSON foi corrigido e salvo!")
            except json.JSONDecodeError:
                print("❌ Não foi possível corrigir o JSON automaticamente.")
        else:
            print("❌ Arquivo JSON completamente corrompido. Criando um novo...")
            with open(filename, "w", encoding="utf-8") as f:
                json.dump([], f, indent=2, ensure_ascii=False)



def carregar_historico():
    """
    Carrega o histórico de estados de um arquivo JSON, se existir,
    aproveitando o que for válido se estiver corrompido no final.
    """
    global historico_estados
    if os.path.exists(HISTORICO_FILE):
        try:
            # Tenta carregar parcialmente
            estados = load_partial_json_array(HISTORICO_FILE)
            historico_estados = estados
            # print(f"✅ Histórico carregado de '{HISTORICO_FILE}' com {len(historico_estados)} estados (possivelmente parcial).")
            # print(f"🔍 Histórico carregado: {historico_estados}")

        except Exception as e:
            print(f"⚠️ Erro ao carregar histórico de '{HISTORICO_FILE}': {e}")
            # Mantém historico_estados vazio ou reverte a um backup, se preferir
            historico_estados = []
    else:
        print(f"📂 Arquivo '{HISTORICO_FILE}' não existe. Será criado ao salvar.")
        salvar_historico()


def salvar_historico():
    """
    Salva o histórico de estados em um arquivo JSON.
    """
    try:
        with open(HISTORICO_FILE, "w", encoding="utf-8") as f:
            json.dump(historico_estados, f, indent=2, ensure_ascii=False)
        print(f"💾 Histórico salvo em '{HISTORICO_FILE}'.")
    except Exception as e:
        print(f"❌ Erro ao salvar histórico em '{HISTORICO_FILE}': {e}")

def registrar_estado():
    global historico_estados
    global mission_manager
    global TRACTORS
    global PIECES_STATUS
    global CELLS_STATUS

    # print(f"🔍 mission_manager.robots antes do registro: {mission_manager.robots}")

    atualizar_status_celulas()  # Atualiza o status antes de salvar no histórico

    robots_clean = {r_id: {
        "position": info.get("position"),
        "goal": info.get("goal"),
        "area": info.get("area"),
        "estimated_time": info.get("estimated_time"),
        "planned_path": info.get("planned_path", []),
        "kit": info.get("kit", []),
        "current_delivery": info.get("current_delivery")
    } for r_id, info in mission_manager.robots.items()}

    mir_data = get_mir_data()  # ou mission_manager.mir.get_status(), etc.

    TRACTORS_CLEAN = {
        tractor_id: {
            "steps": tractor_info.get("steps", []),
            "completed_steps": tractor_info.get("completed_steps", []),
            "delivered": {cell: list(pieces) for cell, pieces in tractor_info.get("delivered", {}).items()}
        }
        for tractor_id, tractor_info in TRACTORS.items()
    }

    snapshot = {
        "timestamp": time.time(),
        "robots": robots_clean,
        "tractors": json.loads(json.dumps(TRACTORS_CLEAN)),
        "pieces_status": {str(k): v for k, v in PIECES_STATUS.items()},
        "cells_status": json.loads(json.dumps(CELLS_STATUS)),  # Inclui o status das células
        # "doors": list(DOORS),
        # "walls": list(WALLS),
        # "corridors": list(CORRIDORS),
        "cells": CELLS,
        "mir": mir_data
    }

    historico_estados.append(snapshot)
    salvar_historico()  # <-- Salva no arquivo JSON sempre que registramos um novo estado



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
        # self.create_subscription(String, '/add_mission', self.add_mission_callback, 10)
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
    # def add_mission_callback(self, msg):
    #     """Recebe uma missão e atribui ao robô"""
    #     try:
    #         data = json.loads(msg.data)
    #         robot_id = data.get("robot_id")
    #         area_name = data.get("area_name")
    #         goal = data.get("goal")  # Adicionado para verificar coordenadas diretas

    #         if not robot_id:
    #             self.get_logger().error("❌ Erro: Robot ID ausente!")
    #             return
            
    #         if robot_id not in self.robots:
    #             self.get_logger().error(f"❌ Erro: Robô {robot_id} não encontrado!")
    #             return

    #         if goal is not None:
    #             goal_x, goal_y = goal
    #         elif area_name and area_name in AREA_MAP:
    #             goal_x, goal_y = AREA_MAP[area_name]
    #         else:
    #             self.get_logger().error("❌ Erro: Nenhuma meta válida encontrada!")
    #             return

    #         self.get_logger().info(f"✅ Missão recebida para {robot_id}: ({goal_x}, {goal_y})")

    #         # Atualiza a missão do robô
    #         self.robots[robot_id]['goal'] = (goal_x, goal_y)

    #         if self.robots[robot_id]['position'] == self.robots[robot_id]['goal']:
    #             self.get_logger().info(f"❌ {robot_id} já está no destino.")
    #             self.robots[robot_id]['goal'] = None
    #             return

    #         # Publica a missão no tópico correto
    #         mission_msg = String()
    #         mission_msg.data = json.dumps({"robot_id": robot_id, "goal": [goal_x, goal_y]})
    #         self.mission_pub.publish(mission_msg)

    #         self.get_logger().info(f"🚀 Missão enviada para {robot_id}: {goal_x}, {goal_y}")

    #     except json.JSONDecodeError:
    #         self.get_logger().error("❌ Erro ao decodificar JSON na missão!")

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
            # print("📥 Dados recebidos no robot_status_callback:", json.dumps(msg.data, indent=4))  
            # print("🔍 Stack trace da chamada:")
            # traceback.print_stack()  # Isso imprime de onde essa função foi chamada!

            robot_id = data['robot_id']
            position = tuple(data['position'])
            position_name = data['area']
            estimated_time = data.get('estimated_time', None)
            planned_path = data.get('planned_path', [])
            if robot_id in self.robots:
                
                self.robots[robot_id]['position'] = position
                 # Garante que a chave 'area' existe antes de comparar
                if 'area' in self.robots[robot_id]:
                    if self.robots[robot_id]['area'] != position_name:
                        send_notification(f"📡 {str(robot_id)} está em {str(position_name)}")
                self.robots[robot_id]['area'] = position_name
                self.robots[robot_id]['estimated_time'] = estimated_time
                self.robots[robot_id]['planned_path'] = planned_path

                delivery = self.robots[robot_id].get('current_delivery')
                if delivery is not None:
                    goal = self.robots[robot_id]['goal']
                    stage = delivery['stage']

                    if goal and tuple(position) == tuple(goal):
                        self.get_logger().info(f"📦 {robot_id} chegou ao objetivo {goal}. Verificando estágio: {stage}")
                        send_notification(f"📦 {str(robot_id) if robot_id else 'Desconhecido'} chegou ao objetivo {str(goal) if goal else 'indefinido'}")

                        tractor_id = delivery.get("tractor_id")
                        if stage == "pickup":
                            items = delivery['items']
                            self.robots[robot_id]['kit'].extend(items)
                            for item in items:
                                PIECES_STATUS[(item, data["tractor_id"])] = "em trânsito"

                                # 🔹 Atualiza a missão no histórico
                                if item in MISSION_HISTORY["active"]:
                                    MISSION_HISTORY["active"][item]["status"] = "delivery"
                            self.get_logger().info(
                                f"🔹 {robot_id} fez pickup das peças {items} em {delivery['pickup_area']}"
                            )
                            send_notification(f" {str(robot_id) if robot_id else 'Desconhecido'} pegou a(s) peça(s) {str(items) if items else 'indefinido'}" \
                                              f"em {str(delivery['pickup_area']) if delivery['pickup_area'] else 'lugar indefinido'} ")

                            delivery['stage'] = "delivery"
                            new_goal = delivery['delivery_coords']
                            self.robots[robot_id]['goal'] = new_goal
                            mission_msg = String()
                            mission_msg.data = json.dumps({
                                "robot_id": robot_id,
                                "goal": [new_goal[0], new_goal[1]],
                                "tractor_id": delivery["tractor_id"]
                            })
                            self.mission_pub.publish(mission_msg)
                            self.get_logger().info(
                                f"🚀 Próxima etapa: entregar peças em {delivery['delivery_area']}"
                            )
                            
                        elif stage == "delivery":
                            self.get_logger().info(f"🔸 {robot_id} realizando entrega das peças {delivery['items']} em {delivery['delivery_area']}")
                            items = delivery['items']
                            for item in items:
                                if item in self.robots[robot_id]['kit']:
                                    self.robots[robot_id]['kit'].remove(item)
                                    # Atualiza o status da peça para a célula de entrega
                                    PIECES_STATUS[(item, delivery["tractor_id"])] = delivery["delivery_area"]

                                    # 🔹 Marca missão como concluída
                                    if item in MISSION_HISTORY["active"]:
                                        del MISSION_HISTORY["active"][item]
                                    MISSION_HISTORY["completed"].add(item)
                            self.get_logger().info(
                                f"🔸 {robot_id} entregou as peças {items} em {delivery['delivery_area']}"
                            )
                            send_notification(f"{str(robot_id) if robot_id else 'Desconhecido'} entregou a(s) peça(s) {str(items) if items else 'indefinido'} em" \
                                               f" {str(delivery['delivery_area']) if delivery['delivery_area'] else 'lugar indefinido'} ")
                            # Agora, para cada trator, verifica se este delivery satisfaz alguma etapa:
                            for tractor_id, production in TRACTORS.items():
                                for step in production['steps']:
                                    if step['cell'] == delivery['delivery_area']:
                                        # Adiciona as peças entregues no dicionário 'delivered'
                                        delivered_set = set(production['delivered'].get(step['cell'], []))  # <- Converte para set
                                        delivered_set.update(items)  # <- Agora pode usar `update()`
                                        production['delivered'][step['cell']] = list(delivered_set)  # <- Converte de volta para lista
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
                            send_notification(f"Entrega do robo {str(robot_id) if robot_id else 'Desconhecido'} finalizada")
                    else:
                        self.get_logger().info(f"📦 {robot_id} não chegou ao objetivo {goal}. Verificando estágio: {stage}")

                self.get_logger().info(f"📡 {robot_id} atualizado para {position}")
                # send_notification(f"📡 {str(robot_id) if robot_id else 'Desconhecido'} atualizado para {str(position) if position else 'posição indefinida'}")

                # registrar_estado()

                atualizar_status_celulas()  # Atualiza o status das células

        except json.JSONDecodeError:
            self.get_logger().error("❌ Erro ao processar status do robô!")


    def publish_missions(self):
        """Envia missões ativas para os robôs, apenas se ainda não chegaram ao destino"""
        
        for robot_id, info in self.robots.items():
            # print("PUBLISH MISSIONSSSSSSS ------------- ")
            # print(robot_id, info)
            if info['goal'] is None:
                continue  # Se não há um objetivo, não envia missão
            
            # Verifica se há uma missão ativa (pickup ou delivery)
            delivery = info.get('current_delivery')

            # Se o robô chegou ao destino, verifique o que deve ser feito
            if info['position'] == info['goal'] and delivery:
                stage = delivery["stage"]

                if stage == "pickup":
                    # O robô precisa coletar os itens
                    items = delivery["items"]
                    info["kit"].extend(items)  # Adiciona os itens ao kit
                    for item in items:
                        PIECES_STATUS[(item, delivery["tractor_id"])] = delivery["delivery_area"]
                    self.get_logger().info(f"📦 {robot_id} fez pickup das peças {items} em {delivery['pickup_area']}")

                    # Atualiza para a próxima etapa (entrega)
                    delivery["stage"] = "delivery"
                    new_goal = delivery["delivery_coords"]
                    info["goal"] = new_goal  # Define o novo objetivo
                    self.get_logger().info(f"🚀 Próxima missão: entregar as peças em {delivery['delivery_area']}")

                elif stage == "delivery":
                    # O robô precisa entregar os itens
                    items = delivery["items"]
                    for item in items:
                        if item in info["kit"]:
                            info["kit"].remove(item)  # Remove do kit
                            PIECES_STATUS[(item, delivery["tractor_id"])] = delivery["delivery_area"]

                    self.get_logger().info(f"🔸 {robot_id} entregou as peças {items} em {delivery['delivery_area']}")
                    
                    # Marca a entrega como concluída
                    info["current_delivery"] = None
                    info["goal"] = None  # Remove o objetivo
                    self.get_logger().info(f"✅ Entrega finalizada para {robot_id}")

                    continue  # Não publica uma nova missão

            # Se a missão ainda está ativa, publique a missão
            mission_msg = String()
            mission_msg.data = json.dumps({"robot_id": robot_id, "goal": info["goal"], "tractor_id": delivery["tractor_id"]})
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
                "goal": [pickup_coords[0], pickup_coords[1]],
                
                "tractor_id": data.get("tractor_id", None)
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
            print(f"📡 Mensagem ROS2 recebida: {data}")  # ✅ Log para ver o formato dos dados recebidos

            tractor_id = data["tractor_id"]

            if tractor_id in TRACTORS:
                self.get_logger().error(f"🚨 Trator {tractor_id} já existe!")
                return

            TRACTORS[tractor_id] = {
                "steps": data["steps"],
                "completed_steps": [],
                "delivered": {step["cell"]: set() for step in data["steps"]}
            }

            for step in data["steps"]:
                for piece in step["kits"]:
                    print(f"📦 Adicionando peça: {piece}, Trator: {tractor_id}")  # ✅ Log de depuração
                    if (piece, tractor_id) not in PIECES_STATUS:
                        PIECES_STATUS[(piece, tractor_id)] = f"Armazenamento 3-{random.randint(1,6)}"

            # print("📌 PIECES_STATUS atualizado:", PIECES_STATUS)  # ✅ Log final para verificar se está correto

            self.get_logger().info(f"✅ Trator {tractor_id} criado com sucesso!")

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
    # Não chama registrar_estado() aqui, para não travar.
    # Retorna apenas o estado atual (sem persistir).
    return {
        "robots": mission_manager.robots,
        "walls": list(WALLS),
        "areas": [{"x1": x1, "y1": y1, "x2": x2, "y2": y2, "name": name} for x1, y1, x2, y2, name in CELLS],
        "doors": list(DOORS)
    }

def background_save_loop():
    while True:
        registrar_estado()   # Salva no histórico
        time.sleep(5) 

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
        "tractor_id": data.tractor_id  # Evita erro se não estiver presente
    }

    robot_info['goal'] = pickup_coords

    mission_msg = String()
    mission_msg.data = json.dumps({
        "robot_id": data.robot_id,
        "goal": [pickup_coords[0], pickup_coords[1]],
        "tractor_id": data.get("tractor_id", None)
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
    # 2) Encontrar robô livre mais próximo do pickup
    best_robot_id, best_distance = min(
        ((r_id, abs(r_info['position'][0] - pickup_coords[0]) + abs(r_info['position'][1] - pickup_coords[1]))
        for r_id, r_info in mission_manager.robots.items() if r_info.get("current_delivery") is None),
        key=lambda x: x[1],
        default=(None, float('inf'))
    )

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
        "delivery_coords": delivery_coords,
        "tractor_id": data.tractor_id  # Adicionando tractor_id
    }
    print(f"ROBOT_INFO {robot_info}")
    # Ajusta o goal para o pickup
    robot_info['goal'] = pickup_coords

    # Publicar no tópico para o robô
    mission_msg = String()
    mission_msg.data = json.dumps({
        "robot_id": best_robot_id,
        "goal": [pickup_coords[0], pickup_coords[1]],
        "tractor_id": data.tractor_id  # ✅ Envia `tractor_id` na mensagem da missão
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

    TRACTORS[data.tractor_id] = {
        "steps": [step.dict() for step in data.steps],
        "completed_steps": [],
        "delivered": {step.cell: set() for step in data.steps}
    }

    for step in data.steps:
        for piece in step.kits:
            print(f"📦 Adicionando peça: {piece}, Trator: {data.tractor_id}")  # ✅ Log de depuração
            if (piece, data.tractor_id) not in PIECES_STATUS:
                PIECES_STATUS[(piece, data.tractor_id)] = f"Armazenamento 3-{random.randint(1,6)}"

    print("📌 PIECES_STATUS atualizado:", PIECES_STATUS)  # ✅ Verificar resultado
    send_notification(f"Trator {data.tractor_id} criado com sucesso!")
    return {"message": f"Trator {data.tractor_id} criado com sucesso!"}



@app.post("/trigger_tractor_missions")
def trigger_tractor_missions(data: dict):
    tractor_id = data.get("tractor_id")
    if not tractor_id or tractor_id not in TRACTORS:
        return {"error": "Trator não encontrado"}

    production = TRACTORS[tractor_id]
    missions_triggered = []

    print(f"🚜 Iniciando missões para trator {tractor_id}...")

    for step in production["steps"]:
        cell = step["cell"]
        for piece in step["kits"]:
            # 🚨 Verifica se a peça já foi entregue ou está ativa
            if (piece, tractor_id) in MISSION_HISTORY["completed"]:
                print(f"✅ Peça {piece} já foi entregue para trator {tractor_id}, ignorando...")
                continue
            if (piece, tractor_id) in MISSION_HISTORY["active"]:
                print(f"⏳ Peça {piece} já está em missão ativa para trator {tractor_id}, ignorando...")
                continue

            # 📍 Verifica a localização da peça
            if (piece, tractor_id) not in PIECES_STATUS:
                print(f"⚠️ Peça {piece} não encontrada em PIECES_STATUS! Criando localização inicial...")
                # Define um local padrão para início, se necessário
                PIECES_STATUS[(piece, tractor_id)] = f"Armazenamento 3-{random.randint(1,6)}"

            current_location = PIECES_STATUS.get((piece, tractor_id))
            pickup_coords = AREA_MAP.get(current_location)
            if not pickup_coords:
                print(f"⚠️ Localização da peça {piece} ainda desconhecida ({current_location}), pulando...")
                continue

            # 🚜 Procura um robô livre
            best_robot_id = None
            best_distance = float("inf")
            for r_id, r_info in mission_manager.robots.items():
                if r_info.get("current_delivery") is not None:
                    continue  # Robô ocupado

                (rx, ry) = r_info['position']
                (px, py) = pickup_coords
                dist = abs(rx - px) + abs(ry - py)

                if dist < best_distance:
                    best_distance = dist
                    best_robot_id = r_id

            if not best_robot_id:
                print(f"🚫 Nenhum robô disponível para transportar {piece}")
                continue  # Nenhum robô disponível

            # 📦 Verifica se a área de entrega existe
            delivery_coords = AREA_MAP.get(cell)
            if not delivery_coords:
                print(f"⚠️ Local de entrega {cell} não encontrado, ignorando missão.")
                continue

            # ✅ Criar os dados da missão
            mission_data = {
                "pickup_area": current_location,
                "delivery_area": cell,
                "items": [piece],
                "tractor_id": tractor_id
            }
            print(f"📦 Criando missão: {mission_data}")

            # 🚀 Dispara a missão
            try:
                res = requests.post("http://127.0.0.1:8000/request_delivery_mission", json=mission_data)
                res.raise_for_status()
                print(f"✅ Missão enviada com sucesso para {piece}")

                # 🔄 Atualiza histórico de missões
                MISSION_HISTORY["active"][(piece, tractor_id)] = {
                    "robot": best_robot_id,
                    "status": "pickup"
                }
                missions_triggered.append({
                    "robot": best_robot_id,
                    "piece": piece,
                    "from": current_location,
                    "to": cell
                })
            except Exception as e:
                print(f"❌ Erro ao disparar missão: {e}")

    if not missions_triggered:
        print("⚠️ Nenhuma missão foi acionada.")

    return {"message": "Missões acionadas", "missions": missions_triggered}


@app.get("/pieces_status")
def get_pieces_status():
    formatted_status = {}

    for key, location in PIECES_STATUS.items():
        if isinstance(key, str) and key.startswith("(") and key.endswith(")"):
            try:
                piece, tractor = ast.literal_eval(key)  # Converte string para tupla
                formatted_status[f"{piece},{tractor}"] = location
            except (ValueError, SyntaxError):
                print(f"⚠️ Falha ao converter chave: {key}")
                formatted_status[key] = location  # Mantém a chave original se der erro
        elif isinstance(key, tuple) and len(key) == 2:
            formatted_status[f"{key[0]},{key[1]}"] = location  # Formatação correta
        else:
            print(f"🚨 Chave inesperada: {key}")
            formatted_status[key] = location  # Mantém caso tenha outro formato inválido

    # print("📌 PIECES_STATUS formatado para retorno:", formatted_status)  # ✅ Depuração
    return formatted_status



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
    send_notification(f"Trator {tractor_id} atualizado com sucesso!")

    return {"message": f"Trator {tractor_id} atualizado com sucesso!", "tractor": current_production}

@app.delete("/tractor_production/{tractor_id}")
def delete_tractor_production(tractor_id: str):
    if tractor_id not in TRACTORS:
        return {"error": "Trator não encontrado"}
    del TRACTORS[tractor_id]
    return {"message": f"Trator {tractor_id} removido com sucesso!"}


mir = MirRobot()

# Variáveis globais (apenas como exemplo)
mir_nome = None
mir_mission = None
mir_battery = None
mir_state = None
mir_mode = None
mir_path = None

mir_x = 0.0
mir_y = 0.0
mir_orientation = 0.0


@app.get("/mir_status")
async def get_mir_status():
    """
    Endpoint para consultar e atualizar o status do MiR.
    """
    global mir_nome, mir_mission, mir_battery, mir_state, mir_mode, mir_path
    global mir_x, mir_y, mir_orientation
    # Enviar notificação para o frontend
    notification_msg = f"TESTE"
    # send_notification(notification_msg)
    try:
        status = await mir.get_status()  # Obtém dados atuais do robô

        # Exemplo de como extrair cada campo de 'status' retornado pelo robô:
        # Você deve ajustar estes nomes de campos conforme a resposta real de 'mir.get_status()'
        mir_nome = status.get("robot_name", "MIR")
        mir_mission = status.get("mission_text", "Sem missão")
        mir_battery = status.get("battery_percentage", 0)
        mir_state = status.get("state_text", "Desconhecido")
        mir_mode = status.get("mode_text", "Desconhecido")
        
        # Se a resposta contiver algo como status["position"]["x"], ...
        position = status.get("position", {})
        mir_x = position.get("x", 0.0)
        mir_y = position.get("y", 0.0)
        mir_orientation = position.get("orientation", 0.0)
        mir_path = status.get("path","Desconhecido")

        return status  # Retorna o próprio status obtido (ou retorne outra estrutura que desejar)
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


def get_mir_data():
    """
    Retorna um dicionário com todos os dados relevantes do MiR,
    usando as variáveis atualizadas no /mir_status.
    """
    return {
        "robot_name": mir_nome,
        "mission_text": mir_mission,
        "battery_percentage": mir_battery,
        "state_text": mir_state,
        "mode_text": mir_mode,
        "position": {
            "x": mir_x,
            "y": mir_y,
            "orientation": mir_orientation
        },
        "path" : mir_path
    }


@app.get("/historico")
def get_historico():
    """
    Retorna todos os frames registrados do histórico (timeline).
    """
    return {"historico": historico_estados}

@app.get("/cells_status")
def get_cells_status():
    atualizar_status_celulas()  # 🔄 Garante que os dados estão sempre atualizados antes de enviar
    return CELLS_STATUS

def atualizar_status_celulas():
    global CELLS_STATUS

    # Resetar os status para "noPiece"
    for cell_name in CELLS_STATUS.keys():
        CELLS_STATUS[cell_name] = "noPiece"

    # Criar dicionários para contar peças necessárias e entregues por célula (com quantidades)
    pecas_necessarias_por_celula = {}
    pecas_entregues_por_celula = {}

    # Primeiro, coletamos todas as peças necessárias e entregues por célula
    for tractor_id, production in TRACTORS.items():
        for step in production["steps"]:
            cell = step["cell"]

            if cell not in pecas_necessarias_por_celula:
                pecas_necessarias_por_celula[cell] = {}

            if cell not in pecas_entregues_por_celula:
                pecas_entregues_por_celula[cell] = {}

            # Adicionar a contagem de peças necessárias para esta célula
            for peca in step["kits"]:
                pecas_necessarias_por_celula[cell][peca] = pecas_necessarias_por_celula[cell].get(peca, 0) + 1

            # Adicionar a contagem de peças entregues para esta célula
            for peca in production["delivered"].get(cell, []):
                pecas_entregues_por_celula[cell][peca] = pecas_entregues_por_celula[cell].get(peca, 0) + 1
    # print(pecas_necessarias_por_celula)
    # print(pecas_entregues_por_celula)
    # Agora, verificamos o status de cada célula
    for cell in CELLS_STATUS.keys():
        if cell in pecas_necessarias_por_celula:
            pecas_necessarias = pecas_necessarias_por_celula[cell]
            pecas_entregues = pecas_entregues_por_celula.get(cell, {})

            # Verificar se todas as peças necessárias foram entregues na quantidade correta
            completa = all(
                pecas_entregues.get(peca, 0) >= quantidade
                for peca, quantidade in pecas_necessarias.items()
            )

            if completa:
                CELLS_STATUS[cell] = "complete"
            else:
                CELLS_STATUS[cell] = "pending"
        else:
            CELLS_STATUS[cell] = "noPiece"  # Se não há peças necessárias, mantém "noPiece"

    # print(CELLS_STATUS)  # Para depuração



def calcular_tempo_por_estado():
    """
    Calcula a porcentagem de células, robôs e tratores em cada estado ao longo do tempo,
    garantindo que a soma dos estados sempre seja 100% apenas para os objetos presentes.
    """
    if not historico_estados:
        return {"message": "Nenhum dado de histórico disponível."}

    historico_distribuicao = {
        "celulas": [],
        "robos": [],
        "tratores": [],
        "timestamps": []
    }

    for i in range(len(historico_estados)):
        atual = historico_estados[i]
        timestamp = atual["timestamp"]

        # Contadores de estados
        estado_celulas = {}
        estado_robos = {}
        estado_tratores = {}

        # Contagem de células por estado
        total_celulas = sum(1 for _ in atual["cells_status"])
        for status in atual["cells_status"].values():
            estado_celulas[status] = estado_celulas.get(status, 0) + 1

        # Contagem de robôs por estado
        total_robos = sum(1 for _ in atual["robots"])
        for info in atual["robots"].values():
            if info.get("current_delivery"):  # Se estiver em uma missão
                stage = info["current_delivery"]["stage"]
                status = "coletando" if stage == "pickup" else "entregando" if stage == "delivery" else "movendo"
            else:
                status = "parado"
            estado_robos[status] = estado_robos.get(status, 0) + 1

        # Contagem de tratores por estado
        total_tratores = sum(1 for _ in atual["tractors"])
        for info in atual["tractors"].values():
            all_completed = set(step["cell"] for step in info.get("steps", [])) == set(info.get("completed_steps", []))
            # print(info.get("steps", []) ,info.get("completed_steps", []) )
            # print(all_completed)
            status = "produzindo" if all_completed else "esperando"
            estado_tratores[status] = estado_tratores.get(status, 0) + 1

        # ✅ Função para calcular percentual corretamente
        def calcular_percentual(estado_dict, total):
            if total == 0:
                return {}  # Retorna vazio se não houver objetos no momento

            # Filtra apenas estados que possuem valor maior que zero
            estado_dict = {estado: qtd for estado, qtd in estado_dict.items() if qtd > 0}

            if len(estado_dict) == 1:
                # Se só há um estado, ele deve ser 100%
                estado_unico = list(estado_dict.keys())[0]
                return {estado_unico: 100}

            # Normaliza os valores para garantir que a soma seja 100%
            total_valid = sum(estado_dict.values())
            return {estado: round((quantidade / total_valid) * 100, 2) for estado, quantidade in estado_dict.items()}

        # Armazena os dados processados
        historico_distribuicao["celulas"].append(calcular_percentual(estado_celulas, total_celulas))
        historico_distribuicao["robos"].append(calcular_percentual(estado_robos, total_robos))
        historico_distribuicao["tratores"].append(calcular_percentual(estado_tratores, total_tratores))
        historico_distribuicao["timestamps"].append(timestamp)

    return historico_distribuicao




@app.get("/historico_estados")
def historico_estados_api():
    carregar_historico()
    data = calcular_tempo_por_estado()
    if(data):
        return JSONResponse(content=data)
    return {"error": "Nenhum histórico disponível"}


active_connections: List[WebSocket] = []
@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    active_connections.append(websocket)
    try:
        while True:
            await websocket.receive_text()
    except WebSocketDisconnect:
        active_connections.remove(websocket)


async def send_notification_async(message: str):
    """Função assíncrona que envia notificações para todos os clientes conectados."""
    print(f"📢 Enviando notificação: {message}")
    for connection in active_connections:
        try:
            await connection.send_text(message)
        except:
            active_connections.remove(connection)

def send_notification(message: str):
    """Função segura para ser chamada de qualquer lugar (sync ou async)."""
    try:
        loop = asyncio.get_running_loop()
        if loop.is_running():
            asyncio.create_task(send_notification_async(message))
        else:
            raise RuntimeError  # Se não há loop rodando, força a exceção para criar um novo
    except RuntimeError:
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        loop.run_until_complete(send_notification_async(message))
        loop.close()


def main():
    rclpy.init()

    global mission_manager
    mission_manager = MissionManager()  # <--- Instancia aqui
    # Chame essa função antes de carregar o histórico
    corrigir_json_corrompido("historico.json")
    carregar_historico()
    backup = 0
    # 2. Se houver algum histórico carregado, restaurar o último estado
    if len(historico_estados) > 0:
        ultimo_estado = historico_estados[-1]
        print(f"Ultimo estado {ultimo_estado}")
        # Exemplo: restaurando robôs e posições
        mission_manager.robots = ultimo_estado["robots"]
        
        # Restaurando tratores, peças, etc.
        global TRACTORS, PIECES_STATUS, CELLS_STATUS
        TRACTORS = ultimo_estado["tractors"]
        # Converter chaves stringificadas para tuplas em PIECES_STATUS
        PIECES_STATUS = {}
        for key, location in ultimo_estado["pieces_status"].items():
            if isinstance(key, str) and key.startswith("(") and key.endswith(")"):
                try:
                    piece, tractor = ast.literal_eval(key)  # Converte string para tupla
                    PIECES_STATUS[(piece, tractor)] = location
                except (ValueError, SyntaxError):
                    print(f"⚠️ Erro ao converter chave: {key}")
                    PIECES_STATUS[key] = location  # Mantém a chave original se falhar
            else:
                PIECES_STATUS[key] = location  # Mantém chaves já corretas

        print(f"🔄 PIECES_STATUS restaurado: {PIECES_STATUS}")  # Depuração
        CELLS_STATUS = ultimo_estado["cells_status"]    

            # 🔄 Restaurar robôs apenas se houver dados
        if "robots" in ultimo_estado and ultimo_estado["robots"]:
            mission_manager.robots = {}  # Limpa os robôs antigos antes de recriar
            print(f"🔄 Restaurando robôs do histórico: {ultimo_estado.get('robots', 'Nenhum histórico encontrado')}")

            # 🚀 Criar novamente as instâncias dos robôs
            for robot_id, robot_info in ultimo_estado["robots"].items():
                x, y = robot_info["position"]

                # 🔹 Criar novamente o robô como subprocesso
                mission_manager.add_robot(robot_id, x, y)

                # 🔹 Restaurar os atributos do robô
                # 🔹 Restaurar o objetivo do robô corretamente
                if robot_info.get("goal"):
                    mission_manager.robots[robot_id]["goal"] = tuple(robot_info["goal"])
                elif robot_info.get("current_delivery"):
                    # Se o robô estava em entrega, garantir que ele vá para o destino correto
                    stage = robot_info["current_delivery"]["stage"]
                    if stage == "pickup":
                        mission_manager.robots[robot_id]["goal"] = tuple(robot_info["current_delivery"]["pickup_coords"])
                    elif stage == "delivery":
                        mission_manager.robots[robot_id]["goal"] = tuple(robot_info["current_delivery"]["delivery_coords"])

                mission_manager.robots[robot_id]["trajectory"] = robot_info.get("trajectory", [])
                mission_manager.robots[robot_id]["kit"] = robot_info.get("kit", [])
                mission_manager.robots[robot_id]["current_delivery"] = robot_info.get("current_delivery", None)

                print(f"✅ Robô {robot_id} restaurado na posição {x}, {y} com objetivo {mission_manager.robots[robot_id]['goal']}")
                print(robot_info)
                # 🚀 Reenviar a missão se o robô tiver uma entrega ativa
                if mission_manager.robots[robot_id]["current_delivery"]:
                    mission_msg = String()
                    mission_msg.data = json.dumps({
                        "robot_id": robot_id,
                        "goal": mission_manager.robots[robot_id]["goal"],
                        "tractor_id": mission_manager.robots[robot_id]["current_delivery"]["tractor_id"]
                    })
                    mission_manager.mission_pub.publish(mission_msg)
                    print(f"🚀 Missão reenviada para {robot_id}: {mission_manager.robots[robot_id]['goal']}")


        if "tractors" in ultimo_estado and ultimo_estado["tractors"]:
            TRACTORS = {}  # Limpa tratores antigos antes de recriar
            print(f"🔄 Restaurando tratores do histórico...")

            for tractor_id, tractor_info in ultimo_estado["tractors"].items():
                TRACTORS[tractor_id] = {
                    "steps": tractor_info.get("steps", []),
                    "completed_steps": tractor_info.get("completed_steps", []),
                    "delivered": tractor_info.get("delivered", {})
                }
                print(f"✅ Trator {tractor_id} restaurado com {len(TRACTORS[tractor_id]['steps'])} etapas pendentes.")

        backup = 1
    threading.Thread(target=rclpy.spin, args=(mission_manager,), daemon=True).start()
    threading.Thread(target=background_save_loop ,daemon=True).start()
    threading.Thread(target=carregar_historico, daemon=True).start()

    uvicorn.run(app, host="0.0.0.0", port=8000, log_level="info")
    if(backup):
        send_notification(f"Backup restaurado")

    rclpy.shutdown()

if __name__ == "__main__":
    main()
