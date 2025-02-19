
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time
import sys

import heapq
import re



import heapq

def get_doors_of_cell(cell, doors):
    """Retorna o conjunto de portas que estão na 'borda' da célula."""
    if not cell:
        return set()  # Se a célula não existe, retorna vazio
    x1, y1, x2, y2, nome = cell
    cell_doors = set()
    for (dx, dy) in doors:
        # Checa se a porta está na borda horizontal
        if x1 <= dx <= x2 and (dy == y1 or dy == y2):
            cell_doors.add((dx, dy))
        # Ou na borda vertical
        elif y1 <= dy <= y2 and (dx == x1 or dx == x2):
            cell_doors.add((dx, dy))
    return cell_doors

def find_closest_door(start, goal, doors):
    """Exemplo simplificado de função para escolher a porta mais próxima do goal."""
    if not doors:
        return None
    return min(doors, key=lambda d: abs(d[0] - goal[0]) + abs(d[1] - goal[1]))

def find_cell_for_point(point, cells):
    """Retorna a célula (x1, y1, x2, y2, nome) que contém 'point', ou None."""
    
    px, py = point
    for (x1, y1, x2, y2, nome) in cells:
        # print(f"CELULA: {nome}")

        # Usamos <= e >= para considerar a área interna.
        # Caso as bordas estejam em walls, não há problema se o goal ficar bem na borda.
        if x1 <= px <= x2 and y1 <= py <= y2:
            return (x1, y1, x2, y2, nome)
    return None

def dijkstra(start, goal, walls, corridors, doors, cells, map_limits=(-100, 100, -100, 100)):
    """
    Executa o algoritmo de Dijkstra para encontrar o caminho mais curto
    evitando paredes e células que não sejam corredor, 
    a menos que seja porta da célula do goal ou o goal em si.
    Em seguida, identifica as portas no path.
    """
    import heapq

    x_min, x_max, y_min, y_max = map_limits
    open_list = []
    heapq.heappush(open_list, (0, start))  # (custo, posição_atual)
    came_from = {}
    g_score = {start: 0}

    print(f"🔍 Iniciando Dijkstra de {start} para {goal}")

    # Verificações iniciais
    if start == goal:
        print("✅ Ponto de início é o destino!")
        return [start]
    if start in walls:
        print("❌ ERRO: O ponto de início está dentro de uma parede!")
        return []
    if goal in walls:
        print("❌ ERRO: O destino está bloqueado por uma parede!")
        return []

# 1) A célula onde o robô está
    robot_cell = find_cell_for_point(start, cells)
    if robot_cell:
        robot_cell_doors = get_doors_of_cell(robot_cell, doors)
    else:
        robot_cell_doors = set()

    # 2) A célula do goal
    goal_cell = find_cell_for_point(goal, cells)
    if goal_cell:
        goal_cell_doors = get_doors_of_cell(goal_cell, doors)
    else:
        goal_cell_doors = set()

    # self.get_logger().info(f"📍 Portas da célula do robô: {robot_cell_doors}")
    # self.get_logger().info(f"🎯 Portas da célula do goal: {goal_cell_doors}")

    # 3) Portas liberadas = robô + goal
    allowed_doors = robot_cell_doors | goal_cell_doors
    blocked_doors = doors - allowed_doors
    walls_with_other_doors = walls | blocked_doors

    while open_list:
        cost, current = heapq.heappop(open_list)

        if current == goal:
            # Reconstrói o caminho
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            path.reverse()

            # "Pinta" as portas no caminho
            annotated_path = []
            for position in path:
                if position in doors:
                    annotated_path.append(f"\033[93m{position}\033[0m")
                else:
                    annotated_path.append(str(position))

            print(f"✅ Caminho encontrado: {annotated_path}")
            return path

        x, y = current
        # Movimentos ortogonais
        neighbors = [
            (x + 1, y),
            (x - 1, y),
            (x, y + 1),
            (x, y - 1)
        ]

        for neighbor in neighbors:
            nx, ny = neighbor

            # Checagens básicas
            if not (x_min <= nx <= x_max and y_min <= ny <= y_max):
                continue
            if neighbor in walls_with_other_doors:
                continue

            # Permitimos andar se:
            #  - está em corredor
            #  - ou está em QUALQUER porta da célula do goal
            #  - ou é o goal em si
            # if neighbor not in corridors and neighbor not in goal_cell_doors and neighbor != goal:
            #     continue

            temp_g_score = g_score[current] + 1
            if neighbor not in g_score or temp_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = temp_g_score
                heapq.heappush(open_list, (temp_g_score, neighbor))

    print("❌ Nenhum caminho encontrado!")
    return []




class Robot(Node):
    def __init__(self, name, x, y, walls, doors,corridors,cells):
        super().__init__(name)
        self.position = (x, y)
        self.goal = None    
        self.trajectory = []
        # Convertendo listas para conjuntos de tuplas
        # walls = set(map(tuple, walls))
        # doors = set(map(tuple, doors))
        # corridors = set(map(tuple, corridors))       
        self.doors = doors

        # print(f"DOORS: {self.doors}")

        # Removendo portas do conjunto de paredes
        self.walls = walls - doors
        self.corridors = corridors
        self.cells = cells
        
        # self.get_logger().info(f"🧱 Tipo das paredes: {type(walls)}, conteúdo: {walls}")
        # self.get_logger().info(f"Tipo das portas: {type(doors)}, conteúdo: {doors}")

        # self.get_logger().info(f"🧱 Paredes finais (sem portas): {self.walls}")
        # Assina as missões destinadas ao robô
        self.subscription = self.create_subscription(String, 'robot_mission', self.mission_callback, 10)
        self.publisher = self.create_publisher(String, 'robot_status', 10)

        # Timer para enviar status a cada segundo
        self.timer = self.create_timer(1.0, self.publish_status)
        self.ajustar_posicao_inicial()

        self.get_logger().info(f"🤖 Robô {name} criado na posição {self.position}")


    def ajustar_posicao_inicial(self):
        """Se o robô estiver encostado na parede, move para um espaço livre."""
        x, y = self.position
        neighbors = [(x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)]
        
        for n in neighbors:
            if n not in self.walls:
                self.position = n
                self.get_logger().info(f"🔄 Ajustando posição inicial para {n}")
                return

        self.get_logger().error("🚧 Não há posição livre ao redor!")
    def mission_callback(self, msg):
        """Recebe missão do ROS2 e define o objetivo"""
        try:
            data = json.loads(msg.data)

            # Somente processa a missão se for destinada a este robô
            if data["robot_id"] == self.get_name():
                self.goal = tuple(data['goal'])
                self.get_logger().info(f"🚀 Missão recebida: {self.goal}")

        except json.JSONDecodeError as e:
            self.get_logger().error(f"❌ Erro ao decodificar JSON: {e}")

    def move_towards_goal(self):
        """Move o robô para o objetivo seguindo o caminho planejado."""
        if self.goal:
            # Se já chegou, finaliza
            if self.position == self.goal:
                self.get_logger().info("🎯 Cheguei ao destino!")
                self.goal = None
                self.path = []
                
                self.trajectory = []
                return

            # Se path não existe ou acabou, chama Dijkstra
            if not hasattr(self, "path") or not self.path:
                self.path = dijkstra(self.position, self.goal, 
                                    self.walls, self.corridors, 
                                    self.doors, self.cells)

            # Se o Dijkstra falhou
            if not self.path:
                self.get_logger().warning("⚠️ Nenhum caminho encontrado! Tentando ajuste...")

                x, y = self.position
                gx, gy = self.goal

                # Se não existir, cria um set de posições já tentadas no fallback
                if not hasattr(self, "fallback_visited"):
                    self.fallback_visited = set()

                # 1) A célula onde o robô está
                robot_cell = find_cell_for_point(self.position, self.cells)
                if robot_cell:
                    robot_cell_doors = get_doors_of_cell(robot_cell, self.doors)
                else:
                    robot_cell_doors = set()

                # 2) A célula do goal
                goal_cell = find_cell_for_point(self.goal, self.cells)
                if goal_cell:
                    goal_cell_doors = get_doors_of_cell(goal_cell, self.doors)
                else:
                    goal_cell_doors = set()

                # 3) Portas liberadas = robô + goal
                allowed_doors = robot_cell_doors | goal_cell_doors
                blocked_doors = self.doors - allowed_doors
                all_blocked = self.walls | blocked_doors

                possible_moves = sorted([
                    (x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1),
                    (x + 1, y + 1), (x - 1, y - 1), (x + 1, y - 1), (x - 1, y + 1)
                ], key=lambda pos: abs(pos[0] - gx) + abs(pos[1] - gy))

                for move in possible_moves:
                    # Se já visitamos esse move no fallback, pula
                    if move in self.fallback_visited:
                        self.get_logger().info(f"🔁 Ignorando {move}: já visitado no fallback.")
                        continue

                    # 1) Não pode ser parede nem porta bloqueada
                    if move in all_blocked:
                        self.get_logger().info(f"🛑 Ignorando {move}: está em all_blocked.")
                        continue

                    # (Opcional) Se quiser restringir a corredores/portas do goal ou goal
                    # descomente a checagem abaixo:
                    #
                    # if (move not in self.corridors 
                    #     and move not in allowed_doors
                    #     and move != self.goal):
                    #     self.get_logger().info(
                    #         f"🛑 Ignorando {move}: não é corredor nem porta do goal, nem o goal."
                    #     )
                    #     continue

                    # Marca essa posição como visitada no fallback
                    self.fallback_visited.add(move)

                    # Move e tenta recalcular
                    self.position = move
                    self.get_logger().info(f"🔄 Movendo para {move} para tentar recalcular caminho...")
                    self.path = dijkstra(self.position, self.goal,
                                        self.walls, self.corridors, 
                                        self.doors, self.cells)

                    if self.position == self.goal:
                        self.get_logger().info("🎯 Cheguei ao destino!")
                        self.goal = None
                        self.path = []
                        return

                    # Sai do loop e da função após um movimento
                    return

            # Se existe um caminho, anda um passo
            if self.path:
                # Limpamos o fallback_visited caso o Dijkstra tenha tido sucesso
                if hasattr(self, "fallback_visited"):
                    self.fallback_visited.clear()

                next_position = self.path.pop(0)
                self.trajectory.append(next_position)
                self.position = next_position
                self.get_logger().info(f"🤖 Movendo-se para {self.position}")

                # Exemplo de print colorido do caminho restante
                colored_path = []
                for pos in self.path:
                    if pos in self.doors:
                        colored_path.append(f"\033[93m{pos}\033[0m")
                    else:
                        colored_path.append(str(pos))
                print(f"✅ Caminho encontrado: {colored_path}")


                


            


    def publish_status(self):
        """Envia a posição do robô"""
        self.move_towards_goal()
        estimated_time = len(self.path) if hasattr(self, "path") else None

        # Identifica a célula ou se está no corredor
        cell_info = find_cell_for_point(self.position, self.cells)
        area_name = cell_info[4] if cell_info else "Corredor"

        status_msg = String()
        status_msg.data = json.dumps({
            'robot_id': self.get_name(),
            'position': self.position,
            'area': area_name,
            'estimated_time': estimated_time,
            'trajectory': self.trajectory,  # Caminho já percorrido
            'planned_path': self.path if hasattr(self, "path") else []  # Caminho planejado
        })
        self.publisher.publish(status_msg)

        self.get_logger().info(f"📡 Status enviado: {status_msg.data}")

def main():
    """Inicia o robô com os argumentos passados pelo subprocesso"""
    if len(sys.argv) < 7:
        print("❌ Erro: argumentos insuficientes para iniciar o robô!")
        return

    robot_id = sys.argv[1]
    x = int(sys.argv[2])
    y = int(sys.argv[3])

    # Ler os arquivos temporários
    with open(sys.argv[4], 'r') as f:
        walls = json.load(f)

    with open(sys.argv[5], 'r') as f:
        doors = json.load(f)

    with open(sys.argv[6], 'r') as f:
        corridors = json.load(f)

    with open(sys.argv[7], 'r') as f:
        raw_cells = json.load(f)  # lista de listas (cada uma com 5 elementos)
    # Expressão regular para encontrar pares de números entre colchetes
    pattern = re.findall(r'\[(-?\d+),\s*(-?\d+)\]', doors)

    # Convertendo os pares encontrados para tuplas de inteiros
    doors_tuples = set((int(x), int(y)) for x, y in pattern)
    # Expressão regular para encontrar pares de números entre colchetes
    pattern_w = re.findall(r'\[(-?\d+),\s*(-?\d+)\]', walls)
    
    # Convertendo os pares encontrados para tuplas de inteiros
    walls_tuples = set((int(x), int(y)) for x, y in pattern_w)
    pattern_c = re.findall(r'\[(-?\d+),\s*(-?\d+)\]', corridors)
    
    # Convertendo os pares encontrados para tuplas de inteiros
    corridors_tuples = set((int(x), int(y)) for x, y in pattern_c)
    
    # print(f"CELULAS: {raw_cells}") 
    padrao = re.compile(
    r'\[\s*(-?\d+)\s*,\s*(-?\d+)\s*,\s*(-?\d+)\s*,\s*(-?\d+)\s*,\s*"([^"]+)"\s*\]'
    )

    matches = padrao.findall(raw_cells)  
    cells = []
    for (x1_str, y1_str, x2_str, y2_str, nome_str) in matches:
        x1 = int(x1_str)
        y1 = int(y1_str)
        x2 = int(x2_str)
        y2 = int(y2_str)
        cells.append( (x1, y1, x2, y2, nome_str) )

    print("CELLS:", cells)
    # print(f"DOORS -- ROBOT -> : {doors_tuples}")
    rclpy.init()
    robot = Robot(robot_id, x, y, walls_tuples, doors_tuples, corridors_tuples,cells)
    rclpy.spin(robot)
    rclpy.shutdown()
if __name__ == '__main__':
    main()
