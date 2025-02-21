# **Documentação do Gerenciador de Robôs e Tratores**
![Texto Alternativo](https://github.com/kakaz1n/fleet_simulator/blob/main/fleet_simulator.png)

## **1. Introdução**
O Gerenciador de Robôs e Tratores é um sistema para controle, monitoramento e integração de robôs móveis e tratores em um ambiente automatizado. Ele permite a gestão de missões, rastreamento de peças e integração com tecnologias como ROS2 e MIR.

## **2. Gerenciamento de Robôs**
### **2.1 Funcionalidades**
- Adição de robôs ao sistema.
- Simulação de caminhos usando A*.
- Controle e navegação dos robôs pelo mapa.
- Robôs se movimentam apenas por portas e entram somente nas células de destino.
- Visualização do status de todos os robôs:
  - Nome.
  - Posição (nome da área no mapa).
  - Objetivo.
  - Tempo estimado para chegada.
  - Peça carregada.
  - Missão em andamento (pickup/delivery e item transportado).

## **3. Missões e Entregas**
### **3.1 Tipos de Missões**
- Solicitação de missão de entrega com:
  - Robô selecionado manualmente.
  - Melhor robô selecionado automaticamente.
- Solicitação de missões relacionadas ao trator:
  - Aciona múltiplos robôs para coletar as peças necessárias.

## **4. Gerenciamento de Tratores**
### **4.1 Funcionalidades**
- CRUD de tratores e suas etapas.
- Definição das etapas necessárias para cada trator.
- Definição das peças necessárias em cada etapa.
- Acompanhamento das etapas pendentes e concluídas.

## **5. Simulação de Pegar e Deixar Peças**
- Simulação de coleta e entrega de peças pelos robôs.
- Atualização em tempo real da posição das peças.

## **6. Mapa e Integração com o MIR**
### **6.1 Funcionalidades**
- Exibição do MIR no mapa via API.
- Integração da API do MIR com o gerenciador.
- Visualização do status do MIR:
  - Missão.
  - Bateria.
  - Estado.
  - Modo.
  - Posição e orientação.

## **7. Monitoramento e Status**
### **7.1 Acompanhamento de Status**
- Status das peças (localização no mapa).
- Status das células:
  - Quantas estão completas.
  - Quantas estão pendentes.
  - Quantas estão ociosas.
- Quais etapas dos tratores estão pendentes e concluídas.

## **8. Integração com ROS2**
### **8.1 Funcionalidades**
- Possibilidade de executar comandos via ROS2.
- Facilitação da integração dos robôs atuais ao gerenciador.

### **8.2 Sobre o ROS2**
O ROS2 (Robot Operating System 2) é um framework que facilita o desenvolvimento de robôs autônomos. Ele fornece um conjunto de ferramentas para comunicação entre processos, permitindo que diferentes partes do sistema robótico se comuniquem de forma eficiente.

### **8.3 Principais Características do ROS2**
- **Arquitetura distribuída**: Diferentes nós podem rodar em máquinas separadas.
- **Comunicação baseada em tópicos**: Mensagens são enviadas/recebidas por tópicos.
- **Uso de serviços e ações**: Alternativas para comunicação síncrona e assíncrona.
- **Suporte a tempo real**: Melhor eficiência em aplicações robóticas críticas.

### **8.4 Comunicação entre os Programas**
A comunicação acontece via tópicos do ROS2, conforme mostrado abaixo:

#### **Criação de Robôs**
O `mission_manager` escuta o tópico `/add_robot` e cria um novo robô.

```bash
ros2 topic pub --once /add_robot std_msgs/msg/String '{"data": "{\"robot_id\": \"R1\", \"x\": 10, \"y\": 10}"}'
```

O `mission_manager` lê a mensagem e executa:

```python
self.create_subscription(String, '/add_robot', self.add_robot_callback, 10)
```

Isso chama `add_robot()`, que cria um subprocesso:

```python
process = subprocess.Popen(["python3", "robot.py", robot_id, str(x), str(y)])
```

#### **Atribuição de Missões**
O `mission_manager` recebe comandos no tópico `/add_mission` e envia as missões para os robôs.

```bash
ros2 topic pub --once /add_mission std_msgs/msg/String '{"data": "{\"robot_id\": \"R1\", \"goal\": [50, 30]}"}'
```

O `mission_manager` lê a missão e publica no tópico `/robot_mission`:

```python
mission_msg = String()
mission_msg.data = json.dumps({"robot_id": robot_id, "goal": [goal_x, goal_y]})
self.mission_pub.publish(mission_msg)
```

O `robot.py` escuta `/robot_mission` e inicia o movimento.

#### **Entrega de Peças**
O `mission_manager` gerencia as entregas com o tópico `/add_delivery`.

```bash
ros2 topic pub --once /add_delivery std_msgs/msg/String '{"data": "{\"robot_id\": \"R1\", \"pickup_area\": \"Armazenamento 1-1\", \"delivery_area\": \"Montagem 1-1\", \"items\": [\"Kit Chassi\"]}"}'
```

O `mission_manager` atualiza a missão e manda o robô buscar a peça.

#### **Status e Monitoramento**
Para visualizar os estados dos robôs e tratores, usamos:

```bash
ros2 topic echo /robot_status
ros2 topic echo /tractor_status
```

### **8.5 Uso do ROS2 no Sistema**
O sistema está implementando ROS2 para controlar robôs móveis e tratores em um ambiente de simulação. Ele utiliza a seguinte estrutura:

| Módulo            | Função                                        | Tópico ROS2 Usado                  |
|--------------------|--------------------------------|--------------------------------|
| mission_manager.py | Gerencia os robôs e suas missões | /add_robot, /add_mission, /add_delivery |
| robot.py          | Simula o movimento do robô e recebe comandos | /robot_mission, /robot_status |
| trator.py         | Gerencia a montagem dos tratores | /add_tractor, /tractor_status |
