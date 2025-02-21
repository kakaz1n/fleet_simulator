import asyncio
import websockets
import json

async def ros_websocket():
    uri = "ws://10.83.131.155:9090"  # Substitua pelo IP do ROS
    try:
        async with websockets.connect(uri) as websocket:
            print("✅ Conectado ao ROS WebSocket!")

            # Assinar um tópico (exemplo: "/odom" para odometria)
            subscribe_msg = {
                "op": "subscribe",
                "topic": "/mirwebapp/web_path"
            }
            await websocket.send(json.dumps(subscribe_msg))

            while True:
                mensagem = await websocket.recv()
                print(f"📩 Mensagem recebida: {mensagem}")

    except Exception as e:
        print(f"🚨 Erro: {e}")

asyncio.run(ros_websocket())
 