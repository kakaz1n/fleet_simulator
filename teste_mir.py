import json 
from mir import MirRobot # supondo que a classe esteja no arquivo mir_robot.py

def main(): # Se a API exigir autenticação, forneça o token; caso contrário, deixe auth_token como None. 
    mir = MirRobot()
    try:
        # Teste de status: obtém e exibe o status atual do MIR
        status = mir.get_status()
        print("Status do MIR:")
        print(json.dumps(status, indent=4))
        
        # # Teste de movimento: envia um comando para mover o robô
        # print("Enviando comando de movimento para (50, 100)...")
        # response = mir.move_to(50, 100)
        # print("Resposta do comando move_to:")
        # print(json.dumps(response, indent=4))
        
        # # Teste de comando genérico (ajuste conforme os comandos disponíveis na API)
        # print("Enviando comando genérico 'start'...")
        # cmd_response = mir.send_robot_command("start")
        # print("Resposta do comando genérico:")
        # print(json.dumps(cmd_response, indent=4))
        
    except Exception as e:
        print("Erro durante o teste:", e)
if __name__ == '__main__': 
    main()