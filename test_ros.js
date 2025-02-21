const ROSLIB = require('roslib'); // Importa a biblioteca ROSLIB

// Conectar ao rosbridge WebSocket
let ros = new ROSLIB.Ros({
  url: "ws://localhost:9090" // Alterar para o IP correto se necessário
});

// Mensagens de status da conexão
ros.on('connection', function() {
  console.log("✅ Conectado ao rosbridge!");
});

ros.on('error', function(error) {
  console.error("❌ Erro na conexão:", error);
});

ros.on('close', function() {
  console.log("⚠️ Conexão com rosbridge fechada.");
});

// Função para listar tópicos disponíveis
function listarTopicos() {
  ros.getTopics(function(result) {
    console.log("📡 Tópicos disponíveis:", result.topics);
  });
}

// Criando um listener para verificar o tópico específico
let webPathListener = new ROSLIB.Topic({
  ros: ros,
  name: "/mirwebapp/web_path",
  messageType: "mirMsgs/WebPath"
});

// Inscrever-se no tópico para receber mensagens
webPathListener.subscribe(function(message) {
  console.log("📌 Mensagem recebida em /mirwebapp/web_path:", message);
});

// Executar após 3 segundos para garantir conexão
setTimeout(listarTopicos, 3000);
