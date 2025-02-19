from flask import Flask, render_template, jsonify
import requests

app = Flask(__name__)
API_URL = "http://127.0.0.1:8000"

@app.route("/")
def index():
    return render_template("index.html")  # Arquivo HTML da interface

@app.route("/get_map")
def get_map():
    """Obtém o status dos robôs e paredes do Mission Manager"""
    try:
        res = requests.get(f"{API_URL}/status")
        return jsonify(res.json())
    except Exception as e:
        return jsonify({"error": f"❌ Erro ao obter mapa: {str(e)}"})

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000, debug=True)
