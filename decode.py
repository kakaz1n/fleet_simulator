import base64
import struct
import zlib
import json
# Caminho codificado recebido (Base64)
encoded_path = "TVBUSAAAAAD8lLRn35H+FwMAAABtYXA9AAAAZ2aPQWdmIEHbD8m/Z2aPQUOtH0GmFs6/Z2aPQR/0HkFyHdO/Z2aPQWU7HkE+JNi/w2SPQUGCHUEKK92/L12PQe/JHEHWMeK/Dk+PQUATHEGiOOe/xzqPQZ5eG0FuP+y/XCCPQUOtGkE6RvG/AACPQQAAGkHRU/u/AACPQQAAGkHf7S/AMzOOQTMzGUHf7S/A/uWNQU7zGEFFcTLAyJiNQWizGEGr9DTAkkuNQYJzGEEReDfAXf6MQQU0GEF3+znAhK+MQbn8F0HdfjzA016MQbfRF0FDAj/A5wyMQZayF0GphUHA9bmLQb+fF0EPCUTAZ2aLQZqZF0HbD0lAZ2ZsQZqZF0HbD0lAQ61rQZqZF0F1jEZAH/RqQZqZF0EPCURAZTtqQZqZF0GphUFAQYJpQeGcF0FDAj9A78loQQisF0HdfjxAQRNoQUzIF0F3+zlAnl5nQdjwF0EReDdAQ61mQa8lGEGr9DRAAABmQWdmGEHf7S9AlWVlQUymGEFFcTJAKstkQTLmGEGr9DRAvjBkQRgmGUEReDdAU5ZjQZVlGUF3+zlAofhiQeGcGUHdfjxAP1diQePHGUFDAj9AaLNhQQTnGUGphUFAhQ1hQdv5GUEPCURAZ2ZgQQAAGkHbD0lAQ61fQQAAGkF1jEbAH/ReQQAAGkEPCUTAZTteQQAAGkGphUHAQYJdQbn8GUFDAj/A78lcQZLtGUHdfjzAQRNcQU7RGUF3+znAnl5bQcGoGUEReDfAQ61aQetzGUGr9DTAAABaQTMzGUHf7S/A8YVZQdbnGEF5ai3AhQ1ZQfaXGEET5yrAJZdYQWdEGEGuYyjA0SJYQSntF0FI4CXAirBXQdGRF0HiXCPAuEBXQcoyF0F82SDAW9NWQRTQFkEWVh7Ac2hWQa1pFkGw0hvAAABWQQAAFkHkyxbAAABWQQAAFkEAAAAAirBTQQAAFkEAAAAAAABUQWZmFkEAAAAA"

# Decodificar Base64 para bytes
decoded_bytes = base64.b64decode(encoded_path)


print(f"📏 Tamanho total: {len(decoded_bytes)} bytes")
print(f"📝 Bytes iniciais: {decoded_bytes[:40]}")

# Tentar descomprimir com zlib
try:
    decompressed_data = zlib.decompress(decoded_bytes)
    print("✔️ Dados descomprimidos com sucesso! 📜 JSON possivelmente encontrado:")
    print(decompressed_data.decode())  # Tentar converter para string
    json_data = json.loads(decompressed_data.decode())
    print("📌 JSON carregado:", json.dumps(json_data, indent=2))
except Exception as e:
    print(f"❌ Erro ao descomprimir com zlib: {e}")

# Tentar descomprimir com gzip (outro formato possível)
try:
    import gzip
    decompressed_data = gzip.decompress(decoded_bytes)
    print("✔️ Dados descomprimidos com gzip! 📜 JSON possivelmente encontrado:")
    print(decompressed_data.decode())  # Tentar converter para string
    json_data = json.loads(decompressed_data.decode())
    print("📌 JSON carregado:", json.dumps(json_data, indent=2))
except Exception as e:
    print(f"❌ Erro ao descomprimir com gzip: {e}")