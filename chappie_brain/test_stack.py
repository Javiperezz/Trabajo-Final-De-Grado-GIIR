import httpx
from qdrant_client import QdrantClient

print("Probando ollama")
r = httpx.post("http://localhost:11434/api/generate",
    json={"model": "qwen2.5:7b", "prompt": "Di hola en una palabra", "stream": False},
    timeout=60)
print("  ->", r.json()["response"][:60])

print("Probando ollama embeddings")
r = httpx.post("http://localhost:11434/api/embed",
    json={"model": "bge-m3", "input": "prueba"}, timeout=30)
print("  -> embedding dim:", len(r.json()["embeddings"][0]))

print("Probando Qdrant")
client = QdrantClient("localhost", port=6333)
print("  -> collections:", client.get_collections())

print("\Todo correcto")
