simport httpx
from qdrant_client import QdrantClient

def embed(text):
    r = httpx.post("http://localhost:11434/api/embed",
        json={"model": "bge-m3", "input": text}, timeout=30)
    return r.json()["embeddings"][0]

client = QdrantClient("localhost", port=6333)

queries = [
    "¿Quién es el director del campus de Alcoy?",
    "¿Cuándo empieza el segundo semestre en 2026?",
    "¿Qué grado cursa Javier?",
    "¿Cuántos grados se imparten en la EPSA?",
    "¿Dónde está el campus de Alcoy?",
    "¿Quién es el tutor del TFG de Javier?",
]

for q in queries:
    print(f"\nQ: {q}")
    results = client.query_points(
        collection_name="chappie_knowledge",
        query=embed(q), limit=2,
    ).points
    for r in results:
        text = r.payload['text']
        print(f"  [{r.score:.3f}] {text[:120]}...")
