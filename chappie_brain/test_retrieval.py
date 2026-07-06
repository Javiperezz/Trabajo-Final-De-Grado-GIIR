import httpx
from qdrant_client import QdrantClient

def embed(text):
    r = httpx.post("http://localhost:11434/api/embed",
        json={"model": "bge-m3", "input": text}, timeout=30)
    return r.json()["embeddings"][0]

client = QdrantClient("localhost", port=6333)

queries = [
    "¿Cómo te llamas?",
    "¿De dónde eres?",
    "¿Cómo te apago?",
    "¿Quién te creó?",
]

for q in queries:
    print(f"\nQ: {q}")
    results = client.query_points(
        collection_name="chappie_knowledge",
        query=embed(q), limit=2,
    ).points
    for r in results:
        print(f"  [{r.score:.3f}] {r.payload['text']}")
