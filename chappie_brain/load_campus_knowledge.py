
import httpx
import uuid
import re
from qdrant_client import QdrantClient
from qdrant_client.models import PointStruct

OLLAMA_URL = "http://localhost:11434"
COLLECTION = "chappie_knowledge"
INPUT_FILE = "knowledge_campus.txt"

def embed(text: str) -> list[float]:
   
    for attempt in range(3):
        r = httpx.post(f"{OLLAMA_URL}/api/embed",
            json={"model": "bge-m3", "input": text}, timeout=60)
        data = r.json()
        if "embeddings" in data:
            return data["embeddings"][0]
        print(f"   Attempt {attempt+1} returned: {data}")
    return None

def chunk_text(text: str) -> list[str]:   #Con esto separamos los parrafos
    
    chunks = []
    for para in re.split(r"\n\s*\n", text):
        para = para.strip()
        if not para:
            continue
        if para.startswith("#") and "\n" not in para:
            continue
        lines = [line for line in para.split("\n")
                 if not line.strip().startswith("#")]
        cleaned = " ".join(line.strip() for line in lines if line.strip())
        if cleaned and len(cleaned) > 20:  
            chunks.append(cleaned)
    return chunks

with open(INPUT_FILE, encoding="utf-8", errors="replace") as f:
    raw = f.read()

chunks = chunk_text(raw)
print(f"Extracted {len(chunks)} chunks from {INPUT_FILE}")

client = QdrantClient("localhost", port=6333)

if not client.collection_exists(COLLECTION):
    print(f"ERROR: Collection '{COLLECTION}' does not exist.")
    print("Run load_knowledge.py first to create it.")
    exit(1)

points = []
skipped = []
for i, chunk in enumerate(chunks):
    vec = embed(chunk)
    if vec is None:
        skipped.append(chunk[:60])
        print(f"  [{i+1}/{len(chunks)}] SKIPPED (embedding failed): {chunk[:60]}...")
        continue
    points.append(PointStruct(
        id=str(uuid.uuid4()),
        vector=vec,
        payload={"text": chunk, "source": "campus_alcoy", "index": i},
    ))
    print(f"  [{i+1}/{len(chunks)}] {chunk[:70]}...")

client.upsert(collection_name=COLLECTION, points=points)
print(f"\nLoaded {len(points)} new points into '{COLLECTION}'")
if skipped:
    print(f"Skipped {len(skipped)} chunks due to embedding errors")
