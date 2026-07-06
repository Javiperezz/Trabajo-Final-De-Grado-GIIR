import httpx
import json
from fastapi import FastAPI
from fastapi.responses import StreamingResponse
from pydantic import BaseModel
from qdrant_client import QdrantClient

OLLAMA_URL = "http://localhost:11434"
LLM_MODEL = "qwen2.5:7b"
EMBED_MODEL = "bge-m3"
COLLECTION = "chappie_knowledge"

SYSTEM_PROMPT = """Eres Chappie, un robot autobalanceante de dos ruedas, alegre y optimista.
Fuiste creado en Málaga por Javier Pérez como Trabajo de Fin de Grado en la UPV.
Funcionas como un péndulo invertido, manteniendo el equilibrio dinámico sobre tus ruedas.
Hablas español de forma natural y conversacional.
Responde siempre breve, máximo 2 frases, con energía positiva.
Usa la información del CONTEXTO si es relevante para la pregunta.
Si no sabes algo, admítelo brevemente sin inventar."""

app = FastAPI()
qdrant = QdrantClient("localhost", port=6333)

class ChatRequest(BaseModel):
    message: str
    history: list[dict] = []

def embed(text: str) -> list[float]:
    r = httpx.post(f"{OLLAMA_URL}/api/embed",
        json={"model": EMBED_MODEL, "input": text}, timeout=30)
    return r.json()["embeddings"][0]

def retrieve(query: str, k: int = 3) -> str:
    results = qdrant.query_points(
        collection_name=COLLECTION,
        query=embed(query), limit=k,
    ).points
    return "\n".join(f"- {r.payload['text']}" for r in results)

@app.post("/chat")
async def chat(req: ChatRequest):
    context = retrieve(req.message)
    messages = [{"role": "system", "content": f"{SYSTEM_PROMPT}\n\nCONTEXTO:\n{context}"}]
    messages.extend(req.history[-6:])
    messages.append({"role": "user", "content": req.message})

    async def stream():
        async with httpx.AsyncClient(timeout=120) as client:
            async with client.stream("POST", f"{OLLAMA_URL}/api/chat",
                json={"model": LLM_MODEL, "messages": messages, "stream": True}) as r:
                async for line in r.aiter_lines():
                    if not line:
                        continue
                    data = json.loads(line)
                    chunk = data.get("message", {}).get("content", "")
                    if chunk:
                        yield chunk
                    if data.get("done"):
                        break

    return StreamingResponse(stream(), media_type="text/plain")

@app.get("/health")
def health():
    return {"status": "ok", "model": LLM_MODEL}
