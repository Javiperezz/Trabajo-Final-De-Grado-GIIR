
import sounddevice as sd
import numpy as np
from openwakeword.model import Model

WAKE_MODEL_PATH = "oh_la_chah_pee.onnx"   
THRESHOLD = 0.3

print("Cargando modelo de wake word personalizado...")
model = Model(wakeword_models=[WAKE_MODEL_PATH], inference_framework="onnx")
WAKE_WORD = list(model.models.keys())[0]
print(f"Modelo cargado: {WAKE_WORD}")
print(f"Escuchando... di 'Hey Chappie'. Ctrl+C para salir.\n")

sample_rate = 16000
frame_length = 1280   # 80ms at 16kHz

last_score = 0
def audio_callback(indata, frames, time, status):
    global last_score
    pcm = (indata[:, 0] * 32767).astype(np.int16)
    scores = model.predict(pcm)
    score = scores.get(WAKE_WORD, 0)
    if score > THRESHOLD:
        print(f">>> WAKE WORD DETECTADO (score={score:.2f}) <<<")
        last_score = 0
    elif score > 0.2 and score != last_score:
        # Show near-misses for tuning
        print(f"   ... near miss: {score:.2f}")
        last_score = score

try:
    with sd.InputStream(samplerate=sample_rate, channels=1, dtype='float32',
                        blocksize=frame_length, callback=audio_callback):
        while True:
            sd.sleep(1000)
except KeyboardInterrupt:
    print("\nSaliendo.")
