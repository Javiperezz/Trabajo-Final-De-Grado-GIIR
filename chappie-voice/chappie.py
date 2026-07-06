import random
import httpx
import sounddevice as sd
import numpy as np
import subprocess
import tempfile
import os
import queue
import time
import threading
import re
import torch
from faster_whisper import WhisperModel
from openwakeword.model import Model as WakeModel


LAPTOP_IP = "192.168.0.109"
SERVER_URL = f"http://{LAPTOP_IP}:8000/chat"
PIPER_MODEL = "voices/es_ES-carlfm-x_low.onnx"
WAKE_MODEL = "oh_la_chah_pee.onnx"
WAKE_THRESHOLD = 0.5
SAMPLE_RATE = 16000
WAKE_FRAME = 1280
VAD_MAX_SECONDS = 8       
VAD_SILENCE_MS = 700     
VAD_MIN_SPEECH_MS = 300   

FILLERS = [
    "fillers/mmm.wav",
    "fillers/a_ver.wav",
    "fillers/djame_pensar.wav",
    "fillers/vale.wav",
]


print("Cargando Whisper...")
whisper = WhisperModel(
    "small", device="cpu", compute_type="int8",
    cpu_threads=4, num_workers=1,
)
print("Whisper listo")

print("Cargando wake word model...")
wake = WakeModel(wakeword_models=[WAKE_MODEL], inference_framework="onnx")
WAKE_NAME = list(wake.models.keys())[0]
print(f"Wake word listo: {WAKE_NAME}")

print("Cargando VAD...")
vad_model, _ = torch.hub.load(
    repo_or_dir='snakers4/silero-vad',
    model='silero_vad',
    trust_repo=True,
    verbose=False,
)
print("VAD listo")

history = []
audio_q = queue.Queue()


def play_cue(filename: str, blocking: bool = True):
    
    if blocking:
        subprocess.run(["aplay", "-q", filename], check=False)
    else:
        subprocess.Popen(["aplay", "-q", filename])


def wake_audio_callback(indata, frames, time_info, status):
    pcm = (indata[:, 0] * 32767).astype(np.int16)
    audio_q.put(pcm)


def wait_for_wake_word():
    print("\nEscuchando... di 'Hola Chappie'")
    while not audio_q.empty():
        audio_q.get_nowait()

    with sd.InputStream(samplerate=SAMPLE_RATE, channels=1, dtype='float32',
                        blocksize=WAKE_FRAME, callback=wake_audio_callback):
        while True:
            pcm = audio_q.get()
            scores = wake.predict(pcm)
            score = scores.get(WAKE_NAME, 0)
            if score > WAKE_THRESHOLD:
                print(f"  -> Despertado! (score={score:.2f})")
                return


def record_audio_vad():
   
    chunk_size = 512   # 32ms a 16kHz, tamaño nativo de Silero VAD
    silence_chunks_needed = int(VAD_SILENCE_MS / 32)
    min_speech_chunks = int(VAD_MIN_SPEECH_MS / 32)

    audio_chunks = []
    silence_count = 0
    speech_chunks = 0
    speech_started = False
    max_chunks = int(VAD_MAX_SECONDS * SAMPLE_RATE / chunk_size)

    print("  Habla ahora...")
    with sd.InputStream(samplerate=SAMPLE_RATE, channels=1,
                        dtype='float32', blocksize=chunk_size) as stream:
        for _ in range(max_chunks):
            chunk, _ = stream.read(chunk_size)
            chunk_f32 = chunk.flatten()
            audio_chunks.append(chunk_f32)
            speech_prob = vad_model(torch.from_numpy(chunk_f32), SAMPLE_RATE).item()
            if speech_prob > 0.5:
                speech_started = True
                speech_chunks += 1
                silence_count = 0
            elif speech_started:
                silence_count += 1
                if silence_count >= silence_chunks_needed and speech_chunks >= min_speech_chunks:
                    break

    return np.concatenate(audio_chunks)


def transcribe(audio: np.ndarray) -> str:
    segments, _ = whisper.transcribe(
        audio,
        language="es",
        beam_size=1,
        vad_filter=True,
        condition_on_previous_text=False,
    )
    return " ".join(s.text for s in segments).strip()


def speak(text: str):
    with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as f:
        wav_path = f.name
    try:
        subprocess.run(
            ["piper", "--model", PIPER_MODEL, "--output_file", wav_path,
             "--length_scale", "0.95"],
            input=text, text=True, check=True, capture_output=True,
        )
        subprocess.run(["aplay", "-q", wav_path], check=True)
    finally:
        if os.path.exists(wav_path):
            os.unlink(wav_path)


def ask_chappie_streaming(message: str, sentence_queue: queue.Queue) -> str:
   
    payload = {"message": message, "history": history}
    full = ""
    buffer = ""

    print("  Chappie: ", end="", flush=True)
    try:
        with httpx.stream("POST", SERVER_URL, json=payload, timeout=120) as r:
            for chunk in r.iter_text():
                print(chunk, end="", flush=True)
                full += chunk
                buffer += chunk
             
                while True:
                    match = re.search(r"([^.!?]*[.!?])", buffer)
                    if not match:
                        break
                    sentence = match.group(1).strip()
                    buffer = buffer[match.end():]
                    if sentence:
                        sentence_queue.put(sentence)
    finally:
       
        if buffer.strip():
            sentence_queue.put(buffer.strip())
        print()
        sentence_queue.put(None)  # señal de fin al hilo TTS
    return full


def tts_worker(sentence_queue: queue.Queue):

    while True:
        sentence = sentence_queue.get()
        if sentence is None:
            break
        try:
            speak(sentence)
        except Exception as e:
            print(f"  Error TTS en '{sentence[:40]}...': {e}")



def main():
    print("\n Chappie esta listo ")
    print("Di 'Hola Chappie' para activarme. Ctrl+C para salir.\n")

    try:
        while True:
            wait_for_wake_word()

            
            play_cue("listen_start.wav", blocking=True)

            try:
                t0 = time.time()
                audio = record_audio_vad()
                t1 = time.time()

                play_cue("listen_end.wav", blocking=False)

                user_text = transcribe(audio)
                t2 = time.time()

                if not user_text:
                    print("  No te entendi, intenta de nuevo.")
                    continue
                print(f"  Tu: {user_text}")

                play_cue(random.choice(FILLERS), blocking=False)

                sentence_queue = queue.Queue()
                tts_thread = threading.Thread(
                    target=tts_worker, args=(sentence_queue,)
                )
                tts_thread.start()

                response = ask_chappie_streaming(user_text, sentence_queue)
                tts_thread.join()
                t3 = time.time()

                if not response.strip():
                    print("  Respuesta vacia.")
                    continue

                history.append({"role": "user", "content": user_text})
                history.append({"role": "assistant", "content": response})

                print(f"\n  [record: {t1-t0:.1f}s | STT: {t2-t1:.1f}s | "
                      f"LLM+TTS: {t3-t2:.1f}s | total: {t3-t0:.1f}s]\n")

            except httpx.RequestError as e:
                print(f"  No me puedo conectar con el servidor: {e}")
            except Exception as e:
                print(f"  Error: {e}")

    except KeyboardInterrupt:
        print("\nAdios!")


if __name__ == "__main__":
    main()
