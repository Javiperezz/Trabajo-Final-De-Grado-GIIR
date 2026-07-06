# Trabajo-Final-De-Grado-GIIR
Repositorio con los programas descritos en la memoria del TFG "Chappie: robot autoequilibrado basado en ROS 2". Javier Pérez Trujillo, UPV EPSA 2026.
# Chappie: robot autoequilibrado con ROS 2 e IA

Trabajo Fin de Grado — Grado en Informática Industrial y Robótica  
Escuela Politécnica Superior de Alcoy — Universitat Politècnica de València  
Convocatoria de julio de 2026

**Autor:** Javier Pérez Trujillo  
**Tutor:** Jaime Masiá Vañó

En este repositorio se encuentran adjuntos todos los programas descritos 
en la memoria del proyecto. Chappie es un robot autoequilibrado de dos 
ruedas basado en ROS 2, con percepción 360° mediante doble LiDAR y un 
subsistema de interacción por voz que integra un modelo de lenguaje 
ejecutado localmente.

## Estructura del repositorio

### `chappie-ws/` — Workspace ROS 2 (Jetson Orin NX)

Contiene los dos paquetes propios desarrollados en el proyecto:

- **`chappie_control`**: nodos de control motriz. Incluye el controlador 
  en cascada de equilibrio (`cascade_balance_node`), el puente de 
  teleoperación con el mando PlayStation 5 (`joy_cmd_vel_node`), el 
  filtro reactivo de seguridad sobre obstáculos (`obstacle_guard_node`) 
  y el launch principal del sistema.
- **`view_robot_pkg`**: descripción cinemática del robot (URDF, mallas 
  STL) y launch de percepción con los drivers de los dos LiDAR, la 
  fusión de escaneos y la publicación del árbol TF.

### `chappie-voice/` — Cliente de voz (Orange Pi 5 Pro)

Cliente Python que gestiona el ciclo completo de interacción por voz: 
detección del wake word "Hola Chappie" con `openWakeWord`, captura con 
detección automática de fin de habla mediante Silero VAD, transcripción 
con `faster-whisper`, consulta HTTP en streaming al servidor de 
inferencia y síntesis con `Piper`. Incluye los modelos externos 
necesarios: voz de Piper (`es_ES-carlfm-x_low.onnx`) y wake word 
entrenado específicamente para el proyecto (`oh_la_chah_pee.onnx`). 
No es un nodo ROS 2: se comunica con el servidor por HTTP.

### `chappie-brain/` — Servidor de inferencia (portátil externo)

Servidor Python con FastAPI que expone la API REST del subsistema 
conversacional. Recupera contexto de una base vectorial Qdrant mediante 
embeddings de `bge-m3`, construye el prompt final y devuelve la 
respuesta del modelo `Qwen2.5:7B` servido a través de Ollama en modo 
streaming. Se ejecuta fuera del middleware ROS 2.

## Requisitos

- Ubuntu 22.04 en las tres unidades de cómputo
- ROS 2 Humble Hawksbill (Jetson)
- Python 3.10+
- Ollama con los modelos `qwen2.5:7b` y `bge-m3` (portátil)
- Qdrant (portátil)
- SSH configurado sin contraseña entre Jetson y Orange Pi

## Puesta en marcha

### 1. Portátil externo — Servidor de inferencia

```bash
cd chappie-brain
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt

# Con Ollama y Qdrant ya en marcha:
uvicorn server:app --host 0.0.0.0 --port 8000
```

### 2. Orange Pi 5 Pro — Cliente de voz

Basta con crear el entorno virtual una única vez; el arranque del 
cliente lo gestiona el launch principal de la Jetson (siguiente paso).

```bash
cd chappie-voice
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```

### 3. Jetson Orin NX — Sistema completo

```bash
cd chappie-ws
colcon build --symlink-install
source install/setup.bash
ros2 launch chappie_control chappie.launch.py
```

Este launch arranca todos los nodos ROS 2 locales (control, percepción, 
seguridad, teleoperación) y, adicionalmente, abre una sesión SSH a la 
Orange Pi que ejecuta el cliente de voz. De este modo, el sistema 
completo se levanta desde una sola terminal.

## Notas de configuración

Antes de ejecutar el sistema en un entorno distinto al del proyecto 
original, deben adaptarse los siguientes parámetros:

- **IPs de red local** en `chappie-ws/.../chappie.launch.py` (variable 
  `ORANGE_HOST`) y en `chappie-voice/chappie.py` (variable `LAPTOP_IP`), 
  según la asignación DHCP de la red donde se despliegue.
- **Rutas absolutas** de la Orange Pi (`ORANGE_DIR`, `ORANGE_USER` en el 
  launch principal) para reflejar el usuario y la ubicación del cliente 
  de voz en la máquina destino.

## Documentación completa

Los detalles de diseño mecánico, sintonización del controlador, 
arquitectura software y validación experimental están en la memoria 
del TFG.
