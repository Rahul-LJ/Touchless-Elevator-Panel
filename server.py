"""
ESP32 Elevator Simulator - Speech Recognition Server
=====================================================
Local speech-to-text server using OpenAI Whisper.
Receives audio from ESP32, returns transcribed text.

Requirements:
    pip install flask openai-whisper

Usage:
    python server.py

Server will start at: http://0.0.0.0:8080
"""

from flask import Flask, request, jsonify
import whisper
import numpy as np
import io
import wave
import os

app = Flask(__name__)

# Load Whisper model (options: tiny, base, small, medium, large)
# 'tiny' is fastest, 'base' is good balance, 'small' is more accurate
print("Loading Whisper model...")
model = whisper.load_model("base")
print("Model loaded!")

# Store recognized commands for elevator control
VALID_COMMANDS = [
    "floor 1", "floor 2", "floor 3",
    "first floor", "second floor", "third floor",
    "go to 1", "go to 2", "go to 3",
    "level 1", "level 2", "level 3",
    "top floor", "penthouse",
    "up", "down", "go up", "go down", "higher", "lower"
]


def parse_wav_audio(audio_bytes: bytes) -> np.ndarray:
    """Convert WAV bytes to numpy array for Whisper."""
    with wave.open(io.BytesIO(audio_bytes), 'rb') as wav_file:
        n_channels = wav_file.getnchannels()
        sample_width = wav_file.getsampwidth()
        framerate = wav_file.getframerate()
        n_frames = wav_file.getnframes()

        # Read all frames
        raw_data = wav_file.readframes(n_frames)

        # Convert to numpy array
        if sample_width == 2:  # 16-bit
            audio = np.frombuffer(raw_data, dtype=np.int16)
        elif sample_width == 1:  # 8-bit
            audio = np.frombuffer(raw_data, dtype=np.int8)
        else:
            raise ValueError(f"Unsupported sample width: {sample_width}")

        # Convert to float32 and normalize to [-1, 1]
        audio = audio.astype(np.float32) / 32768.0

        # If stereo, convert to mono
        if n_channels == 2:
            audio = audio.reshape(-1, 2).mean(axis=1)

        return audio


def normalize_command(text: str) -> str:
    """Normalize recognized text to standard command format."""
    text = text.lower().strip()

    # Remove common filler words
    text = text.replace("please", "").replace("can you", "").replace("i want", "")
    text = " ".join(text.split())  # Remove extra spaces

    return text


@app.route('/api/speech', methods=['POST'])
def speech_to_text():
    """
    ESP32 sends WAV audio, server returns transcribed text.

    Expected ESP32 request:
        POST /api/speech
        Content-Type: audio/wav
        Body: WAV file bytes

    Response:
        {
            "status": "success",
            "text": "floor 2",
            "confidence": 0.92
        }
    """
    try:
        # Get audio data from request
        if request.content_type == 'audio/wav':
            audio_bytes = request.get_data()
        else:
            audio_bytes = request.files['audio'].read()

        print(f"Received audio: {len(audio_bytes)} bytes")

        # Parse WAV to numpy array
        audio = parse_wav_audio(audio_bytes)

        # Transcribe with Whisper
        result = model.transcribe(audio, language="en")
        transcribed_text = result["text"].strip()

        print(f"Transcribed: '{transcribed_text}'")

        # Normalize the command
        normalized = normalize_command(transcribed_text)

        return jsonify({
            "status": "success",
            "text": normalized,
            "original": transcribed_text,
            "confidence": 0.95  # Whisper doesn't provide confidence, using placeholder
        })

    except Exception as e:
        print(f"Error: {str(e)}")
        return jsonify({
            "status": "error",
            "text": "",
            "error": str(e)
        }), 500


@app.route('/api/command', methods=['POST'])
def command():
    """
    ESP32 sends floor command for logging/sync.

    Response:
        {
            "status": "queued",
            "currentFloor": 1,
            "destination": 2,
            "queue": [2]
        }
    """
    try:
        data = request.get_json()
        command = data.get('command', '')
        current_floor = data.get('currentFloor', 1)

        print(f"Command received: {command} (from floor {current_floor})")

        # Determine destination
        destination = current_floor
        if 'floor_1' in command or 'floor 1' in command:
            destination = 1
        elif 'floor_2' in command or 'floor 2' in command:
            destination = 2
        elif 'floor_3' in command or 'floor 3' in command:
            destination = 3
        elif 'up' in command:
            destination = min(current_floor + 1, 3)
        elif 'down' in command:
            destination = max(current_floor - 1, 1)

        return jsonify({
            "status": "queued",
            "currentFloor": current_floor,
            "destination": destination,
            "queue": [destination] if destination != current_floor else []
        })

    except Exception as e:
        print(f"Command error: {str(e)}")
        return jsonify({
            "status": "error",
            "error": str(e)
        }), 500


@app.route('/api/log', methods=['POST'])
def log():
    """
    ESP32 sends log events.
    Just prints to console and returns success.
    """
    try:
        data = request.get_json()
        timestamp = data.get('timestamp', 0)
        floor = data.get('floor', '?')
        state = data.get('state', '?')
        message = data.get('message', '')

        print(f"[LOG] Floor {floor} | State {state} | {message}")

        return jsonify({"status": "logged"})

    except Exception as e:
        return jsonify({"status": "error", "error": str(e)}), 500


@app.route('/health', methods=['GET'])
def health():
    """Health check endpoint."""
    return jsonify({
        "status": "healthy",
        "model": "whisper-base",
        "valid_commands": VALID_COMMANDS
    })


@app.route('/', methods=['GET'])
def index():
    """Info page."""
    return """
    <h1>ESP32 Elevator Speech Server</h1>
    <p>Whisper-based speech recognition for elevator control.</p>

    <h2>Endpoints:</h2>
    <ul>
        <li><code>POST /api/speech</code> - Send WAV audio, get transcribed text</li>
        <li><code>POST /api/command</code> - Send floor command</li>
        <li><code>POST /api/log</code> - Send log event</li>
        <li><code>GET /health</code> - Health check</li>
    </ul>

    <h2>Test with curl:</h2>
    <pre>curl -X POST http://localhost:8080/api/speech -H "Content-Type: audio/wav" --data-binary @test.wav</pre>
    """


if __name__ == '__main__':
    # Get local IP for ESP32 to connect to
    import socket
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(('8.8.8.8', 80))
        local_ip = s.getsockname()[0]
    except:
        local_ip = '127.0.0.1'
    finally:
        s.close()

    print("\n" + "="*50)
    print("ESP32 Elevator Speech Server")
    print("="*50)
    print(f"Server running at: http://{local_ip}:8080")
    print(f"Local: http://localhost:8080")
    print("\nIn ESP32 code, use:")
    print(f'  const char* SPEECH_API_URL = "http://{local_ip}:8080/api/speech";')
    print("="*50 + "\n")

    app.run(host='0.0.0.0', port=8080, debug=False)
