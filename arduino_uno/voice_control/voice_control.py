import json
import queue
import sounddevice as sd
from vosk import Model, KaldiRecognizer
import serial

# ---------- SETTINGS ----------
MODEL_PATH = "vosk-model-small-en-us-0.15"
COM_PORT = "COM15"
BAUD = 115200
SAMPLE_RATE = 16000
# ------------------------------

print("Loading Vosk model...")
model = Model(MODEL_PATH)

recognizer = KaldiRecognizer(
    model,
    SAMPLE_RATE,
    '["up", "down", "stop", "help"]'
)

q = queue.Queue()

print("Connecting to Arduino...")
ser = serial.Serial(COM_PORT, BAUD)

print("Ready!")

def callback(indata, frames, time, status):
    q.put(bytes(indata))

with sd.RawInputStream(
        samplerate=SAMPLE_RATE,
        blocksize=8000,
        dtype='int16',
        channels=1,
        callback=callback):

    print("\nListening...\n")

    while True:

        data = q.get()

        if recognizer.AcceptWaveform(data):

            result = json.loads(recognizer.Result())

            text = result.get("text","")

            if text == "":
                continue

            print("Heard:", text)

            if "up" in text:

                print(">>> UP")
                ser.write(b"v 500\n")

            elif "down" in text:

                print(">>> DOWN")
                ser.write(b"v -500\n")

            elif "stop" in text:

                print(">>> STOP")
                ser.write(b"s\n")

            elif "help" in text:

                print(">>> HELP")
                ser.write(b"s\n")