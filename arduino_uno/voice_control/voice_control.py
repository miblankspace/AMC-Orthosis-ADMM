import json
import queue
import threading
import serial
import sounddevice as sd
from vosk import Model, KaldiRecognizer

# =====================================================
# SETTINGS
# =====================================================

MODEL_PATH = "vosk-model-small-en-us-0.15"

COM_PORT = "COM15"
BAUD = 115200

MIC_DEVICE = 18          # AB13X USB Audio microphone
SAMPLE_RATE = 48000      # Use your microphone's default sample rate

# =====================================================

print("Loading Vosk model...")
model = Model(MODEL_PATH)

recognizer = KaldiRecognizer(
    model,
    SAMPLE_RATE,
    '["up", "down", "stop", "help"]'
)

audio_queue = queue.Queue()

print("Connecting to Arduino...")
ser = serial.Serial(COM_PORT, BAUD, timeout=0.1)

print("Connected!")

# -----------------------------------------------------
# Read Arduino messages continuously
# -----------------------------------------------------

def serial_reader():

    while True:

        try:

            if ser.in_waiting:

                line = ser.readline().decode(errors="ignore").strip()

                if line:

                    print(f"\n[Arduino] {line}")

        except Exception as e:

            print("Serial Error:", e)
            break


threading.Thread(target=serial_reader, daemon=True).start()

# -----------------------------------------------------
# Audio callback
# -----------------------------------------------------

def callback(indata, frames, time, status):

    if status:
        print(status)

    audio_queue.put(bytes(indata))

# -----------------------------------------------------
# Main
# -----------------------------------------------------

print("\n===================================")
print(" Voice Controller Ready")
print(" Say:")
print("   Up")
print("   Down")
print("   Stop")
print("   Help")
print("===================================\n")

with sd.RawInputStream(
        samplerate=SAMPLE_RATE,
        blocksize=4800,
        device=MIC_DEVICE,
        dtype="int16",
        channels=1,
        callback=callback):

    while True:

        data = audio_queue.get()

        if recognizer.AcceptWaveform(data):

            result = json.loads(recognizer.Result())

            text = result.get("text", "").strip()

            if text == "":
                continue

            print(f"\nHeard : {text}")

            # -----------------------------
            # UP
            # -----------------------------

            if text == "up":

                print(">>> Sending UP")

                ser.write(b"v 500\n")

            # -----------------------------
            # DOWN
            # -----------------------------

            elif text == "down":

                print(">>> Sending DOWN")

                ser.write(b"v -500\n")

            # -----------------------------
            # STOP
            # -----------------------------

            elif text == "stop":

                print(">>> Sending STOP")

                ser.write(b"s\n")

            # -----------------------------
            # HELP
            # -----------------------------

            elif text == "help":

                print(">>> EMERGENCY STOP")

                ser.write(b"s\n")

        else:

            partial = json.loads(recognizer.PartialResult())

            p = partial.get("partial", "")

            if p:

                print(f"\rListening : {p}          ", end="")