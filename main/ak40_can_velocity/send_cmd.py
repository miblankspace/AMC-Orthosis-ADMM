"""Send a motor command to the ESP32 over COM (closes cleanly).

Usage:
  python send_cmd.py v 500
  python send_cmd.py s
  python send_cmd.py i

Close Serial Monitor / idf monitor / dashboard first so COM14 is free.
"""
import sys
import time
import serial

PORT = "COM14"
BAUD = 115200

def main():
    if len(sys.argv) < 2:
        print("Usage: python send_cmd.py <cmd>")
        print("Examples: python send_cmd.py v 500")
        print("          python send_cmd.py s")
        sys.exit(1)

    cmd = " ".join(sys.argv[1:]).strip() + "\n"
    print(f"Opening {PORT} @ {BAUD}, sending: {cmd.strip()!r}")

    with serial.Serial(PORT, BAUD, timeout=0.2) as ser:
        time.sleep(0.3)  # allow USB settle after open (may reset board)
        ser.reset_input_buffer()
        ser.write(cmd.encode("utf-8"))
        ser.flush()
        end = time.time() + 3.0
        while time.time() < end:
            line = ser.readline().decode("utf-8", errors="ignore").strip()
            if line:
                print(line)

if __name__ == "__main__":
    main()
