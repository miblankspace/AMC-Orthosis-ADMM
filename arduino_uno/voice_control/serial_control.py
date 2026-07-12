import serial

ser = serial.Serial("COM15",115200)

while True:
    cmd = input("> ").lower()

    if cmd=="up":
        ser.write(b"v 500\n")

    elif cmd=="down":
        ser.write(b"v -500\n")

    elif cmd=="stop":
        ser.write(b"s\n")

    elif cmd=="help":
        ser.write(b"s\n")