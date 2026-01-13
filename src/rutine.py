#!/usr/bin/env python3
import serial
import sys
import random
from queue import Queue, Empty
import time


PORT = "/dev/ttyACM1"
BAUD = 115200

def makeRutine(quantity) -> Queue:
    rutines = Queue()

    for _ in range(quantity):
        MOVE = random.randint(0, 6)
        TURN = random.randint(-360, 360)

        condition = random.randint(0, 1)
        if condition == 0:
            rutine = f"Turn={TURN}\n"
        else:
            rutine = f"Move={MOVE}\n"

        rutines.put(rutine)

    return rutines   

def sendRutines(rutines):
    ser = serial.Serial(PORT, BAUD, timeout=1)
    time.sleep(2)

    while True:
        try:
            rutine = rutines.get(timeout=0.05)
            ser.write(rutine.encode("utf-8"))
            ser.flush()
            print(f"RUTINE SENT: {rutine.strip()} | Remaining: {rutines.qsize()}")
            time.sleep(6)
        except Empty:
            print("All rutines sent successfully!")
            ser.close()
            break

def main():

    rutines = makeRutine(5)

    sending = sendRutines(rutines)

    print(sending)

    return 0

if __name__ == "__main__":
    sys.exit(main())
