import os
import subprocess
import time
import requests

from settings import API_IP_IMAGE, API_PORT_IMAGE
from communication.stm32 import STMLink


IMAGE_API = f"http://{API_IP_IMAGE}:{API_PORT_IMAGE}/predict"
SNAP = "/tmp/task2.jpg"

READ_DISTANCE = 40       # cm from obstacle when reading arrow
TURN_DISTANCE = 55       # back off before large turn

IR_OBJECT = 30           # object beside robot
IR_CLEAR = 38            # hysteresis: clearly past edge

EDGE_STEP = 6
FRONT_SAFETY = 22


def log(msg):
    print(f"[TASK2] {msg}", flush=True)


class Robot:

    def __init__(self):
        self.link = STMLink()

    def connect(self):
        self.link.connect()

    def disconnect(self):
        self.link.disconnect()

    def sensors(self):
        f = self.link.read_sensors()

        if not f or len(f) < 6:
            raise RuntimeError("No STM sensor telemetry")

        return {
            "left": float(f[0]),
            "right": float(f[1]),
            "us": float(f[2]),
            "us_valid": bool(int(float(f[3]))),
            "heading": float(f[4]),
            "gyro_z": float(f[5]),
        }

    def send(self, cmd):
        log(cmd)

        self.link.send(cmd)
        reply = self.link.recv()

        if reply is None:
            raise RuntimeError("STM timeout")

        if reply.startswith("ERROR"):
            raise RuntimeError(reply)

    def forward(self, cm):
        self.send(f"center,0,forward,{int(cm)}")

    def reverse(self, cm):
        self.send(f"center,0,reverse,{int(cm)}")

    def turn_left(self, deg):
        before = self.sensors()["heading"]

        self.send(f"left,{deg},forward,0")

        after = self.sensors()["heading"]

        log(f"LEFT turn: {before:.1f} -> {after:.1f}")

    def turn_right(self, deg):
        before = self.sensors()["heading"]

        self.send(f"right,{deg},forward,0")

        after = self.sensors()["heading"]

        log(f"RIGHT turn: {before:.1f} -> {after:.1f}")

    def stop(self):
        self.send("STOP")


def approach(robot, target=READ_DISTANCE):

    while True:

        s = robot.sensors()

        log(
            f"US={s['us']:.1f} "
            f"L={s['left']:.1f} "
            f"R={s['right']:.1f} "
            f"H={s['heading']:.1f}"
        )

        if not s["us_valid"]:
            time.sleep(0.1)
            continue

        error = s["us"] - target

        if abs(error) <= 4:
            return

        # Too close
        if error < 0:
            robot.reverse(min(6, abs(error)))
            continue

        # Adaptive step size
        if error > 70:
            step = 25
        elif error > 35:
            step = 15
        elif error > 15:
            step = 8
        else:
            step = 4

        robot.forward(step)


def set_turn_distance(robot):

    while True:

        s = robot.sensors()

        error = s["us"] - TURN_DISTANCE

        if abs(error) <= 5:
            return

        if error < 0:
            robot.reverse(min(8, abs(error)))
        else:
            robot.forward(min(8, error))


def take_picture():

    if os.path.exists(SNAP):
        os.remove(SNAP)

    subprocess.run([
        "rpicam-still",
        "-e", "jpg",
        "-n",
        "-t", "250",
        "--width", "640",
        "--height", "480",
        "-q", "80",
        "-o", SNAP
    ])

    with open(SNAP, "rb") as f:

        r = requests.post(
            IMAGE_API,
            files={"image": ("task2.jpg", f, "image/jpeg")},
            timeout=60
        )

    if r.status_code == 404:
        return None

    r.raise_for_status()

    return r.json()

if __name__ == "__main__":

    robot = Robot()

    robot.connect()

    try:

        for i in range(10):

            print(robot.sensors())

            time.sleep(0.5)

    finally:

        robot.disconnect()
