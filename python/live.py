from motion_controller import MotionController

import time
from datetime import datetime
import random
import math3d as m3d


PANELIF = None

def encoder_changed(
    encoder,
    value,
    delta
):

    msg = f"E {encoder}: {value} (delta={delta})"
    log(msg)


def motor_done(
    motor
):
    value = PANELIF.get(motor)
    msg = f"M {motor}: {value}"
    log(msg)

def log(msg):

    current_timestamp = time.time()
    dt = datetime.fromtimestamp(current_timestamp)
    readable_time = dt.strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]

    print(f"{readable_time}  {msg}")



class PanelInterface:
    def __init__(self):
        self.mc = MotionController(
        encoder_callback=encoder_changed,
        encoder_callback_delta=100,
        motor_done_callback=motor_done,
        homed_callback=None,
        estop_callback=None,
        debug=False)

        time.sleep(0.3)

        global PANELIF
        PANELIF = self

    def set(self,m,val,speed=1000,waitFor=True):
        val_e = self.mc.get_encoders()[m]
        val_delta = val - val_e
        self.move(m,val_delta,speed,waitFor)

    def move(self,m,val_delta,speed=1000,waitFor=True):
        steps = int(abs(val_delta) // 5)
        dir = 0 if val_delta > 0 else -1
        self.mc.move_motor(motor=m,steps=steps,direction=dir,speed_us=speed,waitFor=waitFor)

    def get(self,m):
        return self.mc.get_encoders()[m]

    def null(self):
        self.mc.zero_encoder(0)
        self.mc.zero_encoder(1)
        self.mc.zero_encoder(2)

    def setMulti(self,v1,v2,v3):
        self.set(0, v1, waitFor=False)
        self.set(1, v2, waitFor=False)
        self.set(2, v3, waitFor=False)



def get_interface():
    pi = PanelInterface()
    return pi

def demo(pi):
    pi.null()
    d = -1
    i = 0
    while i < 5:
        d *= -1
        pi.move(0,random.randint(-10000,10000),waitFor=False)
        pi.move(1, random.randint(-10000, 10000), waitFor=False, speed=8000)
        pi.move(2, d*15000, waitFor=True)
        i+=1
    pi.set(0,0)
    pi.set(1, 0)
    pi.set(2, 0)
