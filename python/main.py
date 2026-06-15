from motion_controller import MotionController

import time


def encoder_changed(
    encoder,
    value,
    delta
):

    print(
        f"Encoder {encoder}: "
        f"{value} "
        f"(delta={delta})"
    )


def motor_done(
    motor
):

    print(
        f"Motor {motor} done"
    )


def homed(
    motor
):

    print(
        f"Motor {motor} homed"
    )


def estop():

    print("ESTOP triggered")


mc = MotionController(

    encoder_callback=encoder_changed,

    encoder_callback_delta=20,

    motor_done_callback=motor_done,

    homed_callback=homed,

    estop_callback=estop,

    debug=True
)

time.sleep(3)

#mc.enable_motor(0, True)

# blocking move

#mc.move_motor(
#    motor=0,
#    steps=5000,
#    direction=1,
#    speed_us=1000,
#    waitFor=True
#)

#print("Move complete")

# blocking home

#mc.home_motor(
#    motor=0,
#    encoder=0,
#    direction=0,
#    speed_us=2000,
#    waitFor=True
#)

#print("Homing complete")

while True:

    print(
        mc.get_encoders()
    )

    time.sleep(0.2)
