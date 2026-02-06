from gpiozero import Servo
from gpiozero.pins.pigpio import PiGPIOFactory
from time import sleep

factory = PiGPIOFactory()
servo = Servo(12,
              pin_factory=factory,
              min_pulse_width=0.5/1000,
              max_pulse_width=2.5/1000)

servo.min()
sleep(1)
servo.value = None
