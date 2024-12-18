from pybricks import version
from pybricks.tools import wait, StopWatch
from pybricks.hubs import TechnicHub
from pybricks.pupdevices import Motor
from pybricks.parameters import Port


motor_a = Motor(Port.A)
motor_b = Motor(Port.B)

hub = TechnicHub()

target_angle = 0
prev_error = 0
integral = 0

time_delta = 5
angle_delta = 0

k_proportional = 20
k_integral = 0.1
k_derivative = 0

while True:

    velocity = hub.imu.angular_velocity()[1]

    angle_delta += velocity * time_delta / 1000

    error = target_angle - angle_delta
    integral += error * time_delta
    derivative = (error - prev_error) / time_delta
    prev_error = error
    speed = (
        k_proportional * error +
        k_integral * integral +
        k_derivative * derivative)

    motor_a.dc(speed)
    motor_b.dc(-speed)

    wait(time_delta)
