from pybricks import version
from pybricks.tools import wait, StopWatch
from pybricks.hubs import TechnicHub
from pybricks.pupdevices import Motor
from pybricks.parameters import Port, Direction, Axis


motor_a = Motor(Port.A)
motor_b = Motor(Port.B, Direction.COUNTERCLOCKWISE)

hub = TechnicHub()

target_angle = 0
prev_error = 0
integral = 0

angle = 0
watch = StopWatch()
new_time = 0

kp = 20
ki = 6
kd = 0.2

while True:
    if hub.imu.ready():
        velocity = hub.imu.angular_velocity()[1]

        time = watch.time()
        time_delta = max(1, (time - new_time)) / 1000
        new_time = time

        angle += velocity * time_delta

        error = target_angle - angle
        integral += error * time_delta
        derivative = (error - prev_error) / time_delta
        prev_error = error

        PID = kp * error + ki * integral + kd * derivative

        motor_a.dc(PID)
        motor_b.dc(PID)
