import time
from ja_motor_control import *

id = [1,2]                                                         # list of int
initial_positions = [0, 0]                                         # list of int
target_positions = [3309467, 3309467]                              # list of int
port_name= "COM6"                                                  # Windows: "COM*"  Linux: "/dev/ttyUSB*"
motor = MotorControl(port_name, id, 115200)                        # Params: serial port (str), list of actuator ID, baud rate

enable(motor, 1)                                                   # Params: MotorControl object, servo_value  ps:if servo_value == 0：Disable, else: Enable
time.sleep(1)
write_time_position(motor, initial_positions, target_positions)    # Params: MotorControl object, list of current_value