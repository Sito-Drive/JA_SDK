# import time
from .ja_motor_interface import MotorControl
from .ja_motor_constants import *

def enable(motor: MotorControl, value):
    motor.Enable(value)

def write_position(motor: MotorControl, pos_value):
    motor.write_pos(pos_value)

def write_velocity(motor: MotorControl, vel_value):
    motor.write_vel(vel_value)

def write_current(motor: MotorControl, cur_value):
    motor.write_cur(cur_value)

def change_parameters(motor: MotorControl, address, value):
    motor.Change_Parameters(address, value)

def stop(motor: MotorControl):
    motor.Stop()

def clear(motor: MotorControl):
    motor.Clear()

def set_origin(motor: MotorControl):
    motor.Set_Origin()

def return_origin(motor: MotorControl):
    motor.Return_Origin()

def save(motor: MotorControl):
    motor.Save()

def read(motor:MotorControl, flag):
    return motor.read_values(flag)

def write_time_position(motor: MotorControl, initial_positions, target_positions, Hz = 150, duration = 4.0, num_points = 5000):
    motor.s_curve_trajectory(initial_positions, target_positions, Hz, duration, num_points)

def brake(motor: MotorControl, value):
    motor.Brake(value)