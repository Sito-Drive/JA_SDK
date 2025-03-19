import time
import struct
import serial
import numpy as np
from .ja_motor_constants import *

class MotorControl:
    def __init__(self, serial_port: str, id_values, baudrate = 19200):
        try:
            self.ser = serial.Serial(serial_port, baudrate)
            self.ser.timeout = 0.2
            self.ids = id_values
        except serial.SerialException as e:
            print(f"Serial port connection failed: {e}")
            self.ids = id_values
            self.ser = None

    def close_serial(self):
        if self.ser:
            self.ser.close()
            print("Serial port closed")

    #CRC
    def CRC16(self,buffer):
        temp = 0xFFFF 
        for byte in buffer:  
            temp ^= byte
            for _ in range(8):
                if (temp & 0x01) == 0:
                    temp >>= 1
                else:
                    temp >>= 1
                    temp ^= 0xA001
        return temp

    def Error_handling(self, timeout):
        try:
            response = self.ser.read(10)
            if response is not None:
                response_array = [f"0x{response.hex()[i:i+2]}" for i in range(0, len(response.hex()), 2)]
                return response_array

        except serial.SerialException as e:
            return COMMUNICATION_ABNORMALITY
        except ValueError as e:
            return VALUE_ERROR
        except IndexError as e:
            return INDEX_ERROR
        except Exception as e:
            return TIME_OUT
        
    def Hexadecimal_conversion(self,num):
        hex_str = f"{num:08x}"
        parts = [hex_str[i:i+2] for i in range(0, 8, 2)]
        parts = parts[:-1] + [parts[-1]]
        formatted_parts = [f"0x{part}" for part in parts]
        return formatted_parts

    def write(self, id, address, data):
        pack = [0] * 10
        error = 0

        pack[ID]                = id
        pack[FUNCTION]          = WRITE
        location = address.zfill(4)
        location_int = int(location, 16)
        address_1 = (location_int >> 8) & 0xff
        address_2 = location_int & 0xff
        pack[ADDRESS]           = address_1
        pack[ADDRESS + 1]       = address_2
        if data >= 0:
            hex_data = self.Hexadecimal_conversion(data)
            for i in range(0,len(hex_data)):
                hex_data_int = int(hex_data[i],16)

                pack[DATA_CONTENT + i] = hex_data_int
        elif data < 0:
            max_uint32 = (1 << 32) - 1
            complement = max_uint32 + 1 + data
            hex_data = hex(complement)[2:]
            data_32bit_int = int(hex_data,16)
            processing_data =[
            (data_32bit_int >> 24) & 0xff,
            (data_32bit_int >> 16) & 0xff,
            (data_32bit_int >> 8) & 0xff,
            data_32bit_int & 0xff]
            final_data = [f"0x{byte:02x}" for byte in processing_data]
            for i in range(0,len(final_data)):
                final_data_int = int(final_data[i],16)

                pack[DATA_CONTENT + i] = final_data_int
        crc_array = [0] * 8
        crc_array = pack[:-2]
        crc_int = self.CRC16(crc_array)
        crc1 = (crc_int >> 8) & 0xff
        crc2 = crc_int & 0xff
        pack[CRC]               = crc1
        pack[CRC + 1]           = crc2
        try:
            crc_array = pack[:-2]
            crc_int = self.CRC16(crc_array)
            pack[CRC] = crc_int >> 8
            pack[CRC + 1] = crc_int & 0xff
        except Exception as e:
            print(f"CRC calculation failed: {e}")
            return None
        self.ser.write(pack)
        response = self.ser.read(10)
        if response is not None and isinstance(response, bytes):
            response_array = [f"0x{response.hex()[i:i+2]}" for i in range(0, len(response.hex()), 2)]
        else:
            response_array  = None
            error           = response
        if error != 0 and error != 4:
            error = self.Error_handling(1.5)
            pass
        return response_array,error

    def read(self,id,address):
        pack = [0] * 10
        error = 0
        pack[ID]                = id
        pack[FUNCTION]          = READ
        location = address.zfill(4)
        location_int = int(location, 16)
        address_1 = (location_int >> 8) & 0xff
        address_2 = location_int & 0xff
        pack[ADDRESS]               = address_1
        pack[ADDRESS + 1]           = address_2
        pack[DATA_LENGTH]           = 0X00
        pack[DATA_LENGTH + 1]       = 0X00
        pack[DATA_LENGTH + 2]       = 0X00
        pack[DATA_LENGTH + 3]       = 0X02
        crc_array = [0] * 8
        crc_array = pack[:-2]
        crc_int = self.CRC16(crc_array)
        crc1 = (crc_int >> 8) & 0xff
        crc2 = crc_int & 0xff
        pack[CRC]               = crc1
        pack[CRC + 1]           = crc2
        try:
            crc_array = pack[:-2]
            crc_int = self.CRC16(crc_array)
            pack[CRC] = crc_int >> 8
            pack[CRC + 1] = crc_int & 0xff
        except Exception as e:
            print(f"CRC calculation failed: {e}")
            return None
        self.ser.write(pack)
        response = self.ser.read(10)
        if response is not None and isinstance(response, bytes):
            response_array = [f"0x{response.hex()[i:i+2]}" for i in range(0, len(response.hex()), 2)]
            read_data = (int(response_array[DATA_CONTENT][2:], 16) << 24) + \
                        (int(response_array[DATA_CONTENT + 1][2:], 16) << 16) + \
                        (int(response_array[DATA_CONTENT + 2][2:], 16) << 8) + \
                        int(response_array[DATA_CONTENT + 3][2:], 16)
            if read_data >= 0x80000000:
                read_data -= 0x100000000
        else:
            read_data       = None
            response_array  = None
            error           = response
        if error != 0 and error != 4:
            error = self.Error_handling(1.5)
            pass
        return read_data, error, response_array

    def Enable(self, servo_value):
        for motor_id in self.ids:
            try: 
                response_array, error = self.write(motor_id, SERVO_STATUS, servo_value)
                # print(f"{motor_id}\n{response_array}\n{error}")
                if len(response_array) == 10 and error == 0:
                    if servo_value == 0:
                        print(f"ID: {motor_id} DISABLE")
                    else:
                        print(f"ID: {motor_id} ENABLE")
                else:
                    print(f"ID: {motor_id} Enable failed")
            except Exception as e:
                print(f"ID: {motor_id} encountered an error during ENABLE:{e}")

    def write_pos(self, pos_value):
        for motor_id, pos in zip(self.ids, pos_value):
            try:
                response_array, error = self.write(motor_id, VEL_POSITION_MODE, pos)
                # print(f"{motor_id}\n{response_array}\n{error}")
                if len(response_array) == 10 and error == 0:
                    print(f"ID: {motor_id} Position: {pos}")
                else:
                    print(f"ID: {motor_id} Write position failed")
            except Exception as e:
                print(f"ID: {motor_id} encountered an error during write position:{e}")

    def write_tim_pos(self, pos_value):
        for motor_id, pos in zip(self.ids, pos_value):
            try:
                response_array, error = self.write(motor_id, TIM_POSITION_MODE, pos)
                # print(f"{motor_id}\n{response_array}\n{error}")
                if len(response_array) == 10 and error == 0:
                    print(f"ID: {motor_id} Position: {pos}")
                else:
                    print(f"ID: {motor_id} Write position failed")
            except Exception as e:
                print(f"ID: {motor_id} encountered an error during write position:{e}")

    def quintic_polynomial_trajectory(self, t, T, p0, pT, v0=0, vT=0, a0=0, aT=0):
        A = np.array([[0, 0, 0, 0, 0, 1],
                      [T**5, T**4, T**3, T**2, T, 1],
                      [0, 0, 0, 0, 1, 0],
                      [5*T**4, 4*T**3, 3*T**2, 2*T, 1, 0],
                      [0, 0, 0, 2, 0, 0],
                      [20*T**3, 12*T**2, 6*T, 2, 0, 0]])

        B = np.array([p0, pT, v0, vT, a0, aT])
        coeffs = np.linalg.solve(A, B)
        return np.polyval(coeffs, t)

    def s_curve_trajectory(self, initial_positions, target_positions, Hz, duration, num_points):
        time_steps = np.linspace(0, duration, num_points)
        all_trajectories = []

        for initial_pos, target_pos in zip(initial_positions, target_positions):
            trajectory = np.round([self.quintic_polynomial_trajectory(t, duration, initial_pos, target_pos) for t in time_steps]).astype(int)
            all_trajectories.append(trajectory)

        for i in range(len(all_trajectories[0])):
            positions = [all_trajectories[motor_idx][i] for motor_idx in range(len(all_trajectories))]
            self.write_tim_pos(positions)
            if Hz != 0:
                time.sleep(round(1 / Hz, 4))

    def write_vel(self, vel_value):
        for motor_id, vel in zip(self.ids, vel_value):
            try:
                response_array, error = self.write(motor_id, VELOCITY_MODE, vel)
                # print(f"{motor_id}\n{response_array}\n{error}")
                if len(response_array) == 10 and error == 0:
                    print(f"ID: {motor_id} Velocity: {vel}")
                else:
                    print(f"ID: {motor_id} Write velocity failed")
            except Exception as e:
                print(f"ID: {motor_id} encountered an error during write velocity:{e}")

    def write_cur(self, cur_value):
        for motor_id, cur in zip(self.ids, cur_value):
            try:
                response_array, error = self.write(motor_id, CURRENT_MODE, cur)
                # print(f"{motor_id}\n{response_array}\n{error}")
                if len(response_array) == 10 and error == 0:
                    print(f"ID: {motor_id} Current: {cur}")
                else:
                    print(f"ID: {motor_id} Write current failed")
            except Exception as e:
                print(f"ID: {motor_id} encountered an error during write current:{e}")

    def Change_Parameters(self, address, value):
        if address not in CHANGE_PARAMETERS:
            print(f"Please enter a valid address")
            return
        for motor_id, val in zip(self.ids, value):
            try:
                response_array, error = self.write(motor_id, address, val)
                # print(f"{motor_id}\n{response_array}\n{error}")
                if len(response_array) == 10 and error == 0:
                    print(f"ID: {motor_id} {PARAMETERS_NAME[address]}: {val}")
                else:
                    print(f"ID: {motor_id} Modify failed")
            except Exception as e:
                print(f"ID: {motor_id} encountered an error during Modify:{e}")

    def Stop(self):
        for motor_id in self.ids:
            try:
                response_array, error = self.write(motor_id, MOTOR_STOP, 1)
                # print(f"{motor_id}\n{response_array}\n{error}")
                if len(response_array) == 10 and error == 0:
                    print(f"ID: {motor_id} Stop")
                else:
                    print(f"ID: {motor_id} Stop failed")
            except Exception as e:
                print(f"ID: {motor_id} encountered an error during Stop:{e}")

    def Clear(self):
        for motor_id in self.ids:
            try:
                response_array, error = self.write(motor_id, ERROR_CODE, 0)
                # print(f"{motor_id}\n{response_array}\n{error}")
                if len(response_array) == 10 and error == 0:
                    print(f"ID: {motor_id} Clear")
                else:
                    print(f"ID: {motor_id} Clear failed")
            except Exception as e:
                print(f"ID: {motor_id} encountered an error during Clear:{e}")
    
    def Set_Origin(self):
        for motor_id in self.ids:
            try:
                response_array, error = self.write(motor_id, SET_ORIGIN, 1)
                # print(f"{motor_id}\n{response_array}\n{error}")
                if len(response_array) == 10 and error == 0:
                    print(f"ID: {motor_id} Set origin")
                else:
                    print(f"ID: {motor_id} Set origin failed")
            except Exception as e:
                print(f"ID: {motor_id} encountered an error during Set origin:{e}")

    def Return_Origin(self):
        for motor_id in self.ids:
            try:
                response_array, error = self.write(motor_id, RETURN_ORIGIN, 1)
                # print(f"{motor_id}\n{response_array}\n{error}")
                if len(response_array) == 10 and error == 0:
                    print(f"ID: {motor_id} Return to origin")
                else:
                    print(f"ID: {motor_id} Return to origin failed")
            except Exception as e:
                print(f"ID: {motor_id} encountered an error during Return to origin:{e}")
    
    def Save(self):
        for motor_id in self.ids:
            try:
                response_array, error = self.write(motor_id, MOTOR_SAVE, 1)
                # print(f"{motor_id}\n{response_array}\n{error}")
                if len(response_array) == 10 and error == 0:
                    print(f"ID: {motor_id} Save successfully")
                else:
                    print(f"ID: {motor_id} Save failed")
            except Exception as e:
                print(f"ID: {motor_id} encountered an error during Save:{e}")

    def read_values(self, flag):
        values = []
        for motor_id in self.ids:
            value = []
            for value_address in ALL_PARAMETERS:
                try:
                    read_data, error, response_array = self.read(motor_id, value_address)
                    if read_data != None and len(response_array) == 10 and error == 0:
                        value.append(read_data)
                        if flag:
                            print(f"{PARAMETERS_NAME[value_address]}: {read_data}")
                except Exception as e:
                    print(f"ID: {motor_id} Read failed: {e}")
                    continue
            values.append(value)
            if flag:
                print('')
        return values
    
    def Brake(self, value):
        for motor_id in self.ids:
            try:
                response_array, error = self.write(motor_id, BRAKE, value)
                # print(f"{motor_id}\n{response_array}\n{error}")
                if len(response_array) == 10 and error == 0:
                    if value == 0:
                        print(f"ID: {motor_id} Rrake")
                    else:
                        print(f"ID: {motor_id} Release the brake")
                else:
                    print(f"ID: {motor_id} Rrake failed")
            except Exception as e:
                print(f"ID: {motor_id} encountered an error during Rrake:{e}")