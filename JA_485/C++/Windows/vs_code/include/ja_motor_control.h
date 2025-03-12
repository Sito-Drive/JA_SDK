#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <iostream>
#include <iomanip>
#ifdef _WIN32
    #include <windows.h>
#else
    #include <unistd.h>
#endif
#include <vector>
#include <string>
#include <chrono>
#include <thread>
#include <iostream>
#include <stdexcept>
#include <serial/serial.h>
#define sleep_ms(x) this_thread::sleep_for(chrono::milliseconds(x));
using namespace std;

enum class Mode 
{
    READ                                =   3,
    WRITE                               =   6,
    VERSION_INFORMATION                 =   0x00,
    BAUDRATE                            =   0x01,
    MOTOR_ID                            =   0x02,
    TEMPERATURE                         =   0x03,
    BRAKE                               =   0x04,
    SERVO_STATUS                        =   0x10,
    IN_POSITION                         =   0x13,
    MOTOR_VELOCITY                      =   0x14,
    OUT_POSITION                        =   0x15,
    DRIVE_CURRENT                       =   0x19,
    RUNNING_STATUS                      =   0x1A,
    ERROR_CODE                          =   0x1B,
    LIMIT_CURRENT                       =   0x1E,
    POSLOOP_KP                          =   0x20,
    POSLOOP_KI                          =   0x21,
    POSLOOP_KD                          =   0x22,
    ACCELERATION                        =   0x27,
    DECELERATION                        =   0x28,
    MOTOR_SAVE                          =   0x2D,
    TARGET_VELOCITY                     =   0x2E,
    VELOCITY_MODE                       =   0x2F,
    CURRENT_MODE                        =   0x30,
    SET_ORIGIN                          =   0x31,
    RETURN_ORIGIN                       =   0x32,
    MOTOR_STOP                          =   0x33,
    TIM_POSITION_MODE                   =   0x81,
    VEL_POSITION_MODE                   =   0x82
};

enum class SETTINGS 
{
    BAUDRATE                            =   0x01,
    MOTOR_ID                            =   0x02,
    LIMIT_CURRENT                       =   0x1E,
    POSLOOP_KP                          =   0x20,
    POSLOOP_KI                          =   0x21,
    POSLOOP_KD                          =   0x22,
    ACCELERATION                        =   0x27,
    DECELERATION                        =   0x28,
    MOTOR_SAVE                          =   0x2D,
    TARGET_VELOCITY                     =   0x2E,
};

struct MotorStatus
{
    int16_t motor_id = 0;
    int16_t baudrate = 0;
    int16_t temperature = 0;
    bool servo_status = 0;
    int32_t input_position = 0;
    int16_t motor_velocity = 0;
    int32_t output_position = 0;
    int16_t drive_current = 0;
    bool running_status = 0;
    int16_t error_code = 0;
    int16_t limit_current = 0;
    int16_t posloop_kp = 0;
    int16_t posloop_ki = 0;
    int16_t posloop_kd = 0;
    int16_t acceleration = 0;
    int16_t deceleration = 0;
    int16_t target_velocity = 0;
    int16_t velocity_mode = 0;
    int16_t current_mode = 0;
};

class MotorControl 
{
public:
    MotorControl(const string& port, const uint32_t& baudrate, const uint8_t& motor_id);
    // ~MotorControl();
    void closeport();
    void set_object_id(const uint8_t new_id);
    void enable(const bool& value);
    void write_position(const int32_t& pos_value);
    void write_velocity(const int16_t& vel_value);
    void write_current(const int16_t& cur_value);
    void change_parameters(const SETTINGS address, const int16_t& value);
    void stop();
    void clear();
    void set_origin();
    void return_origin();
    void save();
    MotorStatus read(const bool& flag = 0);
    void brake(const bool& flag);
    void tim_position(const int32_t& pos_value);
    static vector<double> linear_system(vector<vector<double>> A, vector<double> B);
    static double tra_point(double t, double T, double p0, double pT, double v0 = 0, double vT = 0, double a0 = 0, double aT = 0);
    static vector<int32_t> trajectory(int initial_positions, int target_positions, double duration, int num_points);
    void write_time_position(int initial_positions, int target_positions, double Hz, double duration, int num_points);

protected:
    serial::Serial& getSerial() { return ser; }
    vector <uint8_t> buildPacket(const uint8_t& id, const uint16_t& address, const int32_t& data, const uint8_t& function = static_cast<uint8_t>(Mode::WRITE))
    {
        vector <uint8_t> pack(10, 0);
        pack.at(0) = id;
        pack.at(1) = function;
        pack.at(2) = (address >> 8) & 0xFF;
        pack.at(3) = address & 0xFF;
        pack.at(4) = (data >> 24) & 0xFF;
        pack.at(5) = (data >> 16) & 0xFF;
        pack.at(6) = (data >> 8) & 0xFF;
        pack.at(7) = data & 0xFF;
        vector <uint8_t> prepare_crc(pack.begin(), pack.begin() + 8);
        uint16_t crc = CRC16(prepare_crc);
        pack.at(8) = (crc >> 8) & 0xFF;
        pack.at(9) = crc & 0xFF;

        //for (int i : pack)
        //{
        //    cout << setw(2) << setfill('0') << uppercase << hex << i << " ";
        //}
        //cout << endl;
        return pack;
    }

private:
    uint8_t motor_id;
    serial::Serial ser;

    uint16_t CRC16(const vector<uint8_t>& buffer) 
    {
        uint16_t temp = 0xFFFF;
        for (uint8_t byte : buffer) 
        {
            temp ^= byte;
            for (int i = 0; i < 8; ++i) 
            {
                if ((temp & 0x01) == 0) 
                {
                    temp >>= 1;
                } 
                else 
                {
                    temp >>= 1;
                    temp ^= 0xA001;
                }
            }
        }
        return temp;
    }

    void send_data(const uint8_t* data, size_t size)
    {
        size_t num_send = ser.write(data, size);
        // this_thread::sleep_for(chrono::milliseconds(10));
        // sleep_ms(10);
        if (num_send != size)
        {
            cout<< "Data send failed. bytes be send: " << num_send << endl;
        }
    }

    vector<uint8_t> receive_data(size_t expected_size, uint16_t timeout_ms = 1000)
    {
        vector<uint8_t> buffer(expected_size);
        size_t total_read = 0;
        auto start_time = chrono::steady_clock::now();

        while (total_read < expected_size) 
        {
            auto current_time = chrono::steady_clock::now();
            auto elapsed_time = chrono::duration_cast<chrono::milliseconds>(current_time - start_time).count();
            if (elapsed_time >= timeout_ms) 
            {
                throw runtime_error("Receive data timeout");
            }

            size_t num_read = ser.read(buffer.data() + total_read, expected_size - total_read);
            if (num_read == 0) 
            {
                this_thread::sleep_for(chrono::milliseconds(10));
                continue;
            }

            total_read += num_read;
        }

        return buffer;
    }

};
#endif
