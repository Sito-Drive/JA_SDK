#include <ja_motor_control.h>
#include <math.h>

vector<Mode> ALL_PARAMETERS = { Mode:: MOTOR_ID, 
                                Mode:: BAUDRATE, 
                                Mode:: TEMPERATURE, 
                                Mode:: SERVO_STATUS, 
                                Mode:: IN_POSITION, 
                                Mode:: MOTOR_VELOCITY, 
                                Mode:: OUT_POSITION, 
                                Mode:: DRIVE_CURRENT, 
                                Mode:: RUNNING_STATUS, 
                                Mode:: ERROR_CODE,
                                Mode:: LIMIT_CURRENT, 
                                Mode:: POSLOOP_KP, 
                                Mode:: POSLOOP_KI, 
                                Mode:: POSLOOP_KD, 
                                Mode:: ACCELERATION, 
                                Mode:: DECELERATION, 
                                Mode:: TARGET_VELOCITY, 
                                Mode:: VELOCITY_MODE, 
                                Mode:: CURRENT_MODE};
static const char* getModeName(Mode mode)
{
    switch (mode) 
    {
        case Mode::MOTOR_ID:            return "MOTOR_ID: ";
        case Mode::BAUDRATE:            return "BAUDRATE: ";
        case Mode::TEMPERATURE:         return "TEMPERATURE: ";
        case Mode::SERVO_STATUS:        return "SERVO_STATUS: ";
        case Mode::IN_POSITION:         return "IN_POSITION: ";
        case Mode::MOTOR_VELOCITY:      return "MOTOR_VELOCITY: ";
        case Mode::OUT_POSITION:        return "OUT_POSITION: ";
        case Mode::DRIVE_CURRENT:       return "DRIVE_CURRENT: ";
        case Mode::RUNNING_STATUS:      return "RUNNING_STATUS: ";
        case Mode::ERROR_CODE:          return "ERROR_CODE: ";
        case Mode::LIMIT_CURRENT:       return "LIMIT_CURRENT: ";
        case Mode::POSLOOP_KP:          return "POSLOOP_KP: ";
        case Mode::POSLOOP_KI:          return "POSLOOP_KI: ";
        case Mode::POSLOOP_KD:          return "POSLOOP_KD: ";
        case Mode::ACCELERATION:        return "ACCELERATION: ";
        case Mode::DECELERATION:        return "DECELERATION: ";
        case Mode::TARGET_VELOCITY:     return "TARGET_VELOCITY: ";
        case Mode::VELOCITY_MODE:       return "VELOCITY_MODE: ";
        case Mode::CURRENT_MODE:        return "CURRENT_MODE: ";
        default:                        return "UNKNOWN";
    }
}

MotorControl::MotorControl(const string& port, const uint32_t& baudrate, const uint8_t& motor_id)
:motor_id(motor_id)
{
    try
    {
        ser.setPort(port);
        ser.setBaudrate(baudrate);
        serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(timeout);
        if (ser.isOpen()) 
        {
            cout << port << " is opened" << endl;
        } 
        else
        {
            ser.open();
            cout << port << " open success" << endl;
        }
    }
    catch (exception& e) 
    {
        cout << "Error: " << e.what() << endl;
    }
}

void MotorControl::closeport()
{
    if (ser.isOpen())
    {
        ser.close();
        cout << "Serial port closed" << endl;
    }
    else 
    {
        cout << "Serial port was not open" << endl;
    }
}

void MotorControl:: set_object_id(const uint8_t new_id)
{
    motor_id = new_id;
}

void MotorControl:: enable(const bool& value)
{
    vector <uint8_t> data;
    data = buildPacket(motor_id, static_cast<uint8_t>(Mode::SERVO_STATUS), value);
    send_data(data.data(), data.size());   
}

void MotorControl:: write_position(const int32_t& pos_value)
{
    vector <uint8_t> data;
    data = buildPacket(motor_id, static_cast<uint8_t>(Mode::VEL_POSITION_MODE), pos_value);
    send_data(data.data(), data.size());
}

void MotorControl:: tim_position(const int32_t& pos_value)
{
    vector <uint8_t> data;
    data = buildPacket(motor_id, static_cast<uint8_t>(Mode::TIM_POSITION_MODE), pos_value);
    send_data(data.data(), data.size());
}

void MotorControl:: write_velocity(const int16_t& vel_value)
{
    vector <uint8_t> data;
    data = buildPacket(motor_id, static_cast<uint8_t>(Mode::VELOCITY_MODE), vel_value);
    send_data(data.data(), data.size());
}

void MotorControl:: write_current(const int16_t& cur_value)
{
    vector <uint8_t> data;
    data = buildPacket(motor_id, static_cast<uint8_t>(Mode::CURRENT_MODE), cur_value);
    send_data(data.data(), data.size());
}

void MotorControl:: change_parameters(const SETTINGS address, const int16_t& value)
{
    vector <uint8_t> data;
    data = buildPacket(motor_id, static_cast<uint8_t>(address), value);
    send_data(data.data(), data.size());
}

void MotorControl:: stop()
{
    vector <uint8_t> data;
    data = buildPacket(motor_id, static_cast<uint8_t>(Mode::MOTOR_STOP), 1);
    send_data(data.data(), data.size());
}

void MotorControl:: clear()
{
    vector <uint8_t> data;
    data = buildPacket(motor_id, static_cast<uint8_t>(Mode::ERROR_CODE), 0);
    send_data(data.data(), data.size());
}

void MotorControl:: set_origin()
{
    vector <uint8_t> data;
    data = buildPacket(motor_id, static_cast<uint8_t>(Mode::SET_ORIGIN), 1);
    send_data(data.data(), data.size());
}

void MotorControl:: return_origin()
{
    vector <uint8_t> data;
    data = buildPacket(motor_id, static_cast<uint8_t>(Mode::RETURN_ORIGIN), 1);
    send_data(data.data(), data.size());
}

void MotorControl:: save()
{
    vector <uint8_t> data;
    data = buildPacket(motor_id, static_cast<uint8_t>(Mode::MOTOR_SAVE), 1);
    send_data(data.data(), data.size());
}

MotorStatus MotorControl::read(const bool& flag) 
{
    vector<uint8_t> data;
    vector<int32_t> output_data;
    MotorStatus status;
    for (Mode i : ALL_PARAMETERS) 
    {
        clearBuffer();
        data = buildPacket(motor_id, static_cast<uint8_t>(i), 2, static_cast<uint8_t>(Mode::READ));
        send_data(data.data(), data.size());
        vector<uint8_t> received_data = receive_data(10);
        vector<uint8_t> op_data(received_data.begin() + 4, received_data.begin() + 8);
        uint32_t sum = 0;
        sum |= static_cast<uint32_t>(op_data[0]) << 24;
        sum |= static_cast<uint32_t>(op_data[1]) << 16;
        sum |= static_cast<uint32_t>(op_data[2]) << 8;
        sum |= static_cast<uint32_t>(op_data[3]);
        int32_t signed_sum = static_cast<int32_t>(sum);
        output_data.push_back(signed_sum);
        if (flag) 
        {
            cout << getModeName(i) << signed_sum << endl;;
        }
    }
    if (flag)
    {
        cout << endl;
    }
    status.motor_id = output_data.at(0);
    status.baudrate = output_data.at(1);
    status.temperature = output_data.at(2);
    status.servo_status = output_data.at(3);
    status.input_position = output_data.at(4);
    status.motor_velocity = output_data.at(5);
    status.output_position = output_data.at(6);
    status.drive_current = output_data.at(7);
    status.running_status = output_data.at(8);
    status.error_code = output_data.at(9);
    status.limit_current = output_data.at(10);
    status.posloop_kp = output_data.at(11);
    status.posloop_ki = output_data.at(12);
    status.posloop_kd = output_data.at(13);
    status.acceleration = output_data.at(14);
    status.deceleration = output_data.at(15);
    status.target_velocity = output_data.at(16);
    status.velocity_mode = output_data.at(17);
    status.current_mode = output_data.at(18);
    return status;
}

void MotorControl:: brake(const bool& flag)
{
    vector <uint8_t> data;
    data = buildPacket(motor_id, static_cast<uint8_t>(Mode::BRAKE), flag);
    send_data(data.data(), data.size());
}

vector<double> MotorControl::linear_system(vector<vector<double>> A, vector<double> B)
{
    const int x_size = 6;
    for (int i = 0; i < x_size; i++)
    {
        int max_row = i;
        for (int j = i + 1; j < x_size; j++)
        {
            if (fabs(A[j][i]) > fabs(A[max_row][i]))
            {
                max_row = j;
            }
        }
        swap(A[i], A[max_row]);
        swap(B[i], B[max_row]);

        double pivot = A[i][i];
        if (fabs(pivot) < 1e-12)
        {
            throw runtime_error("Matrix is singular or nearly singular");
        }
        for (int k = i; k < x_size; k++)
        {
            A[i][k] /= pivot;
        }
        B[i] /= pivot;

        for (int j = i + 1; j < x_size; j++)
        {
            double factor = A[j][i];
            for (int k = i; k < x_size; k++)
            {
                A[j][k] -= A[i][k] * factor;
            }
            B[j] -= B[i] * factor;
        }
    }

    vector<double> results(x_size, 0);
    for (int i = x_size - 1; i >= 0; i--)
    {
        double dividend = B[i];
        for (int j = i + 1; j < x_size; j++)
        {
            dividend -= A[i][j] * results[j];
        }
        results[i] = dividend / A[i][i];
    }
    return results;
}

double MotorControl:: tra_point(double t, double T, double p0, double pT, double v0, double vT, double a0, double aT)
{
    vector<vector<double>> A =
    {
        {0, 0, 0, 0, 0, 1},
        {pow(T, 5), pow(T, 4), pow(T, 3), pow(T, 2), T, 1},
        {0, 0, 0, 0, 1, 0},
        {5 * pow(T, 4), 4 * pow(T, 3), 3 * pow(T, 2), 2 * T, 1, 0},
        {0, 0, 0, 2, 0, 0},
        {20 * pow(T, 3), 12 * pow(T, 2), 6 * T, 2, 0, 0}
    };
    vector<double> B = {p0, pT, v0, vT, a0, aT};
    vector<double> coeffs = linear_system(A, B);

    double position = 0;
    for (int i = 0; i < 6; ++i)
    {
        position += coeffs[i] * pow(t, 5 - i);
    }
    return position;
}

vector<int32_t> MotorControl:: trajectory(int initial_positions, int target_positions, double duration, int num_points)
{
    vector<int32_t> trajectories(num_points);
    vector<double> time_steps(num_points);
    for (int i = 0; i < num_points; ++i)
    {
        time_steps[i] = duration * i / (num_points - 1);
    }
    
    for (int i = 0; i < num_points; ++i)
    {
        trajectories[i] = static_cast<int32_t>(round(tra_point(time_steps[i], duration, initial_positions, target_positions)));
    }
    return trajectories;
}

void MotorControl:: write_time_position(int initial_positions, int target_positions, double Hz, double duration, int num_points)
{
    vector<int32_t> positions = trajectory(initial_positions, target_positions, duration, num_points);
    for (int32_t pos: positions)
    {
        tim_position(pos);
        if (Hz != 0)
        {
            this_thread::sleep_for(chrono::milliseconds(static_cast<int>(1000 / Hz)));
        }
    }
}