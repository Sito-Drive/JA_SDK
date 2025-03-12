#include <ja_multi_motor_control.h>
#include <math.h>

MultiMotorControl::MultiMotorControl(const string& port, const uint32_t& baudrate, const vector<uint8_t>& motor_ids)
:MotorControl(port, baudrate, motor_ids[0]), motor_ids_(motor_ids)
{}

void MultiMotorControl::closeport()
{
    MotorControl:: closeport();
}

void MultiMotorControl::set_object_id(const vector<uint8_t>& new_id)
{
    motor_ids_ = new_id;
}

void MultiMotorControl:: enable(const bool& value)
{
    for (uint8_t id: motor_ids_)
    {
        MotorControl:: set_object_id(id);
        MotorControl:: enable(value);
        this_thread::sleep_for(chrono::microseconds(1));
    }
}

void MultiMotorControl:: write_position(const vector<int32_t>& pos_value)
{
    validate_param_size(pos_value, "Position_Values");
    for (size_t i = 0; i < motor_ids_.size(); ++i)
    {
        uint8_t id = motor_ids_[i];
        int32_t Pos_value = pos_value.size() == 1 ? pos_value[0] : pos_value[i];
        MotorControl:: set_object_id(id);
        MotorControl:: write_position(Pos_value);
        this_thread::sleep_for(chrono::microseconds(1));
    }
}

void MultiMotorControl:: write_velocity(const vector<int16_t>& vel_value)
{
    validate_param_size(vel_value, "Velocity_Values");
    for (size_t i = 0; i < motor_ids_.size(); ++i)
    {
        uint8_t id = motor_ids_[i];
        int32_t Vel_value = vel_value.size() == 1 ? vel_value[0] : vel_value[i];
        MotorControl:: set_object_id(id);
        MotorControl:: write_velocity(Vel_value);
        this_thread::sleep_for(chrono::microseconds(1));
    }
}

void MultiMotorControl:: write_current(const vector<int16_t>& cur_value)
{
    validate_param_size(cur_value, "Current_Values");
    for (size_t i = 0; i < motor_ids_.size(); ++i)
    {
        uint8_t id = motor_ids_[i];
        int32_t Cur_value = cur_value.size() == 1 ? cur_value[0] : cur_value[i];
        MotorControl:: set_object_id(id);
        MotorControl:: write_current(Cur_value);
        this_thread::sleep_for(chrono::microseconds(1));
    }
}

void MultiMotorControl:: change_parameters(const SETTINGS address, const vector<int16_t>& value)
{
    validate_param_size(value, "Values");
    for (size_t i = 0; i < motor_ids_.size(); ++i)
    {
        uint8_t id = motor_ids_[i];
        int32_t Value = value.size() == 1 ? value[0] : value[i];
        MotorControl:: set_object_id(id);
        MotorControl:: change_parameters(address, Value);
        this_thread::sleep_for(chrono::microseconds(1));
    }
}

void MultiMotorControl:: stop()
{
    for (size_t i = 0; i < motor_ids_.size(); ++i)
    {
        uint8_t id = motor_ids_[i];
        MotorControl:: set_object_id(id);
        MotorControl:: stop();
        this_thread::sleep_for(chrono::microseconds(1));
    }
}

void MultiMotorControl:: clear()
{
    for (size_t i = 0; i < motor_ids_.size(); ++i)
    {
        uint8_t id = motor_ids_[i];
        MotorControl:: set_object_id(id);
        MotorControl:: clear();
        this_thread::sleep_for(chrono::microseconds(1));
    }
}

void MultiMotorControl:: set_origin()
{
    for (size_t i = 0; i < motor_ids_.size(); ++i)
    {
        uint8_t id = motor_ids_[i];
        MotorControl:: set_object_id(id);
        MotorControl:: set_origin();
        this_thread::sleep_for(chrono::microseconds(1));
    }
}

void MultiMotorControl:: return_origin()
{
    for (size_t i = 0; i < motor_ids_.size(); ++i)
    {
        uint8_t id = motor_ids_[i];
        MotorControl:: set_object_id(id);
        MotorControl:: return_origin();
        this_thread::sleep_for(chrono::microseconds(1));
    }
}

void MultiMotorControl:: save()
{
    for (size_t i = 0; i < motor_ids_.size(); ++i)
    {
        uint8_t id = motor_ids_[i];
        MotorControl:: set_object_id(id);
        MotorControl:: save();
        this_thread::sleep_for(chrono::microseconds(1));
    }
}

vector<MotorStatus> MultiMotorControl:: read(const bool& flag)
{
    vector<MotorStatus> status;
    for (size_t i = 0; i < motor_ids_.size(); i++)
    {
        uint8_t id = motor_ids_[i];
        MotorControl:: set_object_id(id);
        status.push_back(MotorControl:: read(flag));
        this_thread::sleep_for(chrono::microseconds(1));
    }
    return status;
}

void MultiMotorControl:: brake(const bool& flag)
{
    for (size_t i = 0; i < motor_ids_.size(); ++i)
    {
        uint8_t id = motor_ids_[i];
        MotorControl:: set_object_id(id);
        MotorControl:: brake(flag);
        this_thread::sleep_for(chrono::microseconds(1));
    }
}

void MultiMotorControl:: write_time_position(const vector<int>& initial_positions, const vector<int>& target_positions, const double& Hz, const double& duration, const int& num_points)
{
    vector<vector<int32_t>> positions;
    vector<int32_t> pos;
    for (size_t i = 0; i < motor_ids_.size(); ++i)
    {
        int32_t initial_position = initial_positions.size() == 1 ? initial_positions[0] : initial_positions[i];
        int32_t target_position = target_positions.size() == 1 ? target_positions[0] : target_positions[i];
        pos = trajectory(initial_position, target_positions[i], Hz, duration, num_points);
        positions.push_back(pos);
    }

    for (size_t i = 0; i < positions[0].size(); i++)
    {
        for (size_t j = 0; j < motor_ids_.size(); ++j)
        {            
            uint8_t id = motor_ids_[j];
            MotorControl:: set_object_id(id);
            MotorControl:: tim_position(positions[j][i]);
        }
        if (Hz != 0)
        {
            sleep_ms(static_cast<int>(1000 / Hz));
        }
    }

}