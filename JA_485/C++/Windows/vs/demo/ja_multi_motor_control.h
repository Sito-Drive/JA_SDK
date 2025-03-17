#ifndef MULTI_MOTOR_CONTROL_H
#define MULTI_MOTOR_CONTROL_H

#include <vector>
#include <memory>
#include <unordered_map>
#include <stdexcept>
#include "ja_motor_control.h"

class MultiMotorControl: public MotorControl
{
public:
    MultiMotorControl(const string& port, const uint32_t& baudrate, const vector<uint8_t>& motor_ids);
    // ~MultiMotorControl();
    void closeport();
    void set_object_id(const vector<uint8_t>& new_id);
    void enable(const bool& value);
    void write_position(const vector<int32_t>& pos_value);
    void write_velocity(const vector<int16_t>& vel_value);
    void write_current(const vector<int16_t>& cur_value);
    void change_parameters(const SETTINGS address, const vector<int16_t>& value);
    void stop();
    void clear();
    void set_origin();
    void return_origin();
    void save();
    vector<MotorStatus> read(const bool& flag = 0);
    void brake(const bool& flag);
    void write_time_position(const vector<int>& initial_positions, const vector<int>& target_positions, const double& Hz = 100, const double& duration = 20, const int& num_points = 15000);
private:
    vector<uint8_t> motor_ids_;

    template <typename T>
    void validate_param_size(const vector<T>& values, const string& param_name)
    {
        if (values.size() != motor_ids_.size() && values.size() != 1)
        {
            throw invalid_argument(param_name + " size must match the number of motors or be 1");
        }
    }
};

#endif