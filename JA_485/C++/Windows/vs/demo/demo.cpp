#include "ja_motor_control.h"
#include "ja_multi_motor_control.h"
vector<uint8_t> ID = { 1, 2 };
string port_name = "COM6";
//MotorControl motor(port_name, 115200, 1);
MultiMotorControl motor(port_name, 115200, ID);

int main()
{
	motor.enable(1);
}