#include <ja_motor_control.h>
#include <ja_multi_motor_control.h>
string port_name= "/dev/ttyUSB1";
// MotorControl motor(port_name, 115200, 1);
MultiMotorControl motors(port_name, 19200, {1, 2});

int main()
{

}