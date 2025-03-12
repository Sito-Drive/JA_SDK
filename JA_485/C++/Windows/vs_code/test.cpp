#include <ja_motor_control.h>
string port_name = "COM6";                    // Windows: "COM*"  Linux: "/dev/ttyUSB*"
MotorControl motor(port_name, 115200, 1);   // Params: serial port path (string), baud rate (uint32_t), actuator id

int main()
{
    motor.enable(1);
}