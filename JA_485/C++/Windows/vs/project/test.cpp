#include <ja_motor_control.h>
string port_name= "COM6";
 MotorControl motor(port_name, 115200, 1);

int main()
{
    //MotorStatus status = motor.read(1);
    //Sleep(1000);
    //cout << "MOTOR_ID: " << status.motor_id << " ";
    //cout << "BAUDRATE: " << status.baudrate << " ";
    //cout << "TEMPERATURE: " << status.temperature << " ";
    //cout << "SERVO_STATUS: " << status.servo_status << " ";
    //cout << "IN_POSITION: " << status.input_position << " ";
    //cout << "MOTOR_VELOCITY: " << status.motor_velocity << " ";
    //cout << "OUT_POSITION: " << status.output_position << " ";
    //cout << "DRIVE_CURRENT: " << status.drive_current << " ";
    //cout << "RUNNING_STATUS: " << status.running_status << " ";
    //cout << "ERRIR_CODE: " << status.error_code << " ";
    //cout << "LIMIT_CURRENT: " << status.limit_current << " ";
    //cout << "POSLOOP_KP: " << status.posloop_kp << " ";
    //cout << "POSLOOP_KI: " << status.posloop_ki << " ";
    //cout << "POSLOOP_KD: " << status.posloop_kd << " ";
    //cout << "ACCELERATION: " << status.acceleration << " ";
    //cout << "DECELERATION: " << status.deceleration << " ";
    //cout << "TARGET_VELOCITY" << status.target_velocity << " ";
    //cout << "VELOCITY_MODE: " << status.velocity_mode << " ";
    //cout << "CURRENT_MODE: " << status.current_mode << endl;
    //cout << endl;
}