# Address
VERSION_INFORMATION                 =   '0'
BAUDRATE                            =   '1'
MOTOR_ID                            =   '2'
TEMPERATURE                         =   '3'
BRAKE                               =   '4'
SERVO_STATUS                        =   '10'
IN_POSITION                         =   '13'
MOTOR_VELOCITY                      =   '14'
OUT_POSITION                        =   '15'
DRIVE_CURRENT                       =   '19'
RUNNING_STATUS                      =   '1A'
ERROR_CODE                          =   '1B'
LIMIT_CURRENT                       =   '1E'
POSLOOP_KP                          =   '20'
POSLOOP_KI                          =   '21'
POSLOOP_KD                          =   '22'
ACCELERATION                        =   '27'
DECELERATION                        =   '28'
MOTOR_SAVE                          =   '2D'
TARGET_VELOCITY                     =   '2E'
VELOCITY_MODE                       =   '2F'
CURRENT_MODE                        =   '30'
SET_ORIGIN                          =   '31'
RETURN_ORIGIN                       =   '32'
MOTOR_STOP                          =   '33'
TIM_POSITION_MODE                   =   '81'
VEL_POSITION_MODE                   =   '82'

ID                                  =   0
FUNCTION                            =   1
ADDRESS                             =   2
DATA_CONTENT                        =   4
DATA_LENGTH                         =   4
CRC                                 =   8
READ                                =   3
WRITE                               =   6

COMMUNICATION_ABNORMALITY           =   1
VALUE_ERROR                         =   2
INDEX_ERROR                         =   3
TIME_OUT                            =   4

CHANGE_PARAMETERS = [MOTOR_ID, 
                     BAUDRATE, 
                     LIMIT_CURRENT, 
                     POSLOOP_KP, 
                     POSLOOP_KI, 
                     POSLOOP_KD, 
                     ACCELERATION, 
                     DECELERATION, 
                     TARGET_VELOCITY]

ALL_PARAMETERS = [MOTOR_ID, 
                  BAUDRATE, 
                  TEMPERATURE, 
                  SERVO_STATUS, 
                  IN_POSITION, 
                  MOTOR_VELOCITY, 
                  OUT_POSITION, 
                  DRIVE_CURRENT, 
                  RUNNING_STATUS, 
                  ERROR_CODE,
                  LIMIT_CURRENT, 
                  POSLOOP_KP, 
                  POSLOOP_KI, 
                  POSLOOP_KD, 
                  ACCELERATION, 
                  DECELERATION, 
                  TARGET_VELOCITY, 
                  VELOCITY_MODE, 
                  CURRENT_MODE]

PARAMETERS_NAME = {  MOTOR_ID: 'ID',
                    BAUDRATE: 'Baudrate',
                    TEMPERATURE: 'Temperature',
                    SERVO_STATUS: 'Servo Status',
                    IN_POSITION: 'Input Position',
                    MOTOR_VELOCITY: 'Motor Velocity',
                    OUT_POSITION: 'Output Position',
                    DRIVE_CURRENT: 'Drive Current',
                    RUNNING_STATUS: 'Running Status',
                    ERROR_CODE: 'Error',
                    LIMIT_CURRENT: 'Limit Current',
                    POSLOOP_KP: 'Position Loop Kp',
                    POSLOOP_KI: 'Position Loop Ki',
                    POSLOOP_KD: 'Position Loop Kd',
                    ACCELERATION: 'Acceleration',
                    DECELERATION: 'Deceleration',
                    TARGET_VELOCITY: 'Target Velocity',
                    VELOCITY_MODE: 'Velocity Mode Value',
                    CURRENT_MODE: 'Current Mode Value'}