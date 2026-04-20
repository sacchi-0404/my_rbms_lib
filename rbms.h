#ifndef INCLUDED_RBMS_H
#define INCLUDED_RBMS_H

#include "mbed.h"
#include <vector>

enum MotorType {
    M2006 = 0,
    M3508 = 1
};

enum ControlMode {
    SPD_MODE,
    POS_MODE,
    TORQUE_MODE
};

class rbms {
public:
    rbms(CAN &can, MotorType* motor_type, int motor_num);

    void set_control_mode(int id, ControlMode mode);
    void set_target_speed(int id, int speed);
    void set_target_torque(int id, int torque);
    void set_target_angle(int id, float angle);

    void reset_angle(int id);

    void spd_control();     // スレッド開始
    int rbms_send();

    bool handle_message(const CANMessage &msg);

private:
    CAN &_can;
    MotorType* _motor_type;
    int _motor_num;

    // ===== PID =====
    float _kp, _ki, _kd;
    float _kp_p, _ki_p, _kd_p;

    struct PIDState {
        float prev_err;
        float integral;

        float pos_prev_err;
        float pos_integral;

        uint16_t last_raw_angle;
        float accumulated_angle;
        bool is_initialized;

        Timer timer;
    };

    PIDState _pid_states[8];

    ControlMode _control_modes[8];
    int _target_speeds[8];
    int _target_torques[8];
    float _target_angles[8];

    int _output_torques[8];

    CANMessage _msg_buffer[8];
    uint8_t _new_data_mask = 0;

    Mutex _data_mutex;
    EventFlags _event_flags;
    Thread _thread;

    CANMessage _tx_msg_low, _tx_msg_high;

    // ===== 内部関数 =====
    int get_motor_max(MotorType type);
    float get_gear_ratio(MotorType type);

    float pid_calculate(int id, float target, float current, float dt);
    float pos_pid_calculate(int id, float target, float current, float dt);

    void control_thread_entry();
    void parse_can_data(int id, const CANMessage &msg, short *rotation, short *speed);
};

#endif
