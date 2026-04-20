#include "rbms.h"

rbms::rbms(CAN &can, MotorType* motor_type, int motor_num)
    : _can(can), _motor_type(motor_type), _motor_num(motor_num) {

    _can.frequency(1000000);
    _can.mode(CAN::Normal);

    for (int i = 0; i < _motor_num; i++) {
        _control_modes[i] = SPD_MODE;
        _target_speeds[i] = 0;
        _target_torques[i] = 0;
        _target_angles[i] = 0;

        _output_torques[i] = 0;

        _pid_states[i].prev_err = 0;
        _pid_states[i].integral = 0;
        _pid_states[i].pos_prev_err = 0;
        _pid_states[i].pos_integral = 0;
        _pid_states[i].last_raw_angle = 0;
        _pid_states[i].accumulated_angle = 0;
        _pid_states[i].is_initialized = false;

        _pid_states[i].timer.start();
    }
}

int rbms::get_motor_max(MotorType type) {
    return (type == M3508) ? 16384 : 10000;
}

float rbms::get_gear_ratio(MotorType type) {
    return (type == M3508) ? 19.0f : 36.0f;
}

void rbms::set_control_mode(int id, ControlMode mode) {
    if (id >= _motor_num) return;
    _control_modes[id] = mode;

    _pid_states[id].integral = 0;
    _pid_states[id].prev_err = 0;
    _pid_states[id].pos_integral = 0;
    _pid_states[id].pos_prev_err = 0;
}

void rbms::set_target_speed(int id, int speed) {
    if (id < _motor_num) _target_speeds[id] = speed;
}

void rbms::set_target_torque(int id, int torque) {
    if (id < _motor_num) _target_torques[id] = torque;
}

void rbms::set_target_angle(int id, float angle) {
    if (id < _motor_num) _target_angles[id] = angle;
}

void rbms::reset_angle(int id) {
    if (id >= _motor_num) return;
    _pid_states[id].accumulated_angle = 0;
}

float rbms::pid_calculate(int id, float target, float current, float dt) {
    float kp = (_motor_type[id] == M3508) ? 35.0f : 15.0f;
    float ki = (_motor_type[id] == M3508) ? 50.0f : 12.0f;

    float err = target - current;

    _pid_states[id].integral += err * dt;

    float out = kp * err + ki * _pid_states[id].integral;

    _pid_states[id].prev_err = err;
    return out;
}

float rbms::pos_pid_calculate(int id, float target, float current, float dt) {
    float kp = (_motor_type[id] == M3508) ? 5.0f : 7.0f;
    float kd = (_motor_type[id] == M3508) ? 0.15f : 0.3f;

    float err = target - current;
    float d = (err - _pid_states[id].pos_prev_err) / dt;

    float out = kp * err + kd * d;

    _pid_states[id].pos_prev_err = err;
    return out;
}

void rbms::spd_control() {
    _thread.start(callback(this, &rbms::control_thread_entry));
}

void rbms::control_thread_entry() {
    while (true) {
        _event_flags.wait_any(0x01);

        for (int id = 0; id < _motor_num; id++) {

            if (!(_new_data_mask & (1 << id))) continue;

            CANMessage msg = _msg_buffer[id];
            _new_data_mask &= ~(1 << id);

            short rot, spd;
            parse_can_data(id, msg, &rot, &spd);

            float dt = _pid_states[id].timer.read();
            _pid_states[id].timer.reset();

            float rpm = spd / get_gear_ratio(_motor_type[id]);

            int out = 0;

            if (_control_modes[id] == SPD_MODE) {
                out = pid_calculate(id, _target_speeds[id], rpm, dt);
            }
            else if (_control_modes[id] == POS_MODE) {
                float target_rpm = pos_pid_calculate(
                    id,
                    _target_angles[id],
                    _pid_states[id].accumulated_angle,
                    dt
                );
                out = pid_calculate(id, target_rpm, rpm, dt);
            }
            else {
                out = _target_torques[id];
            }

            int limit = get_motor_max(_motor_type[id]);
            if (out > limit) out = limit;
            if (out < -limit) out = -limit;

            _output_torques[id] = out;
        }
    }
}

int rbms::rbms_send() {
    _tx_msg_low.id = 0x200;
    _tx_msg_high.id = 0x1FF;

    for (int i = 0; i < 8; i++) {
        int val = _output_torques[i];

        if (i < 4) {
            _tx_msg_low.data[i*2] = val >> 8;
            _tx_msg_low.data[i*2+1] = val & 0xFF;
        } else {
            _tx_msg_high.data[(i-4)*2] = val >> 8;
            _tx_msg_high.data[(i-4)*2+1] = val & 0xFF;
        }
    }

    return (_can.write(_tx_msg_low) &&
           (_motor_num > 4 ? _can.write(_tx_msg_high) : true)) ? 1 : -1;
}

void rbms::parse_can_data(int id, const CANMessage &msg, short *rotation, short *speed) {
    uint16_t raw = (msg.data[0] << 8) | msg.data[1];
    *speed = (int16_t)((msg.data[2] << 8) | msg.data[3]);

    if (!_pid_states[id].is_initialized) {
        _pid_states[id].last_raw_angle = raw;
        _pid_states[id].is_initialized = true;
    }

    int diff = raw - _pid_states[id].last_raw_angle;

    if (diff > 4096) diff -= 8192;
    if (diff < -4096) diff += 8192;

    _pid_states[id].last_raw_angle = raw;

    float ratio = get_gear_ratio(_motor_type[id]);
    _pid_states[id].accumulated_angle += (diff / 8192.0f) * 360.0f / ratio;

    *rotation = raw;
}

bool rbms::handle_message(const CANMessage &msg) {
    int id = msg.id - 0x201;

    if (id >= 0 && id < _motor_num) {
        _msg_buffer[id] = msg;
        _new_data_mask |= (1 << id);
        _event_flags.set(0x01);
        return true;
    }
    return false;
}
