#ifndef INCLUDED_RBMS_H
#define INCLUDED_RBMS_H

#include "mbed.h"
#include <vector>

enum MotorType {
    M2006,
    M3508
};

class rbms {
public:
    rbms(CAN &can, MotorType* type, int num);

    void can_read();
    int rbms_send(int* motor);

    void angle_control(int* target_deg, int* motor);

private:
    int get_reduction(MotorType type);
    int get_motor_max(MotorType type);

    float pid(float dt, float now, float target,
              float &prev, float &integ,
              float kp, float ki, float kd);

private:
    CAN &_can;
    MotorType* _type;
    int _num;

    CANMessage _msg_buf[8];
    Mutex _mutex;

    // 角度管理
    int16_t last_raw[8];
    float accumulated_deg[8];
    bool initialized[8];
};

#endif
