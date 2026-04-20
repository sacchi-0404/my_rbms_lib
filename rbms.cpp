#include "rbms.h"
#include <chrono>

using namespace std::chrono;

rbms::rbms(CAN &can, MotorType* type, int num)
    : _can(can), _type(type), _num(num)
{
    _can.frequency(1000000);
    _can.mode(CAN::Normal);

    for(int i=0;i<8;i++){
        last_raw[i] = 0;
        accumulated_deg[i] = 0;
        initialized[i] = false;
    }
}

int rbms::get_reduction(MotorType type){
    return (type == M3508) ? 19 : 36;
}

int rbms::get_motor_max(MotorType type){
    return (type == M3508) ? 16384 : 10000;
}

void rbms::can_read(){
    CANMessage msg;
    while(_can.read(msg)){
        int id = msg.id - 0x201;
        if(id >= 0 && id < _num){
            _mutex.lock();
            _msg_buf[id] = msg;
            _mutex.unlock();
        }
    }
}

float rbms::pid(float dt, float now, float target,
                float &prev, float &integ,
                float kp, float ki, float kd)
{
    float err = target - now;
    integ += (err + prev) * dt * 0.5f;
    float der = (err - prev) / dt;
    prev = err;

    return kp*err + ki*integ + kd*der;
}

int rbms::rbms_send(int* motor){
    CANMessage tx1, tx2;
    tx1.id = 0x200; tx1.len = 8;
    tx2.id = 0x1FF; tx2.len = 8;

    for(int i=0;i<8;i++){
        int val = motor[i];

        if(i < 4){
            tx1.data[i*2]   = val >> 8;
            tx1.data[i*2+1] = val & 0xFF;
        }else{
            tx2.data[(i-4)*2]   = val >> 8;
            tx2.data[(i-4)*2+1] = val & 0xFF;
        }
    }

    return (_can.write(tx1) && _can.write(tx2)) ? 1 : -1;
}

void rbms::angle_control(int* target_deg, int* motor){

    std::vector<float> prev_pos(_num,0), integ_pos(_num,0);
    std::vector<float> prev_spd(_num,0), integ_spd(_num,0);
    std::vector<Timer> tm(_num);

    for(int i=0;i<_num;i++) tm[i].start();

    while(true){
        for(int i=0;i<_num;i++){

            _mutex.lock();
            CANMessage msg = _msg_buf[i];
            _mutex.unlock();

            if(msg.id == 0x201 + i){

                int16_t raw = (msg.data[0] << 8) | msg.data[1];
                int16_t spd = (int16_t)((msg.data[2] << 8) | msg.data[3]);

                // ===== 初期化 =====
                if(!initialized[i]){
                    last_raw[i] = raw;
                    accumulated_deg[i] = 0;
                    initialized[i] = true;
                }

                // ===== 差分計算（wrap対応）=====
                int16_t diff = raw - last_raw[i];
                if(diff > 4096) diff -= 8192;
                else if(diff < -4096) diff += 8192;

                last_raw[i] = raw;

                // ===== モータ側角度加算 =====
                float motor_deg = diff * 360.0f / 8192.0f;

                // ===== 出力軸に変換（ここが統一の核）=====
                int reduction = get_reduction(_type[i]);
                accumulated_deg[i] += motor_deg / reduction;

                // ===== 時間 =====
                float dt = tm[i].read();
                tm[i].reset();

                // ===== 位置PID → 速度指令 =====
                float target = target_deg[i];
                float pos_out = pid(dt,
                                    accumulated_deg[i],
                                    target,
                                    prev_pos[i], integ_pos[i],
                                    5.0f, 0.0f, 0.2f);

                // ===== 制限 =====
                if(pos_out > 300) pos_out = 300;
                if(pos_out < -300) pos_out = -300;

                // ===== 速度PID =====
                float current_rpm = spd / (float)reduction;

                float target_internal = pos_out * reduction;

                float torque = pid(dt,
                                   spd,
                                   target_internal,
                                   prev_spd[i], integ_spd[i],
                                   20.0f, 5.0f, 0.0f);

                int limit = get_motor_max(_type[i]);
                if(torque > limit) torque = limit;
                if(torque < -limit) torque = -limit;

                motor[i] = (int)torque;
            }
        }

        ThisThread::sleep_for(2ms);
    }
}
