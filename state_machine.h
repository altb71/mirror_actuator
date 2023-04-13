#pragma once
#include "mbed.h"
#include "sensors_actuators.h"
#include "Controller_Loop.h"

#define INIT 1
#define REFERENCE 2
#define CONTROL 3
#define STATE_GPA 4


// This is the loop class, it is not a controller at first hand, it guarantees a cyclic call
class state_machine
{
public:
    state_machine(sensors_actuators *,Controller_Loop *,float Ts);
    virtual     ~state_machine();
    void start_loop(void);

private:
    void loop(void);
    uint8_t CS;             // the current state
    Thread thread;
    Ticker ticker;
    ThreadFlag threadFlag;
    Timer ti;
    float Ts;
    void sendSignal();
    sensors_actuators *m_sa;
    Controller_Loop *m_loop;
};
