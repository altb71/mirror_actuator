#pragma once
#include "mbed.h"
#include "EncoderCounter.h"
#include "EncoderCounterIndex.h"
#include "LinearCharacteristics.h"
#include "ThreadFlag.h"
#include "PID_Cntrl.h"
#include "Mirror_Kinematic.h"
#include "data_structs.h"
#include "GPA.h"
#include "DataLogger.h"
#include "FastPWM.h"
#include "sensors_actuators.h"

#define CNTRL_IDLE 0
#define FIND_INDEX 1
#define GPA_IDENT_PLANT 10
#define CNTRL_VEL 20

// This is the loop class, it is not a controller at first hand, it guarantees a cyclic call
class Controller_Loop
{
public:
    Controller_Loop(Data_Xchange *,sensors_actuators *,Mirror_Kinematic *,float Ts);
    virtual ~Controller_Loop();
    void start_loop(void);
    void init_controllers(void);
    void reset_pids(void);
    void switch_to_find_index(void);
    void switch_to_GPA_ident(void);
    void switch_to_cntrl_vel(void);

private:
    void loop(void);
    Thread thread;
    Ticker ticker;
    ThreadFlag threadFlag;
    Timer ti;
    float Ts;
    void sendSignal();
    bool is_initialized;
    void find_index(void);
    PID_Cntrl v_cntrl_1, v_cntrl_2;
    Data_Xchange *m_data;
    sensors_actuators *m_sa;
    Mirror_Kinematic *m_mk;
    uint8_t controller_state;
};
