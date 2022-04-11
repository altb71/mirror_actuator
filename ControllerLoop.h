#include "mbed.h"
#include "EncoderCounter.h"
#include "EncoderCounterIndex.h"
#include "LinearCharacteristics.h"
#include "ThreadFlag.h"
#include "path_1d.h"
#include "PID_Cntrl.h"
#include "Mirror_Kinematic.h"
#include "data_structs.h"
#include "GPA.h"
#include "DataLogger.h"
#include "FastPWM.h"
#include "sensors_actuators.h"

#define IDENT_VEL_PLANT 10
#define VEL_CNTRL   20
#define IDENT_POS_PLANT 30
#define POS_CNTRL   40
#define CIRCLE   50



//extern AnalogIn i_act2;

// This is the loop class, it is not a controller at first hand, it guarantees a cyclic call
class ControllerLoop
{
public:
    ControllerLoop(Data_Xchange *,sensors_actuators *,Mirror_Kinematic *,float Ts);
    virtual     ~ControllerLoop();
    void start_loop(void);
    void init_controllers(void);
    void reset_pids(void);


private:
    void loop(void);
    void reference_loop(void);
    Thread thread;
    Ticker ticker;
    ThreadFlag threadFlag;
    Timer ti;
    float Ts;
    float i_des[2];
    void sendSignal();
    bool is_initialized;
    void find_index(void);
    PID_Cntrl v_cntrl[2];
    float Kv[2];
    float pos_cntrl(float);
    uint8_t controller_type;
    Data_Xchange *m_data;
    sensors_actuators *m_sa;
    Mirror_Kinematic *m_mk;
};
