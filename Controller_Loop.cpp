#include "Controller_Loop.h"
#include <cstdint>
using namespace std;

extern GPA myGPA;
extern DataLogger myDataLogger;

// contructor for controller loop
Controller_Loop::Controller_Loop(Data_Xchange *data,sensors_actuators *sa, Mirror_Kinematic *mk, float Ts) : thread(osPriorityHigh,4096)
{
    this->Ts = Ts;
    this->m_data = data;        // link to data
    this->m_sa = sa;            // link to hardware
    this->m_mk = mk;            // link to kinematics
    ti.reset();
    ti.start();
    controller_state = CNTRL_IDLE;  // the local state machine
    }
// decontructor for controller loop
Controller_Loop::~Controller_Loop() {}
// ----------------------------------------------------------------------------
// this is the main loop called every Ts with high priority
void Controller_Loop::loop(void){
    float i_des,v_des;
    uint8_t k = 0;
    while(1)
        {
        ThisThread::flags_wait_any(threadFlag);
        // THE LOOP ------------------------------------------------------------
        m_sa->read_encoders_calc_speed();       // first read encoders and calculate speed
        // -------------------------------------------------------------
        // at very beginning: move system slowly to find the zero pulse
        float ti_loc = ti.read();
        switch(controller_state)
            {
            case CNTRL_IDLE:
                i_des = 0;
                break;
            case FIND_INDEX:
                find_index();
                break;
            case GPA_IDENT_PLANT:
                m_sa->enable_motors(true);      // enable motors, still read the bigButton to enable
                i_des = myGPA.update(i_des,m_data->sens_Vphi[0]);
                break;
            case CNTRL_VEL:
                m_sa->enable_motors(true);      // enable motors
                // v_des = myDataLogger.get_set_value(ti_loc);
                // myDataLogger.write_to_log(ti_loc,data1, data2, data3);
            break;
            // ------------------------ do the control first
            // calculate desired currents here, you can do "anything" here, 
            // data from sensors etc. are available via the m_data->... structure 
            default:
                break;
            }
        m_sa->write_current(0,i_des);
        m_sa->write_current(1,0);       // set 2nd motor to 0A
        m_sa->set_laser_on_off(m_data->laser_on);
        if(++k>=10)     // kinematic transformation from angles to xy values.
            {
            m_mk->P2X(m_data->sens_phi,m_data->est_xy);
            k = 0;
            }
            
        }// endof the main loop
}

void Controller_Loop::sendSignal() {
    thread.flags_set(threadFlag);
}
void Controller_Loop::start_loop(void)
{
    thread.start(callback(this, &Controller_Loop::loop));
    ticker.attach(callback(this, &Controller_Loop::sendSignal), Ts);
}
// several public functions to allow the controller statemachine to switch 
// to other states from external.
void Controller_Loop::switch_to_find_index()
{
    controller_state = FIND_INDEX;
}
void Controller_Loop::switch_to_GPA_ident()
{
    controller_state = GPA_IDENT_PLANT;
}
void Controller_Loop::switch_to_cntrl_vel()
{
    controller_state = CNTRL_VEL;
}
void Controller_Loop::init_controllers(void)
{
    // set values for your velocity and position controller here!
}
// find_index: move axis slowly to detect the zero-pulse
void Controller_Loop::find_index(void)
{
    // use a simple P-controller to get system spinning, add a constant current to overcome friction
    
}
    
void Controller_Loop::reset_pids(void)
{
    // reset all cntrls.
}