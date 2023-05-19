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
    v_cntrl_1 = PID_Cntrl(0.02,4,0,0,Ts,-0.8,0.8); // nur PI-Regler
    ableit_vorst = IIR_filter(0.0005,Ts);
    //v_cntrl_1 = PID_Cntrl(0.0119, 1.45, 1.98e-05, 0.000769,Ts,-0.8,0.8);
    }
// decontructor for controller loop
Controller_Loop::~Controller_Loop() {}
// ----------------------------------------------------------------------------
// this is the main loop called every Ts with high priority
void Controller_Loop::loop(void){
    float i_des,v_des,phi_des,v_des_vorst;
    uint8_t k = 0;
    float kv = 149;
    v_des = 0;
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
                // *** Identifikation GEschwindigkeitsregelstrecke
/*Variante 1*/    // i_des = myGPA.update(i_des,m_data->sens_Vphi[0]);
/*Variante 2*/    // i_des = 0.02*(50+myGPA.update(i_des,m_data->sens_Vphi[0]) - m_data->sens_Vphi[0]);
/*Variante 3*/    // i_des = 0.02*(50.0 - m_data->sens_Vphi[0]) + myGPA.update(i_des,m_data->sens_Vphi[0]);
                // *** Identifikation Positionsregelstrecke
                v_des = myGPA.update(v_des,m_data->sens_phi[0]);
                i_des = v_cntrl_1(v_des - m_data->sens_Vphi[0]);
                break;
            case CNTRL_VEL:
                m_sa->enable_motors(true);      // enable motors
            // Geschwindigkeitsregler
                v_des = myDataLogger.get_set_value(ti_loc);
                i_des = v_cntrl_1(v_des - m_data->sens_Vphi[0]);
                myDataLogger.write_to_log(ti_loc,v_des, m_data->sens_Vphi[0], i_des);
                break;
            case CNTRL_POS:
                m_sa->enable_motors(true);      // enable motors
            // Winkelregler 
                phi_des = myDataLogger.get_set_value(ti_loc);
                v_des_vorst = ableit_vorst(phi_des);
                v_des = kv*(phi_des - m_data->sens_phi[0]) + v_des_vorst;
                i_des = v_cntrl_1(v_des - m_data->sens_Vphi[0]);
                myDataLogger.write_to_log(ti_loc,phi_des, m_data->sens_phi[0], i_des);
                break;
            // ------------------------ do the control first
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
void Controller_Loop::switch_to_cntrl_pos()
{
    controller_state = CNTRL_POS;
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