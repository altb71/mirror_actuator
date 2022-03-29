#include "ControllerLoop.h"
using namespace std;

extern GPA myGPA;
extern DataLogger myDataLogger;

// contructor for controller loop
ControllerLoop::ControllerLoop(Data_Xchange *data,sensors_actuators *sa, Mirror_Kinematic *mk, float Ts) : thread(osPriorityHigh,4096)
{
    this->Ts = Ts;
    this->m_data = data;
    this->m_sa = sa;
    this->m_mk = mk;
    v_cntrl[0].setup(0.017,3.48,0,1,Ts,-0.8,0.8);
    v_cntrl[1].setup(0.014,2.76,0,1,Ts,-0.8,0.8);
    Kv[0] = 130;
    Kv[1] = 130;
    ti.reset();
    ti.start();
    }

// decontructor for controller loop
ControllerLoop::~ControllerLoop() {}

// ----------------------------------------------------------------------------
// this is the main loop called every Ts with high priority
void ControllerLoop::loop(void){
    float i_des, v_des;
    uint8_t k = 0;
    while(1)
        {
        ThisThread::flags_wait_any(threadFlag);
        // THE LOOP ------------------------------------------------------------
        m_sa->read_encoders_calc_speed();       // first read encoders and calculate speed
        // -------------------------------------------------------------
        // at very beginning: move system slowly to find the zero pulse
        // set "if(0)" if you like to ommit at beginning
        if(0)//!is_initialized) //this is for initialization (find Index pulse)
            {
            find_index();
            if(0)//index1.positionAtIndexPulse != 0 && index2.positionAtIndexPulse != 0) 
                is_initialized=true;
            }
        else
            {
            
            // ------------------------ do the control first
            // calculate desired currents here, you can do "anything" here, 
            // if you like to refer to values e.g. from the gui or from the trafo,
            // please use m_data->xxx values, 
            
            // ------------------------ pos contrl
            float phi_des = myDataLogger.get_set_value(ti.read());
            float dphi = phi_des - m_data->sens_phi[0];
            i_des = myGPA.update(i_des,m_data->sens_Vphi[0]);
            //v_des = Kv[0] * dphi;
            //v_des = myDataLogger.get_set_value(ti.read());
            // Motor 1
            //float error = v_des - m_data->sens_Vphi[0];
            //i_des = v_cntrl[0](error);
            m_sa->write_current(0,i_des);
            // Motor 2
            //error = v_des/8 - m_data->sens_Vphi[1];
            //i_des = v_cntrl[1](error);
            m_sa->write_current(1,0);
            // enabling all
            m_sa->enable_motors(true);      // enable motors
            m_sa->set_laser_on_off(m_data->laser_on);
            //myDataLogger.write_to_log(ti.read(),phi_des,m_data->sens_phi[0],i_des); 

            }
        if(++k>=10)
            {
            m_mk->P2X(m_data->sens_phi,m_data->est_xy);
            k = 0;
            }
            
        }// endof the main loop
}

void ControllerLoop::sendSignal() {
    thread.flags_set(threadFlag);
}
void ControllerLoop::start_loop(void)
{
    thread.start(callback(this, &ControllerLoop::loop));
    ticker.attach(callback(this, &ControllerLoop::sendSignal), Ts);
}

float ControllerLoop::pos_cntrl(float d_phi)
{
   
   // write position controller here
   return 0.0;
    }

void ControllerLoop::init_controllers(void)
{
    // set values for your velocity and position controller here!
    
    
}
// find_index: move axis slowly to detect the zero-pulse
void ControllerLoop::find_index(void)
{
    // use a simple P-controller to get system spinning, add a constant current to overcome friction
    
}
    
void ControllerLoop::reset_pids(void)
{
    // reset all cntrls.
}