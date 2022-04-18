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
    v_cntrl[0].setup(0.019, 3.333, 2.36111e-05, 0.0005,Ts,-.8,.8);//(0.01,1,0,1,Ts,-0.8,0.8);
    v_cntrl[1].setup(0.019, 3.333, 2.36111e-05, 0.0005,Ts,-.8,.8);
    //v_cntrl[1].setup(0.01,1,0,1,Ts,-0.8,0.8);
    Kv[0] = 120;
    Kv[1] = 120;  
    ti.reset();
    ti.start();
    i_des[0] = i_des[1] = 0;
    //controller_type = IDENT_VEL_PLANT; // use 1st version of GPA constructor in main.cpp (line ~23)
    //controller_type = VEL_CNTRL;
    //controller_type = IDENT_POS_PLANT;   // use 2nd version of GPA constructor in main.cpp (line ~24)
    //controller_type = POS_CNTRL;
    controller_type = CIRCLE;
    }

// decontructor for controller loop
ControllerLoop::~ControllerLoop() {}
  
// ----------------------------------------------------------------------------
// this is the main loop called every Ts with high priority
void ControllerLoop::loop(void){
    float v_des;
    uint8_t k = 0;
    float phi_des[2],dphi,error;
    float xy_des[2];
    float om = 2*3.1415 *20;
    uint8_t mot_num = 1;
    while(1)
        {
        ThisThread::flags_wait_any(threadFlag);
        // THE LOOP ------------------------------------------------------------
        m_sa->read_encoders_calc_speed();       // first read encoders and calculate speed
        if(!m_sa->motors_are_referenced())
            reference_loop();
        else
            {
            float tim = ti.read();
            i_des[0] = i_des[1] = 0;
            switch(controller_type)
                {
                case IDENT_VEL_PLANT:
                    i_des[mot_num] = myGPA.update(i_des[mot_num], m_data->sens_Vphi[mot_num]);
                    break;
                case VEL_CNTRL:
                    v_des = myDataLogger.get_set_value(tim);
                    error = v_des - m_data->sens_Vphi[mot_num];
                    i_des[mot_num] = v_cntrl[mot_num](error);
                    myDataLogger.write_to_log(tim,v_des,m_data->sens_Vphi[mot_num],i_des[mot_num]); 
                    break;
                case IDENT_POS_PLANT:
                    v_des = myGPA.update(v_des, m_data->sens_phi[mot_num]);;
                    error = v_des - m_data->sens_Vphi[mot_num];
                    i_des[mot_num] = v_cntrl[mot_num](error);
                    break;
                case POS_CNTRL:
                    phi_des[mot_num] = myDataLogger.get_set_value(tim);
                    dphi = phi_des[mot_num] - m_data->sens_phi[mot_num];
                    v_des = Kv[mot_num] * dphi;
                    error = v_des - m_data->sens_Vphi[mot_num];
                    i_des[mot_num] = v_cntrl[mot_num](error);
                    myDataLogger.write_to_log(tim,phi_des[mot_num],m_data->sens_phi[mot_num],i_des[mot_num]); 
                    break;
                case CIRCLE:
                    xy_des[0] = 25+25*cos(om * tim); 
                    xy_des[1] = 25 * sin(om * tim);
                    m_mk->X2P(xy_des,phi_des);
                    for(uint8_t k=0;k<2;k++)
                        {
                        dphi = phi_des[k] - m_data->sens_phi[k];
                        v_des = Kv[k] * dphi;
                        error = v_des - m_data->sens_Vphi[k];
                        i_des[k] = v_cntrl[k](error);
                        }
                    break;
                default:
                    break;
                }
                m_sa->enable_motors(true);      // enable motors
            }
            // Motor 1 or 2
            m_sa->write_current(0,i_des[0]);
            m_sa->write_current(1,i_des[1]);
            // enabling all
            m_sa->set_laser_on_off(m_data->laser_on);
            
        }// endof the main loop
}

// ----------------------------------------------------------------------------
// this is the main loop called every Ts with high priority
void ControllerLoop::reference_loop(void){
    float error;
    m_sa->read_encoders_calc_speed();       // first read encoders and calculate speed
    error = 5 - m_data->sens_Vphi[0];
    i_des[0] = v_cntrl[0](error);
    error = 5 - m_data->sens_Vphi[1];
    i_des[1] = v_cntrl[1](error);
    m_sa->force_enable_motors(true);
}
// ----------------------------------------------------------------------------
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