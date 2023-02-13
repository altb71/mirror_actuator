#include "sensors_actuators.h"

#define PI 3.1415927
// constructors


// Deconstructor
/*sensors_actuators::sensors_actuators(Data_Xchange *data,float Ts) : di1(.0005,Ts),di2(.0005,Ts),big_button(PC_3),counter1(PA_6, PC_7),
                            indexpulse1(PA_8),index1(counter1,indexpulse1), counter2(PB_6, PB_7),indexpulse2(PB_4),index2(counter2,indexpulse2),
                            i_enable(PC_4),laser_on(PB_0), i_des1(PA_5),i_des2(PA_4),uw1(4000,16),uw2(4000,16)*/
sensors_actuators::sensors_actuators(Data_Xchange *data, float Ts) : di1(.002,Ts), di2(.002,Ts), big_button(USER_BUTTON1), counter2(ENCODER2_CHA, ENCODER2_CHB),
                            indexpulse1(PA_8), index1(counter1, indexpulse1), counter1(ENCODER1_CHA, ENCODER1_CHB), indexpulse2(PB_4), index2(counter2, indexpulse2),
                            i_enable1(ESCON1_ENABLE), i_enable2(ESCON2_ENABLE), laser_on(PB_0), i_des1(ESCON1_AIN), i_des2(ESCON2_AIN), uw1(4*6400, 16), uw2(4*4096, 16)
{
    this->m_data = data;
    
    this->i2u.setup(-3.0f, 3.0f, 0.0f, 1.0f);
    this->i_des1 = 0.5f;
    this->i_des2 = 0.5f;
    this->i_enable1 = 0;       // disable current first
    this->i_enable2 = 0;       // disable current first
    
    this->counter1.reset();    // encoder reset
    this->counter2.reset();    // encoder reset
    
    this->set_laser_on_off(false);
    
    this->enc_offsets[0] = 0;
    this->enc_offsets[1] = 0;

    return;    
}
sensors_actuators::~sensors_actuators() {} 

void sensors_actuators::read_encoders_calc_speed(void)
{
    m_data->sens_phi[0] = uw1(counter1 - enc_offsets[0]);// - index1.getPositionAtIndexPulse());
    m_data->sens_phi[1] = uw2(counter2 - enc_offsets[1]);// - index2.getPositionAtIndexPulse());
    m_data->sens_Vphi[0] = di1(m_data->sens_phi[0]);
    m_data->sens_Vphi[1] = di2(m_data->sens_phi[1]);
}
void sensors_actuators::set_enc_offsets(float o1,float o2)
{
    enc_offsets[0] = o1;
    enc_offsets[1] = o2;
}

void sensors_actuators::enable_motors(bool enable)
{
    i_enable1 = big_button && enable;
    i_enable2 = big_button && enable;
}
void sensors_actuators::force_enable_motors(bool enable)
{
    i_enable1 = enable;
    i_enable2 = enable;
}
bool sensors_actuators::motors_are_referenced(void)
{
    return (index1.getPositionAtIndexPulse() !=0 && index2.getPositionAtIndexPulse() !=0);
}
void sensors_actuators::write_current(uint8_t mot_nb, float setvalue)
{
    if(mot_nb == 0)
        i_des1 = i2u(setvalue);
    else if(mot_nb == 1)
        i_des2 = i2u(setvalue);    
}
void sensors_actuators::set_laser_on_off(bool laser_on_off)
{
    laser_on = laser_on_off;
}
