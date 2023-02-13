#pragma once
/* class sensors_actuators
Tasks for students:
    - scale ios correctly
    - define derivative filter correctly
*/
#include <cstdint>
#include "EncoderCounter.h"
#include "EncoderCounterIndex.h"
#include "IIR_filter.h"
#include "LinearCharacteristics.h"
#include "data_structs.h"
#include "Enc_unwrap_scale.h"
#include "LaserScannerPCBSetup.h"


class sensors_actuators
{
public:
    sensors_actuators(Data_Xchange *,float Ts);        // default constructor
    virtual ~sensors_actuators();   // deconstructor
    void read_encoders_calc_speed(void);       // read both encoders and calculate speeds
    float get_phi(uint8_t k);         // get angle of motor k
    float get_om(uint8_t k);          // get speed of motor k
    void set_des_current(uint8_t k);  // set desired current on actuator k
    void enable_motors(bool);       // enable/disable motors via DigitalOut, send a "true" and also press button
    void force_enable_motors(bool);       // enable/disable motors via DigitalOut, send a "true" and also press button
    bool motors_are_referenced(void);
    void write_current(uint8_t,float);  // write current to motors (0,...) for motor 1, (1,...) for motor 2
    void set_laser_on_off(bool);    // set laser on or off
    void set_enc_offsets(float,float);
    int16_t enc_offsets[2];
    DigitalIn big_button;         // Enable button an backside
private:
    IIR_filter di1;
    IIR_filter di2;
    
    AnalogOut i_des1;           // desired current values
    AnalogOut i_des2;
    DigitalOut i_enable1;
    DigitalOut i_enable2;
    DigitalOut laser_on;
    
    ///------------- Encoder -----------------------
    EncoderCounter counter1;    // initialize counter on PA_6 and PC_7
    InterruptIn indexpulse1;
    EncoderCounterIndex index1;   
    // ------------------------------------
    EncoderCounter counter2;    // initialize counter on PB_6 and PB_7
    InterruptIn indexpulse2;
    EncoderCounterIndex index2;    // 
    
    //-------------------------------------
    LinearCharacteristics i2u;
    
    Enc_unwrap_scale uw1;
    Enc_unwrap_scale uw2;
    Data_Xchange *m_data;

};