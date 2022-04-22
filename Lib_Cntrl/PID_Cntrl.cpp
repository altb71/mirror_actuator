#include "PID_Cntrl.h"

// Matlab
// Tn = .005;
// Gpi= tf([Tn 1],[Tn 0]);
// Kp = 0.0158;
// pid(Kp*Gpi);

PID_Cntrl::PID_Cntrl(float P, float I, float D, float tau_f, float Ts, float uMin, float uMax)
{
    // ------------------
    this->kp = P;
    this->ki = I;
    this->kd = D;
    this->tau_f = tau_f;
    this->Ts = Ts;
    this->uMin = uMin;
    this->uMax = uMax;
    this->b01 = D*2.0f/(Ts+2.0f*tau_f);
    this->a0 = (Ts-2.0f*tau_f)/(Ts+2.0f*tau_f);
    reset(0);
}
void PID_Cntrl::setup(float P, float I, float D, float tau_f, float Ts, float uMin, float uMax)
{
    // ------------------
    this->kp = P;
    this->ki = I;
    this->kd = D;
    this->tau_f = tau_f;
    this->Ts = Ts;
    this->uMin = uMin;
    this->uMax = uMax;
    this->b01 = D*2.0f/(Ts+2.0f*tau_f);
    this->a0 = (Ts-2.0f*tau_f)/(Ts+2.0f*tau_f);
    reset(0);
}

PID_Cntrl::~PID_Cntrl() {}

void PID_Cntrl::reset(float initValue)
{
    // -----------------------
    Ipart = 0.0;
    Dpart = 0.0;
    e_old = 0.0;
}


float PID_Cntrl::update(float e)
{
    // the main update, PID Controller is discretized with bilinear/tustin (see Blatt 2, A1)
    float Ppart = kp * e;
    Ipart = Ipart + ki * Ts/2.0f * (e + e_old);
    Dpart = -a0 * Dpart + b01 * (e-e_old);
    e_old = e;
    Ipart = saturate(Ipart);

    return saturate(Ppart + Ipart + Dpart); 
   
}

float PID_Cntrl::saturate(float x)
{
if(x > uMax)
    return uMax;
else if(x < uMin)
    return uMin;
return x;
}