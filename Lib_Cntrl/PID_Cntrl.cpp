#include "PID_Cntrl.h"

// Matlab
// Tn = .005;
// Gpi= tf([Tn 1],[Tn 0]);
// Kp = 0.0158;
// pid(Kp*Gpi);

PID_Cntrl::PID_Cntrl(float kp, float ki, float kd, float tau_f, float Ts, float uMin, float uMax)
{
    // ------------------
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
    this->tau_f = tau_f;
    this->Ts = Ts;
    this->uMin = uMin;
    this->uMax = uMax;
    reset(0);
}

PID_Cntrl::~PID_Cntrl() {}

void PID_Cntrl::reset(float initValue)
{
    // -----------------------
    Ipart = Dpart = e_old = 0;
}


float PID_Cntrl::update(float e)
{
    // the main update function
    // AUFGABE 6.3, 6.4, 7.1
    Ipart += ki*Ts/2 * (e + e_old);
    Ipart = saturate(Ipart);
    e_old = e;
    float ret_val = saturate(kp*e + Ipart);
    return ret_val;   // saturate and return 
}

float PID_Cntrl::saturate(float x)
{
if(x > uMax)
    return uMax;
else if(x < uMin)
    return uMin;
return x;
}