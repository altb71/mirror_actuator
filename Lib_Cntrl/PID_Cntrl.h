#ifndef PID_CNTRL_H_
#define PID_CNTRL_H_

class PID_Cntrl
{
public:

    PID_Cntrl(float P, float I, float D, float tau_f, float Ts, float uMin, float uMax);
    void setup(float P, float I, float D, float tau_f, float Ts, float uMin, float uMax);
    PID_Cntrl() {};

    float operator()(float e)
    {
        return update(e);
    }

    virtual ~PID_Cntrl();

    void    reset(float initValue);
    void    setCoefficients(float P, float I, float D, float tau_f, float Ts, float uMin, float uMax);
    float   update(float e);
    float   saturate(float);


private:
    float kp,ki,kd,tau_f,Ts,uMax,uMin,b01,a0;
    float Ipart,Dpart,e_old;

};

#endif