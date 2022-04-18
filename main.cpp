#include "mbed.h"
#include <stdint.h>
#include "math.h" 
#include "path_1d.h"
#include "GPA.h"
#include "DataLogger.h"
#include "ControllerLoop.h"
#include "Mirror_Kinematic.h"
#include "data_structs.h"
#include "FastPWM.h"
#include "sensors_actuators.h"
#include "uart_comm_thread.h"
 
static BufferedSerial serial_port(USBTX, USBRX);

float Ts = 0.0002f;                    // sampling time

// --------- local functions
void reset_data(Data_Xchange *);

//----------------------------------------- global variables (uhh!) ---------------------------
//init values:    (f0,   f1, nbPts, A0, A1, Ts)
//GPA          myGPA(5 , 1000,    30,.35,.5, Ts); // % para for vel-cntrl
GPA          myGPA( 1,  150,    30,100,10, Ts); // para for position controller


DataLogger myDataLogger(1);

//******************************************************************************
//---------- main loop -------------
//******************************************************************************

int main()
{

    // --------- Mirror kinematik, define values, trafos etc there
    Data_Xchange data;              // data exchange structure, see data_structs.h in the "Lib_Misc" library
    Mirror_Kinematic mk(&data);     // Mirror_Kinematics class, the geom. parameters, trafos etc. are done
    uart_comm_thread uart_com(&data, &mk,&serial_port,.05f);   // this is the communication thread
    sensors_actuators hardware(&data, Ts);         // in this class all the physical ios are handled
    ControllerLoop loop(&data,&hardware,&mk,Ts);       // this is forthe main controller loop
    reset_data(&data);
    ThisThread::sleep_for(200);
// ----------------------------------
    serial_port.set_baud(115200);
    serial_port.set_format(8,BufferedSerial::None,1);
    serial_port.set_blocking(false); // force to send whenever possible and data is there
    hardware.set_enc_offsets(1037,-188);          // individal set values for global position
    mk.trafo_is_on =  true;

    loop.init_controllers();
    uart_com.start_uart();
    loop.start_loop();
    ThisThread::sleep_for(200);
    uart_com.send_text((char *)"Start Mirroractuator 2.7");
    while(1)
        ThisThread::sleep_for(200);
        
}   // END OF main


void reset_data(Data_Xchange *da)
{
    for(uint8_t k=0;k<2;k++)
        {
        da->sens_phi[k] = 0;
        da->est_xy[k] = 0;
        da->sens_Vphi[k] = 0;
        da->cntrl_phi_des[k] = 0;
        da->cntrl_Vphi_des[k] = 0;
        da->cntrl_xy_des[k] = 0;
        da->i_des[k] = 0;        
        da->wMot[k] = 0;         
        }
    da->laser_on = false;
    da->num_it = 0;
} 
