#include "mbed.h"
#include <stdint.h>
#include "math.h" 
#include "path_1d.h"
#include "GPA.h"
#include "DataLogger.h"
#include "ControllerLoop.h"
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
GPA          myGPA( 5,  1000,    30,2.5,2.5, Ts); // para for position controller
DataLogger myDataLogger(1);

//******************************************************************************
//---------- main loop -------------
//******************************************************************************

int main()
{

    // --------- Mirror kinematik, define values, trafos etc there
    Data_Xchange data;              // data exchange structure, see data_structs.h in the "Lib_Misc" library
    Mirror_Kinematic mk(&data);
    uart_comm_thread uart_com(&data,&serial_port,.025f);   // this is the communication thread
    sensors_actuators hardware(&data, Ts);         // in this class all the physical ios are handled
    ControllerLoop loop(&data,&hardware,&mk,Ts);       // this is forthe main controller loop
    reset_data(&data);
    ThisThread::sleep_for(200ms);
// ----------------------------------
    serial_port.set_baud(115200);
    serial_port.set_format(8,BufferedSerial::None,1);
    serial_port.set_blocking(false); // force to send whenever possible and data is there
    serial_port.sync();


    loop.init_controllers();
    uart_com.start_uart();
    loop.start_loop();
    ThisThread::sleep_for(200ms);
    uart_com.send_text((char *)"Start SmartScanner 0.3");
    //printf("Start SmartScanner 0.1\r\n");
    
    while(1)
    {
        ThisThread::sleep_for(500ms);
     //printf("%f %f\r\n",data.sens_Vphi[0],data.sens_Vphi[1]); 
    }  
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
