// includes
#include <cstdint>
#include "uart_comm_thread.h"



extern DataLogger myDataLogger;

extern GPA myGPA;



/*
-------- DATA PROTOCOL----------------------------
   254	 1	255	201	1	  4	     0		...
    n1	n2	rec id1	id2	#Byte1 #Byte2 thedata
--------------------------------------------------
1-20 sensor values,
		id1		id2	
		10			 	Counter values
				1		c1
				2		c2	(increments)
		11		actual current
				1		i1 / A
				2		i2 / A
--------------------------------------------------
// NOT USED: 21-40 cntrl values,		21		desired values			1 		Phi1 / rad				2 		Phi2 / rad				3 		x / mm				4 		y / mm
--------------------------------------------------
101-120 estimates/actual values,
		101		angles and calculated x,y
				1 		Phi1 / rad
				2 		Phi2 / rad
				3		x / mm
				4		y / mm
--------------------------------------------------
121-140 send techn. values, like offsets
		id1		id2	
		121
				1 		inc_offset phi1 / increments	int16_t
				2		inc_offset phi2	/ 		"			"
				3 		inc_additional_offset phi1 / increments	int16_t
				4		inc_additional_offset phi2	/ 		"			"
		125
				1		num_it of X2P trafo
--------------------------------------------------
2xx: set-value and commands (like disable controller...)
		id1		id2	
		
		202:			set desired absolute values	
				1		phi1		rad		float
				2		phi2		rad		float
				3		x			mm		float
				4		y			mm		float
		203				Increment values
				1		dphi1		rad		float
				2		dphi2		rad		float
				3		dx			mm		float
				4		dy			mm		float
        210:    Log data
                100     log status
                101     start log
                1       send log values
        211     Log type
                1   step
                2   sine
		220			Laser on/off
				1	0 = off, 1 = on
		221			Trafo on/off
				1	0 = off, 1 = on
		230			external control on/off
				1	0 = off, 1 = on
		241     
				1 Send text
		250 
				1 GPA message
*/



// #### constructor
uart_comm_thread::uart_comm_thread(Data_Xchange *data,Mirror_Kinematic *mk,BufferedSerial *com, float Ts): thread(osPriorityBelowNormal, 4096)
 {  
    // init serial
    this->uart = com;
    this->m_data = data;
    this->m_mk = mk;
    this->Ts = Ts;
    this->csm = 0;
    gpa_stop_sent = false;
}

// #### destructor
uart_comm_thread::~uart_comm_thread() {}

// #### run the statemachine
void uart_comm_thread::run(void)
{
    // returnvalue
    bool retVal = false;
	uint8_t checksum,k;
	uint16_t send_state =1011;
	while(true)
    	{
        ThisThread::flags_wait_any(threadFlag);
        //---  The LOOP --------------------------------------------------------
        uint32_t num = uart->read(buffer, sizeof(buffer));
        if (num >0) 
    		{
        	if(buffer[0] == 254 && buffer[1] == 1)
            	{
            	if(analyse_received_data())
            			;//		led1 = !led1;
            	}
            }
		switch(send_state)
			{
			case 1011:
				send(101,12,2*4,(char *)&(m_data->sens_phi[0]));		// send actual phi values (1 and 2)
				send_state = 1012;
				break;	
			case 1012:
				send(101,34,2*4,(char *)&(m_data->est_xy[0]));		// send actual xy values 
				send_state = 210;
				break;	
			case 125:		// number of iterations in the trafo
				send(125,1,1,(char *)&m_data->num_it);		
				send_state = 210;
				break;
			case 210:		// number of iterations in the trafo
				if(myDataLogger.new_data_available)
                    {
                        if(myDataLogger.packet*PACK_SIZE<4*myDataLogger.N_col*myDataLogger.N_row)
                            send(210,1+myDataLogger.packet,PACK_SIZE,(char *)&(myDataLogger.log_data[myDataLogger.packet*(PACK_SIZE/4)]));
                        else
                            {
                            send(210,1+myDataLogger.packet,4*myDataLogger.N_col*myDataLogger.N_row-myDataLogger.packet*PACK_SIZE,(char *)&(myDataLogger.log_data[myDataLogger.packet*PACK_SIZE/4]));
                            myDataLogger.log_status = 1;
                            myDataLogger.new_data_available = false;
                            send_state = 211;
                            }
                        ++myDataLogger.packet;
                    }
                else
                    send_state = 250;
				break;
            case 211:
                send(210,99,0,buffer);
                send_state = 250;
                break;
			case 250:		// send GPA values
				if(myGPA.new_data_available)
					{
					float dum[8];
					myGPA.getGPAdata(dum);
					send(250,1,32,(char *)&(dum[0]));	// send new values (8 floats)
                    }
				else if(myGPA.start_now)
					{
                    char dum = 0;
					send(250,2,1,&dum);			// send start flag
					myGPA.start_now = false;
					gpa_stop_sent  = false;
					}
				else if(myGPA.meas_is_finished && !gpa_stop_sent && !myGPA.new_data_available)
					{
                    char dum = 0;
                    send(250,255,1,&dum);		// send stop flag
					gpa_stop_sent = true;
					}
				send_state = 1011;
				break;			
			default:
				break;
			}
        }// loop
}




// ------------------- start uart ----------------
void uart_comm_thread::start_uart(void){
		
		thread.start(callback(this, &uart_comm_thread::run));
		ticker.attach(callback(this, &uart_comm_thread::sendThreadFlag), Ts);
}
// this is for realtime OS
void uart_comm_thread::sendThreadFlag() {
    thread.flags_set(threadFlag);
}

void uart_comm_thread::send_text(const char *txt)
{	
	uint16_t N=0;
   	while(txt[N] != 0)		// get length of text
     	N++;
    buffer[0]=254;buffer[1]=1;buffer[2]=255;	// standard pattern
	buffer[3] = 241;
	buffer[4] = 1;
	buffer[5] = N%256;
    buffer[6] = N/256;
	uart->write(buffer, 7);
	uart->write(txt,N);
    char dum = 0;
	uart->write(&dum,1);		// line end

}
// ---------------------  send N char data --------------------------------------
void uart_comm_thread::send(uint8_t id1, uint8_t id2, uint16_t N, char *m)
{
	char buffer[7], csm = 0;
	/* Add header */
	buffer[0] = 254;
	buffer[1] = 1;
	buffer[2] = 255;
	/* Add message IDs*/
	buffer[3] = id1;
	buffer[4] = id2;
	/* Add number of bytes*/
	*(uint16_t *)&buffer[5] = N; // cast targt to appropriate data type
	/* send header */
    uart->write(buffer, 7);
	for (int i = 0; i < 7; ++i)
		csm += buffer[i];
	/* send data */
    if(N>0)
        uart->write(m,N);
   	for (uint16_t i = 0; i < N; ++i)
		csm += m[i];
	uart->write(&csm,1);
}
// -----------------------------------------------------------------------------
// analyse data, see comments at top of this file for numbering
bool uart_comm_thread::analyse_received_data(void){
	char msg_id1 = buffer[3];
	char msg_id2 = buffer[4];
	uint16_t N = 256 * buffer[6] + buffer[5];
	switch(msg_id1)
		{
		case 202:				// set desired phi or xy-values
			if(N != 4)
				return false;
			switch(msg_id2)
				{
				case 1:
					m_data->cntrl_phi_des[0] = *(float *)&buffer[7];
					return true;
					break;
				case 2:
					m_data->cntrl_phi_des[1] = *(float *)&buffer[7];
					return true;
					break;
				case 3:
					m_data->cntrl_xy_des[0] = 	*(float *)&buffer[7];
					return true;
					break;
				case 4:
					m_data->cntrl_xy_des[1] = *(float *)&buffer[7];
					return true;
					break;
				default:
					break;
				}
			break;		// case 202
		case 203: 		// increment desired values
			if(N != 4)
				return false;
			switch(msg_id2)
				{
				case 1:
					m_data->cntrl_phi_des[0] += *(float *)&buffer[7];
					return true;
					break;
				case 2:
					m_data->cntrl_phi_des[1] += *(float *)&buffer[7];
					return true;
					break;
				case 3:
					m_data->cntrl_xy_des[0] += 	*(float *)&buffer[7];
					return true;
					break;
				case 4:
					m_data->cntrl_xy_des[1] += *(float *)&buffer[7];
					return true;
					break;
				default:
					break;
				}
			break;		// case 203
        case 210:
            switch(msg_id2)
                {
                case 101:           // receive start signal and Amp, Freq, offset values
                    if(myDataLogger.log_status == 1)
                        {
                        myDataLogger.reset_data();
                        myDataLogger.input_type = (uint8_t)buffer[7];
                        myDataLogger.Amp = *(float *)&buffer[8];
                        myDataLogger.omega = *(float *)&buffer[12];
                        myDataLogger.offset = *(float *)&buffer[16];
                        myDataLogger.downsamp = (uint8_t)buffer[20];
                        send_text((char *)"Started time measure");
                        myDataLogger.log_status = 2;
                        }
                    break;
                }
            break;      // case 210
		case 220:				// switch laser on/off
			if(N != 1)
				return false;
			switch(msg_id2)
				{
				case 1:
					if(buffer[7] == 1)
						m_data->laser_on = true;
					else 
						m_data->laser_on = false;
					return true;
					break;
				}
			break;
		case 221:				// switch trafo on/off
			if(N != 1)
				return false;
			switch(msg_id2)
				{
				case 1:
					if(buffer[7] == 1)
						m_mk->trafo_is_on = true;
					else 
						m_mk->trafo_is_on = false;
					return true;
					break;
				}
			break;
		case 230:				// set internal/external control
			if(N != 1)
				return false;
			switch(msg_id2)
				{
				case 1:
					if(buffer[7] == 1)
						m_mk->external_control = true;
					else 
						m_mk->external_control = false;
					return true;
					break;
				}
			break;
		}
	return false;	
}