#ifndef UART_COMM_THREAD_H_
#define UART_COMM_THREAD_H_

#include "GPA.h"
#include "DataLogger.h"
#include "mbed.h"
#include "ThreadFlag.h"
#include "data_structs.h"
#include "Mirror_Kinematic.h"


using namespace std;

// "protocol" specifics

#define BUF_LEN      	20  // max 256
#define DATA_LEN      	20  // max 256

// states
#define IDLE    0
#define WAIT    1
#define RECEIVE 2
#define DONE    3
#define PACK_SIZE 200

#define LEN_OF_EXP_TYPE2 1   // length in bytes of expected Type
#define NUM_OF_VALUE    7   // number of expected values
#define EXPECTED2        LEN_OF_EXP_TYPE2 * NUM_OF_VALUE  // byte per Value * expected values = total expected bytes

// predefiniton for callback (couldn't implement as member)

class uart_comm_thread{
public:
// public members
    uart_comm_thread(Data_Xchange *,Mirror_Kinematic *, BufferedSerial*,float);
    virtual ~uart_comm_thread();
    void run(void);             // runs the statemachine, call this function periodicly, returns true when new data ready (only true for 1 cycle)
   // void request();         // request new set of data  
    void start_uart(void);

    // public vars
    // public vars
	const uint8_t N = DATA_LEN;
	char 	buffer[256];     // RX buffer
	void send_text(const char *);
	
private:

    // private members 
    void sendCmd(char);     // sends comand to device
    void callBack();        // ISR for storing serial bytes
    float Ts;
    void send(uint8_t , uint8_t , uint16_t, char *);
    bool analyse_received_data(void);
    bool gpa_stop_sent;
    char csm;
    
// -------------------
//	uint8_t buffer[BUF_LEN];     // RX buffer
//	uint8_t buffCnt;            // max 255
	uint8_t state;              // statemachine state variable
	BufferedSerial* uart;   // pointer to uart for communication with device
    ThreadFlag              threadFlag;
    Thread                  thread;
    Ticker                  ticker;
    Mutex mutex;
	void sendThreadFlag();
    Data_Xchange *m_data;
    Mirror_Kinematic *m_mk;
};

#endif


