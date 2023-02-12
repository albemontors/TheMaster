#include "mbed.h"
#include <cstdint>

#include "mat.h"

Thread canPdo;
Ticker canTick;

CAN canBus(PD_0, PD_1);

int receive = 1;

// CAN DATA AS GLOBAL OBJECTS
uint8_t inputWords[32];
uint8_t outputWords[32];

void canPdoTransaction(){
    receive = 1;
}

// main() runs in its own thread in the OS
int main()
{

    // INIT

    DigitalOut led(LED1);
    canBus.monitor(1);
    
    // THREADS
    printf("Starting... \n");

    // MAIN
    while (true) {
        outputWords[0] = 1;// not button.read();

        if(receive){
            //canBus.attach(canPdoTransaction, CAN::RxIrq);
            CANMessage msg( inputWords[0] );
            if( canBus.read(msg) )
                printf("packet received with value: %d \n", inputWords[0]);
            else
                printf("packet failed to receive. \n");
            //receive = 0;
            wait_us(100000);
        }
    }
}
