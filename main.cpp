#include "mbed.h"
#include <cstdint>

Thread canPdo;
Ticker canTick;

CAN canBus(PD_0, PD_1);

int send = 0;

// CAN DATA AS GLOBAL OBJECTS
uint8_t inputWords[32];
uint8_t outputWords[32];

void canPdoTransaction(){
    send = 1;
}

// main() runs in its own thread in the OS
int main()
{

    // INIT

    DigitalIn button(USER_BUTTON);
    
    // THREADS

    canTick.attach(canPdoTransaction, 1s);

    // MAIN
    while (true) {
        outputWords[0] = 1;// not button.read();

        if(send){
            CANMessage msg( outputWords[0] );
            if( canBus.write(msg) )
                printf("packet sent with value: %d \n", outputWords[0]);
            else
                printf("packet failed to send. \n");
            send = 0;
        }
    }
}
