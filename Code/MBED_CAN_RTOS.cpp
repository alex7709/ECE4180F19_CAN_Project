#include "mbed.h"
#include "CAN.h"
#include "rtos.h"

DigitalOut led1(LED1);
DigitalOut led2(LED2);
DigitalOut led4(LED4);
CAN can2(p30, p29);
Serial pc(USBTX, USBRX); // tx, rx

//Mutex can_mtx;

void errors(void const *args)
{
    while(1){        
        //can_mtx.lock();
        unsigned char error=can2.rderror();
        int error_int=error;
        pc.printf("Error: %d\n",error_int);
        if(error_int>0) {
            led1=1;
            can2.reset();
            Thread::wait(1);
            led1=0;
        }
        //can_mtx.unlock();
        Thread::wait(1);
    }
}

void can_read(void const *args)
{
    while(1){
        //can_mtx.lock();
        CANMessage msg;
        int arrived = can2.read(msg,0);//dont forget handle when redoing
        pc.printf("Message Arrived: %d\n",arrived);
        if(arrived) {
            pc.printf("Message Byte 1: %d\n", msg.data[0]);
            pc.printf("Message Byte 2: %d\n", msg.data[1]);
            pc.printf("Message Byte 3: %d\n", msg.data[2]);
            led2 = !led2;
        }
        //can_mtx.unlock();
        Thread::wait(1);
    }
}

int main()
{
    pc.printf("Beginning CAN Read\n");
    can2.frequency(150000);can2.mode(CAN::Normal);
    Thread thread2(errors);
    Thread thread3(can_read);
    while(1) {
        led4 = !led4;
        Thread::wait(1000);
    }
}
