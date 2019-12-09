#include "mbed.h"
#include "CAN.h"

DigitalOut led1(LED1);
DigitalOut led2(LED2);
DigitalOut led3(LED3);
DigitalOut led4(LED4);
CAN can2(p30, p29);
Serial pc(USBTX, USBRX); // tx, rx

int arrived;
CANMessage msg;

void can_read()
{
    pc.printf("CAN Read\n");
    unsigned char error=can2.rderror();
    int error_int=error;
    pc.printf("Error: %d\n",error_int);
    if(error_int>0) {
        led1=1;
        can2.reset();
        wait(0.1);
        led1=0;
    } 
    else {
        arrived = can2.read(msg);//dont forget handle when redoing
        pc.printf("Message Arrived: %d\n",arrived);
        if(arrived) {
            led2 = !led2;
            pc.printf("Message Byte 1: %d\n", msg.data[0]);
            pc.printf("Message Byte 2: %d\n", msg.data[1]);
            pc.printf("Message Byte 3: %d\n", msg.data[2]);
        }
    }
}

void err_warn()
{
    led3=1;
    pc.printf("Error Warning\n");
    can2.reset();
    wait(0.1);
    led3=0;
}

void data_or()
{
    led3=1;
    pc.printf("Data Overrun\n");
    can2.reset();
    wait(0.1);
    led3=0;
}

void err_pass()
{
    led3=1;
    pc.printf("Passive Error\n");
    can2.reset();
    wait(0.1);
    led3=0;
}

void arb_lost()
{
    led3=1;
    pc.printf("Arbitration Lost\n");
    can2.reset();    
    wait(0.1);
    led3=0;
}

void err_bus()
{
    led3=1;
    pc.printf("Bus Error\n");
    can2.reset();
    wait(0.1);
    led3=0;
}

int main()
{
    pc.printf("Beginning CAN Read\n");
    can2.frequency(150000);
    can2.mode(CAN::Normal);
    can2.attach(&can_read,CAN::RxIrq);
    can2.attach(&err_warn,CAN::EwIrq);
    can2.attach(&data_or,CAN::DoIrq);
    can2.attach(&err_pass,CAN::EpIrq);
    can2.attach(&arb_lost,CAN::AlIrq);
    can2.attach(&err_bus,CAN::BeIrq);
    while(1) {
        led4 = !led4;
        wait(0.5);
    }
}
