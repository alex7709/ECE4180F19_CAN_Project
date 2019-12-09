#include "mbed.h"
#include "CAN.h"
#include "PinDetect.h"

DigitalOut led1(LED1);
DigitalOut led2(LED2);
DigitalOut led4(LED4);
PinDetect filter_in(p8);
CAN can2(p30, p29);
Serial pc(USBTX, USBRX); // tx, rx


int volatile handle=0x00;
unsigned int volatile id=0xAA;//message id for address 0. Default and initial
unsigned int volatile mask=0xFF;
CANFormat volatile format = CANAny;
int arrived;

void filter_0(void)
{
    handle=0;
    pc.printf("Filter Off\n");
}

void filter_1(void)
{
    //id = 0xff;change if different than initiailization value
    handle = can2.filter(id, mask, format, 1);
    pc.printf("Filter On\n");
    //need message ID and mask for it if applicable. ID 11 bits
}

int main()
{
    filter_in.mode(PullUp); //Need PinDetect
    filter_in.attach_deasserted(&filter_0);
    filter_in.attach_asserted(&filter_1);
    pc.printf("Beginning CAN Read\n");
    CANMessage msg;can2.frequency(150000);
    while(1) {
        unsigned char error=can2.rderror();
        int error_int=error;
        pc.printf("Error: %d\n",error_int);
        if(error_int>0) {
            led1=1;
            can2.reset();
            wait(0.1);
            led1=0;
        }
        arrived = can2.read(msg);//dont forget handle when redoing
        pc.printf("Message Arrived: %d\n",arrived);
        if(arrived) {
            pc.printf("Message Byte 1: %d\n", msg.data[0]);
            pc.printf("Message Byte 2: %d\n", msg.data[1]);
            pc.printf("Message Byte 3: %d\n", msg.data[2]);
            led2 = !led2;
        }
        led4 = !led4;
        wait(0.1);
    }
}
