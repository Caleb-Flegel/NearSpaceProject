//Code implimentation for DSHOT150 controller

#include <DSHOT150.h>
#include <mbed.h>


DSHOT150::DSHOT150(PinName p22) : _pin(p22)
{
    _pin = 0;
    tel = 0;
}

void DSHOT150::write_zero() {

    _pin.write(1);
    wait_ns(1423);

    _pin.write(0);
    wait_ns(3247);
 
}

void DSHOT150::write_one() {

    _pin.write(1);
    wait_ns(3247);
    
    _pin.write(0);
    wait_ns(1423);
    
}

void DSHOT150::check_sum(unsigned int v) {
    v = v<<1;
    v = v|tel;
    uint8_t cs = 0;
    for( int i = 0; i < 3; ++i){
        cs ^= v;
        v>>=4;
    }
    cs&=0xF;
    packet[15] = cs&0x1;
    packet[14] = cs&0x2;
    packet[13] = cs&0x4;
    packet[12] = cs&0x8;
}

void DSHOT150::set_tel(bool v) {
    tel = v == true? 1 : 0;
}

void DSHOT150::send_packet() {
    
    for(int j = 0; j < 1; ++j) {
        for(int i = 0; i < 16; ++i) {
            if(packet[i])
                write_one();
            else
                write_zero();
        }
        ThisThread::sleep_for(5ms);
        /* value + 6 us = time duration interval between packets, Ideally 300. Found experimentally, the maximum time is s
        somewhere between 40,000 and 100,000 or between 10 and 50 ms. Above that, the ESC doesn't interpret things correctly.
        LARKIN AND CALEB this is the duration between packets that we want the thread to sleep for (?) */
    }
}

void DSHOT150::set_3d_on()
{
    unsigned int val;
    val = 10;                    //the command to set 3d mode has a corresponding throttle value of 10 (see "special commands.txt")

    check_sum(val); //Calculate the checksum and insert it into the packet
   
    for(int i = 10; i >= 0; --i) {  //Insert the throttle bits into the packet
        packet[i] = val&0x1;
        val = val>>1;
    }

    packet[11] = tel;   //Set the telemetry bit in the packet

    for (int i=0; i<10; i++) {               
        send_packet();  //the command to set 3d mode to on has to be sent 10 times
    }
}

void DSHOT150::special_cmd(unsigned int code)
{

    check_sum(code); //Calculate the checksum and insert it into the packet
   
    for(int i = 10; i >= 0; --i) {  //Insert the throttle bits into the packet
        packet[i] = code&0x1;
        code = code>>1;
    }

    packet[11] = tel;   //Set the telemetry bit in the packet
    
    for (int i=0; i<10; i++) {               
        send_packet();  //supposed to send the code 10 times
    }
}

//while (i < 1) {          //it should take about 1 second for 2500 packets to send
//        special_cmd(9);     //sets 3d off (enables "2d")
//        i++;
//    }

void DSHOT150::arm() //DON'T USE THIS
{
        Timer t;
        t.start();

        while (t.elapsed_time() < 5s) {
            special_cmd(0);     //disarms and stops (brakes) any motor rotation by setting throttle-type value to 0 (not 48, which would be 0% throttle)
                                //step 1/2 of arming sequence is for ESC to read these 0s
        }

        while (t.elapsed_time() < 7s) {
            throttle(0);        //gives throttle signal of 0% which is 48
                                //step 2/2 of arming sequence is for ESC to read 0 rpm
        }
}


void DSHOT150::arm_3d() //USE THIS INSTEAD
{
    Timer t;
    t.start();

    while (t.elapsed_time() < 5s) {
        special_cmd(0);     //disarms and stops (brakes) any motor rotation by setting throttle-type value to 0 (not 1048, which would be 0% throttle)
                            //step 1/2 of arming sequence is for ESC to read these 0s
    }
}



void DSHOT150::throttle(float speed)
{
    unsigned int val;
    speed = speed > 1 ? 1 : speed; //Bound checking and restricitng of the input
    speed = speed < 0 ? 0 : speed; //Anything below 0 is converted to zero
                                   //Anything above 1 is converted to one
                                   
    val = (unsigned int)(speed * 2000); //Scale the throttle value. 0 - 48 are reserved for the motor
    val +=48;                           //Throttle of zero starts at 48
   
    check_sum(val); //Calculate the checksum and insert it into the packet
   
    for(int i = 10; i >= 0; --i) {  //Insert the throttle bits into the packet
        packet[i] = val&0x1;
        val = val>>1;
    }
   
    packet[11] = 0;   //Set the telemetry bit in the packet
   
    send_packet();
}

void DSHOT150::throttle_3d(float speed)
{
    signed int val;
                                     //Bounds checking and restricting of the input
    speed = speed > .99 ? .99 : speed;   //Anything above 1 is converted to one
    speed = speed < -.99 ? -.99 : speed; //Anything below -1 is converted to -1

    speed = abs(speed) < MIN_THROTTLE ? 0 : speed; 

    if (speed < 0) {
        speed = (speed*(-1) - 1);       //for some reason, the ESC interprets -10% throttle as -90% throttle, so this converts -.1 to -.9
    }                                   //the issue is only in the negative region so we only need to make this change for speed < 0
                                   
    val = (signed int)(speed*1000 + 1048); //Scale the throttle value. 0 - 48 are reserved for the motor
                                           //Throttle of zero starts at 1048
   
    check_sum(val); //Calculate the checksum and insert it into the packet
   
    for(int i = 10; i >= 0; --i) {  //Insert the throttle bits into the packet
        packet[i] = val&0x1;
        val = val>>1;
    }
   
    packet[11] = tel;   //Set the telemetry bit in the packet
   
    send_packet();
}