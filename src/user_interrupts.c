// This is where the "user" interrupts handlers should go
// The *must* be declared in "user_interrupts.h"

#include "maindefs.h"
#ifndef __XC8
#include <timers.h>
#else
#include <plib/timers.h>
#endif
#include "user_interrupts.h"
#include "messages.h"
#include "my_motor.h"
#include "my_i2c_master.h"
#include "timer1_thread.h"

// A function called by the interrupt handler
// This one does the action I wanted for this program on a timer0 interrupt
int timer1_extender = 0;

void timer0_int_handler() {
    unsigned int val;
    int length, msgtype;
    LATBbits.LATB6 = !LATBbits.LATB6;
    // toggle an LED
    // LATBbits.LATB0 = !LATBbits.LATB0;
    // reset the timer
#if defined (MOTOR_PIC)
    WriteTimer0(TIMER0_START);
    ticks_right++;
    ticks_right_C++;
#else
    WriteTimer0(0);
#endif

    // try to receive a message and, if we get one, echo it back
    //length = FromMainHigh_recvmsg(sizeof(val), (unsigned char *)&msgtype, (void *) &val);
    //if (length == sizeof (val)) {
     //   ToMainHigh_sendmsg(sizeof (val), MSGT_TIMER0, (void *) &val);
    //}
}

// A function called by the interrupt handler
// This one does the action I wanted for this program on a timer1 interrupt

void timer1_int_handler() {
    unsigned int result;

    // read the timer and then send an empty message to main()
    //LATBbits.LATB7 = !LATBbits.LATB7;
    result = ReadTimer1();
    //ToMainLow_sendmsg(0, MSGT_TIMER1, (void *) 0);
    

#if defined (MOTOR_PIC)
    // reset the timer
    WriteTimer1(TIMER1_START);
    ticks_left++;
    ticks_left_C++;
#elif defined(MAIN_PIC)
//    if ( timer1_extender > 10) {
//        // Request sensor data for parallel calculations
//        i2cMstrMsgState = I2CMST_LOCAL_WALLSENSOR;
//        i2c_master_recv(0x0A, 0x15, 0x4E);
//        timer1_extender = 0;
//    } else {
//        timer1_extender ++;
//    }
    
    WriteTimer1(0);
#else
    WriteTimer1(0);
#endif

}