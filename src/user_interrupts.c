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
    if ( executingEncode && ticks_right_C > 0)
        ticks_right_C--;
    
    if ( executingEncodeLong && ticks_right_C_Long > 0)
        ticks_right_C_Long--;

    if ( ticks_left_C  <= 1 && ticks_right_C <= 1 && executingEncode ) {
        if ( motor_state == RoverMsgMotorForward)
            RoverForward();
        else if ( motor_state == RoverMsgMotorLeft2 || motor_state == RoverMsgMotorRight2 ) {
            motor_encode_lthread(RoverMsgMotorForwardCMDelim+10);
        }
        else
            RoverStop();
         executingEncode = 0;
    }
    else if ( ticks_left_C_Long  <= 1 && ticks_right_C_Long <= 1 && executingEncodeLong ) {
        if ( motor_state == RoverMsgMotorLeft2 ) {
            motor_encode_lthread(RoverMsgMotorRight2);
        }
        else if ( motor_state == RoverMsgMotorRight2 )
        {
            motor_encode_lthread(RoverMsgMotorLeft2);
        }
        else {
            RoverStop();
        }
        
         motor_state = RoverMsgMotorStop;
         executingEncodeLong = 0;
    }
    
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
    
    if ( executingEncode && ticks_left_C > 0)
        ticks_left_C--;
    if ( executingEncodeLong && ticks_left_C_Long > 0)
        ticks_left_C_Long--;

    ticks_left_total++;

    if ( ticks_left_C  <= 1 && ticks_right_C <= 1 && executingEncode ) {
        if ( motor_state == RoverMsgMotorForward)
            RoverForward();
        
        else if ( motor_state == RoverMsgMotorLeft2 || motor_state == RoverMsgMotorRight2 ) {
            motor_encode_lthread(RoverMsgMotorForwardCMDelim+10);
        }
        else
            RoverStop();

         executingEncode = 0;
    }
    else if ( ticks_right_C_Long  <= 1 && ticks_left_C_Long <= 1 && executingEncodeLong ) {
        if ( motor_state == RoverMsgMotorLeft2 ) {
            motor_encode_lthread(RoverMsgMotorRight2);
        }
        else if ( motor_state == RoverMsgMotorRight2 )
        {
            motor_encode_lthread(RoverMsgMotorLeft2);
        }
        else {
         RoverStop();
        }

         motor_state = RoverMsgMotorStop;
         executingEncodeLong = 0;
    }
    
#elif defined(MAIN_PIC)
    if ( !i2c_master_busy() )
        ToMainHigh_sendmsg(10, MSGT_GET_SENSOR_DATA, (void *) 0);
    
    if ( timer1_extender > 100 && tempWallCorrection == 0 ) {
        tempWallCorrection = 1;
    }
    
    timer1_extender++;

    WriteTimer1(0);
#else
    WriteTimer1(0);
#endif

}