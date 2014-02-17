#include "maindefs.h"
#include <stdio.h>
#include "timer0_thread.h"
#include "messages.h"

// This is a "logical" thread that processes messages from TIMER0
// It is not a "real" thread because there is only the single main thread
// of execution on the PIC because we are not using an RTOS.

void init_timer0_lthread(timer0_thread_struct *tptr) {
    tptr->msgcount = 0;
}

int timer0_lthread(timer0_thread_struct *tptr, int msgtype, int length, unsigned char *msgbuffer) {
//    unsigned int *msgval;
//
//    msgval = (unsigned int *) msgbuffer;

       signed char retval;

    tptr->msgcount++;
    // Every tenth message we get from timer1 we
    // send something to the High Priority Interrupt
    if ((tptr->msgcount % 10) == 9) {
        retval = FromMainHigh_sendmsg(sizeof (tptr->msgcount), MSGT_MAIN1, (void *) &(tptr->msgcount));
        if (retval < 0) {
            // We would handle the error here
        }
    }

}