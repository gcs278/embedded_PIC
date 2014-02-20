#include "maindefs.h"
#include <stdio.h>
#include "uart_thread.h"
#include "interrupts.h"
#include "messages.h"
// This is a "logical" thread that processes messages from the UART
// It is not a "real" thread because there is only the single main thread
// of execution on the PIC because we are not using an RTOS.
int i = 0;
int uart_lthread(uart_thread_struct *uptr, int msgtype, int length, unsigned char *msgbuffer) {
    if (msgtype == MSGT_OVERRUN) {
    } else if (msgtype == MSGT_UART_DATA) {
        // print the message (this assumes that the message
        // 		was a printable string)
        msgbuffer[length] = '\0'; // null-terminate the array as a string
        
        ToMainHigh_sendmsg(length, MSGT_I2C_DATA, (void *) msgbuffer);
        
        //putsUSART(msgbuffer);
//        if(msgbuffer[0] == 'W'){
//             WriteUSART(1);
//             WriteUSART(129);
//        }
//        else if (msgbuffer[0] == 'Q')
//            WriteUSART(0x00);
//        else if (msgbuffer[0] == 'A'){
//            WriteUSART(219);
//            WriteUSART(35);
//        }
//        else if (msgbuffer[0] == 'D'){
//            WriteUSART(92);
//            WriteUSART(163);
//        }
//        else if (msgbuffer[0] == 'S') {
//            WriteUSART(77);
//            WriteUSART(204);
//        }
        i++;
    }
}
