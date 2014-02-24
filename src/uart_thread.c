#include "maindefs.h"
#include <stdio.h>
#include "uart_thread.h"
#include "interrupts.h"
#include "messages.h"
#include "my_i2c.h"
#include <plib/usart.h>
// This is a "logical" thread that processes messages from the UART
// It is not a "real" thread because there is only the single main thread
// of execution on the PIC because we are not using an RTOS.
int i = 0;

// Receiving UART data
int uart_lthread(uart_thread_struct *uptr, int msgtype, int length, unsigned char *msgbuffer) {
    if (msgtype == MSGT_OVERRUN) {
    } else if (msgtype == MSGT_UART_DATA) {
        // print the message (this assumes that the message
        // 		was a printable string)
        msgbuffer[length] = '\0'; // null-terminate the array as a string
//LATBbits.LATB7 = !LATBbits.LATB7;
#if defined(ARM_PIC)
        // UART to I2C slave reply
        start_i2c_slave_reply(1, msgbuffer);

#elif defined(MAIN_PIC)
        // UART to I2C Master request
        ToMainHigh_sendmsg(length, MSGT_I2C_DATA, (void *) msgbuffer);
#endif

        i++;
    }
}

// Sending UART data
int uart_sendthread(int length, unsigned char *msgbuffer) {
    while(BusyUSART());
    WriteUSART(UART_HEADER1);
    WriteUSART(UART_HEADER2);
    WriteUSART(length);
    //WriteUSART()
}