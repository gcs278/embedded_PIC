#include "maindefs.h"
#include <stdio.h>
#include "uart_thread.h"


#include "my_uart.h"
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
        //int length =
        //start_i2c_slave_reply(1, msgbuffer);

#elif defined(MAIN_PIC)
        LATB = 2; // Sequence 2
        // UART to I2C Master request
        ToMainHigh_sendmsg(length, MSGT_I2C_DATA, (void *) msgbuffer);
#endif

        i++;
    }
}

// Sending UART data
int uart_sendthread(int length, unsigned char *msgbuffer, unsigned char count) {
    unsigned char buf[I2CMSGLEN+1];
    LATB = 1; // Sequence 9
    LATAbits.LA0 = 1;
    // Put the count on
    buf[0] = count;

    // Put the data on
    int i;
    for (i=0; i < I2CMSGLEN; i++) {
        buf[i+1] = msgbuffer[i]; // Start data
    }

    // Send out
    uart_send_data(buf,I2CMSGLEN+1);
}