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
        //int length =
        //start_i2c_slave_reply(1, msgbuffer);

#elif defined(MAIN_PIC)
        // UART to I2C Master request
        ToMainHigh_sendmsg(length, MSGT_I2C_DATA, (void *) msgbuffer);
#endif

        i++;
    }
}

// Sending UART data
int uart_sendthread(int length, unsigned char *msgbuffer) {
        // Put the I2C request on UART
    while(BusyUSART());
    WriteUSART(0x2B);
    while(BusyUSART());
    WriteUSART(0x9F);
    while(BusyUSART());
    WriteUSART(0x03); // Write Count
    while(BusyUSART());
    WriteUSART(0x04); // Write length
    while(BusyUSART());
    WriteUSART(0x01); // Start data
    while(BusyUSART());
    WriteUSART(0x02);
    while(BusyUSART());
    WriteUSART(0x03);
    while(BusyUSART());
    WriteUSART(0x04); // Write length
    while(BusyUSART());
    WriteUSART(0x05); // Write length
    while(BusyUSART());
    WriteUSART(0x06); // Write length
    while(BusyUSART());
    WriteUSART(0x07); // Write length
    while(BusyUSART());
    WriteUSART(0x08); // Write length
    while(BusyUSART());
    WriteUSART(0x09); // Write length
    while(BusyUSART());
    WriteUSART(0x5C);
}