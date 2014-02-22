#include "maindefs.h"
#include <stdio.h>
#include "uart_thread.h"
#include "interrupts.h"
#include "messages.h"

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

#if defined(ARM_PIC)
                    // SEND I2C Message TO UART
                    msgbuffer[0] = last_reg_recvd;
                    ToMainLow_sendmsg(1, MSGT_UART_SEND, (void *) msgbuffer);

                    // start_i2c_slave_reply(length, msgbuffer);
#elif defined(SENSOR_PIC)

#elif defined(MAIN_PIC)

#elif defined(MOTOR_PIC)
        if(msgbuffer[0] == 0x01){
             WriteUSART(1);
             WriteUSART(129);
        }
        else if (msgbuffer[0] == 0x05) {
            WriteUSART(0x00);
        }
        else if (msgbuffer[0] == 'A'){
            WriteUSART(219);
            WriteUSART(35);
        }
        else if (msgbuffer[0] == 'D'){
            WriteUSART(92);
            WriteUSART(163);
        }
        else if (msgbuffer[0] == 'S') {
            WriteUSART(77);
            WriteUSART(204);
        }
        ToMainHigh_sendmsg(length, MSGT_I2C_DATA, (void *) msgbuffer);
#endif

        i++;
    }
}

// Sending UART data
int uart_sendthread(int msgtype, int length, unsigned char *msgbuffer) {

    
}