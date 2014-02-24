#include "maindefs.h"
#ifndef __XC8
#include <usart.h>
#else
#include <plib/usart.h>
#endif
#include "my_uart.h"

static uart_comm *uc_ptr;

void uart_recv_state(unsigned char byte) {

    switch (uc_ptr->state) {

        case UART_STATE_HEADER1:
            if ( byte == UART_HEADER1) {
                
                uc_ptr->state = UART_STATE_HEADER2;
            }
            break;

        case UART_STATE_HEADER2:
            if ( byte == UART_HEADER2 ) {
           
#if defined (MAIN_PIC)
                uc_ptr->state = UART_STATE_MSGTYPE;
#elif defined (ARM_PIC)
                uc_ptr->state = UART_STATE_LENGTH;
#endif
            }
            else {
                uc_ptr->state = UART_STATE_HEADER1;
            }
            break;

        case UART_STATE_MSGTYPE:
            uc_ptr->msgtype = byte;
            uc_ptr->state = UART_STATE_LENGTH;
            break;

        case UART_STATE_LENGTH:
            uc_ptr->data_length = byte;
#if defined (MAIN_PIC)
            uc_ptr->state = UART_STATE_FOOTER;
#elif defined (ARM_PIC)
            uc_ptr->state = UART_STATE_DATA;
#endif
            break;
            
        case UART_STATE_DATA:
            // read data while loop
            break;
        case UART_STATE_FOOTER:
            if ( byte == UART_FOOTER ) {
                uc_ptr->state = UART_STATE_HEADER1;
                //LATBbits.LATB7 = !LATBbits.LATB7;
                ToMainHigh_sendmsg(uc_ptr->buflen, MSGT_I2C_DATA, (void *) uc_ptr->buffer);
                //ToMainLow_sendmsg(uc_ptr->buflen, MSGT_UART_DATA, (void *) uc_ptr->buffer);
            }
            else {
                uc_ptr->state = UART_STATE_HEADER1;
            }
            uc_ptr->buflen = 0;
            break;
        default:
            break;
    }
    
}

void uart_recv_int_handler() {
    if (DataRdyUSART()) {
        uc_ptr->buffer[uc_ptr->buflen] = ReadUSART();
        
        // WriteUSART(uc_ptr->buffer[uc_ptr->buflen]);
        uart_recv_state(uc_ptr->buffer[uc_ptr->buflen]);
        uc_ptr->buflen++;
//        // check if a message should be sent
//        if (uc_ptr->buflen == MAXUARTBUF) {
//
//            // ToMainLow_sendmsg(uc_ptr->buflen, MSGT_UART_DATA, (void *) uc_ptr->buffer);
//            uc_ptr->buflen = 0;
//        }
    }
    if (USART_Status.OVERRUN_ERROR == 1) {
        // we've overrun the USART and must reset
        // send an error message for this
        RCSTAbits.CREN = 0;
        RCSTAbits.CREN = 1;
        ToMainLow_sendmsg(0, MSGT_OVERRUN, (void *) 0);
    }
}

void init_uart_recv(uart_comm *uc) {
    uc_ptr = uc;
    uc_ptr->buflen = 0;
    uc_ptr->state = UART_STATE_HEADER1;
}