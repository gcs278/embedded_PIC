#include "maindefs.h"
#ifndef __XC8
#include <usart.h>
#else
#include <plib/usart.h>
#endif
#include "my_uart.h"
#include "my_i2c.h"
static uart_comm *uc_ptr;

void uart_recv_state(unsigned char byte) {

    uc_ptr->buffer[uc_ptr->buflen] = byte;
    uc_ptr->buflen++;
    
    switch (uc_ptr->state) {

        case UART_STATE_HEADER1:
            if ( byte == UART_HEADER1) {
#if defined (ARM_PIC)
                while(BusyUSART());
                WriteUSART(0x01);
#endif
                uc_ptr->state = UART_STATE_HEADER2;
            }
            break;

        case UART_STATE_HEADER2:
            if ( byte == UART_HEADER2 ) {
#if defined (ARM_PIC)
                while(BusyUSART());
                WriteUSART(0x02);
#endif
#if defined (ARM_PIC)
                uc_ptr->state = UART_STATE_COUNT;
#elif defined (MAIN_PIC)
                uc_ptr->state = UART_STATE_MSGTYPE;
#endif
            }
            else {
                uc_ptr->state = UART_STATE_HEADER1;
            }
            break;

        case UART_STATE_MSGTYPE:
            #if defined (ARM_PIC)
            while(BusyUSART());
            WriteUSART(0x03);
#endif
            uc_ptr->msgtype = byte;
            uc_ptr->state = UART_STATE_COUNT;
            break;

        case UART_STATE_COUNT:
            #if defined (ARM_PIC)
            while(BusyUSART());
            WriteUSART(0x04);
#endif
            uc_ptr->count = byte;
#if defined (ARM_PIC)
                uc_ptr->state = UART_STATE_LENGTH;
#elif defined (MAIN_PIC)
                uc_ptr->state = UART_STATE_FOOTER;
#endif    
             break;

        case UART_STATE_LENGTH:
            #if defined (ARM_PIC)
            while(BusyUSART());
            WriteUSART(0x05);
#endif
            uc_ptr->data_length = byte;
            uc_ptr->state = UART_STATE_DATA;
            break;
            
        case UART_STATE_DATA:
            #if defined (ARM_PIC)
            while(BusyUSART());
            WriteUSART(0x06);
#endif
            // Increment because we read 1 more byte
            uc_ptr->data_read += 1;
            // Keep reading data until
            if(9 == uc_ptr->data_read) {
                uc_ptr->state = UART_STATE_FOOTER;
            }
            
            break;
        case UART_STATE_FOOTER:
            #if defined (ARM_PIC)
            while(BusyUSART());
            WriteUSART(0x07);
#endif
            if ( byte == UART_FOOTER ) {
                uc_ptr->state = UART_STATE_HEADER1;
                LATBbits.LATB7 = !LATBbits.LATB7;

                int i = 0; // Won't work without this?
#if defined (ARM_PIC)
                unsigned char temp[MAXUARTBUF];

                for (i = 0; i < MAXUARTBUF; ++i)
                    temp[i] = uc_ptr->buffer[i];
        
                int length = uc_ptr->buflen;
                int length = 10;

                start_i2c_slave_reply(length, temp);
#elif defined (MAIN_PIC)
                ToMainHigh_sendmsg(1, MSGT_I2C_DATA, (void *) uc_ptr->buffer);
#endif
                // ToMainHigh_sendmsg(length, MSGT_I2C_DATA, (void *) uc_);
                //ToMainLow_sendmsg(uc_ptr->buflen, MSGT_UART_DATA, (void *) uc_ptr->buffer);
            }
            else {
                uc_ptr->state = UART_STATE_HEADER1;
            }
            uc_ptr->buflen = 0;
            uc_ptr->data_read = 0;
            break;
        default:
            break;
    }
    
}

void uart_recv_int_handler() {
    if (DataRdyUSART()) {
        uart_recv_state(ReadUSART());
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
    uc_ptr->data_read = 0;
    uc_ptr->state = UART_STATE_HEADER1;
}