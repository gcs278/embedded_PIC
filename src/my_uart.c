#include "maindefs.h"
#ifndef __XC8
#include <usart.h>
#else
#include <plib/usart.h>
#endif
#include "my_uart.h"
#include "my_i2c.h"
#include <string.h>
#include "my_i2c_master.h"

static uart_comm *uc_ptr;

void uart_recv_state(unsigned char byte) {
    
    switch (uc_ptr->state) {

        case UART_STATE_HEADER1:
            if ( byte == UART_HEADER1) {
//                WriteUSART(0x01);
                uc_ptr->state = UART_STATE_HEADER2;
            }
            break;

        case UART_STATE_HEADER2:
            if ( byte == UART_HEADER2 ) {
                //WriteUSART(0x02);
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
            uc_ptr->buflen = 0;
            uc_ptr->data_read = 0;
//            WriteUSART(0x03);
            uc_ptr->msgtype = byte;
            uc_ptr->buffer[0] = byte;
            uc_ptr->state = UART_STATE_COUNT;
            break;

        case UART_STATE_COUNT:
//            WriteUSART(0x04);
            uc_ptr->count = byte;
#if defined (ARM_PIC)
                uc_ptr->state = UART_STATE_LENGTH;
                uc_ptr->buffer[0] = byte;
#elif defined (MAIN_PIC)
                uc_ptr->buffer[1] = byte;
                uc_ptr->state = UART_STATE_FOOTER;
#endif    
             break;

        case UART_STATE_LENGTH:
//            WriteUSART(0x05);
            uc_ptr->buffer[1] = byte;
            uc_ptr->data_length = byte;
            uc_ptr->buflen = 0;
            uc_ptr->data_read = 0;
            uc_ptr->state = UART_STATE_DATA;
            break;
            
        case UART_STATE_DATA:
//            WriteUSART(0x06);
            
            // Store the byte into buffer
            if ( uc_ptr->data_read + 2 < MAXUARTBUF ) {             
                uc_ptr->buffer[uc_ptr->data_read+2] = byte;
            }
            
            // Increment because we read 1 more byte
            uc_ptr->data_read += 1;
            
            // Keep reading data until, 9 BYTES for now
            if(UARTDATALEN == uc_ptr->data_read) {
                uc_ptr->state = UART_STATE_FOOTER;
            }
            
            break;
        case UART_STATE_FOOTER:
            // WriteUSART(0x07);
            if ( byte == UART_FOOTER ) {
            // WriteUSART(0x08);
                uc_ptr->state = UART_STATE_HEADER1;
                //LATBbits.LATB7 = !LATBbits.LATB7;

#if defined (ARM_PIC)
                // Copy over into a buffer before passing
                unsigned char temp[I2CMSGLEN];
                int i;
                for (i = 0; i < I2CMSGLEN; i++) {
                    temp[i] = uc_ptr->buffer[i];
                }
                LATB = 7; // Sequence 7
                LATAbits.LA0 = 0;
                // Put it in the roverDataBuf
                ToMainLow_sendmsg(I2CMSGLEN, MSGT_BUF_PUT_DATA, (void *) temp);
#elif defined (MAIN_PIC)
                // Wait for first to finish
                
                LATBbits.LATB0 = 1;
                LATBbits.LATB1 = 0;
                LATBbits.LATB2 = 0; // Sequence 1
                LATAbits.LA0 = 0;

                i2c_master_cmd message;

                message.msgType = uc_ptr->buffer[0];
                message.msgCount = uc_ptr->buffer[1];


                //if ( i2c_master_busy() == 0 )
                    // Put the message in the queue
                while(i2c_master_busy());
                putQueue(&i2c_q,message);
                //ToMainLow_sendmsg(2, MSGT_BUF_PUT_DATA, (void *) uc_ptr->buffer);
#endif
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
#ifdef __USE18F46J50
    if (DataRdy1USART()) {
        // UART state machine
        uart_recv_state(Read1USART());
    }
    if (USART1_Status.OVERRUN_ERROR == 1) {
        // we've overrun the USART and must reset
        // send an error message for this
        RCSTAbits.CREN = 0;
        RCSTAbits.CREN = 1;
        ToMainLow_sendmsg(0, MSGT_OVERRUN, (void *) 0);
    }
#else
    if (DataRdyUSART()) {
        // UART state machine
        uart_recv_state(ReadUSART());
    }
    if (USART_Status.OVERRUN_ERROR == 1) {
        // we've overrun the USART and must reset
        // send an error message for this
        RCSTAbits.CREN = 0;
        RCSTAbits.CREN = 1;
        ToMainLow_sendmsg(0, MSGT_OVERRUN, (void *) 0);
    }
#endif

}

void uart_send_int_handler() {
    //LATBbits.LATB7 = 0;
    // Make sure we still have data to send
    if(uc_ptr->sendSize > 0) {
#ifdef __USE18F46J50
        Write1USART(uc_ptr->sendBuffer[uc_ptr->sendCurrent]);
#else
        WriteUSART(uc_ptr->sendBuffer[uc_ptr->sendCurrent]);
#endif
        


        // Flag interrupt for next byte
        PIE1bits.TXIE = 1;

        // Increment/decrement size and index
        uc_ptr->sendCurrent++;
        uc_ptr->sendSize--;
    }
}

void uart_send_data(unsigned char* data, int size) {
#if defined (MOTOR_PIC)
    // Put the data on
    int i = 0;
    for(i; i < size; i++)
        uc_ptr->sendBuffer[i] = data[i];

    uc_ptr->sendSize = size;
#else
    //LATB = 6; // Sequence 6
    LATAbits.LA0 = 0;
    // Copy over the data to send to UART buffer
    uc_ptr->sendBuffer[0] = UART_HEADER1;
    uc_ptr->sendBuffer[1] = UART_HEADER2;

    // Put the data on
    int i = 0;
    for(i; i < size; i++)
        uc_ptr->sendBuffer[i+2] = data[i];

    // Put the footer on
    uc_ptr->sendBuffer[i+2] = UART_FOOTER;

    // Set the params
    uc_ptr->sendSize = size+3;
#endif
    
    uc_ptr->sendCurrent = 0;

    // Flag the interrupt
    PIE1bits.TXIE = 1;
}

void init_uart_recv(uart_comm *uc) {
    uc_ptr = uc;
    uc_ptr->buflen = 0;
    uc_ptr->data_read = 0;
    uc_ptr->state = UART_STATE_HEADER1;
}
