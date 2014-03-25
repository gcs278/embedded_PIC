/*
 TEAM 1 - GRANT SPENCE, TAYLOR MCGOUGH, MARTIN ANILANE, MATT O'NEIL
 Milestone 1 = 2/11/2014
 */
#include <string.h>
#include "maindefs.h"
#include "messages.h"
#ifndef __XC8
#include <i2c.h>
#else
#include <plib/i2c.h>
#endif
#include "my_i2c_master.h"
#include "interrupts.h"
#include "i2c_queue.h"
#ifdef __USE18F46J50
//#include "PIC18F46J50.h"
#else
#include "PIC18F45J10.h"
#endif
static i2c_master_comm *ic_ptr;
i2c_mode mode;
unsigned char slave_address;

// Configure for I2C Master mode -- the variable "slave_address" should be stored in
//   i2c_comm (as pointed to by ic_ptr) for later use.

void i2c_configure_master(void) {
    // set clock and data as inputs
#ifdef __USE18F46J50
    SCL1 = 1;
    SDA1 = 1;
#else
    I2C1_SCL = 1;
    I2C1_SDA = 1;
#endif


 
    // configure status bits
    SSP1STATbits.SMP = 1;
    SSP1STATbits.CKE = 0;


#ifdef __USE18F46J50
    SSP1ADD = 119;
#else
    // set frequency to 100kHz
    SSP1ADD = 29;
#endif



    // set to master mode
    SSP1CON1 = 0x8;
    SSP1CON2 = 0;

    // start I2C Master mode
    SSP1CON1bits.SSPEN = 1;

    ic_ptr->state = IDLE; 
}

// Sending in I2C Master mode [slave write]
// 		returns -1 if the i2c bus is busy
// 		return 0 otherwise
// Will start the sending of an i2c message -- interrupt handler will take care of
//   completing the message send.  When the i2c message is sent (or the send has failed)
//   the interrupt handler will send an internal_message of type MSGT_MASTER_SEND_COMPLETE if
//   the send was successful and an internal_message of type MSGT_MASTER_SEND_FAILED if the
//   send failed (e.g., if the slave did not acknowledge).  Both of these internal_messages
//   will have a length of 0.
// The subroutine must copy the msg to be sent from the "msg" parameter below into
//   the structure to which ic_ptr points [there is already a suitable buffer there].
unsigned char sync_mode=0, slew=0, add1,w,data,status,length;

unsigned char I2C_Send[21] = "MICROCHIP:I2C_MASTER" ;
unsigned char I2C_Recv[21];
unsigned char i2c_master_send(unsigned char length, unsigned char *msg, unsigned char slave_address)
{
    mode = MASTER_WRITE;
    memcpy(ic_ptr->buffer, msg, length);
    ic_ptr->buffer_length = length;
    ic_ptr->buffer_index = 0;
    ic_ptr->slave_address = slave_address << 1;
    SSP1CON2bits.SEN = 1;
    ic_ptr->state = START_BIT;
    LATDbits.LATD7 = !LATDbits.LATD7;
    return 0;
}

// Receiving in I2C Master mode [slave read]
// 		returns -1 if the i2c bus is busy
// 		return 0 otherwise
// Will start the receiving of an i2c message -- interrupt handler will take care of
//   completing the i2c message receive.  When the receive is complete (or has failed)
//   the interrupt handler will send an internal_message of type MSGT_MASTER_RECV_COMPLETE if
//   the receive was successful and an internal_message of type MSGT_MASTER_RECV_FAILED if the
//   receive failed (e.g., if the slave did not acknowledge).  In the failure case
//   the internal_message will be of length 0.  In the successful case, the
//   internal_message will contain the message that was received [where the length
//   is determined by the parameter passed to i2c_master_recv()].
// The interrupt handler will be responsible for copying the message received into

unsigned char i2c_master_recv(unsigned char length, unsigned char data, unsigned char slave_address)
{
    // Check if we are in the middle of something
    if ( ic_ptr->state != IDLE) {
        return 0;
    }

    mode = MASTER_READ;

    ic_ptr->buffer_length = length;
    ic_ptr->buffer_index = 0;
    ic_ptr->slave_address = slave_address << 1;
    ic_ptr->address = data;
    SSP1CON2bits.SEN = 1;
    ic_ptr->state = START_BIT;
    
    return 1;
}

void i2c_master_handler()
{
    switch (ic_ptr->state)
    {
        case START_BIT:
        {

            SSPBUF = (ic_ptr->slave_address);
            ic_ptr->state = WRITE_ADDRESS;
            break;
        } 

        case WRITE_ADDRESS:
        {
            if (1 == SSPCON2bits.ACKSTAT)
            {
                    // NACK 
            }
            else
            {
                if (mode == MASTER_WRITE )
                {
                    SSPBUF = ic_ptr->buffer[ic_ptr->buffer_index];
                } 
                else if (mode == MASTER_READ )
                {
                    SSPBUF = ic_ptr->address;
                }
                ic_ptr->state = DATA;
            }
            break;
        }

        case DATA:
        {

        if (1 == SSPCON2bits.ACKSTAT)
        {
            // NACK
        }
        else
        {
            if (mode == MASTER_WRITE )
            {
                        
                ic_ptr->buffer_index++;

                if (ic_ptr->buffer_index < ic_ptr->buffer_length)
                {     
                    SSPBUF = ic_ptr->buffer[ic_ptr->buffer_index];
                    ic_ptr->state = DATA;
                }
                else
                {
                    SSPCON2bits.PEN = 1;
                    ic_ptr->state = STOP_MESSAGE;
                }
            }
            else if (mode == MASTER_READ )
            {
                SSPCON2bits.RSEN = 1;
                ic_ptr->state = RESTART;
            }

        }
        break;
        }
                
        case RESTART:
        {
            SSPBUF = (ic_ptr->slave_address) | 0x01;
            ic_ptr->state = READ_ADDRESS;
            break;
        } 

        case READ_ADDRESS:
        {
            if (1 == SSPCON2bits.ACKSTAT)
            {
                    // NACK 
            } else
            {        
                SSPCON2bits.RCEN = 1;
                ic_ptr->state = SLAVE_RESPONSE;
            }
            break;
        }
        case SLAVE_RESPONSE:
        {
            ic_ptr->buffer[ic_ptr->buffer_index] = SSPBUF;

                
            ic_ptr->buffer_index++;    
            if (ic_ptr->buffer_index < ic_ptr->buffer_length)
            {
                SSPCON2bits.ACKDT = 0;
                ic_ptr->state = ACK;

            }
            else
            {
                SSPCON2bits.ACKDT = 1;
                ic_ptr->state = NACK;
            }
            SSPCON2bits.ACKEN = 1;
            break;
            } 

        case ACK:
        {
            SSPCON2bits.RCEN = 1;
            ic_ptr->state = SLAVE_RESPONSE;
            break;
        } 
        case NACK:
        {
            SSPCON2bits.PEN = 1;
            ic_ptr->state = STOP_MESSAGE;
            break;
        }

        case STOP_MESSAGE:
        {
            if ( mode == MASTER_WRITE )
            {
                ToMainHigh_sendmsg(0, MSGT_I2C_MASTER_SEND_COMPLETE, 0);
            } else if (mode == MASTER_READ )
            {
                ToMainHigh_sendmsg(ic_ptr->buffer_index, MSGT_I2C_MASTER_RECV_COMPLETE, ic_ptr->buffer);
            }

            mode = MASTER_IDLE;
            ic_ptr->state = IDLE;
            break;
        }

        default:
        {
            break;
        } // End default case
    }

}



/*
void start_i2c_slave_reply(unsigned char length, unsigned char *msg) {

    for (ic_ptr->outbuflen = 0; ic_ptr->outbuflen < length; ic_ptr->outbuflen++) {
        ic_ptr->outbuffer[ic_ptr->outbuflen] = msg[ic_ptr->outbuflen];
    }
    ic_ptr->outbuflen = length;
    ic_ptr->outbufind = 1; // point to the second byte to be sent

    // put the first byte into the I2C peripheral
    SSPBUF = ic_ptr->outbuffer[0];
    // we must be ready to go at this point, because we'll be releasing the I2C
    // peripheral which will soon trigger an interrupt
    SSPCON1bits.CKP = 1;

}

// an internal subroutine used in the slave version of the i2c_int_handler

void handle_start(unsigned char data_read) {
    ic_ptr->event_count = 1;
    ic_ptr->buflen = 0;
    // check to see if we also got the address
    if (data_read) {
        if (SSPSTATbits.D_A == 1) {
            // this is bad because we got data and
            // we wanted an address
            ic_ptr->status = I2C_IDLE;
            ic_ptr->error_count++;
            ic_ptr->error_code = I2C_ERR_NOADDR;
        } else {
            if (SSPSTATbits.R_W == 1) {
                ic_ptr->status = I2C_SLAVE_SEND;
            } else {
                ic_ptr->status = I2C_RCV_DATA;
            }
        }
    } else {
        ic_ptr->status = I2C_STARTED;
    }
}

// this is the interrupt handler for i2c -- it is currently built for slave mode
// -- to add master mode, you should determine (at the top of the interrupt handler)
//    which mode you are in and call the appropriate subroutine.  The existing code
//    below should be moved into its own "i2c_slave_handler()" routine and the new
//    master code should be in a subroutine called "i2c_master_handler()"

void i2c_int_handler() {
    unsigned char i2c_data;
    unsigned char data_read = 0;
    unsigned char data_written = 0;
    unsigned char msg_ready = 0;
    unsigned char msg_to_send = 0;
    unsigned char overrun_error = 0;
    unsigned char error_buf[3];

    // clear SSPOV
    if (SSPCON1bits.SSPOV == 1) {
        SSPCON1bits.SSPOV = 0;
        // we failed to read the buffer in time, so we know we
        // can't properly receive this message, just put us in the
        // a state where we are looking for a new message
        ic_ptr->status = I2C_IDLE;
        overrun_error = 1;
        ic_ptr->error_count++;
        ic_ptr->error_code = I2C_ERR_OVERRUN;
    }
    // read something if it is there
    if (SSPSTATbits.BF == 1) {
        i2c_data = SSPBUF;
        data_read = 1;
    }

    // toggle an LED
    //LATBbits.LATB2 = !LATBbits.LATB2;

    if (!overrun_error) {
        switch (ic_ptr->status) {
            case I2C_IDLE:
            {
                // ignore anything except a start
                if (SSPSTATbits.S == 1) {
                    handle_start(data_read);
                    // if we see a slave read, then we need to handle it here
                    if (ic_ptr->status == I2C_SLAVE_SEND) {
                        data_read = 0;
                        msg_to_send = 1;
                    }
                }
                break;
            }
            case I2C_STARTED:
            {
                // in this case, we expect either an address or a stop bit
                if (SSPSTATbits.P == 1) {
                    // we need to check to see if we also read an
                    // address (a message of length 0)
                    ic_ptr->event_count++;
                    if (data_read) {
                        if (SSPSTATbits.D_A == 0) {
                            msg_ready = 1;
                        } else {
                            ic_ptr->error_count++;
                            ic_ptr->error_code = I2C_ERR_NODATA;
                        }
                    }
                    ic_ptr->status = I2C_IDLE;
                } else if (data_read) {
                    ic_ptr->event_count++;
                    if (SSPSTATbits.D_A == 0) {
                        if (SSPSTATbits.R_W == 0) { // slave write
                            ic_ptr->status = I2C_RCV_DATA;
                        } else { // slave read
                            ic_ptr->status = I2C_SLAVE_SEND;
                            msg_to_send = 1;
                            // don't let the clock stretching bit be let go
                            data_read = 0;
                        }
                    } else {
                        ic_ptr->error_count++;
                        ic_ptr->status = I2C_IDLE;
                        ic_ptr->error_code = I2C_ERR_NODATA;
                    }
                }
                break;
            }
            case I2C_SLAVE_SEND:
            {
                if (ic_ptr->outbufind < ic_ptr->outbuflen) {
                    SSPBUF = ic_ptr->outbuffer[ic_ptr->outbufind];
                    ic_ptr->outbufind++;
                    data_written = 1;
                } else {
                    // we have nothing left to send
                    ic_ptr->status = I2C_IDLE;
                }
                break;
            }
            case I2C_RCV_DATA:
            {
                // we expect either data or a stop bit or a (if a restart, an addr)
                if (SSPSTATbits.P == 1) {
                    // we need to check to see if we also read data
                    ic_ptr->event_count++;
                    if (data_read) {
                        if (SSPSTATbits.D_A == 1) {
                            ic_ptr->buffer[ic_ptr->buflen] = i2c_data;
                            ic_ptr->buflen++;
                            msg_ready = 1;
                        } else {
                            ic_ptr->error_count++;
                            ic_ptr->error_code = I2C_ERR_NODATA;
                            ic_ptr->status = I2C_IDLE;
                        }
                    } else {
                        msg_ready = 1;
                    }
                    ic_ptr->status = I2C_IDLE;
                } else if (data_read) {
                    ic_ptr->event_count++;
                    if (SSPSTATbits.D_A == 1) {
                        ic_ptr->buffer[ic_ptr->buflen] = i2c_data;
                        ic_ptr->buflen++;
                    } else // a restart
                        {
                        if (SSPSTATbits.R_W == 1) {
                            ic_ptr->status = I2C_SLAVE_SEND;
                            msg_ready = 1;
                            msg_to_send = 1;
                            // don't let the clock stretching bit be let go
                            data_read = 0;
                        } else { // bad to recv an address again, we aren't ready
                            ic_ptr->error_count++;
                            ic_ptr->error_code = I2C_ERR_NODATA;
                            ic_ptr->status = I2C_IDLE;
                        }
                    }
                }
                break;
            }
        }
    }

    // release the clock stretching bit (if we should)
    if (data_read || data_written) {
        // release the clock
        if (SSPCON1bits.CKP == 0) {
            SSPCON1bits.CKP = 1;
        }
    }

    // must check if the message is too long, if
    if ((ic_ptr->buflen > MAXI2CBUF - 2) && (!msg_ready)) {
        ic_ptr->status = I2C_IDLE;
        ic_ptr->error_count++;
        ic_ptr->error_code = I2C_ERR_MSGTOOLONG;
    }

    if (msg_ready) {
        ic_ptr->buffer[ic_ptr->buflen] = ic_ptr->event_count;
        ToMainHigh_sendmsg(ic_ptr->buflen + 1, MSGT_I2C_DATA, (void *) ic_ptr->buffer);
        ic_ptr->buflen = 0;
    } else if (ic_ptr->error_count >= I2C_ERR_THRESHOLD) {
        error_buf[0] = ic_ptr->error_count;
        error_buf[1] = ic_ptr->error_code;
        error_buf[2] = ic_ptr->event_count;
        ToMainHigh_sendmsg(sizeof (unsigned char) *3, MSGT_I2C_DBG, (void *) error_buf);
        ic_ptr->error_count = 0;
    }
    if (msg_to_send) {
        int length;
        unsigned char msgbuffer[MSGLEN + 1];
        length = 2;
        //for(int i = 0; i <3; i++)
        msgbuffer[0] = returnADCValue();
        //msgbuffer[0] = 0x04;

//        switch (ic_ptr->buflen) {
//                        case 0xaa:
//                        {
//                            length = 2;
//                            msgbuffer[0] = 0x55;
//                            //msgbuffer[0] = returnADCValue();
//                            msgbuffer[1] = 0xAA;
//                            break;
//                        }
//                        case 0xa8:
//                        {
//                            length = 1;
//                            msgbuffer[0] = 0x3A;
//                            //msgbuffer[0] = returnADCValue();
//                            break;
//                        }
//                        case 0xa9:
//                        {
//                            length = 1;
//                            msgbuffer[0] = 0xA3;
//                            // msgbuffer[0] = returnADCValue();
//                            break;
//                        }
//                    };
        start_i2c_slave_reply(length, msgbuffer);
        msg_to_send = 0;
    }
}

*/
// set up the data structures for this i2c code
// should be called once before any i2c routines are called


void init_i2c_master(i2c_master_comm *ic) {
    mode = MASTER_IDLE;

    ic_ptr = ic;
    
    ic_ptr->state = IDLE;
    //ic_ptr->buffer = 0
    //ic_ptr->buffer_length = 0;
}

unsigned char i2c_master_busy() {
    if( ic_ptr->state == IDLE ) {
        return 0;
    }
    else {
        return 1;
    }
}