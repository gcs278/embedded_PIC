/*
 TEAM 1 - GRANT SPENCE, TAYLOR MCGOUGH, MARTIN ANILANE, MATT O'NEIL
 Milestone 1 = 2/11/2014
 */
#include <string.h>
#include <pic18f45j10.h>
#include "maindefs.h"
#include "messages.h"
#ifndef __XC8
#include <i2c.h>
#else
#include <plib/i2c.h>
#endif
#include "my_i2c_master.h"
#include "interrupts.h"

static i2c_master_comm *ic_ptr;

unsigned char slave_addr;

// Configure for I2C Master mode -- the variable "slave_addr" should be stored in
//   i2c_comm (as pointed to by ic_ptr) for later use.

void i2c_configure_master(void) {
        // Make sure the pins are set as input
    I2C1_SCL = 1;
    I2C1_SDA = 1;

    // Set the config bits located in the status register
    SSP1STATbits.SMP = 1;
    SSP1STATbits.CKE = 0;

    // Reset the control registers (just in case)
    SSP1CON1 = 0;
    SSP1CON2 = 0;

    // Set the module for I2C master mode (SSPM field of SSP1CON1)
    SSP1CON1 |= 0b01000;

    // Set the module for 100kHz operation using the given formula:
    // Fscl = Fosc / (4*(SSPxADD+1))
    // Solved for SSPxADD:
    // SSPxADD = (Fosc / (4*Fscl)) - 1
    // With Fscl = 100kHz and Fosc = 12MHz,  SSPxADD = 29
    SSP1ADD = 29;

    // Enable the module
    SSP1CON1bits.SSPEN = 1;
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
unsigned char i2c_master_send(unsigned char length, unsigned char *msg, unsigned char slave_addr)
{
        LATDbits.LATD7 = !LATDbits.LATD7;
                // Set the main state to indicate a write in progress
        ic_ptr->state = MASTER_WRITE;

        // Copy the provided data to the internal buffer
        memcpy(ic_ptr->buffer, msg, length);

        // Save the length of the copied data
        ic_ptr->buffer_length = length;

        // Reset the buffer index
        ic_ptr->buffer_index = 0;

        // Save the slave address
        ic_ptr->slave_addr = slave_addr;

        // Assert a Start condition and move to the next substate
        SSP1CON2bits.SEN = 1;
        ic_ptr->substate = I2C_SUBSTATE_START_SENT;
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

unsigned char i2c_master_recv(unsigned char length, unsigned char data, unsigned char slave_addr)
{
        LATBbits.LATB7 = !LATBbits.LATB7;
            // Set the main state to indicate a read in progress
        ic_ptr->state = MASTER_READ;

        // Save the requested data length
        ic_ptr->buffer_length = length;
                // Copy the provided data to the internal buffer
        //memcpy(ic_ptr->buffer, data, 0x1);


        // Reset the buffer index
        ic_ptr->buffer_index = 0;

        // Save the slave address
        ic_ptr->slave_addr = slave_addr;

        // Save the register
        ic_ptr->register_byte = data;



        // Assert a Start condition and move to the next substate
        SSP1CON2bits.SEN = 1;
        ic_ptr->substate = I2C_SUBSTATE_START_SENT;
    return(0);
}

void i2c_master_handler() {
    // Make sure we are not in an idle state, or else we don't know why this
    // interrupt triggered.
    if (ic_ptr->state != MASTER_IDLE) {
        switch (ic_ptr->substate) {
            case I2C_SUBSTATE_START_SENT:
            {
                // The Start has completed, send the address and W bit (0)
                SSPBUF = (ic_ptr->slave_addr << 1);

                // Move to the next substate
                ic_ptr->substate = I2C_SUBSTATE_ADDR_W_SENT;

                break;
            } // End case I2C_SUBSTATE_START_SENT

            case I2C_SUBSTATE_ADDR_W_SENT:
            {
                // Sending of the address has completed, check the ACK status
                // and send some data.

                // Check for a NACK (ACKSTAT 0 indicates ACK received)
                if (1 == SSPCON2bits.ACKSTAT) {
                    // NACK 
                } else

                {
                    unsigned char first_byte;

                    // The first byte differs for reads and writes
                    if (MASTER_WRITE == ic_ptr->state) {
                        first_byte = ic_ptr->buffer[ic_ptr->buffer_index];
                    } else if (MASTER_READ == ic_ptr->state) {
                        first_byte = ic_ptr->register_byte;
                    }

                    // Send the first byte
                    SSPBUF = first_byte;

                    // Move to the next substate
                    ic_ptr->substate = I2C_SUBSTATE_DATA_SENT;
                }
                break;
            } // End case I2C_SUBSTATE_ADDR_W_SENT

            case I2C_SUBSTATE_DATA_SENT:
            {
                // A data byte has completed, send the next one or finish the
                // write.

                // Check for a NACK (ACKSTAT 0 indicates ACK received)
                if (1 == SSPCON2bits.ACKSTAT) {
                    // NACK 
                }
                else
                {
                    // The path differs for reads and writes
                    if (MASTER_WRITE == ic_ptr->state) {
                        // Increment the data index
                        ic_ptr->buffer_index++;

                        // Check if there is more data to send
                        if (ic_ptr->buffer_index < ic_ptr->buffer_length) {
                            // Send the next byte
                            SSPBUF = ic_ptr->buffer[ic_ptr->buffer_index];

                            // Stay in the same substate (we just sent data)
                            ic_ptr->substate = I2C_SUBSTATE_DATA_SENT;

                        }// Otherwise there is no more data
                        else {
                            // Assert a Stop condition (the write is complete)
                            SSPCON2bits.PEN = 1;

                            // Move to the next substate
                            ic_ptr->substate = I2C_SUBSTATE_STOP_SENT;
                        }
                    }
                    else if (MASTER_READ == ic_ptr->state) {
                        // Assert a Restart condition
                        SSPCON2bits.RSEN = 1;

                        // Move to the next substate
                        ic_ptr->substate = I2C_SUBSTATE_RESTART_SENT;
                    }

                } // End ACK check - else

                break;
            } // End case I2C_SUBSTATE_DATA_SENT

            case I2C_SUBSTATE_ERROR:
                // Both substates indicate a stop has completed
            case I2C_SUBSTATE_STOP_SENT:
            {
                // The Stop has completed, return to idle and send a message to
                // main as appropriate

                // Send a success message to main only if this point was reached
                // through normal operation.
                if (ic_ptr->substate != I2C_SUBSTATE_ERROR) {
                    // Send a success message to main
                    if (MASTER_WRITE == ic_ptr->state) {
                        ToMainHigh_sendmsg(0, MSGT_I2C_MASTER_SEND_COMPLETE, 0);
                    } else if (MASTER_READ == ic_ptr->state) {
                        ToMainHigh_sendmsg(ic_ptr->buffer_index, MSGT_I2C_MASTER_RECV_COMPLETE, ic_ptr->buffer);
                    }

                }// Otherwise send the appropriate error message to main
                else {
                    // Send an error message to main
                    if (MASTER_WRITE == ic_ptr->state) {
                        ToMainHigh_sendmsg(0, MSGT_I2C_MASTER_SEND_FAILED, 0);
                    } else if (MASTER_READ == ic_ptr->state) {
                        ToMainHigh_sendmsg(0, MSGT_I2C_MASTER_RECV_FAILED, 0);
                    }
                }

                // Return to idle states
                ic_ptr->state = MASTER_IDLE;
                ic_ptr->substate = I2C_SUBSTATE_IDLE;

                break;
            } // End case I2C_SUBSTATE_STOP_SENT

            case I2C_SUBSTATE_RESTART_SENT:
            {
                // The restart has completed, send the address and R bit (1)
                SSPBUF = (ic_ptr->slave_addr << 1) | 0x01;

                // Move to the next substate
                ic_ptr->substate = I2C_SUBSTATE_ADDR_R_SENT;

                break;
            } // End case I2C_SUBSTATE_RESTART_SENT

            case I2C_SUBSTATE_ADDR_R_SENT:
            {
                // Sending of the address has completed, check the ACK status
                // and prepare to receive data.


                if (1 == SSPCON2bits.ACKSTAT) {
                    // NACK 
                } else
                {
                    // Prepare to receive a byte
                    SSPCON2bits.RCEN = 1;

                    // Move to the next substate
                    ic_ptr->substate = I2C_SUBSTATE_WAITING_TO_RECEIVE;
                }

                break;
            } // End cases I2C_SUBSTATE_ADDR_R_SENT and I2C_SUBSTATE_ACK_SENT

            case I2C_SUBSTATE_WAITING_TO_RECEIVE:
            {
                // A byte has been received, ACK or NACK it as appropriate.

                // Save the byte
                ic_ptr->buffer[ic_ptr->buffer_index] = SSPBUF;

                // Increment the index (received byte counter)
                ic_ptr->buffer_index++;

                // Check if all requested bytes have been received
                if (ic_ptr->buffer_index < ic_ptr->buffer_length) {
                    // Prepare to ACK the byte
                    SSPCON2bits.ACKDT = 0;

                    // Move to the next substate
                    ic_ptr->substate = I2C_SUBSTATE_ACK_SENT;

                }// Otherwise no more data is needed from the slave
                else {
                    // Prepare to NACK the byte
                    SSPCON2bits.ACKDT = 1;

                    // Move to the next substate
                    ic_ptr->substate = I2C_SUBSTATE_NACK_SENT;
                }

                // Send the ACK/NACK as configured above
                SSPCON2bits.ACKEN = 1;

                break;
            } // End case I2C_SUBSTATE_WAITING_TO_RECEIVE

            case I2C_SUBSTATE_ACK_SENT:
            {
                // Sending of the ACK has completed, prepare to receive a byte
                SSPCON2bits.RCEN = 1;

                // Move to the next substate
                ic_ptr->substate = I2C_SUBSTATE_WAITING_TO_RECEIVE;

                break;
            } // End case I2C_SUBSTATE_ACK_SENT

            case I2C_SUBSTATE_NACK_SENT:
            {
                // Sending of the NACK has completed, assert a Stop to complete
                // the read.
                SSPCON2bits.PEN = 1;

                // Move to the next substate
                ic_ptr->substate = I2C_SUBSTATE_STOP_SENT;

                break;
            } // End case I2C_SUBSTATE_NACK_SENT

            default:
            {
                //i2c_master_handle_error();
                break;
            } // End default case
        }
    } else {
        // The only time we should get an interrupt while in IDLE is if a Stop
        // was asserted after an error.  If that's not what heppened, indicate
        // an error
        if (I2C_SUBSTATE_ERROR != ic_ptr->substate) {
            //SET_I2C_ERROR_PIN();
        }

        ic_ptr->state = MASTER_IDLE;
        ic_ptr->substate = I2C_SUBSTATE_IDLE;
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
    ic_ptr = ic;
    ic_ptr->state = MASTER_IDLE;
    ic_ptr->substate = I2C_SUBSTATE_IDLE;
    //ic_ptr->buffer = 0
    //ic_ptr->buffer_length = 0;
}

