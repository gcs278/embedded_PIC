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
static i2c_master_comm *ic_ptr2;
i2c_mode mode;
i2c_mode mode2;
unsigned char slave_address;

// Configure for I2C Master mode -- the variable "slave_address" should be stored in
//   i2c_comm (as pointed to by ic_ptr) for later use.

void i2c_configure_master(void) {
     createQueue(&i2c_q,15);
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

void i2c_configure_master2(void) {
    // set clock and data as inputs
#ifdef __USE18F46J50
    SCL2 = 1;
    SDA2 = 1;
#else
    I2C2_SCL = 1;
    I2C2_SDA = 1;
#endif



    // configure status bits
    SSP2STATbits.SMP = 1;
    SSP2STATbits.CKE = 0;


#ifdef __USE18F46J50
    SSP2ADD = 119;
#else
    // set frequency to 100kHz
    SSP2ADD = 29;
#endif



    // set to master mode
    SSP2CON1 = 0x8;
    SSP2CON2 = 0;

    // start I2C Master mode
    SSP2CON1bits.SSPEN = 1;

    ic_ptr2->state = IDLE;
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

unsigned char i2c_master_send2(unsigned char length, unsigned char *msg, unsigned char slave_address)
{
    mode2 = MASTER_WRITE;
    memcpy(ic_ptr2->buffer, msg, length);
    ic_ptr2->buffer_length = length;
    ic_ptr2->buffer_index = 0;
    ic_ptr2->slave_address = slave_address << 1;
    SSP2CON2bits.SEN2 = 1;
    ic_ptr2->state = START_BIT;
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
    //LATB = 7; // F
    LATAbits.LA0 = 1;
    // Check if we are in the middle of something
//    if ( ic_ptr->state != IDLE) {
//        return 0;
//        //Reset();
//    }

    i2c_busy = 1;
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
            i2c_busy = 0;
            mode = MASTER_IDLE;
            ic_ptr->state = IDLE;
            break;
        }

        default:
        {
            break;
        } // End default case
    }
  //  }
//    else {
//        i2c_busy = 0;
//        ic_ptr->state = IDLE;
//        mode = MASTER_IDLE;
//    }

}

void i2c_master_handler2()
{
    switch (ic_ptr2->state)
    {
        case START_BIT:
        {

            SSP2BUF = (ic_ptr2->slave_address);
            ic_ptr2->state = WRITE_ADDRESS;
            break;
        } 

        case WRITE_ADDRESS:
        {
            if (1 == SSP2CON2bits.ACKSTAT)
            {
                    // NACK 
            }
            else
            {
                if (mode2 == MASTER_WRITE )
                {
                    SSP2BUF = ic_ptr2->buffer[ic_ptr2->buffer_index];
                } 
                else if (mode2 == MASTER_READ )
                {
                    SSP2BUF = ic_ptr2->address;
                }
                ic_ptr2->state = DATA;
            }
            break;
        }

        case DATA:
        {

        if (1 == SSP2CON2bits.ACKSTAT)
        {
            // NACK
        }
        else
        {
            if (mode2 == MASTER_WRITE )
            {
                        
                ic_ptr2->buffer_index++;

                if (ic_ptr2->buffer_index < ic_ptr2->buffer_length)
                {     
                    SSP2BUF = ic_ptr2->buffer[ic_ptr2->buffer_index];
                    ic_ptr2->state = DATA;
                }
                else
                {
                    SSP2CON2bits.PEN = 1;
                    ic_ptr2->state = STOP_MESSAGE;
                }
            }
            else if (mode2 == MASTER_READ )
            {
                SSP2CON2bits.RSEN = 1;
                ic_ptr2->state = RESTART;
            }

        }
        break;
        }
                
        case RESTART:
        {
            SSP2BUF = (ic_ptr2->slave_address) | 0x01;
            ic_ptr2->state = READ_ADDRESS;
            break;
        } 

        case READ_ADDRESS:
        {
            if (1 == SSP2CON2bits.ACKSTAT)
            {
                    // NACK 
            } else
            {        
                SSP2CON2bits.RCEN = 1;
                ic_ptr2->state = SLAVE_RESPONSE;
            }
            break;
        }
        case SLAVE_RESPONSE:
        {
            ic_ptr2->buffer[ic_ptr2->buffer_index] = SSP2BUF;

                
            ic_ptr2->buffer_index++;
            if (ic_ptr2->buffer_index < ic_ptr2->buffer_length)
            {
                SSP2CON2bits.ACKDT = 0;
                ic_ptr2->state = ACK;

            }
            else
            {
                SSP2CON2bits.ACKDT = 1;
                ic_ptr2->state = NACK;
            }
            SSP2CON2bits.ACKEN = 1;
            break;
            } 

        case ACK:
        {
            SSP2CON2bits.RCEN = 1;
            ic_ptr2->state = SLAVE_RESPONSE;
            break;
        } 
        case NACK:
        {
            SSP2CON2bits.PEN = 1;
            ic_ptr2->state = STOP_MESSAGE;
            break;
        }

        case STOP_MESSAGE:
        {
            if ( mode2 == MASTER_WRITE )
            {
                //ToMainHigh_sendmsg(0, MSGT_I2C_MASTER_SEND_COMPLETE, 0);
            } else if (mode2 == MASTER_READ )
            {
               // ToMainHigh_sendmsg(ic_ptr2->buffer_index, MSGT_I2C_MASTER_RECV_COMPLETE, ic_ptr2->buffer);
            }

            mode2 = MASTER_IDLE;
            ic_ptr2->state = IDLE;
            break;
        }

        default:
        {
            break;
        } // End default case
    }

}


void init_i2c_master(i2c_master_comm *ic) {
    mode = MASTER_IDLE;

    ic_ptr = ic;
    
    ic_ptr->state = IDLE;
    //ic_ptr->buffer = 0
    //ic_ptr->buffer_length = 0;
}

unsigned char i2c_master_busy() {
    if( i2c_busy == 0 ) {
        return 0;
    }
    else {
        return 1;
    }
}