/* 
 * File:   my_i2c_master.h
 * Author: TaylorM7
 *
 * Created on February 22, 2014, 5:15 PM
 */

#ifndef MY_I2C_MASTER_H
#define	MY_I2C_MASTER_H

#include "messages.h"

#define MAXI2CBUF MSGLEN

typedef enum
{
    MASTER_IDLE,
    MASTER_WRITE,
    MASTER_READ
}i2c_master_state;
/*
typedef enum {
    //No activity, ready for a new operation to begin
    I2C_IDLE,
    //A write to a slave is in progress (i2c_master_write() has been called)
    I2C_WRITE,
    // A read from a slave is in progress(i2c_master_read() has been called)
    I2C_READ
} i2c_master_state;
*/
typedef enum {
    /** Substate machine is idle.  Next substate is always START_SENT. */
    I2C_SUBSTATE_IDLE = 0,
    /** Start has been sent. */
    I2C_SUBSTATE_START_SENT,
    /** Restart has been sent. */
    I2C_SUBSTATE_RESTART_SENT,
    /** Slave address and Write bit have been sent. */
    I2C_SUBSTATE_ADDR_W_SENT,
    /** Slave address and Read bit have been sent. */
    I2C_SUBSTATE_ADDR_R_SENT,
    /** A data byte has been sent. */
    I2C_SUBSTATE_DATA_SENT,
    /** I2C has been placed into receive mode and is awaiting a byte. */
    I2C_SUBSTATE_WAITING_TO_RECEIVE,
    /** An ACK has been sent. */
    I2C_SUBSTATE_ACK_SENT,
    /** A NACK has been sent. */
    I2C_SUBSTATE_NACK_SENT,
    /** A STOP has been sent.  Next substate is always IDLE. */
    I2C_SUBSTATE_STOP_SENT,
    /** An error has occurred and a STOP has been sent to free the bus. */
    I2C_SUBSTATE_ERROR
} i2c_master_substate;

typedef struct __i2c_master_comm {
    /** Main state for I2C master operations. */
    i2c_master_state state;
    /** Substate for I2C master operations. */
    i2c_master_substate substate;
    /** Buffer used for bytes to be sent or bytes received on I2C */
    unsigned char buffer[MAXI2CBUF];
    /** Number of bytes buffered or to be buffered in 'buffer'. */
    unsigned char buffer_length;
    /** Index of next byte to use in 'buffer'. */
    unsigned char buffer_index;
    /** "Register address" being read from slave. */
    unsigned char register_byte;
    /**
     * I2C_SLAVE: Slave address for this device.
     * I2C_MASTER: Address of the device currently being communicated with.
     */
    unsigned char slave_addr;
} i2c_master_comm;



#define I2C_ERR_THRESHOLD 1
#define I2C_ERR_OVERRUN 0x4
#define I2C_ERR_NOADDR 0x5
#define I2C_ERR_NODATA 0x6
#define I2C_ERR_MSGTOOLONG 0x7
#define I2C_ERR_MSG_TRUNC 0x8

void init_i2c_master(i2c_master_comm *);
void i2c_master_handler(void);
void i2c_configure_master(void);
unsigned char i2c_master_send(unsigned char,unsigned char *, unsigned char);
unsigned char i2c_master_recv(unsigned char, unsigned char, unsigned char);


#endif	/* MY_I2C_MASTER_H */

