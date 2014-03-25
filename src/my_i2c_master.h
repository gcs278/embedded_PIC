/* 
 * File:   my_i2c_master.h
 * Author: TaylorM7
 *
 * Created on February 22, 2014, 5:15 PM
 */

#ifndef MY_I2C_MASTER_H
#define	MY_I2C_MASTER_H

#include "messages.h"
#include "i2c_queue.h"

#define I2CMST_SENSOR 1
#define I2CMST_MOTOR 2

unsigned char i2cMstrMsgState;

typedef enum
{
    MASTER_IDLE,
    MASTER_WRITE,
    MASTER_READ
}i2c_mode;

typedef enum {

    IDLE = 0,
    START_BIT,

    RESTART,
    WRITE_ADDRESS,
    READ_ADDRESS,
    DATA,
    SLAVE_RESPONSE,
    ACK,
    NACK,
    STOP_MESSAGE,
    ERROR
} i2c_state;

typedef struct __i2c_master_comm {
    i2c_state state;
    unsigned char buffer[MSGLEN];
    unsigned char buffer_length;
    unsigned char buffer_index;
    unsigned char address;
    unsigned char slave_address;
} i2c_master_comm;

i2c_queue* i2c_q;

void init_i2c_master(i2c_master_comm *);
void i2c_master_handler(void);
void i2c_configure_master(void);
unsigned char i2c_master_send(unsigned char,unsigned char *, unsigned char);
unsigned char i2c_master_recv(unsigned char, unsigned char, unsigned char);
unsigned char i2c_master_busy();

#endif	/* MY_I2C_MASTER_H */

