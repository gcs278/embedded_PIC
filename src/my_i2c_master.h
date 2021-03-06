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

#define I2CMST_LOCAL_SENSOR 1
#define I2CMST_MOTOR 2
#define I2CMST_LOCAL_WALLSENSOR 3
#define I2CMST_MOTOR_LOCAL 4
#define I2CMST_ARM_REQUEST 5
#define I2CMST_MOTOR_LOCAL_DEBUG 6

unsigned char i2cMstrMsgState;
unsigned char i2c_busy = 0;

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
    i2c_queue i2c_q;
   
void init_i2c_master(i2c_master_comm *);
void i2c_master_handler(void);
void i2c_master_handler2(void);
void i2c_configure_master(void);
void i2c_configure_master2(void);
unsigned char i2c_master_send(unsigned char,unsigned char *, unsigned char);
unsigned char i2c_master_send2(unsigned char,unsigned char *, unsigned char);
unsigned char i2c_master_recv(unsigned char, unsigned char, unsigned char);
unsigned char i2c_master_busy();

#endif	/* MY_I2C_MASTER_H */

