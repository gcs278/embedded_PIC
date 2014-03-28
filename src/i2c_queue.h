/* 
 * File:   i2c_queue.h
 * Author: grantspence
 *
 * Created on March 16, 2014, 3:51 PM
 */

#ifndef I2C_QUEUE_H
#define	I2C_QUEUE_H

#include "messages.h"


#ifdef	__cplusplus
extern "C" {
#endif

    typedef struct i2c_master_cmd {
            unsigned char msgCount;
            unsigned char msgType;
            unsigned char data[I2CMSGLEN];
        } i2c_master_cmd;
    
    
    typedef struct i2c_queue {
        unsigned char size;
        int front;
        int end;
        i2c_master_cmd elements[10];
    } i2c_queue;

    unsigned char q_semiphore = 1;
    
    unsigned char createQueue(i2c_queue* queue, unsigned char size);
    unsigned char putQueue(i2c_queue* queue, i2c_master_cmd element);
    unsigned char getQueue(i2c_queue* queue, i2c_master_cmd* element);
    inline unsigned char isEmpty(i2c_queue* queue);

#ifdef	__cplusplus
}
#endif

#endif	/* I2C_QUEUE_H */

