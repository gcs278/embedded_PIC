/* 
 * File:   i2c_queue.h
 * Author: grantspence
 *
 * Created on March 16, 2014, 3:51 PM
 */

#ifndef I2C_QUEUE_H
#define	I2C_QUEUE_H

#ifdef	__cplusplus
extern "C" {
#endif

    typedef struct i2c_queue {
        unsigned char size;
        int front;
        int end;
        unsigned char* elements;
    } i2c_queue;

    unsigned char createQueue(i2c_queue* queue, unsigned char size);
    unsigned char putQueue(i2c_queue* queue, unsigned char element);
    unsigned char getQueue(i2c_queue* queue, unsigned char* element);
    unsigned char isEmpty(i2c_queue* queue);
    

#ifdef	__cplusplus
}
#endif

#endif	/* I2C_QUEUE_H */

