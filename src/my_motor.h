/* 
 * File:   my_motor.h
 * Author: grantspence
 *
 * Created on March 3, 2014, 5:42 PM
 */

#ifndef MY_MOTOR_H
#define	MY_MOTOR_H

#ifdef	__cplusplus
extern "C" {
#endif

// MSG TYPE conversions
#define moveForwardFull 0x01
#define moveLeft 0x02
#define moveRight 0x03
#define moveBack 0x04
#define moveStop 0x05
#define motorDataLeft 0x07
#define motorDataRight 0x08
    
    unsigned char motor_state;
    unsigned int ticks_right;
    unsigned int ticks_left;

    void motor_encode_lthread(unsigned char msg);
    void motor_init();


#ifdef	__cplusplus
}
#endif

#endif	/* MY_MOTOR_H */

