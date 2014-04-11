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
#define moveLeft 0x03
#define moveRight 0x02
#define moveBack 0x04
#define moveStop 0x05
#define motorDataLeft 0x07
#define motorDataRight 0x08
#define sensorDataFull 0x11
    
#define motorArraySize 10

#define TIMER1_START 65523
#define TIMER0_START 243

    int motor_value = 0;
    unsigned char motor_semaphore = 0;
    unsigned int motor_index = 1;
    unsigned char motorArrayLeft[motorArraySize];
    unsigned char motorArrayRight[motorArraySize];

    unsigned char motor_state;
    unsigned int ticks_right;
    unsigned int ticks_left;
    unsigned int ticks_left_total;
    unsigned int ticks_left_C;
    unsigned int ticks_right_C;
    void motor_encode_lthread(unsigned char msg);
    void motor_init();
    unsigned char * motorTickValue(unsigned char msgRequest);

#ifdef	__cplusplus
}
#endif

#endif	/* MY_MOTOR_H */

