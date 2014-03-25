/* 
 * File:   my_motor.c
 * Author: grantspence
 *
 * Created on March 3, 2014, 5:42 PM
 */

#include <stdio.h>
#include <stdlib.h>
#include "my_motor.h"
#include "my_uart.h"
#include <plib/usart.h>
#include "messages.h"
/*
 * 
 */

// Motor encoding definitions
static char forwardFull[] = { 1, 129 };
static char forwardHalf[] = {35, 163 };
static char forwardSlow[] = { 50, 177 };
static char left[] = { 219, 35 };
static char left2[] = { 180 };
static char right2[] = { 45 };
static char right[] = { 92, 163 };
static char back[] = { 77, 204 };
static char stop[] = { 0 };

void motor_init() {
    motor_state = moveStop;
    ticks_left = 0; // Timer 1
    ticks_right = 0; // Timer 0
    ticks_left_C = 0;
    ticks_right_C = 0;
}

void motor_encode_lthread(unsigned char msg) {
    // Switch on the msg type
    switch (msg) {

        case moveForwardFull:
            // Make it so you have to hit start twice for full speed
     //       if ( motor_state != moveForwardFull ) {
                uart_send_data(forwardSlow, 2);
//            }
//            else
//                uart_send_data(forwardFull, 2);
            
            motor_state = moveForwardFull;
            break;

        case moveStop:
            uart_send_data(stop, 1);
            motor_state = moveStop;
            break;

        case moveLeft:
            ticks_left_C = 0;
            uart_send_data(left, 2);
            while (ticks_left_C < 20 ); // 200 is about 90
            if ( motor_state == moveStop )
                uart_send_data(stop,1);
            else if ( motor_state == moveForwardFull)
                uart_send_data(forwardHalf, 2);
            else
                uart_send_data(stop,1);
            //motor_state = moveStop;
            break;

        case moveRight:
            ticks_left_C = 0;
            uart_send_data(right, 2);
            while (ticks_left_C < 20 ); // 210 is about 90
            if ( motor_state == moveStop )
                uart_send_data(stop,1);
            else if ( motor_state == moveForwardFull)
                uart_send_data(forwardHalf, 2);
            else
                uart_send_data(stop,1);

            //motor_state = moveStop;
            break;

        case moveBack:
            uart_send_data(back, 2);
            motor_state = moveBack;
            break;

        case RoverMsgMotorLeft2:
            ticks_left_C = 0;
            uart_send_data(left2, 1);
            while (ticks_left_C < 20 ); // 200 is about 90
            if ( motor_state == moveStop )
                uart_send_data(stop,1);
            else if ( motor_state == moveForwardFull)
                uart_send_data(forwardHalf, 2);
            else
                uart_send_data(stop,1);
            break;

        case RoverMsgMotorRight2:
            ticks_right_C = 0;
            uart_send_data(right2, 1);
            while (ticks_right_C < 20 ); // 200 is about 90
            if ( motor_state == moveStop )
                uart_send_data(stop,1);
            else if ( motor_state == moveForwardFull)
                uart_send_data(forwardHalf, 2);
            else
                uart_send_data(stop,1);
            break;

        case RoverMsgMotorLeft90:
            ticks_left_C = 0;
            uart_send_data(left, 2);
            while (ticks_left_C < 185 ); // 200 is about 90
            if ( motor_state == moveStop )
                uart_send_data(stop,1);
            else if ( motor_state == moveForwardFull)
                uart_send_data(forwardSlow, 2);
            else
                uart_send_data(stop,1);
            break;

        case RoverMsgMotorRight90:
            ticks_left_C = 0;
            uart_send_data(right, 2);
            while (ticks_left_C < 210 ); // 210 is about 90
            if ( motor_state == moveStop )
                uart_send_data(stop,1);
            else if ( motor_state == moveForwardFull)
                uart_send_data(forwardSlow, 2);
            else
                uart_send_data(stop,1);
            break;
            
        default:
            break;

    }
}


unsigned char* motorTickValue(unsigned char msgRequest)
{
    while(motor_semaphore == 1){};

    if ( msgRequest == motorDataLeft ) {
        motorArrayLeft[0] = motor_index - 1;
        motor_index = 1;
        return motorArrayLeft;
    }
    else {//if (msgRequest == motorDataRight ) {
        motorArrayRight[0] = motor_index - 1;
        motor_index = 1;
        return motorArrayRight;
    }

}