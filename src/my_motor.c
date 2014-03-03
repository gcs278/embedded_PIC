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
/*
 * 
 */

// Motor encoding definitions
static char forwardFull[] = { 1, 129 };
static char forwardHalf[] = {35, 163 };
static char left[] = { 219, 35 };
static char right[] = { 92, 163 };
static char back[] = { 77, 204 };
static char stop[] = { 0 };

void motor_init() {
    motor_state = moveStop;
}

void motor_encode_lthread(unsigned char msg) {
    // Switch on the msg type
    switch (msg) {

        case moveForwardFull:
            // Make it so you have to hit start twice for full speed
            if ( motor_state != moveForwardFull )
                uart_send_data(forwardHalf, 2);
            else
                uart_send_data(forwardFull, 2);
            
            motor_state = moveForwardFull;
            break;

        case moveStop:
            uart_send_data(stop, 1);
            motor_state = moveStop;
            break;

        case moveLeft:
            uart_send_data(left, 2);
            motor_state = moveLeft;
            break;

        case moveRight:
            uart_send_data(right, 2);
            motor_state = moveRight;
            break;

        case moveBack:
            uart_send_data(back, 2);
            motor_state = moveBack;
            break;
            
        default:
            break;

    }
}
