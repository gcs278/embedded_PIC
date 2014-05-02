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
#include <plib/timers.h>
#include "maindefs.h"
/*
 * 
 */
// RIGHT SIDE
// 209 Ticks for 1 rotations = ~37.5 cm
// 6.06 ticks for 1 cm
// 606 ticks for 1 meter

// LEFT SIDE
// 6 Ticks for 1 cm
// 223 Tick values is 1 rotation = ~37.5 cm
// 600 Tick values is 1 meter
unsigned char rightWheelSpeed = 55;
unsigned char leftWheelSpeed = 183;

// Motor encoding definitions
static unsigned char forwardVariable[] = { 55, 183 };
static unsigned char forwardFull[] = { 1, 129 };
static unsigned char forwardHalf[] = {42, 170 };
static unsigned char forwardSlow[] = { 50, 176 };
static unsigned char left[] = { 219, 35 };
static unsigned char left2[] = { 180 };
static unsigned char right2[] = { 45 };
static unsigned char right[] = { 92, 163 };
static unsigned char back[] = { 77, 204 };
static unsigned char stop[] = { 0 };

void motor_init() {
    motor_state = moveStop;
    ticks_left = 0; // Timer 1
    ticks_right = 0; // Timer 0
    ticks_left_C = 0;
    ticks_right_C = 0;
    ticks_right_C_Long = 0;
    ticks_right_C_Long = 0;
    ticks_left_total = 0;
    executingEncode = 0;
    executingEncodeLong = 0;
    executingLeftAdj = 0;
    executingRightAdj = 0;
    motor_speed = RoverMsgMotorSpeedSlow;
}

void motor_encode_lthread(unsigned char msg) {
    LATB = 7; // Sequence 1
    // Check if we have a move ___ distance
    if ( msg >= RoverMsgMotorForwardCMDelim ) {
//        ticks_left_C = 0;
//        ticks_right_C = 0;
        WriteTimer0(TIMER0_START);
        WriteTimer1(TIMER1_START);
        uart_send_data(forwardVariable,2);

        if ( motor_state != RoverMsgMotorLeft2 && motor_state != RoverMsgMotorRight2 )
            motor_state = RoverMsgMotorForward;
        
        if ( msg <= 0xC5 ) {
            ticks_left_C_Long = 6*(msg-RoverMsgMotorForwardCMDelim+1);
            ticks_right_C_Long = 6.06*(msg-RoverMsgMotorForwardCMDelim+1);
            executingEncodeLong = 1;
//            while( ticks_left_C < 6*(msg-RoverMsgMotorForwardCMDelim+1)
//                    || ticks_right_C < 6.06*(msg-RoverMsgMotorForwardCMDelim+1) );
        }
        else {
            ticks_left_C_Long = ( 60*(msg-RoverMsgMotorForwardCMDelim+1-150) + 6*150);
            ticks_right_C_Long = ( 60.6*(msg-RoverMsgMotorForwardCMDelim+1-150) + 6.06*150);
            executingEncodeLong = 1;
//            while( ticks_left_C < ( 60*(msg-RoverMsgMotorForwardCMDelim+1-150) + 6*150 )
//                    || ticks_right_C < ( 60.6*(msg-RoverMsgMotorForwardCMDelim+1-150) + 6.06*150) ){
//            }
        }

       
    } else {
        // Switch on the msg type
        switch (msg) {

            case RoverMsgMotorInRange:
                if ( executingLeftAdj || executingRightAdj ) {
                    if ( motor_state == RoverMsgMotorForward )
                        uart_send_data(forwardVariable, 2);
                    else
                        uart_send_data(stop,1);

                    executingLeftAdj = 0;
                    executingRightAdj = 0;
                }
                
                break;
            case RoverMsgMotorForward:
                if ( !executingEncode )
                    uart_send_data(forwardVariable, 2);

                motor_state = RoverMsgMotorForward;
                break;

            case RoverMsgMotorStop:
                // Only execute when it isn't already executing a turn
                if ( !executingEncode ) 
                    uart_send_data(stop, 1);       
                
                // This will make turns stop afterwards
                motor_state = RoverMsgMotorStop;
                executingLeftAdj = 0;
                executingRightAdj = 0;
                break;

            case RoverMsgMotorLeft:
                uart_send_data(left, 2);
                ticks_left_C = 20;
                executingEncode = 1;
                break;

            case RoverMsgMotorRight:
                uart_send_data(right, 2);
                ticks_left_C = 20;
                executingEncode = 1;

                break;

            case RoverMsgMotorBack:
                uart_send_data(back, 2);
                motor_state = moveBack;
                break;

            case RoverMsgMotorLeft2:
            {
                int diff = 192 - leftWheelSpeed;
                left2[0] = 192 + diff + 10;
                uart_send_data(left2, 1);
                if ( motor_speed == RoverMsgMotorSpeedMediumFast && motor_speed == RoverMsgMotorSpeedFastBRAH )
                    ticks_left_C = 6;
                else
                    ticks_left_C = 12;
                executingEncode = 1;

//                  ticks_left_C = 135;
//                left2[0] = leftWheelSpeed - 20;
//                left2[1] = rightWheelSpeed + 40;
//                uart_send_data(left2,2);
//                executingEncode = 1;
//                motor_state = RoverMsgMotorLeft2;

                    
//                if ( !executingEncode && !executingRightAdj ) {
//                    // Make the Right wheel go faster
//                    int diff = rightWheelSpeed - 15;
//                    unsigned char forwardRight[] = { 0, 0 };
////                    int diff = 192 - leftWheelSpeed;
//                    forwardRight[0] = leftWheelSpeed + 5;
//                    forwardRight[1] = diff;
//                    //right2[0] = 64 + diff + 10;
//                    uart_send_data(forwardRight, 2);
////                    ticks_right_C = 17;
//                    executingRightAdj = 1;
//                }
                break;
            }

            case RoverMsgMotorRight2:
            {
                int diff = 64 - rightWheelSpeed;
                right2[0] = 64 + diff + 10;
                uart_send_data(right2, 1);
                if ( motor_speed == RoverMsgMotorSpeedMediumFast && motor_speed == RoverMsgMotorSpeedFastBRAH )
                    ticks_right_C = 7;
                else
                    ticks_right_C = 15;
                executingEncode = 1;

//                right2[0] = leftWheelSpeed + 40;
//                right2[1] = rightWheelSpeed - 20;
//                uart_send_data(right2,2);
//                ticks_right_C = 150;
//                executingEncode = 1;
//                motor_state = RoverMsgMotorRight2;

//                executingRightAdj = 0;
//                if ( !executingEncode && !executingLeftAdj ) {
//                    // Make the left wheel go faster
//                    unsigned char forwardLeft[] = { 0, 0 };
//                    int diff = leftWheelSpeed - 15;
////                    int diff = 192 - leftWheelSpeed;
//                    forwardLeft[0] = diff;
//                    forwardLeft[1] = rightWheelSpeed + 5;
//                   // left2[0] = 192 + diff + 10;
//                    uart_send_data(forwardLeft, 2);
////                    ticks_left_C = 15;
//                    executingLeftAdj = 1;
//                }
                break;
            }

            case RoverMsgMotorLeft7:
            {
                int diff = 192 - leftWheelSpeed;
                left2[0] = 192 + diff + 10;
                uart_send_data(left2, 1);
                if ( motor_speed == RoverMsgMotorSpeedMediumFast || motor_speed == RoverMsgMotorSpeedFastBRAH )
                    ticks_left_C = 16;
                else
                    ticks_left_C = 35;
                executingEncode = 1;

                break;
            }
            case RoverMsgMotorRight7:
            {
                int diff = 64 - rightWheelSpeed;
                right2[0] = 64 + diff + 10;
                uart_send_data(right2, 1);
                if ( motor_speed == RoverMsgMotorSpeedMediumFast || motor_speed == RoverMsgMotorSpeedFastBRAH )
                    ticks_right_C = 20;
                else
                    ticks_right_C = 40;
                executingEncode = 1;
                
                break;
            }


            case RoverMsgMotorLeft90:
                uart_send_data(left, 2);
                ticks_left_C = 205;
                executingEncode = 1;
                executingEncodeLong = 0;
                motor_state = RoverMsgMotorStop;
                break;

            case RoverMsgMotorRight90:
                uart_send_data(right, 2);
                ticks_left_C = 218;
                executingEncode = 1;
                executingEncodeLong = 0;
                motor_state = RoverMsgMotorStop;
                break;

            case RoverMsgMotorSpeedCreepin:
                motor_speed = RoverMsgMotorSpeedCreepin;
                rightWheelSpeed = 58;
                leftWheelSpeed = 186;
                forwardVariable[0] = leftWheelSpeed;
                forwardVariable[1] = rightWheelSpeed;
                //uart_send_data(forwardVariable, 2);
                break;
            case RoverMsgMotorSpeedSlow:
                motor_speed = RoverMsgMotorSpeedSlow;
                rightWheelSpeed = 55;
                leftWheelSpeed = 183;
                forwardVariable[0] = leftWheelSpeed;
                forwardVariable[1] = rightWheelSpeed;
                //uart_send_data(forwardVariable, 2);
                break;
            case RoverMsgMotorSpeedMedium:
                motor_speed = RoverMsgMotorSpeedMedium;
                rightWheelSpeed = 52;
                leftWheelSpeed = 180;
                forwardVariable[0] = leftWheelSpeed;
                forwardVariable[1] = rightWheelSpeed;
                //uart_send_data(forwardVariable, 2);
                break;
            case RoverMsgMotorSpeedMediumFast:
                motor_speed = RoverMsgMotorSpeedMediumFast;
                rightWheelSpeed = 43;
                leftWheelSpeed = 171;
                forwardVariable[0] = leftWheelSpeed;
                forwardVariable[1] = rightWheelSpeed;
                //uart_send_data(forwardVariable, 2);
                break;
            case RoverMsgMotorSpeedFastBRAH:
                motor_speed = RoverMsgMotorSpeedFastBRAH;
                rightWheelSpeed = 15;
                leftWheelSpeed = 143;
                forwardVariable[0] = leftWheelSpeed;
                forwardVariable[1] = rightWheelSpeed;
                //uart_send_data(forwardVariable, 2);
                break;
            default:
                break;

        }
    }
}

void RoverForward() {
    uart_send_data(forwardVariable, 2);
}

void RoverStop() {
    uart_send_data(stop,1);
}

unsigned char* motorTickValue(unsigned char msgRequest)
{
    while(motor_semaphore == 1){};
    LATB = 2; // Sequence 2
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

int insertionSort(unsigned char *d, int size){
        int i, j;
        for (i = 1; i < size; i++) {
                int tmp = d[i];
                for (j = i; j >= 1 && tmp < d[j-1]; j--)
                        d[j] = d[j-1];
                d[j] = tmp;
        }
}