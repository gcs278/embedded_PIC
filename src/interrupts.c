/*
 TEAM 1 - GRANT SPENCE, TAYLOR MCGOUGH, MARTIN ANILANE, MATT O'NEIL
 Milestone 1 = 2/11/2014
 */

#include "maindefs.h"
#include "interrupts.h"
#include <stdio.h>
#include "user_interrupts.h"
#include "messages.h"
#include "my_i2c_master.h"
#include "my_uart.h"
#include "my_motor.h"
#include "my_adc.h"
#include "my_wall_correction.h"
#include <time.h>
#include <stdlib.h>
//----------------------------------------------------------------------------
// Note: This code for processing interrupts is configured to allow for high and
//       low priority interrupts.  The high priority interrupt can interrupt the
//       the processing of a low priority interrupt.  However, only one of each type
//       can be processed at the same time.  It is possible to enable nesting of low
//       priority interrupts, but this code is not setup for that and this nesting is not
//       enabled.
int i = 0;
int timer2_extender = 0;
int start_IR_receiver = 0;
int IR_count = 0;

int timer0_count = 0;
int display_count = 0;
int adc_channel = 0;

unsigned char buffer_1[10];
unsigned char buffer_4[10];

void enable_interrupts() {
    // Peripheral interrupts can have their priority set to high or low
    // enable high-priority interrupts and low-priority interrupts
    RCONbits.IPEN = 1;
    INTCONbits.GIEH = 1;
    INTCONbits.GIEL = 1;

    INTCONbits.TMR0IE = 1;
    // Timer0 interrupt at high priority
    INTCON2bits.TMR0IP = 1;
}

int in_high_int() {
    return (!INTCONbits.GIEH);
}

int low_int_active() {
    return (!INTCONbits.GIEL);
}

int in_low_int() {
    if (INTCONbits.GIEL == 1) {
        return (0);
    } else if (in_high_int()) {
        return (0);
    } else {
        return (1);
    }
}

int in_main() {
    if ((!in_low_int()) && (!in_high_int())) {
        return (1);
    } else {
        return (0);
    }
}

#ifdef __XC8
// Nothing is needed for this compiler
#else
// Set up the interrupt vectors
void InterruptHandlerHigh();
void InterruptHandlerLow();

#pragma code InterruptVectorLow = 0x18

void
InterruptVectorLow(void) {
    _asm
    goto InterruptHandlerLow //jump to interrupt routine
            _endasm
}

#pragma code InterruptVectorHigh = 0x08

void
InterruptVectorHigh(void) {
    _asm
    goto InterruptHandlerHigh //jump to interrupt routine
            _endasm
}
#endif
//----------------------------------------------------------------------------
// High priority interrupt routine
// this parcels out interrupts to individual handlers

#ifdef __XC8
interrupt
#else
#pragma code
#pragma interrupt InterruptHandlerHigh
#endif
void InterruptHandlerHigh() {
    // We need to check the interrupt flag of each enabled high-priority interrupt to
    // see which device generated this interrupt.  Then we can call the correct handler.

    // check to see if we have an I2C interrupt
    if (PIR1bits.SSPIF) {
        // clear the interrupt flag
        PIR1bits.SSPIF = 0;
        // call the handler
        #if defined (MAIN_PIC)
        {
            i2c_master_handler();
        }
        #else
        {
            i2c_int_handler();
        }
        #endif
    }

    if (PIR3bits.SSP2IF) {
        PIR3bits.SSP2IF = 0;
#if defined(MAIN_PIC) || defined(MOTOR_PIC)
        i2c_master_handler2();
#endif
    }

    // Check to see if we have an interrupt on timer 2
    if ( PIR1bits.TMR2IF ) {
        // LATBbits.LATB7 = !LATBbits.LATB7;
        PIR1bits.TMR2IF = 0;

        #if defined (MOTOR_PIC)
        {
            if ( timer2_extender > 10 ) {
                LATB = 4; // Sequence 3
                motor_semaphore = 1;
//            if(motor_index == 10)
//            {
//                //indicate message lost
//                motor_index = 1;
//            }
                //i2c_master_send2(1, msg[0], 0x71);

                unsigned char msg[10];
                msg[0] = ticks_left_total/6 % 10;
                msg[1] = (ticks_left_total/6/10) % 10;
                msg[2] = (ticks_left_total/6/100) % 10;
                msg[3] = (ticks_left_total/6/1000) % 10;
                LATB = 5;
                ToMainLow_sendmsg(10,MSGT_DISPLAY_LED,msg);
                motorArrayLeft[motor_index % 10] = ticks_left;
                motorArrayRight[motor_index % 10] = ticks_right;
                ticks_left = 0;
                ticks_right = 0;
                LATB = 6;
                motor_index++;
                motor_semaphore = 0;
                timer2_extender = 0;
                LATB = 5;
            } else {
                timer2_extender++;
            }
        }
#endif

    }

    // check to see if we have an interrupt on timer 0
    if (INTCONbits.TMR0IF) {
        start_IR_receiver = 0;
        //LATBbits.LATB7 = !LATBbits.LATB7;
        INTCONbits.TMR0IF = 0; // clear this interrupt flag
        // call whatever handler you want (this is "user" defined)
#ifndef SENSOR_PIC
        timer0_int_handler();
#endif
        // LATDbits.LATD7 = !LATDbits.LATD7;
#if defined (MAIN_PIC)
        {
        
#if defined (MOTOR_SCRIPT_MS4)
            if ( timer2_extender > 50 ) {
//                unsigned char msg[5];
//                msg[0] = 'C';
//                msg[1] = 0x00;
//                msg[2] = 0xFF;
//                msg[3] = 0x00;
//                i2c_master_send2(4, msg, 0x09);
                
                i2cMstrMsgState = I2CMST_MOTOR_LOCAL_DEBUG;
                i2c_master_recv(0x0A, RoverMsgMotorLeftData, 0x4F);
                timer2_extender = 0;
            } else {
                timer2_extender++;
            }
#elif defined (ADJUST_SCRIPT_MS4)
            if ( timer2_extender == 0) {
                i2cMstrMsgState = I2CMST_MOTOR_LOCAL;
                i2c_master_recv(0x0A, RoverMsgMotorForward, 0x4F);
            }
            if ( timer2_extender > 50 ) {
                unsigned char msg[5];
                msg[0] = 'C';
                msg[1] = 0x00;
                msg[2] = 0xFF;
                msg[3] = 0xFF;
                i2c_master_send2(4, msg, 0x09);
                unsigned char buffer[10];
                buffer[0] = 1;
                buffer[1] = i;
                i2cMstrMsgState = I2CMST_LOCAL_WALLSENSOR;
                ToMainHigh_sendmsg(10, MSGT_I2C_MASTER_RECV_COMPLETE, buffer);
                timer2_extender = 0;
                i= i + 5;
                if ( i > 50 )
                    i=1;
            } else {
                timer2_extender++;
            }
#endif
//            if ( timer2_extender > 750 ) {
//                i2cMstrMsgState = I2CMST_MOTOR_LOCAL;
//                //i2c_master_recv(0x0A, RoverMsgMotorForwardCMDelim + i + 150, 0x4F);
//                if ( i == 0 ) {
//                    i2c_master_recv(0x0A, RoverMsgMotorSpeedSlow, 0x4F);
//                }else if ( i == 1 )
//                    i2c_master_recv(0x0A, RoverMsgMotorForward, 0x4F);
//                else if ( i % 2 ) {
//                    i2c_master_recv(0x0A, RoverMsgMotorLeft2, 0x4F);
//                }
//                else {
//                    i2c_master_recv(0x0A, RoverMsgMotorRight2, 0x4F);
//                }
//
//                i++;
//                timer2_extender = 0;
//            } else {
//                timer2_extender ++;
//            }

//            WriteUSART(i2c_q->end);
            // Check queue
            ToMainLow_sendmsg(0,MSGT_QUEUE_GET_DATA,(void*)0);

//            if(start_stop == 0)
//            {
//                i2c_master_recv(0x0A, 0x05, 0x4F);
//                start_stop = 1;
//            }
//            else if (start_stop == 1)
//            {
             //   i2cMstrMsgState = I2CMST_SENSOR;
             //   i2c_master_recv(0x0A, 0x05, 0x4E);
//            }
//                start_stop = 0;
//            }

        }
#elif defined(SENSOR_PIC)
{
                         if(timer0_count == 0)
            {
                LATDbits.LATD6 = 1;

                LATBbits.LATB7 = 0;
                LATBbits.LATB6 = 1;
                LATBbits.LATB5 = 0;
                LATBbits.LATB4 = 0;
                timer0_count = timer0_count + 1;
            }
            else if (timer0_count == 60 && display_count != 10)
            {
                buffer_1[display_count] = ADCArray[1];
                buffer_4[display_count] = ADCArray[4];
                timer0_count = timer0_count + 1;

            }
            else if (timer0_count == 60 && display_count == 10)
            {
                LATBbits.LATB7 = 0;
                LATBbits.LATB6 = 0;
                LATBbits.LATB5 = 0;
                LATBbits.LATB4 = 1;

                int value_1 = (buffer_1[0] + buffer_1[1] + buffer_1[2] + buffer_1[3] + buffer_1[4] + buffer_1[5] + buffer_1[6] + buffer_1[7] + buffer_1[8] + buffer_1[9])/10;
                int value_4 = (buffer_4[0] + buffer_4[1] + buffer_4[2] + buffer_4[3] + buffer_4[4] + buffer_4[5] + buffer_4[6] + buffer_4[7] + buffer_4[8] + buffer_4[9])/10;
                unsigned char msgbuffer[MSGLEN + 1];
                msgbuffer[0] = 0x76;
                msgbuffer[1] = ADCArray[1] >> 4;
                msgbuffer[2] = ADCArray[1] % 0xF;
                msgbuffer[3] = ADCArray[4] >> 4;
                msgbuffer[4] = ADCArray[4] % 0xF;
//                  msgbuffer[1] = value_1 >> 4;
//                  msgbuffer[2] = value_1 % 0xF;
//                  msgbuffer[3] = value_4 >> 4;
//                  msgbuffer[4] = value_4 % 0xF;
                uart_send_data(msgbuffer, 5);
                timer0_count = timer0_count + 1;
                display_count = 0;
            }
            else if (timer0_count == 610)
            {
                display_count = display_count + 1;
                timer0_count = 0;
            }
            else if (timer0_count == 10)
            {
                LATDbits.LATD6 = 0;
                timer0_count = timer0_count + 1;
                readSensor(0);
                sensor_count = 1;
            }
            else
            {
                LATDbits.LATD6 = 0;

                LATBbits.LATB7 = 1;
                LATBbits.LATB6 = 0;
                LATBbits.LATB5 = 0;
                LATBbits.LATB4 = 0;

                timer0_count = timer0_count + 1;
            }
   
//                if(motor_index == 10)
//                    motor_index = 1;
//                motorArray[motor_index++] = sensor_value++;
//                motorArray[motor_index++] = sensor_value++;
//                motorArray[motor_index++] = sensor_value++;
                
}
#endif



    }

    // here is where you would check other interrupt flags.
#ifdef SENSOR_PIC
        //check if the interrupt is caused by ADC
    if(PIR1bits.ADIF == 1) {
        //ADCValue = ReadADC();
        //Reset interrupt flag and start conversion again
        PIR1bits.ADIF = 0;
        // LATDbits.LATD6 = !LATDbits.LATD6;
        int pureADCValue = ReadADC();
        ADCValue = pureADCValue >> 2;

        adc_semaphore = 1;
        if(adc_index > 9)
        {
            //indicate message lost
            messages_lost = messages_lost + 8;
            adc_index = 1;
        }
        ADCArray[adc_index] = ADCValue;
        adc_index++;
        adc_semaphore = 0;
    }
#endif

    // The *last* thing I do here is check to see if we can
    // allow the processor to go to sleep
    // This code *DEPENDS* on the code in messages.c being
    // initialized using "init_queues()" -- if you aren't using
    // this, then you shouldn't have this call here
    //SleepIfOkay();


#if defined (MOTOR_PIC)
//    if (INTCONbits.RBIF && INTCONbits.RBIE){
//        INTCONbits.RBIF = 0;
//
//        ticks += 1;
//    }
#endif

#if defined (MAIN_PIC)
    if (INTCON3bits.INT1IF && INTCON3bits.INT1IE ) //RD4
    {
        if(start_IR_receiver == 0)
        {
            start_IR_receiver = 1;
            IR_count = 0;
        }
        else
        {
            //LATDbits.LATD7 = !LATDbits.LATD7;
            //finishLine = 1;
            IR_count = IR_count + 1;
            if(IR_count == 4)
            {
                LATDbits.LATD7 = !LATDbits.LATD7;
                finishLine = 1; 
            }
        }

        INTCON3bits.INT1IF = 0;
    }

    if (INTCON3bits.INT2IF && INTCON3bits.INT2IE ) {
        // LATDbits.LATD7 = !LATDbits.LATD7;
        blindSpotSensor = 0;
        INTCON3bits.INT2IF = 0;
    }
#endif

}

//----------------------------------------------------------------------------
// Low priority interrupt routine
// this parcels out interrupts to individual handlers
// This works the same way as the "High" interrupt handler
#ifdef __XC8
interrupt low_priority
#else
#pragma code
#pragma interruptlow InterruptHandlerLow
#endif
void InterruptHandlerLow() {
    // check to see if we have an interrupt on timer 1
    if (PIR1bits.TMR1IF) {
        PIR1bits.TMR1IF = 0; //clear interrupt flag
        timer1_int_handler();
        LATBbits.LATB7 = !LATBbits.LATB7;
    }

    // check to see if we have an interrupt on USART RX
    if (PIR1bits.RCIF) {
        PIR1bits.RCIF = 0; //clear interrupt flag
        uart_recv_int_handler();
    }

    // Check if we can an UART message to send
    if ( PIE1bits.TXIE && PIR1bits.TX1IF ) {
        // Stop the interrupt
        PIE1bits.TXIE = 0;

        // Handle that tx data
        uart_send_int_handler();
    }
}
