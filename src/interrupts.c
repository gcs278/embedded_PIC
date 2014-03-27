/*
 TEAM 1 - GRANT SPENCE, TAYLOR MCGOUGH, MARTIN ANILANE, MATT O'NEIL
 Milestone 1 = 2/11/2014
 */

#include "maindefs.h"
#include "interrupts.h"
#include "user_interrupts.h"
#include "messages.h"
#include "my_i2c_master.h"
#include "my_uart.h"
#include "my_motor.h"
#include "my_adc.h"
//----------------------------------------------------------------------------
// Note: This code for processing interrupts is configured to allow for high and
//       low priority interrupts.  The high priority interrupt can interrupt the
//       the processing of a low priority interrupt.  However, only one of each type
//       can be processed at the same time.  It is possible to enable nesting of low
//       priority interrupts, but this code is not setup for that and this nesting is not
//       enabled.

unsigned char timer2_extender = 0;

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

    // Check to see if we have an interrupt on timer 2
    if ( PIR1bits.TMR2IF ) {
        // LATBbits.LATB7 = !LATBbits.LATB7;
        PIR1bits.TMR2IF = 0;

        #if defined (MOTOR_PIC)
        {
            if ( timer2_extender == 10 ) {
                motor_semaphore = 1;
//            if(motor_index == 10)
//            {
//                //indicate message lost
//                motor_index = 1;
//            }
                motorArrayLeft[motor_index % 10] = ticks_left;
                motorArrayRight[motor_index % 10] = ticks_right;
                ticks_left = 0;
                ticks_right = 0;

                motor_index++;
                motor_semaphore = 0;
                timer2_extender = 0;
            } else {
                timer2_extender++;
            }
        }
#endif

    }

    // check to see if we have an interrupt on timer 0
    if (INTCONbits.TMR0IF) {
        LATBbits.LATB7 = !LATBbits.LATB7;
        INTCONbits.TMR0IF = 0; // clear this interrupt flag
        // call whatever handler you want (this is "user" defined)
#ifndef SENSOR_PIC
        timer0_int_handler();
#endif
        // LATDbits.LATD7 = !LATDbits.LATD7;
#if defined (MAIN_PIC)
        {
//            WriteUSART(i2c_q->end);
            // Check queue
//            if ( !isEmpty(i2c_q) ) {
//               unsigned char message;
//               LATBbits.LATB7 = !LATBbits.LATB7;
//               getQueue(i2c_q,message);
////               i2c_q->end = -1;
//               WriteUSART(i2c_q->end);
                //i2c_master_recv(0x0A, message, 0x4F);
//            }
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
                ConvertADC();
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
