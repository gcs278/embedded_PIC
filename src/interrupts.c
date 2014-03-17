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
//----------------------------------------------------------------------------
// Note: This code for processing interrupts is configured to allow for high and
//       low priority interrupts.  The high priority interrupt can interrupt the
//       the processing of a low priority interrupt.  However, only one of each type
//       can be processed at the same time.  It is possible to enable nesting of low
//       priority interrupts, but this code is not setup for that and this nesting is not
//       enabled.

#define motorArraySize 10

int semaphore = 0;
int motor_value = 0;
int sensor_value = 30;
unsigned int motor_index = 1;
unsigned char motorArrayLeft[motorArraySize];
unsigned char motorArrayRight[motorArraySize];
unsigned char motorSend[10];

#if defined(SENSOR_PIC)

char ADCValue;
// ADC buffer, matches size of screen on ARM LCD
char ADCArray[299];
// ADC logic variables
int responding = 0;
int arrayPlaceHolder = 0;
int sendingPlaceHolder = 299;
int start_stop = 0;


// PIC is responding to ARM I2C request
void setStateResponding()
{
    responding = 1;
}

// PIC is reading ADC values
void setStateReading()
{
    responding = 0;
}

// Returns a current ADC value
char returnADCValue()
{
    // BUFFER is full
    if(sendingPlaceHolder == 299)
    {
        responding = 0;
        sendingPlaceHolder = 0;
    }
    else
        sendingPlaceHolder++;
    //if(sendingPlaceHolder == 0)
        //return 0xFF;
    //else
    // Return current buffer value
    return ADCArray[sendingPlaceHolder];
}

#elif defined (MOTOR_PIC)
unsigned char timer2_extender = 0;

#endif

unsigned char* motorTickValue(unsigned char msgRequest)
{
    while(semaphore == 1){};
    
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
                semaphore = 1;
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
                semaphore = 0;
                timer2_extender = 0;
            } else {
                timer2_extender++;
            }
        }
#endif

    }

    // check to see if we have an interrupt on timer 0
    if (INTCONbits.TMR0IF) {
        //LATBbits.LATB7 = !LATBbits.LATB7;
        INTCONbits.TMR0IF = 0; // clear this interrupt flag
        // call whatever handler you want (this is "user" defined)
        timer0_int_handler();
        // LATDbits.LATD7 = !LATDbits.LATD7;
        #if defined (MAIN_PIC)
        {
//            if(start_stop == 0)
//            {
//                i2c_master_recv(0x0A, 0x05, 0x4F);
//                start_stop = 1;
//            }
//            else if (start_stop == 1)
//            {
//                i2c_master_recv(0x0A, 0x05, 0x4E);
//                start_stop = 0;
//            }
        }
        #else
        {
            semaphore = 1;
            #if defined(SENSOR_PIC)
            {
                if(motor_index == 10)
                    motor_index = 1;
//                motorArray[motor_index++] = sensor_value++;
//                motorArray[motor_index++] = sensor_value++;
//                motorArray[motor_index++] = sensor_value++;
//                motorArray[motor_index++] = sensor_value++;
            }
            #endif
            semaphore = 0;
        }
        #endif



        //ConvertADC();
    }

    // here is where you would check other interrupt flags.

        //check if the interrupt is caused by ADC
    if(PIR1bits.ADIF == 1)
    {
#if defined (SENSOR_PIC)
        //ADCValue = ReadADC();
        //Reset interrupt flag and start conversion again
        PIR1bits.ADIF = 0;
        LATDbits.LATD6 = !LATDbits.LATD6;
        int pureADCValue = ReadADC();
        ADCValue = pureADCValue >> 2;
        if(responding == 0)
        {
            if(arrayPlaceHolder == 299)
            {
                responding = 1;
                arrayPlaceHolder = 0;
            }
            else
            {
                ADCArray[arrayPlaceHolder] = ADCValue;
                arrayPlaceHolder++;
            }
        }
#endif
        //ConvertADC();
    }
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
