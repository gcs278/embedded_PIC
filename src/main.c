/*
 TEAM 1 - GRANT SPENCE, TAYLOR MCGOUGH, MARTIN ANILANE, MATT O'NEIL
 Milestone 1 = 2/11/2014
 */

#include "maindefs.h"
#include <stdio.h>
#ifndef __XC8
#include <usart.h>
#include <i2c.h>
#include <timers.h>
#else
#include <plib/usart.h>
#include <plib/i2c.h>
#include <plib/timers.h>
#endif
#include "interrupts.h"
#include "messages.h"
#include "my_uart.h"
#include "my_i2c.h"
#include "uart_thread.h"
#include "timer1_thread.h"
#include "timer0_thread.h"
#include "my_adc.h"
#include "my_wifly.h"
#include "my_motor.h"
#include "i2c_queue.h"
#include "my_i2c_master.h"

//Setup configuration registers
#ifdef __USE18F45J10
// CONFIG1L
#pragma config WDTEN = OFF      // Watchdog Timer Enable bit (WDT disabled (control is placed on SWDTEN bit))
#pragma config STVREN = OFF     // Stack Overflow/Underflow Reset Enable bit (Reset on stack overflow/underflow disabled)
#ifndef __XC8
// Have to turn this off because I don't see how to enable this in the checkboxes for XC8 in this IDE
#pragma config XINST = ON       // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode enabled)
#else
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode enabled)
#endif

// CONFIG1H
#pragma config CP0 = OFF        // Code Protection bit (Program memory is not code-protected)

// CONFIG2L
#pragma config FOSC = HSPLL     // Oscillator Selection bits (HS oscillator, PLL enabled and under software control)
#pragma config FOSC2 = ON       // Default/Reset System Clock Select bit (Clock selected by FOSC as system clock is enabled when OSCCON<1:0> = 00)
#pragma config FCMEN = ON       // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor enabled)
#pragma config IESO = ON        // Two-Speed Start-up (Internal/External Oscillator Switchover) Control bit (Two-Speed Start-up enabled)

// CONFIG2H
#pragma config WDTPS = 32768    // Watchdog Timer Postscale Select bits (1:32768)

// CONFIG3H
#pragma config CCP2MX = DEFAULT // CCP2 MUX bit (CCP2 is multiplexed with RC1)

#else

#ifdef __USE18F2680
#pragma config OSC = IRCIO67    // Oscillator Selection bits (Internal oscillator block, port function on RA6 and RA7)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF       // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

// CONFIG2L
#pragma config PWRT = OFF       // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = OFF      // Brown-out Reset Enable bits (Brown-out Reset disabled in hardware and software)
#pragma config BORV = 3         // Brown-out Reset Voltage bits (VBOR set to 2.1V)

// CONFIG2H
#pragma config WDT = OFF        // Watchdog Timer Enable bit (WDT disabled (control is placed on the SWDTEN bit))
#pragma config WDTPS = 32768    // Watchdog Timer Postscale Select bits (1:32768)

// CONFIG3H
#pragma config PBADEN = OFF     // PORTB A/D Enable bit (PORTB<4:0> pins are configured as digital I/O on Reset)
#pragma config LPT1OSC = OFF    // Low-Power Timer 1 Oscillator Enable bit (Timer1 configured for higher power operation)
#pragma config MCLRE = ON       // MCLR Pin Enable bit (MCLR pin enabled; RE3 input pin disabled)

// CONFIG4L
#pragma config STVREN = OFF     // Stack Full/Underflow Reset Enable bit (Stack full/underflow will not cause Reset)
#pragma config LVP = OFF        // Single-Supply ICSP Enable bit (Single-Supply ICSP disabled)
#pragma config BBSIZ = 1024     // Boot Block Size Select bits (1K words (2K bytes) Boot Block)
#ifndef __XC8
// Have to turn this off because I don't see how to enable this in the checkboxes for XC8 in this IDE
#pragma config XINST = ON       // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode enabled)
#endif

#else

#ifdef __USE18F26J50

// PIC18F26J50 Configuration Bit Settings

// CONFIG1L
#pragma config WDTEN = OFF      // Watchdog Timer (Disabled - Controlled by SWDTEN bit)
#pragma config PLLDIV = 3       // PLL Prescaler Selection bits (Divide by 3 (12 MHz oscillator input))
#pragma config STVREN = OFF     // Stack Overflow/Underflow Reset  (Disabled)
#pragma config XINST = ON       // Extended Instruction Set (Enabled)

// CONFIG1H
#pragma config CPUDIV = OSC1    // CPU System Clock Postscaler (No CPU system clock divide)
#pragma config CP0 = OFF        // Code Protect (Program memory is not code-protected)

// CONFIG2L
#pragma config OSC = HSPLL      // Oscillator (HS+PLL, USB-HS+PLL)
#pragma config T1DIG = OFF      // T1OSCEN Enforcement (Secondary Oscillator clock source may not be selected)
#pragma config LPT1OSC = OFF    // Low-Power Timer1 Oscillator (High-power operation)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor (Disabled)
#pragma config IESO = ON        // Internal External Oscillator Switch Over Mode (Enabled)

// CONFIG2H
#pragma config WDTPS = 32768    // Watchdog Postscaler (1:32768)

// CONFIG3L
#pragma config DSWDTOSC = T1OSCREF// DSWDT Clock Select (DSWDT uses T1OSC/T1CKI)
#pragma config RTCOSC = T1OSCREF// RTCC Clock Select (RTCC uses T1OSC/T1CKI)
#pragma config DSBOREN = OFF    // Deep Sleep BOR (Disabled)
#pragma config DSWDTEN = OFF    // Deep Sleep Watchdog Timer (Disabled)
#pragma config DSWDTPS = G2     // Deep Sleep Watchdog Postscaler (1:2,147,483,648 (25.7 days))

// CONFIG3H
#pragma config IOL1WAY = ON     // IOLOCK One-Way Set Enable bit (The IOLOCK bit (PPSCON<0>) can be set once)
#pragma config MSSP7B_EN = MSK7 // MSSP address masking (7 Bit address masking mode)

// CONFIG4L
#pragma config WPFP = PAGE_63   // Write/Erase Protect Page Start/End Location (Write Protect Program Flash Page 63)
#pragma config WPEND = PAGE_WPFP// Write/Erase Protect Region Select (valid when WPDIS = 0) (Page WPFP<5:0> through Configuration Words erase/write protected)
#pragma config WPCFG = OFF      // Write/Erase Protect Configuration Region (Configuration Words page not erase/write-protected)

// CONFIG4H
#pragma config WPDIS = OFF      // Write Protect Disable bit (WPFP<5:0>/WPEND region ignored)

#else

#ifdef __USE18F46J50

// PIC18F46J50 Configuration Bit Settings

// CONFIG1L
#pragma config WDTEN = OFF      // Watchdog Timer (Disabled - Controlled by SWDTEN bit)
#pragma config PLLDIV = 3       // PLL Prescaler Selection bits (Divide by 3 (12 MHz oscillator input))
#pragma config STVREN = OFF     // Stack Overflow/Underflow Reset (Disabled)
#ifndef __XC8
// Have to turn this off because I don't see how to enable this in the checkboxes for XC8 in this IDE
#pragma config XINST = ON       // Extended Instruction Set (Enabled)
#else
#pragma config XINST = OFF      // Extended Instruction Set (Disabled)
#endif

// CONFIG1H
#pragma config CPUDIV = OSC1    // CPU System Clock Postscaler (No CPU system clock divide)
#pragma config CP0 = OFF        // Code Protect (Program memory is not code-protected)

// CONFIG2L
#pragma config OSC = HSPLL      // Oscillator (HS+PLL, USB-HS+PLL)
#pragma config T1DIG = OFF      // T1OSCEN Enforcement (Secondary Oscillator clock source may not be selected)
#pragma config LPT1OSC = OFF    // Low-Power Timer1 Oscillator (High-power operation)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor (Disabled)
#pragma config IESO = ON        // Internal External Oscillator Switch Over Mode (Enabled)

// CONFIG2H
#pragma config WDTPS = 32768    // Watchdog Postscaler (1:32768)

// CONFIG3L
#pragma config DSWDTOSC = T1OSCREF// DSWDT Clock Select (DSWDT uses T1OSC/T1CKI)
#pragma config RTCOSC = T1OSCREF// RTCC Clock Select (RTCC uses T1OSC/T1CKI)
#pragma config DSBOREN = OFF    // Deep Sleep BOR (Disabled)
#pragma config DSWDTEN = OFF    // Deep Sleep Watchdog Timer (Disabled)
#pragma config DSWDTPS = G2     // Deep Sleep Watchdog Postscaler (1:2,147,483,648 (25.7 days))

// CONFIG3H
#pragma config IOL1WAY = ON     // IOLOCK One-Way Set Enable bit (The IOLOCK bit (PPSCON<0>) can be set once)
#pragma config MSSP7B_EN = MSK7 // MSSP address masking (7 Bit address masking mode)

// CONFIG4L
#pragma config WPFP = PAGE_63   // Write/Erase Protect Page Start/End Location (Write Protect Program Flash Page 63)
#pragma config WPEND = PAGE_WPFP// Write/Erase Protect Region Select (valid when WPDIS = 0) (Page WPFP<5:0> through Configuration Words erase/write protected)
#pragma config WPCFG = OFF      // Write/Erase Protect Configuration Region (Configuration Words page not erase/write-protected)

// CONFIG4H
#pragma config WPDIS = OFF      // Write Protect Disable bit (WPFP<5:0>/WPEND region ignored)

#else

Something is messed up.
The PIC selected is not supported or the preprocessor directives are wrong.

#endif
#endif
#endif
#endif

void main(void) {
    //int master_sent = 0;
    //char c;
    signed char length;
    unsigned char msgtype;
    unsigned char last_reg_recvd;
    uart_comm uc;
    i2c_comm ic;
    unsigned char msgbuffer[MSGLEN + 1];
    
    uart_thread_struct uthread_data; // info for uart_lthread
    timer1_thread_struct t1thread_data; // info for timer1_lthread
    timer0_thread_struct t0thread_data; // info for timer0_lthread

    unsigned char movingtest = 0; 
#ifdef __USE18F2680
    OSCCON = 0xFC; // see datasheet
    // We have enough room below the Max Freq to enable the PLL for this chip
    OSCTUNEbits.PLLEN = 1; // 4x the clock speed in the previous line
#else
#ifdef __USE18F45J10
    OSCCON = 0x82; // see datasheeet
    OSCTUNEbits.PLLEN = 0; // Makes the clock exceed the PIC's rated speed if the PLL is on
#else
#ifdef __USE18F26J50
    OSCCON = 0xE0; // see datasheeet
    OSCTUNEbits.PLLEN = 1;
#else
#ifdef __USE18F46J50
    OSCCON = 0xE0; //see datasheet
    OSCTUNEbits.PLLEN = 1;
#else
    Something is messed up.
    The PIC selected is not supported or the preprocessor directives are wrong.
#endif
#endif
#endif
#endif

    // initialize my uart recv handling code
    init_uart_recv(&uc);

    // initialize the i2c code
    init_i2c(&ic);

    // init the timer1 lthread
    init_timer1_lthread(&t1thread_data);

    init_timer0_lthread(&t0thread_data);
    // initialize message queues before enabling any interrupts
    init_queues();

    // set direction for PORTB to output
    // TRISB = 0x0;
#ifndef __USE18F26J50
    // set direction for PORTB to output
    TRISB = 0x0;
    LATB = 0x0;
    
#ifdef __USE18F46J50
    TRISBbits.TRISB4 = 1;
    TRISBbits.TRISB5 = 1;
#endif
    
#endif
    //TRISBbits.RB7 = 0;
    //LATB = 0x0;

    // UART TX interrupt flag
    IPR1bits.TXIP = 0;
    // how to set up PORTA for input (for the V4 board with the PIC2680)
    /*
            PORTA = 0x0;	// clear the port
            LATA = 0x0;		// clear the output latch
            ADCON1 = 0x0F;	// turn off the A2D function on these pins
            // Only for 40-pin version of this chip CMCON = 0x07;	// turn the comparator off
            TRISA = 0x0F;	// set RA3-RA0 to inputs
     */

    // initialize Timers

    //OpenTimer1(TIMER_INT_ON & T1_PS_1_1 & T1_16BIT_RW & T1_SOURCE_INT & T1_OSC1EN_OFF & T1_SYNC_EXT_OFF);

    // Peripheral interrupts can have their priority set to high or low
    // enable high-priority interrupts and low-priority interrupts
    enable_interrupts();

    // Decide on the priority of the enabled peripheral interrupts
    // 0 is low, 1 is high
    // Timer1 interrupt
    IPR1bits.TMR1IP = 0;
    // USART RX interrupt
    IPR1bits.RCIP = 0;
    // I2C interrupt
    IPR1bits.SSPIP = 1;

    IPR1bits.ADIP = 1;


    // configure the hardware i2c device as a slave (0x9E -> 0x4F) or (0x9A -> 0x4D)
#if 1
    // Note that the temperature sensor Address bits (A0, A1, A2) are also the
    // least significant bits of LATB -- take care when changing them
    // They *are* changed in the timer interrupt handlers if those timers are
    //   enabled.  They are just there to make the lights blink and can be
    //   disabled.
#else
    // If I want to test the temperature sensor from the ARM, I just make
    // sure this PIC does not have the same address and configure the
    // temperature sensor address bits and then just stay in an infinite loop
    i2c_configure_slave(0x9A);
    LATBbits.LATB1 = 1;
    LATBbits.LATB0 = 1;
    LATBbits.LATB2 = 1;
    for (;;);
#endif

    // must specifically enable the I2C interrupts
    PIE1bits.SSPIE = 1;


    // Calculating the UART baud rate, equation in documentation
    // 12000000 / (16 * (77 + 1)) = ~ 9600
    // 12000000 / (16 * (38 + 1)) = ~ 19200
    // 48000000 / (16 * (155 + 1)) = ~ 19200
    // 48000000 / (16 * (12 + 1)) = 230,400
    // configure the hardware USART device
#ifdef __USE18F46J50
    Open1USART(USART_TX_INT_OFF & USART_RX_INT_ON & USART_ASYNCH_MODE & USART_EIGHT_BIT &
            USART_CONT_RX & USART_BRGH_HIGH, 12);
    
#else
    OpenUSART(USART_TX_INT_OFF & USART_RX_INT_ON & USART_ASYNCH_MODE & USART_EIGHT_BIT &
            USART_CONT_RX & USART_BRGH_HIGH, 38);
#endif

#if defined(ARM_PIC)
    OpenTimer0(TIMER_INT_ON & T0_16BIT & T0_SOURCE_INT & T0_PS_1_256);
    // Initialize the WiFLy
    // 3/2/14 - SET UP DOESN'T REQUIRE WIFLY INIT ANY MORE
    //initWiFly();
    i2c_configure_slave(0x9E);

    // Rover data buffer - buffer for giving ARM most recent data
    // Allows for asynchronous communication
    unsigned char roverDataBuf[5][I2CMSGLEN];
    unsigned char roverDataBufIndex = 0;
    int j = 0;
    // Initialize the first bit to FF, the garage bit
    for (j; j < I2CMSGLEN; j++) {
        roverDataBuf[j][0] = 0xFF;
    }
    // initialize my uart recv handling code
    init_uart_recv(&uc);

    i2c_queue* i2c_q;

    createQueue(&i2c_q,10);
#elif defined(SENSOR_PIC)
    OpenTimer0(TIMER_INT_ON & T0_16BIT & T0_SOURCE_INT & T0_PS_1_2);
    // Set up ADC
    init_ADC();
    i2c_configure_slave(0x9C); // send with address of 4E from master and aardvark
#elif defined(MOTOR_PIC)
    // Timer 2 Interrupt
    IPR1bits.TMR2IP = 1;
    
    TRISBbits.RB5 = 1;
    TRISCbits.RC0 = 1;
    TRISBbits.RB6 = 0;
    LATBbits.LATB6 = 0;

    OpenTimer0(TIMER_INT_ON & T0_8BIT & T0_SOURCE_EXT & T0_PS_1_1 & T0_EDGE_FALL & T0_EDGE_RISE );
    OpenTimer1(TIMER_INT_ON & T1_8BIT_RW & T1_SOURCE_EXT & T1_SYNC_EXT_OFF & T1_PS_1_1 & T1_OSC1EN_OFF );
    OpenTimer2(TIMER_INT_ON & T2_PS_1_16 & T2_POST_1_16); // 10 ms is 16 and 7

    // Configure timer 1 so it only takes 12 ticks
    WriteTimer1(65523);
    WriteTimer0(243);

    T1CONbits.RD16 = 0;
    i2c_configure_slave(0x9E);
    motor_init();

//    TRISBbits.RB0 = 1;
//    TRISBbits.RB1 = 1;
//
//    // Enable interrupt
//    INTCONbits.RBIE = 1;
//    // Set flag to zero
//    INTCONbits.RBIF = 0;
//
//
//    INTCON2bits.RBIP = 1;
//    INTCON2bits.INTEDG0 = 1;
    
#elif defined(MAIN_PIC)
    i2c_queue i2c_q;
    i2c_configure_master();
    // createQueue(i2c_q,10);

    // (12,000,00 / Prescale) * 65535
    OpenTimer0(TIMER_INT_ON & T0_16BIT & T0_SOURCE_INT & T0_PS_1_1); // = 5.46 ms
    OpenTimer1(TIMER_INT_ON & T1_16BIT_RW & T0_SOURCE_INT & T0_PS_1_32); // = 174 ms


    // A count buffer, to store the count while I2C slave respond
    unsigned char msgCount;

    createQueue(&i2c_q,5);
#endif

    /* Junk to force an I2C interrupt in the simulator (if you wanted to)
    PIR1bits.SSPIF = 1;
    _asm
    goto 0x08
    _endasm;
     */

    LATBbits.LATB7 = 1;
    /*
     WE ARE NOT USING THE FOLLOWING WHILE LOOP FOR MESSAGE PASSING
     */

    // printf() is available, but is not advisable.  It goes to the UART pin
    // on the PIC and then you must hook something up to that to view it.
    // It is also slow and is blocking, so it will perturb your code's operation
    // Here is how it looks: printf("Hello\r\n");


    // loop forever
    // This loop is responsible for "handing off" messages to the subroutines
    // that should get them.  Although the subroutines are not threads, but
    // they can be equated with the tasks in your task diagram if you
    // structure them properly.
    while (1) {

        // Call a routine that blocks until either on the incoming
        // messages queues has a message (this may put the processor into
        // an idle mode)
        block_on_To_msgqueues();

        // At this point, one or both of the queues has a message.  It
        // makes sense to check the high-priority messages first -- in fact,
        // you may only want to check the low-priority messages when there
        // is not a high priority message.  That is a design decision and
        // I haven't done it here.
        length = ToMainHigh_recvmsg(MSGLEN, &msgtype, (void *) msgbuffer);
        if (length < 0) {
            // no message, check the error code to see if it is concern
            if (length != MSGQUEUE_EMPTY) {
                // This case be handled by your code.
            }
        } else {
            switch (msgtype) {
                case MSGT_TIMER0:
                {
                    timer0_lthread(&t0thread_data, msgtype, length, msgbuffer);
                    break;
                };
                case MSGT_I2C_DATA:
                {
#if defined(ARM_PIC)
                    // LATBbits.LATB7 = !LATBbits.LATB7;

                    i2c_master_cmd message;
                    
                    if ( !isEmpty(&i2c_q) ) {
                        getQueue(&i2c_q,&message);
                    }
                    else {
                        int i;
                        for (i = 0; i < I2CMSGLEN; i++)
                            message.data[i] = 0xFF;
                    }

                    // Reply with most recent data in buffer
                    start_i2c_slave_reply(I2CMSGLEN, message.data);

                    // Marked the buffer used
                    roverDataBuf[roverDataBufIndex][0] = 0xFF;

#elif defined(SENSOR_PIC)

#elif defined(MOTOR_PIC)
                // Reply with random tick values
                int length = 10;
                    unsigned char motorTEST[10];
                    int i =0;
                    for(i;i<10;i++) {
                        motorTEST[i] = 0x12;
                    }
                unsigned char * tickbuffer = motorTickValue(msgbuffer[0]);
                start_i2c_slave_reply(length, tickbuffer);

#elif defined(MAIN_PIC)
                    // LATBbits.LATB7 = !LATBbits.LATB7;
                        LATBbits.LATB6 = !LATBbits.LATB6;
                        
                        i2c_master_cmd message;
                        
                        message.msgType = msgbuffer[0];
                        message.msgCount = msgbuffer[1];
                        
                        putQueue(&i2c_q,message);
                
//                    if ( master_sent == 0 ) {
                   
                           //ToMainHigh_sendmsg(2, MSGT_I2C_DATA, (void *) msgbuffer);
                
//                        master_sent = 1;
                      // msgCount = msgbuffer[1];

                        // i2c_master_recv(0x02, 0x01, 0x4E);
                   // }
//                    else if (msgbuffer[0] == 0x05) {
//                        i2c_master_recv(0x02, 0x05, 0x4F);
//                        // i2c_master_recv(0x02, 0x05, 0x4E);
//                    }
                    // LATDbits.LATD7 = !LATDbits.LATD7;

#endif
                    break;
                }
                case MSGT_I2C_DBG:
                {
                    // Here is where you could handle debugging, if you wanted
                    // keep track of the first byte received for later use (if desired)
                    last_reg_recvd = msgbuffer[0];
                    break;
                };

                case MSGT_QUEUE_GET_DATA:
                {
#ifdef MAIN_PIC
                    if ( !isEmpty(&i2c_q) && q_semiphore == 0 ) {
                       i2c_master_cmd message;
                       LATBbits.LATB7 = !LATBbits.LATB7;
                       getQueue(&i2c_q,&message);

                        i2cMstrMsgState = I2CMST_MOTOR;
                        if ( msgbuffer[0] == moveForwardFull )
                            movingtest = 1;
                        else if( msgbuffer[0] == moveStop )
                            movingtest = 0;

                        msgCount = message.msgCount;
                        // See what pic the message goes to
                        if ( msgbuffer[0] == sensorDataFull )
                            i2c_master_recv(0x0A, 0x05, 0x4E);
                        else
                            i2c_master_recv(0x0A, message.msgType, 0x4F);
                        
                    }
#endif
                    break;
                }
                case MSGT_I2C_RQST:
                {
                    // NOT USING THIS RIGHT NOW - GRANT

                    // Generally, this is *NOT* how I recommend you handle an I2C slave request
                    // I recommend that you handle it completely inside the i2c interrupt handler
                    // by reading the data from a queue (i.e., you would not send a message, as is done
                    // now, from the i2c interrupt handler to main to ask for data).
                    //
                    // The last byte received is the "register" that is trying to be read
                    // The response is dependent on the register.
                   /* switch (last_reg_recvd) {
                        case 0xaa:
                        {
                            length = 1;
                            //msgbuffer[0] = 0x55;
                            //msgbuffer[0] = returnADCValue();
                            //msgbuffer[1] = 0xAA;
                            break;
                        }
                        case 0xa8:
                        {
                            length = 1;
                            //msgbuffer[0] = 0x3A;
                            //msgbuffer[0] = returnADCValue();
                            break;
                        }
                        case 0xa9:
                        {
                            length = 1;
                            //msgbuffer[0] = 0xA3;
                            //msgbuffer[0] = returnADCValue();
                            break;
                        }
                    };*/

                    break;
                };
                case MSGT_I2C_MASTER_RECV_COMPLETE:
                {
                    // What type of message we are getting
                    // Allows for us to tell if it is a LOCAL message
                    switch (i2cMstrMsgState) {
                        case I2CMST_LOCAL_SENSOR: {

    //                        if ( msgbuffer[1] > 30 ) {
    //
    //                            if ( movingtest == 0) {
    //                                i2cMstrMsgState = I2CMST_LOCAL_MOTOR;
    //                                movingtest = 1;
    //                                i2c_master_recv(0x0A, moveForwardFull, 0x4F);
    //                            }
    //                        }
                            int length = msgbuffer[0];
                            if ( length > 0 ) {
                                int i;
                                int average;
                                for (i=0; i < length; i++)
                                    average += msgbuffer[i];
                                average = average / length;
                                if ( msgbuffer[1] < 30 ) {
                                    if ( movingtest == 1) {
                                        i2cMstrMsgState = I2CMST_MOTOR;
                                        movingtest = 0;
                                        i2c_master_recv(0x0A, RoverMsgMotorStop, 0x4F);
                                    }
                                }
                            }
                            break;
                        }

                        case I2CMST_LOCAL_WALLSENSOR: {
                            // Calculate correction response
                            int length = msgbuffer[0];
                            if ( length > 0 ) {
                                int i;
                                int average;
                                for (i=0; i < length; i++) {
                                    if (i < 9)
                                        average += msgbuffer[i];
                                }
                                average = average / length;

                                if ( average < 10 ) {
                                    // Tell motors to move away from the wall
                                    i2cMstrMsgState = I2CMST_MOTOR;
                                    i2c_master_recv(0x0A, RoverMsgMotorLeft2, 0x4F);
                                }
                                else if ( average > 38) {
                                    // Tell motors to move closer to the wall
                                    i2cMstrMsgState = I2CMST_MOTOR;
                                    i2c_master_recv(0x0A, RoverMsgMotorRight2, 0x4F);
                                }
                            }
                            break;
                        }
                        
                        default: {
                            // Just a regular message from the ARM, send it back
                            ToMainLow_sendmsg(length, MSGT_UART_SEND, msgbuffer );
                            break;
                        }
                    }
                    break;
                }
                case MSGT_BUF_PUT_DATA:
                {
#if defined(ARM_PIC)
                    // Store data that we received in the buffer for ARM
                    i2c_master_cmd message;

                    // Put the data into the current position of the buffer
                    int i;
                    for (i=0; i<I2CMSGLEN; i++) {
                        message.data[i] = msgbuffer[i];
                    }
                    
                    putQueue(&i2c_q,message);
                    break;
#endif
                }
                default:
                {
                    // Your code should handle this error
                    break;
                };
            };
        }

        // Check the low priority queue
        length = ToMainLow_recvmsg(MSGLEN, &msgtype, (void *) msgbuffer);
        if (length < 0) {
            // no message, check the error code to see if it is concern
            if (length != MSGQUEUE_EMPTY) {
                // Your code should handle this situation
            }
        } else {
            switch (msgtype) {
                case MSGT_TIMER1:
                {
                    timer1_lthread(&t1thread_data, msgtype, length, msgbuffer);
                    break;
                };
                case MSGT_OVERRUN:
                case MSGT_UART_SEND:
                {
                    //uart_send_thread()
#if defined(ARM_PIC)
                    uart_send_data(msgbuffer,2);                 
#elif defined(SENSOR_PIC)

#elif defined(MAIN_PIC)
                    uart_sendthread(length, msgbuffer,msgCount);

#elif defined(MOTOR_PIC)

                    // Send the msg to the motor control thread
                    motor_encode_lthread(msgbuffer[0]);
#endif


                }
                case MSGT_UART_DATA:
                {
                    uart_lthread(&uthread_data, msgtype, length, msgbuffer);
                    break;
                };
                default:
                {
                    // Your code should handle this error
                    break;
                };
            };
        }
    }

}