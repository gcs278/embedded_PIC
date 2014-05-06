#include "my_wall_correction.h"
#include "messages.h"
#include "my_i2c_master.h"
#include "maindefs.h"
#include <stdlib.h>

void wallCorrectionInit() {
#if defined MAIN_PIC
    int i;
    for ( i=0; i < I2CMSGLEN; i++) {
        sensorDataBuf[i] = 0x00;
    }

    for (i=0; i< SENSORBUFLEN; i++) {
        wallSensorBack[i] = 0x00;
        wallSensorFront[i] = 0x00;
        wallSensorAhead[i] = 0x00;
        wallSensorIR[i] = 0x00;
        runningWallAverage[i] = 0x00;
    }

    // Set up close wall calibration port D3
    RPOR20 = 13;
    RPINR2 = 20;

    INTCON3bits.INT2IF = 0; // Set flag zero
    INTCON3bits.INT2IE = 1; // Enabled

    INTCON3bits.INT2IP = 1; // High priority

    INTCON2bits.INTEDG1 = 1; // Rising edge
#endif
}

void parallelComputation(int averageFront, int averageIR) {
    // Running wall correction
    runningWallAverage[runningIndex] = averageFront;
    runningIndex++;

    if ( runningIndex >= SENSORBUFLEN ) {
           if ( tempWallCorrection && wallCorrection ) {
               int difference = 0;
               for (int i=0; i < SENSORBUFLEN-1; i++) {
                   if ( abs(runningWallAverage[i+1] - runningWallAverage[i]) < 13
                           && runningWallAverage[i] < 150 && runningWallAverage[i+1] < 150 )
                        difference += runningWallAverage[i+1] - runningWallAverage[i];
                }

               if ( averageFront > 175 ||  averageIR < 7 ) {
                   //
               }
               else if ( difference > 7 ) {
                    i2cMstrMsgState = I2CMST_MOTOR_LOCAL;
                    i2c_master_recv(0x0A, RoverMsgMotorRight7, 0x4F);
                    rightCorrect = 50;
                    leftCorrect = 70;
               }
               else if ( difference > 3 ) {
                    i2cMstrMsgState = I2CMST_MOTOR_LOCAL;
                    i2c_master_recv(0x0A, RoverMsgMotorRight2, 0x4F);
                    rightCorrect = 50;
                    leftCorrect = 30;
               }

               else if ( difference < -7 ) {
                    i2cMstrMsgState = I2CMST_MOTOR_LOCAL;
                    i2c_master_recv(0x0A, RoverMsgMotorLeft7, 0x4F);
                    rightCorrect = 70;
                    leftCorrect = 50;
               }
               else if ( difference < -3 ) {
                    i2cMstrMsgState = I2CMST_MOTOR_LOCAL;
                    i2c_master_recv(0x0A, RoverMsgMotorLeft2, 0x4F);
                    rightCorrect = 30;
                    leftCorrect = 50;
               }
           }


       runningIndex=0;
    }

}

void boundaryComputation(int averageFront, int averageIR) {
   
   if ( firstSensorRead == 0 && wallCorrection && tempWallCorrection ) {

       // Check for boundary issues
       if ( averageFront > 175 || averageIR < 7 ) {
            // Dont do anything
       }

       else if ( (sensorOffset - averageFront) < -25 && rightCorrect == 0 ) {
            i2cMstrMsgState = I2CMST_MOTOR_LOCAL;
            i2c_master_recv(0x0A, RoverMsgMotorRight7, 0x4F);
            rightCorrect = 70;
            leftCorrect = 50;
       }

       else if ( (sensorOffset - averageFront) < -15 && rightCorrect == 0 ) {
            i2cMstrMsgState = I2CMST_MOTOR_LOCAL;
            i2c_master_recv(0x0A, RoverMsgMotorRight2, 0x4F);
            rightCorrect = 50;
            leftCorrect = 30;
       }

       else if ( averageIR > 105 && leftCorrect == 0 ) {
            i2cMstrMsgState = I2CMST_MOTOR_LOCAL;
            i2c_master_recv(0x0A, RoverMsgMotorLeft7, 0x4F);
            rightCorrect = 50;
            leftCorrect = 70;
       }
       else if ( averageIR > 90 && leftCorrect == 0) {
            i2cMstrMsgState = I2CMST_MOTOR_LOCAL;
            i2c_master_recv(0x0A, RoverMsgMotorLeft2, 0x4F);
            rightCorrect = 30;
            leftCorrect = 50;
       }

       // Check if we are in our blind spot
       if ( blindSpotSensor != 0 && blindSpotSensor != 0xFF && averageFront < 150  ) {
           if ( averageFront - 10 < blindSpotSensor && leftCorrect == 0 ) {
                i2cMstrMsgState = I2CMST_MOTOR_LOCAL;
                i2c_master_recv(0x0A, RoverMsgMotorLeft7, 0x4F);
                rightCorrect = 50;
                leftCorrect = 70;
           }
       }
    }

}

void correctionMain(unsigned char* msgbuffer) {
#if defined MAIN_PIC
    // Decrement delay variables if not 0
    if ( leftCorrect > 0 )
        leftCorrect--;
    if ( rightCorrect > 0 )
        rightCorrect--;


    // Only read sensor message if greater than 3 (4) and they aren't 0
    if ( msgbuffer[0] > 3 && msgbuffer[4] != 0  && msgbuffer[3] != 0 ) {
        LATAbits.LA3 = !LATAbits.LA3; // Toggle sensor throughput LED

        // Load the running buffers
        wallSensorFront[wallSensorIndex] = msgbuffer[3];
        wallSensorBack[wallSensorIndex] = msgbuffer[4];//-sensorOffset;
        wallSensorAhead[wallSensorIndex] = msgbuffer[2];
        wallSensorIR[wallSensorIndex] = msgbuffer[1];

        wallSensorIndex++; // Increment index

        // Once all of these arrays are filled up
        if ( wallSensorIndex >= SENSORBUFLEN ) {
           // insertionSort(wallSensorFront,10);
           // insertionSort(wallSensorBack,10);

            // Calculate the averages
            int averageAhead = 0;
            int averageIR = 0;
            int averageFront = 0;
            int averageBack = 0;
            for (int i =0; i < SENSORBUFLEN; i++) {
                averageFront += wallSensorFront[i];
                averageBack += wallSensorBack[i];
                averageAhead += wallSensorAhead[i];
                averageIR += wallSensorIR[i];
            }
            averageFront = averageFront/SENSORBUFLEN;
            averageBack = averageBack/SENSORBUFLEN;
            averageAhead = averageAhead/SENSORBUFLEN;
            averageIR = averageIR/SENSORBUFLEN;
            
            //Write1USART(averageAhead);
            // Give ARM buffer the averages
            sensorDataSem = 1;
            sensorDataBuf[0] = 4;
            sensorDataBuf[1] = averageIR;
            sensorDataBuf[2] = averageAhead;
            sensorDataBuf[3] = averageFront;
            sensorDataBuf[4] = averageBack;

            // When we calibrate the blind spot
            if ( !blindSpotSensor ) {
                LATAbits.LA2 = 1; // Calibrate LED
                blindSpotSensor = averageFront;
                Write1USART(blindSpotSensor);
            }

            // Set the finish line variable if we crossed
            if ( finishLine ) {
                sensorDataBuf[5] = 0x01;
                finishLine = 0; // Reset to 0
            }
            else {
                sensorDataBuf[5] = 0x00;
            }

            sensorDataSem = 0;

           // Calibrate Sensors
           if ( firstSensorRead == 1 ){
               sensorOffset = averageFront;
               Write1USART(sensorOffset);
               firstSensorRead = 0;
           }
           else {
               parallelComputation(averageFront, averageIR);
           }
//            unsigned char valuesGood = 1;
//            for (int i =0; i < SENSORBUFLEN-1; i++)
//            {
//                if ( abs(wallSensorFront[i+1]-wallSensorFront[i]) > 10 )
//                    valuesGood = 0;
//
//            }
//            if ( valuesGood )
            //Write1USART(averageIR);
             boundaryComputation(averageFront, averageIR);
//           while(Busy1USART());
//           Write1USART(averageBack);
//           while(Busy1USART());
//           Write1USART(medianFront);
//           while(Busy1USART());
//           Write1USART(medianBack);

            filledSensor = 1;
            wallSensorIndex = 0;
        }


    }
#endif
}

void configIR() {
#if defined MAIN_PIC
    // Enable external interrupt for finish line
    RPOR21 = 13;
    RPINR1 = 21;

    INTCON3bits.INT1IF = 0; // Set flag zero
    INTCON3bits.INT1IE = 1; // Enabled

    INTCON3bits.INT1IP = 1; // High priority

    INTCON2bits.INTEDG1 = 1; // Rising edge

    TRISDbits.TRISD7 = 0;
    LATDbits.LATD7 = 0;
#endif
}