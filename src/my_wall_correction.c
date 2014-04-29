#include "my_wall_correction.h"
#include "messages.h"
#include "my_i2c_master.h"
#include "maindefs.h"
#include <stdlib.h>

void wallCorrectionInit() {
    int i;
    for ( i=0; i < I2CMSGLEN; i++) {
        sensorDataBuf[i] = 0x00;
    }

    for (i=0; i< SENSORBUFLEN; i++) {
        wallSensorBack[i] = 0x00;
        wallSensorFront[i] = 0x00;
        wallSensorAhead[i] = 0x00;
        runningWallAverage[i] = 0x00;
    }

    // Set up close wall calibration port D3
    RPOR20 = 13;
    RPINR2 = 20;

    INTCON3bits.INT2IF = 0; // Set flag zero
    INTCON3bits.INT2IE = 1; // Enabled

    INTCON3bits.INT2IP = 1; // High priority

    INTCON2bits.INTEDG1 = 1; // Rising edge

}

void parallelComputation(int averageFront) {
    // Running wall correction
    runningWallAverage[runningIndex] = averageFront;
    runningIndex++;

    if ( runningIndex >= 10 ) {
           if ( tempWallCorrection && wallCorrection ) {
               int difference = 0;
               for (int i=0; i < 9; i++) {
                   if ( abs(runningWallAverage[i+1] - runningWallAverage[i]) < 20 )
                        difference += runningWallAverage[i+1] - runningWallAverage[i];
                }

               if ( difference > 8 ) {
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

               else if ( difference < -8 ) {
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

void boundaryComputation(int averageFront) {
    
   if ( firstSensorRead == 0 && wallCorrection && tempWallCorrection ) {

       // Check for boundary issues
       if ( (sensorOffset - averageFront) < -50 ) {
            // Dont do anything
       }
       else if ( (sensorOffset - averageFront) > 25 && leftCorrect==0) {
            i2cMstrMsgState = I2CMST_MOTOR_LOCAL;
            i2c_master_recv(0x0A, RoverMsgMotorLeft7, 0x4F);
            rightCorrect = 50;
            leftCorrect = 70;
       }
       else if ( (sensorOffset - averageFront) > 15 && leftCorrect == 0) {
            i2cMstrMsgState = I2CMST_MOTOR_LOCAL;
            i2c_master_recv(0x0A, RoverMsgMotorLeft2, 0x4F);
            rightCorrect = 30;
            leftCorrect = 50;
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

       // Check if we are in our blind spot
       if ( blindSpotSensor != 0 && blindSpotSensor != 0xFF ) {
           if ( averageFront - 5 < blindSpotSensor && leftCorrect == 0 ) {
                i2cMstrMsgState = I2CMST_MOTOR_LOCAL;
                i2c_master_recv(0x0A, RoverMsgMotorLeft7, 0x4F);
                rightCorrect = 50;
                leftCorrect = 70;
           }
       }
    }

}

void correctionMain(unsigned char* msgbuffer) {
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

        wallSensorIndex++; // Increment index

        // Once all of these arrays are filled up
        if ( wallSensorIndex >= 10 ) {
           // insertionSort(wallSensorFront,10);
           // insertionSort(wallSensorBack,10);

            // Calculate the averages
            int averageAhead = 0;
            int averageFront = 0;
            int averageBack = 0;
            for (int i =0; i < 10; i++) {
                averageFront += wallSensorFront[i];
                averageBack += wallSensorBack[i];
                averageAhead += wallSensorAhead[i];
            }
            averageFront = averageFront/10;
            averageBack = averageBack/10;
            averageAhead = averageAhead/10;

            // Give ARM buffer the averages
            sensorDataSem = 1;
            sensorDataBuf[0] = 4;
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
               parallelComputation(averageFront);
           }

           boundaryComputation(averageFront);
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
}

void configIR() {
    // Enable external interrupt for finish line
    RPOR21 = 13;
    RPINR1 = 21;

    INTCON3bits.INT1IF = 0; // Set flag zero
    INTCON3bits.INT1IE = 1; // Enabled

    INTCON3bits.INT1IP = 1; // High priority

    INTCON2bits.INTEDG1 = 1; // Rising edge

    TRISDbits.TRISD7 = 0;
    LATDbits.LATD7 = 0;
}