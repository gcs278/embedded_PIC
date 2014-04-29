/* 
 * File:   my_wall_correction.h
 * Author: grantspence
 *
 * Created on April 28, 2014, 8:27 PM
 */

#ifndef MY_WALL_CORRECTION_H
#define	MY_WALL_CORRECTION_H

#ifdef	__cplusplus
extern "C" {
#endif

#include "messages.h"

#define SENSORBUFLEN 10
    unsigned char sensorDataBuf[I2CMSGLEN];


    unsigned char wallSensorBack[SENSORBUFLEN];
    unsigned char wallSensorFront[SENSORBUFLEN];
    unsigned char wallSensorAhead[SENSORBUFLEN];
    unsigned char runningWallAverage[SENSORBUFLEN];


    int wallSensorIndex = 0;
    int runningIndex = 0;
    int lastSensor = 0;

    char sensorOffset = 0;

    unsigned char filledSensor = 0;
    unsigned char sensorDataSem = 0;

    unsigned char leftCorrect = 0;
    unsigned char rightCorrect = 0;

    unsigned char blindSpotSensor = 0xFF;

    void wallCorrectionInit();

    void parallelComputation(int averageFront);
    void boundaryComputation(int averageFront);

    void correctionMain(unsigned char* msgbuffer);

    void configIR();


#ifdef	__cplusplus
}
#endif

#endif	/* MY_WALL_CORRECTION_H */

