/* 
 * File:   my_adc.h
 * Author: grantspence
 *
 * Created on February 22, 2014, 1:36 PM
 */

#ifndef MY_ADC_H
#define	MY_ADC_H

#ifdef	__cplusplus
extern "C" {
#endif
    char ADCValue;
    
    int adc_index = 1;
    // ADC buffer, matches size of screen on ARM LCD
    unsigned char ADCArray[10];

    // ADC logic variables
    int responding = 0;
    int arrayPlaceHolder = 0;
    unsigned char adc_semaphore = 0;
    int messages_lost = 0;
    
    void setStateResponding(void);
    void setStateReading(void);
    unsigned char * SensorValues();
    void init_ADC();


#ifdef	__cplusplus
}
#endif

#endif	/* MY_ADC_H */

