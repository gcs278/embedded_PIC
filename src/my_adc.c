#include "maindefs.h"
#include <plib/adc.h>
#include "my_adc.h"
// Configure the ADC to run on timer 0 interupt

void init_ADC()
{
    // Debug output pins set up
    TRISDbits.TRISD7 = 0;
    TRISDbits.TRISD6 = 0;

    TRISBbits.TRISB7 = 0;
    TRISBbits.TRISB6 = 0;
    TRISBbits.TRISB5 = 0;
    TRISBbits.TRISB4 = 0;


    //LATBbits.LATB6 = !LATBbits.LATB6;
    int ADCValue = 0;

    //Configure ADC
    OpenADC(ADC_FOSC_2 & ADC_RIGHT_JUST & ADC_20_TAD, ADC_CH0 & ADC_INT_ON & ADC_VREFPLUS_VDD & ADC_VREFMINUS_VSS, ADC_3ANA);
    ADC_INT_ENABLE(); //Easy ADC interrupt setup
    ei();

    // Begin ADC process
    ConvertADC();
}


void readSensor(int sensor)
{
    LATBbits.LATB7 = 1;
    LATBbits.LATB6 = 1;
    LATBbits.LATB5 = 0;
    LATBbits.LATB4 = 0;
    if(sensor == 0)
         OpenADC(ADC_FOSC_2 & ADC_RIGHT_JUST & ADC_20_TAD, ADC_CH0 & ADC_INT_ON & ADC_VREFPLUS_VDD & ADC_VREFMINUS_VSS, ADC_3ANA);
    else if (sensor == 1)
         OpenADC(ADC_FOSC_2 & ADC_RIGHT_JUST & ADC_20_TAD, ADC_CH1 & ADC_INT_ON & ADC_VREFPLUS_VDD & ADC_VREFMINUS_VSS, ADC_3ANA);
    else if(sensor == 2)
         OpenADC(ADC_FOSC_2 & ADC_RIGHT_JUST & ADC_20_TAD, ADC_CH2 & ADC_INT_ON & ADC_VREFPLUS_VDD & ADC_VREFMINUS_VSS, ADC_3ANA);
    else if (sensor == 3)
         OpenADC(ADC_FOSC_2 & ADC_RIGHT_JUST & ADC_20_TAD, ADC_CH3 & ADC_INT_ON & ADC_VREFPLUS_VDD & ADC_VREFMINUS_VSS, ADC_3ANA);
    ei();
    ConvertADC();
}

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

unsigned char* SensorValues() {
    ADCArray[0] = adc_index + messages_lost - 1;

    adc_index = 1;
    messages_lost = 0;
    return ADCArray;
}

/*
void readMyADC()
{
    ADCValue = ReadADC();
}
*/
