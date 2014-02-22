#include "maindefs.h"
#include <plib/adc.h>
#include "my_adc.h"
// Configure the ADC to run on timer 0 interupt

void init_ADC()
{
    // Debug output pins set up
    TRISDbits.TRISD7 = 0;
    TRISDbits.TRISD6 = 0;

    //LATBbits.LATB6 = !LATBbits.LATB6;
    int ADCValue = 0;

    //Configure ADC
    OpenADC(ADC_FOSC_2 & ADC_RIGHT_JUST & ADC_20_TAD, ADC_CH0 & ADC_INT_ON & ADC_VREFPLUS_VDD & ADC_VREFMINUS_VSS, ADC_0ANA);
    ADC_INT_ENABLE(); //Easy ADC interrupt setup
    ei();
    // Begin ADC process
    ConvertADC();
}

/*
void readMyADC()
{
    ADCValue = ReadADC();
}
*/
