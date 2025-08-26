

#ifndef INC_STM32L4XX_ADC_DRIVER_H_
#define INC_STM32L4XX_ADC_DRIVER_H_


#include <stdint.h>

void adc_init(void);   //adc intialisation
uint32_t adc_read(void); //read the sensor value
void start_conversion(void); //start the conversion
#endif /* INC_STM32L4XX_ADC_DRIVER_H_ */
