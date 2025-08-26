
#ifndef INC_STM32L4XX_TIM_H_
#define INC_STM32L4XX_TIM_H_


void timer2_1hz_init(void);
void timer2_pa5_output_compare(void);
void timer3_pa6_input_capture(void);

#define SR_UIF		(1U<<0)
#define SR_CC1IF		(1U<<1)

#endif /* INC_STM32L4XX_TIM_H_ */
