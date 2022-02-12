#ifndef ZEPHYR_INCLUDE_DRIVERS_ADC_STM32_H_
#define ZEPHYR_INCLUDE_DRIVERS_ADC_STM32_H_

typedef void (*adc_stm32_read_continuous_half_complete_callback)(int status);

int adc_stm32_read_continuously(const struct device *dev, const struct adc_sequence *sequence, adc_stm32_read_continuous_half_complete_callback cb);

#endif /* ZEPHYR_INCLUDE_DRIVERS_ADC_STM32_H_ */
