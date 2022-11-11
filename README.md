# STM32_device_driver_for_DHT22_sensor
 Device driver for DHT22 humidity and temperature sensor portable to any STM32 microcontroller

Software was developed on an STM32WL55 microcontroller on the Cortex M4 core but can be employed on any STM32 device by paying attention to:

1.  One GPIO pin is required to connect to the data pin of the DHT22 in open drain output stage. 
    A 4.7k resistor is connected between data and VDD and a 100 nF decoupling capacitor across the power pins.

2.  Refer to the datasheet of the specific STM32 device used to determine the bit-mask which needs
    to be applied to the MODER register in order to switch the GPIO pin between INPUT/OUTPUT. The mask data shown applies to PC2 but can easily be changed.

3.  The timer used in this case is TIM16 but again, most timers can be used. The pre-scalar for the
    timer clock should ensure a frequency of 1 MHz so that the clock count increments every 1 us.
    In this case the max clock frequency for APB which the timer is connected is 32 MHz and the pre-sclalar is 31 (because 0 counts as pre-scalar = 1).

4.  USART2 is selected to display data because this is the same serial connection used by
    ST-LINK and allows display of data on the built in serial console within the CubeIDE.

5.  By storing the duration of start sequence, training bits, and data bits, it was found that
    timeing is accurate to +/- 2us.
    During a continuous 5h test, no checksum nor timing errors were recorded.