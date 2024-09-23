# Development notes

- When adding new files, add `.c` and `.h` files to the `Core/Inc/` and `Core/Src/` directories, respectively. In order for these files to be seen by the compiler, we need to add them to `CMakeLists.txt`. Simply add the `.c` files to the `target_sources(stm32cubemx INTERFACE` list section. The `.h` files will be included automatically.

# Documentation

Documentation for the HAL libraries could be found here: https://www.st.com/en/embedded-software/stm32cube-mcu-mpu-packages/documentation.html

For specifically STM32F0 processors, see here: https://www.st.com/resource/en/user_manual/um1785-description-of-stm32f0-hal-and-lowlayer-drivers-stmicroelectronics.pdf

Different file-open modes: http://elm-chan.org/fsw/ff/doc/open.html

Good blog on STM32 "gotcha's": http://www.efton.sk/STM32/gotcha/index.html


# Design Notes

## For controlling mux select and adc

> Note: We'll measure the VBATT_QTR, Thermisters, Isense, 2.5V net, etc. after a full frame read
> of the halls

Gameplan:
 
1. Select MUX 0,0
2. Start ADC with interrupt
3. In ADC ISR:
   1. Save value in global array (for correct px idx)
   2. Increment idx
   3. Increment MUX sel
   4. Start Timer 7 with interrupt (This will wait for propagation delay for level shifter and settling delay for analog mux)
   5. If idx == NUM_PX, then start changing the ADC channels to get the extra adc values.
4. In Timer 7 ISR:
   1. Start ADC with interrupt again

In main(), we will constantly be iterating through the hall ADC values and changing the LED colors.
We will still be able to stop/start the ADC interrupt from within main() using HAL_ADC_Stop_IT(). 

### Delay times:

- Level shifter: max of 12ns
- Analog mux: max of 60ns (might be safe to double this)

Delay to program in: 1us (count of 48, no prescaler)