# Development notes

- When adding new files, add `.c` and `.h` files to the `Core/Inc/` and `Core/Src/` directories, respectively. In order for these files to be seen by the compiler, we need to add them to `CMakeLists.txt`. Simply add the `.c` files to the `target_sources(stm32cubemx INTERFACE` list section. The `.h` files will be included automatically.

# Documentation

Documentation for the HAL libraries could be found here: https://www.st.com/en/embedded-software/stm32cube-mcu-mpu-packages/documentation.html

For specifically STM32F0 processors, see here: https://www.st.com/resource/en/user_manual/um1785-description-of-stm32f0-hal-and-lowlayer-drivers-stmicroelectronics.pdf