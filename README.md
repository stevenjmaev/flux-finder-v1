# Development notes

- When adding new files, add `.c` and `.h` files to the `Core/Inc/` and `Core/Src/` directories, respectively. In order for these files to be seen by the compiler, we need to add them to `CMakeLists.txt`. Simply add the `.c` files to the `target_sources(stm32cubemx INTERFACE` list section. The `.h` files will be included automatically.