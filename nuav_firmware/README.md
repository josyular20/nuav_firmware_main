
# NUAV Firmware
Ooga booga firmware

## Build
```
mkdir build
cd build
cmake -G "MinGW Makefiles" -DCMAKE_TOOLCHAIN_FILE="../arm-none-eabi-gcc.cmake" -DCMAKE_BUILD_TYPE="Debug" ..
make
```

### Note for MacOS
```
cmake -G "Unix Makefiles" -DCMAKE_TOOLCHAIN_FILE="../arm-none-eabi-gcc.cmake" -DCMAKE_BUILD_TYPE="Debug" ..
```

## Upload 
```
dfu-util -a 0 -i 0 -s 0x08000000:leave -D .\active_tether.bin
```
NOTE: If entering bootloader via software, the reset button is the only way to exit it

## Requirements
- [Cmake](https://cmake.org/download/)
- [ARM compiler](https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm/downloads)
- [dfu-util upload tool](http://dfu-util.sourceforge.net/releases/) (download dfu-utils-0.11-binaries.tar.gz and extract with 7-zip)
- [MinGW build tools](https://genome.sph.umich.edu/wiki/Installing_MinGW_%26_MSYS_on_Windows) (such as the `make` command)

## Resources
- [CubeMX configurator](https://www.st.com/en/development-tools/stm32cubemx.html)

- [STM32405 feather schematic](https://cdn-learn.adafruit.com/assets/assets/000/083/680/original/feather_boards_schem.png?1573012880)
- [STM32 HAL programming guide](https://www.st.com/resource/en/user_manual/dm00105879-description-of-stm32f4-hal-and-ll-drivers-stmicroelectronics.pdf)
- [STM32F405 datasheet](https://www.st.com/resource/en/datasheet/dm00037051.pdf)

- [MPU6050 register documentation](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf)
- [LSM6DSO32 datasheet](https://www.st.com/resource/en/datasheet/lsm6dso32.pdf)


## Installing CMake on MacOS

### Installing homebrew (if you have homebrew installed skip this step)
```
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
```
### Install CMake
```
brew install cmake
```

### Install Arm Compiler
```
brew tap osx-cross/arm
brew install arm-gcc-bin
```

### Install dfu-util

```
brew install dfu-util
```