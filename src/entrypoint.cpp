#include "main.h"
#include <stdio.h>
#include "usbd_cdc_if.h"
#include "reset_cause.h"
#include "bootloader_reset.h"

// buffer, which is an array of characters, to store user input
const int maxBufferSize = 64;
char usb_cdc_buffer[maxBufferSize];

int main(){

  SetupHAL();

  HandleJumpToBootloader();

  ResetBootloaderRegister();

  HAL_Delay(2000); 

  while (1){

    
    if (strcmp(usb_cdc_buffer, "bootloader\n") == 0) {   // type in "bootloader" to jump to bootloader mode
      printf("Preparing to jump to bootloader mode\n");
      printf("Be sure to press the reset button on the STM32F4 when you wish to exit bootloader mode!\n");
      printf("Now doing a system reset, then you will be in bootloader mode\n");
      HAL_Delay(1000);
      PrepareJumpToBootloader();
    }
    else if (strcmp(usb_cdc_buffer, "reset\n") == 0) {     // type in "reset" to do a software reset on the microcontroller
      printf("Doing a microcontroller reset now\n");
      HAL_Delay(1000);
      NVIC_SystemReset();
    }
   
    HAL_GPIO_TogglePin(RED_LED_GPIO_Port, RED_LED_Pin);
    printf("normal mode\n");
    HAL_Delay(500);
  }
}
