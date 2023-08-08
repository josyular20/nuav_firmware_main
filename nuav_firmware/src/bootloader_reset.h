//
// Created by Rohit on 7/2/22
//

#ifndef BOOTLOADER_RESET
#define BOOTLOADER_RESET

#include "stm32f4xx_hal_rtc.h"

extern RTC_HandleTypeDef hrtc;

// sets the register to 1 and does a software reset since they are the conditions to get into bootloader mode
void PrepareJumpToBootloader(void) {

  // Writes data of 1 in a RTC Backup data Register 1
  HAL_PWR_EnableBkUpAccess();
  HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, 0x0001);
  HAL_PWR_DisableBkUpAccess();

	NVIC_SystemReset();
  Error_Handler();
}

// jumps to bootloader mode provided the conditions are met, else the program exits the function
void HandleJumpToBootloader(void) {

    // get last reset cause 
    reset_cause_t reset_cause = reset_cause_get();

    // exit this function if the last reset cause was not a software reset or it is a software reset but backup register value is not 1
    if (reset_cause != RESET_CAUSE_SOFTWARE_RESET)
    {
      return;
    }
    else if (reset_cause == RESET_CAUSE_SOFTWARE_RESET && HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR1) != 0x0001)
    {
      return;
    }

    void (*SysMemBootJump)(void);
   
    volatile uint32_t addr = 0x1FFF0000;
  

    HAL_RCC_DeInit();
    HAL_DeInit();
    
    /**
     * Step: Disable systick timer and reset it to default values
     */

    SysTick->CTRL = 0;
    SysTick->LOAD = 0;
    SysTick->VAL = 0;
    
    /**
     * Step: Remap system memory to address 0x0000 0000 in address space
     *       For each family registers may be different. 
     *       Check reference manual for each family.
     *
     *       For STM32F4xx, MEMRMP register in SYSCFG is used (bits[1:0])
     *       For STM32F0xx, CFGR1 register in SYSCFG is used (bits[1:0])
     *       For others, check family reference manual
     */
    
    __HAL_SYSCFG_REMAPMEMORY_SYSTEMFLASH();    //Call HAL macro to do this for you
    
    /**
     * Step: Set jump memory location for system memory
     *       Use address with 4 bytes offset which specifies jump location where program starts
     */
    SysMemBootJump = (void (*)(void)) (*((uint32_t *)(addr + 4)));
    
    /**
     * Step: Set main stack pointer.
     *       This step must be done last otherwise local variables in this function
     *       don't have proper value since stack pointer is located on different position
     *
     *       Set direct address location which specifies stack pointer in SRAM location
     */
    __set_MSP(*(uint32_t *)addr);
    
    /**
     * Step: Actually call our function to jump to set location
     *       This will start system memory execution
     */
    SysMemBootJump();
    
}

// resets the register used for the bootloader condition back to 0 once done with uploading code in DFU bootloader mode
void ResetBootloaderRegister(void)
{
    // Writes data of 0 in a RTC Backup data Register 1
    HAL_PWR_EnableBkUpAccess();
    HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, 0x0000);
    HAL_PWR_DisableBkUpAccess(); 
}

#endif // BOOTLOADER_RESET