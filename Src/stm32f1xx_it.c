/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "stm32f1xx.h"
#include "stm32f1xx_it.h"

/* USER CODE BEGIN 0 */
#include "main.h"
#include "stdbool.h"
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim6;
extern UART_HandleTypeDef huart4;

/******************************************************************************/
/*            Cortex-M3 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
* @brief This function handles Non maskable interrupt.
*/
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
* @brief This function handles Hard fault interrupt.
*/
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
  }
  /* USER CODE BEGIN HardFault_IRQn 1 */

  /* USER CODE END HardFault_IRQn 1 */
}

/**
* @brief This function handles Memory management fault.
*/
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
  }
  /* USER CODE BEGIN MemoryManagement_IRQn 1 */

  /* USER CODE END MemoryManagement_IRQn 1 */
}

/**
* @brief This function handles Prefetch fault, memory access fault.
*/
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
  }
  /* USER CODE BEGIN BusFault_IRQn 1 */

  /* USER CODE END BusFault_IRQn 1 */
}

/**
* @brief This function handles Undefined instruction or illegal state.
*/
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
  }
  /* USER CODE BEGIN UsageFault_IRQn 1 */

  /* USER CODE END UsageFault_IRQn 1 */
}

/**
* @brief This function handles System service call via SWI instruction.
*/
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
* @brief This function handles Debug monitor.
*/
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
* @brief This function handles Pendable request for system service.
*/
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
* @brief This function handles UART4 global interrupt.
*/
void UART4_IRQHandler(void)
{
  /* USER CODE BEGIN UART4_IRQn 0 */

  /* USER CODE END UART4_IRQn 0 */
  HAL_UART_IRQHandler(&huart4);
  /* USER CODE BEGIN UART4_IRQn 1 */

  /* USER CODE END UART4_IRQn 1 */
}

/**
* @brief This function handles TIM6 global interrupt and DAC underrun error interrupts.
*/
void TIM6_DAC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_DAC_IRQn 0 */
	
	
		a[INC_RED] = HAL_GPIO_ReadPin(IncA_port_red, IncA_pin_red);
		b[INC_RED] = HAL_GPIO_ReadPin(IncB_port_red, IncB_pin_red);
	
		a[INC_AQUA] = HAL_GPIO_ReadPin(IncA_port_aquamarin, IncA_pin_aquamarin);
		b[INC_AQUA] = HAL_GPIO_ReadPin(IncB_port_aquamarin, IncB_pin_aquamarin);
	
		a[INC_BLACKS] = HAL_GPIO_ReadPin(IncA_port_black, IncA_pin_black);
		b[INC_BLACKS] = HAL_GPIO_ReadPin(IncB_port_black, IncB_pin_black);
	
		a[INC_BLUE] = HAL_GPIO_ReadPin(IncA_port_blue, IncA_pin_blue);
		b[INC_BLUE] = HAL_GPIO_ReadPin(IncB_port_blue, IncB_pin_blue);
	
		a[INC_YELLOW] = HAL_GPIO_ReadPin(IncA_port_yellow, IncA_pin_yellow);
		b[INC_YELLOW] = HAL_GPIO_ReadPin(IncB_port_yellow, IncB_pin_yellow);
	
		a[INC_GREEN] = HAL_GPIO_ReadPin(IncA_port_green, IncA_pin_green);
		b[INC_GREEN] = HAL_GPIO_ReadPin(IncB_port_green, IncB_pin_green);
	
		a[INC_CLARITY] = HAL_GPIO_ReadPin(IncA_port_clarity, IncA_pin_clarity);
		b[INC_CLARITY] = HAL_GPIO_ReadPin(IncB_port_clarity, IncB_pin_clarity);
		
		a[INC_CONTRAST] = HAL_GPIO_ReadPin(IncA_port_contrast, IncA_pin_contrast);
		b[INC_CONTRAST] = HAL_GPIO_ReadPin(IncB_port_contrast, IncB_pin_contrast);
		
		a[INC_CROP] = HAL_GPIO_ReadPin(IncA_port_crop, IncA_pin_crop);
		b[INC_CROP] = HAL_GPIO_ReadPin(IncB_port_crop, IncB_pin_crop);
		
		a[INC_VIBRANCE] = HAL_GPIO_ReadPin(IncA_port_dynamic, IncA_pin_dynamic);
		b[INC_VIBRANCE] = HAL_GPIO_ReadPin(IncB_port_dynamic, IncB_pin_dynamic);
		
		a[INC_EXPOSURE] = HAL_GPIO_ReadPin(IncA_port_exposure, IncA_pin_exposure);
		b[INC_EXPOSURE] = HAL_GPIO_ReadPin(IncB_port_exposure, IncB_pin_exposure);
		
		a[INC_HIGHLIGHTS] = HAL_GPIO_ReadPin(IncA_port_lights, IncA_pin_lights);
		b[INC_HIGHLIGHTS] = HAL_GPIO_ReadPin(IncB_port_lights, IncB_pin_lights);
		
		a[INC_MAGENTA] = HAL_GPIO_ReadPin(IncA_port_magenta, IncA_pin_magenta);
		b[INC_MAGENTA] = HAL_GPIO_ReadPin(IncB_port_magenta, IncB_pin_magenta);
		
		a[INC_ORANGE] = HAL_GPIO_ReadPin(IncA_port_orange, IncA_pin_orange);
		b[INC_ORANGE] = HAL_GPIO_ReadPin(IncB_port_orange, IncB_pin_orange);
		
		a[INC_PROG] = HAL_GPIO_ReadPin(IncA_port_prog, IncA_pin_prog);
		b[INC_PROG] = HAL_GPIO_ReadPin(IncB_port_prog, IncB_pin_prog);
		
		a[INC_SATURATION] = HAL_GPIO_ReadPin(IncA_port_saturation, IncA_pin_saturation);
		b[INC_SATURATION] = HAL_GPIO_ReadPin(IncB_port_saturation, IncB_pin_saturation);
		
		a[INC_SHADOW] = HAL_GPIO_ReadPin(IncA_port_shadow, IncA_pin_shadow);
		b[INC_SHADOW] = HAL_GPIO_ReadPin(IncB_port_shadow, IncB_pin_shadow);
		
		a[INC_PURPLE] = HAL_GPIO_ReadPin(IncA_port_violett, IncA_pin_violett);
		b[INC_PURPLE] = HAL_GPIO_ReadPin(IncB_port_violett, IncB_pin_violett);
		
		a[INC_WHITES] = HAL_GPIO_ReadPin(IncA_port_white, IncA_pin_white);
		b[INC_WHITES] = HAL_GPIO_ReadPin(IncB_port_white, IncB_pin_white);
	
		if(timecount>0){
			timecount--;
		}
  /* USER CODE END TIM6_DAC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim6);
  /* USER CODE BEGIN TIM6_DAC_IRQn 1 */

  /* USER CODE END TIM6_DAC_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
