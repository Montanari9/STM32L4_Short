/**
  ******************************************************************************
  * File Name          : main.c
  * Date               : 14/04/2015 11:20:30
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2015 STMicroelectronics
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
#include "stm32l4xx_hal.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
HAL_StatusTypeDef HAL_Status;
uint32_t CallbackIRQCount;
uint32_t ADC_Result[10], ADC_Avg;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* USER CODE BEGIN 2 */
  HAL_Status = HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
  HAL_Status = HAL_ADC_Start_DMA(&hadc1, ADC_Result, 10);
  HAL_Status = HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
      HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_2);
      HAL_Delay(250);  
      HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_8);
      HAL_Delay(250);  

      printf("\r\n\nADC Average is: 0x%x\r\n", ADC_Avg); 

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration */


/* USER CODE BEGIN 4 */

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  CallbackIRQCount++;
  ADC_Avg = 0;
  
  for (uint32_t x=0; x!=10; x++)
  { 
      ADC_Avg += ADC_Result[x];
  }
  ADC_Avg = ADC_Avg / 10;
}


int fputc(int ch, FILE *f)  // Retarget printf() to USART2
{
  HAL_Status = HAL_UART_Transmit(&huart2, (uint8_t*) &ch, 1, 2000);

  return ch;
}

int fgetc(FILE *f)  // Retarget scanf() to USART2
{
  uint8_t TempByte;

  HAL_Status = HAL_UART_Receive(&huart2, (uint8_t*) &TempByte, 1, 2000);

  return TempByte;
}

/* USER CODE END 4 */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
