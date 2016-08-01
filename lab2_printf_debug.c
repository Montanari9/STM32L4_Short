/**
  ******************************************************************************
  * File Name          : main.c
  * Date               : 17/03/2015 15:18:35
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
char RxComByte = 0;
uint32_t SysTickValue;
volatile uint32_t JoystickPress;

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


  /* USER CODE BEGIN 2 */
  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_2);
  HAL_Delay(300);  
  HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_8);
  HAL_Delay(300);  
  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_2);
  HAL_Delay(300);  
  HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_8);
  HAL_Delay(300);  
  
  printf("\f\r\nHello Printf!!");

  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_SET);  // Green LED ON
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      printf("\r\n\nPress 'C' to Continue..."); 

      while((RxComByte != 'c') && (RxComByte != 'C'))
      {
        HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_8);  // toggle Green LED
        scanf("%c",&RxComByte);
        if (JoystickPress)
        {
          JoystickPress = 0;
          printf("\r\nJoystick Pressed.. ");
          HAL_Delay(200);  
        }
      }

      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);  // Red LED ON
      SysTickValue = HAL_GetTick();
      printf("\r\nSysTick =  %d ", SysTickValue);
      HAL_Delay(1000);  
      RxComByte = 0;
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);  // Red LED OFF

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}


/* USER CODE BEGIN 4 */

int fputc(int ch, FILE *f)  // Retarget printf() to USART2
{
  HAL_Status = HAL_UART_Transmit(&huart2, (uint8_t*) &ch, 1, 200);

  return ch;
}

int fgetc(FILE *f)  // Retarget scanf() to USART2
{
  uint8_t TempByte;

  HAL_Status = HAL_UART_Receive(&huart2, (uint8_t*) &TempByte, 1, 200);

  return TempByte;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  JoystickPress++;  
}

/* USER CODE END 4 */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
