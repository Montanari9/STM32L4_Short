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

uint8_t SPITxBuf[10], SPIRxBuf[10];
int16_t Gyro_X, Gyro_Y, Gyro_Z;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/

/* USER CODE BEGIN PFP */
HAL_StatusTypeDef GYRO_IO_Read(uint8_t* pBuffer, uint8_t ReadAddr, uint16_t NumByteToRead);
HAL_StatusTypeDef GYRO_IO_Write(uint8_t* pBuffer, uint8_t WriteAddr, uint16_t NumByteToWrite);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* USER CODE BEGIN 2 */

  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_2);
  HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_8);
  HAL_Delay(1000);  
  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_2);
  HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_8);
  HAL_Delay(1000);  
  
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_SET);  // Green LED ON
                                           
  printf("\r\n\n\nL3GD20 Gyro Test..  ");
  HAL_Delay(500);  

  HAL_Status = GYRO_IO_Read(SPIRxBuf, 0x0F, 1);  // 0xD4 ?
  printf("\r\nGyro WHOAMI Value = 0x%x\r\n", SPIRxBuf[0]);
  HAL_Delay(1);  

  SPITxBuf[0] = 0x3F;  // CTRL1: X/Y/Z enable, datarate1, BW4
  HAL_Status = GYRO_IO_Write(SPITxBuf, 0x20, 1);  // address 0x20: CTRL1
  HAL_Delay(10);  

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {      
      printf("\r\n\nPress 'C' to Continue... \r\n\n"); 
      scanf("%c",&RxComByte);
      while((RxComByte != 'c') && (RxComByte != 'C'))
      {
        scanf("%c",&RxComByte);
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_2);
        HAL_Delay(500);  
        HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_8);
        HAL_Delay(500);  
      }
      
      HAL_Status = GYRO_IO_Read(SPIRxBuf, 0x27, 7);  // STATUS reg read 
      if ((SPIRxBuf[0] & 0x08) == 0x08)  // ZYXDA ready bit set?
      {
        Gyro_X = SPIRxBuf[1] + (SPIRxBuf[2]<<8);
        Gyro_Y = SPIRxBuf[3] + (SPIRxBuf[4]<<8);
        Gyro_Z = SPIRxBuf[5] + (SPIRxBuf[6]<<8);
        printf("\r\n\nGyro X Data = %d ", Gyro_X);      
        printf("\r\nGyro Y Data = %d ", Gyro_Y);      
        printf("\r\nGyro Z Data = %d ", Gyro_Z);      
      }        

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}


/* USER CODE BEGIN 4 */

HAL_StatusTypeDef GYRO_IO_Write(uint8_t* pBuffer, uint8_t WriteAddr, uint16_t NumByteToWrite)
{
  HAL_StatusTypeDef Status;
  uint8_t AddrByte = WriteAddr | 0x40;  // set write mode & autoincrement
  
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_RESET);  // Gyro CS low

  Status = HAL_SPI_Transmit(&hspi2, &AddrByte, 1, 1000);  // send address
  if (Status != HAL_OK)
     return Status;

  Status = HAL_SPI_Transmit(&hspi2, pBuffer, NumByteToWrite, 1000);

  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_SET);  // Gyro CS high

  return Status;
}

  
HAL_StatusTypeDef GYRO_IO_Read(uint8_t* pBuffer, uint8_t ReadAddr, uint16_t NumByteToRead)
{  
  HAL_StatusTypeDef Status;
  uint8_t AddrByte = ReadAddr | 0xC0;  // set read mode & autoincrement

  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_RESET);  // Gyro CS low

  Status = HAL_SPI_Transmit(&hspi2, &AddrByte, 1, 1000);  // send address
  if (Status != HAL_OK)
     return Status;

  Status = HAL_SPI_Receive(&hspi2, pBuffer, NumByteToRead, 1000);

  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_SET);  // Gyro CS high

  return Status;
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

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  
}


/* USER CODE END 4 */

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
