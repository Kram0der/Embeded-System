/**
 ******************************************************************************
 * File Name          : main.c
 * Description        : Main program body
 ******************************************************************************
 *
 * COPYRIGHT(c) 2024 STMicroelectronics
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
#include "stm32f4xx_hal.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"
#include "stm32f4xx_hal_iwdg.h"
/* USER CODE BEGIN Includes */
#include "zlg7290.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#define ZLG_WRITE_ADDRESS 0x10
#define INF 99999999
#define IWDG_FEED() HAL_IWDG_Refresh(&IWDG_Handler)
// 0~D对应0~D, EF分别为 * 和 #
char input2sign[0x1D] = "\x10\xd\xf\x0\xe\x10\x10\x10\x10\xc\x9\x8\x7\x10\x10\x10\x10\xb\x6\x5\x4\x10\x10\x10\x10\xa\x3\x2\x1";
// 输出对应字节 0123456789
char output_char[0xa] = "\xFC\x0C\xDA\xF2\x66\xB6\xBE\xE0\xFE\xE6";

uint8_t len = 0, calc = 0;
uint32_t input_int[2] = {0};
uint8_t flag2 = 0, flag3 = 0;
uint8_t Rx_Buffer[8] = {0};

uint8_t flag1 = 0;
uint32_t last_input_time = 0, last_free_time = 0;

// uint8_t len __attribute__(());

IWDG_HandleTypeDef IWDG_Handler;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
uint8_t Safe_Read(void);
void Receive_Number(uint8_t input);
void Clear_Display(void);
void Receive_Calculating(uint8_t input);
void Result_Handle(void);

void IWDG_Init(uint8_t prer, uint16_t rlr);
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();
  IWDG_Init(1, 2000);
  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  SystemClock_Config();
  MX_I2C1_Init();
  MX_USART1_UART_Init();

  /* USER CODE BEGIN 2 */
  uint8_t input;
  /* USER CODE END 2 */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
    if (flag1 == 1)
    {
      // for(i=0;i<8;i++){
      // HAL_GPIO_WritePin(GPIOG,GPIO_PIN_6,GPIO_PIN_SET);
      // HAL_Delay(4);
      // HAL_GPIO_WritePin(GPIOG,GPIO_PIN_6,GPIO_PIN_RESET);
      // HAL_Delay(4);
      //}
      IWDG_FEED();
      flag1 = 0;
      input = Safe_Read();
      last_input_time = HAL_GetTick();

      if (input > 0x1C || input == 0x0) // 输入不合法
        continue;
      input = input2sign[input]; // 转换为符号

      if (input == 0x10)
        continue;
      // printf("%#x\t", input);

      if (input == 0xf || flag3) // 接受到清除键或者运算完成
        Clear_Display();
      else if (input <= 0x9) // 接受到数字
        Receive_Number(input);
      else if (input < 0xe) // 接受到运算符
        Receive_Calculating(input);
      else
        Result_Handle();
    }
    else
    {
      last_free_time = HAL_GetTick();
      uint32_t time_diff = last_free_time - last_input_time;

      if (time_diff == 10000)
        Clear_Display();
      if (time_diff % 300 == 0) // 300ms刷新一次数码管并喂狗
      {
        IWDG_FEED();
        I2C_ZLG7290_Write(&hi2c1, 0x70, ZLG_WRITE_ADDRESS, Rx_Buffer, 8);
      }
      HAL_PWR_EnterSLEEPMode(PWR_LOWPOWERREGULATOR_ON, PWR_SLEEPENTRY_WFI);
    }
  }
}
/* USER CODE END 3 */

/** System Clock Configuration
 */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  __PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */
// 安全读取
uint8_t Safe_Read()
{
  uint8_t tmp1 = 0, tmp2 = 0, tmp3 = 0;
  I2C_ZLG7290_Read(&hi2c1, 0x71, 0x01, &tmp1, 1);
  I2C_ZLG7290_Read(&hi2c1, 0x71, 0x01, &tmp2, 1);
  I2C_ZLG7290_Read(&hi2c1, 0x71, 0x01, &tmp3, 1);
  if (tmp1 == tmp2 && tmp2 == tmp3)
  {
    return tmp1;
  }
	return 0;
}
// 状态还原
void Clear_Display()
{
  input_int[0] = input_int[1] = 0;
  flag2 = flag3 = 0;
  len = 0;
  memset(Rx_Buffer, 0, 8);
  I2C_ZLG7290_Write(&hi2c1, 0x70, ZLG_WRITE_ADDRESS, Rx_Buffer, 8); // 清除显示
}
// 接收到数字
void Receive_Number(uint8_t input)
{
  if (len == 8) // 输入已满
    return;
  input_int[flag2] = (input_int[flag2] << 3) + (input_int[flag2] << 1) + input; // 保存数字
  input = output_char[input];
  Rx_Buffer[len] = input;
  len += 1;
  I2C_ZLG7290_Write(&hi2c1, 0x70, ZLG_WRITE_ADDRESS + len, Rx_Buffer, len);
}
// 接收到运算符
void Receive_Calculating(uint8_t input)
{
  if (flag2 == 0) // 接受到运算符且参数未满
  {
    calc = input; // 保存运算符
    memset(Rx_Buffer, 0, 8);
    I2C_ZLG7290_Write(&hi2c1, 0x70, ZLG_WRITE_ADDRESS, Rx_Buffer, 8);
    len = 0;
    flag2 = 1;
  }
}
// 结果处理
void Result_Handle()
{
  if (flag2 != 1)
    return; // 参数未满
  flag3 = 1;
  uint32_t result;
  switch (calc)
  {
  case 0xa:
    result = input_int[0] + input_int[1]; // 加法
    flag3 += input_int[0] <= INF;         // 结果是否超出范围
    break;
  case 0xb:
    flag3 = input_int[0] >= input_int[1];
    result += input_int[0] - input_int[1]; // 减法
    break;
  case 0xc:
    flag3 = INF / input_int[1] > input_int[0];
    result += input_int[0] * input_int[1]; // 乘法
    break;
  case 0xd:
    flag3 = input_int[1] != 0;
    result += input_int[0] / input_int[1]; // 除法
    break;
  }
  // printf("%d\t", flag3);
  if (flag3 == 2) // 结果合法
  {
    uint8_t i;
    len = 0;
    // 结果转化为字节
    for (i = 0; result != 0; i++)
    {
      Rx_Buffer[i] = output_char[result % 10];
      result /= 10;
      len++;
    }
    // 结果反转
    for (i = 0; i < len - i - 1; i++)
    {
      Rx_Buffer[i] ^= Rx_Buffer[len - i - 1];
      Rx_Buffer[len - i - 1] ^= Rx_Buffer[i];
      Rx_Buffer[i] ^= Rx_Buffer[len - i - 1];
    }
    I2C_ZLG7290_Write(&hi2c1, 0x70, ZLG_WRITE_ADDRESS, Rx_Buffer, 8);
  }
  else // 结果不合法
  {
    Clear_Display();
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  flag1 = 1;
}
int fputc(int ch, FILE *f)
{
  uint8_t tmp[1] = {0};
  tmp[0] = (uint8_t)ch;
  HAL_UART_Transmit(&huart1, tmp, 1, 10);
  return ch;
}

void IWDG_Init(uint8_t prer, uint16_t rlr)
{
  IWDG_Handler.Instance = IWDG;
  IWDG_Handler.Init.Prescaler = prer;
  IWDG_Handler.Init.Reload = rlr;
  HAL_IWDG_Init(&IWDG_Handler);
  HAL_IWDG_Start(&IWDG_Handler);
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}

#endif

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
