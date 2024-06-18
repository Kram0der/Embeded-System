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
#define UINT8_CHECK 0xFF
#define UINT32_CHECK 0xFFFFFFFF
#define ZLG_WRITE_ADDRESS 0x10
#define INF 99999999
#define RESET() HAL_NVIC_SystemReset()
#define Random_Delay(x) HAL_Delay(rand() % x)
#define IWDG_FEED() HAL_IWDG_Refresh(&IWDG_Handler)
#define Calc_update() variable_backup_update(&calc_bkp1, &calc_bkp2, &calc_bkp3, calc)
#define Len_update() variable_backup_update(&len_bkp1, &len_bkp2, &len_bkp3, len)
#define Flag2_update() variable_backup_update(&flag2_bkp1, &flag2_bkp2, &flag2_bkp3, flag2)
// 0~D对应0~D, EF分别为 * 和 #
uint8_t input2sign[0x1D] = {0x10, 0xd, 0xf, 0x0, 0xe, 0x10, 0x10, 0x10, 0x10, 0xc, 0x9, 0x8, 0x7, 0x10, 0x10, 0x10, 0x10, 0xb, 0x6, 0x5, 0x4, 0x10, 0x10, 0x10, 0x10, 0xa, 0x3, 0x2, 0x1};
// 输出对应字节 0123456789
uint8_t output_char[0xa] = {0xFC, 0x0C, 0xDA, 0xF2, 0x66, 0xB6, 0xBE, 0xE0, 0xFE, 0xE6};

uint32_t input_int[2] = {0};
uint8_t len = 0, calc = 0;
uint8_t flag2 = 0;
uint8_t Rx_Buffer[8] = {0};

struct backup_32
{
	uint32_t data[2];
	uint32_t checksum;
};

struct backup_8
{
	uint8_t data[1];
	uint8_t checksum;
};

struct backup_8_arr
{
	uint8_t data[8];
	uint8_t checksum;
};

uint8_t input2sign_bkp1[0x1D] __attribute__((at(0x10000000)));
uint8_t input2sign_bkp2[0x1D] __attribute__((at(0x10001000)));
uint8_t input2sign_bkp3[0x1D] __attribute__((at(0x10002000)));

uint8_t input2sign_bkp1_checksum __attribute__((at(0x10000030)));
uint8_t input2sign_bkp2_checksum __attribute__((at(0x10001030)));
uint8_t input2sign_bkp3_checksum __attribute__((at(0x10002030)));

uint8_t output_char_bkp1[0xa] __attribute__((at(0x10000100)));
uint8_t output_char_bkp2[0xa] __attribute__((at(0x10001100)));
uint8_t output_char_bkp3[0xa] __attribute__((at(0x10002100)));

uint8_t output_char_bkp1_checksum __attribute__((at(0x10000130)));
uint8_t output_char_bkp2_checksum __attribute__((at(0x10001130)));
uint8_t output_char_bkp3_checksum __attribute__((at(0x10002130)));

struct backup_32 input_int_bkp1 __attribute__((at(0x10000200)));
struct backup_32 input_int_bkp2 __attribute__((at(0x10001200)));
struct backup_32 input_int_bkp3 __attribute__((at(0x10002200)));

struct backup_8 len_bkp1 __attribute__((at(0x10000300)));
struct backup_8 len_bkp2 __attribute__((at(0x10001300)));
struct backup_8 len_bkp3 __attribute__((at(0x10002300)));

struct backup_8 calc_bkp1 __attribute__((at(0x10000400)));
struct backup_8 calc_bkp2 __attribute__((at(0x10001400)));
struct backup_8 calc_bkp3 __attribute__((at(0x10002400)));

struct backup_8 flag2_bkp1 __attribute__((at(0x10000500)));
struct backup_8 flag2_bkp2 __attribute__((at(0x10001500)));
struct backup_8 flag2_bkp3 __attribute__((at(0x10002500)));

struct backup_8_arr Rx_Buffer_bkp1 __attribute__((at(0x10000700)));
struct backup_8_arr Rx_Buffer_bkp2 __attribute__((at(0x10001700)));
struct backup_8_arr Rx_Buffer_bkp3 __attribute__((at(0x10002700)));

uint32_t sign __attribute__((at(0x10003000)));

uint8_t flag1 = 0, order = 0;
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
uint8_t check_uint8_array(uint8_t *arr, uint8_t len, uint8_t checksum, uint8_t type);
uint32_t check_uint32_array(uint32_t *arr, uint32_t checksum, uint8_t type);
void static_backup_init();
void backup_init();
void uint8_array_copy(uint8_t *dest, uint8_t *src, uint8_t len);
void uint32_array_copy(uint32_t *dest, uint32_t *src, uint8_t len);
void variable_backup_update(struct backup_8 *bkp1, struct backup_8 *bkp2, struct backup_8 *bkp3, uint8_t data);
void Rx_update();
void input_int_update();
void generate_random_uint8_array(uint8_t *arr, uint8_t len);
void uint8_array_backup_check(uint8_t flag);
void uint8_backup_check(struct backup_8 *bkp1, struct backup_8 *bkp2, struct backup_8 *bkp3, uint8_t flag);
void input_int_backup_update();
void backup_check();
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
	IWDG_Init(1, 3000);
	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	SystemClock_Config();
	MX_I2C1_Init();
	MX_USART1_UART_Init();

	/* USER CODE BEGIN 2 */
	uint8_t input;
	if (sign == 302141191) // 热启动
	{
		backup_check();
		sign = 302141191;
	}
	else // 冷启动
	{
		static_backup_init();
		backup_init();
		sign = 302141191;
	}
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
			if (last_input_time - last_free_time > 10000) // 连续输入10s以上，重启
			{
				sign = 0;
				RESET();
			}
			Random_Delay(10);

			if (input > 0x1C || input == 0x0) // 输入不合法
				continue;
			input = input2sign[input]; // 转换为符号
			Random_Delay(10);
			if (input == 0x10)
				continue;
			// printf("%#x\t", input);

			if (input == 0xf || flag2 >= 2) // 接受到清除键或者运算完成
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
			if (time_diff % 400 == 0) // 400ms刷新一次数码管并喂狗
			{
				IWDG_FEED();
				uint8_array_backup_check(2);
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
	if (!order)
		RESET();
	uint8_t tmp1 = 0, tmp2 = 0, tmp3 = 0;
	I2C_ZLG7290_Read(&hi2c1, 0x71, 0x01, &tmp1, 1);
	I2C_ZLG7290_Read(&hi2c1, 0x71, 0x01, &tmp2, 1);
	I2C_ZLG7290_Read(&hi2c1, 0x71, 0x01, &tmp3, 1);
	if (tmp1 == tmp2 && tmp2 == tmp3)
	{
		order = 1;
		return tmp1;
	}
	return 0;
}
// 状态还原
void Clear_Display()
{
	input_int[0] = input_int[1] = 0;
	flag2 = 0;
	len = 0;
	order = 0;
	memset(Rx_Buffer, 0, 8);
	backup_init();
	I2C_ZLG7290_Write(&hi2c1, 0x70, ZLG_WRITE_ADDRESS, Rx_Buffer, 8); // 清除显示
}
// 接收到数字
void Receive_Number(uint8_t input)
{
	if (order != 1)
		RESET();
	if (len < 8 && flag2 < 2) // 输入未满
	{
		input_int[flag2] = (input_int[flag2] << 3) + (input_int[flag2] << 1) + input; // 保存数字
		input = output_char[input];
		Rx_Buffer[len] = input;
		len += 1;
		Random_Delay(5);
		I2C_ZLG7290_Write(&hi2c1, 0x70, ZLG_WRITE_ADDRESS + len, Rx_Buffer, len);
		input_int_update();
		Rx_update();
		Len_update();
	}
	order = 0;
}
// 接收到运算符
void Receive_Calculating(uint8_t input)
{
	if (order != 1)
		RESET();
	if (flag2 == 0) // 接受到运算符且参数未满
	{
		calc = input; // 保存运算符
		Calc_update();
		memset(Rx_Buffer, 0, 8);
		Rx_update();
		I2C_ZLG7290_Write(&hi2c1, 0x70, ZLG_WRITE_ADDRESS, Rx_Buffer, 8);
		Random_Delay(5);
		len = 0;
		flag2 = 1;
		Len_update();
		Flag2_update();
	}
	order = 0;
}
// 结果处理
void Result_Handle()
{
	if (order != 1)
		RESET();
	if (flag2 == 1) // 运算符和参数已满
	{
		flag2 = 2;
		Flag2_update();
		uint32_t result;
		switch (calc)
		{
		case 0xa:
			result = input_int[0] + input_int[1]; // 加法
			flag2 += input_int[0] <= INF;		  // 结果是否超出范围
			break;
		case 0xb:
			flag2 += input_int[0] >= input_int[1];
			result = input_int[0] - input_int[1]; // 减法
			break;
		case 0xc:
			flag2 += INF / input_int[1] > input_int[0];
			result = input_int[0] * input_int[1]; // 乘法
			break;
		case 0xd:
			flag2 += input_int[1] != 0;
			result = input_int[0] / input_int[1]; // 除法
			break;
		}
		Flag2_update();
		Random_Delay(5);
		if (flag2 == 3) // 结果合法
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
			Rx_update();
			I2C_ZLG7290_Write(&hi2c1, 0x70, ZLG_WRITE_ADDRESS, Rx_Buffer, 8);
		}
		else // 结果不合法
		{
			Clear_Display();
		}
	}
	order = 0;
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
// 初始化看门狗
void IWDG_Init(uint8_t prer, uint16_t rlr)
{
	IWDG_Handler.Instance = IWDG;
	IWDG_Handler.Init.Prescaler = prer;
	IWDG_Handler.Init.Reload = rlr;
	HAL_IWDG_Init(&IWDG_Handler);
	HAL_IWDG_Start(&IWDG_Handler);
}
// 静态备份初始化
void static_backup_init()
{
	uint8_t i = rand() & 1;
	if (i == 0)
	{
		uint8_array_copy(input2sign_bkp1, input2sign, 0x1D);
		input2sign_bkp1_checksum = check_uint8_array(input2sign_bkp1, 0x1D, 0, 0);
		uint8_array_copy(input2sign_bkp2, input2sign, 0x1D);
		input2sign_bkp2_checksum = check_uint8_array(input2sign_bkp2, 0x1D, 0, 0);
		uint8_array_copy(input2sign_bkp3, input2sign, 0x1D);
		input2sign_bkp3_checksum = check_uint8_array(input2sign_bkp3, 0x1D, 0, 0);
	}
	else
	{
		uint8_array_copy(output_char_bkp1, output_char, 0xa);
		output_char_bkp1_checksum = check_uint8_array(output_char_bkp1, 0xa, 0, 0);
		uint8_array_copy(output_char_bkp2, output_char, 0xa);
		output_char_bkp2_checksum = check_uint8_array(output_char_bkp2, 0xa, 0, 0);
		uint8_array_copy(output_char_bkp3, output_char, 0xa);
		output_char_bkp3_checksum = check_uint8_array(output_char_bkp3, 0xa, 0, 0);
	}
}
// uint8_t备份更新
void variable_backup_update(struct backup_8 *bkp1, struct backup_8 *bkp2, struct backup_8 *bkp3, uint8_t data)
{
	uint8_t i, order[3];
	generate_random_uint8_array(order, 3);
	for (i = 0; i < 3; i++)
	{
		switch (order[i])
		{
		case 0:
			bkp1->data[0] = data;
			bkp1->checksum = check_uint8_array(bkp1->data, 1, 0, 0);
			break;
		case 1:
			bkp2->data[0] = data;
			bkp2->checksum = check_uint8_array(bkp2->data, 1, 0, 0);
			break;
		case 2:
			bkp3->data[0] = data;
			bkp3->checksum = check_uint8_array(bkp3->data, 1, 0, 0);
			break;
		}
	}
}
// Rx_Buffer更新
void Rx_update()
{
	uint8_array_copy(Rx_Buffer_bkp1.data, Rx_Buffer, 8);
	Rx_Buffer_bkp1.checksum = check_uint8_array(Rx_Buffer_bkp1.data, 8, 0, 0);
	uint8_array_copy(Rx_Buffer_bkp2.data, Rx_Buffer, 8);
	Rx_Buffer_bkp2.checksum = check_uint8_array(Rx_Buffer_bkp2.data, 8, 0, 0);
	uint8_array_copy(Rx_Buffer_bkp3.data, Rx_Buffer, 8);
	Rx_Buffer_bkp3.checksum = check_uint8_array(Rx_Buffer_bkp3.data, 8, 0, 0);
}
// input_int更新
void input_int_update()
{
	uint32_array_copy(input_int_bkp1.data, input_int, 2);
	input_int_bkp1.checksum = check_uint32_array(input_int_bkp1.data, 0, 0);
	uint32_array_copy(input_int_bkp2.data, input_int, 2);
	input_int_bkp2.checksum = check_uint32_array(input_int_bkp2.data, 0, 0);
	uint32_array_copy(input_int_bkp3.data, input_int, 2);
	input_int_bkp3.checksum = check_uint32_array(input_int_bkp3.data, 0, 0);
}
// 备份初始化
void backup_init()
{
	uint8_t i, order[5];
	generate_random_uint8_array(order, 5);
	for (i = 0; i < 5; i++)
	{
		switch (order[i])
		{
		case 0:
			Len_update();
			break;
		case 1:
			Calc_update();
			break;
		case 2:
			Flag2_update();
			break;
		case 3:
			Rx_update();
			break;
		case 4:
			input_int_update();
			break;
		}
	}
}
// 字节数组备份校验 0:input2sign  1:output_char  2:Rx_BUffer
void uint8_array_backup_check(uint8_t flag)
{
	uint8_t *bkp1, *bkp2, *bkp3, *right;
	uint8_t *checksum1, *checksum2, *checksum3, len;
	if (!flag)
	{
		bkp1 = input2sign_bkp1;
		bkp2 = input2sign_bkp2;
		bkp3 = input2sign_bkp3;
		len = 0x1D;
		checksum1 = &input2sign_bkp1_checksum;
		checksum2 = &input2sign_bkp2_checksum;
		checksum3 = &input2sign_bkp3_checksum;
	}
	else if (flag == 1)
	{
		bkp1 = output_char_bkp1;
		bkp2 = output_char_bkp2;
		bkp3 = output_char_bkp3;
		len = 0xa;
		checksum1 = &output_char_bkp1_checksum;
		checksum2 = &output_char_bkp2_checksum;
		checksum3 = &output_char_bkp3_checksum;
	}
	else
	{
		bkp1 = Rx_Buffer_bkp1.data;
		bkp2 = Rx_Buffer_bkp2.data;
		bkp3 = Rx_Buffer_bkp3.data;
		len = 8;
		checksum1 = &Rx_Buffer_bkp1.checksum;
		checksum2 = &Rx_Buffer_bkp2.checksum;
		checksum3 = &Rx_Buffer_bkp3.checksum;
	}
	uint8_t f1, f2, f3;
	f1 = check_uint8_array(bkp1, len, *checksum1, 1);
	f2 = check_uint8_array(bkp2, len, *checksum2, 1);
	f3 = check_uint8_array(bkp3, len, *checksum3, 1);
	if (!f1 && !f2 && !f3)
	{
		sign = 0;
		HAL_NVIC_SystemReset();
	}
	if (f1)
		right = bkp1;
	else if (f2)
	{
		right = bkp2;
		bkp2 = bkp1;
		checksum2 = checksum1;
	}
	else
	{
		right = bkp3;
		bkp3 = bkp1;
		checksum3 = checksum1;
	}
	uint8_array_copy(bkp2, right, len);
	*checksum2 = check_uint8_array(bkp2, len, 0, 0);
	uint8_array_copy(bkp3, right, len);
	*checksum3 = check_uint8_array(bkp3, len, 0, 0);
	switch (flag)
	{
	case 0:
		uint8_array_copy(input2sign, right, len);
		break;
	case 1:
		uint8_array_copy(output_char, right, len);
		break;
	case 2:
		uint8_array_copy(Rx_Buffer, right, len);
		break;
	}
}
// 字节备份检验	0:len  1:calc  2:flag2
void uint8_backup_check(struct backup_8 *bkp1, struct backup_8 *bkp2, struct backup_8 *bkp3, uint8_t flag)
{
	struct backup_8 *right;
	uint8_t f1, f2, f3;
	f1 = check_uint8_array(bkp1->data, 1, bkp1->checksum, 1);
	f2 = check_uint8_array(bkp2->data, 1, bkp2->checksum, 1);
	f3 = check_uint8_array(bkp3->data, 1, bkp3->checksum, 1);
	if (!f1 && !f2 && !f3)
	{
		sign = 0;
		HAL_NVIC_SystemReset();
	}
	if (f1)
		right = bkp1;
	else if (f2)
	{
		right = bkp2;
		bkp2 = bkp1;
	}
	else
	{
		right = bkp3;
		bkp3 = bkp1;
	}
	bkp2->data[0] = right->data[0];
	bkp2->checksum = check_uint8_array(bkp2->data, 1, 0, 0);
	bkp3->data[0] = right->data[0];
	bkp3->checksum = check_uint8_array(bkp3->data, 1, 0, 0);
	switch (flag)
	{
	case 0:
		len = right->data[0];
		break;
	case 1:
		calc = right->data[0];
		break;
	case 2:
		flag2 = right->data[0];
		break;
	}
}
// input_int备份更新
void input_int_backup_update()
{
	uint8_t f1, f2, f3;
	f1 = check_uint32_array(input_int_bkp1.data, input_int_bkp1.checksum, 1);
	f2 = check_uint32_array(input_int_bkp2.data, input_int_bkp2.checksum, 1);
	f3 = check_uint32_array(input_int_bkp3.data, input_int_bkp3.checksum, 1);
	if (!f1 && !f2 && !f3)
	{
		sign = 0;
		HAL_NVIC_SystemReset();
	}
	if (f1)
	{
		uint32_array_copy(input_int_bkp2.data, input_int_bkp1.data, 2);
		input2sign_bkp2_checksum = check_uint32_array(input_int_bkp2.data, 0, 0);
		uint32_array_copy(input_int_bkp3.data, input_int_bkp1.data, 2);
		input2sign_bkp3_checksum = check_uint32_array(input_int_bkp3.data, 0, 0);
	}
	else if (f2)
	{
		uint32_array_copy(input_int_bkp1.data, input_int_bkp2.data, 2);
		input2sign_bkp1_checksum = check_uint32_array(input_int_bkp1.data, 0, 0);
		uint32_array_copy(input_int_bkp3.data, input_int_bkp2.data, 2);
		input2sign_bkp3_checksum = check_uint32_array(input_int_bkp3.data, 0, 0);
	}
	else
	{
		uint32_array_copy(input_int_bkp1.data, input_int_bkp3.data, 2);
		input2sign_bkp1_checksum = check_uint32_array(input_int_bkp1.data, 0, 0);
		uint32_array_copy(input_int_bkp2.data, input_int_bkp3.data, 2);
		input2sign_bkp2_checksum = check_uint32_array(input_int_bkp2.data, 0, 0);
	}
	uint32_array_copy(input_int, input_int_bkp1.data, 2);
}
// 备份检验
void backup_check()
{
	uint8_t order[7], i;
	generate_random_uint8_array(order, 7);
	for (i = 0; i < 7; i++)
	{
		switch (order[i])
		{
		case 0:
			uint8_array_backup_check(0);
			break;
		case 1:
			uint8_array_backup_check(1);
			break;
		case 2:
			uint8_array_backup_check(2);
			break;
		case 3:
			uint8_backup_check(&len_bkp1, &len_bkp2, &len_bkp3, 0);
			break;
		case 4:
			uint8_backup_check(&calc_bkp1, &calc_bkp2, &calc_bkp3, 1);
			break;
		case 5:
			uint8_backup_check(&flag2_bkp1, &flag2_bkp2, &flag2_bkp3, 2);
			break;
		case 6:
			input_int_backup_update();
			break;
		}
	}
}
// 校验字节数组		0:计算校验和 1:检验校验和
uint8_t check_uint8_array(uint8_t *arr, uint8_t len, uint8_t checksum, uint8_t type)
{
	uint8_t i;
	uint8_t sum = UINT8_CHECK;
	for (i = 0; i < len; i++)
	{
		sum ^= arr[i];
	}
	if (!type)
		return sum;
	return sum == checksum;
}
// 校验uint32数组	0:计算校验和 1:检验校验和	默认长度为2
uint32_t check_uint32_array(uint32_t *arr, uint32_t checksum, uint8_t type)
{
	uint8_t i;
	uint32_t sum = UINT32_CHECK;
	for (i = 0; i < 2; i++)
	{
		sum ^= arr[i];
	}
	if (!type)
		return sum;
	return sum == checksum;
}
// 字节数组拷贝
void uint8_array_copy(uint8_t *dest, uint8_t *src, uint8_t len)
{
	while (len--)
		dest[len] = src[len];
}
// uint32数组拷贝
void uint32_array_copy(uint32_t *dest, uint32_t *src, uint8_t len)
{
	while (len--)
		dest[len] = src[len];
}
// 生成随机序列
void generate_random_uint8_array(uint8_t *arr, uint8_t len)
{
	uint8_t i, j;
	for (i = 0; i < len; i++)
		arr[i] = i;
	for (i = 0; i < len; i++)
	{
		j = rand() % len;
		arr[i] ^= arr[j];
		arr[j] ^= arr[i];
		arr[i] ^= arr[j];
	}
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
