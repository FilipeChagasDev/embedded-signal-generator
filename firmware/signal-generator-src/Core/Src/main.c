/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include "nokia5110_LCD.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2S_HandleTypeDef hi2s2;
DMA_HandleTypeDef hdma_spi2_tx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2S2_Init(void);
/* USER CODE BEGIN PFP */

void select_freq();
void select_amp();
void select_wave();
void select_harm();
double generate_wave_cycle();

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define PI 3.14159265358979323846
#define TAU (2.0 * PI)
#define AMPLITUDE_FACTOR 28500
#define DAC_FREQ 44000

#define TXBUFF_LEN 1024
uint16_t txbuff[TXBUFF_LEN];

int frequency = 1000;
int amplitude = 100;
int harmonics = 50;

#define WAVEFORM_SIN 0
#define WAVEFORM_SQUARE 1
#define WAVEFORM_TRIANGLE 2
#define WAVEFORM_SAW 3

int waveform = WAVEFORM_SIN;

typedef struct {
	void (*init)(void);
	void (*cw_rotation)(void);
	void (*ccw_rotation)(void);
	void (*push)(void);
	int selected;
} ui_state_t;

#define UI_SELECTED_FREQ 0
#define UI_SELECTED_AMP 1
#define UI_SELECTED_WAVE 2
#define UI_SELECTED_HARM 3

ui_state_t ui_global_state;

void update_ui()
{
	char freq_txt[20];
	char amp_txt[20];
	char wave_txt[20];
	char harm_txt[20];

	LCD_clrScr();

	//freq text
	if(frequency < 1000)
	{
		sprintf(freq_txt, " Freq: %dHz", frequency);
	}
	else
	{
		sprintf(freq_txt, " Freq: %dkHz", frequency/1000);
	}

	LCD_print(freq_txt, 0, 0);

	//Amp text
	sprintf(amp_txt, " Amp: %d%%", amplitude);
	LCD_print(amp_txt, 0, 1);

	//Wave text
	switch(waveform)
	{
	case WAVEFORM_SIN:
		sprintf(wave_txt, " Wave: SIN");
		break;
	case WAVEFORM_SQUARE:
		sprintf(wave_txt, " Wave: SQR");
		break;
	case WAVEFORM_TRIANGLE:
		sprintf(wave_txt, " Wave: TRI");
		break;
	case WAVEFORM_SAW:
		sprintf(wave_txt, " Wave: SAW");
		break;
	}

	LCD_print(wave_txt, 0, 2);

	//Harm text
	sprintf(harm_txt, " Harm: %d", harmonics);
	LCD_print(harm_txt, 0, 3);

	//Selection
	LCD_print("*", 0, ui_global_state.selected);
}


void freq_selected_init()
{
	update_ui();
}

void freq_selected_cw()
{
	if(frequency < 20000)
	{
		if(frequency < 1000)
		{
			frequency = (frequency/10)*10;
			frequency += 10;
		}
		else if(frequency >= 1000)
		{
			frequency = (frequency/1000)*1000;
			frequency += 1000;
		}
	}

	update_ui();
	generate_wave_cycle();
}

void freq_selected_ccw()
{
	if(frequency > 20)
	{
		if(frequency <= 1000)
		{
			frequency = (frequency/10)*10;
			frequency -= 10;
		}
		else if(frequency > 1000)
		{
			frequency = (frequency/1000)*1000;
			frequency -= 1000;
		}
	}

	update_ui();
	generate_wave_cycle();
}

void freq_selected_push()
{
	select_amp();
	ui_global_state.init();
}

void select_freq()
{
	ui_global_state.init = freq_selected_init;
	ui_global_state.cw_rotation = freq_selected_cw;
	ui_global_state.ccw_rotation = freq_selected_ccw;
	ui_global_state.push = freq_selected_push;
	ui_global_state.selected = UI_SELECTED_FREQ;
}

void amp_selected_init()
{
	update_ui();
}

void amp_selected_cw()
{
	if(amplitude < 100) amplitude += 1;
	update_ui();
	generate_wave_cycle();
}

void amp_selected_ccw()
{
	if(amplitude > 0) amplitude -= 1;
	update_ui();
	generate_wave_cycle();
}

void amp_selected_push()
{
	select_wave();
	ui_global_state.init();
}

void select_amp()
{
	ui_global_state.init = amp_selected_init;
	ui_global_state.cw_rotation = amp_selected_cw;
	ui_global_state.ccw_rotation = amp_selected_ccw;
	ui_global_state.push = amp_selected_push;
	ui_global_state.selected = UI_SELECTED_AMP;
}

void wave_selected_init()
{
	update_ui();
}

void wave_selected_cw()
{
	switch(waveform)
	{
	case WAVEFORM_SIN:
		waveform = WAVEFORM_SQUARE;
		break;
	case WAVEFORM_SQUARE:
		waveform = WAVEFORM_TRIANGLE;
		break;
	case WAVEFORM_TRIANGLE:
		waveform = WAVEFORM_SAW;
		break;
	case WAVEFORM_SAW:
		waveform = WAVEFORM_SIN;
		break;
	}
	update_ui();
	generate_wave_cycle();
}

void wave_selected_ccw()
{
	switch(waveform)
	{
		case WAVEFORM_SIN:
			waveform = WAVEFORM_SAW;
			break;
		case WAVEFORM_SQUARE:
			waveform = WAVEFORM_SIN;
			break;
		case WAVEFORM_TRIANGLE:
			waveform = WAVEFORM_SQUARE;
			break;
		case WAVEFORM_SAW:
			waveform = WAVEFORM_TRIANGLE;
			break;
	}
	update_ui();
	generate_wave_cycle();
}

void wave_selected_push()
{
	select_harm();
	ui_global_state.init();
}

void select_wave()
{
	ui_global_state.init = wave_selected_init;
	ui_global_state.cw_rotation = wave_selected_cw;
	ui_global_state.ccw_rotation = wave_selected_ccw;
	ui_global_state.push = wave_selected_push;
	ui_global_state.selected = UI_SELECTED_WAVE;
}

void harm_selected_init()
{
	update_ui();
}

void harm_selected_cw()
{
	if(harmonics < 100) harmonics += 1;
	update_ui();
	generate_wave_cycle();
}

void harm_selected_ccw()
{
	if(harmonics > 1) harmonics -= 1;
	update_ui();
	generate_wave_cycle();
}

void harm_selected_push()
{
	select_freq();
	ui_global_state.init();
}

void select_harm()
{
	ui_global_state.init = harm_selected_init;
	ui_global_state.cw_rotation = harm_selected_cw;
	ui_global_state.ccw_rotation = harm_selected_ccw;
	ui_global_state.push = harm_selected_push;
	ui_global_state.selected = UI_SELECTED_HARM;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2S2_Init();
  /* USER CODE BEGIN 2 */

  //CONFIGURE LCD
  HAL_GPIO_WritePin(LCD_VCC_GPIO_Port, LCD_VCC_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LCD_BL_GPIO_Port, LCD_BL_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LCD_GND_GPIO_Port, LCD_GND_Pin, GPIO_PIN_RESET);

  LCD_setCE(LCD_CE_GPIO_Port, LCD_CE_Pin);
  LCD_setCLK(LCD_CLK_GPIO_Port, LCD_CLK_Pin);
  LCD_setDC(LCD_DC_GPIO_Port, LCD_DC_Pin);
  LCD_setDIN(LCD_DIN_GPIO_Port, LCD_DIN_Pin);
  LCD_setRST(LCD_RST_GPIO_Port, LCD_RST_Pin);
  LCD_init();

  LCD_print("SIGNAL", 0, 0);
  LCD_print("GENERATOR", 0, 1);
  LCD_print("By Filipe", 0, 3);
  LCD_print("Chagas", 0, 4);
  LCD_print("SEP/2021", 0, 5);
  HAL_Delay(3000);
  select_freq();
  ui_global_state.init();
  generate_wave_cycle();
  HAL_I2S_Transmit_DMA(&hi2s2, txbuff, 1024);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S;
  PeriphClkInitStruct.PLLI2S.PLLI2SN = 200;
  PeriphClkInitStruct.PLLI2S.PLLI2SM = 16;
  PeriphClkInitStruct.PLLI2S.PLLI2SR = 2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2S2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S2_Init(void)
{

  /* USER CODE BEGIN I2S2_Init 0 */

  /* USER CODE END I2S2_Init 0 */

  /* USER CODE BEGIN I2S2_Init 1 */

  /* USER CODE END I2S2_Init 1 */
  hi2s2.Instance = SPI2;
  hi2s2.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s2.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s2.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s2.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
  hi2s2.Init.AudioFreq = I2S_AUDIOFREQ_44K;
  hi2s2.Init.CPOL = I2S_CPOL_LOW;
  hi2s2.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s2.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S2_Init 2 */

  /* USER CODE END I2S2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LCD_GND_GPIO_Port, LCD_GND_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LCD_BL_Pin|LCD_VCC_Pin|LCD_CLK_Pin|LCD_DIN_Pin
                          |LCD_DC_Pin|LCD_CE_Pin|LCD_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : ENC_CLK_Pin ENC_DT_Pin */
  GPIO_InitStruct.Pin = ENC_CLK_Pin|ENC_DT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : ENC_SW_Pin */
  GPIO_InitStruct.Pin = ENC_SW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(ENC_SW_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_GND_Pin */
  GPIO_InitStruct.Pin = LCD_GND_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LCD_GND_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_BL_Pin LCD_VCC_Pin LCD_CLK_Pin LCD_DIN_Pin
                           LCD_DC_Pin LCD_CE_Pin LCD_RST_Pin */
  GPIO_InitStruct.Pin = LCD_BL_Pin|LCD_VCC_Pin|LCD_CLK_Pin|LCD_DIN_Pin
                          |LCD_DC_Pin|LCD_CE_Pin|LCD_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

}

/* USER CODE BEGIN 4 */
int dt_state_array[2];
int clk_state_array[2];

void encoder_callback()
{
	dt_state_array[1] = !HAL_GPIO_ReadPin(ENC_DT_GPIO_Port, ENC_DT_Pin);
	clk_state_array[1] = !HAL_GPIO_ReadPin(ENC_CLK_GPIO_Port, ENC_CLK_Pin);

	/*   "For clockwise motion you can only perform the following actions:
		  (11 > 10), (10 > 00), (00 > 01) and (01 >11)
		  Similarly only the following encoder output transitions are valid for Anti-Clockwise rotation:
	      (01 > 00), (00 > 10), (10 > 11), and (11 > 01)"
	      Source: https://www.best-microcontroller-projects.com/rotary-encoder.html
	 */

	if((clk_state_array[0] == 1 && dt_state_array[0] == 1 && clk_state_array[1] == 1 && dt_state_array[1] == 0) ||
		//(clk_state_array[0] == 1 && dt_state_array[0] == 0 && clk_state_array[1] == 0 && dt_state_array[1] == 0) ||
		//(clk_state_array[0] == 0 && dt_state_array[0] == 0 && clk_state_array[1] == 1 && dt_state_array[1] == 0) ||
		//(clk_state_array[0] == 0 && dt_state_array[0] == 1 && clk_state_array[1] == 1 && dt_state_array[1] == 1) ||
		0)
	{
		ui_global_state.ccw_rotation();
	}

	if((clk_state_array[0] == 0 && dt_state_array[0] == 1 && clk_state_array[1] == 0 && dt_state_array[1] == 0) ||
		//(clk_state_array[0] == 0 && dt_state_array[0] == 0 && clk_state_array[1] == 1 && dt_state_array[1] == 0) ||
		//(clk_state_array[0] == 1 && dt_state_array[0] == 0 && clk_state_array[1] == 1 && dt_state_array[1] == 1) ||
		//(clk_state_array[0] == 1 && dt_state_array[0] == 1 && clk_state_array[1] == 0 && dt_state_array[1] == 1) ||
		0)
	{
		ui_global_state.cw_rotation();
	}

	dt_state_array[0] = dt_state_array[1];
	clk_state_array[0] = clk_state_array[1];
}

void switch_callback()
{
	int new_enc_sw_state = !HAL_GPIO_ReadPin(ENC_SW_GPIO_Port, ENC_SW_Pin);
	int new_enc_dt_state = !HAL_GPIO_ReadPin(ENC_DT_GPIO_Port, ENC_DT_Pin);
	int new_enc_clk_state = !HAL_GPIO_ReadPin(ENC_CLK_GPIO_Port, ENC_CLK_Pin);

	if(new_enc_clk_state == 0 && new_enc_dt_state == 0 && new_enc_sw_state == 1)
		ui_global_state.push();
}

double square_wave(double x)
{
	double y = 0;
	for(int k = 1; k <= harmonics; k++)
	{
		y += ((double)4/PI)*(sin(x*(2*k-1))/(2*k-1));
	}
	return y;
}

double triangle_wave(double x)
{
	double y = 0;
	for(int k = 1; k <= harmonics; k++)
	{
		y += ((double)8/(PI*PI))*sin((double)(k*PI)/2)*(sin(k*x)/(k*k));
	}
	return y;
}

double saw_wave(double x)
{
	double y = 0;
	for(int k = 1; k <= harmonics; k++)
	{
		y += ((double)2/PI)*(k % 2 == 0 ? 1 : -1)*(sin(k*x)/k);
	}
	return y;
}

double t_step = (double)1/(double)DAC_FREQ;
uint16_t cycle_buff[TXBUFF_LEN/2];
int cycle_len = 0;
int cycle_buff_lock = 0;

double generate_wave_cycle()
{
	cycle_buff_lock = 1;

	double t = 0;
	double (*wf)(double);
	switch(waveform)
	{
	case WAVEFORM_SIN:
		wf = sin;
		break;
	case WAVEFORM_SQUARE:
		wf = square_wave;
		break;
	case WAVEFORM_TRIANGLE:
		wf = triangle_wave;
		break;
	case WAVEFORM_SAW:
		wf = saw_wave;
		break;
	}

	double A = (double)amplitude/100;
	int i = 0;
	while(t <= (double)1/frequency)
	{
		cycle_buff[i] = (int)(AMPLITUDE_FACTOR*((A*wf(frequency*TAU*t)+A)/2));
		i += 1;
		t += t_step;
	}

	cycle_len = i-1;
	cycle_buff_lock = 0;
}

int j = 0;


/*
void HAL_I2S_TxHalfCpltCallback(I2S_HandleTypeDef *hi2s)
{
	if(cycle_buff_lock == 0)
	{
		for(int i = 0; i < TXBUFF_LEN/2; i+=2)
		{
			txbuff[i] = cycle_buff[j % cycle_len];
			txbuff[i+1] = txbuff[i];
			j += 1;
		}
	}
}
*/

void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef *hi2s)
{
	if(cycle_buff_lock == 0)
	{
		for(int i = 0; i < TXBUFF_LEN; i+=2)
		{
			txbuff[i] = cycle_buff[j % cycle_len];
			txbuff[i+1] = txbuff[i];
			j += 1;
		}
	}
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
