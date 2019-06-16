/**
 *
 *
 *
 * test
 ******************************************************************************
 * File Name          : main.c
 * Description        : Main program body
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
#include "stm32f4xx_hal.h"
#include "usb_device.h"

/* USER CODE BEGIN Includes */
#include "main.h"
#include "debug.h"
#include "config_drone.h"
#include "timer.h"
#include "rc.h"
#include "steval_fcu001_v1.h"
#include "steval_fcu001_v1_accelero.h"
#include "steval_fcu001_v1_gyro.h"
#include "steval_fcu001_v1_magneto.h"
#include "steval_fcu001_v1_pressure.h"
#include "steval_fcu001_v1_temperature.h"
#include "sensor_data.h"
#include "quaternion.h"
#include "ahrs.h"
#include "flight_control.h"
#include "motor.h"
#include "ble_status.h"
#include "SPBTLE_RF.h"
#include "bluenrg_gatt_server.h"
#include "sensor_service.h"
#include "bluenrg_utils.h"
#include "TargetFeatures.h"

#include "bluenrg_l2cap_aci.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
volatile uint32_t HCI_ProcessEvent = 0;
uint8_t joydata[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };

uint32_t uhCCR4_Val = 500;
uint32_t uhCCR1_Val = 5000;

ADC_HandleTypeDef hadc1;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim9;

UART_HandleTypeDef huart1;
int32_t rc_cal_flag = 0;
int32_t rc_enable_motor = 0;
int32_t rc_cal_cnt = 0;
int32_t fly_ready = 0;


/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM9_Init(void);
static void MX_USART1_UART_Init(void);

void UART1_init();
int myprint(uint8_t *format);
void ADC1_Init(void);


typedef struct {
	int16_t X_Degree;
	int16_t Y_Degree;
	int16_t Z_Degree;
} Attitude_Degree;

typedef struct {
	float a1, a2, b0, b1, b2;
} IIR_Coeff;


Attitude_Degree Fly, Fly_offset, Fly_origin;
Gyro_Rad gyro_rad, gyro_degree, gyro_cali_degree;
MotorControlTypeDef motor_pwm;
int count1 = 0, count2 = 0;
AHRS_State_TypeDef ahrs;
float press, press_zero_level;
float temperature;

uint32_t VBAT_Sense;
float VBAT = 0;

uint8_t tmp_lis2mdl;
SensorAxes_t tmp_mag;

/* BLE */
extern uint8_t set_connectable;
uint16_t service_handle, dev_name_char_handle, appearance_char_handle;
uint32_t ConnectionBleStatus = 0;
uint8_t BufferToWrite[256];
int32_t BytesToWrite;

/* USER CODE END 0 */

int main(void) {


	  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	  HAL_Init();

	  /* Configure the system clock */
	  SystemClock_Config();

	  /* Initialize all configured peripherals */
	  MX_GPIO_Init();

	  MX_TIM2_Init();
	  MX_TIM4_Init();
	  MX_TIM9_Init();
	  UART1_init();
	  ADC1_Init();

	//  Initialize Onboard LED
	  BSP_LED_Init(LED1);
	  BSP_LED_Init(LED2);
	  BSP_LED_Off(LED1);
	  BSP_LED_Off(LED2);
	  BSP_LED_On(LED1);
	  BSP_LED_On(LED2);


	  /* Initialize General purpose TIM9 50Hz*/
	  HAL_TIM_Base_Start_IT(&htim9);

	  /* Infinite loop */
	  /* USER CODE BEGIN WHILE */
	  while (1) {

		/********************************************************************************/
		/* Your implementation starts here                                                         */
		/********************************************************************************/ 
		  
		  
		/********************************************************************************/
		/* Your implementation starts here                                                         */
		/********************************************************************************/ 

	}
}

/********************************************************************************/
/* Your implementation starts here                                                         */
/********************************************************************************/ 
void ADC1_Init(void)
{
	//Your ADC code here

}
void UART1_init()
 {

	//Your UART code here
 }
 int myprint(uint8_t *format)
 {
	//Your UART code here

	 return 0;
 }
/********************************************************************************/
/* Your implementation stops here                                                         */
/********************************************************************************/ 


/** System Clock Configuration
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;

	/**Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE()
	;

	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 16;
	RCC_OscInitStruct.PLL.PLLN = 336;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
	RCC_OscInitStruct.PLL.PLLQ = 7;

	HAL_RCC_OscConfig(&RCC_OscInitStruct);

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
	HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);

	/**Configure the Systick interrupt time
	 */
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

	/**Configure the Systick
	 */
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	/* SysTick_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);

}


/* TIM2 init function */
void MX_TIM2_Init(void) {

	TIM_ClockConfigTypeDef sClockSourceConfig;
	TIM_MasterConfigTypeDef sMasterConfig;
	TIM_IC_InitTypeDef sConfigIC;

	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 20;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 32767;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	HAL_TIM_Base_Init(&htim2);

	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig);

	HAL_TIM_IC_Init(&htim2);

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig);

	sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
	sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
	sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
	sConfigIC.ICFilter = 0;
	HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1);

	HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_2);

	HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_3);

	HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_4);

}

/* TIM4 init function */
void MX_TIM4_Init(void) {

	TIM_ClockConfigTypeDef sClockSourceConfig;
	TIM_MasterConfigTypeDef sMasterConfig;
	TIM_OC_InitTypeDef sConfigOC;

	htim4.Instance = TIM4;
#ifdef MOTOR_DC
	htim4.Init.Prescaler = 84; /* DC motor configuration - Freq 494Hz*/
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim4.Init.Period = 1999;
#endif
#ifdef MOTOR_ESC
	htim4.Init.Prescaler = 100; /* ESC motor configuration - Freq 400Hz*/
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim4.Init.Period = 2075;
#endif

	htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	HAL_TIM_Base_Init(&htim4);

	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig);

	HAL_TIM_PWM_Init(&htim4);

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig);

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1);

	HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2);

	HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3);

	HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4);

}

/* TIM9 init function */
void MX_TIM9_Init(void) {

	TIM_ClockConfigTypeDef sClockSourceConfig;

	htim9.Instance = TIM9;
	htim9.Init.Prescaler = 51;
	htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim9.Init.Period = 1999;
	htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	HAL_TIM_Base_Init(&htim9);

	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig);

}


/** Configure pins as
 * Analog
 * Input
 * Output
 * EVENT_OUT
 * EXTI
 * Free pins are configured automatically as Analog (this feature is enabled through
 * the Code Generation settings)
 */
void MX_GPIO_Init(void) {

	GPIO_InitTypeDef GPIO_InitStruct;

	/* GPIO Ports Clock Enable */
	__GPIOC_CLK_ENABLE()
	;
	__GPIOA_CLK_ENABLE()
	;
	__GPIOB_CLK_ENABLE()
	;

	/*Configure GPIO pins : PB4 PB5 */
	GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_5;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
/*
 *  Handle Timer9 interrupt @ 800Hz
 *  Set the event flag and increase time index
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {

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

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
