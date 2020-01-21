/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 ** This notice applies to any and all portions of this file
 * that are not between comment pairs USER CODE BEGIN and
 * USER CODE END. Other portions of this file, whether
 * inserted by the user or by software development tools
 * are owned by their respective copyright owners.
 *
 * COPYRIGHT(c) 2020 STMicroelectronics
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
#include "main.h"
#include "stm32f1xx_hal.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#define AVANCE 	GPIO_PIN_SET
#define RECULE  GPIO_PIN_RESET
#define POURCENT 640
#define Seuil_Dist_4 1000 // corespond à 10 cm.
#define Seuil_Dist_3 1000
#define Seuil_Dist_1 1000
#define Seuil_Dist_2 1000
#define V1 38
#define V2 56
#define V3 76
#define Vmax 95
#define T_2_S 1000 //( pwm période = 2 ms )
#define T_200_MS 100
#define T_2000_MS 1000
#define CKp_D 100  //80 Robot1
#define CKp_G 100  //80 Robot1
#define CKi_D 80  //50 Robot1
#define CKi_G 80  //50 Robot1
#define CKd_D 0
#define CKd_G 0
#define DELTA 0x50
#define DECALAGE 30
#define TOLERANCE 3
const char MON_ADRESSE[] = { 77 };

struct Position {
	int x, y, z;
};
volatile struct Position position_ref_o = { .x = -1, .y = -1, .z = -1 };
volatile struct Position position_ref_i = { .x = -1, .y = -1, .z = -1 };
enum CMDE {
	START, STOP, AVANT, ARRIERE, DROITE, GAUCHE, PARK, MOV_PARK, ATTENTE_PARK
};
volatile enum CMDE CMDE;
enum MODE {
	SLEEP, ACTIF_MODE
};
volatile enum MODE Mode;
enum MOV_PARK_ETAT {
	AVANCER_50cm, TOURNER_CCW, MOV_Z, TOURNER_CW, ALIGNER
};
enum GST_CMDE_ETAT {
	VEILLE,
	ARRET,
	AV1,
	AV2,
	AV3,
	RV1,
	RV2,
	RV3,
	DV1,
	DV2,
	DV3,
	GV1,
	GV2,
	GV3,
	PARK_STATE,
	MOV_PARK_STATE,
	ATTENTE_PARK_STATE
};
volatile enum GST_CMDE_ETAT gstCmdeEtat = VEILLE;
enum DIRECTION {
	POS_X, POS_Y, POS_Z
};
volatile enum DIRECTION direction;
volatile unsigned char New_CMDE = 0;
volatile char address_i;
volatile uint16_t Dist_ACS_1, Dist_ACS_2, Dist_ACS_3, Dist_ACS_4;
volatile unsigned int Time = 0;
volatile unsigned int Tech = 0;
uint16_t adc_buffer[10];
uint16_t Buff_Dist[8];
uint8_t BLUE_RX;
char XBEE_RX[20] = {0};
char XBEE_TX[20] = {0};

uint16_t _DirG, _DirD, CVitG, CVitD, DirD, DirG;
uint16_t _CVitD = 0;
uint16_t _CVitG = 0;
uint16_t VitD, VitG, DistD, DistG;
uint16_t DistD_old = 0;
uint16_t DistG_old = 0;
int Cmde_VitD = 0;
int Cmde_VitG = 0;
unsigned long Dist_parcours = 0;
volatile uint32_t Dist_Obst;
uint32_t Dist_Obst_;
uint32_t Dist_Obst_cm;
uint32_t Dist;
uint32_t Dist_mur;
uint8_t UNE_FOIS = 1;
uint32_t OV = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_NVIC_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void Gestion_Commandes(void);
void regulateur(void);
void controle(void);
void Calcul_Vit(void);
void ACS(void);
void Direction_Sonar(enum DIRECTION direction);
void movPark(void);
void Park(void);
void printToXBEE(char out[]);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 *
 * @retval None
 */
int main(void) {
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration----------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */
	Dist_Obst = 0;
	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_ADC1_Init();
	MX_TIM2_Init();
	MX_TIM3_Init();
	MX_TIM4_Init();
	MX_USART3_UART_Init();
	MX_TIM1_Init();
	MX_USART1_UART_Init();

	/* Initialize interrupts */
	MX_NVIC_Init();
	/* USER CODE BEGIN 2 */
	HAL_SuspendTick(); // suppresion des Tick interrupt pour le mode sleep.

	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);  // Start PWM motor
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);	//Start Servo
	CMDE = STOP;
	New_CMDE = 1;
	HAL_TIM_Base_Start_IT(&htim2);  // Start IT sur font montant PWM
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
	HAL_UART_Receive_IT(&huart3, &BLUE_RX, 1);
	HAL_UART_Receive_IT(&huart1, (uint8_t *) XBEE_RX, 1);
	HAL_ADC_Start_IT(&hadc1);
	HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_2);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
	Direction_Sonar(POS_X);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		Gestion_Commandes();
		controle();
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

	}
	/* USER CODE END 3 */

}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {

	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_PeriphCLKInitTypeDef PeriphClkInit;

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = 16;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
	PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV8;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Configure the Systick interrupt time
	 */
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

	/**Configure the Systick
	 */
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	/* SysTick_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/**
 * @brief NVIC Configuration.
 * @retval None
 */
static void MX_NVIC_Init(void) {
	/* EXTI15_10_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
	/* USART3_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(USART3_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(USART3_IRQn);
	/* USART1_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(USART1_IRQn);
	/* ADC1_2_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(ADC1_2_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(ADC1_2_IRQn);
}

/* USER CODE BEGIN 4 */
void Direction_Sonar(enum DIRECTION direction) {
	switch (direction) {
	case POS_X:
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 2300);
		break;
	case POS_Y:
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 795);
		break;
	case POS_Z:
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 4100);
		break;
	}
}

void Gestion_Commandes(void) {
	if (New_CMDE) {
		New_CMDE = 0;
		switch (CMDE) {
		case STOP: {
			_CVitD = _CVitG = 0;
			// Mise en sommeil: STOP mode , réveil via IT BP1
			gstCmdeEtat = VEILLE;
			Mode = SLEEP;

			break;
		}
		case START: {
			// réveil sytème grace à l'IT BP1
			gstCmdeEtat = ARRET;
			Mode = SLEEP;

			break;
		}
		case AVANT: {
			switch (gstCmdeEtat) {
			case VEILLE: {
				gstCmdeEtat = VEILLE;
				Mode = SLEEP;
				break;
			}
			case ARRET: {
				_DirG = AVANCE;
				_DirD = AVANCE;
				_CVitG = V1;
				_CVitD = V1;
				gstCmdeEtat = AV1;
				Mode = ACTIF_MODE;
				break;
			}
			case AV1: {
				_DirG = AVANCE;
				_DirD = AVANCE;
				_CVitG = V2;
				_CVitD = V2;
				gstCmdeEtat = AV2;
				Mode = ACTIF_MODE;
				break;
			}
			case AV2: {
				_DirG = AVANCE;
				_DirD = AVANCE;
				_CVitG = V3;
				_CVitD = V3;
				gstCmdeEtat = AV3;
				Mode = ACTIF_MODE;
				break;
			}
			case AV3: {
				_DirG = AVANCE;
				_DirD = AVANCE;
				_CVitG = V3;
				_CVitD = V3;
				gstCmdeEtat = AV3;
				Mode = ACTIF_MODE;
				break;
			}
			case RV1: {
				_DirG = RECULE;
				_DirD = RECULE;
				_CVitG = 0;
				_CVitD = 0;
				gstCmdeEtat = ARRET;
				Mode = SLEEP;

				break;
			}
			case RV2: {
				_DirG = RECULE;
				_DirD = RECULE;
				_CVitG = V1;
				_CVitD = V1;
				gstCmdeEtat = RV1;
				Mode = ACTIF_MODE;
				break;
			}
			case RV3: {
				_DirG = RECULE;
				_DirD = RECULE;
				_CVitG = V2;
				_CVitD = V2;
				gstCmdeEtat = RV2;
				Mode = ACTIF_MODE;
				break;
			}
			case DV1: {
				_DirG = AVANCE;
				_DirD = AVANCE;
				_CVitG = V1;
				_CVitD = V1;
				gstCmdeEtat = AV1;
				Mode = ACTIF_MODE;
				break;
			}
			case DV2: {
				_DirG = AVANCE;
				_DirD = AVANCE;
				_CVitG = V2;
				_CVitD = V2;
				gstCmdeEtat = AV2;
				Mode = ACTIF_MODE;
				break;
			}
			case DV3: {
				_DirG = AVANCE;
				_DirD = AVANCE;
				_CVitG = V3;
				_CVitD = V3;
				gstCmdeEtat = AV3;
				Mode = ACTIF_MODE;
				break;
			}
			case GV1: {
				_DirG = AVANCE;
				_DirD = AVANCE;
				_CVitG = V1;
				_CVitD = V1;
				gstCmdeEtat = AV2;
				Mode = ACTIF_MODE;
				break;
			}
			case GV2: {
				_DirG = AVANCE;
				_DirD = AVANCE;
				_CVitG = V2;
				_CVitD = V2;
				gstCmdeEtat = AV2;
				Mode = ACTIF_MODE;
				break;
			}
			case GV3: {
				_DirG = AVANCE;
				_DirD = AVANCE;
				_CVitG = V3;
				_CVitD = V3;
				gstCmdeEtat = AV3;
				Mode = ACTIF_MODE;
				break;
			}
			default:
				break;
			}
			break;
		}
		case ARRIERE: {
			switch (gstCmdeEtat) {
			case VEILLE: {
				gstCmdeEtat = VEILLE;
				Mode = SLEEP;
				break;
			}
			case ARRET: {
				_DirG = RECULE;
				_DirD = RECULE;
				_CVitG = V1;
				_CVitD = V1;
				gstCmdeEtat = RV1;
				Mode = ACTIF_MODE;
				break;
			}
			case AV1: {
				_DirG = AVANCE;
				_DirD = AVANCE;
				_CVitG = 0;
				_CVitD = 0;
				gstCmdeEtat = ARRET;
				Mode = SLEEP;

				break;
			}
			case AV2: {
				_DirG = AVANCE;
				_DirD = AVANCE;
				_CVitG = V1;
				_CVitD = V1;
				gstCmdeEtat = AV1;
				Mode = ACTIF_MODE;
				break;
			}
			case AV3: {
				_DirG = AVANCE;
				_DirD = AVANCE;
				_CVitG = V2;
				_CVitD = V2;
				gstCmdeEtat = AV2;
				Mode = ACTIF_MODE;
				break;
			}
			case RV1: {
				_DirG = RECULE;
				_DirD = RECULE;
				_CVitG = V2;
				_CVitD = V2;
				gstCmdeEtat = RV2;
				Mode = ACTIF_MODE;
				break;
			}
			case RV2: {
				_DirG = RECULE;
				_DirD = RECULE;
				_CVitG = V3;
				_CVitD = V3;
				gstCmdeEtat = RV3;
				Mode = ACTIF_MODE;
				break;
			}
			case RV3: {
				_DirG = RECULE;
				_DirD = RECULE;
				_CVitG = V3;
				_CVitD = V3;
				gstCmdeEtat = RV3;
				Mode = ACTIF_MODE;
				break;
			}
			case DV1: {
				_DirG = RECULE;
				_DirD = RECULE;
				_CVitG = V1;
				_CVitD = V1;
				gstCmdeEtat = RV1;
				Mode = ACTIF_MODE;
				break;
			}
			case DV2: {
				_DirG = RECULE;
				_DirD = RECULE;
				_CVitG = V2;
				_CVitD = V2;
				gstCmdeEtat = RV2;
				Mode = ACTIF_MODE;
				break;
			}
			case DV3: {
				_DirG = RECULE;
				_DirD = RECULE;
				_CVitG = V3;
				_CVitD = V3;
				gstCmdeEtat = RV3;
				Mode = ACTIF_MODE;
				break;
			}
			case GV1: {
				_DirG = RECULE;
				_DirD = RECULE;
				_CVitG = V1;
				_CVitD = V1;
				gstCmdeEtat = RV1;
				Mode = ACTIF_MODE;
				break;
			}
			case GV2: {
				_DirG = RECULE;
				_DirD = RECULE;
				_CVitG = V2;
				_CVitD = V2;
				gstCmdeEtat = RV2;
				Mode = ACTIF_MODE;
				break;
			}
			case GV3: {
				_DirG = RECULE;
				_DirD = RECULE;
				_CVitG = V3;
				_CVitD = V3;
				gstCmdeEtat = RV3;
				Mode = ACTIF_MODE;
				break;
			}
			default:
				break;
			}
			break;
		}
		case DROITE: {
			switch (gstCmdeEtat) {
			case VEILLE: {
				gstCmdeEtat = VEILLE;
				Mode = SLEEP;
				break;
			}
			case ARRET: {
				_DirG = AVANCE;
				_DirD = RECULE;
				_CVitG = V1;
				_CVitD = V1;
				gstCmdeEtat = DV1;
				Mode = ACTIF_MODE;
				break;
			}
			case AV1: {
				_DirG = AVANCE;
				_DirD = RECULE;
				_CVitG = V1;
				_CVitD = V1;
				gstCmdeEtat = DV1;
				Mode = ACTIF_MODE;
				break;
			}
			case AV2: {
				_DirG = AVANCE;
				_DirD = RECULE;
				_CVitG = V2;
				_CVitD = V2;
				gstCmdeEtat = DV2;
				Mode = ACTIF_MODE;
				break;
			}
			case AV3: {
				_DirG = AVANCE;
				_DirD = RECULE;
				_CVitG = V3;
				_CVitD = V3;
				gstCmdeEtat = DV3;
				Mode = ACTIF_MODE;
				break;
			}
			case RV1: {
				_DirG = AVANCE;
				_DirD = RECULE;
				_CVitG = V1;
				_CVitD = V1;
				gstCmdeEtat = DV1;
				Mode = ACTIF_MODE;
				break;
			}
			case RV2: {
				_DirG = AVANCE;
				_DirD = RECULE;
				_CVitG = V2;
				_CVitD = V2;
				gstCmdeEtat = DV2;
				Mode = ACTIF_MODE;
				break;
			}
			case RV3: {
				_DirG = AVANCE;
				_DirD = RECULE;
				_CVitG = V3;
				_CVitD = V3;
				gstCmdeEtat = DV3;
				Mode = ACTIF_MODE;
				break;
			}
			case DV1: {
				_DirG = AVANCE;
				_DirD = RECULE;
				_CVitG = V2;
				_CVitD = V2;
				gstCmdeEtat = DV2;
				Mode = ACTIF_MODE;
				break;
			}
			case DV2: {
				_DirG = AVANCE;
				_DirD = RECULE;
				_CVitG = V3;
				_CVitD = V3;
				gstCmdeEtat = DV3;
				Mode = ACTIF_MODE;
				break;
			}
			case DV3: {
				_DirG = AVANCE;
				_DirD = RECULE;
				_CVitG = V3;
				_CVitD = V3;
				gstCmdeEtat = DV3;
				Mode = ACTIF_MODE;
				break;
			}
			case GV1: {
				_DirG = RECULE;
				_DirD = RECULE;
				_CVitG = 0;
				_CVitD = 0;
				gstCmdeEtat = ARRET;
				Mode = SLEEP;

				break;
			}
			case GV2: {
				_DirG = RECULE;
				_DirD = AVANCE;
				_CVitG = V1;
				_CVitD = V1;
				gstCmdeEtat = GV1;
				Mode = ACTIF_MODE;
				break;
			}
			case GV3: {
				_DirG = RECULE;
				_DirD = AVANCE;
				_CVitG = V2;
				_CVitD = V2;
				gstCmdeEtat = GV2;
				Mode = ACTIF_MODE;
				break;
			}
			default:
				break;
			}
			break;
		}
		case GAUCHE: {
			switch (gstCmdeEtat) {
			case VEILLE: {
				gstCmdeEtat = VEILLE;
				Mode = SLEEP;
				break;
			}
			case ARRET: {
				_DirG = RECULE;
				_DirD = AVANCE;
				_CVitG = V1;
				_CVitD = V1;
				gstCmdeEtat = GV1;
				Mode = ACTIF_MODE;
				break;
			}
			case AV1: {
				_DirG = RECULE;
				_DirD = AVANCE;
				_CVitG = V1;
				_CVitD = V1;
				gstCmdeEtat = GV1;
				Mode = ACTIF_MODE;
				break;
			}
			case AV2: {
				_DirG = RECULE;
				_DirD = AVANCE;
				_CVitG = V2;
				_CVitD = V2;
				gstCmdeEtat = GV2;
				Mode = ACTIF_MODE;
				break;
			}
			case AV3: {
				_DirG = RECULE;
				_DirD = AVANCE;
				_CVitG = V3;
				_CVitD = V3;
				gstCmdeEtat = GV3;
				Mode = ACTIF_MODE;
				break;
			}
			case RV1: {
				_DirG = RECULE;
				_DirD = AVANCE;
				_CVitG = V1;
				_CVitD = V1;
				gstCmdeEtat = GV1;
				Mode = ACTIF_MODE;
				break;
			}
			case RV2: {
				_DirG = RECULE;
				_DirD = AVANCE;
				_CVitG = V2;
				_CVitD = V2;
				gstCmdeEtat = GV2;
				Mode = ACTIF_MODE;
				break;
			}
			case RV3: {
				_DirG = RECULE;
				_DirD = AVANCE;
				_CVitG = V3;
				_CVitD = V3;
				gstCmdeEtat = GV3;
				Mode = ACTIF_MODE;
				break;
			}
			case DV1: {
				_DirG = RECULE;
				_DirD = RECULE;
				_CVitG = 0;
				_CVitD = 0;
				gstCmdeEtat = ARRET;
				Mode = SLEEP;

				break;
			}
			case DV2: {
				_DirG = AVANCE;
				_DirD = RECULE;
				_CVitG = V1;
				_CVitD = V1;
				gstCmdeEtat = DV1;
				Mode = ACTIF_MODE;
				break;
			}
			case DV3: {
				_DirG = AVANCE;
				_DirD = RECULE;
				_CVitG = V2;
				_CVitD = V2;
				gstCmdeEtat = DV2;
				Mode = ACTIF_MODE;
				break;
			}
			case GV1: {
				_DirG = RECULE;
				_DirD = AVANCE;
				_CVitG = V2;
				_CVitD = V2;
				gstCmdeEtat = GV2;
				Mode = ACTIF_MODE;
				break;
			}
			case GV2: {
				_DirG = RECULE;
				_DirD = AVANCE;
				_CVitG = V3;
				_CVitD = V3;
				gstCmdeEtat = GV3;
				Mode = ACTIF_MODE;
				break;
			}
			case GV3: {
				_DirG = RECULE;
				_DirD = AVANCE;
				_CVitG = V3;
				_CVitD = V3;
				gstCmdeEtat = GV3;
				Mode = ACTIF_MODE;
				break;
			}
			default:
				break;
			}
			break;

		}
		case PARK: {
			if (gstCmdeEtat != VEILLE) {
				gstCmdeEtat = PARK_STATE;
				Mode = SLEEP;
			}
			break;
		}
		case MOV_PARK: {
			if (gstCmdeEtat != VEILLE) {
				gstCmdeEtat = MOV_PARK_STATE;
			}
			break;
		}
		case ATTENTE_PARK: {
			if (gstCmdeEtat != VEILLE) {
				gstCmdeEtat =
						(gstCmdeEtat != ATTENTE_PARK_STATE) ?
								ATTENTE_PARK_STATE : ARRET;
				Mode = SLEEP;
			}
			break;
		}
		}
	}

	if (gstCmdeEtat == MOV_PARK_STATE) {
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
		movPark();
	}

	if (gstCmdeEtat == PARK_STATE) {
		Park();
	}
}

void movPark(void) {
	static enum MOV_PARK_ETAT movParkEtat = AVANCER_50cm;
	static unsigned int cpt = 0;
	switch (movParkEtat) {
	case AVANCER_50cm: {
		if (cpt++ >= 40000) {
			movParkEtat = TOURNER_CCW;
			cpt = 0;
		} else {
			_CVitD = V1;
			_CVitG = V1;
			_DirD = AVANCE;
			_DirG = AVANCE;
			Mode = ACTIF_MODE;
		}
		break;
	}
	case TOURNER_CCW: {
		if (cpt++ >= 40000) {
			movParkEtat = MOV_Z;
			cpt = 0;
		} else {
			_CVitD = V1;
			_CVitG = V1;
			_DirD = AVANCE;
			_DirG = RECULE;
			Mode = ACTIF_MODE;
		}
		break;
	}
	case MOV_Z: {
		if (((long) Dist_mur - position_ref_i.z - DECALAGE) < -TOLERANCE
				|| ((long) Dist_mur - position_ref_i.z - DECALAGE) > TOLERANCE) {
			if ((long) Dist_mur < (position_ref_i.z + DECALAGE)) {
				_CVitD = V1;
				_CVitG = V1;
				_DirD = RECULE;
				_DirG = RECULE;
				Mode = ACTIF_MODE;
			} else if ((long) Dist_mur > (position_ref_i.z + DECALAGE)) {
				_CVitD = V1;
				_CVitG = V1;
				_DirD = AVANCE;
				_DirG = AVANCE;
				Mode = ACTIF_MODE;
			}
		} else {
			movParkEtat = TOURNER_CW;
		}
		break;
	}
	case TOURNER_CW: {
		if (cpt++ == 40000) {
			movParkEtat = ALIGNER;
			cpt = 0;
		} else {
			_CVitD = V1;
			_CVitG = V1;
			_DirD = RECULE;
			_DirG = AVANCE;
			Mode = ACTIF_MODE;
		}
		break;
	}
	case ALIGNER: {
		if ((long) Dist_mur > position_ref_i.x) {
			_CVitD = V1;
			_CVitG = V1;
			_DirD = AVANCE;
			_DirG = AVANCE;
			Mode = ACTIF_MODE;
		} else {
			_CVitD = 0;
			_CVitG = 0;
			Mode = SLEEP;
			movParkEtat = AVANCER_50cm;
			gstCmdeEtat = PARK_STATE;
			cpt = 0;
		}
		break;
	}
	}
}

void Park(void) {
	static int cpt = 0;
	static int offset = 0;
	if (cpt == 0) {
		address_i = 0;
		offset = 0;
		memset(XBEE_TX, 0, strlen(XBEE_TX));  // Clear buffer
		strncpy(XBEE_TX, ":@", 2);
		printToXBEE(XBEE_TX);  // First time : Ask for addresses.
	}
	cpt++;
	if (address_i) {
		if (offset == 0) {
			offset = cpt;
			cpt = 1;
		}
		if (cpt < 200000) {
			Direction_Sonar(POS_X);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
			position_ref_o.x = Dist_mur;
		} else if (cpt >= 200000 && cpt < 400000) {
			Direction_Sonar(POS_Y);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
			position_ref_o.y = Dist_mur;
		} else if (cpt >= 400000 && cpt < 600000) {
			Direction_Sonar(POS_Z);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
			position_ref_o.z = Dist_mur;
		} else {
			snprintf(XBEE_TX, 20, "%%x%05iy%05iz%05i", position_ref_o.x,
					position_ref_o.y, position_ref_o.z);
			printToXBEE(XBEE_TX);

			snprintf(XBEE_TX, 3, "#%i", address_i);
			printToXBEE(XBEE_TX);

			memset(XBEE_TX, 0, strlen(XBEE_TX));  // Clear buffer
			strncpy(XBEE_TX, ":M", 2);
			printToXBEE(XBEE_TX);
			cpt = 0;
			gstCmdeEtat = ARRET;
		}
	} else if (cpt > 2000000) {
		gstCmdeEtat = ARRET;
		cpt = 0;
	}
}

void controle(void) {

	if (Tech >= T_200_MS) {
		Tech = 0;
		ACS();
		Calcul_Vit();
		regulateur();
	}

}

void ACS(void) {
	enum ETAT {
		ARRET, ACTIF
	};
	static enum ETAT Etat = ARRET;
	static uint16_t Delta1 = 0;
	static uint16_t Delta2 = 0;
	static uint16_t Delta3 = 0;
	static uint16_t Delta4 = 0;

	switch (Etat) {
	case ARRET: {
		if (Mode == ACTIF_MODE)
			Etat = ACTIF;
		else {
			CVitD = _CVitD;
			CVitG = _CVitG;
			DirD = _DirD;
			DirG = _DirG;
		}
		break;
	}
	case ACTIF: {
		if (Mode == SLEEP)
			Etat = ARRET;
		if (_DirD == AVANCE && _DirG == AVANCE) {
			if ((Dist_ACS_1 < Seuil_Dist_1 - Delta1)
					&& (Dist_ACS_2 < Seuil_Dist_2 - Delta2)) {
				CVitD = _CVitD;
				CVitG = _CVitG;
				DirD = _DirD;
				DirG = _DirG;
				Delta1 = Delta2 = 0;
			} else if ((Dist_ACS_1 < Seuil_Dist_1)
					&& (Dist_ACS_2 > Seuil_Dist_2)) {
				CVitD = V1;
				CVitG = V1;
				DirG = AVANCE;
				DirD = RECULE;
				Delta2 = DELTA;
			} else if ((Dist_ACS_1 > Seuil_Dist_1)
					&& (Dist_ACS_2 < Seuil_Dist_2)) {
				CVitD = V1;
				CVitG = V1;
				DirD = AVANCE;
				DirG = RECULE;
				Delta1 = DELTA;
			} else if ((Dist_ACS_1 > Seuil_Dist_1)
					&& (Dist_ACS_2 > Seuil_Dist_2)) {
				CVitD = 0;
				CVitG = 0;
				DirD = RECULE;
				DirG = RECULE;
			}
		} else if (_DirD == RECULE && _DirG == RECULE) {
			if ((Dist_ACS_3 < Seuil_Dist_3 - Delta3)
					&& (Dist_ACS_4 < Seuil_Dist_4 - Delta4)) {
				CVitD = _CVitD;
				CVitG = _CVitG;
				DirD = _DirD;
				DirG = _DirG;
				Delta3 = Delta4 = 0;
			} else if ((Dist_ACS_3 > Seuil_Dist_3)
					&& (Dist_ACS_4 < Seuil_Dist_4)) {
				CVitD = V1;
				CVitG = V1;
				DirD = AVANCE;
				DirG = RECULE;
				Delta3 = DELTA;
			} else if ((Dist_ACS_3 < Seuil_Dist_3)
					&& (Dist_ACS_4 > Seuil_Dist_4)) {
				CVitD = V1;
				CVitG = V1;
				DirG = AVANCE;
				DirD = RECULE;
				Delta4 = DELTA;
			} else if ((Dist_ACS_3 > Seuil_Dist_3)
					&& (Dist_ACS_4 > Seuil_Dist_4)) {
				CVitD = 0;
				CVitG = 0;
				DirD = RECULE;
				DirG = RECULE;
			}
		} else {
			CVitD = _CVitD;
			CVitG = _CVitG;
			DirD = _DirD;
			DirG = _DirG;
		}
		break;
	}
	}
}

void Calcul_Vit(void) {

	DistD = __HAL_TIM_GET_COUNTER(&htim3);
	DistG = __HAL_TIM_GET_COUNTER(&htim4);
	VitD = abs(DistD - DistD_old);
	VitG = abs(DistG - DistG_old);
	DistD_old = DistD;
	DistG_old = DistG;
	if (DirD == DirG) {
		Dist_parcours = Dist_parcours + ((VitD + VitG) >> 1);
	}
}

void regulateur(void) {
	enum ETAT {
		ARRET, ACTIF
	};
	static enum ETAT Etat = ARRET;
	uint16_t Kp_D = CKp_D;
	uint16_t Kp_G = CKp_G;
	uint16_t Ki_D = CKi_D;
	uint16_t Ki_G = CKi_G;
	uint16_t Kd_D = CKd_D;
	uint16_t Kd_G = CKd_G;

	static int16_t ErreurD = 0;
	static int16_t ErreurG = 0;
	static int16_t ErreurD_old = 0;
	static int16_t ErreurG_old = 0;
	static int16_t S_erreursD = 0;
	static int16_t S_erreursG = 0;
	static int16_t V_erreurD = 0;
	static int16_t V_erreurG = 0;

	switch (Etat) {
	case ARRET: {
		if (Mode == ACTIF_MODE)
			Etat = ACTIF;
		else {
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
			HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_4);
			HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
			HAL_GPIO_WritePin(IR3_out_GPIO_Port, IR3_out_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(IR4_out_GPIO_Port, IR4_out_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(IR1_out_GPIO_Port, IR1_out_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(IR2_out_GPIO_Port, IR2_out_Pin, GPIO_PIN_RESET);

			HAL_PWR_EnterSLEEPMode(PWR_LOWPOWERREGULATOR_ON,
			PWR_SLEEPENTRY_WFI);

			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
			HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
			HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
			Time = 0;
		}
		break;
	}
	case ACTIF: {
		if ((CVitD != 0) && (CVitG != 0))
			Time = 0;
		if ((Mode == SLEEP) && (VitD == 0) && (VitG == 0) && Time > T_2_S)
			Etat = ARRET;
		else {
			ErreurD = CVitD - VitD;
			ErreurG = CVitG - VitG;
			S_erreursD += ErreurD;
			S_erreursG += ErreurG;
			V_erreurD = ErreurD - ErreurD_old;
			V_erreurG = ErreurG - ErreurG_old;
			ErreurD_old = ErreurD;
			ErreurG_old = ErreurG;
			Cmde_VitD = (unsigned int) Kp_D * (int) (ErreurD)
					+ (unsigned int) Ki_D * ((int) S_erreursD)
					+ (unsigned int) Kd_D * (int) V_erreurD;
			Cmde_VitG = (unsigned int) Kp_G * (int) (ErreurG)
					+ (unsigned int) Ki_G * ((int) S_erreursG)
					+ (unsigned int) Kd_G * (int) V_erreurG;

			//Cmde_VitD = _CVitD*640;
			//Cmde_VitG = _CVitG*640;
			//	DirD = _DirD;
			//	DirG= _DirG;

			if (Cmde_VitD < 0)
				Cmde_VitD = 0;
			if (Cmde_VitG < 0)
				Cmde_VitG = 0;
			if (Cmde_VitD > 100 * POURCENT)
				Cmde_VitD = 100 * POURCENT;
			if (Cmde_VitG > 100 * POURCENT)
				Cmde_VitG = 100 * POURCENT;
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, (uint16_t ) Cmde_VitG);
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, (uint16_t ) Cmde_VitD);
			HAL_GPIO_WritePin(DIR1_GPIO_Port, DIR1_Pin, (GPIO_PinState) DirD);
			HAL_GPIO_WritePin(DIR2_GPIO_Port, DIR2_Pin, (GPIO_PinState) DirG);

		}
		break;
	}
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {

	if (huart->Instance == USART3) {

		switch (BLUE_RX) {
		case 'F': {
			CMDE = AVANT;
			New_CMDE = 1;
			break;
		}

		case 'B': {
			CMDE = ARRIERE;
			New_CMDE = 1;
			break;
		}

		case 'L': {
			CMDE = GAUCHE;
			New_CMDE = 1;
			break;
		}

		case 'R': {
			CMDE = DROITE;
			New_CMDE = 1;
			break;
		}

		case 'W': {
			CMDE = PARK;
			New_CMDE = 1;
			break;
		}

		case 'U': {
			CMDE = ATTENTE_PARK;
			New_CMDE = 1;
			break;
		}
		case 'u': {
			CMDE = ATTENTE_PARK;
			New_CMDE = 1;
			break;
		}

		case 'D': {
			// disconnect bluetooth
			break;
		}
		default:
			break;
		}

		HAL_UART_Receive_IT(&huart3, &BLUE_RX, 1);//arme la réception du caractère suivant

	} else if (huart->Instance == USART1) {
		switch (XBEE_RX[0]) {
		case '#':   // #[char]
			address_i = XBEE_RX[1];
			break;

		case ':':  // :[command], do not forgot to get address
			switch (XBEE_RX[1]) {
			case '@':
				if (gstCmdeEtat == ATTENTE_PARK_STATE) {
					snprintf(XBEE_TX, 3, "#%s", MON_ADRESSE);
					printToXBEE(XBEE_TX);
				}
				break;

			case 'M':
				if (address_i == MON_ADRESSE[0]) {
					CMDE = MOV_PARK;
					New_CMDE = 1;
				}
				break;

			default:  //[TODO] faire comme avant avec 1 caractère
				break;
			}
			break;  // case ':'

		case '%':   // %x12345y12345z12345
			sscanf((char *) XBEE_RX, "%%x%05iy%05iz%05i", &position_ref_i.x,
					&position_ref_i.y, &position_ref_i.z);
			break;

		default:
			break;
		}

		memset(XBEE_RX, 0, sizeof(XBEE_RX));
		while (HAL_UART_Receive_IT(&huart1, (uint8_t *) XBEE_RX, 1) == HAL_BUSY);
	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {

	Dist_ACS_3 = adc_buffer[0] - adc_buffer[5];
	Dist_ACS_4 = adc_buffer[3] - adc_buffer[8];
	Dist_ACS_1 = adc_buffer[1] - adc_buffer[6];
	Dist_ACS_2 = adc_buffer[2] - adc_buffer[7];
	HAL_ADC_Stop_DMA(hadc);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef * htim) {
	static unsigned char cpt = 0;

	if (htim->Instance == TIM2) {
		cpt++;
		Time++;
		Tech++;

		switch (cpt) {
		case 1: {
			HAL_GPIO_WritePin(IR3_out_GPIO_Port, IR3_out_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(IR4_out_GPIO_Port, IR4_out_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(IR1_out_GPIO_Port, IR1_out_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(IR2_out_GPIO_Port, IR2_out_Pin, GPIO_PIN_SET);
			break;
		}
		case 2: {
			HAL_ADC_Start_DMA(&hadc1, (uint32_t *) adc_buffer, 10);
			break;
		}
		case 3: {
			HAL_GPIO_WritePin(IR3_out_GPIO_Port, IR3_out_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(IR4_out_GPIO_Port, IR4_out_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(IR1_out_GPIO_Port, IR1_out_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(IR2_out_GPIO_Port, IR2_out_Pin, GPIO_PIN_RESET);
			break;
		}
		case 4: {
			HAL_ADC_Start_DMA(&hadc1, (uint32_t *) adc_buffer, 10);
			break;
		}
		default:
			cpt = 0;
		}
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {

	static unsigned char TOGGLE = 0;

	if (TOGGLE)
		CMDE = STOP;
	else
		CMDE = START;
	TOGGLE = ~TOGGLE;
	New_CMDE = 1;

}

void HAL_ADC_LevelOutOfWindowCallback(ADC_HandleTypeDef* hadc1) {
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef* htim) {
	if (htim->Instance == TIM1) {
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
			Dist_mur = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
	}
}

void printToXBEE(char out[]) {
	HAL_UART_Transmit(&huart1, (uint8_t *) out, strlen(out), 10);
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  file: The file name as string.
 * @param  line: The line in file as a number.
 * @retval None
 */
void _Error_Handler(char *file, int line) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	while (1) {
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
void assert_failed(uint8_t* file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	 tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
