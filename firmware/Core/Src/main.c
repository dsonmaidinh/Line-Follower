/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "motor_control.h"
#include "sensor.h"
#include "math.h"
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
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t test = 0;
//unsigned long lastTime = 0;
//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
//		unsigned long currentTime = HAL_GetTick();
//    if (currentTime - lastTime < 200){
//				return;
//		}
//		if ( S2 == Thay_Line && S4 == Khong_Thay_Line){
//			TurnRight();
//			test = 3;
//	 }
//		else if( S2 == Khong_Thay_Line && S4 == Thay_Line){
//			TurnLeft();
//			test = 2;
//	 }
//	 else if ( S2 == Khong_Thay_Line && S4 == Khong_Thay_Line){
//			GoAhead();
//			test = 1;
//	}
//	else {
//			Stop();
//			test = 0;
//	}
//	lastTime = currentTime;
//}
//const uint16_t ADC_THRESHOLD = 2000;

//void read_sensors(void)
//{
////  uint8_t s1 = HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_0); // Ngo�i c�ng b�n tr�i
////  uint8_t s2 = HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_11);
////  uint8_t s3 = HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_1);
////  uint8_t s4 = HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_10); // Ngo�i c�ng b�n ph?i
//	
//	read_adc_tcrt5000();
//  // T�nh to�n 'error' d?a tr�n tr?ng th�i c?m bi?n.
//  // Gi� tr? duong nghia l� xe l?ch ph?i, �m l� l?ch tr�i.
////  if      ((s1==0) && (s2==0) && (s3==0) && (s4==1)) error = 3;
////  else if ((s1==0) && (s2==0) && (s3==1) && (s4==1)) error = 2;
////  else if ((s1==0) && (s2==0) && (s3==1) && (s4==0)) error = 1;
////  else if ((s1==0) && (s2==1) && (s3==1) && (s4==0)) error = 0; // V? tr� l� tu?ng
////  else if ((s1==0) && (s2==1) && (s3==0) && (s4==0)) error = -1;
////  else if ((s1==1) && (s2==1) && (s3==0) && (s4==0)) error = -2;
////  else if ((s1==1) && (s2==0) && (s3==0) && (s4==0)) error = -3;
////  // N?u kh�ng th?y line, gi? l?i cu?i c�ng
////  else if ((s1==0) && (s2==0) && (s3==0) && (s4==0)) {
////			GPIOB->ODR &= ~((1 << 3) | (1 << 4) | (1 << 5) |(1 << 6));
////      if (lastError > 0) error = 4;
////      else error = -4;
////  }
//}

// --- PID Constants ---
double Kp = 110;   // T? l?
double Ki = 0;    // T�ch ph�n
double Kd = 20;   // Vi ph�n

// --- PID Variables ---
int error = 0;
int lastError = 0;
double integral = 0;
double derivative = 0;
double pid_value = 0;

// --- Motor Speed ---
int motor_speed = 650; // T?c d? co b?n (0-999)
int left_motor_speed = 0;
int right_motor_speed = 0;

uint16_t adc_val[5] = {0};  // Lưu giá trị ADC cho 5 mắt
uint8_t digital_sensor[5] = {0};  // Sau khi so với ngưỡng thì là 0 hoặc 1
const uint16_t ADC_THRESHOLD = 2000;

void read_sensors_digital_from_adc(void)
{
    ADC_ChannelConfTypeDef sConfig = {0};

    for (int i = 0; i < 5; i++) {
        sConfig.Channel = ADC_CHANNEL_3 + i;  // ADC1_IN3 đến IN7
        sConfig.Rank = ADC_REGULAR_RANK_1;
        sConfig.SamplingTime = ADC_SAMPLETIME_71CYCLES_5;
        HAL_ADC_ConfigChannel(&hadc1, &sConfig);

        HAL_ADC_Start(&hadc1);
        if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) {
            adc_val[i] = HAL_ADC_GetValue(&hadc1);
        }
        HAL_ADC_Stop(&hadc1);

        // So sánh với ngưỡng để tạo digital (TTL style)
        if (adc_val[i] < ADC_THRESHOLD)
            digital_sensor[i] = 1;  // phát hiện line (đen)
        else
            digital_sensor[i] = 0;  // không phát hiện
    }
}

void calculate_error_from_digital_sensor(void)
{
    uint8_t s0 = digital_sensor[4]; // trái ngoài
    uint8_t s1 = digital_sensor[3]; // trái giữa
    uint8_t s2 = digital_sensor[2]; // giữa
    uint8_t s3 = digital_sensor[1]; // phải giữa
    uint8_t s4 = digital_sensor[0]; // phải ngoài

    if      ((s0==0) && (s1==0) && (s2==0) && (s3==0) && (s4==1)) error = 8;
    else if ((s0==0) && (s1==0) && (s2==0) && (s3==1) && (s4==1)) error = 5;
    else if ((s0==0) && (s1==0) && (s2==0) && (s3==1) && (s4==0)) error = 5;
    else if ((s0==0) && (s1==0) && (s2==1) && (s3==0) && (s4==0)) error = 0;
    else if ((s0==0) && (s1==1) && (s2==1) && (s3==0) && (s4==0)) error = -5;
    else if ((s0==1) && (s1==1) && (s2==0) && (s3==0) && (s4==0)) error = -5;
    else if ((s0==1) && (s1==0) && (s2==0) && (s3==0) && (s4==0)) error = -8;
    else if ((s0==0) && (s1==0) && (s2==0) && (s3==0) && (s4==0)) {
        if (lastError > 0) error = 4;
        else error = -4;
    }

    lastError = error;
}


void calculate_pid(void)
{
    // Th�nh ph?n t? l? (Proportional)
    // Ph?n ?ng t?c th?i v?i l?i hi?n t?i.
    // P = Kp * error;

    // Th�nh ph?n t�ch ph�n (Integral)
    // T?ng h?p c�c l?i trong qu� kh?, gi�p lo?i b? sai s? x�c l?p.
    integral += error;

    // Th�nh ph?n vi ph�n (Derivative)
    // D? do�n l?i trong tuong lai b?ng c�ch xem x�t t?c d? thay d?i c?a l?i.
    derivative = error - lastError;

    // T�nh to�n t?ng gi� tr? PID
    pid_value = (Kp * error) + (Ki * integral) + (Kd * derivative);

    // C?p nh?t l?i cu?i c�ng
    lastError = error;
}
int left_motor_speed_raw;
int right_motor_speed_raw;
int speed = 0;
void control_motors(void) {
    // T�nh to�n t?c d? raw (c� th? �m ho?c duong)
    left_motor_speed_raw 	= motor_speed - pid_value;
    right_motor_speed_raw = motor_speed + pid_value;
		// Gi?i h?n t?c d? t?i da 999
		if (left_motor_speed_raw > 800) 				left_motor_speed_raw 	= 800;
		else if( right_motor_speed_raw > 800) 	right_motor_speed_raw = 800;
    if (left_motor_speed_raw < -800 ) 			left_motor_speed_raw  = -800;
		else if( right_motor_speed_raw < -800) 	right_motor_speed_raw = -800;
		speed = 0;
		if(left_motor_speed_raw > right_motor_speed_raw) {
				speed = left_motor_speed_raw; // B? d?u �m
				if(speed < 0) speed = -speed;
				
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, speed);
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, speed);
			
				GPIOB->ODR &= ~((1 << 3) | (1 << 4) | (1 << 5) |(1 << 6));
				GPIOB->ODR |= (1<<4) | (1<<5);	// trai lui phai tien
		}
		else if (left_motor_speed_raw < right_motor_speed_raw) {
				speed = right_motor_speed_raw; // B? d?u �m
				if(speed < 0) speed = -speed;
			
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, speed);
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, speed);
				
				GPIOB->ODR &= ~((1 << 3) | (1 << 4) | (1 << 5) |(1 << 6));
				GPIOB->ODR |= (1<<3) | (1<<6);	// trai tien phai lui
		}
		else{
				speed = right_motor_speed_raw; // B? d?u �m
			
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, speed);
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, speed);
				
				GPIOB->ODR &= ~((1 << 3) | (1 << 4) | (1 << 5) |(1 << 6));
				GPIOB->ODR |= (1<<3) | (1<<5);
		}
		
		//        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);   // IN1 = 1
//        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET); // IN2 = 0
//				
//        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET); // IN3 = 0
//        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);   // IN4 = 1
//    // --- X? l� motor TR�I ---
//    if (left_motor_speed_raw > 0) {
//				
//    } 
//    else {
//        
//        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, speed);
//        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET); // IN1 = 0
//        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);   // IN2 = 1
//    }

//    // --- X? l� motor PH?I ---
//    if (right_motor_speed_raw > 0) {
//				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, speed);
//        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);   // IN3 = 1
//        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET); // IN4 = 0
//    } 
//    else {
//				
//    }
}
void control_motors_v2(void) {
    // T�nh to�n t?c d? raw (c� th? �m ho?c duong)
    left_motor_speed_raw 	= motor_speed - pid_value;
    right_motor_speed_raw = motor_speed + pid_value;
		// Gi?i h?n t?c d? t?i da 999
		if (left_motor_speed_raw > 800) 				left_motor_speed_raw 	= 800;
		else if( right_motor_speed_raw > 800) 	right_motor_speed_raw = 800;
    if (left_motor_speed_raw < -800 ) 			left_motor_speed_raw  = -800;
		else if( right_motor_speed_raw < -800) 	right_motor_speed_raw = -800;
		speed = 0;
		if(left_motor_speed_raw < 0) {
				speed = left_motor_speed_raw; // B? d?u �m
				if(speed < 0) speed = -speed;
				
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, speed);
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, speed);
			
				GPIOB->ODR &= ~((1 << 3) | (1 << 4) | (1 << 5) |(1 << 6));
				GPIOB->ODR |= (1<<4) | (1<<5);	// trai lui phai tien
		}
		else if (right_motor_speed_raw < 0) {
				speed = right_motor_speed_raw; // B? d?u �m
				if(speed < 0) speed = -speed;
			
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, speed);
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, speed);
				
				GPIOB->ODR &= ~((1 << 3) | (1 << 4) | (1 << 5) |(1 << 6));
				GPIOB->ODR |= (1<<3) | (1<<6);	// trai tien phai lui
		}
		else{
				speed = right_motor_speed_raw; // B? d?u �m
			
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, speed);
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, speed);
				
				GPIOB->ODR &= ~((1 << 3) | (1 << 4) | (1 << 5) |(1 << 6));
				GPIOB->ODR |= (1<<3) | (1<<5);
		}
}
int son = 0;
//4 trai tien, 3 trai lui, 5 phai lui, 6 phai tien
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
  MX_TIM1_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	Stop();
//	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 800);
//	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 800);

//	GPIOB->ODR &= ~((1 << 3) | (1 << 4) | (1 << 5) |(1 << 6));
//	GPIOB->ODR |= (1<<3) | (1<<5);	// trai tien phai lui
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		read_sensors_digital_from_adc();     // Đọc ADC + so ngưỡng
    calculate_error_from_digital_sensor(); // Chuyển sang error kiểu TTL
    calculate_pid();
    control_motors();
		
    HAL_Delay(2);
//		calculate_error_from_digital_sensor();
//    calculate_pid();
//    control_motors();
//    HAL_Delay(10); // Th�m m?t d? tr? nh?
//		line_follow();
//		HAL_Delay(10);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 71;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB0 PB1 PB10 PB11 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB3 PB4 PB5 PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
