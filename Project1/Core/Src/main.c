/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Uses two potentiometers to read the travel of an acceleration pedal.
 * These values are taken from the ADC via the DMA(oversampled(64) and then averaged)
 * Calculates the percentage travel of the values after applying a deadzone upper and lower threshold
 * The system then checks for two types of implausibilities
 	 * Brake pedal is pressed while acceleration is pressed past a certain threshold
 	 * The two potentiometers differ larger than the allowed threshold
 * After everything is calculated a CAN message is made and sent (sending implausibility and avg pedal acceleration)
 * The system always starts in calibration mode to make sure it gets the full range of potentiometer values before it drives
 * The reset button it pressed to turn calibration mode off manually.
 * Alternatively the bool can be turned off and the values preset to something close to the know values and let them update
 * */

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define pedalImplausibilityThresh 10
#define pedalDeadzoneThresh 8
#define minMaxOccurancesThresh 2
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

FDCAN_HandleTypeDef hfdcan1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* INPUT VARIABLES START */
volatile uint16_t adcDMA[2]; //Memory for DMA results
const int adcmsgCnt = 2; //Number of DMA conversions
volatile int adcConvComp = 0; //Wait until completion boolean
uint8_t btn0 = 0; //Blue button(1 Default)
uint8_t brakeBtn = 0; //button(0 Default) (pin 4)
uint8_t resetBtn = 0; //button(0 Default) (pin 5)
/* INPUT VARIABLES END */

/* RUNNING STATE VARIABLES START*/
uint8_t calibrationModeBool = 1; //To get the full range of potentiometer values before driving
uint8_t testingModeBool = 0; //For testing makes sure CAN always sends 0
uint8_t pedalImplausibilityBool = 0; //Bool for acceleration pedal implausibility
uint8_t brakeImplausibilityBool = 0; //Bool for brake pedal implausibility
uint8_t implausibility = 0; //True if any of the above are true (used for CAN)
/* RUNNING STATE VARIABLES END*/

/* ANALOG VALUES START */
//Represent the true min/max analog values for the 5V and 3V
uint16_t accel5VMax = 2000;
uint16_t accel5VMin = 600;
uint16_t accel3VMax = 2000;
uint16_t accel3VMin = 600;

//Represent the min/max values after threshold has been applied for the 5V and 3V
uint16_t accel5VMaxThresh = 2000;
uint16_t accel5VMinThresh = 600;
uint16_t accel3VMaxThresh = 2000;
uint16_t accel3VMinThresh = 600;

//Variables to keep count of new max/min to meet minimum threshold to change values
uint8_t new5VMaxCnt = 0;
uint8_t new5VMinCnt = 0;
uint8_t new3VMaxCnt = 0;
uint8_t new3VMinCnt = 0;

//Variables to keep the percentages (0-100%)
uint8_t accel5VPercentage = 0;
uint8_t accel3VPercentage = 0;
uint8_t accelImplausabilityPercentage = 0;
uint8_t accelAvgPercentage = 0;
uint8_t accelCANPercentage = 0;
/* ANALOG VALUES END */

/* MILLIS TIMER VARIABLES START */
uint32_t accelerationImplausibilityTimer = 0;
uint32_t brakeImplausibilityTimer = 0;
/* MILLIS TIMER VARIABLES START */

/* CAN START */
uint8_t CANErrorMsgCnt = 0;
uint8_t CANReceivedMsgCnt = 0;
uint16_t CANId = 0x115;
/* CAN END */

/* UART START */ //Used for debugging using UART
uint8_t msgCnt = 0;
char msgBuff[80];
uint8_t buffSz = 80;
uint8_t msgSz = 0;

/* UART END */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_FDCAN1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* Reads the inputs from the ADC through the DMA
 * The ADC is oversampling at 64 samples and shifted right 6 times
 * Readings:
 	 	 	* Potentiometer 1 (5V)
 	 	 	* Potentiometer 2 (3V)
 	 	 	* Button 0 -> Blue board pin has no current use
 	 	 	* Button 1(pin 5) -> Reset pin for all of the implausibilities
 	 	 	* Button 2(pin 6) -> Repesents the brake pedal reading as a boolean value */
void readInputs(){
	//Get DMA readings for both potentiometers
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adcDMA, adcmsgCnt);
	while (!adcConvComp){ /*Wait will next conversion is done*/ }
	adcConvComp = 0; //Reset event variable

	//Get button readings
	btn0 = HAL_GPIO_ReadPin(Btn_GPIO_Port, Btn_Pin);

	resetBtn = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5);
	if(resetBtn){
		calibrationModeBool = 0;
		pedalImplausibilityBool = 0;
		brakeImplausibilityBool = 0;
	}

	brakeBtn = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4);
}

/* Prints all values for debugging */
void printValues(){
	//Potentiometer & button readings
	msgSz = snprintf(msgBuff, buffSz, "%d 5VPot: %d, 3VPot: %d\r\n", msgCnt, adcDMA[0], adcDMA[1]);
	HAL_UART_Transmit(&huart2, (uint8_t*)msgBuff, msgSz, HAL_MAX_DELAY);
	msgSz = snprintf(msgBuff, buffSz, "%d BrakeBtn: %d, ResetBtn: %d\r\n", msgCnt, brakeBtn, resetBtn);
	HAL_UART_Transmit(&huart2, (uint8_t*)msgBuff, msgSz, HAL_MAX_DELAY);

	//Accelerator petal Min/Max
	msgSz = snprintf(msgBuff, buffSz, "%d 5VMax: %d, 5VMin: %d\r\n", msgCnt, accel5VMax, accel5VMin);
	HAL_UART_Transmit(&huart2, (uint8_t*)msgBuff, msgSz, HAL_MAX_DELAY);
	msgSz = snprintf(msgBuff, buffSz, "%d 3VMax: %d, 3VMin: %d\r\n", msgCnt, accel3VMax, accel3VMin);
	HAL_UART_Transmit(&huart2, (uint8_t*)msgBuff, msgSz, HAL_MAX_DELAY);

	//Accelerator petal Min/Max Calibrated
	msgSz = snprintf(msgBuff, buffSz, "%d Thresh 5VMax: ", msgCnt);
	HAL_UART_Transmit(&huart2, (uint8_t*)msgBuff, msgSz, HAL_MAX_DELAY);
	msgSz = snprintf(msgBuff, buffSz, "%d, 5VMin: %d\r\n", accel5VMaxThresh, accel5VMinThresh);
	HAL_UART_Transmit(&huart2, (uint8_t*)msgBuff, msgSz, HAL_MAX_DELAY);

	msgSz = snprintf(msgBuff, buffSz, "%d Thresh 3VMax: ", msgCnt);
	HAL_UART_Transmit(&huart2, (uint8_t*)msgBuff, msgSz, HAL_MAX_DELAY);
	msgSz = snprintf(msgBuff, buffSz, "%d, 3VMin: %d\r\n", accel3VMaxThresh, accel3VMinThresh);
	HAL_UART_Transmit(&huart2, (uint8_t*)msgBuff, msgSz, HAL_MAX_DELAY);

	//Accelerator percentages
	msgSz = snprintf(msgBuff, buffSz, "%d Percentage: ", msgCnt);
	HAL_UART_Transmit(&huart2, (uint8_t*)msgBuff, msgSz, HAL_MAX_DELAY);
	msgSz = snprintf(msgBuff, buffSz, "5VP: %d, 3VP: %d, AvgP: %d\r\n", accel5VPercentage, accel3VPercentage, accelAvgPercentage);
	HAL_UART_Transmit(&huart2, (uint8_t*)msgBuff, msgSz, HAL_MAX_DELAY);

	//Implausibilities
	msgSz = snprintf(msgBuff, buffSz, "%d PedIm: %d, BrakeIm: %d\r\n", msgCnt, pedalImplausibilityBool, brakeImplausibilityBool);
	HAL_UART_Transmit(&huart2, (uint8_t*)msgBuff, msgSz, HAL_MAX_DELAY);

	msgSz = snprintf(msgBuff, buffSz, "%d Calibration: %d, ", msgCnt, calibrationModeBool);
	HAL_UART_Transmit(&huart2, (uint8_t*)msgBuff, msgSz, HAL_MAX_DELAY);
	msgSz = snprintf(msgBuff, buffSz, "Testing: %d\n\n \r\n", testingModeBool);
	HAL_UART_Transmit(&huart2, (uint8_t*)msgBuff, msgSz, HAL_MAX_DELAY);

	msgCnt++;
}

/* Update the upper and lower bounds for the potentiometer reading
 * Doesn't update directly as new min/max must appear the minimum threshold number of times
 * Checks for a new min or max for both 5V and 3V*/
void updateBounds(){
	//New 5V max
	if(adcDMA[0] > accel5VMax){
		//Meets threshold -> make max
		if(new5VMaxCnt >= minMaxOccurancesThresh){
			accel5VMax = adcDMA[0];
			new5VMaxCnt = 0;
		}
		//Doesn't meet threshold -> update count
		else{
			new5VMaxCnt++;
		}
		new5VMinCnt = 0;
	}
	//New 5V min
	else if(adcDMA[0] < accel5VMin){
		//Meets threshold -> make min
		if(new5VMinCnt >= minMaxOccurancesThresh){
			accel5VMin = adcDMA[0];
			new5VMinCnt = 0;
		}
		//Doesn't meet threshold -> update count
		else{
			new5VMinCnt++;
		}
		new5VMaxCnt = 0;
	}
	//Neither new min or max -> reset counters
	else{
		new5VMinCnt = 0;
		new5VMaxCnt = 0;
	}

	//New 3V max
	if(adcDMA[1] > accel3VMax){
		//Meets threshold -> make max
		if(new3VMaxCnt >= minMaxOccurancesThresh){
			accel3VMax = adcDMA[1];
			new3VMaxCnt = 0;
		}
		//Doesn't meet threshold -> update count
		else{
			new3VMaxCnt++;
		}
		new3VMinCnt = 0;
	}
	//New 3V min
	else if(adcDMA[1] < accel3VMin){
		//Meets threshold -> make min
		if(new3VMinCnt >= minMaxOccurancesThresh){
			accel3VMin = adcDMA[1];
			new3VMinCnt = 0;
		}
		//Doesn't meet threshold -> update count
		else{
			new3VMinCnt++;
		}
		new3VMaxCnt = 0;
	}
	//Neither new min or max -> reset counters
	else{
		new3VMinCnt = 0;
		new3VMaxCnt = 0;
	}
}

/* Uses the current max/min to calculate what should be the min/max used
 * Created a upper and lower deadzone respective to the given deadzone threshold
 	 * Anything above the max thresh is automatically 100
 	 * Anything below the min thresh is automatically 0
 * Used to reduce any mechanical and environmental noise */
void calculateMaxMinThresholds(){
	//Calculates what is considered 1% and decrements or increments that to the max/min based on threshold
	uint16_t accelDifference = accel5VMax - accel5VMin;
	accel5VMaxThresh = accel5VMax - ((accelDifference * .01) * pedalDeadzoneThresh);
	accel5VMinThresh = accel5VMin + ((accelDifference * .01) * pedalDeadzoneThresh);

	accelDifference = accel3VMax - accel3VMin;
	accel3VMaxThresh = accel3VMax - ((accelDifference * .01) * pedalDeadzoneThresh);
	accel3VMinThresh = accel3VMin + ((accelDifference * .01) * pedalDeadzoneThresh);
}

/* Verifies the percentage falls between 0-100
 * Anything below <0 -> 0 and above >100 -> 100 */
uint16_t checkPercentage(uint16_t percentage){
	if(percentage < 0){
		percentage = 0;
	}
	else if (percentage > 100){
		percentage = 100;
	}
	return percentage;
}

/*Converts the float percentage into an int(negative -> 0):
 * Used to cover the edge case where the calculated percentage is negative and
 * the conversion turns what should be 0 into a large integer value */
uint8_t convertToInt(float percentage){
	if (percentage < 0){
		return 0;
	}
	return (uint8_t)percentage;
}

/* Converts the current reading into a percentage out of 100 with the threshold min/max as boundaries */
void convertToPercentages(){

	//Calculate the percentage represented using threshold min & max
	float temp5V = (((float)(adcDMA[0] - accel5VMinThresh) / (float)(accel5VMaxThresh - accel5VMinThresh))*100);
	float temp3V = (((float)(adcDMA[1] - accel3VMinThresh) / (float)(accel3VMaxThresh - accel3VMinThresh))*100);

	//Save the new calculated percentages as integers
	accel5VPercentage = convertToInt(temp5V);
	accel3VPercentage = convertToInt(temp3V);

	//If falls in lower or upper threshold adjust to 0 or 100 respectively
	accel5VPercentage = checkPercentage(accel5VPercentage);
	accel3VPercentage = checkPercentage(accel3VPercentage);

	//Get the average of the two potentiometers
	accelAvgPercentage = (accel5VPercentage + accel3VPercentage)>>1;
}

/*Finds the absolute value of the given float number*/
uint8_t absValue(float value){
	if(value < 0){
		value *= -1;
	}
	return (uint8_t)value;
}

/* Checks for acceleration pedal implausibility
 * This is when the 5V and 3V readings have a difference larger than the allowed threshold
 * Automatically resets if the readings fall back within threshold and 500 milliseconds have passed */
void accelerationImplausability(){
	//Calculate the difference in 5V and 3V percentages
	accelImplausabilityPercentage = absValue(accel5VPercentage - accel3VPercentage);

	//If past the threshold then implausibility is true
	if(accelImplausabilityPercentage > pedalImplausibilityThresh){
		pedalImplausibilityBool = 1;
		accelerationImplausibilityTimer = HAL_GetTick();
	}
	//If belows threshold and 500 milliseconds have passed reset implausibility
	else if (pedalImplausibilityBool && (HAL_GetTick() - accelerationImplausibilityTimer > 500)){
		pedalImplausibilityBool = 0;
	}
}

/* Checks for brake pedal implausibility
 * This is when the the brake pedal is on while the acceleration pedal is pressed past 5 percent
 * Automatically resets if (the readings fall back within threshold OR brake is off) and 500 milliseconds have passed*/
void brakeImplausibility(){
	//Brakes applied with acceleration past threshold(5%)
	if(brakeBtn && accelAvgPercentage >= 5){
		brakeImplausibilityBool = 1;
		brakeImplausibilityTimer = HAL_GetTick();
	}
	//Back within threshold(%5)
	else if (brakeImplausibilityBool && (HAL_GetTick() - brakeImplausibilityTimer > 500) && (accelAvgPercentage < 5)){
		brakeImplausibilityBool = 0;
	}
	//Brakes no longer applied
	else if(brakeImplausibilityBool && !brakeBtn && (HAL_GetTick() - brakeImplausibilityTimer > 500)){
		brakeImplausibilityBool = 0;
	}
}

/* Sends a can message with information about implausibility and the avg percentage
 * TxData[0] = var1; -> implausibility bool
 * TxData[1] = var2; -> acceleration pedal percentage
 * Can be in one of two modes:
   	 * Testing: Use FDCAN_MODE_INTERNAL_LOOPBACK
 	 * Running: Change back to normal mode*/
void SendFDCANData(uint8_t var1, uint8_t var2) {
	//Build the data array
    //2 bytes: 2 x (uint8_t)
    uint8_t TxData[2];
    TxData[0] = var1;
    TxData[1] = var2;

    //Configure FDCAN Tx Header
    FDCAN_TxHeaderTypeDef TxHeader;
    TxHeader.Identifier = CANId;
    TxHeader.IdType = FDCAN_STANDARD_ID;
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.DataLength = FDCAN_DLC_BYTES_2;
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;

    //Check that the tx is free to send the message
    uint32_t txFreeLevel;
    txFreeLevel = HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1);
    if (txFreeLevel > 0) {

    	//Transmit CAN message and check if error occurred
        if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData) != HAL_OK) {
            msgSz = snprintf(msgBuff, buffSz, "Failed to send CAN MSG: %d \r\n", CANErrorMsgCnt);
            HAL_UART_Transmit(&huart2, (uint8_t*)msgBuff, msgSz, HAL_MAX_DELAY);
            CANErrorMsgCnt++;
        }
        //Message send successfully
        else {
            CANErrorMsgCnt = 0;
            msgSz = snprintf(msgBuff, buffSz, "Sent CAN MSG\r\n");
            HAL_UART_Transmit(&huart2, (uint8_t*)msgBuff, msgSz, HAL_MAX_DELAY);
        }
    }
    //Tx is not free, FIFO is full -> send error message
    else {
        msgSz = snprintf(msgBuff, buffSz, "Tx FIFO full, cannot send message\r\n");
        HAL_UART_Transmit(&huart2, (uint8_t*)msgBuff, msgSz, HAL_MAX_DELAY);
    }
}

/*Used for debugging with CAN on internal loop back mode to read our own CAN messages*/
void Receive_FDCAN_Message(void) {
    FDCAN_RxHeaderTypeDef RxHeader;
    uint8_t RxData[2];

    //Get the current tick time to track how long we wait for CAN message to be available
    uint32_t timeWaiting = HAL_GetTick();

    //Wait for a new CAN message or until 3 seconds pass
    while ((HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) == 0) && (HAL_GetTick()-timeWaiting < 3000));

    //Get the message and parse it
    if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK) {
        uint8_t receivedImplausibility = RxData[0];
        uint8_t receivedPercentage = RxData[1];

        msgSz = snprintf(msgBuff, buffSz, "Received CAN: Implausibility");
		HAL_UART_Transmit(&huart2, (uint8_t*)msgBuff, msgSz, HAL_MAX_DELAY);
		msgSz = snprintf(msgBuff, buffSz, ": %d, Percentage: %d\n\n\r\n", receivedImplausibility, receivedPercentage);
		HAL_UART_Transmit(&huart2, (uint8_t*)msgBuff, msgSz, HAL_MAX_DELAY);
    }
    //Failed to receive message and send error message
    else {
    	msgSz = snprintf(msgBuff, buffSz, "Failed to read CAN MSG\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*)msgBuff, msgSz, HAL_MAX_DELAY);
    }
}

/*Checks for CAN errors
 * Resets the FDCAN peripheral if an error exists
 * Checks if the CAN Bus is active*/
void checkCANErrors(){
	uint32_t errorStatus = HAL_FDCAN_GetError(&hfdcan1);

	//Check for the FDCAN peripheral error
	if (errorStatus != 0) {
	    msgSz = snprintf(msgBuff, buffSz, "FDCAN Error: 0x%08lX  %d\r\n", errorStatus, msgCnt);
	    HAL_UART_Transmit(&huart2, (uint8_t*)msgBuff, msgSz, HAL_MAX_DELAY);

	    //Deinitialize and Reinitialize the FDCAN peripheral
	    HAL_FDCAN_DeInit(&hfdcan1);
	    HAL_FDCAN_Init(&hfdcan1);
	}

	//Check if the CAN BUS is in the off state
	if (__HAL_FDCAN_GET_FLAG(&hfdcan1, FDCAN_FLAG_BUS_OFF)) {
	    msgSz = snprintf(msgBuff, buffSz, "FDCAN Bus Off state!\r\n");
	    HAL_UART_Transmit(&huart2, (uint8_t*)msgBuff, msgSz, HAL_MAX_DELAY);
	}
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
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_FDCAN1_Init();
  /* USER CODE BEGIN 2 */

  //Start the FDCAN to be able to start communication
  HAL_FDCAN_Start(&hfdcan1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  readInputs();
	  updateBounds();
	  calculateMaxMinThresholds();
	  convertToPercentages();
	  accelerationImplausability();
	  brakeImplausibility();

	  //If implausibility is true or testing/calibration mode is on then sent CAN 0
	  if(!calibrationModeBool && !pedalImplausibilityBool && !brakeImplausibilityBool && !testingModeBool){
		  implausibility = 0;
		  accelCANPercentage = accelAvgPercentage;
	  }
	  else{
		  accelCANPercentage = 0;
		  implausibility = 1;
	  }

	  printValues(); //For testing/debugging

	  //Must be in FDCAN_MODE_INTERNAL_LOOPBACK for testing & normal mode for actual use
	  SendFDCANData(implausibility, accelCANPercentage);
	  checkCANErrors();
	  Receive_FDCAN_Message(); //For testing/debugging

	  HAL_Delay(100); //For testing/debugging


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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.LowPowerAutoPowerOff = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_1CYCLE_5;
  hadc1.Init.SamplingTimeCommon2 = ADC_SAMPLETIME_1CYCLE_5;
  hadc1.Init.OversamplingMode = ENABLE;
  hadc1.Init.Oversampling.Ratio = ADC_OVERSAMPLING_RATIO_64;
  hadc1.Init.Oversampling.RightBitShift = ADC_RIGHTBITSHIFT_6;
  hadc1.Init.Oversampling.TriggeredMode = ADC_TRIGGEREDMODE_SINGLE_TRIGGER;
  hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_INTERNAL_LOOPBACK;
  hfdcan1.Init.AutoRetransmission = DISABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 16;
  hfdcan1.Init.NominalSyncJumpWidth = 1;
  hfdcan1.Init.NominalTimeSeg1 = 2;
  hfdcan1.Init.NominalTimeSeg2 = 2;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 1;
  hfdcan1.Init.DataTimeSeg2 = 1;
  hfdcan1.Init.StdFiltersNbr = 0;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* USER CODE END FDCAN1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : Btn_Pin */
  GPIO_InitStruct.Pin = Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : Btn2_Pin Btn1_Pin */
  GPIO_InitStruct.Pin = Btn2_Pin|Btn1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc){

	adcConvComp = 1;

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
