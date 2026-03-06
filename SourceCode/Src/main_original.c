/*USER CODE BEGIN Header */
/**
 ******************************************************************************
 *@file           : main.c
 *@brief          : Main program body
 ******************************************************************************
 *@attention
 *
 *Copyright (c) 2024 STMicroelectronics.
 *All rights reserved.
 *
 *This software is licensed under terms that can be found in the LICENSE file
 *in the root directory of this software component.
 *If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

/*USER CODE END Header */
/*Includes ------------------------------------------------------------------*/
#include "main.h"
#include "helper.h"
//#include <stdarg.h>
//#include <stdio.h>
//#include <stdio.h>
//#include <string.h>
//#include <ctype.h>


//#include "stm32c0xx_ll_dma.h"
//#include "stm32c0xx_hal.h"
//#include "stm32c0xx_hal_uart.h"
//#include "stm32c0xx_hal_dma.h"

/*Private includes ----------------------------------------------------------*/
/*USER CODE BEGIN Includes */

/*USER CODE END Includes */

/*Private typedef -----------------------------------------------------------*/
/*USER CODE BEGIN PTD */

/*USER CODE END PTD */

/*Private define ------------------------------------------------------------*/
/*USER CODE BEGIN PD */

/*USER CODE END PD */

/*Private macro -------------------------------------------------------------*/
/*USER CODE BEGIN PM */

/*USER CODE END PM */

/*Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_i2c1_rx;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/*USER CODE BEGIN PV */
int computeEncoderCounterPeriod();

/*USER CODE END PV */

/*Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC1_Init(void);

/*USER CODE BEGIN PFP */
TIM_HandleTypeDef htim1;	// Assuming TIM1 is used



//#define strcmp_embedded strcmp
int strcmp_embedded(const char *str1, const char *str2) {
    // Compare characters until they differ or until we reach the end of either string
    while (*str1 && *str2 && *str1 == *str2) {
        str1++;
        str2++;
    }

    // Return the difference between the two differing characters
    return (unsigned char)(*str1) - (unsigned char)(*str2);
}

/*USER CODE END PFP */

/*Private user code ---------------------------------------------------------*/
/*USER CODE BEGIN 0 */


#define I2C_BUFFER_SIZE 64
uint8_t i2cRxBuffer[I2C_BUFFER_SIZE];
uint8_t i2cTxBuffer[I2C_BUFFER_SIZE]; // Sample response data



#define DMA_BUFFER_SIZE 2048
uint8_t rxBuffer[DMA_BUFFER_SIZE]  __attribute__((section(".bss")));	// Define the buffer to store received data
//uint8_t txBuffer[] = "Hello, DMA UART TX!";	// Define the buffer to be transmitted



//char i2cTxBuffer[BUFFER_SIZE];  // The buffer array
int head = 0;                   // Index for the next write position
int tail = 0;                   // Index for the next read position
int is_full = 0;           // Flag to indicate if the buffer is full

// Insert a character into the cyclic buffer
void buffer_insert(char c) {

	is_full = buffer_is_full();

    i2cTxBuffer[head] = c;  // Write the character at the head position
    head = (head + 1) % I2C_BUFFER_SIZE;  // Advance the head

    // If the buffer is full, advance the tail to overwrite the oldest character
    if (is_full == 1) {
        tail = (tail + 1) % I2C_BUFFER_SIZE;
    }

    // Set the full flag if the head meets the tail
    if(head == tail)
    	is_full = 1;
}

// Remove a character from the cyclic buffer
int buffer_remove(char *c) {
    if (head == tail && is_full == 0) {
        // Buffer is empty
        return 0;
    }

    *c = i2cTxBuffer[tail];  // Read the character at the tail position
    tail = (tail + 1) % I2C_BUFFER_SIZE;  // Advance the tail
    is_full = 0;  // Buffer is no longer full after removing an element
    return 1;
}

// Check if the buffer is empty
int buffer_is_empty() {
    return (head == tail && is_full == 0);
}

// Check if the buffer is full
int buffer_is_full() {
    return is_full;
}


/**
  * @brief  Transmit in slave mode an amount of data in non-blocking mode with Interrupt
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @retval HAL status
  */


int dmaTransmissionComplete = 1;
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART2)
	{
		dmaTransmissionComplete = 1;
		// Transmission complete callback
		// You can notify that the transmission is complete or take other actions
	}
}



/*
static void UART_SendString(char *string, int lenght)
{
	do {HAL_Delay(1);}
	while (dmaTransmissionComplete == 0);

	dmaTransmissionComplete = 0;
	if (HAL_UART_Transmit_DMA(&huart2, string, lenght) != HAL_OK)
	{
		dmaTransmissionComplete = 1;
		// Handle error
		Error_Handler();
	}

	//for interrupt uart
	//  HAL_UART_Transmit(&huart2, (uint8_t*) string, lenght, HAL_MAX_DELAY);
}
*/

//static void printBuffer(const char *string,uint16_t length)
//{
//        HAL_UART_Transmit(&huart2, (uint8_t *)string, length, HAL_MAX_DELAY);
//}

//static uint8_t index_b = 0; //index to read first
//uint32_t size_bb = 20; //size
//uint8_t i2cTxBuffer[I2C_BUFFER_SIZE] = "1234567890abcdefghijklmnoprstyx"; // Sample response data


#define I2C  1
#define UART 0
int peripheral = UART;

static void printString(const char *string)
{
	while (*string)
    {
		if(peripheral == UART)
			HAL_UART_Transmit(&huart2, (uint8_t *)string++, 1, HAL_MAX_DELAY);
		else if(peripheral == I2C)
			buffer_insert(string++);
    }
}


static void printBuffer(const uint8_t *pData, uint16_t Size)
{
	if(peripheral == UART)
	{
		HAL_UART_Transmit(&huart2, pData, Size, HAL_MAX_DELAY);
	}
	else if(peripheral == I2C)
	{
		for(int i=0;i<Size;i++)
		{
			buffer_insert((char)pData[i]);
		}
	}
}

static void printChar(char ch)
{

	if(peripheral == UART)
	{
	    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
	}
	else if(peripheral == I2C)
	{
		buffer_insert(ch);
	}
}

static void printUInt64(uint64_t value)
{
    char buffer[21];       // Buffer large enough for a 64-bit integer string with null terminator
    int index = 20;        // Start filling from the last position

    buffer[index] = '\0';  // Null terminator at the end of the buffer
    index--;

    // Convert the number to a string by extracting digits from the end
    do {
        buffer[index] = '0' + (value % 10);  // Get the last digit
        value /= 10;                         // Move to the next digit
        index--;                             // Move left in the buffer
    } while (value > 0);

    // Print the resulting string starting from the first non-empty position
    printString(&buffer[index + 1]);
}

static void printInt64(int64_t value)
{
    char buffer[21];       // Buffer large enough for a 64-bit integer string with null terminator
    int index = 20;        // Start filling from the last position
    int isNegative = value < 0;

    buffer[index] = '\0';  // Null terminator at the end of the buffer
    index--;

    // Special handling for INT64_MIN because it cannot be directly negated
    if (value == INT64_MIN) {
        value = INT64_MAX;     // Use INT64_MAX temporarily
        buffer[index] = '8';   // Manually set last digit for INT64_MIN
        index--;
    }
    else if (isNegative) {
        value = -value;        // Convert to positive if the value is negative
    }

    // Convert the number to a string by extracting digits from the end
    do {
        buffer[index] = '0' + (value % 10);  // Get the last digit
        value /= 10;                         // Move to the next digit
        index--;                             // Move left in the buffer
    } while (value > 0);

    // Add a negative sign if the original number was negative
    if (isNegative)
    {
        buffer[index] = '-';
        index--;
    }

    // Print the resulting string starting from the first non-empty position
    printString(&buffer[index + 1]);
}

static void printFloat(float value, int decimalPlaces)
{
    //char buffer[20];  // Buffer for the formatted string
    int integerPart = (int)value;
    float fractionalPart = value - (float)integerPart;

    if (value < 0)
    {
        //UART_SendChar('-');
        printChar('-');
        integerPart = -integerPart;
        fractionalPart = -fractionalPart;
    }

    // Send the integer part
    printInt64(integerPart);
    //UART_SendInt64(integerPart);

    // Send the decimal point
    //UART_SendChar('.');
    printChar('.');

    // Send the fractional part up to the specified decimal places
    for (int i = 0; i < decimalPlaces; i++)
    {
        fractionalPart *= 10;
        int digit = (int)fractionalPart;
        printChar('0' + digit);
        fractionalPart -= digit;
    }
}




static void UpdatePWMDutyCycle(uint32_t channel, uint32_t value)
{
	TIM_OC_InitTypeDef sConfigOC ;

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = value;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	HAL_TIM_PWM_ConfigChannel_optimized(&htim1, &sConfigOC, channel);
	HAL_TIM_PWM_Start_optimized(&htim1, channel);	// Start PWM
}



volatile int64_t full_encoder_count = 0;	// 32-bit counter
volatile uint32_t tim_counter = 0;

//DMA1_Channel1_IRQHandler
//void DMA1_Channel1_IRQHandler(void)
//{
//	UART_SendFormattedString("DMA1_Channel1_IRQHandler\n");
//
//    HAL_DMA_IRQHandler(&hdma_usart2_rx);
//}

//doesnt get fired
//void USART2_IRQHandler(void)
//{
//	UART_SendFormattedString("USART2_IRQHandler\n");
//}

volatile uint8_t halfBufferReceived = 0;
volatile uint8_t fullBufferReceived = 0;



void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	//if (huart->Instance == USART2)
	//{

		//__HAL_UART_CLEAR_FLAG(&huart2, UART_FLAG_ORE | UART_FLAG_NE | UART_FLAG_FE | UART_FLAG_PE);
		//HAL_UART_DMAStop(&huart2);
		if (HAL_UART_Receive_DMA(&huart2, rxBuffer, DMA_BUFFER_SIZE) != HAL_OK)
		{
			//	UART_SendFormattedString("ERROR: Failed to restart DMA reception\n");
		}
//	}
}


#define ABS(number) (number < 0) ? -number : number;

//period of logging pos, amps, tps
uint32_t logPeriod = 0;
uint32_t confirm = 0;

// Define constraints
float V_max = 20.0f;    // Max velocity
float A_max = 0.05f;    // Max acceleration
float controlInterval = 0.01f; // Control interval in seconds (10 ms)

int64_t startPos, targetPos;
int64_t setpointPos;
int64_t totalDistance;
int64_t D_accel;
float T_accel, T_const, T_total;
int64_t direction = 1;
int8_t trapezoid = 0;


float currentTime = 0.0f;     // Current time in seconds


// Initialize trajectory parameters
void initializeTrajectory(int64_t start, int64_t target) {
    startPos = start;
    targetPos = target;
    setpointPos = startPos;
    totalDistance = targetPos - startPos;
    if(totalDistance<0)
    	totalDistance = -totalDistance;



    // Precompute parameters for acceleration
    D_accel = (int64_t)((V_max * V_max) / (2.0f * A_max));

    int64_t adjusted_D_accel = -1;

    if (totalDistance < 2 * D_accel) {
        // Triangular profile: max velocity is not reached
    	trapezoid = 0;
        adjusted_D_accel = totalDistance / 2;
        T_accel = custom_sqrtf(2.0f * adjusted_D_accel / A_max);
        D_accel = adjusted_D_accel;
        T_const = 0.0f; // No constant velocity phase
    } else {
        // Trapezoidal profile
    	trapezoid = 1;
        T_accel = V_max / A_max;
        T_const = (totalDistance - 2 * D_accel) / V_max;
    }

    // Total time for the trajectory
    T_total = 2.0f * T_accel + T_const;

    printFloat(T_accel,4);
        printChar(' ');
    printFloat(T_const,4);
    printChar(' ');
    printFloat(adjusted_D_accel,4);
    printChar(' ');

    currentTime = 0.0f;     // Current time in seconds

    direction = (targetPos - startPos)>0?(1):(-1);
}



int64_t get_full_encoder_count();

// Update trajectory
void updateTrajectory(float dt)
{
	currentTime += dt;

    if (currentTime <= T_accel) {
        // Acceleration phase
        setpointPos = startPos + direction*(int64_t)((A_max * currentTime * currentTime) / 2.0f);
    } else if (currentTime <= T_accel + T_const && trapezoid ==1) {
        // Constant velocity phase
        setpointPos = startPos + direction*D_accel + direction*(int64_t)(V_max * (currentTime - T_accel));
    }
    else if (currentTime <= T_total) {
        // Deceleration phase
        float t_decel = currentTime - (T_accel + T_const); // Time elapsed in deceleration phase

        // Ensure deceleration distance formula is consistent
        float d_decel = V_max * t_decel - 0.5f * A_max * t_decel * t_decel;

        // Compute position considering all phases
        setpointPos = (int64_t)(startPos
            + direction * D_accel
            + direction * (T_const * V_max)
            + direction * d_decel);
    }

    else {
        // Trajectory complete
        setpointPos = targetPos;
    }
}


int64_t get_full_encoder_count()
{
	int64_t count;
    //__disable_irq(); // Disable interrupts
    count = full_encoder_count + __HAL_TIM_GET_COUNTER(&htim3) - 65536;
    //__enable_irq();  // Re-enable interrupts
    return count;
}

//int32_t signedCounter = tim_counter_;
//int64_t largeCoutner = signedCounter ;//+ full_encoder_count;

// Define PID constants
float Kp = 0.0001;      // Proportional gain
float Ki = 0.0;      // Integral gain
float Kd = 0.0;     // Derivative gain

// Define PID variables
static int64_t previousError = 0;
static int64_t integral = 0;

int64_t targetPos = 0;

#define INTEGRAL_MAX 100

void update_PID()
{
    // Get the current position and cast it to int64_t for correct signed calculation
	int64_t currentPos = get_full_encoder_count();

    // Calculate error as a signed difference
	int64_t error = setpointPos - currentPos;

	// Calculate integral (limit to prevent windup)
	integral += error;
	if (integral > INTEGRAL_MAX) integral = INTEGRAL_MAX;
	if (integral < -INTEGRAL_MAX) integral = -INTEGRAL_MAX;

	// Calculate derivative
	int64_t derivative = error - previousError;

	// Calculate PID output
	double output = (int64_t)(Kp * error + Ki * integral + Kd * derivative)/10;

	// Update the previous error
	previousError = error;

	// Set the PWM duty cycle based on PID output
	// Offset from the center and constrain to valid range
	uint64_t pwmDutyCycle = output;  //htim1.Init.Period / 2 + ;

	if(output < 0)
		pwmDutyCycle = -output;

	if (pwmDutyCycle > htim1.Init.Period) pwmDutyCycle = htim1.Init.Period;
	if (pwmDutyCycle < 0) pwmDutyCycle = 0;

	// Apply the output to the H-bridge channels for direction and duty cycle
	if (output > 0) {
		UpdatePWMDutyCycle(TIM_CHANNEL_3, pwmDutyCycle); // Forward
		UpdatePWMDutyCycle(TIM_CHANNEL_4, 0);
	} else {
		UpdatePWMDutyCycle(TIM_CHANNEL_3, 0);
		UpdatePWMDutyCycle(TIM_CHANNEL_4, pwmDutyCycle); // Reverse
	}
}


uint32_t readADCValue()
{
    uint32_t adcValue = 0;

    // Start the ADC conversion
    HAL_ADC_Start(&hadc1);

    // Poll for end of conversion with a timeout
    if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK)
    {
        // Get the converted value
        adcValue = HAL_ADC_GetValue(&hadc1);
    }

    // Stop the ADC to save power
    HAL_ADC_Stop(&hadc1);

    return adcValue;
}

void handleCommand(uint8_t *buffer, int bufferSize, uint16_t commandIndex);

int driveMode = 0;
#define MOTOR_FLOAT 			0
#define MOTOR_BRAKE 			1
#define MOTOR_PID 		2
#define MOTOR_TRAPEZOID 	3


void ProcessCommand(uint8_t *buffer_, int bufferSize, DMA_HandleTypeDef *hdmarx, uint16_t *lastParsedIndex_)
{
	// Continuously scan the DMA buffer for new '\n' characters
	while (*lastParsedIndex_ != (bufferSize - __HAL_DMA_GET_COUNTER(hdmarx)))
	{
		if (buffer_[*lastParsedIndex_] == '\n')
		{
			handleCommand(buffer_, bufferSize, *lastParsedIndex_);
		}

		// Move to the next index and wrap around if necessary
		*lastParsedIndex_ = (*lastParsedIndex_ + 1) % bufferSize;
	}
}


int atoi_embedded(const char *str) {
    // Initialize variables
    int result = 0;
    int sign = 1; // Assume positive by default

    // Skip leading whitespace
    while (*str == ' ' || *str == '\t' || *str == '\n' || *str == '\r' || *str == '\f' || *str == '\v') {
        str++;
    }

    // Check for optional sign
    if (*str == '-') {
        sign = -1;
        str++;
    } else if (*str == '+') {
        str++;
    }

    // Process each character
    while (*str >= '0' && *str <= '9') {
        // Accumulate the digit into the result
        result = result * 10 + (*str - '0');
        str++;
    }

    // Return result with the appropriate sign
    return result * sign;
}

#define MAX_COMMAND_LENGTH 25
void handleCommand(uint8_t *buffer, int bufferSize, uint16_t commandIndex)
{
	char command[MAX_COMMAND_LENGTH];

	// Reconstruct command from the cyclic buffer in decrementing order
	for (int i = 0; i < MAX_COMMAND_LENGTH; i++)
	{
		int buffer_i = (commandIndex - i);
		if (buffer_i < 0)
			buffer_i = bufferSize - buffer_i;

		command[MAX_COMMAND_LENGTH - i - 1] = buffer[buffer_i];
	}

	char commandString[MAX_COMMAND_LENGTH] ;
	char valueString[MAX_COMMAND_LENGTH] ;

	extractKeywordAndValue(command, MAX_COMMAND_LENGTH, commandString, valueString);

	if (strcmp_embedded(commandString, "SETSPD") == 0 && valueString[0] != '\0')
	{
		int speed = atoi_embedded(valueString);
		if (speed >= 0 && speed <= 100)
		{
			//printFormattedString("Speed %d%%\n", speed);
			printString("Speed ");
			printInt64(speed);
			printChar('\n');
			// Set motor speed to 'speed'
		}
		else
		{
			//		UART_SendFormattedString("ERROR: Invalid speed value\n");
		}
	}
	else if (strcmp_embedded(commandString, "SETPOS") == 0 && valueString[0] != '\0')
	{
		targetPos = atoi_embedded(valueString);
		printString("Pos ");
		printString(valueString);
		printChar('\n');
		driveMode = MOTOR_PID;

		//		int position = atoi(valueString);
		//	UART_SendFormattedString("Position set to %d\n", position);
		// Set motor position to 'position'
	}
	else if (strcmp_embedded(commandString, "FLOAT") == 0)
	{
		//	UART_SendFormattedString("H-Bridge set to floating state\n");
		// Set H-Bridge to floating state
		driveMode = MOTOR_FLOAT;
		UpdatePWMDutyCycle(TIM_CHANNEL_3, 0);
		UpdatePWMDutyCycle(TIM_CHANNEL_4, 0);
		//printString("OK_FLOAT");
	}
	else if (strcmp_embedded(commandString, "BRAKE") == 0)
	{
		//	UART_SendFormattedString("H-Bridge set to brake state\n");
		// Set H-Bridge to brake state
		driveMode = MOTOR_BRAKE;
		UpdatePWMDutyCycle(TIM_CHANNEL_3, htim1.Init.Period/2);
		UpdatePWMDutyCycle(TIM_CHANNEL_4, htim1.Init.Period/2);
		//printString("OK_BREAK");
	}
	else if (strcmp_embedded(commandString, "GETSPD") == 0)
	{
		//printString("Speed: ");
		printFloat(computeEncoderCounterPeriod(),4);
		printChar('\n');
		// Replace with actual speed value retrieval
		//	UART_SendFormattedString("Current speed: 50%%\n");
	}

	else if (strcmp_embedded(commandString, "GETPOS") == 0)
	{
		// Replace with actual position value retrieval
		//UART_SendFormattedString("Current position: 1000\n");

		tim_counter = __HAL_TIM_GET_COUNTER(&htim3);
		int64_t t = get_full_encoder_count();

		if(confirm)
		{
			//printString("Pos:");
			//printInt64(t);
			//printChar('\n');
		}
	}
	else if (strcmp_embedded(commandString, "STATUS") == 0)
	{
		// Replace with actual status retrieval
		//	UART_SendFormattedString("Motor status: running\n");
		if(driveMode == MOTOR_BRAKE)
		{
			//printString("STATUS: MOTOR_BRAKE");
		}
		else if(driveMode == MOTOR_FLOAT)
		{
			//printString("STATUS: MOTOR_FLOAT");
		}
		else if(driveMode == MOTOR_PID)
		{
		//	printString("STATUS: MOTOR_PID");
		}
		else if(driveMode == MOTOR_TRAPEZOID)
		{
			//printString("STATUS: MOTOR_TRAPEZOID");
		}
		else
		{
			//printString("STATUS: ERROR");
		}
	}
	else if (strcmp_embedded(commandString, "SET_P") == 0 && valueString[0] != '\0')
	{
		Kp = ratof(valueString); //this eats up so much memory

		if(confirm)
		{
			//printString("P is set to ");
			//printFloat(Kp,7);
			//printChar('\n');
		}
	}
	else if (strcmp_embedded(commandString, "SET_I") == 0 && valueString[0] != '\0')
	{
		Ki = ratof(valueString); //this eats up so much memory


		if(confirm)
		{
			//printString("I is set to ");
			//printFloat(Ki,7);
			//printChar('\n');
		}
	}
	else if (strcmp_embedded(commandString, "SET_D") == 0 && valueString[0] != '\0')
	{
		Kd = ratof(valueString); //this eats up so much memory


		if(confirm)
		{
			//printString("D is set to ");
			//printFloat(Kd,7);
			//printChar('\n');
		}
	}
	else if (strcmp_embedded(commandString, "GET_PID") == 0)
	{
		printString("Kp ");
		printFloat(Kp,7);
		printString(",Ki ");
		printFloat(Ki,7);
		printString(",Kd ");
		printFloat(Kd,7);
		printChar('\n');
	}
	else if (strcmp_embedded(commandString, "GET_ADC") == 0)
	{
		// VIPROPI (V) = IPROPI (A) x RIPROPI (1.5K Ω)
		//adc range: 0 - 4095

		printString("ADC:");
		uint32_t t = readADCValue();

		float Vipropi = ((float)t*(3.3f/4095.0f));

		float Ipropi = Vipropi/1.5f;

		printFloat(Ipropi,3);
		printString(" mA\n");
	}
	else if (strcmp_embedded(commandString, "MOVETO") == 0 && valueString[0] != '\0')
	{
		driveMode = MOTOR_TRAPEZOID;
		targetPos = atoi_embedded(valueString);


		if(confirm)
		{
			printString("Moving to ");
			printString(valueString);
			printChar('\n');
		}

		startPos = get_full_encoder_count();
		initializeTrajectory(startPos, targetPos);
	}
	else if (strcmp_embedded(commandString, "LOG") == 0 && valueString[0] != '\0')
	{
		logPeriod = atoi_embedded(valueString);

		if(confirm)
		{
			printString("logPeriod ");
			printString(valueString);
			printChar('\n');
		}
	}
	else if (strcmp_embedded(commandString, "CONFIRM") == 0 && valueString[0] != '\0')
	{
		confirm = atoi_embedded(valueString);

		if(confirm)
		{
			printString("logPeriod ");
			printString(valueString);
			printChar('\n');
		}
	}
	else if (strcmp_embedded(commandString, "SETMAX_VEL") == 0 && valueString[0] != '\0')
	{
		V_max = ratof(valueString); //this eats up so much memory
		//printString("Max Vel is set to");
		//printFloat(V_max,7);
		//printChar('\n');
	}
	else if (strcmp_embedded(commandString, "SETMAX_ACC") == 0 && valueString[0] != '\0')
	{
		A_max = ratof(valueString); //this eats up so much memory
		//printString("Max Acc is set to");
		//printFloat(A_max,7);
		//printChar('\n');
	}
	else if (strcmp_embedded(commandString, "GETMAX_VEL") == 0)
	{
		//printString("Max Vel:");
		//printFloat(V_max,7);
		//printChar('\n');
	}
	else if (strcmp_embedded(commandString, "GETMAX_ACC") == 0)
	{
		//printString("Max Acc:");
		//printFloat(A_max,7);
		//printChar('\n');
	}
	else if (strcmp_embedded(commandString, "RESET") == 0)
	{
		//printString("RESETTING...");
		//NVIC_SystemReset();
	}
}


int64_t encoder_lastPos;
int64_t encoder_currentPos;

uint32_t encoder_lastTick;
uint32_t encoder_currentTick;
uint32_t adcReader_lastTick;

int computeEncoderCounterPeriod()
{
	int32_t nCounts = encoder_lastPos - encoder_currentPos;
	int32_t nTicks = encoder_lastTick - encoder_currentTick;

	int32_t res = ((int32_t)nTicks/(int32_t)nCounts);
	return res;
}


//#define I2C_ISR_TXIS_Msk             (0x1UL << I2C_ISR_TXIS_Pos)               /*!< 0x00000002 */
//#define I2C_ISR_TXIS                 I2C_ISR_TXIS_Msk                          /*!< Transmit interrupt status */

void I2C_transmissionCallback(I2C_HandleTypeDef *hi2c)
{
    //uint8_t txBuffer[] = "testting!!";
    //I2C_TransmitData(txBuffer, sizeof(txBuffer));

	//volatile uint8_t *tex__ = "MAMAMAMAMAMAMAMAMAM";
	//volatile uint8_t *tex = "KEKEKEKEKEKEKEK";
	//hi2c->State = HAL_I2C_STATE_READY;
	//HAL_ERROR;

	//hi2c1.Instance->CR2 |= I2C_CR2_STOP;  // Set STOP condition
	//while (hi2c1.Instance->ISR & I2C_ISR_BUSY) {
	    // Wait until the BUSY flag is cleared
	//}

	//HAL_StatusTypeDef res = HAL_I2C_Slave_Transmit(hi2c, tex, 4, 5000);//millisecond
    //res = HAL_I2C_Slave_Transmit(hi2c, &(tex[1]), 1, 5000);//millisecond
    //res = HAL_I2C_Slave_Transmit(hi2c, &(tex[2]), 1, 5000);//millisecond
    //res = HAL_I2C_Slave_Transmit(hi2c, &(tex[3]), 1, 5000);//millisecond

    //printString("PP");
    //printUInt64(res);
    //printString("PP");

}

#define TIMEOUT_MS 100000  // Define a timeout duration in milliseconds
#define SYSTEM_CLOCK 16000000  // Define your system clock frequency


void I2C_TransmitData(uint8_t *data, uint16_t size)
{
    for (uint16_t i = 0; i < size; i++)
    {
        uint32_t startTick = 0;//HAL_GetTick();  // Record the current system tick

        // Wait for TXIS flag or timeout
        while (!__HAL_I2C_GET_FLAG(&hi2c1, I2C_FLAG_TXIS))
        {
        	//printString("While2\n");
        	startTick++;
            if (startTick > TIMEOUT_MS)
            {
                // Timeout occurred
                printString("Error: TXIS flag timeout!\n");
                return;  // Exit the function
            }
        }

        hi2c1.Instance->TXDR = data[i];  // Write data to the TXDR register
    }

    uint32_t startTick = HAL_GetTick();  // Record the current system tick

    // Wait for the STOPF flag or timeout
    while (!__HAL_I2C_GET_FLAG(&hi2c1, I2C_FLAG_STOPF))
    {
    	//printString("While1\n");
        if ((HAL_GetTick() - startTick) > TIMEOUT_MS)
        {
            // Timeout occurred
        	printString("STOPF flag timeout!\n");
            return;  // Exit the function
        }
    }

    __HAL_I2C_CLEAR_FLAG(&hi2c1, I2C_FLAG_STOPF);  // Clear STOPF flag
}












void I2C_IRQHandler_User(I2C_HandleTypeDef *hi2c)
{
    // Check if the relevant flags are set
    if (!__HAL_I2C_GET_FLAG(&hi2c1, I2C_FLAG_TXIS) && !__HAL_I2C_GET_FLAG(&hi2c1, I2C_FLAG_STOPF))
        return;

    //uint8_t i2cTxBuffer[I2C_BUFFER_SIZE]; // Sample response data

    const uint32_t TIMEOUT = 500;  // Timeout in milliseconds

    // Handle TXIS (Transmit Interrupt Status)
    while (!__HAL_I2C_GET_FLAG(&hi2c1, I2C_FLAG_STOPF))
    {
        uint32_t startTick = HAL_GetTick();  // Get the current system tick

        // Wait for TXIS flag with timeout
        while (!__HAL_I2C_GET_FLAG(&hi2c1, I2C_FLAG_TXIS))
        {
            if ((HAL_GetTick() - startTick) > TIMEOUT)
            {
  //              printString("Error: TXIS timeout.\n");
                return;  // Exit the function to avoid hanging
            }

            if(__HAL_I2C_GET_FLAG(&hi2c1, I2C_FLAG_STOPF))
            {
//            	printString("Error: STOP Flag.\n");

            	__HAL_I2C_CLEAR_FLAG(&hi2c1, I2C_FLAG_STOPF);  // Clear STOPF flag
                // Optional: If ADDR flag needs handling
                if (__HAL_I2C_GET_FLAG(&hi2c1, I2C_FLAG_ADDR))
                {
                    __HAL_I2C_CLEAR_FLAG(&hi2c1, I2C_FLAG_ADDR);  // Clear ADDR flag
                }
            	return;  // Exit the function to avoid hanging
            }
        }

        if(buffer_is_empty())
        {
        	hi2c1.Instance->TXDR = '\0';
        }
        else
        {
        	char removed_char;
        	buffer_remove(&removed_char);

			// Transmit the next byte
			hi2c1.Instance->TXDR = removed_char;
			//size_bb--;
        	//index_b++;
        	//if(index_b >= I2C_BUFFER_SIZE)
        	//{
        	//	index_b = 0;
        	//}

        }
    }

    // Handle STOPF (Stop Flag)
    if (__HAL_I2C_GET_FLAG(&hi2c1, I2C_FLAG_STOPF))
    {
//        printString("I2C_FLAG_STOPF detected.");

        //index_b = 0;  // Reset index for the next transmission

        __HAL_I2C_CLEAR_FLAG(&hi2c1, I2C_FLAG_STOPF);  // Clear STOPF flag

        // Optional: If ADDR flag needs handling
        if (__HAL_I2C_GET_FLAG(&hi2c1, I2C_FLAG_ADDR))
        {
            __HAL_I2C_CLEAR_FLAG(&hi2c1, I2C_FLAG_ADDR);  // Clear ADDR flag
        }
    }
}




	//printString("I2C_IRQHandler.\n");

	//if (__HAL_I2C_GET_FLAG(&hi2c1, I2C_FLAG_ADDR)) {
		//printString("I2C_FLAG_ADDR  .\n");
	//        	        __HAL_I2C_CLEAR_FLAG(&hi2c1, I2C_FLAG_ADDR);
	  //  }


	//if (__HAL_I2C_GET_FLAG(&hi2c1, I2C_FLAG_STOPF)) {
		//printString("I2C_FLAG_STOPF  .\n");
	//	__HAL_I2C_CLEAR_FLAG(&hi2c1, I2C_FLAG_STOPF);
//	}





/*USER CODE END 0 */


void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode)
{
//    printString("I2C_AddrCallback");

	//if (hi2c->Instance == I2C1) {
    //    printString("I2C_AddrCallback");
   // }
}


void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  //  if (hi2c->Instance == I2C1)
//    {
        // Process received data
        printString("I2C Received: ");
        printBuffer((char *)i2cRxBuffer, I2C_BUFFER_SIZE);

        // Restart I2C DMA reception
        if (HAL_I2C_Slave_Receive_DMA(hi2c, i2cRxBuffer, I2C_BUFFER_SIZE) != HAL_OK)
        {
        	//printString("Failed to restart I2C DMA Receive\n");
        }
    //}
}

#define BUFFER_SIZE 20  // Size of the cyclic buffer

// Cyclic buffer for uint32_t values
uint32_t currentBuffer[BUFFER_SIZE];
int bufferHead = 0;  // Index for the next write position
int bufferTail = 0;  // Index for the oldest value
int bufferCount = 0; // Track the number of valid elements in the buffer

// Insert a new value into the buffer
void ADCbuffer_insert(uint32_t newValue) {
    currentBuffer[bufferHead] = newValue;  // Overwrite the oldest value
    bufferHead = (bufferHead + 1) % BUFFER_SIZE;

    // If the buffer is full, move the tail to the next oldest value
    if (bufferCount < BUFFER_SIZE) {
        bufferCount++;
    } else {
        bufferTail = (bufferTail + 1) % BUFFER_SIZE;
    }
}

// Compute the filtered value using Simple Moving Average
float filter_sma() {
    uint64_t sum = 0;
    for (int i = 0; i < bufferCount; i++) {
        sum += currentBuffer[(bufferTail + i) % BUFFER_SIZE];
    }
    return (bufferCount > 0) ? (sum / bufferCount) : 0;
}


int main(void)
{

	/*USER CODE BEGIN 1 */

	/*USER CODE END 1 */

	/*MCU Configuration--------------------------------------------------------*/

	/*Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/*USER CODE BEGIN Init */

	/*USER CODE END Init */

	/*Configure the system clock */
	SystemClock_Config();

	/*USER CODE BEGIN SysInit */

	/*USER CODE END SysInit */

	/*Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_USART2_UART_Init();
	MX_I2C1_Init();
	//basicI2CInit();

	MX_TIM1_Init();
	MX_TIM3_Init();
	MX_ADC1_Init();

	//setupI2CReceive();


	/*USER CODE BEGIN 2 */
	//HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start_optimized_(&htim3);
	__enable_irq();
	/*USER CODE END 2 */

	/*HAL_StatusTypeDef uartResult = */HAL_UART_Receive_DMA(&huart2, rxBuffer, DMA_BUFFER_SIZE);

	/*if (uartResult == HAL_OK) {
	    	//printString("UART DMA receive setup successful.\n");
	    } else if (uartResult == HAL_ERROR) {
	    	//printString("UART DMA error: failed to initialize UART receive.\n");
	        // Additional error handling or retry logic can go here if needed
	    } else if (uartResult == HAL_BUSY) {
	    	//printString("UART DMA error: UART is busy.\n");
	        // Optionally add a retry mechanism if needed
	    } else if (uartResult == HAL_TIMEOUT) {
	    	//printString("UART DMA error: initialization timed out.\n");
	        // Handle timeout or retry as appropriate
	    }*/

	    HAL_Delay(1000);


	    HAL_I2C_DeInit(&hi2c1);  // Reset the I2C peripheral
	    HAL_Delay(100);

	    if (HAL_I2C_Init(&hi2c1) == HAL_OK) {
	        //printString("I2C reinitialized\n");
	    } //else {
	      //  printString("Failed to reinitialize I2C.\n");
	    //}

	    HAL_Delay(500);

	    while(hi2c1.State != HAL_I2C_STATE_READY)
	    {
		    //HAL_Delay(500);
		    //printString("I2C_STATE:");
		    //printUInt64(hi2c1.State);

	    }
	    //printString("HAL_I2C_STATE_READY");
	    HAL_Delay(500);


	    HAL_StatusTypeDef result = HAL_I2C_Slave_Receive_DMA(&hi2c1, i2cRxBuffer, I2C_BUFFER_SIZE);

	    //if (result == HAL_OK)
	    //{
	        //uint32_t error = HAL_I2C_GetError(&hi2c1);
	        //printString("HAL_OK.\n");
	    //}
	    //else if(result == HAL_BUSY)
	    //{
	        //printString("I2C HAL_BUSY.\n");
	    //}
	    //else
	    //{
	        //printString("I2C non-DMA receive setup success.\n");
	    //}
        //printString("I2C HAL Error Code.\n");

	    printUInt64(result);

	    HAL_Delay(100);

	    //__HAL_I2C_DISABLE_IT(&hi2c1, I2C_IT_ERRI | I2C_IT_TCI | I2C_IT_STOPI | I2C_IT_NACKI | I2C_IT_ADDRI | I2C_IT_RXI);
	    __HAL_I2C_DISABLE_IT(&hi2c1, I2C_IT_ERRI | I2C_IT_TCI | I2C_IT_NACKI | I2C_IT_RXI);
	    __HAL_I2C_DISABLE_IT(&hi2c1, I2C_IT_NACKI);

	    HAL_Delay(100);
	    __HAL_I2C_CLEAR_FLAG(&hi2c1, I2C_FLAG_AF);
		__HAL_I2C_CLEAR_FLAG(&hi2c1, I2C_FLAG_STOPF);
		__HAL_I2C_CLEAR_FLAG(&hi2c1, I2C_FLAG_OVR);
		__HAL_I2C_CLEAR_FLAG(&hi2c1, I2C_FLAG_BUSY);
		__HAL_I2C_CLEAR_FLAG(&hi2c1, I2C_FLAG_ALERT);

		__HAL_I2C_CLEAR_FLAG(&hi2c1, I2C_FLAG_AF | I2C_FLAG_STOPF | I2C_FLAG_OVR | I2C_FLAG_BERR);

		__HAL_I2C_ENABLE_IT(&hi2c1,I2C_IT_STOPI);
		__HAL_I2C_ENABLE_IT(&hi2c1,I2C_IT_ADDRI);
		__HAL_I2C_ENABLE_IT(&hi2c1,I2C_IT_TXI);



	    //printString("HAL_I2C_Interrupt_Enabled\n");

	/*Infinite loop */
	/*USER CODE BEGIN WHILE */
	//printString("READY\n");

	encoder_lastPos = get_full_encoder_count();
	encoder_lastTick = HAL_GetTick();
	adcReader_lastTick = HAL_GetTick();

	uint32_t frame_lastTick = HAL_GetTick();
	uint32_t frame_Tick = HAL_GetTick();

	while(1)
	{
		static uint16_t lastParsedIndex_uart = 0;	// Tracks the last parsed index in the DMA buffer
		ProcessCommand(rxBuffer, DMA_BUFFER_SIZE, (huart2.hdmarx), &lastParsedIndex_uart);


		static uint16_t lastParsedIndex_i2c = 0;	// Tracks the last parsed index in the DMA buffer
		ProcessCommand(i2cRxBuffer, I2C_BUFFER_SIZE, (hi2c1.hdmarx), &lastParsedIndex_i2c);
		__HAL_I2C_CLEAR_FLAG(&hi2c1, I2C_FLAG_AF | I2C_FLAG_STOPF | I2C_FLAG_OVR | I2C_FLAG_BUSY);




		encoder_currentPos = get_full_encoder_count();
		encoder_currentTick = HAL_GetTick();


		if(driveMode == MOTOR_TRAPEZOID)
		{

			frame_lastTick = frame_Tick;
			frame_Tick = HAL_GetTick();

			float dt = (frame_Tick - frame_lastTick)/1000.0f;

			updateTrajectory(dt);

			update_PID();
		}

		if(driveMode == MOTOR_PID)
		{
	        setpointPos = targetPos;
			update_PID();
		}



		if(logPeriod != 0)
		{
			if(logPeriod < 20)
			{
				logPeriod = 20;
			}

			static int64_t prev_encoderCount = 0;
			if(HAL_GetTick() - adcReader_lastTick > 80)
			{
				ADCbuffer_insert(readADCValue());
				adcReader_lastTick = HAL_GetTick();
			}

			if(HAL_GetTick() - encoder_lastTick > logPeriod)
			{
				int64_t encoder_currentPos = get_full_encoder_count();



				float filtered = filter_sma();
				printString("pos ");
				printInt64(encoder_currentPos);


				// VIPROPI (V) = IPROPI (A) x RIPROPI (1.5K Ω)
				//adc range: 0 - 4095

		//		printString("ADC:");
	//			uint32_t t = readADCValue();

				float Vipropi = (filtered*(3.3f/4095.0f));
				float Ipropi = Vipropi/1.5f;

				//printFloat(Ipropi,3);
				//printString(" mA\n");

				printString(",amps ");
				printFloat(Ipropi,3);


				//rpm computation:
				uint32_t deltaTick = HAL_GetTick() - encoder_lastTick;
				int64_t encoderCount = get_full_encoder_count();
				int64_t deltaCount = (int32_t)(encoderCount - prev_encoderCount);

				int64_t rpm = 0;
				if(deltaTick == 0)
				{
					rpm = 99999999;
				}
				else if(deltaTick > 1000)
				{
					rpm = 0;
				}
				else
				{
					rpm = deltaCount*1000/deltaTick;
				}

				prev_encoderCount = encoderCount;

				printString(",tps ");
				printInt64(rpm);
				printString("\n");

				//peripheral = I2C;
				//printBuffer("(GGGGG)",7);
				peripheral = UART;
				//HAL_UART_Transmit(&huart2, i2cTxBuffer, I2C_BUFFER_SIZE, HAL_MAX_DELAY);
				encoder_lastTick = HAL_GetTick();
			}

		}



	}

	/*USER CODE END WHILE */

	/*USER CODE BEGIN 3 */

	/*USER CODE END 3 */
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

  ADC_ChannelConfTypeDef sConfig;// = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.LowPowerAutoPowerOff = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_1CYCLE_5;
  hadc1.Init.SamplingTimeCommon2 = ADC_SAMPLETIME_1CYCLE_5;
  hadc1.Init.OversamplingMode = DISABLE;
  hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

//uint64_t full_encoder_count = 0;

void TIM3_IRQHandler(void)
{
	if (__HAL_TIM_GET_FLAG(&htim3, TIM_FLAG_UPDATE) != RESET)
	{
		if (__HAL_TIM_GET_IT_SOURCE(&htim3, TIM_IT_UPDATE) != RESET)
		{
			__HAL_TIM_CLEAR_IT(&htim3, TIM_IT_UPDATE);

			if (__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim3))
			{
				full_encoder_count -= 0x10000;	// Decrement high-order counter on underflow
				//UART_SendFormattedString("underflow\n");
			}
			else
			{
				full_encoder_count += 0x10000;	// Increment high-order counter on overflow
				//UART_SendFormattedString("overflow\n");
			}
		}
	}
}

/**
 *@brief System Clock Configuration
 *@retval None
 */
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct;// ;

	RCC_ClkInitTypeDef RCC_ClkInitStruct;// ;

	/**Initializes the RCC Oscillators according to the specified parameters
	 *in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV4;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/**Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
		RCC_CLOCKTYPE_PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
 *@brief I2C1 Initialization Function
 *@param None
 *@retval None
 */
static void MX_I2C1_Init(void)
{
	hi2c1.Instance = I2C1;
	  hi2c1.Init.Timing = 0x40000A0B; //100khz
	  hi2c1.Init.OwnAddress1 = 64; //0x20
	  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	  hi2c1.Init.OwnAddress2 = 0;
	  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_ENABLE;
	  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
	  {
	    Error_Handler();
	  }

	  /** Configure Analogue filter
	  */
	  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
	  {
	    Error_Handler();
	  }

	  /** Configure Digital filter
	  */
	  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
	  {
	    Error_Handler();
	  }


	/*USER CODE BEGIN I2C1_Init 2 */

	 GPIO_InitTypeDef GPIO_InitStruct;// = {0};
	  /* USER CODE BEGIN I2C1_MspInit 0 */

	  /* USER CODE END I2C1_MspInit 0 */
	    //LL_RCC_SetI2CClockSource(LL_RCC_I2C1_CLKSOURCE_PCLK1);

	  /* USER CODE BEGIN I2C1_MspInit 1 */
	  RCC_PeriphCLKInitTypeDef PeriphClkInit;// = {0};

	    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;

	       PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
	       if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
	       {
	         Error_Handler();
	       }

	       __HAL_RCC_GPIOB_CLK_ENABLE();
	       __HAL_RCC_GPIOC_CLK_ENABLE();
	       /**I2C1 GPIO Configuration
	       PB7     ------> I2C1_SCL
	       PC14-OSCX_IN (PC14)     ------> I2C1_SDA
	       */
	       GPIO_InitStruct.Pin = GPIO_PIN_7;
	       GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
	       GPIO_InitStruct.Pull = GPIO_NOPULL;
	       GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	       GPIO_InitStruct.Alternate = GPIO_AF14_I2C1;
	       HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	       GPIO_InitStruct.Pin = GPIO_PIN_14;
	       GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
	       GPIO_InitStruct.Pull = GPIO_NOPULL;
	       GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	       GPIO_InitStruct.Alternate = GPIO_AF14_I2C1;
	       HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	       /* Peripheral clock enable */
	       __HAL_RCC_I2C1_CLK_ENABLE();
	       /* I2C1 interrupt Init */
	       HAL_NVIC_SetPriority(I2C1_IRQn, 0, 0);
	       HAL_NVIC_EnableIRQ(I2C1_IRQn);
	  /* USER CODE END I2C1_MspInit 1 */

}

/**
 *@brief TIM1 Initialization Function
 *@param None
 *@retval None
 */
static void MX_TIM1_Init(void)
{
	/*USER CODE BEGIN TIM1_Init 0 */

	/*USER CODE END TIM1_Init 0 */

	TIM_MasterConfigTypeDef sMasterConfig ;

	TIM_OC_InitTypeDef sConfigOC ;

	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig ;

	/*USER CODE BEGIN TIM1_Init 1 */

	/*USER CODE END TIM1_Init 1 */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 0;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 65535;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
	{
		Error_Handler();
	}

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
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
	if (HAL_TIM_PWM_ConfigChannel_optimized(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
	{
		Error_Handler();
	}

	if (HAL_TIM_PWM_ConfigChannel_optimized(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
	{
		Error_Handler();
	}

	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 0;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.BreakFilter = 0;
	sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
	sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
	sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
	sBreakDeadTimeConfig.Break2Filter = 0;
	sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
	{
		Error_Handler();
	}

	/*USER CODE BEGIN TIM1_Init 2 */

	/*USER CODE END TIM1_Init 2 */
	HAL_TIM_MspPostInit(&htim1);

}


/**
 *@brief TIM3 Initialization Function
 *@param None
 *@retval None
 */
static void MX_TIM3_Init(void)
{
	/*USER CODE BEGIN TIM3_Init 0 */

	/*USER CODE END TIM3_Init 0 */

	TIM_Encoder_InitTypeDef sConfig ;

	TIM_MasterConfigTypeDef sMasterConfig ;

	TIM_IC_InitTypeDef sConfigIC ;

	/*USER CODE BEGIN TIM3_Init 1 */

	/*USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 0;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 65535;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_IC_Init(&htim3) != HAL_OK)
	{
		Error_Handler();
	}

	sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
	sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC1Filter = 0;
	sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC2Filter = 0;
	if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}

	sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
	sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
	sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
	sConfigIC.ICFilter = 0;
	if (HAL_TIM_IC_ConfigChannel_optimized(&htim3, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
	{
		Error_Handler();
	}

	if (HAL_TIM_IC_ConfigChannel_optimized(&htim3, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
	{
		Error_Handler();
	}

	/*USER CODE BEGIN TIM3_Init 2 */
	__HAL_TIM_ENABLE_IT(&htim3, TIM_IT_UPDATE);

	NVIC_EnableIRQ(TIM3_IRQn);
	/*USER CODE END TIM3_Init 2 */

}

void HAL_TIM_Encoder_MspInit(TIM_HandleTypeDef *htim_encoder)
{
	GPIO_InitTypeDef GPIO_InitStruct ;

	if (htim_encoder->Instance == TIM3)
	{
		/*TIM3 clock enable */
		__HAL_RCC_TIM3_CLK_ENABLE();

		__HAL_RCC_GPIOB_CLK_ENABLE();
		/**TIM3 GPIO Configuration
		PB0     ------> TIM3_CH3
		PB1     ------> TIM3_CH4
		*/
		GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_PULLDOWN;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

		HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(TIM3_IRQn);
	}
}

/**
 *@brief USART2 Initialization Function
 *@param None
 *@retval None
 */
static void MX_USART2_UART_Init(void)
{
	/*USER CODE BEGIN USART2_Init 0 */

	/*USER CODE END USART2_Init 0 */

	/*USER CODE BEGIN USART2_Init 1 */

	/*USER CODE END USART2_Init 1 */
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

	/*USER CODE BEGIN USART2_Init 2 */

	/*USER CODE END USART2_Init 2 */

}

/**
 *Enable DMA controller clock
 */
static void MX_DMA_Init(void)
{
	/*DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE();

	// Configure DMA request for I2C1_RX on DMA1_Channel3
	hdma_i2c1_rx.Instance = DMA1_Channel3;
	hdma_i2c1_rx.Init.Request = DMA_REQUEST_I2C1_RX;
	hdma_i2c1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
	hdma_i2c1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
	hdma_i2c1_rx.Init.MemInc = DMA_MINC_ENABLE;
	hdma_i2c1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	hdma_i2c1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
	hdma_i2c1_rx.Init.Mode = DMA_CIRCULAR; // Set to CIRCULAR for continuous receive
	hdma_i2c1_rx.Init.Priority = DMA_PRIORITY_HIGH;

	if (HAL_DMA_Init(&hdma_i2c1_rx) != HAL_OK)
	{
		Error_Handler();
	}

	// Link DMA handle to I2C handle
	__HAL_LINKDMA(&hi2c1, hdmarx, hdma_i2c1_rx);

	// Configure DMA request hdma_usart2_rx on DMA1_Channel1
	hdma_usart2_rx.Instance = DMA1_Channel1;
	hdma_usart2_rx.Init.Request = DMA_REQUEST_USART2_RX;
	hdma_usart2_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
	hdma_usart2_rx.Init.PeriphInc = DMA_PINC_DISABLE;
	hdma_usart2_rx.Init.MemInc = DMA_MINC_ENABLE;
	hdma_usart2_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	hdma_usart2_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
	hdma_usart2_rx.Init.Mode = DMA_CIRCULAR;
	hdma_usart2_rx.Init.Priority = DMA_PRIORITY_LOW;

	if (HAL_DMA_Init(&hdma_usart2_rx) != HAL_OK)
	{
		Error_Handler();
	}

	__HAL_LINKDMA(&huart2, hdmarx, hdma_usart2_rx);

	// Configure DMA request hdma_usart2_tx on DMA1_Channel1
	hdma_usart2_tx.Instance = DMA1_Channel2;
	hdma_usart2_tx.Init.Request = DMA_REQUEST_USART2_TX;	// Adjust this line as needed
	hdma_usart2_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
	hdma_usart2_tx.Init.PeriphInc = DMA_PINC_DISABLE;
	hdma_usart2_tx.Init.MemInc = DMA_MINC_ENABLE;
	hdma_usart2_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	hdma_usart2_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
	hdma_usart2_tx.Init.Mode = DMA_NORMAL;	// Use DMA_CIRCULAR for continuous transfer
	hdma_usart2_tx.Init.Priority = DMA_PRIORITY_LOW;

	if (HAL_DMA_Init(&hdma_usart2_tx) != HAL_OK)
	{
		Error_Handler();
	}

	__HAL_LINKDMA(&huart2, hdmatx, hdma_usart2_tx);

	/*DMA interrupt init */
	/*DMA1_Channel1_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

	HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);

	HAL_NVIC_SetPriority(USART2_IRQn, 0, 1);
	HAL_NVIC_EnableIRQ(USART2_IRQn);

	/* Configure DMA interrupt for DMA1_Channel3_IRQn */
	//HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 0);
	//HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);


	HAL_NVIC_SetPriority(I2C1_IRQn, 0, 1);
	HAL_NVIC_EnableIRQ(I2C1_IRQn);
}

/**
 *@brief GPIO Initialization Function
 *@param None
 *@retval None
 */
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct ;

	/*USER CODE BEGIN MX_GPIO_Init_1 */
	/*USER CODE END MX_GPIO_Init_1 */

	/*GPIO Ports Clock Enable */
	__HAL_RCC_GPIOF_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();

	/*Configure GPIO pin : PF2 */
	GPIO_InitStruct.Pin = GPIO_PIN_2;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

	/*USER CODE BEGIN MX_GPIO_Init_2 */
	/*USER CODE END MX_GPIO_Init_2 */
}

/*USER CODE BEGIN 4 */

/*USER CODE END 4 */

/**
 *@brief  This function is executed in case of error occurrence.
 *@retval None
 */
void Error_Handler(void)
{
	/*USER CODE BEGIN Error_Handler_Debug */
	/*User can add his own implementation to report the HAL error return state */
	//UART_SendFormattedString("ERROR\n");

	//__disable_irq();
	while (1)
	{
    	//printString("While3\n");
	}

	/*USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 *@brief  Reports the name of the source file and the source line number
 *        where the assert_param error has occurred.
 *@param  file: pointer to the source file name
 *@param  line: assert_param error line source number
 *@retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
	/*USER CODE BEGIN 6 */
	/*User can add his own implementation to report the file name and line number,
	   ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/*USER CODE END 6 */
}
#endif /*USE_FULL_ASSERT */
