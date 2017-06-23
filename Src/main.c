/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f3xx_hal.h"
#include "spi.h"
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "TextOut.h"
#include "string.h"
#include "commsfpga.h"
#include "speedcalc.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void shoot(uint8_t intensity, SPI_HandleTypeDef* spiHandle, uint8_t freqChannel, uint8_t address);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
//
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USB_DEVICE_Init();
  MX_SPI3_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();

  /* USER CODE BEGIN 2 */
  //rik nieuwe code
  TIM_OC_InitTypeDef dribbler;
  int kickprev=0;

  dribbler.OCMode = TIM_OCMODE_PWM2;
  dribbler.Pulse = 0;
  dribbler.OCPolarity = TIM_OCPOLARITY_HIGH;
  dribbler.OCFastMode = TIM_OCFAST_DISABLE;

  uint8_t showPacket = 0;
  uint8_t showCalculation = 0;
  uint8_t showFPGAdebug = 0;
  uint8_t comFailSecurity = 1;
  //HAL_Delay(1000);
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &dribbler, TIM_CHANNEL_2) != HAL_OK){
		 Error_Handler();
  }

  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
  /*while(!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0)){
	  HAL_Delay(1);
  }*/
  fun();


  int intToMotor = 0;

  int loopcntr = 0;


  wheelVelocityPacket wheely;
  wheely.enablesWheels = 0xFF;
  wheely.velocityWheel1 = 000;
  wheely.velocityWheel2 = 000;
  wheely.velocityWheel3 = 000;
  wheely.velocityWheel4 = 000;

  wheelVelocityPacket backWheely;

  float prevWheelCommand[4];

  uint8_t address = 3;

  uint8_t freqChannel =  0x2A;
  nssHigh(&hspi3);
  HAL_Delay(100);
  initRobo(&hspi3, freqChannel, address);
  HAL_Delay(100);
  printAllRegisters(&hspi3);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  int negative = 0;


  uint8_t stopSending = 0;
  int breakCnt = 0;
  dataPacket dataStruct;


  while (1)
  {
	  //Check the blue buttom (for debugging purposes)
	  if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0)){
		  //print all the registers of the NRF24 chip
		  printAllRegisters(&hspi3);

		  //bling LED's
		  fun();
	  }

	  //check for interrupts of the NRF24 chip (messanges from the base station)
	  if(irqRead(&hspi3)){
		  //if the robot does not get a packet for a long time, it should stop for safety reasons. This line resets that counter
		  breakCnt = 0;

		  //read the message and put it in a struct
		  roboCallback(&hspi3, &dataStruct);

		  //if there is a shoot command, shoot
		  //kickprev!=dataStruct.kickForce prevents the robot from shooting twice in a row.
		  //Shooting for a longer time is problematic, because it will short the battery over the solenoid.
		  if (dataStruct.kickForce != 0 && kickprev!=dataStruct.kickForce){
			  shoot(dataStruct.kickForce, &hspi3, freqChannel, address);
		  }

		  kickprev = dataStruct.kickForce;

		  //give the dribbler the right speed using pwm
		  dribbler.Pulse=125*dataStruct.driblerSpeed;

		  if (HAL_TIM_PWM_ConfigChannel(&htim2, &dribbler, TIM_CHANNEL_2) != HAL_OK){
				 Error_Handler();
		  }

		  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);

		  //print the packet to the screen if that debug option is selected
		  if(showPacket){
			  printDataStruct(&dataStruct);
		  }

		  //calculate the speed of the individual motors using the commands from the base station and put it in a struct
		  calcMotorSpeed(&dataStruct, &wheely, &prevWheelCommand);

		  //print the calculated motor speeds to the screen if that debug option is selected
		  if(showCalculation){
			  sprintf(smallStrBuffer, "wheely: %i   %i   %i   %i\n", wheely.velocityWheel1, wheely.velocityWheel2, wheely.velocityWheel3, wheely.velocityWheel4);
			  TextOut(smallStrBuffer);
		  }
	  }

	  HAL_Delay(1);

	//if the robot did not get messanges for a long time, break unless the debug option that disables this option is selected
	if(breakCnt >= 250 && comFailSecurity == 1){
		wheely.velocityWheel1 = 0;
		wheely.velocityWheel2 = 0;
		wheely.velocityWheel3 = 0;
		wheely.velocityWheel4 = 0;
		initRobo(&hspi3, 0x2A, address);
		breakCnt = 0;
	}

	breakCnt++;

	//if the robot gets a message over USB (done for debugging purposes)
	if(usbLength != 0){

		//for every element in the USB-buffer (most often there is just one element)
		for(int i = 0; i <= usbLength; i++){

			//if the element is a number, interpretend it as a number.
			if(usbData[i] >= 48 && usbData[i] <= 57){
				sprintf(smallStrBuffer, "%i", usbData[i]-48);
				TextOut(smallStrBuffer);
				intToMotor = intToMotor*10;
				intToMotor += (usbData[i] - 48);
				stopSending = 1;
			}

			//if space is pressed, interpretend the number as a robot-speed and send a command to the FPGA
			else if(usbData[i] == 32){
				if(negative == 1){
					intToMotor = -intToMotor;
					negative = 0;
				}
				TextOut("\n");
				wheely.velocityWheel1 = intToMotor;
				wheely.velocityWheel2 = intToMotor;
				wheely.velocityWheel3 = -intToMotor;
				wheely.velocityWheel4 = -intToMotor;

				stopSending = 0;
				intToMotor = 0;
			}
			//if the '-' is presed, make the number negative
			else if(usbData[i] == 45){
				negative = 1;
			}
			//if 's' is pressed, interpreted the number as a kicking-intensity, and send a command to the kicking-module
			else if(usbData[i] == 's'){

				shoot(intToMotor, &hspi3, freqChannel, address);

				stopSending = 0;
				intToMotor = 0;

			}
			//if 'd' is pressed, interpreted the number as a dribler-intensity, and put the dribbler on
			else if(usbData[i] == 'd'){
				dribbler.Pulse=125*intToMotor;
				stopSending = 0;
				intToMotor = 0;

				sprintf(smallStrBuffer, "dribler pulse = %i", dribbler.Pulse);
				TextOut(smallStrBuffer);
				if (HAL_TIM_PWM_ConfigChannel(&htim2, &dribbler, TIM_CHANNEL_2) != HAL_OK){
					Error_Handler();
				 }

				 HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
			}
			//if 'i' is pressed, interpretend the number as the robot id, and send a message the the NRF24 chip to change the id.
			else if(usbData[i] == 'i'){
				if(intToMotor >= 0 && intToMotor <= 9){
					address = intToMotor;
					initRobo(&hspi3, freqChannel, address);
					sprintf(smallStrBuffer, "address = %i\n", intToMotor);
					TextOut(smallStrBuffer);
				}
				else{
					TextOut("error: id should be between 0 and 9");
				}
			}
			//if 'p' is pressed, enable a debug option so that all messanges of the NRF24 module will be printed
			else if(usbData[i] == 'p'){
				if(showPacket == 0){
					showPacket = 1;
					TextOut("showPacket enabled\n");
				}
				else{
					showPacket = 0;
					TextOut("showPacket disabled\n");
				}
			}
			//if 'p' is pressed, enable a debug option so that all calculated motor speed will be printed
			else if(usbData[i] == 'c'){
				if(showCalculation == 0){
					showCalculation = 1;
					TextOut("showCalculation enabled\n");
				}
				else{
					showCalculation = 0;
					TextOut("showCalculation disabled\n");
				}
			}
			//if 'f' is pressed, enable a debug option so that all commands send to the FPGA, and all speeds of the wheels the FPGA send back will be printed
			else if(usbData[i] == 'f'){
				if(showFPGAdebug == 0){
					showFPGAdebug = 1;
					TextOut("showFPGAdebug enabled\n");
				}
				else{
					showFPGAdebug = 0;
					TextOut("showFPGAdebug disabled\n");
				}
			}
			//if 'b' is pressed, disable the security that lets the robot stop if it gets no messanges.
			//This is usefull when controlling the robot over USB.
			else if(usbData[i] == 'b'){
				if(comFailSecurity == 0){
					comFailSecurity = 1;
					TextOut("comFailSecurity enabled\n");
				}
				else{
					comFailSecurity = 0;
					TextOut("comFailSecurity disabled\n");
				}
			}
			//if 'r' is pressed, print all registers of the NRF24 chip
			else if(usbData[i] == 'r'){
				//initRobo(&hspi3, freqChannel, address);
				printAllRegisters(&hspi3);
			}
			//in all other cases, throw away the number
			else if(usbData[i] != 0){
				TextOut("no number send\n");

				stopSending = 0;
				intToMotor = 0;
			}
		}
		usbLength = 0;

	}


	//once in a while (loopcntr == 1 means every time, but this can be increased)
	if(loopcntr == 1){
		//send the wheelcommands to the FPGA
		sendReceivePacket(&hspi1, &wheely, &backWheely);
		if(showFPGAdebug){
			uint8_t regTest;
			sprintf(smallStrBuffer, "wheely: %i, %i, %i, %i,", wheely.velocityWheel1, wheely.velocityWheel2, wheely.velocityWheel3, wheely.velocityWheel4);
			TextOut(smallStrBuffer);
			sprintf(smallStrBuffer, "%i, %i, %i, %i, %x\n", backWheely.velocityWheel1, backWheely.velocityWheel2, backWheely.velocityWheel3, backWheely.velocityWheel4, backWheely.enablesWheels);
			TextOut(smallStrBuffer);
		}

		loopcntr = 0;
	}

	if(!stopSending){
		loopcntr++;
	}


  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_TIM1;
  PeriphClkInit.USBClockSelection = RCC_USBCLKSOURCE_PLL;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

void shoot(uint8_t intensity, SPI_HandleTypeDef* spiHandle, uint8_t freqChannel, uint8_t address){
	sprintf(smallStrBuffer, "intensity = %i\n", intensity);
	TextOut(intensity);
	int period = 1000 + intensity*15;

	htim1.Init.Period = period;



	HAL_TIM_Base_Init(&htim1);

	HAL_GPIO_WritePin(trigger_GPIO_Port, trigger_Pin,GPIO_PIN_SET);
	HAL_TIM_Base_Start_IT(&htim1);

	//wheely.velocityWheel1 = intToMotor;
	//wheely.velocityWheel2 = intToMotor;
	//wheely.velocityWheel3 = -intToMotor;
	//wheely.velocityWheel4 = -intToMotor;
	//HAL_Delay(100);
	//initRobo(spiHandle, freqChannel, address);
	//HAL_Delay(100);
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

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
