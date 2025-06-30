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
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdlib.h> // za atoi
#include <string.h>

#include <stdio.h>
#ifdef _GNUC_
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)// Definicija funkcije za ispis znaka (koristi se za printf redirekciju)
#define GETCHAR_PROTOTYPE int __io_getchar(void)// Definicija funkcije za unos znaka (koristi se za getchar)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)// Alternativna definicija funkcije za ispis znaka
#define GETCHAR_PROTOTYPE int fgetc(FILE *f)// Alternativna definicija funkcije za unos znaka
#endif

volatile uint8_t distance_ready = 0;// Globalna varijabla koja označava da je udaljenost uspješno izmjerena
volatile uint8_t lcd_needs_update = 0;// Globalna varijabla koja može označavati da LCD treba biti ažuriran
char lcd_message[16];// Niz znakova (string) duljine 16 znakova za prikaz poruke na LCD zaslonu
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

/* USER CODE BEGIN PV */
uint8_t rx_data;// Varijabla za spremanje primljenog znaka
extern TIM_HandleTypeDef htim3; // Vanjska deklaracija upravljačke strukture za tajmer TIM3
uint32_t echo_start = 0;// Vrijeme kada je ECHO pin prešao u HIGH (početak mjerenja udaljenosti)
uint32_t echo_end = 0;// Vrijeme kada je ECHO pin prešao u LOW (kraj mjerenja udaljenosti)
uint8_t  capturing = 0;// Oznaka je li trenutno u tijeku mjerenje udaljenosti
float distance_cm = 0;// Izračunata udaljenost u centimetrima na temelju trajanja ECHO impulsa
char last_command = 'S'; // zadnja zaprimljena komanda ('F', 'B', 'L', 'R', 'S')

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void pogon(void);// Deklaracija funkcije za upravljanje pogonom (motorima) na temelju primljenih podataka
void ultrasonic_trigger(void);// Deklaracija funkcije koja pokreće ultrazvučni senzor — šalje impuls na TRIG pin i pokreće mjerenje udaljenosti
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
	// Funkcija za pokretanje ultrazvučnog senzora – šalje TRIG impuls i pokreće mjerenje vremena ECHO signala
	void ultrasonic_trigger(void)
	{
		 capturing = 0;// Resetiramo status
		 __HAL_TIM_DISABLE_IT(&htim2, TIM_IT_CC2); // Privremeno isključujemo interrupt na kanalu 2 timer-a 2 (TIM2_CH2)
		 __HAL_TIM_SET_COUNTER(&htim2, 0); // Resetiramo brojač timer-a na 0

		 // Postavljamo da se prekid okida na uzlazni brid (početak ECHO signala)
		 __HAL_TIM_SET_CAPTUREPOLARITY(&htim2, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_RISING);
		 __HAL_TIM_ENABLE_IT(&htim2, TIM_IT_CC2); // Ponovno omogućavamo interrupt za CH2

		 // Trigger impuls na TRIG pinu
		 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);// Najprije ga postavimo na nisku razinu
		 HAL_Delay(2);// Pauza (2 ms)
		 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);// Postavljamo na visoku razinu (aktivacija TRIG)
		 for (volatile int i = 0; i < 300; i++);  // kratak delay (10–15 us ovisno o takta)
		 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);// Vraćamo TRIG na nisku razinu (završetak impulsa)
	}

	// Callback funkcija koja se automatski poziva kad UART primi jedan bajt podataka
	void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
	{
		// Provjeravamo je li UART koji je primio podatak baš USART1
		if (huart->Instance == USART1)
		{
			last_command = rx_data; // spremi zadnju komandu
			pogon();// Pokrećemo funkciju za upravljanje motorima na temelju primljenog znaka
			HAL_UART_Receive_IT(&huart1, &rx_data, 1);// Ponovno aktiviramo primanje sljedećeg znaka u interrupt modu
		}
	}



	void pogon()
	{
		// Provjera je li primljen znak 'F' (naprijed)
		if (rx_data == 'F')

				{
			// Postavljanje PWM vrijednosti (duty cycle) za 4 motora – 75%
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 750);// PWM za motor 1
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 750);// PWM za motor 2
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 750);// PWM za motor 3
				__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 750);// PWM za motor 4

				//zadnji desni
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, 1); //IN2
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 0); //IN1
				//prednji lijevi
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, 1); //IN21
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, 0); //IN22
				//prednji desni
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 1); //IN3
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 0); //IN4
				//zadnji lijevi
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, 1); //IN23
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 0); //IN24
				}

			else if (rx_data == 'B')

				{
				// Postavi PWM duty cycle na 750
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 750);
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 750);
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 750);
				__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 750);

				//zadnji desni
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, 0); //IN2
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 1); //IN1
				//prednji lijevi
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, 0); //IN21
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, 1); //IN22
				//prednji desni
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 0); //IN3
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 1); //IN4
				//zadnji lijevi
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, 0); //IN23
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 1); //IN24

				}

			else if (rx_data == 'S')

				{
				// Postavi PWM duty cycle na 0
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
				__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);

			    //zadnji desni
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, 0); //IN2
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 0); //IN1
				//prednji lijevi
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, 0); //IN21
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, 0); //IN22
				//prednji desni
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 0); //IN3
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 0); //IN4
				//zadnji lijevi
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, 0); //IN23
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 0); //IN24

				}

			else if (rx_data == 'R')

				{
				// Postavi PWM duty cycle na 750
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 500);
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 500);
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 750);
				__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 750);

				//zadnji desni
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, 0); //IN2
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 1); //IN1
				//prednji lijevi
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, 1); //IN21
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, 0); //IN22
				//prednji desni
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 0); //IN3
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 1); //IN4
				//zadnji lijevi
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, 1); //IN23
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 0); //IN24

					}

			else if (rx_data == 'L')

				{
				// Postavi PWM duty cycle na 750
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 750);
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 750);
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 500);
				__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 500);

				//zadnji desni
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, 1); //IN2
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 0); //IN1
				//prednji lijevi
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, 0); //IN21
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, 1); //IN22
				//prednji desni
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 1); //IN3
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 0); //IN4
				//zadnji lijevi
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, 0); //IN23
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 1); //IN24

			}
	}

	void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
	{
		// Provjera je li interrupt došao s timera TIM12 i kanala CH1
	    if (htim->Instance == TIM12 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
	    {
	    	// Ako još nije započeto mjerenje (čekamo početak impulsa)
	        if (capturing == 0)
	        {
	        	// Pohrani vrijednost vremena kad je signal narastao (RISING edge) – početak impulsa
	            echo_start = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
	            capturing = 1; // Oznaci da je mjerenje u tijeku
	            // Postavi polaritet na FALLING kako bismo uhvatili kraj impulsa (kad signal padne)
	            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
	        }
	        else // Ako je signal već narastao i sada pada – završetak impulsa
	        {
	        	// Pohrani vrijednost vremena kada je signal pao (FALLING edge)
	            echo_end = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
	            capturing = 0;// Oznaci da je mjerenje završeno
	            // Vrati polaritet na RISING za sljedeće mjerenje
	            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
	            // Izračunaj trajanje impulsa, uz provjeru je li se brojač preokrenuo
	            uint32_t duration = (echo_end > echo_start) ?
	                (echo_end - echo_start) :
	                (0xFFFF - echo_start + echo_end);

	            distance_cm = duration / 58.0f;  // Pretvori trajanje impulsa u centimetre (brzina zvuka = 343 m/s → 58 µs/cm)
	            distance_ready = 1;// Oznaci da je udaljenost spremna za korištenje
	        }
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
	  setvbuf(stdin, NULL, _IONBF, 0);
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
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM8_Init();
  MX_I2C1_Init();
  MX_TIM12_Init();
  /* USER CODE BEGIN 2 */

	  // Inicijalizacija LCD-a
	  lcd_init(&hi2c1);

	  // Postavljanje kursora na prvi red, prvu poziciju
	  lcd_put_cursor(0, 0);
	  lcd_send_string("Startamo...");
	  HAL_Delay(1000);

	  // Slanje teksta na LCD
	  //lcd_send_string("Spreman");

	  HAL_UART_Receive_IT(&huart1, &rx_data, 1); // Pokreni primanje znaka putem Bluetootha

	  // Pokrećemo PWM signale na 4 različita timera – svaki timer kontrolira brzinu jednog motora (kroz ENA/ENB pinove H-mostova)
	  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	  HAL_TIM_IC_Start_IT(&htim12, TIM_CHANNEL_1);// Pokreće input capture s prekidima na timeru TIM12 kanal 1 – koristi se za mjerenje trajanja ECHO
	  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);// Pokreće PWM signal za zujalicu (buzzer), na TIM8 – kontrolira jačinu tona ovisno o udaljenosti

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	 while (1)
	  {
		 ultrasonic_trigger(); // Pokreće slanje ultrazvučnog impulsa preko TRIG pina (PC5) i priprema za mjerenje ECHO impulsa
		 while (capturing);  // Čeka dok se mjerenje ne završi
		 HAL_Delay(100);// Pauza od 100 ms između svakog mjerenja (za stabilnost)

			 // čekaj da rezultat bude spreman
			 if (distance_ready)
			 {
				 lcd_clear();// Briše LCD prikaz
				 lcd_put_cursor(0, 0);// Postavlja kursor na početak prvog reda

				 if (distance_cm <= 10.0f) {
					 lcd_send_string("Pazi zid!");// Ako je prepreka jako blizu, upozori
				 } else {
					 char buffer[16];
					 // Formatira string s udaljenosti na najbliži cijeli broj
					 sprintf(buffer, "Udaljenost:%d cm", (int)(distance_cm + 0.5f));
					 lcd_send_string(buffer);// Ispisuje na LCD
				 }

				 distance_ready = 0;// Resetira flag za sljedeće mjerenje
				 // Ovisno o udaljenosti, podešava zvuk zujalice (PWM)
				 if (distance_cm <= 10.0f) {
							__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 32767); // glasno (50% duty)
							// Pošalji poruku preko UART (Bluetooth)
							HAL_UART_Transmit(&huart1, (uint8_t*)"Pazi zid!\r\n", strlen("Pazi zid!\r\n"), HAL_MAX_DELAY);

						} else if (distance_cm <= 15.0f) {
							__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 16383); // srednje
						} else if (distance_cm <= 20.0f) {
							__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 8191); // tiho
						} else {
							__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 0); // isključi zvuk
						}
				 // LED signalizacija prema udaljenosti
				 if (distance_cm < 10.0f) {
					 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);  // LED svijetli stalno
				 } else if (distance_cm < 20.0f) {
					 static uint32_t last_blink = 0;
					 if (HAL_GetTick() - last_blink > 300) { // blink svakih 300 ms
						 HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_15);
						 last_blink = HAL_GetTick();
					 }
				 } else {
					 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);  // LED ugašena
				 }
				 // Ako je udaljenost manja od 10 cm i ne pokušavaš ići unazad ('B')
				 if (distance_cm < 10.0f && last_command != 'B') {
					 // Stop PWM
					 // Postavi PWM duty cycle na 0
					__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
					__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
					__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
					__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);

					//zadnji desni
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, 0); //IN2
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 0); //IN1
					//prednji lijevi
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, 0); //IN12
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, 0); //IN22
					//prednji desni
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 0); //IN3
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 0); //IN4
					//zadnji lijevi
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, 0); //IN23
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 0); //IN24
				 }


			 }

			 HAL_Delay(400);



	  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 50;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
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
