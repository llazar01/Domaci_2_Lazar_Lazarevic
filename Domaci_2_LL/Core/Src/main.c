/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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


//Lazar Lazarevic: 14 * 5 = 70
float zeljena_temp=70;
int epruvete[]={3,1,4,1,0,3,1,4,1,0,2,3,0,3};
int kutija[14];
int brojac_kutija = 0;
float trenutna_temp;
int velicina_niza = sizeof(epruvete) / sizeof(epruvete[0]);
uint16_t rez;
int pokretna_traka = 50; //mm
int pomeraj_trake = 5; //mm


/*Moje Ime i Prezime sacinjava 15 slova, kako pun krug ima 360 stepeni,
 *epruvete ce se nalaziti na 25.71 stepena pomeraja motora.
 *Jedan impuls motor okrene za 1.8 stepen sto znaci da je potrebno 14.28
 *impulsa kako bi motor pomerio epruvete za po jedno mesto.
 *Kako bih mogao da simuliram sa celim brojevima usvojicu da je 14 impulsa
 *potrebno kako bi motor pomerio epruvete za po jedno mesto.
 *
 */
int pomeraj_epruvete = 14;

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	// Funkcija za rotaciju niza u desno za određeni broj koraka


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
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  HAL_ADC_Start(&hadc1);
	  HAL_ADC_PollForConversion(&hadc1, 100);
	  rez = HAL_ADC_GetValue(&hadc1);
	  trenutna_temp = (1.5*rez/4096)*100;

	  while(trenutna_temp < zeljena_temp){
	  	//ukljucujemo motor M1, okretanje u smeru kazaljke na satu
		  for (int i = 0; i < pomeraj_epruvete; i++) {
		      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET); // Enable
		      HAL_Delay(0.000005); //Enable mora biti ispred Direction minimalno 5µs
		      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET); // Direction=1, smer kazaljke
		      HAL_Delay(0.000005); //Direction mora biti ispred Impulsa minimalno 5µs
		      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET); // Impulse
		      HAL_Delay(0.0000025); //Minimalna sirina visokog nivoa ne sme biti manja od 2.5µs
		      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET); // Impulse
		      HAL_Delay(0.0000025); //Minimalna sirina niskog nivoa ne sme biti manja od 2.5µs
		  }
		    //kada se motor pomerio za 14 impulsa sada treba rotirati pozicije epruveta
		    // Čuvanje vrednosti poslednjeg elementa
	        int temp = epruvete[velicina_niza - 1];
	        // Pomeranje svih elemenata u desno
	        for (int i = velicina_niza - 1; i > 0; i--) {
	        	epruvete[i] = epruvete[i - 1];
	        }
	        // Postavljanje čuvane vrednosti na prvo mesto
	        epruvete[0] = temp;
	  }

	  //Zaustavljanje motora M1, epruvete zagrejane do zeljene temperature
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET); //


	  // Biranje epruvete koju zelimo:
	  // 0 , 1, 2, 3, 4
	  int uneti_broj = 0;

	  int indeks_pocetak = -1;
	  int indeks_kraj = -1;

	    for (int i = 0; i < velicina_niza; ++i) {
	        if (epruvete[i] == uneti_broj) {
	            indeks_pocetak = i;
	            break;
	        }
	    }

	    // Pretraga niza od kraja
	    int k = 1;  // Pomeranje indeksa od desne strane
	    for (int i = velicina_niza - 1; i >= 0; --i) {
	        if (epruvete[i] == uneti_broj) {
	            indeks_kraj = k;
	            break;
	        }
	        k++;
	    }


	    //funlcije koje ce rotirati elemente niza na osnovu pomeranja motora

	    // Funkcija za rotaciju niza u desno za određeni broj koraka
		void rotateRight(int arr[], int n, int steps) {
		    int temp;
		    for (int i = 0; i < steps; ++i) {
		        temp = arr[n - 1];
		        for (int j = n - 1; j > 0; --j) {
		            arr[j] = arr[j - 1];
		        }
		        arr[0] = temp;
		    }
		}

		// Funkcija za rotaciju niza u levo za određeni broj koraka
		void rotateLeft(int arr[], int n, int steps) {
		    int temp;
		    for (int i = 0; i < steps; ++i) {
		        temp = arr[0];
		        for (int j = 0; j < n - 1; ++j) {
		            arr[j] = arr[j + 1];
		        }
		        arr[n - 1] = temp;
		    }
		}


	    // Ispis rezultata
	    if (indeks_pocetak != -1 && indeks_kraj != -1) {
	        if (indeks_pocetak <= indeks_kraj) {
	            printf("Motor 1 se okreće u desno za %d koraka\n", indeks_pocetak);
	            //za pomeraj epruvete za 1 korak potrebno je dovesti 14 impulsa, okrece se u desno
	            for (int i = 0; i < pomeraj_epruvete * indeks_pocetak ; i++) {
	  		         HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET); // Enable
	  		         HAL_Delay(0.000005); //Enable mora biti ispred Direction minimalno 5µs
	  		         HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET); // Direction=1, motor se okrece u desno
	  		         HAL_Delay(0.000005); //Direction mora biti ispred Impulsa minimalno 5µs
	  		         HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET); // Impulse
	  		         HAL_Delay(0.0000025); //Minimalna sirina visokog nivoa ne sme biti manja od 2.5µs
	  		         HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET); // Impulse
	  		         HAL_Delay(0.0000025); //Minimalna sirina niskog nivoa ne sme biti manja od 2.5µs
	            }
	        //Motor 1 se okrenuo udesno potreban broj impulsa i doveo je epruvetu na zeljeno mesto
	        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
	        //stanje epruveta(niza) nakon okretanja motora
	        rotateLeft(epruvete, velicina_niza, indeks_pocetak);

	        } else {
	            printf("Motor 1 se okreće u levo za %d koraka\n", indeks_kraj);
	            //za pomeraj epruvete za 1 korak potrebno je dovesti 14 impulsa, okrece se u levo
	            for (int i = 0; i < pomeraj_epruvete * indeks_kraj ; i++) {
	  		         HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET); // Enable
	  		         HAL_Delay(0.000005); //Enable mora biti ispred Direction minimalno 5µs
	  		         HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET); // Direction=0, motor se okrece u levo
	  		         HAL_Delay(0.000005); //Direction mora biti ispred Impulsa minimalno 5µs
	  		         HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET); // Impulse
	  		         HAL_Delay(0.0000025); //Minimalna sirina visokog nivoa ne sme biti manja od 2.5µs
	  		         HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET); // Impulse
	  		         HAL_Delay(0.0000025); //Minimalna sirina niskog nivoa ne sme biti manja od 2.5µs
	       	    }
	        //Motor 1 se okrenuo ulevo potreban broj impulsa i doveo je epruvetu na zeljeno mesto
	        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
	        //stanje epruveta(niza) nakon okretanja motora
	        rotateRight(epruvete, velicina_niza, indeks_kraj);
	        }

	        // Ispis niza nakon pomeranja
	        printf("Niz nakon pomeranja: ");
	        for (int i = 0; i < velicina_niza; ++i) {
	            printf("%d ", epruvete[i]);
	        }
	        printf("\n");

	    } else {
	        printf("Broj %d nije pronađen u nizu.\n", uneti_broj);
	    }


	    /* Kada smo dosli ovde epruveta stoji na mestu odakle ce je pokretna traka odvesti u kutiju
	     * simuliranje uzimanja eprivete cemo izvrsiti tako sto cemo je smestiti u niz kutija
	     *
	     */

	    //pokretanje Motora 2, koji ima zadatak da pomera traku
	    /*Da bi se motor okrenuo za jedan korak potrebno mu je 14 step signala
	     * posto je traka 50mm a jedan korak okrene traku za 5mm
	     * potrebno je da se napravi 50/5 koraka  * 14 step signala
	     */
	    for (int i = 0; i < pomeraj_epruvete*(pokretna_traka/pomeraj_trake) ; i++) {
		       HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET); // Enable
		       HAL_Delay(0.000005); //Enable mora biti ispred Direction minimalno 5µs
		       HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET); // Direction=1, pretpostavka da traka okretanjem u desno prenosi epruvete
	           HAL_Delay(0.000005); //Direction mora biti ispred Impulsa minimalno 5µs
		       HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET); // Impulse
		       HAL_Delay(0.0000025); //Minimalna sirina visokog nivoa ne sme biti manja od 2.5µs
		       HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET); // Impulse
		       HAL_Delay(0.0000025); //Minimalna sirina niskog nivoa ne sme biti manja od 2.5µs
	    }
	           HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET); // Zaustavljamo Moror2

	           // Premestanje nultog elementa u kutiju
	           kutija[brojac_kutija] = epruvete[0];
	           brojac_kutija++;

	           // Postavljanje vrednosti 404 na mesto nultog elementa
	           epruvete[0] = 404;

	           // Prikazivanje elemenata u kutiji
	           printf("\nElementi u kutiji: ");
	           for (int i = 0; i < brojac_kutija; i++) {
	               printf("%d ", kutija[i]);
	           }

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
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
  hadc1.Init.ContinuousConvMode = ENABLE;
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
  sConfig.Channel = ADC_CHANNEL_1;
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
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4
                          |GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB1 PB2 PB3 PB4
                           PB5 PB6 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4
                          |GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
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
