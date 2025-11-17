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

DCMIPP_HandleTypeDef hdcmipp;

I2C_HandleTypeDef hi2c1;

LTDC_HandleTypeDef hltdc;

RAMCFG_HandleTypeDef hramcfg_SRAM3;
RAMCFG_HandleTypeDef hramcfg_SRAM4;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* Buffer used for reception */
volatile uint8_t ubRxComplete = 0;
volatile uint32_t uwRxIndex = 0;
uint8_t aRxBuffer[RX_BUF_SIZE] = { 0 };
static volatile uint32_t NbMainFrames = 0;
//volatile uint8_t ubFrameData[FRAME_BUFFER_SIZE] = { 0 };
static uint8_t ubCameraEnable = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_DCMIPP_Init(void);
static void MX_LTDC_Init(void);
static void MX_RAMCFG_Init(void);
static void SystemIsolation_Config(void);
/* USER CODE BEGIN PFP */

#if defined(__ICCARM__)
/* New definition from EWARM V9, compatible with EWARM8 */
int iar_fputc(int ch);
#define PUTCHAR_PROTOTYPE int iar_fputc(int ch)
#define GETCHAR_PROTOTYPE int fgetc(FILE *f)
#elif defined (__CC_ARM) || defined(__ARMCC_VERSION)
/* ARM Compiler 5/6 */
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#define GETCHAR_PROTOTYPE int fgetc(FILE *f)
#elif defined(__GNUC__)
int __io_putchar(int ch);
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
int __io_getchar(void);
#define GETCHAR_PROTOTYPE int __io_getchar(void)
#endif /* __ICCARM__ */

void MPU_Config(void);
static int Command_Execute(uint8_t *commandData, uint16_t commandSize);
static int I2C1_WriteReg(uint16_t devAddr, uint16_t reg, uint16_t regSize, uint8_t *data, uint16_t dataSize);
static int I2C1_ReadReg(uint16_t devAddr, uint16_t reg, uint16_t regSize, uint8_t *data, uint16_t dataSize);
static int TEVS_I2C_WriteReg16(uint16_t reg, uint16_t data);
static int TEVS_I2C_ReadReg16(uint16_t reg, uint16_t *data);
static int TEVS_GPIO_Init(void);
static int TEVS_ResetPin_Ctrl(uint8_t state);
static int TEVS_StandbyPin_Ctrl(uint8_t state);
static int TEVS_Init(void);
static int TEVS_Set_Stream(uint8_t enable);
static int TEVS_Set_Format(uint16_t width, uint16_t height, uint16_t fps);
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
	int32_t ret = 0;

	MPU_Config();
	/* USER CODE END 1 */

	/* Enable the CPU Cache */

	/* Enable I-Cache---------------------------------------------------------*/
	SCB_EnableICache();

	/* Enable D-Cache---------------------------------------------------------*/
	SCB_EnableDCache();

	/* MCU Configuration--------------------------------------------------------*/
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_I2C1_Init();
	MX_USART1_UART_Init();
	MX_DCMIPP_Init();
	MX_LTDC_Init();
	MX_RAMCFG_Init();
	SystemIsolation_Config();
	/* USER CODE BEGIN 2 */

	/*## Configure UART peripheral for reception process (using LL) ##########*/
	/* Any data received will be stored "aRxBuffer" buffer : the number max of
	 data received is RXBUFFERSIZE */
	/* Enable RXNE and Error interrupts */
	LL_USART_EnableIT_RXNE(USART1);
	LL_USART_EnableIT_ERROR(USART1);

	printf("\n\r ========== TechNexion TEVS ========== \r\n\n");
	fflush(stdout);

	ret = TEVS_GPIO_Init();
	if (ret != 0) {
		printf("[  ERROR ] %s(): expander init failed, error = %ld\n", __func__, ret);
		fflush(stdout);
		Error_Handler();
	}

	ret = TEVS_Init();
	if (ret != 0) {
		printf("[  ERROR ] %s(): TEVS init failed, error = %ld\n", __func__, ret);
		fflush(stdout);
		Error_Handler();
	}

	ret = HAL_DCMIPP_CSI_PIPE_Start(&hdcmipp, DCMIPP_PIPE0,
	DCMIPP_VIRTUAL_CHANNEL0, BUFFER_ADDRESS, DCMIPP_MODE_CONTINUOUS);
	if (ret != 0) {
		printf("[  ERROR ] %s(): DCMIPP PIPE0 start failed, error = %ld\n", __func__, ret);
		fflush(stdout);
		Error_Handler();
	}

	ret = HAL_DCMIPP_PIPE_Suspend(&hdcmipp, DCMIPP_PIPE0);
	if (ret != 0) {
		printf("[  ERROR ] %s(): DCMIPP PIPE0 suspend failed, error = %ld\n", __func__, ret);
		fflush(stdout);
		Error_Handler();
	}

	HAL_Delay(1000);
	ret = TEVS_Set_Stream(1);
	if (ret != 0) {
		printf("[  ERROR ] %s(): TEVS set stream failed, error = %ld\n", __func__, ret);
		fflush(stdout);
		Error_Handler();
	}

	ret = HAL_DCMIPP_PIPE_Resume(&hdcmipp, DCMIPP_PIPE0);
	if (ret != 0) {
		printf("[  ERROR ] %s(): DCMIPP PIPE0 suspend failed, error = %ld\n", __func__, ret);
		fflush(stdout);
		Error_Handler();
	}

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		if (ubRxComplete) {
			Command_Execute(aRxBuffer, uwRxIndex);
			ubRxComplete = 0;
			uwRxIndex = 0;
			memset(aRxBuffer, 0, RX_BUF_SIZE);
		}

		if (ubCameraEnable) {
			HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin);
			HAL_Delay(250);
		}
		else {
			HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);
		}
	}
	/* USER CODE END 3 */
}
/* USER CODE BEGIN CLK 1 */
/* USER CODE END CLK 1 */

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the System Power Supply
	 */
	if (HAL_PWREx_ConfigSupply(PWR_EXTERNAL_SOURCE_SUPPLY) != HAL_OK) {
		Error_Handler();
	}

	/* Enable HSI */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL1.PLLState = RCC_PLL_NONE;
	RCC_OscInitStruct.PLL2.PLLState = RCC_PLL_NONE;
	RCC_OscInitStruct.PLL3.PLLState = RCC_PLL_NONE;
	RCC_OscInitStruct.PLL4.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Get current CPU/System buses clocks configuration and if necessary switch
	 to intermediate HSI clock to ensure target clock can be set
	 */
	HAL_RCC_GetClockConfig(&RCC_ClkInitStruct);
	if ((RCC_ClkInitStruct.CPUCLKSource == RCC_CPUCLKSOURCE_IC1)
	        || (RCC_ClkInitStruct.SYSCLKSource == RCC_SYSCLKSOURCE_IC2_IC6_IC11)) {
		RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_CPUCLK | RCC_CLOCKTYPE_SYSCLK);
		RCC_ClkInitStruct.CPUCLKSource = RCC_CPUCLKSOURCE_HSI;
		RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
		if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct) != HAL_OK) {
			/* Initialization Error */
			Error_Handler();
		}
	}

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_NONE;
	RCC_OscInitStruct.PLL1.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL1.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL1.PLLM = 4;
	RCC_OscInitStruct.PLL1.PLLN = 75;
	RCC_OscInitStruct.PLL1.PLLFractional = 0;
	RCC_OscInitStruct.PLL1.PLLP1 = 1;
	RCC_OscInitStruct.PLL1.PLLP2 = 1;
	RCC_OscInitStruct.PLL2.PLLState = RCC_PLL_NONE;
	RCC_OscInitStruct.PLL3.PLLState = RCC_PLL_NONE;
	RCC_OscInitStruct.PLL4.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_CPUCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1
	        | RCC_CLOCKTYPE_PCLK2 | RCC_CLOCKTYPE_PCLK5 | RCC_CLOCKTYPE_PCLK4;
	RCC_ClkInitStruct.CPUCLKSource = RCC_CPUCLKSOURCE_IC1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_IC2_IC6_IC11;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
	RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;
	RCC_ClkInitStruct.APB5CLKDivider = RCC_APB5_DIV1;
	RCC_ClkInitStruct.IC1Selection.ClockSelection = RCC_ICCLKSOURCE_PLL1;
	RCC_ClkInitStruct.IC1Selection.ClockDivider = 2;
	RCC_ClkInitStruct.IC2Selection.ClockSelection = RCC_ICCLKSOURCE_PLL1;
	RCC_ClkInitStruct.IC2Selection.ClockDivider = 3;
	RCC_ClkInitStruct.IC6Selection.ClockSelection = RCC_ICCLKSOURCE_PLL1;
	RCC_ClkInitStruct.IC6Selection.ClockDivider = 3;
	RCC_ClkInitStruct.IC11Selection.ClockSelection = RCC_ICCLKSOURCE_PLL1;
	RCC_ClkInitStruct.IC11Selection.ClockDivider = 3;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief DCMIPP Initialization Function
 * @param None
 * @retval None
 */
static void MX_DCMIPP_Init(void)
{

	/* USER CODE BEGIN DCMIPP_Init 0 */

	/* USER CODE END DCMIPP_Init 0 */

	DCMIPP_CSI_PIPE_ConfTypeDef pCSI_PipeConfig = { 0 };
	DCMIPP_CSI_ConfTypeDef pCSI_Config = { 0 };
	DCMIPP_PipeConfTypeDef pPipeConfig = { 0 };

	/* USER CODE BEGIN DCMIPP_Init 1 */

	/* USER CODE END DCMIPP_Init 1 */
	hdcmipp.Instance = DCMIPP;
	if (HAL_DCMIPP_Init(&hdcmipp) != HAL_OK) {
		Error_Handler();
	}

	/** Pipe 0 Config
	 */
	pCSI_PipeConfig.DataTypeMode = DCMIPP_DTMODE_DTIDA;
	pCSI_PipeConfig.DataTypeIDA = DCMIPP_DT_YUV422_8;
	pCSI_PipeConfig.DataTypeIDB = DCMIPP_DT_YUV422_8;
	if (HAL_DCMIPP_CSI_PIPE_SetConfig(&hdcmipp, DCMIPP_PIPE0, &pCSI_PipeConfig) != HAL_OK) {
		Error_Handler();
	}
	pCSI_Config.PHYBitrate = DCMIPP_CSI_PHY_BT_800;
	pCSI_Config.DataLaneMapping = DCMIPP_CSI_PHYSICAL_DATA_LANES;
	pCSI_Config.NumberOfLanes = DCMIPP_CSI_TWO_DATA_LANES;
	if (HAL_DCMIPP_CSI_SetConfig(&hdcmipp, &pCSI_Config) != HAL_OK) {
		Error_Handler();
	}
	pPipeConfig.FrameRate = DCMIPP_FRAME_RATE_ALL;
	pPipeConfig.PixelPipePitch = 10;
	pPipeConfig.PixelPackerFormat = DCMIPP_PIXEL_PACKER_FORMAT_YUV422_1;
	if (HAL_DCMIPP_PIPE_SetConfig(&hdcmipp, DCMIPP_PIPE0, &pPipeConfig) != HAL_OK) {
		Error_Handler();
	}
	HAL_DCMIPP_CSI_SetVCConfig(&hdcmipp, 0U, DCMIPP_CSI_DT_BPP16);
	/* USER CODE BEGIN DCMIPP_Init 2 */
	if (HAL_DCMIPP_PIPE_SetFrameCounterConfig(&hdcmipp, DCMIPP_PIPE0) != HAL_OK) {
		printf("[  ERROR ] %s(): DCMIPP PIPE0 set frame counter failed\n", __func__);
		fflush(stdout);
		Error_Handler();
	}
	/* USER CODE END DCMIPP_Init 2 */

}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void)
{

	/* USER CODE BEGIN I2C1_Init 0 */

	/* USER CODE END I2C1_Init 0 */

	/* USER CODE BEGIN I2C1_Init 1 */

	/* USER CODE END I2C1_Init 1 */
	hi2c1.Instance = I2C1;
	hi2c1.Init.Timing = 0x30C0EDFF;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
		Error_Handler();
	}

	/** Configure Analogue filter
	 */
	if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK) {
		Error_Handler();
	}

	/** Configure Digital filter
	 */
	if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */

}

/**
 * @brief LTDC Initialization Function
 * @param None
 * @retval None
 */
static void MX_LTDC_Init(void)
{

	/* USER CODE BEGIN LTDC_Init 0 */

	/* USER CODE END LTDC_Init 0 */

	LTDC_LayerFlexYUVCoPlanarTypeDef pLayerFlexYUVCoPlanar = { 0 };

	/* USER CODE BEGIN LTDC_Init 1 */

	/* USER CODE END LTDC_Init 1 */
	hltdc.Instance = LTDC;
	hltdc.Init.HSPolarity = LTDC_HSPOLARITY_AL;
	hltdc.Init.VSPolarity = LTDC_VSPOLARITY_AL;
	hltdc.Init.DEPolarity = LTDC_DEPOLARITY_AL;
	hltdc.Init.PCPolarity = LTDC_PCPOLARITY_IPC;
	hltdc.Init.HorizontalSync = 4;
	hltdc.Init.VerticalSync = 4;
	hltdc.Init.AccumulatedHBP = 12;
	hltdc.Init.AccumulatedVBP = 12;
	hltdc.Init.AccumulatedActiveW = 812;
	hltdc.Init.AccumulatedActiveH = 492;
	hltdc.Init.TotalWidth = 820;
	hltdc.Init.TotalHeigh = 500;
	hltdc.Init.Backcolor.Blue = 0;
	hltdc.Init.Backcolor.Green = 0;
	hltdc.Init.Backcolor.Red = 0;
	if (HAL_LTDC_Init(&hltdc) != HAL_OK) {
		Error_Handler();
	}
	pLayerFlexYUVCoPlanar.Layer.WindowX0 = 0;
	pLayerFlexYUVCoPlanar.Layer.WindowX1 = 640;
	pLayerFlexYUVCoPlanar.Layer.WindowY0 = 0;
	pLayerFlexYUVCoPlanar.Layer.WindowY1 = 480;
	pLayerFlexYUVCoPlanar.Layer.Alpha = 255;
	pLayerFlexYUVCoPlanar.Layer.Alpha0 = 0;
	pLayerFlexYUVCoPlanar.Layer.BlendingFactor1 = LTDC_BLENDING_FACTOR1_CA;
	pLayerFlexYUVCoPlanar.Layer.BlendingFactor2 = LTDC_BLENDING_FACTOR2_CA;
	pLayerFlexYUVCoPlanar.Layer.ImageWidth = 640;
	pLayerFlexYUVCoPlanar.Layer.ImageHeight = 480;
	pLayerFlexYUVCoPlanar.Layer.Backcolor.Blue = 0;
	pLayerFlexYUVCoPlanar.Layer.Backcolor.Green = 0;
	pLayerFlexYUVCoPlanar.Layer.Backcolor.Red = 0;
	pLayerFlexYUVCoPlanar.FlexYUV.YUVOrder = LTDC_YUV_ORDER_LUMINANCE_FIRST;
	pLayerFlexYUVCoPlanar.FlexYUV.LuminanceOrder = LTDC_YUV_LUMINANCE_ORDER_ODD_FIRST;
	pLayerFlexYUVCoPlanar.FlexYUV.ChrominanceOrder = LTDC_YUV_CHROMIANCE_ORDER_U_FIRST;
	pLayerFlexYUVCoPlanar.FlexYUV.LuminanceRescale = LTDC_YUV_LUMINANCE_RESCALE_DISABLE;
	pLayerFlexYUVCoPlanar.YUVAddress = 0;
	pLayerFlexYUVCoPlanar.ColorConverter = LTDC_YUV2RGBCONVERTOR_BT601_FULL_RANGE;
	if (HAL_LTDC_ConfigLayerFlexYUVCoPlanar(&hltdc, &pLayerFlexYUVCoPlanar, 0) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN LTDC_Init 2 */

	pLayerFlexYUVCoPlanar.YUVAddress = BUFFER_ADDRESS;
	if (HAL_LTDC_ConfigLayerFlexYUVCoPlanar(&hltdc, &pLayerFlexYUVCoPlanar, 0) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE END LTDC_Init 2 */

}

/**
 * @brief RAMCFG Initialization Function
 * @param None
 * @retval None
 */
static void MX_RAMCFG_Init(void)
{

	/* USER CODE BEGIN RAMCFG_Init 0 */

	/* USER CODE END RAMCFG_Init 0 */

	/* USER CODE BEGIN RAMCFG_Init 1 */

	/* USER CODE END RAMCFG_Init 1 */

	/** Initialize RAMCFG SRAM3
	 */
	hramcfg_SRAM3.Instance = RAMCFG_SRAM3_AXI;
	if (HAL_RAMCFG_Init(&hramcfg_SRAM3) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_RAMCFG_Erase(&hramcfg_SRAM3) != HAL_OK) {
		Error_Handler();
	}

	/** Initialize RAMCFG SRAM4
	 */
	hramcfg_SRAM4.Instance = RAMCFG_SRAM4_AXI;
	if (HAL_RAMCFG_Init(&hramcfg_SRAM4) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_RAMCFG_Erase(&hramcfg_SRAM4) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN RAMCFG_Init 2 */

	/* USER CODE END RAMCFG_Init 2 */

}

/**
 * @brief RIF Initialization Function
 * @param None
 * @retval None
 */
static void SystemIsolation_Config(void)
{

	/* USER CODE BEGIN RIF_Init 0 */

	/* USER CODE END RIF_Init 0 */

	/* set all required IPs as secure privileged */
	__HAL_RCC_RIFSC_CLK_ENABLE();
	RIMC_MasterConfig_t RIMC_master = { 0 };
	RIMC_master.MasterCID = RIF_CID_1;
	RIMC_master.SecPriv = RIF_ATTRIBUTE_SEC | RIF_ATTRIBUTE_PRIV;

	/*RIMC configuration*/
	HAL_RIF_RIMC_ConfigMasterAttributes(RIF_MASTER_INDEX_DCMIPP, &RIMC_master);
	HAL_RIF_RIMC_ConfigMasterAttributes(RIF_MASTER_INDEX_LTDC1, &RIMC_master);

	/*RISUP configuration*/
	HAL_RIF_RISC_SetSlaveSecureAttributes(RIF_RISC_PERIPH_INDEX_DCMIPP, RIF_ATTRIBUTE_SEC | RIF_ATTRIBUTE_PRIV);
	HAL_RIF_RISC_SetSlaveSecureAttributes(RIF_RISC_PERIPH_INDEX_LTDCL1, RIF_ATTRIBUTE_SEC | RIF_ATTRIBUTE_PRIV);

	/* RIF-Aware IPs Config */

	/* set up GPIO configuration */
	HAL_GPIO_ConfigPinAttributes(GPIOA, GPIO_PIN_0, GPIO_PIN_SEC | GPIO_PIN_NPRIV);
	HAL_GPIO_ConfigPinAttributes(GPIOA, GPIO_PIN_1, GPIO_PIN_SEC | GPIO_PIN_NPRIV);
	HAL_GPIO_ConfigPinAttributes(GPIOA, GPIO_PIN_2, GPIO_PIN_SEC | GPIO_PIN_NPRIV);
	HAL_GPIO_ConfigPinAttributes(GPIOA, GPIO_PIN_7, GPIO_PIN_SEC | GPIO_PIN_NPRIV);
	HAL_GPIO_ConfigPinAttributes(GPIOA, GPIO_PIN_8, GPIO_PIN_SEC | GPIO_PIN_NPRIV);
	HAL_GPIO_ConfigPinAttributes(GPIOB, GPIO_PIN_2, GPIO_PIN_SEC | GPIO_PIN_NPRIV);
	HAL_GPIO_ConfigPinAttributes(GPIOB, GPIO_PIN_11, GPIO_PIN_SEC | GPIO_PIN_NPRIV);
	HAL_GPIO_ConfigPinAttributes(GPIOB, GPIO_PIN_12, GPIO_PIN_SEC | GPIO_PIN_NPRIV);
	HAL_GPIO_ConfigPinAttributes(GPIOB, GPIO_PIN_13, GPIO_PIN_SEC | GPIO_PIN_NPRIV);
	HAL_GPIO_ConfigPinAttributes(GPIOB, GPIO_PIN_14, GPIO_PIN_SEC | GPIO_PIN_NPRIV);
	HAL_GPIO_ConfigPinAttributes(GPIOB, GPIO_PIN_15, GPIO_PIN_SEC | GPIO_PIN_NPRIV);
	HAL_GPIO_ConfigPinAttributes(GPIOC, GPIO_PIN_1, GPIO_PIN_SEC | GPIO_PIN_NPRIV);
	HAL_GPIO_ConfigPinAttributes(GPIOD, GPIO_PIN_8, GPIO_PIN_SEC | GPIO_PIN_NPRIV);
	HAL_GPIO_ConfigPinAttributes(GPIOD, GPIO_PIN_9, GPIO_PIN_SEC | GPIO_PIN_NPRIV);
	HAL_GPIO_ConfigPinAttributes(GPIOD, GPIO_PIN_15, GPIO_PIN_SEC | GPIO_PIN_NPRIV);
	HAL_GPIO_ConfigPinAttributes(GPIOE, GPIO_PIN_5, GPIO_PIN_SEC | GPIO_PIN_NPRIV);
	HAL_GPIO_ConfigPinAttributes(GPIOE, GPIO_PIN_6, GPIO_PIN_SEC | GPIO_PIN_NPRIV);
	HAL_GPIO_ConfigPinAttributes(GPIOE, GPIO_PIN_11, GPIO_PIN_SEC | GPIO_PIN_NPRIV);
	HAL_GPIO_ConfigPinAttributes(GPIOG, GPIO_PIN_0, GPIO_PIN_SEC | GPIO_PIN_NPRIV);
	HAL_GPIO_ConfigPinAttributes(GPIOG, GPIO_PIN_1, GPIO_PIN_SEC | GPIO_PIN_NPRIV);
	HAL_GPIO_ConfigPinAttributes(GPIOG, GPIO_PIN_6, GPIO_PIN_SEC | GPIO_PIN_NPRIV);
	HAL_GPIO_ConfigPinAttributes(GPIOG, GPIO_PIN_8, GPIO_PIN_SEC | GPIO_PIN_NPRIV);
	HAL_GPIO_ConfigPinAttributes(GPIOG, GPIO_PIN_11, GPIO_PIN_SEC | GPIO_PIN_NPRIV);
	HAL_GPIO_ConfigPinAttributes(GPIOG, GPIO_PIN_12, GPIO_PIN_SEC | GPIO_PIN_NPRIV);
	HAL_GPIO_ConfigPinAttributes(GPIOG, GPIO_PIN_13, GPIO_PIN_SEC | GPIO_PIN_NPRIV);
	HAL_GPIO_ConfigPinAttributes(GPIOG, GPIO_PIN_15, GPIO_PIN_SEC | GPIO_PIN_NPRIV);
	HAL_GPIO_ConfigPinAttributes(GPIOH, GPIO_PIN_3, GPIO_PIN_SEC | GPIO_PIN_NPRIV);
	HAL_GPIO_ConfigPinAttributes(GPIOH, GPIO_PIN_4, GPIO_PIN_SEC | GPIO_PIN_NPRIV);
	HAL_GPIO_ConfigPinAttributes(GPIOH, GPIO_PIN_6, GPIO_PIN_SEC | GPIO_PIN_NPRIV);
	HAL_GPIO_ConfigPinAttributes(GPIOH, GPIO_PIN_9, GPIO_PIN_SEC | GPIO_PIN_NPRIV);

	/* USER CODE BEGIN RIF_Init 1 */

	/* USER CODE END RIF_Init 1 */
	/* USER CODE BEGIN RIF_Init 2 */

	/* USER CODE END RIF_Init 2 */

}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void)
{

	/* USER CODE BEGIN USART1_Init 0 */

	/* USER CODE END USART1_Init 0 */

	/* USER CODE BEGIN USART1_Init 1 */

	/* USER CODE END USART1_Init 1 */
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 115200;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
	huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart1) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART1_Init 2 */

	/* USER CODE END USART1_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_GPIOG_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();

	/* USER CODE BEGIN MX_GPIO_Init_2 */

	/* LED GPIO Configuration
	 * PO1  -----> LED1 (Green)
	 * PG10	-----> LED2 (Red)
	 */
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	__HAL_RCC_GPIOO_CLK_ENABLE();
	GPIO_InitStruct.Pin = LED_GREEN_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(LED_GREEN_GPIO_Port, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = LED_RED_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(LED_RED_GPIO_Port, &GPIO_InitStruct);

//	HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
//	HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);

	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

#if defined(__ICCARM__)
/**
 * @brief  Retargets the C library __write function to the IAR function iar_fputc.
 * @param  file: file descriptor.
 * @param  ptr: pointer to the buffer where the data is stored.
 * @param  len: length of the data to write in bytes.
 * @retval length of the written data in bytes.
 */
size_t __write(int file, unsigned char const *ptr, size_t len)
{
	size_t idx;
	unsigned char const *pdata = ptr;

	for (idx = 0; idx < len; idx++) {
		iar_fputc((int)*pdata);
		pdata++;
	}
	return len;
}
#endif /* __ICCARM__ */

/**
 * @brief  Retargets the C library printf function to the USART.
 * @param  None
 * @retval None
 */
PUTCHAR_PROTOTYPE
{
	/* Place your implementation of fputc here */
	/* e.g. write a character to the USART1 and Loop until the end of transmission */
	HAL_UART_Transmit(&huart1, (uint8_t*) &ch, 1, 1000U);
	return ch;
}

GETCHAR_PROTOTYPE
{
	uint8_t ch = 0;
	/* Place your implementation of fgetc here */

	/* Clear the Overrun flag just before receiving the first character */
	__HAL_UART_CLEAR_OREFLAG(&huart1);

	/* Wait for reception of a character on the USART1 RX line
	 and echo this character on console */
	HAL_UART_Receive(&huart1, (uint8_t*) &ch, 1, 0xFFFF);
	HAL_UART_Transmit(&huart1, (uint8_t*) &ch, 1, 0xFFFF);
	return ch;
}

/**
 * @brief  Rx Transfer completed callback
 * @note   This example shows a simple way to report end of IT Rx transfer, and
 *         you can add your own implementation.
 * @retval None
 */
void UART_CharReception_Callback(void)
{
	/* Read Received character. RXNE flag is cleared by reading of RDR register */
	aRxBuffer[uwRxIndex++] = LL_USART_ReceiveData8(USART1);
	printf("%c", aRxBuffer[(uwRxIndex - 1)]);

	/* Check if reception is completed (expected nb of bytes has been received) */
	if (aRxBuffer[(uwRxIndex - 1)] == '\b') {
		aRxBuffer[(uwRxIndex - 1)] = 0;
		if (--uwRxIndex == 0) return;
		aRxBuffer[(uwRxIndex - 1)] = 0;
		printf(" \b");
		uwRxIndex--;
	}
	else if ((uwRxIndex == RX_BUF_SIZE) || aRxBuffer[(uwRxIndex - 1)] == '\r') {
		/* Set Reception complete boolean to 1 */
		aRxBuffer[(uwRxIndex - 1)] = '\r';
		ubRxComplete = 1;
		printf("\n");
	}
	fflush(stdout);
}

/**
 * @brief  UART error callbacks
 * @note   This example shows a simple way to report transfer error, and you can
 *         add your own implementation.
 * @retval None
 */
void UART_Error_Callback(void)
{
	__IO uint32_t isr_reg;

	/* Disable USARTx_IRQn */
	NVIC_DisableIRQ(USART1_IRQn);

	/* Error handling example :
	 - Read USART ISR register to identify flag that leads to IT raising
	 - Perform corresponding error handling treatment according to flag
	 */
	isr_reg = LL_USART_ReadReg(USART1, ISR);
	if (isr_reg & LL_USART_ISR_NE) {
		printf("[  ERROR ] %s(): uart noise error detection flag\n", __func__);
		fflush(stdout);
	}
}

void HAL_DCMIPP_PIPE_FrameEventCallback(DCMIPP_HandleTypeDef *hdcmipp, uint32_t Pipe)
{
	UNUSED(hdcmipp);
	UNUSED(Pipe);

	NbMainFrames++;
//	printf("[   INFO ] %s(): new frame!\n", __func__);
//	fflush(stdout);
}

/**
 * @brief  Vsync Event callback on pipe
 * @param  hdcmipp DCMIPP device handle
 *         Pipe    Pipe receiving the callback
 * @retval None
 */
void HAL_DCMIPP_PIPE_VsyncEventCallback(DCMIPP_HandleTypeDef *hdcmipp, uint32_t Pipe)
{
	UNUSED(hdcmipp);
	/* Update the frame counter and call the ISP statistics handler */
	switch (Pipe) {
		case DCMIPP_PIPE0:
//			printf("[   INFO ] %s(): DCMIPP_PIPE0\n", __func__);
			break;
	}
//	fflush(stdout);
}

void HAL_DCMIPP_PIPE_ErrorCallback(DCMIPP_HandleTypeDef *hdcmipp, uint32_t Pipe)
{
	UNUSED(hdcmipp);

	printf("[  ERROR ] %s(): pipe %ld\n", __func__, Pipe);
	fflush(stdout);

	__HAL_DCMIPP_ENABLE_IT(hdcmipp, DCMIPP_IT_PIPE0_OVR);
}

void HAL_DCMIPP_ErrorCallback(DCMIPP_HandleTypeDef *hdcmipp)
{
	UNUSED(hdcmipp);

	printf("[  ERROR ] %s()\n", __func__);
	fflush(stdout);
}

/* MPU Configuration */
void MPU_Config(void)
{
	MPU_Region_InitTypeDef default_config = { 0 };
	MPU_Attributes_InitTypeDef attr_config = { 0 };
	uint32_t primask_bit = __get_PRIMASK();
	__disable_irq();

	/* disable the MPU */
	HAL_MPU_Disable();

	/* create an attribute configuration for the MPU */
	attr_config.Attributes = INNER_OUTER(MPU_DEVICE_NGNRNE);
	attr_config.Number = MPU_ATTRIBUTES_NUMBER0;

	HAL_MPU_ConfigMemoryAttributes(&attr_config);

	/* Create a non cacheable region */
	/*Normal memory type, code execution allowed */

	default_config.Enable = MPU_REGION_ENABLE;
	default_config.Number = MPU_REGION_NUMBER0;
	default_config.BaseAddress = 0x341F8000;
	default_config.LimitAddress = 0x341F817F;
	default_config.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
	default_config.AccessPermission = MPU_REGION_ALL_RW;
	default_config.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
	default_config.AttributesIndex = MPU_ATTRIBUTES_NUMBER0;
	HAL_MPU_ConfigRegion(&default_config);

	/* MPU Region 1 Config */
	attr_config.Attributes = INNER_OUTER(MPU_RW_ALLOCATE);
	attr_config.Number = MPU_ATTRIBUTES_NUMBER1;

	HAL_MPU_ConfigMemoryAttributes(&attr_config);

	default_config.Enable = MPU_REGION_ENABLE;
	default_config.Number = MPU_REGION_NUMBER1;
	default_config.BaseAddress = 0x341F8180;
	default_config.LimitAddress = 0x341FFFFF;
	default_config.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
	default_config.AccessPermission = MPU_REGION_ALL_RW;
	default_config.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
	default_config.AttributesIndex = MPU_ATTRIBUTES_NUMBER1;
	HAL_MPU_ConfigRegion(&default_config);

	/* enable the MPU */
	HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

	/* Exit critical section to lock the system and avoid any issue around MPU mechanisme */
	__set_PRIMASK(primask_bit);
}

static int Command_Execute(uint8_t *commandData, uint16_t commandSize)
{
	int32_t ret = 0;
	uint16_t index = 0;
	uint16_t devAddr = 0;
	uint16_t regAddr = 0;
	uint16_t regSize = 0;
	uint8_t regData[COMMAND_I2C_DATA_BUF_SIZE] = { 0 };
	uint16_t dataSize = 0;
	uint8_t isDevAddr = 0;
	uint8_t isRegAddr = 0;
	uint8_t isRegData = 0;

	if (commandData[0] == 'w' || commandData[0] == 'W') {
		printf("[   INFO ] %s(): i2c write command\n", __func__);

		while (index++ < commandSize) {
			if (commandData[index] == ' ') {
				if (isDevAddr == 0 && isRegAddr == 0 && isRegData == 0) {
					isDevAddr = 1;
				}
				else if (isDevAddr) {
					isDevAddr = 0;
					isRegAddr = 1;
				}
				else if (isRegAddr) {
					isRegAddr = 0;
					isRegData = 1;
				}
			}

			if (isDevAddr) {
				if (commandData[index] == '0' && commandData[index + 1] == 'x') {
					devAddr = (uint16_t) strtol((char*) &commandData[index], NULL, 16);
					index += 3;
					printf("[   INFO ] %s(): devAddr = 0x%02X\n", __func__, devAddr);
				}
			}
			else if (isRegAddr) {
				if (commandData[index] == '0' && commandData[index + 1] == 'x') {
					regAddr = (uint16_t) strtol((char*) &commandData[index], NULL, 16);
					if (commandData[index + 4] == ' ') {
						regSize = 1;
						index += 3;
					}
					else if (commandData[index + 6] == ' ') {
						regSize = 2;
						index += 5;
					}
					else {
						printf("[  ERROR ] %s(): register size should be 1 byte or 2 bytes\n", __func__);
						return -22;
					}
					printf("[   INFO ] %s(): regAddr [%d] = 0x%04X\n", __func__, regSize, regAddr);
				}
			}
			else if (isRegData) {
				if (commandData[index] == '0' && commandData[index + 1] == 'x') {
					if (dataSize == COMMAND_I2C_DATA_BUF_SIZE) {
						printf("[  ERROR ] %s(): data size only support %d\n", __func__, COMMAND_I2C_DATA_BUF_SIZE);
						return -22;
					}
					regData[dataSize++] = (uint16_t) strtol((char*) &commandData[index], NULL, 16);
					index += 3;
					printf("[   INFO ] %s(): regData [%d] = 0x%02X\n", __func__, dataSize, regData[dataSize - 1]);
				}

			}
		}
		ret = I2C1_WriteReg((devAddr << 1), regAddr, regSize, regData, dataSize);
	}
	else if (commandData[0] == 'r' || commandData[0] == 'R') {
		printf("[   INFO ] %s(): i2c read command\n", __func__);

		while (index++ < commandSize) {
			if (commandData[index] == ' ') {
				if (isDevAddr == 0 && isRegAddr == 0 && isRegData == 0) {
					isDevAddr = 1;
				}
				else if (isDevAddr) {
					isDevAddr = 0;
					isRegAddr = 1;
				}
				else if (isRegAddr) {
					isRegAddr = 0;
					isRegData = 1;
				}
				index++;
			}

			if (isDevAddr) {
				if (commandData[index] == '0' && commandData[index + 1] == 'x') {
					devAddr = (uint16_t) strtol((char*) &commandData[index], NULL, 16);
					index += 3;
					printf("[   INFO ] %s(): devAddr = 0x%02X\n", __func__, devAddr);
				}
			}
			else if (isRegAddr) {
				if (commandData[index] == '0' && commandData[index + 1] == 'x') {
					regAddr = (uint16_t) strtol((char*) &commandData[index], NULL, 16);
					if (commandData[index + 4] == ' ') {
						regSize = 1;
						index += 3;
					}
					else if (commandData[index + 6] == ' ') {
						regSize = 2;
						index += 5;
					}
					else {
						printf("[  ERROR ] %s(): register size should be 1 byte or 2 bytes\n", __func__);
						return -22;
					}
					printf("[   INFO ] %s(): regAddr [%d] = 0x%04X\n", __func__, regSize, regAddr);
				}
			}
			else if (isRegData) {
				dataSize = (uint16_t) strtol((char*) &commandData[index], NULL, 10);
				if (dataSize > COMMAND_I2C_DATA_BUF_SIZE) {
					printf("[  ERROR ] %s(): data size only support %d\n", __func__, COMMAND_I2C_DATA_BUF_SIZE);
					return -22;
				}
				printf("[   INFO ] %s(): regData [%d]\n", __func__, dataSize);
				break;
			}
		}
		ret = I2C1_ReadReg((devAddr << 1), regAddr, regSize, regData, dataSize);
		for (index = 0; index < dataSize; index++) {
			printf("[   INFO ] %s(): data [%d] = 0x%02X \n", __func__, index, regData[index]);
		}
	}
	else if (commandData[0] == 's') {
		if (!ubCameraEnable) {
			ret = TEVS_Set_Stream(1);
			if (ret != 0) {
				printf("[  ERROR ] %s(): TEVS set stream failed, error = %ld\n", __func__, ret);
				Error_Handler();
			}
			fflush(stdout);

			ret = HAL_DCMIPP_PIPE_Resume(&hdcmipp, DCMIPP_PIPE0);
			if (ret != 0) {
				printf("[  ERROR ] %s(): DCMIPP PIPE0 suspend failed, error = %ld\n", __func__, ret);
				Error_Handler();
			}
		}
		else {
			ret = TEVS_Set_Stream(0);
			if (ret != 0) {
				printf("[  ERROR ] %s(): TEVS set stream failed, error = %ld\n", __func__, ret);
				Error_Handler();
			}
			fflush(stdout);

			ret = HAL_DCMIPP_PIPE_Suspend(&hdcmipp, DCMIPP_PIPE0);
			if (ret != 0) {
				printf("[  ERROR ] %s(): DCMIPP PIPE0 suspend failed, error = %ld\n", __func__, ret);
				Error_Handler();
			}
		}
	}
	else if (commandData[0] != '\r' && commandData[0] != '\0') {
		printf("[  ERROR ] %s(): unknow command\n", __func__);
	}

	fflush(stdout);
	return ret;
}

static int I2C1_WriteReg(uint16_t devAddr, uint16_t reg, uint16_t regSize, uint8_t *data, uint16_t dataSize)
{
	int32_t ret = HAL_I2C_ERROR_NONE;

	if (HAL_I2C_Mem_Write(&hi2c1, devAddr, reg, regSize, data, dataSize, 1000) != HAL_OK) {
		ret = HAL_I2C_GetError(&hi2c1);
	}

	return ret;
}

static int I2C1_ReadReg(uint16_t devAddr, uint16_t reg, uint16_t regSize, uint8_t *data, uint16_t dataSize)
{
	int32_t ret = HAL_I2C_ERROR_NONE;

	if (HAL_I2C_Mem_Read(&hi2c1, devAddr, reg, regSize, data, dataSize, 1000) != HAL_OK) {
		ret = HAL_I2C_GetError(&hi2c1);
	}

	return ret;
}

static int TEVS_I2C_WriteReg16(uint16_t reg, uint16_t data)
{
	uint8_t pData[2] = { 0 };

	pData[0] = (data >> 8) & 0xFF;
	pData[1] = data & 0xFF;

	return I2C1_WriteReg(TEVS_ADDRESS, reg, I2C_MEMADD_SIZE_16BIT, pData, 2);
}

static int TEVS_I2C_ReadReg16(uint16_t reg, uint16_t *data)
{
	int32_t ret = HAL_I2C_ERROR_NONE;
	uint8_t pData[2] = { 0 };

	ret = I2C1_ReadReg(TEVS_ADDRESS, reg, I2C_MEMADD_SIZE_16BIT, pData, 2);
	*data = ((uint16_t) pData[0] << 8) | pData[1];

	return ret;
}

static int TEVS_GPIO_Init(void)
{
	int32_t ret = 0;
	uint8_t data;

#ifdef DEBUG
	printf("[  DEBUG ] %s()\n", __func__);
	fflush(stdout);
#endif
	data = 0xBF;
	ret = I2C1_WriteReg(IO_EXPANDER_ADDRESS, 0x01, I2C_MEMADD_SIZE_8BIT, &data, 1);
	ret += I2C1_WriteReg(IO_EXPANDER_ADDRESS, 0x03, I2C_MEMADD_SIZE_8BIT, &data, 1);

	return ret;
}

static int TEVS_ResetPin_Ctrl(uint8_t state)
{
#ifdef DEBUG
	printf("[  DEBUG ] %s(): state=%d\n", __func__, state);
	fflush(stdout);
#endif
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, state); /* PC8  CAM_NRST */
	return 0;
}

static int TEVS_StandbyPin_Ctrl(uint8_t state)
{
	uint8_t data;

#ifdef DEBUG
	printf("[  DEBUG ] %s(): state=%d\n", __func__, state);
	fflush(stdout);
#endif
	data = 0xBF | (state & 0x01) << 6;
	return I2C1_WriteReg(IO_EXPANDER_ADDRESS, 0x01, I2C_MEMADD_SIZE_8BIT, &data, 1);
}

static int TEVS_Check_BootState(void)
{
	uint16_t boot_state;
	uint16_t timeout = 0;
	int32_t ret = 0;

#ifdef DEBUG
	printf("[  DEBUG ] %s()\n", __func__);
	fflush(stdout);
#endif
	while (timeout < 20) {
		TEVS_I2C_ReadReg16(HOST_COMMAND_TEVS_BOOT_STATE, &boot_state);
		printf("[   INFO ] %s(): bootup state: 0x%04X\n", __func__, boot_state);
		if (boot_state == TEVS_BOOT_STATE_NORMAL) break;
		if (++timeout >= 20) {
			printf("[  ERROR ] %s(): bootup timeout: state: 0x%04X\n", __func__, boot_state);
			ret = -22;
		}
		HAL_Delay(20);
	}
	fflush(stdout);
	return ret;
}

static int TEVS_Init(void)
{
	int32_t ret = 0;
	uint16_t ver[2] = { 0 };

#ifdef DEBUG
	printf("[  DEBUG ] %s()\n", __func__);
	fflush(stdout);
#endif

	ret = TEVS_StandbyPin_Ctrl(0);
	if (ret != 0) {
		printf("[  ERROR ] %s(): set TEVS standby pin failed, error = %ld\n", __func__, ret);
		return -22;
	}
	ret = TEVS_ResetPin_Ctrl(0);
	if (ret != 0) {
		printf("[  ERROR ] %s(): set TEVS reset pin failed, error = %ld\n", __func__, ret);
		return -22;
	}
	HAL_Delay(10);
	ret = TEVS_ResetPin_Ctrl(1);
	if (ret != 0) {
		printf("[  ERROR ] %s(): set TEVS reset pin failed, error = %ld\n", __func__, ret);
		return -22;
	}
	HAL_Delay(TEVS_BOOT_TIME);
	ret = TEVS_Check_BootState();
	if (ret != 0) {
		printf("[  ERROR ] %s(): TEVS startup failed, error = %ld\n", __func__, ret);
		return -22;
	}

	printf("[   INFO ] %s(): set TEVS default data frequency\n", __func__);
	ret = TEVS_I2C_WriteReg16(HOST_COMMAND_ISP_CTRL_MIPI_FREQ, TEVS_DEF_DATA_FREQ);
	if (ret != 0) {
		printf("[  ERROR ] %s(): set TEVS mipi data frequency failed, error = %ld\n", __func__, ret);
		return -22;
	}
	HAL_Delay(TEVS_BOOT_TIME);
	ret = TEVS_Check_BootState();
	if (ret != 0) {
		printf("[  ERROR ] %s(): TEVS startup failed, error = %ld\n", __func__, ret);
		return -22;
	}

	printf("[   INFO ] %s(): set TEVS preview format UYVY\n", __func__);
	ret = TEVS_I2C_WriteReg16(HOST_COMMAND_ISP_CTRL_PREVIEW_FORMAT, PREVIEW_FORMAT_UYVY);
	if (ret != 0) {
		printf("[  ERROR ] %s(): set TEVS preview format failed, error = %ld\n", __func__, ret);
		return -22;
	}

	printf("[   INFO ] %s(): set TEVS 2 lanes\n", __func__);
	ret = TEVS_I2C_WriteReg16(HOST_COMMAND_ISP_CTRL_PREVIEW_HINF_CTRL, PREVIEW_HINF_CTRL);
	if (ret != 0) {
		printf("[  ERROR ] %s(): set TEVS preview hinf failed, error = %ld\n", __func__, ret);
		return -22;
	}

	ret = TEVS_Set_Stream(0);
	if (ret != 0) {
		printf("[  ERROR ] %s(): TEVS set stream failed, error = %ld\n", __func__, ret);
		return -22;
	}

	ret = TEVS_I2C_ReadReg16(HOST_COMMAND_TEVS_INFO_VERSION_MSB, &ver[0]);
	if (ret != 0) {
		printf("[  ERROR ] %s(): TEVS read version MSB failed, error = %ld\n", __func__, ret);
		return -22;
	}

	ret = TEVS_I2C_ReadReg16(HOST_COMMAND_TEVS_INFO_VERSION_LSB, &ver[1]);
	if (ret != 0) {
		printf("[  ERROR ] %s(): TEVS read version LSB failed, error = %ld\n", __func__, ret);
		return -22;
	}

	printf("[   INFO ] %s(): TEVS version: %02d.%02d.%02d.%02d\n", __func__,
			((ver[0] >> 8) & 0xFF), (ver[0] & 0xFF),
	        ((ver[1] >> 8) & 0xFF), (ver[1] & 0xFF));

	fflush(stdout);
	return ret;
}

static int TEVS_Set_Format(uint16_t width, uint16_t height, uint16_t fps)
{
	int32_t ret = 0;

#ifdef DEBUG
	printf("[  DEBUG ] %s()\n", __func__);
	fflush(stdout);
#endif

	ret = TEVS_I2C_WriteReg16(HOST_COMMAND_ISP_CTRL_PREVIEW_WIDTH, width);
	if (ret != 0) {
		printf("[  ERROR ] %s(): set TEVS preview width failed, error = %ld\n", __func__, ret);
		return -22;
	}

	ret = TEVS_I2C_WriteReg16(HOST_COMMAND_ISP_CTRL_PREVIEW_HEIGHT, height);
	if (ret != 0) {
		printf("[  ERROR ] %s(): set TEVS preview heigth failed, error = %ld\n", __func__, ret);
		return -22;
	}

	ret = TEVS_I2C_WriteReg16(HOST_COMMAND_ISP_CTRL_PREVIEW_SENSOR_MODE, PREVIEW_SENSOR_MODE);
	if (ret != 0) {
		printf("[  ERROR ] %s(): set TEVS preview sensor mode failed, error = %ld\n", __func__, ret);
		return -22;
	}

	ret = TEVS_I2C_WriteReg16(HOST_COMMAND_ISP_CTRL_PREVIEW_MAX_FPS, fps);
	if (ret != 0) {
		printf("[  ERROR ] %s(): set TEVS preview max fps failed, error = %ld\n", __func__, ret);
		return -22;
	}

	printf("[   INFO ] %s(): TEVS resolution: %dx%d@%d\n", __func__, width, height, fps);

	fflush(stdout);
	return ret;
}

static int TEVS_Set_Stream(uint8_t enable)
{
	uint16_t v = 0xFFFF;
	uint16_t timeout = 0;

#ifdef DEBUG
	printf("[  DEBUG ] %s(): enable=%d\n", __func__, enable);
	fflush(stdout);
#endif

	if (enable == 0) {
		TEVS_I2C_WriteReg16(HOST_COMMAND_ISP_CTRL_SYSTEM_START, 0x0000);
		while (timeout < 100) {
			HAL_Delay(10);
			TEVS_I2C_ReadReg16(HOST_COMMAND_ISP_CTRL_SYSTEM_START, &v);
			if ((v & 0xFF00) == 0x0000) break;
			if (++timeout >= 100) {
				printf("[  ERROR ] %s(): timeout: value=0x%04X\n", __func__, v);
				fflush(stdout);
				return -22;
			}
		}
		printf("[   INFO ] %s(): sensor standby\n", __func__);
	}
	else {
		TEVS_I2C_WriteReg16(HOST_COMMAND_ISP_CTRL_SYSTEM_START, 0x0001);
		while (timeout < 100) {
			HAL_Delay(10);
			TEVS_I2C_ReadReg16(HOST_COMMAND_ISP_CTRL_SYSTEM_START, &v);
			if ((v & 0xFF00) == 0x0100) break;
			if (++timeout >= 100) {
				printf("[  ERROR ] %s(): timeout: value=0x%04X\n", __func__, v);
				fflush(stdout);
				return -22;
			}
		}
		printf("[   INFO ] %s(): sensor wakeup\n", __func__);

		TEVS_Set_Format(PREVIEW_WIDTH, PREVIEW_HEIGHT, PREVIEW_FPS);
	}
	ubCameraEnable = enable;

	fflush(stdout);
	return 0;
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
//	__disable_irq();
	printf(" !!!!!!!!!! Error !!!!!!!!!!\n");
	fflush(stdout);
	HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
	while (1) {
		HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin);
		HAL_Delay(1000);
	}
	/* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
