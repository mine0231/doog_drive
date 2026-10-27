/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "adc.h"
#include "dma.h"
#include "fdcan.h"
#include "iwdg.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "motor_control.h"
#include "foc.h"
#include "drv8353.h"
#include "fdcan_app.h"
#include "uart_cmd.h"
#include "dwt_delay.h"
#include "kth7823.h"    
#include "kth7823_2.h"
#include "dbg_monitor.h"
#include "protect.h"     


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

volatile uint8_t fault_triggered = 0;							// 驱动故障标志
volatile uint8_t  fault_status_valid = 0;					// 1=上面STATUS1/2是本次 nFAULT 读到的；0=本次没读到，值不可信
volatile uint8_t  fault_code = FAULT_CODE_NONE;		// 故障期间保持，离开 FAULT 态时由 FOC_SetMode 清零
volatile uint16_t fault_mask = 0;									// 本次故障事件内触发过的所有保护位（bit=FAULT_CODE_n），与 fault_code 同步清零
volatile uint16_t fault_status1 = 0;							// 硬件故障 STATUS1 锁存（Disable 前读取）
volatile uint16_t fault_status2 = 0;							// 硬件故障 STATUS2 锁存

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

// 驱动故障回调函数
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin != GPIO_PIN_12) return;  // 处理 DRV8353 nFault 引脚
	
    fault_code = FAULT_CODE_HARDWARE;
		Protect_StopPWM();

    if(hspi2.State == HAL_SPI_STATE_READY) {
			// 先锁存硬件故障详情（此时 EN 尚未拉低、SPI 可用，读到的才是真实故障位）
			fault_status1 = DRV8353_ReadReg(DRV8353_REG_STATUS1);
			fault_status2 = DRV8353_ReadReg(DRV8353_REG_STATUS2);
			fault_status_valid = 1;
		} else {
      fault_status_valid = 0;
    }

    DRV8353_Disable();                        
    fault_triggered = 1;
}

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

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
	
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  DWT_Init();   // 使能内核 DWT Cycle 计数器

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_ADC1_Init();
  MX_FDCAN2_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_SPI3_Init();
  MX_USART3_UART_Init();
  MX_ADC2_Init();
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */
	
	FDCAN_App_Init();
	UART_Cmd_Init();
	MotorControl_Init();
	MotorControl_BootSequence();   // 上电自动执行启动序列
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	FOC_SetOpenLoopVoltage(0.0f);  // 开环初始电压 0V（速度默认 0，上电不转）
	
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

		// 故障恢复
		FOC_FaultRecoverProcess();

		// 调试监控快照：集中刷新关键运行变量到 dbg，供 KEIL Watch 观察
//		DBG_Monitor_Update();
		
		// 温度更新
		TempSense_Update();
		
		// 处理 CAN 消息（多节点：只处理发给本节点 NODE_ID 的帧，过滤器已按 ID 区分）
    FDCAN_App_Process();
		
		// can工作时屏蔽串口
		if (!Protect_CommIsArmed()){
			
			// 硬件配置自检结果串口上报
			UART_ReportAlignResult();			
			
			// log=1实时状态串口上报
			UART_SendStatus();
		
			// 故障信息串口上报
			UART_FaultReport();
			
			// 处理串口命令
			UART_Cmd_Process();

			// 自动停止：v= 设为非0 后 3 秒自动把电压归零（测试保护，防忘记停机过热）
			UART_Cmd_AutoStop();
			
		}else {
			UART_Cmd_Discard();      // 丢弃屏蔽期间累积的串口输入，防解除后执行陈旧命令
		}
		
		// 非阻塞发送兜底：发送链路异常（HAL 启动失败/出错）时恢复，正常无开销
		UART_Tx_Poll();
		
		// 编码器刷新，主循环做滤波角度（sin/cos + atan2，约 7µs，按 1kHz 跑）
		kth7823_SlowUpdate();
		kth7823_2_SlowUpdate();   // 输出轴编码器单圈滤波角度
		DWT_DelayMs(1);          // 定 1ms 节拍

		// 喂独立看门狗：主循环 1ms 周期喂狗，远小于 1s 超时
		HAL_IWDG_Refresh(&hiwdg);
		 
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 18;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
