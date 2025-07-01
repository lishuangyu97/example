/***
	*************************************************************************************************
	*	@file  	main.c
	*	@version V1.0
	*  @date    2023-4-10
	*	@author  反客科技
   ************************************************************************************************
   *  @description
	*
	*	实验平台：反客STM32H723ZGT6核心板 （型号：FK723M1-ZGT6）
	*	淘宝地址：https://shop212360197.taobao.com
	*	QQ交流群：536665479
	*
>>>>> 功能说明：
	*
	*	1.点亮LED，使用HAL库自带的延时函数实现闪烁
	*	2.使用 OSPI1 驱动W25Q64，进行简单的读写测试
	*	3.OSPI间接模式，使用库函数直接读写，
	*	4.默认配置QSPI驱动时钟为137.5M
	*
>>>>> 串口打印说明：
	*
	*	USART1使用的是PA9/PA10，串口波特率115200
	*	
	************************************************************************************************
***/

#include "main.h"
#include "led.h"
#include "usart.h"
#include "ospi_w25q64.h"

/********************************************** 变量定义 *******************************************/

#define W25Qxx_NumByteToTest   	32*1024					// 测试数据的长度，32K

int32_t OSPI_Status ; 		 //检测标志位

uint8_t  W25Qxx_WriteBuffer[W25Qxx_NumByteToTest];		//	写数据数组
uint8_t  W25Qxx_ReadBuffer[W25Qxx_NumByteToTest];		//	读数据数组




/***************************************************************************************************
*	函 数 名: OSPI_W25Qxx_Test
*	入口参数: 无
*	返 回 值: OSPI_W25Qxx_OK - 测试成功并通过
*	函数功能: 进行简单的读写测试，并计算速度
*	说    明: 无	
***************************************************************************************************/

int8_t OSPI_W25Qxx_Test(void)		//Flash读写测试
{
	uint32_t i = 0X8000;	// 计数变量
	uint32_t W25Qxx_TestAddr  =	0	;							// 测试地址	
	uint32_t ExecutionTime_Begin;		// 开始时间
	uint32_t ExecutionTime_End;		// 结束时间
	uint32_t ExecutionTime;				// 执行时间	
	float    ExecutionSpeed;			// 执行速度

// 擦除 >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   
	
	
	ExecutionTime_Begin 	= HAL_GetTick();	// 获取 systick 当前时间，单位ms
	OSPI_Status 			= OSPI_W25Qxx_BlockErase_32K(W25Qxx_TestAddr);	// 擦除32K字节
	ExecutionTime_End		= HAL_GetTick();	// 获取 systick 当前时间，单位ms
	
	ExecutionTime = ExecutionTime_End - ExecutionTime_Begin; // 计算擦除时间，单位ms
	
	if( OSPI_Status == OSPI_W25Qxx_OK )
	{
		printf ("\r\nW25Q64 擦除成功, 擦除32K字节所需时间: %d ms\r\n",ExecutionTime);		
	}
	else
	{
		printf ("\r\n 擦除失败!!!!!  错误代码:%d\r\n",OSPI_Status);
		while (1);
	}	
	
// 写入 >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

	for(i=0;i<W25Qxx_NumByteToTest;i++)  //先将数据写入数组
	{
		W25Qxx_WriteBuffer[i] = i;
	}
	ExecutionTime_Begin 	= HAL_GetTick();	// 获取 systick 当前时间，单位ms
	OSPI_Status				= OSPI_W25Qxx_WriteBuffer(W25Qxx_WriteBuffer,W25Qxx_TestAddr,W25Qxx_NumByteToTest); // 写入数据
	ExecutionTime_End		= HAL_GetTick();	// 获取 systick 当前时间，单位ms
	
	ExecutionTime  = ExecutionTime_End - ExecutionTime_Begin; 		// 计算擦除时间，单位ms
	ExecutionSpeed = (float)W25Qxx_NumByteToTest / ExecutionTime ; // 计算写入速度，单位 KB/S
	if( OSPI_Status == OSPI_W25Qxx_OK )
	{
		printf ("\r\n写入成功,数据大小：%d KB, 耗时: %d ms, 写入速度：%.2f KB/s\r\n",W25Qxx_NumByteToTest/1024,ExecutionTime,ExecutionSpeed);		
	}
	else
	{
		printf ("\r\n写入错误!!!!!  错误代码:%d\r\n",OSPI_Status);
		while (1);
	}	
	
// 读取	>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> 

	
	ExecutionTime_Begin 	= HAL_GetTick();	// 获取 systick 当前时间，单位ms	
	OSPI_Status				= OSPI_W25Qxx_ReadBuffer(W25Qxx_ReadBuffer,W25Qxx_TestAddr,W25Qxx_NumByteToTest);	// 读取数据
	ExecutionTime_End		= HAL_GetTick();	// 获取 systick 当前时间，单位ms
	
	ExecutionTime  = ExecutionTime_End - ExecutionTime_Begin; 					// 计算擦除时间，单位ms
	ExecutionSpeed = (float)W25Qxx_NumByteToTest/1024/1024 / ExecutionTime*1000 ; 	// 计算读取速度，单位 MB/S 
	
	if( OSPI_Status == OSPI_W25Qxx_OK )
	{
		printf ("\r\n读取成功,数据大小：%d KB, 耗时: %d ms, 读取速度：%.2f MB/s \r\n",W25Qxx_NumByteToTest/1024,ExecutionTime,ExecutionSpeed);		
	}
	else
	{
		printf ("\r\n读取错误!!!!!  错误代码:%d\r\n",OSPI_Status);
		while (1);
	}			
// 数据校验 >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   
	
	for(i=0;i<W25Qxx_NumByteToTest;i++)	//验证读出的数据是否等于写入的数据
	{
		if( W25Qxx_WriteBuffer[i] != W25Qxx_ReadBuffer[i] )	//如果数据不相等，则返回0	
		{
			printf ("\r\n数据校验失败!!!!!出错位置：%d\r\n",i);	
			while(1);
		}
	}			
	printf ("\r\n校验通过!!!!! QSPI驱动W25Q64测试正常\r\n");		
	
// 读取整片Flash的数据，用以测试速度 >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
	
	printf ("\r\n*****************************************************************************************************\r\n");		
	printf ("\r\n上面的测试中，读取的数据比较小，耗时很短，加之测量的最小单位为ms，计算出的读取速度误差较大\r\n");		
	printf ("\r\n接下来读取整片flash的数据用以测试速度，这样得出的速度误差比较小\r\n");		
	printf ("\r\n开始读取>>>>\r\n");		
	ExecutionTime_Begin 	= HAL_GetTick();	// 获取 systick 当前时间，单位ms		
	
	for(i=0;i<W25Qxx_FlashSize/(W25Qxx_NumByteToTest);i++)	// 每次读取 W25Qxx_NumByteToTest 字节的数据
	{
		OSPI_Status				= OSPI_W25Qxx_ReadBuffer(W25Qxx_ReadBuffer,W25Qxx_TestAddr,W25Qxx_NumByteToTest);	// 读取数据
		W25Qxx_TestAddr = W25Qxx_TestAddr + W25Qxx_NumByteToTest;		
	}
	ExecutionTime_End		= HAL_GetTick();	// 获取 systick 当前时间，单位ms
	
	ExecutionTime  = ExecutionTime_End - ExecutionTime_Begin; 								// 计算擦除时间，单位ms
	ExecutionSpeed = (float)W25Qxx_FlashSize/1024/1024 / ExecutionTime*1000  ; 	// 计算读取速度，单位 MB/S 

	if( OSPI_Status == OSPI_W25Qxx_OK )
	{
		printf ("\r\n读取成功,数据大小：%d MB, 耗时: %d ms, 读取速度：%.2f MB/s \r\n",W25Qxx_FlashSize/1024/1024,ExecutionTime,ExecutionSpeed);		
	}
	else
	{
		printf ("\r\n读取错误!!!!!  错误代码:%d\r\n",OSPI_Status);
		while (1);
	}	
	
	return OSPI_W25Qxx_OK ;  // 测试通过				
	
}


/********************************************** 函数声明 *******************************************/

void SystemClock_Config(void);		// 时钟初始化

/***************************************************************************************************
*	函 数 名: main
*	入口参数: 无
*	返 回 值: 无
*	函数功能: 运行主程序
*	说    明: 无
****************************************************************************************************/

int main(void)
{
	SCB_EnableICache();		// 使能ICache
	SCB_EnableDCache();		// 使能DCache
	HAL_Init();					// 初始化HAL库
	SystemClock_Config();	// 配置系统时钟，主频550MHz
	LED_Init();					// 初始化LED引脚
	USART1_Init();				// USART1初始化	

	OSPI_W25Qxx_Init();		// 初始化OSPI和W25Q64
	
	OSPI_W25Qxx_Test();		// Flash读写测试
	
	while (1)
	{
		LED1_Toggle;
		HAL_Delay(100);
	}
}


/****************************************************************************************************/
/**
  *   系统时钟配置：
  *
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 550000000 (CPU 主频 550MHz)
  *            HCLK(Hz)                       = 275000000 (AXI and AHBs Clock)
  *            AHB Prescaler                  = 1 (AHB  Clock  275 MHz)
  *            D1 APB3 Prescaler              = 2 (APB3 Clock  137.5MHz)
  *            D2 APB1 Prescaler              = 2 (APB1 Clock  137.5MHz)
  *            D2 APB2 Prescaler              = 2 (APB2 Clock  137.5MHz)
  *            D3 APB4 Prescaler              = 2 (APB4 Clock  137.5MHz)
  *            HSE Frequency(Hz)              = 25000000  (外部晶振频率)
  *            PLL_M                          = 10
  *            PLL_N                          = 220
  *            PLL_P                          = 1
  *
  *				CPU主频 = HSE Frequency / PLL_M * PLL_N / PLL_P = 25M /10*220/1 = 550M
  */   
/****************************************************************************************************/


void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
  
  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 10;
  RCC_OscInitStruct.PLL.PLLN = 220;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_1;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
  
 
	PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_OSPI;    
    
	/* 为了设置方便，将 高速时钟HCLK3 作为OSPI的内核时钟，速度为275M，再经过2分频得到 137.5M 的驱动时钟  */  
	/* 虽然W25Q64JV 所允许的最高时钟为 133M */
	/* 但实际测试中，将驱动时钟拉到将近200M，w25q64还是可以正常读写，所以只超出4.5M的频率不用担心稳定性*/
   /* 若用户使用厂家严苛，可以重新配置内核时钟 */
 	PeriphClkInitStruct.OspiClockSelection = RCC_OSPICLKSOURCE_D1HCLK; 
  
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
	{ 
		Error_Handler();
	}      
}



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

