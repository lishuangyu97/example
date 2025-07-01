/***
	*************************************************************************************************
	*	@file  	main.c
	*	@version V1.0
	*  @date    2023-4-10
	*	@author  ���ͿƼ�
   ************************************************************************************************
   *  @description
	*
	*	ʵ��ƽ̨������STM32H723ZGT6���İ� ���ͺţ�FK723M1-ZGT6��
	*	�Ա���ַ��https://shop212360197.taobao.com
	*	QQ����Ⱥ��536665479
	*
>>>>> ����˵����
	*
	*	1.����LED��ʹ��HAL���Դ�����ʱ����ʵ����˸
	*	2.ʹ�� OSPI1 ����W25Q64�����м򵥵Ķ�д����
	*	3.OSPI���ģʽ��ʹ�ÿ⺯��ֱ�Ӷ�д��
	*	4.Ĭ������QSPI����ʱ��Ϊ137.5M
	*
>>>>> ���ڴ�ӡ˵����
	*
	*	USART1ʹ�õ���PA9/PA10�����ڲ�����115200
	*	
	************************************************************************************************
***/

#include "main.h"
#include "led.h"
#include "usart.h"
#include "ospi_w25q64.h"

/********************************************** �������� *******************************************/

#define W25Qxx_NumByteToTest   	32*1024					// �������ݵĳ��ȣ�32K

int32_t OSPI_Status ; 		 //����־λ

uint8_t  W25Qxx_WriteBuffer[W25Qxx_NumByteToTest];		//	д��������
uint8_t  W25Qxx_ReadBuffer[W25Qxx_NumByteToTest];		//	����������




/***************************************************************************************************
*	�� �� ��: OSPI_W25Qxx_Test
*	��ڲ���: ��
*	�� �� ֵ: OSPI_W25Qxx_OK - ���Գɹ���ͨ��
*	��������: ���м򵥵Ķ�д���ԣ��������ٶ�
*	˵    ��: ��	
***************************************************************************************************/

int8_t OSPI_W25Qxx_Test(void)		//Flash��д����
{
	uint32_t i = 0X8000;	// ��������
	uint32_t W25Qxx_TestAddr  =	0	;							// ���Ե�ַ	
	uint32_t ExecutionTime_Begin;		// ��ʼʱ��
	uint32_t ExecutionTime_End;		// ����ʱ��
	uint32_t ExecutionTime;				// ִ��ʱ��	
	float    ExecutionSpeed;			// ִ���ٶ�

// ���� >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   
	
	
	ExecutionTime_Begin 	= HAL_GetTick();	// ��ȡ systick ��ǰʱ�䣬��λms
	OSPI_Status 			= OSPI_W25Qxx_BlockErase_32K(W25Qxx_TestAddr);	// ����32K�ֽ�
	ExecutionTime_End		= HAL_GetTick();	// ��ȡ systick ��ǰʱ�䣬��λms
	
	ExecutionTime = ExecutionTime_End - ExecutionTime_Begin; // �������ʱ�䣬��λms
	
	if( OSPI_Status == OSPI_W25Qxx_OK )
	{
		printf ("\r\nW25Q64 �����ɹ�, ����32K�ֽ�����ʱ��: %d ms\r\n",ExecutionTime);		
	}
	else
	{
		printf ("\r\n ����ʧ��!!!!!  �������:%d\r\n",OSPI_Status);
		while (1);
	}	
	
// д�� >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

	for(i=0;i<W25Qxx_NumByteToTest;i++)  //�Ƚ�����д������
	{
		W25Qxx_WriteBuffer[i] = i;
	}
	ExecutionTime_Begin 	= HAL_GetTick();	// ��ȡ systick ��ǰʱ�䣬��λms
	OSPI_Status				= OSPI_W25Qxx_WriteBuffer(W25Qxx_WriteBuffer,W25Qxx_TestAddr,W25Qxx_NumByteToTest); // д������
	ExecutionTime_End		= HAL_GetTick();	// ��ȡ systick ��ǰʱ�䣬��λms
	
	ExecutionTime  = ExecutionTime_End - ExecutionTime_Begin; 		// �������ʱ�䣬��λms
	ExecutionSpeed = (float)W25Qxx_NumByteToTest / ExecutionTime ; // ����д���ٶȣ���λ KB/S
	if( OSPI_Status == OSPI_W25Qxx_OK )
	{
		printf ("\r\nд��ɹ�,���ݴ�С��%d KB, ��ʱ: %d ms, д���ٶȣ�%.2f KB/s\r\n",W25Qxx_NumByteToTest/1024,ExecutionTime,ExecutionSpeed);		
	}
	else
	{
		printf ("\r\nд�����!!!!!  �������:%d\r\n",OSPI_Status);
		while (1);
	}	
	
// ��ȡ	>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> 

	
	ExecutionTime_Begin 	= HAL_GetTick();	// ��ȡ systick ��ǰʱ�䣬��λms	
	OSPI_Status				= OSPI_W25Qxx_ReadBuffer(W25Qxx_ReadBuffer,W25Qxx_TestAddr,W25Qxx_NumByteToTest);	// ��ȡ����
	ExecutionTime_End		= HAL_GetTick();	// ��ȡ systick ��ǰʱ�䣬��λms
	
	ExecutionTime  = ExecutionTime_End - ExecutionTime_Begin; 					// �������ʱ�䣬��λms
	ExecutionSpeed = (float)W25Qxx_NumByteToTest/1024/1024 / ExecutionTime*1000 ; 	// �����ȡ�ٶȣ���λ MB/S 
	
	if( OSPI_Status == OSPI_W25Qxx_OK )
	{
		printf ("\r\n��ȡ�ɹ�,���ݴ�С��%d KB, ��ʱ: %d ms, ��ȡ�ٶȣ�%.2f MB/s \r\n",W25Qxx_NumByteToTest/1024,ExecutionTime,ExecutionSpeed);		
	}
	else
	{
		printf ("\r\n��ȡ����!!!!!  �������:%d\r\n",OSPI_Status);
		while (1);
	}			
// ����У�� >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   
	
	for(i=0;i<W25Qxx_NumByteToTest;i++)	//��֤�����������Ƿ����д�������
	{
		if( W25Qxx_WriteBuffer[i] != W25Qxx_ReadBuffer[i] )	//������ݲ���ȣ��򷵻�0	
		{
			printf ("\r\n����У��ʧ��!!!!!����λ�ã�%d\r\n",i);	
			while(1);
		}
	}			
	printf ("\r\nУ��ͨ��!!!!! QSPI����W25Q64��������\r\n");		
	
// ��ȡ��ƬFlash�����ݣ����Բ����ٶ� >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
	
	printf ("\r\n*****************************************************************************************************\r\n");		
	printf ("\r\n����Ĳ����У���ȡ�����ݱȽ�С����ʱ�̣ܶ���֮��������С��λΪms��������Ķ�ȡ�ٶ����ϴ�\r\n");		
	printf ("\r\n��������ȡ��Ƭflash���������Բ����ٶȣ������ó����ٶ����Ƚ�С\r\n");		
	printf ("\r\n��ʼ��ȡ>>>>\r\n");		
	ExecutionTime_Begin 	= HAL_GetTick();	// ��ȡ systick ��ǰʱ�䣬��λms		
	
	for(i=0;i<W25Qxx_FlashSize/(W25Qxx_NumByteToTest);i++)	// ÿ�ζ�ȡ W25Qxx_NumByteToTest �ֽڵ�����
	{
		OSPI_Status				= OSPI_W25Qxx_ReadBuffer(W25Qxx_ReadBuffer,W25Qxx_TestAddr,W25Qxx_NumByteToTest);	// ��ȡ����
		W25Qxx_TestAddr = W25Qxx_TestAddr + W25Qxx_NumByteToTest;		
	}
	ExecutionTime_End		= HAL_GetTick();	// ��ȡ systick ��ǰʱ�䣬��λms
	
	ExecutionTime  = ExecutionTime_End - ExecutionTime_Begin; 								// �������ʱ�䣬��λms
	ExecutionSpeed = (float)W25Qxx_FlashSize/1024/1024 / ExecutionTime*1000  ; 	// �����ȡ�ٶȣ���λ MB/S 

	if( OSPI_Status == OSPI_W25Qxx_OK )
	{
		printf ("\r\n��ȡ�ɹ�,���ݴ�С��%d MB, ��ʱ: %d ms, ��ȡ�ٶȣ�%.2f MB/s \r\n",W25Qxx_FlashSize/1024/1024,ExecutionTime,ExecutionSpeed);		
	}
	else
	{
		printf ("\r\n��ȡ����!!!!!  �������:%d\r\n",OSPI_Status);
		while (1);
	}	
	
	return OSPI_W25Qxx_OK ;  // ����ͨ��				
	
}


/********************************************** �������� *******************************************/

void SystemClock_Config(void);		// ʱ�ӳ�ʼ��

/***************************************************************************************************
*	�� �� ��: main
*	��ڲ���: ��
*	�� �� ֵ: ��
*	��������: ����������
*	˵    ��: ��
****************************************************************************************************/

int main(void)
{
	SCB_EnableICache();		// ʹ��ICache
	SCB_EnableDCache();		// ʹ��DCache
	HAL_Init();					// ��ʼ��HAL��
	SystemClock_Config();	// ����ϵͳʱ�ӣ���Ƶ550MHz
	LED_Init();					// ��ʼ��LED����
	USART1_Init();				// USART1��ʼ��	

	OSPI_W25Qxx_Init();		// ��ʼ��OSPI��W25Q64
	
	OSPI_W25Qxx_Test();		// Flash��д����
	
	while (1)
	{
		LED1_Toggle;
		HAL_Delay(100);
	}
}


/****************************************************************************************************/
/**
  *   ϵͳʱ�����ã�
  *
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 550000000 (CPU ��Ƶ 550MHz)
  *            HCLK(Hz)                       = 275000000 (AXI and AHBs Clock)
  *            AHB Prescaler                  = 1 (AHB  Clock  275 MHz)
  *            D1 APB3 Prescaler              = 2 (APB3 Clock  137.5MHz)
  *            D2 APB1 Prescaler              = 2 (APB1 Clock  137.5MHz)
  *            D2 APB2 Prescaler              = 2 (APB2 Clock  137.5MHz)
  *            D3 APB4 Prescaler              = 2 (APB4 Clock  137.5MHz)
  *            HSE Frequency(Hz)              = 25000000  (�ⲿ����Ƶ��)
  *            PLL_M                          = 10
  *            PLL_N                          = 220
  *            PLL_P                          = 1
  *
  *				CPU��Ƶ = HSE Frequency / PLL_M * PLL_N / PLL_P = 25M /10*220/1 = 550M
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
    
	/* Ϊ�����÷��㣬�� ����ʱ��HCLK3 ��ΪOSPI���ں�ʱ�ӣ��ٶ�Ϊ275M���پ���2��Ƶ�õ� 137.5M ������ʱ��  */  
	/* ��ȻW25Q64JV ����������ʱ��Ϊ 133M */
	/* ��ʵ�ʲ����У�������ʱ����������200M��w25q64���ǿ���������д������ֻ����4.5M��Ƶ�ʲ��õ����ȶ���*/
   /* ���û�ʹ�ó����Ͽ����������������ں�ʱ�� */
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

