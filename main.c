/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : main.c
* Author             : MCD Application Team
* Version            : V2.0.3
* Date               : 09/22/2008
* Description        : Main program body
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_lib.h"
#include "stm32f10x_dbgmcu.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define maxBufLen 10

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
ErrorStatus HSEStartUpStatus;
u8 rxFrameOK;
u8 delay100ms;
u8 RXBUF[maxBufLen];
u8 TXBUF[maxBufLen];
u8 TXBUF_test[2]={0x03,0x04};

/* Private function prototypes -----------------------------------------------*/
void RCC_Configuration(void);
void GPIO_Configuration(void);
void ADC_Configuration(void);
void USART_Configuration(void);
void TIM_Configuration(void);
void NVIC_Configuration(void);
u32 ADC_Check(void);
void Init_Iwdg(void);
void UART_Send(u8 *str,u8 len3);//串口1

/* Private functions ---------------------------------------------------------*/

/*******************************************************************************
* Function Name  : main
* Description    : Main program
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
int main(void)
{
#ifdef DEBUG
  debug();
#endif
  
  RCC_Configuration();
  GPIO_Configuration();
  ADC_Configuration();
  USART_Configuration();
  TIM_Configuration();
  NVIC_Configuration();
  Init_Iwdg();
  // SysTick end of count event each 1ms with input clock equal to 9MHz (HCLK/8, default)
  SysTick_SetReload(9000);
  // Enable SysTick interrupt
  SysTick_ITConfig(ENABLE);
  // Enable the SysTick Counter
  SysTick_CounterCmd(SysTick_Counter_Enable);
   
	while(1)
	{
		// 处理数据
		if(rxFrameOK)
		{
			if((RXBUF[1]=='z')||(RXBUF[1]=='Z'))
				if((RXBUF[2]=='m')||(RXBUF[2]=='M'))
					if((RXBUF[3]=='k')||(RXBUF[3]=='K'))
						if((RXBUF[4]=='m')||(RXBUF[4]=='M'))
						{
							GPIO_SetBits((GPIO_TypeDef *)GPIOB_BASE, GPIO_Pin_7);
							delay100ms = 0;
							while(delay100ms<=2)IWDG_ReloadCounter();
							GPIO_ResetBits((GPIO_TypeDef *)GPIOB_BASE, GPIO_Pin_7);
							delay100ms = 0;
							while(delay100ms<=5)IWDG_ReloadCounter();
							
							TXBUF[0] = 2;
							TXBUF[1] = 0x01;//'k';
							TXBUF[2] = 0x02;//'o';
//                                                        GPIO_ResetBits((GPIO_TypeDef *)GPIOA_BASE, GPIO_Pin_12);
//                                                        UART_Send(TXBUF_test,2);//串口1
							USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
						}
			RXBUF[0] = 0;
			rxFrameOK = 0;
                      //  GPIO_SetBits((GPIO_TypeDef *)GPIOA_BASE, GPIO_Pin_12);
		}
		
		// 喂狗
		IWDG_ReloadCounter();
	}
}





/*串口1发送*/
void UART_Send(u8 *str,u8 len3)
{
	 u8 i;
	 while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);	
         for(i=0;i<len3;i++)
	{
            USART_SendData(USART1, *(str+i));
	   while(USART_GetFlagStatus(USART1,USART_FLAG_TC)==RESET);  //while?a??ê±??μèμ?oü??êy?Y?1??·￠3?è￥  ò??-ê???ò????D??á?
	}
}


/*u32 ADC_Check(void)
{
	u32 ResultVolt = 0;
	u8 i;
	for(i=0; i<8; i++)
	{
		ADC_SoftwareStratConvCmd(ADC1, ENABLE);
		while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC));
		ResultVolt += (u32)ADC_GetConversionValue(ADC1);
	}
	ResultVolt = ResultVolt>>3;
	ResultVolt = (ResultVolt*3300)>>12;
	reture ResultVolt;
}*/

/*******************************************************************************
* Function Name  : RCC_Configuration
* Description    : Configures the different system clocks.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void RCC_Configuration(void)
{  
  /* RCC system reset(for debug purpose) */
  RCC_DeInit();

  /* Enable HSE */
  RCC_HSEConfig(RCC_HSE_ON);

  /* Wait till HSE is ready */
  HSEStartUpStatus = RCC_WaitForHSEStartUp();

  if(HSEStartUpStatus == SUCCESS)
  {
    /* Enable Prefetch Buffer */
    FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);

    /* Flash 2 wait state */
    FLASH_SetLatency(FLASH_Latency_2);
 
    /* HCLK = SYSCLK */
    RCC_HCLKConfig(RCC_SYSCLK_Div1); 
  
    /* PCLK2 = HCLK */
    RCC_PCLK2Config(RCC_HCLK_Div1); 

    /* PCLK1 = HCLK/2 */
    RCC_PCLK1Config(RCC_HCLK_Div2);

    /* PLLCLK = 8MHz * 9 = 72 MHz */
    RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);

    /* Enable PLL */ 
    RCC_PLLCmd(ENABLE);

    /* Wait till PLL is ready */
    while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
    {
    }

    /* Select PLL as system clock source */
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

    /* Wait till PLL is used as system clock source */
    while(RCC_GetSYSCLKSource() != 0x08)
    {
    }
  } 
  
  /* Enable USART1, GPIOA and GPIOB clocks */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA 
                         | RCC_APB2Periph_GPIOB| RCC_APB2Periph_AFIO
						 | RCC_APB2Periph_ADC1, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3, ENABLE);
}

/*******************************************************************************
* Function Name  : GPIO_Configuration
* Description    : Configures the different GPIO ports.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void GPIO_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
      
  /* USART1 TX (PA9) */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
      
  /* USART1 RX (PA10) */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
 
  /* PA3锁状态输入 */
//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
//  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  /* PB7锁开关输出 */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  
  /* PA5电池电量检测ADC12_IN5 */  
//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
// GPIO_Init(GPIOA, &GPIO_InitStructure);  
}

void ADC_Configuration(void)
{
  ADC_InitTypeDef ADC_InitStructure;
  
  /* ADC1 Configuration ------------------------------------------------------*/
  ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;					//独立工作模式
  ADC_InitStructure.ADC_ScanConvMode = DISABLE;							//单通道模式
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;					//单次转化模式
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;	//软件触发
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;				//右对齐
  ADC_InitStructure.ADC_NbrOfChannel = 1;								//1个通道
  ADC_Init(ADC1, &ADC_InitStructure);

  /* ADC1 regular channel5 configuration */ 
  ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 1, ADC_SampleTime_55Cycles5);

  /* Enable ADC1 */
  ADC_Cmd(ADC1, ENABLE);
  
  /* Enable ADC1 reset calibaration register */   
  ADC_ResetCalibration(ADC1);
  
  /* Check the end of ADC1 reset calibration register */
  while(ADC_GetResetCalibrationStatus(ADC1));
  
  /* Start ADC1 calibaration */
  ADC_StartCalibration(ADC1);
  
  /* Check the end of ADC1 calibration */
  while(ADC_GetCalibrationStatus(ADC1));
}

/*******************************************************************************
* Function Name  : USART_Configuration
* Description    : Configures the USARTS.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void USART_Configuration(void)
{
  USART_InitTypeDef USART_InitStructure;
  /* USART configured as follow:
        - BaudRate = 9600 baud  
        - Word Length = 8 Bits
        - One Stop Bit
        - No parity
        - Hardware flow control disabled (RTS and CTS signals)
        - Receive and transmit enabled
  */
  USART_InitStructure.USART_BaudRate = 9600;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  
  /* Configure USART1 */
  USART_Init(USART1, &USART_InitStructure);  
  /* Enable USART1 Receive interrupts */
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
  /* Enable the USART1 */
  USART_Cmd(USART1, ENABLE);
}

void TIM_Configuration(void)
{
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  
   //ARR的值
  TIM_TimeBaseStructure.TIM_Period=200-1;//100ms
  //分频
  TIM_TimeBaseStructure.TIM_Prescaler=36000-1;
  //采样分频
  TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;
  //向上计数模式
  TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;
  
  //初始化定时器
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
  //打开中断 溢出中断
  TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
  //打开定时器
  TIM_Cmd(TIM3, ENABLE);
  
  TIM_TimeBaseStructure.TIM_Period=200-1;//100ms
  //初始化定时器
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
  //打开中断 溢出中断
  TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
  //打开定时器
  TIM_Cmd(TIM2, ENABLE);
}

/*******************************************************************************
* Function Name  : NVIC_Configuration
* Description    : Configures the nested vectored interrupt controller.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;

#ifdef  VECT_TAB_RAM  
  /* Set the Vector Table base location at 0x20000000 */ 
  NVIC_SetVectorTable(NVIC_VectTab_RAM, 0x0); 
#else  /* VECT_TAB_FLASH  */
  /* Set the Vector Table base location at 0x08000000 */ 
  NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);   
#endif 

  /* Configure the NVIC Preemption Priority Bits */  
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
  
  /* Enable the USART1 Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQChannel;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
   
  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQChannel;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_Init(&NVIC_InitStructure);
  
  NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQChannel;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_Init(&NVIC_InitStructure);
}

void Init_Iwdg(void)
{
	/* IWDG timeout equal to 200 ms (the timeout may varies due to LSI frequency dispersion) */

	/* Enable write access to IWDG_PR and IWDG_RLR registers */
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);

	/* IWDG counter clock: 40KHz(LSI) / 32 = 1.25 KHz */
	IWDG_SetPrescaler(IWDG_Prescaler_32);

	/* Set counter reload value to 1249: (12499+1)/1.25=1000ms */
	IWDG_SetReload(12499);

	/* Reload IWDG counter */
	IWDG_ReloadCounter();

	/* Enable IWDG (the LSI oscillator will be enabled by hardware) */
	IWDG_Enable();

	/* Configures the IWDG clock mode when the MCU under Debug mode */
	DBGMCU_Config(DBGMCU_IWDG_STOP, ENABLE);
}
/*******************************************************************************
* Interrupt Service Routines
*******************************************************************************/
void USART1_InterruptHandler(void)
{    
  //接收中断
  if(USART_GetITStatus(USART1, USART_IT_RXNE) == SET)
  {
    if(RXBUF[0]>maxBufLen-2)RXBUF[0] = 0;
    if(rxFrameOK)
      USART_ReceiveData(USART1);
    else
      RXBUF[++RXBUF[0]] = USART_ReceiveData(USART1);
    TIM_SetCounter(TIM3, 0);
  }
  
  //发送中断
  if(USART_GetITStatus(USART1, USART_IT_TXE) != RESET)
  {
    if(TXBUF[0]>0)
      USART_SendData(USART1, TXBUF[TXBUF[0]--]);
    else
      USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
  }
}

void TIM2_InterruptHandler(void)
{
  if(TIM_GetITStatus(TIM2,TIM_IT_Update)!=RESET)
  {
    //清中断标志
    TIM_ClearITPendingBit(TIM2,TIM_IT_Update);
    delay100ms++;
  }
}

void TIM3_InterruptHandler(void)
{
  if(TIM_GetITStatus(TIM3,TIM_IT_Update)!=RESET)
  {
    //清中断标志
    TIM_ClearITPendingBit(TIM3,TIM_IT_Update);
    if(RXBUF[0]>0)rxFrameOK = 1;
  }
}

#ifdef  DEBUG
/*******************************************************************************
* Function Name  : assert_failed
* Description    : Reports the name of the source file and the source line number
*                  where the assert_param error has occurred.
* Input          : - file: pointer to the source file name
*                  - line: assert_param error line source number
* Output         : None
* Return         : None
*******************************************************************************/
void assert_failed(u8* file, u32 line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/
