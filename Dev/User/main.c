/**
  ******************************************************************************
  * @file    main.c 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    18-April-2011
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************  
  */ 

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "config.h"
#include <string.h>
#include <stdio.h>
# include <stdlib.h>

#define OUTSIDE 1
#define INSIDE 2
#define NOT_FULL 0
#define FULL 1

int record1[4];
int record2[4];
int outside_state=0;
int inside_state=0;
/* Global variables ----------------------------------------------------------*/
volatile uint8_t status_10ms = 0;
volatile uint8_t status_50ms = 0;

volatile uint8_t usart_buffer_pointer = 0;
volatile uint8_t usart_receive_buffer[256];
volatile uint8_t usart_command_status = 0;

uint32_t global_supply_counter = 0;

uint32_t Encoder = 0;
int Moto1 = 0;
int Target_position = COUNTER_RESET;
const int container_1 = COUNTER_RESET;
const int container_2 = COUNTER_RESET - 200;

uint8_t ir_pd_1_flag[4] = {0};
uint8_t ir_pd_2_flag[4] = {0};
uint8_t ir_pd_1_counter[4] = {0};
uint8_t ir_pd_2_counter[4] = {0};

uint32_t container_counter[2] = {0};

/*Servo output PWM should between 200 to 1000 */
const uint16_t CCR1_Open = 210;
const uint16_t CCR1_Close = 400;
uint16_t CCR1_Val = CCR1_Close;

const uint16_t CCR2_Open = 350;
const uint16_t CCR2_Close = 650;
uint16_t CCR2_Val = CCR2_Close;

const uint16_t CCR3_Open = 250;
const uint16_t CCR3_Close = 600;
uint16_t CCR3_Val = CCR3_Open;

const uint16_t CCR4_1 = 300;
const uint16_t CCR4_2 = 1000;
uint16_t CCR4_Val = CCR4_1;
uint8_t CCR4_Count_up_flag = 1;

/* FSM state counter 
 * 0 - first 200 bullets, supply container 1
 * 1 - first 200 bullets, supply container 2
 * 2 - first 200 bullets, supply none
 * 3 - bullets after first time, supply container 1
 * 4 - bullets after first time, supply container 2
 * 5 - bullets after first time, supply none
*/
uint8_t FSM_state = 0;
uint8_t FSM_last_state = 0;

/* Private function prototypes -----------------------------------------------*/
void  Delay(uint32_t nCount);
void	PWM_TIM3_Config(void);
void PWM_TIM10_Config(void);
void	GPIO_Config(void);
void update_IR_state(void);
void update_SW_state(void);
void set_servo_state(void);
int get_bullet_quantity(int);
void check_supply_state(int);
void revolve_servo(void);

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
	RCC_ClocksTypeDef RCC_Clocks;
//	uint32_t counter_200ms = 0;
//	uint32_t vibration_counter = 0;
	//If the container is not full after some time, the motor will perform anti-piping method.
//	uint32_t anti_piping_counter = 0;

	/* Systick Timer Config */
	RCC_GetClocksFreq(&RCC_Clocks);
	//Config Systick at period of 10ms
	SysTick_Config(RCC_Clocks.HCLK_Frequency / 100);
	
	GPIO_Config();
	PWM_TIM3_Config();
	PWM_TIM10_Config();
	//TIM_SetCompare1(TIM10, 5);
	
	USART_Configuration();
	USART_ITConfig(Open_USARTx, USART_IT_RXNE, ENABLE); 
	USART_NVIC_Config();
	
	/* Motor Encoder Config */
	MiniBalance_PWM_Init();
	Encoder_Init_TIM2();            //初始化编码器
	
	/* Infinite loop */
	while (1)
	{
		if (status_10ms)
		{	
			status_10ms = 0;
			/* Read Encoder and Control PID */
			Encoder=Read_Encoder(2);                                  //===读取编码器的位置数据 初始值是10000，详见encoder.c 和encoder.h
			Moto1=Position_PID(Encoder,Target_position);              //===位置PID控制器
			Xianfu_Pwm();                                             //===PWM限幅
			Set_Pwm(Moto1);  
		}
		if(status_50ms)
		{
			status_50ms = 0;
			set_servo_state();
			update_IR_state();//更新两侧红外检测状态
			update_SW_state();//更新开关按下状态，并控制改变相应舵机的开和关
			check_supply_state(INSIDE);
			check_supply_state(OUTSIDE);
			
			if(outside_state==FULL&&inside_state==FULL)CCR3_Val = CCR3_Close;
			else{
				revolve_servo();//转动搅弹电机
				CCR3_Val = CCR3_Open;
			}
			
			switch(inside_state){
				case FULL:Target_position = container_1;break;
				case NOT_FULL:Target_position = container_2;break;
			}
			switch(outside_state){
				case FULL:Target_position = container_2;break;
				case NOT_FULL:Target_position = container_1;break;
			}
			//根据两边是否满弹控制拨弹电机，外侧优先
		}
	}
}

/**
  * @brief  Configure PA2-PA5, PB10, PB11, PC0-PC5 as digital I/O pins.
  * @param  None
  * @retval None
  */
void GPIO_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
	
	/* GPIOA, GPIOB, GPIOC Periph clock enable */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	
	//////////////////////////////////////////////////////////////
	//                                                          //
	//           IR_PD_1 Config, GPIO PA2-PA5 INPUT             //
	//                                                          //
	//////////////////////////////////////////////////////////////

  /* Configure PA2-PA5 in input mode */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;	
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	//////////////////////////////////////////////////////////////
	//                                                          //
	//           IR_PD_2 Config, GPIO PC0-PC3 INPUT             //
	//                                                          //
	//////////////////////////////////////////////////////////////
	
  /* Configure PC0-PC3 in input mode */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;	
	GPIO_Init(GPIOC, &GPIO_InitStructure);	
	
	//////////////////////////////////////////////////////////////
	//                                                          //
	//   Switch Config, GPIO PC4(CH1 front end side),						//
	//    PC5(CH2 front), PB10(CH1 back), PB11(CH2 back) INPUT; //
	//                                                          //
	//////////////////////////////////////////////////////////////	
	
	/* Configure PC4,PC5 in input mode */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;	
	GPIO_Init(GPIOC, &GPIO_InitStructure);	
	
	/* Configure PB10,PB11 in input mode */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	//////////////////////////////////////////////////////////////
	//                                                          //
	//   LED Config, GPIO PB0-PB3,GPIO PB0-PB3,PA8-PA11,        //
	//                                                          //
	//                                                          //
	//////////////////////////////////////////////////////////////	
	
	/* Configure PB0-PB3 in input mode */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;	
	GPIO_Init(GPIOB, &GPIO_InitStructure);	
	
	/* Configure PA8-PA11 in input mode */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}

/**
  * @brief  Configure the TIM3 and PC6-PC9 as PWM output pins.
  * @param  None
  * @retval None
  */
void PWM_TIM3_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;	
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	uint16_t PrescalerValue = 0;

  /* TIM3 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

  /* GPIOC clock enable */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
  
  /* GPIOC Configuration: TIM3 CH1 (PC6), TIM3 CH2 (PC7), TIM3 CH3 (PC8) and TIM3 CH4 (PC9) */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOC, &GPIO_InitStructure); 

  /* Connect TIM3 pins to AF2 */  
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_TIM3);
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_TIM3); 
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource8, GPIO_AF_TIM3);
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource9, GPIO_AF_TIM3); 
	
	/* Compute the prescaler value, config TIM3 counter clock at 420kHz */
  PrescalerValue = (uint16_t) ((SystemCoreClock /2) / 420000) - 1;

  /* Time base configuration, config PWM output cycle at 50Hz */
  TIM_TimeBaseStructure.TIM_Period = 8399;
  TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

  /* PWM1 Mode configuration: Channel1 */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = CCR1_Val;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

  TIM_OC1Init(TIM3, &TIM_OCInitStructure);

  TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);

  /* PWM1 Mode configuration: Channel2 */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = CCR2_Val;

  TIM_OC2Init(TIM3, &TIM_OCInitStructure);

  TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);

  /* PWM1 Mode configuration: Channel3 */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = CCR3_Val;

  TIM_OC3Init(TIM3, &TIM_OCInitStructure);

  TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);

  /* PWM1 Mode configuration: Channel4 */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = CCR4_Val;

  TIM_OC4Init(TIM3, &TIM_OCInitStructure);

  TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);

  TIM_ARRPreloadConfig(TIM3, ENABLE);

  /* TIM3 enable counter */
  TIM_Cmd(TIM3, ENABLE);
}

/**
  * @brief  Configure the TIM10 and PC6-PC9 as PWM output pins.
  * @param  None
  * @retval None
  */
void PWM_TIM10_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;	
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	uint16_t PrescalerValue = 0;

  /* TIM10 clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM10, ENABLE);

  /* GPIOB clock enable */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
  
  /* GPIOB Configuration: TIM10 CH1 (PB8) */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOB, &GPIO_InitStructure); 

  /* Connect TIM10 pins to AF2 */  
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_TIM10);
	
	/* Compute the prescaler value, config TIM10 counter clock at 84 MHz */
  PrescalerValue = (uint16_t) ((SystemCoreClock ) / 84000000) - 1;

  /* Time base configuration, config PWM output cycle at 38 kHz */
  TIM_TimeBaseStructure.TIM_Period = 2210;
  TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

  TIM_TimeBaseInit(TIM10, &TIM_TimeBaseStructure);

  /* PWM1 Mode configuration: Channel1 */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 1105;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

  TIM_OC1Init(TIM10, &TIM_OCInitStructure);

  TIM_OC1PreloadConfig(TIM10, TIM_OCPreload_Enable);

  

  TIM_ARRPreloadConfig(TIM10, ENABLE);

  /* TIM10 enable counter */
  TIM_Cmd(TIM10, ENABLE);
}
/**
  * @brief  get bullet quantity
  * @param  the site of IR
  * @retval lights counts
  */
int get_bullet_quantity(int site){
	int lights=0;
	switch(site){
		case OUTSIDE:for(int i=0;i<4;i++)lights+=ir_pd_1_flag[i];break;
		case INSIDE:for(int i=0;i<4;i++)lights+=ir_pd_2_flag[i];break;
	}
	return lights;
}
/**
  * @brief  revolve the servo
  * @param  none
  * @retval none
  */
void revolve_servo(){
	static uint32_t counter_2s = 0;
	if (++counter_2s >= 40)
	{
		counter_2s = 0;
		if (CCR4_Val == CCR4_1)
			CCR4_Val = CCR4_2;
		else
			CCR4_Val = CCR4_1;
	}
}
/**
  * @brief  check supply state
  * @param  the site of supply pipe
  * @retval none
  */
void check_supply_state(int site){
	switch(site){
		case OUTSIDE:if(get_bullet_quantity(OUTSIDE)>=FULL)outside_state=FULL;else outside_state=NOT_FULL;break;
		case INSIDE:if(get_bullet_quantity(INSIDE)>=FULL)inside_state=FULL;else inside_state=NOT_FULL;break;
	}
}
/**
  * @brief  set sercvo state.
  * @param  None
  * @retval None
  * @tip	it will be executed every 50ms.
  */
void set_servo_state(){
	/* Update Servo PWM Outputs */
	TIM_SetCompare1(TIM3, CCR1_Val);
	TIM_SetCompare2(TIM3, CCR2_Val);
	TIM_SetCompare3(TIM3, CCR3_Val);
	TIM_SetCompare4(TIM3, CCR4_Val);
}
/**
  * @brief  update IR state.
  * @param  None
  * @retval None
  * @tip	it will be executed every 50ms.
  */
void update_IR_state(){
	/* Read IR_PD_1 Pins */

	for(int i = 0; i < 4; i++)
	{
		record1[i]=GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_2 << i);
		if (ir_pd_1_counter[i] == 0)
		{
			if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_2 << i))
				ir_pd_1_counter[i]++;
		}
		// Delay 100ms
		else if (++ir_pd_1_counter[i] >= 3)
		{
			if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_2 << i))
				ir_pd_1_flag[i] = 1;
			else
			{
				ir_pd_1_flag[i] = 0;
				ir_pd_1_counter[i] = 0;
			}
		}
	}

	/* Read IR_PD_2 Pins */

	for(int i = 0; i < 4; i++)
	{
		record2[i]=GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_0 << i);
		if (ir_pd_2_counter[i] == 0)
		{
			if (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_0 << i))
				ir_pd_2_counter[i]++;
		}
		// Delay 100ms
		else if (++ir_pd_2_counter[i] >= 3)
		{
			if (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_0 << i))
				ir_pd_2_flag[i] = 1;
			else
			{
				ir_pd_2_flag[i] = 0;
				ir_pd_2_counter[i] = 0;
			}
		}
	}
}
/**
  * @brief  update SW state and control the related servos.
  * @param  None
  * @retval None
  * @tip	it will be executed every 50ms.
  */
void update_SW_state(){
	/* Read SW pins and Write SW_LED output */
	for(int i = 0; i < 2; i++)
	{
		if (container_counter[i] == 0)
		{
			if (!GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_4 << i))// HIGH valid here
			{
				container_counter[i]++;//container_counter[i]=4;
			}
		}
		//Check the SW pin status after 200ms
		//这里存在一个可能的BUG，比如按动任意一个开关，200ms内再按动另一个开关，那么就会直接打开，而无视了延时
		//虽然没啥用
		else if (container_counter[i]++ == 4)
		{
			if (!GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_4 << i))			
			{
				//whenever the supply is taken once, the counter plus 1
				global_supply_counter++;
				//open container here
				if (i == 0)
				{	
					CCR1_Val = CCR1_Open;
					GPIO_WriteBit(GPIOB,GPIO_Pin_10,Bit_SET);//打开补弹指示灯
				}
				else
				{
					CCR2_Val = CCR2_Open;
					GPIO_WriteBit(GPIOB,GPIO_Pin_11,Bit_SET);//打开补弹指示灯
				}
			}
			else
				container_counter[i] = 0;
		}
		//Open container for 1.5s
		else if(container_counter[i] == 30 + 4)
		{
			//close container here
			if (i == 0)
			{
				CCR1_Val = CCR1_Close;
				GPIO_WriteBit(GPIOB,GPIO_Pin_10,Bit_RESET);
			}
			else
			{
				CCR2_Val = CCR2_Close;
				GPIO_WriteBit(GPIOB,GPIO_Pin_11,Bit_RESET);
			}
		}
		else if(GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_4 << i) && container_counter[i] > 30 + 4)
		{
			container_counter[i] = 0;
		}
	}
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {

  }
}
#endif

/**
  * @}
  */ 

/**
  * @}
  */ 

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
