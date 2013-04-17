/**
  ******************************************************************************
  * @file    main.c
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    07/16/2010 
  * @brief   Main program body
  ******************************************************************************
  * @copy
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2010 STMicroelectronics</center></h2>
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32_eth.h"
#include "ethernetif.h"
#include "netconf.h"
#include "main.h"
#include "httpserver.h" 
#include "tftpserver.h"
#include "flash_if.h"
#include "helloworld.h"

#include "stm32f10x_bkp.h"

#include "stdio.h"

/* Private typedef -----------------------------------------------------------*/
typedef  void (*pFunction)(void);

/* Private define ------------------------------------------------------------*/
#define SYSTEMTICK_PERIOD_MS  10

#ifdef __GNUC__
  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
__IO uint32_t LocalTime = 0; /* this variable is used to create a time reference incremented by 10ms */
uint32_t timingdelay;
pFunction Jump_To_Application;
uint32_t JumpAddress;

volatile uint16_t flag = 0;

static uint32_t mark = 0;
#define  mark_addr    (*((volatile uint8_t  *)0x0807FFF0uL))

/* Private function prototypes -----------------------------------------------*/
void System_Periodic_Handle(void);
void USART2_init(void);
void GPIO_config(void);
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int main(void)
{
  //DI0配置
  GPIO_config();
  
  //test
  uint32_t user_flash = USER_FLASH_FIRST_PAGE_ADDRESS;

  //DI0断开则读为1，跳转程序  
  if(GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_9) != 0x00) 
  {
    /* Check if valid stack address (RAM address) then jump to user application */
    if (((*(__IO uint32_t*)USER_FLASH_FIRST_PAGE_ADDRESS) & 0x2FFE0000 ) == 0x20000000)
    {
      /* Jump to user application */
      JumpAddress = *(__IO uint32_t*) (USER_FLASH_FIRST_PAGE_ADDRESS + 4);
      Jump_To_Application = (pFunction) JumpAddress;
      /* Initialize user application's Stack Pointer */
      __set_MSP(*(__IO uint32_t*) USER_FLASH_FIRST_PAGE_ADDRESS);
      Jump_To_Application();
    }
    else
    {
      /* do nothing */
      while(1);
    }
    while(1)
    {}
  }
  //DI0接通则读为0，升级程序
  else 
  {
    /* Setup STM32 system (clocks, Ethernet, GPIO, NVIC) and STM3210C-EVAL resources */
    System_Setup();
  
    /* Initilaize the LwIP stack */
    LwIP_Init();
    
#ifdef USE_IAP_HTTP
    /* Initilaize the webserver module */
    IAP_httpd_init();
#endif
  
#ifdef USE_IAP_TFTP    
    /* Initialize the TFTP server */
    IAP_tftpd_init();
#endif    
    
    /* Infinite loop */
    while (1)
    {
      /* check if any packet received */
      if (ETH_GetRxPktSize()!=0)
      { 
        /* process received eth packet */
        LwIP_Pkt_Handle();
      }
      /* Periodic tasks */
      System_Periodic_Handle();
    }
  }
  return 0;
}

void GPIO_config(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);
  
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
  GPIO_Init(GPIOE, &GPIO_InitStructure);
  
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
  
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
}

void USART2_init(void)
{
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD | RCC_APB2Periph_AFIO,ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
  
  /*config GPIO of USART2*/
   GPIO_InitTypeDef GPIO_InitStructure;
   
   GPIO_PinRemapConfig(GPIO_Remap_USART2,ENABLE);
   
   GPIO_InitStructure.GPIO_Pin=GPIO_Pin_5;          /*PD5 for 232 Tx*/
   GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
   GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP;
   GPIO_Init(GPIOD,&GPIO_InitStructure);
   
   GPIO_InitStructure.GPIO_Pin=GPIO_Pin_6;          /*PD6 for 232 Rx*/
   GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
   GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN_FLOATING;
   GPIO_Init(GPIOD,&GPIO_InitStructure);
   
   /*Initiate USART2*/
   USART_InitTypeDef USART_InitStructure;
   
   USART_InitStructure.USART_BaudRate=115200;
   USART_InitStructure.USART_WordLength=USART_WordLength_8b;
   USART_InitStructure.USART_StopBits=USART_StopBits_1;
   USART_InitStructure.USART_Parity=USART_Parity_No;
   USART_InitStructure.USART_HardwareFlowControl=USART_HardwareFlowControl_None;
   USART_InitStructure.USART_Mode=USART_Mode_Rx | USART_Mode_Tx;
   
   USART_Init(USART2,&USART_InitStructure);
   
   //USART_ITConfig(USART2,USART_IT_RXNE,ENABLE);
   
   USART_Cmd(USART2,ENABLE);
}

PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART */
  USART_SendData(USART2, (uint8_t) ch);

  /* Loop until the end of transmission */
  while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET)
  {}

  return ch;
}

/**
  * @brief  Inserts a delay time.
  * @param  nCount: number of 10ms periods to wait for.
  * @retval None
  */
void Delay(uint32_t nCount)
{
  /* Capture the current local time */
  timingdelay = LocalTime + nCount;  

  /* wait until the desired delay finish */  
  while(timingdelay > LocalTime)
  {     
  }
}

/**
  * @brief  Updates the system local time
  * @param  None
  * @retval None
  */
void Time_Update(void)
{
  LocalTime += SYSTEMTICK_PERIOD_MS;
}

/**
  * @brief  Handles the periodic tasks of the system
  * @param  None
  * @retval None
  */
void System_Periodic_Handle(void)
{
#ifdef USE_LCD 
  
  /* Update the LCD display and the LEDs status */
  /* Manage the IP address setting */
  Display_Periodic_Handle(LocalTime);
  
#endif
  
  /* LwIP periodic services are done here */
  LwIP_Periodic_Handle(LocalTime);
}


#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
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
  {}
}
#endif


/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/
