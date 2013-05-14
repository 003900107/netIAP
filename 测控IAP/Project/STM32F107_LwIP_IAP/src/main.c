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

/* Private typedef -----------------------------------------------------------*/
typedef  void (*pFunction)(void);

/* Private define ------------------------------------------------------------*/
#define SYSTEMTICK_PERIOD_MS  10
#define ETH_RESET    GPIOC, GPIO_Pin_10
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
__IO uint32_t LocalTime = 0; /* this variable is used to create a time reference incremented by 10ms */
uint32_t timingdelay;
pFunction Jump_To_Application;
uint32_t JumpAddress;

/* Private function prototypes -----------------------------------------------*/
void System_Periodic_Handle(void);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int main(void)
{
  FlagStatus status;
  uint16_t bak_dr10;
  //uint16_t bak_dr9;
  
  //RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);
  //PWR_BackupAccessCmd(ENABLE);  
  
  //bak_dr10 = BKP_ReadBackupRegister(BKP_DR10);
  //bak_dr9 = BKP_ReadBackupRegister(BKP_DR9);
  
  //BKP_WriteBackupRegister(BKP_DR10, 0x0707);
  //BKP_WriteBackupRegister(BKP_DR9, 0x0001);
  
  
  /* Test if Key push-button on STM3210C-EVAL Board is not pressed */
  //if((bak_dr10 == 0x0707) /*&& (bak_dr9 != 0x0000)*/)
  if(1)
  {
    /* Setup STM32 system (clocks, Ethernet, GPIO, NVIC) and STM3210C-EVAL resources */
    System_Setup();
    
    Delay(1000);
    //可添加延时, 用以确保DM9000启动时电压满足芯片要求
    //"nRST must not go high until after the VDDIO and VDD_CORE supplies are stable"  手册P51
    GPIO_WriteBit(ETH_RESET,  Bit_SET);   //拉高DM9000 nRST, 复位启动    
    /* Configure the Ethernet peripheral */
    Ethernet_Configuration();
    
    /*
    //判断设备是否是初次上电, 如果'是', 重启一次, 确保以太网初始化正常
    status = RCC_GetFlagStatus(RCC_FLAG_SFTRST);
    RCC_ClearFlag(); 
    if(!status)
    {   
      Delay(10);
      
      NVIC_SystemReset();
    }
    */
 
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
  /* enter in IAP mode */
  else
  {
    /* Key push-button not pressed: jump to user application */
    
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
    {/* Otherwise, do nothing */
      /* LED3 (RED) ON to indicate bad software (when not valid stack address) */
      //STM_EVAL_LEDInit(LED3);
      //STM_EVAL_LEDOn(LED3);
      /* do nothing */
      while(1);
    }    
  }
  
  return 0;
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
