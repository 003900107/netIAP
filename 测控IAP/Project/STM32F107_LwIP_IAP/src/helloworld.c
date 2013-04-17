/**
  ******************************************************************************
  * @file    helloworld.c 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    11/20/2009
  * @brief   A hello world example based on a Telnet connection
  *          The application works as a server which wait for the client request
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
  * <h2><center>&copy; COPYRIGHT 2009 STMicroelectronics</center></h2>
  */ 

/* Includes ------------------------------------------------------------------*/
#include "helloworld.h"
#include "stm3210c_eval_lcd.h"  // w w w . a r m j i s h u . c o m
#include "lwip/tcp.h"
#include <string.h>
#include <stdio.h>

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/


#define MAX_NAME_SIZE 50

struct tcp_pcb *pcb_command;

extern volatile uint16_t flag;

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
struct RxBuffer 
{
  int length;
  char bytes[MAX_NAME_SIZE];
};

/* Private function prototypes -----------------------------------------------*/
static err_t HelloWorld_recv(void *arg, struct tcp_pcb *pcb, struct pbuf *p, err_t err);
//static err_t DataTest_recv(void *arg, struct tcp_pcb *pcb, struct pbuf *p, err_t err);

static err_t HelloWorld_accept(void *arg, struct tcp_pcb *pcb, err_t err);
static void HelloWorld_conn_err(void *arg, err_t err);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Called when a data is received on the telnet connection
  * @param  arg	the user argument
  * @param  pcb	the tcp_pcb that has received the data
  * @param  p	the packet buffer
  * @param  err	the error value linked with the received data
  * @retval error value
  */
static err_t HelloWorld_recv(void *arg, struct tcp_pcb *pcb, struct pbuf *p, err_t err)
{
  struct pbuf *q;
  //struct RxBuffer *RxBuffer = (struct RxBuffer *)arg;
  uint8_t Rx_Buf[20];
  uint8_t j=0;
  int done;
  char *c,Tx_Buf[100];
  uint16_t i,charsum=0;
  

  /* We perform here any necessary processing on the pbuf */
  if (p != NULL) 
  {        
	/* We call this function to tell the LwIp that we have processed the data */
	/* This lets the stack advertise a larger window, so more data can be received*/
	tcp_recved(pcb, p->tot_len);


    done = 0;
    for(q=p; q != NULL; q = q->next) 
    {
      charsum += q->len;
      c = q->payload;
      for(i=0; i<q->len && !done; i++) 
      {
        if(j < MAX_NAME_SIZE) 
	{
          Rx_Buf[j++] = c[i];
          done=((c[0]=='u')&&(c[1]=='p')&&(c[2]=='d')&&(c[3]=='a')&&(c[4]=='t')&&(c[5]=='e')&&(i==5));
        }
      }
    }
    
    if(done)
    {
      flag = 1;
      
      tcp_write(pcb, Rx_Buf, charsum, TCP_WRITE_FLAG_COPY);
      
      memset(Rx_Buf,0,charsum);
    }
	/* End of processing, we free the pbuf */
    pbuf_free(p);
  }  
  //else if (err == ERR_OK) 
  else
  {
    /* When the pbuf is NULL and the err is ERR_OK, the remote end 
                                        is closing the connection. */
    /* We free the allocated memory and we close the connection */
    //mem_free(RxBuffer);
    //pbuf_free(p);
    return tcp_close(pcb);
  }
  return ERR_OK;

}

/**
  * @brief  This function when the Telnet connection is established
  * @param  arg  user supplied argument 
  * @param  pcb	 the tcp_pcb which accepted the connection
  * @param  err	 error value
  * @retval ERR_OK
  */
static err_t HelloWorld_accept(void *arg, struct tcp_pcb *pcb, err_t err)
{     
  pcb_command=pcb;
  
  /* Tell LwIP to associate this structure with this connection. */
  //tcp_arg(pcb, mem_calloc(sizeof(struct RxBuffer), 1));	
  
  /* Configure LwIP to use our call back functions. */
  //tcp_err(pcb, HelloWorld_conn_err);
  tcp_recv(pcb, HelloWorld_recv); 
  
  return ERR_OK;
}

/**
  * @brief  Initialize the hello application  
  * @param  None 
  * @retval None 
  */
void HelloWorld_init(void)
{
  //INT8U err;
  
  struct tcp_pcb *pcb;	            		
  
  /* Create a new TCP control block  */
  pcb = tcp_new();	                		 	

  /* Assign to the new pcb a local IP address and a port number */
  /* Using IP_ADDR_ANY allow the pcb to be used by any local interface */
  tcp_bind(pcb, IP_ADDR_ANY, 5185); 

  /* Set the connection to the LISTEN state */
  pcb = tcp_listen(pcb);				

  /* Specify the function to be called when a connection is established */	
  tcp_accept(pcb, HelloWorld_accept);   
										
}

/**
  * @brief  This function is called when an error occurs on the connection 
  * @param  arg
  * @parm   err
  * @retval None 
  */
static void HelloWorld_conn_err(void *arg, err_t err)
{
  struct RxBuffer *RxBuffer;
  RxBuffer = (struct RxBuffer *)arg;

  mem_free(RxBuffer);
}

void command_response(uint8_t *buf,uint8_t len)
{
  tcp_write(pcb_command, buf, len, TCP_WRITE_FLAG_COPY);
  tcp_output(pcb_command);
}


/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/


