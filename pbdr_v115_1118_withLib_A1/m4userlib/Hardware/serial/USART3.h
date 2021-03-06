
/**  @param
  *
  *
  */
 
#ifndef __USART3_H__
#define __USART3_H__
 /*
*********************************************************************************************************
*                                                 MODULE
*
* Note(s) : (1) This header file is protected from multiple pre-processor inclusion through use of the
*               BSP_SER present pre-processor macro definition.
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*                                              EXTERNS
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*                                               DEFINES
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*                                               INCLUDES
*********************************************************************************************************
*/

#include <string.h>
#include "BSP.h"

/*
*********************************************************************************************************
*                                             RS-232 SERVICES
*********************************************************************************************************
*/
void  BSP_Uart3_Init (CPU_INT32U  baud_rate);
void BSP_Uart3_EnableTxE(void);
CPU_INT08U  BSP_Uart3_RdByte (void);
void  BSP_Uart3_WrByte(CPU_INT08U  c);
CPU_INT08U  BSP_Uart3_RdBytelock (CPU_INT32U Time,CPU_INT08U *rx_byte);
CPU_INT08U	 BSP_Uart3_SendCommand(const CPU_INT08U *Comand,CPU_INT16U Length,CPU_INT08U *Respons,CPU_INT16U *ReLength,CPU_INT16U WaitTime);


int BSP_Uart3_SendByte(CPU_INT08U  data);
int BSP_Uart3_SendBuff(CPU_INT16U  len,CPU_INT08U *wrPt);


/*
*********************************************************************************************************
*                                              MODULE END
*********************************************************************************************************
*/

#endif /*USART3_H*/

