/*H**************************************************************************
* NAME:  comm_task.h
*----------------------------------------------------------------------------
* Copyright (c) 2008.
*----------------------------------------------------------------------------
* RELEASE:   2008.12.17
* REVISION:  1.0
*----------------------------------------------------------------------------
* PURPOSE:
* This file contains the communication task definition
****************************************************************************/

#ifndef _COMM_TASK_H_
#define _COMM_TASK_H_


/*_____ I N C L U D E S ____________________________________________________*/
#include "config.h"                         /* system configuration */
#include "lib_mcu\c51_drv.h"                /* c51 driver definition */
#include "lib_mcu\uart\uart_drv.h"          /* uart drive definition */


/*_____ M A C R O S ________________________________________________________*/
//#define COMM_START       0
//#define COMM_IDLE        1


/*_____ D E F I N I T I O N ________________________________________________*/


/*_____ D E C L A R A T I O N ______________________________________________*/
void  comm_task_init(void);
void  comm_task(void);
Byte  uart_get_buffer(void);
Byte  uart2_get_buffer(void);

Uint16 change_to_LR(Uint16 val);


#endif  /* _COMM_TASK_H_ */
