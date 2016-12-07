/*H**************************************************************************
* NAME:  alarm_task.h
*----------------------------------------------------------------------------
* Copyright (c) 2013.
*----------------------------------------------------------------------------
* RELEASE:      2013.09.06  
* REVISION:     9.4
*----------------------------------------------------------------------------
* PURPOSE:
* This file contains the beep/alarm task definition
****************************************************************************/

#ifndef _ALARM_TASK_H_
#define _ALARM_TASK_H_


/*_____ I N C L U D E S ____________________________________________________*/
#include "config.h"                         /* system configuration */
#include "lib_mcu\c51_drv.h"                /* c51 driver definition */


/*_____ M A C R O S ________________________________________________________*/


/*_____ D E F I N I T I O N ________________________________________________*/   
#define ALARM_TASK_START   0
#define ALARM_TASK_IDLE    1


/*_____ D E C L A R A T I O N ______________________________________________*/
void  alarm_task_init(void);
void  alarm_task(void);


#endif  /* _ALARM_TASK_H_ */
