/*H**************************************************************************
* NAME:    doorkeep_task.h
*----------------------------------------------------------------------------
* Copyright (c) 2008.
*----------------------------------------------------------------------------
* RELEASE:      2008.11.15   
* REVISION:     1.0
*----------------------------------------------------------------------------
* PURPOSE:
* This file contains the doorkeep scaning task definition
****************************************************************************/

#ifndef _DOORKEEP_TASK_H_
#define _DOORKEEP_TASK_H_


/*_____ I N C L U D E S ____________________________________________________*/
#include "config.h"                         /* system configuration */
#include "lib_mcu\c51_drv.h"                /* c51 driver definition */


/*_____ M A C R O S ________________________________________________________*/
/* ÈÎÎñ×´Ì¬ */
#define DK_READ_START    0
#define DK_READ_IDLE     1
#define DK_READ_DELAY    2


/*_____ D E C L A R A T I O N ______________________________________________*/
void  doorkeep_task_init(void);
void  doorkeep_task(void);


#endif  /* _DOORKEEP_TASK_H_ */
