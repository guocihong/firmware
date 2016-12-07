/*H**************************************************************************
* NAME:    c51_drv.h         
*----------------------------------------------------------------------------
* Copyright (c) 2008.
*----------------------------------------------------------------------------
* RELEASE:      2008.11.15      
* REVISION:     1.0     
*----------------------------------------------------------------------------
* PURPOSE:
* This file contains the C51 driver definition
*****************************************************************************/

#ifndef _C51_DRV_H_
#define _C51_DRV_H_


/*_____ I N C L U D E S ____________________________________________________*/
#include "lib_mcu\regs52.h"                /* SFR definition */


/*_____ M A C R O S ________________________________________________________*/


/*_____ D E F I N I T I O N ________________________________________________*/
/* interrupt enable */
#define Enable_interrupt()      (EA = 1)
#define Disable_interrupt()     (EA = 0)

#define EX0_enable_int()        (EX0 = 1)     
#define EX0_disable_int()       (EX0 = 0)
#define EX1_enable_int()        (EX1 = 1)
#define EX1_disable_int()       (EX1 = 0)

/* interrupt prio */
void  ex0_set_prio(Byte priority);   //外部中断0优先级设置
void  ex1_set_prio(Byte priority);   //外部中断1优先级设置

/** power mode **/
#define Set_idle_mode()         (PCON |= IDL_)
#define Set_power_down_mode()   (PCON |= PD_)


/*_____ D E C L A R A T I O N ______________________________________________*/


#endif  /* _C51_DRV_H_ */
