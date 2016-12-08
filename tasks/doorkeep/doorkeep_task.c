/*C**************************************************************************
* NAME:   doorkeep_task.c
*----------------------------------------------------------------------------
* Copyright (c) 2008.
*----------------------------------------------------------------------------
* RELEASE:      2008.11.15
* REVISION:     1.0     
*----------------------------------------------------------------------------
* PURPOSE:          
* This file contains the doorkeep scanning task and attached routines
*
* NOTES:
*****************************************************************************/

/*_____ I N C L U D E S ____________________________________________________*/
#include "config.h"                  /* system configuration */
#include "lib_mcu\c51_drv.h"         /* c51 driver definition */
#include "doorkeep_task.h"           /* doorkeep task definition */


/*_____ D E F I N I T I O N ________________________________________________*/
#define DK_CHECK_TEMP    (1000/SCHEDULER_TICK)	  /* �Ŵſ��ؼ������ */
#define DK_CONFORM_TEMP  ( 100/SCHEDULER_TICK)    /* �Ŵſ��ر仯ȷ����ʱ */


/*_____ D E C L A R A T I O N ______________________________________________*/
extern bdata  bit   gl_dk_status;    //�Ŵſ���״̬��ÿ1s��̬��⣩: 1 - �պ�; 0 - ��(��Ҫ����)                    
extern  data  Byte  gl_dk_tick;  	 //�Ŵż���ʱtick
extern idata  Byte  system_status;   //ϵͳ״̬	
static  data  Byte  dk_read_state;   //task state

extern void save_alarm_detail_info(void);

/*F**************************************************************************
* NAME: doorkeep_task_init
*----------------------------------------------------------------------------
* PARAMS:
* return:
*----------------------------------------------------------------------------
* PURPOSE:  doorkeep scan task initialization
*****************************************************************************/
void doorkeep_task_init(void)
{ 
  gl_dk_status = 1;      //�ϵ�ʱ��ȱʡΪ�պ�  
  dk_read_state = DK_READ_START;
}


/*F**************************************************************************
* NAME: doorkeep_task
*----------------------------------------------------------------------------
* PARAMS:
* return:
*----------------------------------------------------------------------------
* PURPOSE: doorkeep scanning process
*****************************************************************************/
void doorkeep_task(void)
{ 
  switch (dk_read_state)
  {
    case DK_READ_START: //��ʼ����
                    if (system_status > SYS_SAMP_BASE)
                    { //ϵͳ����ʵʱ���׶βſ�ʼ����Ŵ�
                      gl_dk_tick = 0;
                      dk_read_state = DK_READ_IDLE;
                    }
                    break;  
    
    case DK_READ_IDLE://����Ŵ����ޱ仯
                    if (gl_dk_tick > DK_CHECK_TEMP)
                    { //ÿ����һ��
                      gl_dk_tick = 0;
                      if (gl_dk_status == 1)
                      { //ԭ��Ϊ�Ŵűպ�
                        if (bDoorKeeper == 0)
                        { //�Ŵſ��ܱ���, ������ʱȷ�Ͻ׶�   					     	                      
                          dk_read_state = DK_READ_DELAY;
                        }
                      }
                      else
                      { //ԭ��Ϊ�ŴŴ�   
                        if (bDoorKeeper == 1)
                        { //�Ŵſ��ܱ��պ�, ������ʱȷ�Ͻ׶�   		                      
                          dk_read_state = DK_READ_DELAY;
                        }
                      }
                    }
                    break;     

    case DK_READ_DELAY://��ʱȷ��
                    if (gl_dk_tick > DK_CONFORM_TEMP)
                    { //��ʱʱ�䵽, �ж��Ƿ����ȶ��ı仯    
                      if ((gl_dk_status == 1) && (bDoorKeeper == 0))
                      { //�Ŵ��Ѿ���
                        gl_dk_status = 0;
                          
                        //2016-12-07����
                        //���汨����ϸ��Ϣ
                        save_alarm_detail_info();
                      }						  
                      else if ((gl_dk_status == 0) && (bDoorKeeper == 1))
                      { //�Ŵ��Ѿ��պ�
                        gl_dk_status = 1;
                      }				  
                      gl_dk_tick = 0;
                      dk_read_state = DK_READ_IDLE;                                    
                    }
	                  break;
  }//end switch (dk_read_state)
}//end FUNC()
