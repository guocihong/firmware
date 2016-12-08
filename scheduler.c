/*C**************************************************************************
* NAME:  scheduler.c
*----------------------------------------------------------------------------
* Copyright (c) 2013.
*----------------------------------------------------------------------------
* RELEASE:     2013.         
* REVISION:     9.4     
*----------------------------------------------------------------------------
* PURPOSE:
* This file contains the scheduler routines
*
* NOTES:
* Configuration:
*   - SCH_TYPE in scheduler.h header file
*   - SCH_TIMER in scheduler.h header file
*****************************************************************************/

/*_____ I N C L U D E S ____________________________________________________*/
#include "config.h"                         /* system definition */
#include "lib_mcu\wdt\wdt_drv.h"            /* WDT driver definition */
#include "lib_mcu\timer\timer_drv.h"        /* timer definition */
#include "lib_mcu\uart\uart_drv.h"          /* uart definition */
#include "scheduler.h"                      /* scheduler definition */


/*_____ D E F I N I T I O N ________________________________________________*/
/* ϵͳ��ʱ */
extern idata  Uint16  gl_ack_tick;	  /* Ӧ����ʱ��ʱ tick */
extern  data  Uint16  gl_delay_tick;  /* ͨ����ʱ��tick */ 

/* for UART1/2 : ��ʱ���� */
extern idata  Byte  recv_state;      // uart receive state
extern idata  Byte  recv_timer;      // uart receive time-out
extern idata  Byte  recv2_state;     // UART2 receive state
extern idata  Byte  recv2_timer;     // UART2 receive time-out	 

/* for beep */
extern bdata  bit     beep_flag;     // ������־: 0 - ����; 1 - ���ڷ���
extern  data  Uint16  beep_timer;    // ��ʱ��������ʣ��ʱ��, ��λ:tick

/* for alarm output (�̵�����LED) */
extern bdata  Byte    alarm_out_flag;  //���������־
extern bdata  bit     alarm3_flag;	 
extern bdata  bit     alarm2_flag;	 
extern bdata  bit     alarm1_flag;
extern bdata  bit     alarm1C_flag;
extern  data  Uint16  alarm1_timer;    // ��ʱ����������1�ѱ���ʱ��,��λtick 
extern  data  Uint16  alarm2_timer;    // ��ʱ����������2�ѱ���ʱ��,��λtick
extern  data  Uint16  alarm3_timer;    // ��ʱ�������������ѱ���ʱ��,��λtick
extern  data  Uint16  ld_timer;        // ��ʱ��������ʣ�����ʱ��, ��λtick

/* for AD */
extern idata  Byte        ad_index;     //���ڲ�����ͨ����, ȡֵ��Χ0~13
extern  data  sAD_Sample  ad_sample;    //���浱ǰ����ֵ
extern bdata  bit  ad_sensor_extent;    //�Ƿ����չ��: 1 - ��; 0 - ����

/* Doorkeep */   
extern  data  Byte   gl_dk_tick;        //�Ŵż���ʱtick  
			  
/* ����������ƫ�� */
extern xdata  Uint16   sensor_sample_offset[14];    //����������ƫ�û������ʱ������������ֵ��Ϊ0����Լ310���ң���Ҫ������˲������ = ����ֵ - ����ƫ��                 

/*_____ D E C L A R A T I O N ______________________________________________*/
static  void sch_time_init(void);



/*F**************************************************************************
* NAME:     sch_scheduler_init
*----------------------------------------------------------------------------
* PARAMS:
* return:
*----------------------------------------------------------------------------
* PURPOSE: Scheduler initialization
*----------------------------------------------------------------------------
* NOTE:	Task_x_init() and Task_x_fct() are defined in scheduler.h
*****************************************************************************/
void sch_scheduler_init(void)
{ 
  Task_1_init();   
  Task_2_init();     
  Task_3_init(); 
  Task_4_init();  
  Task_5_init();  
  Task_6_init(); 
  Task_7_init();  
  Task_8_init();  
  Task_9_init(); 
  Task_10_init(); 

  sch_time_init();      
}



/*F**************************************************************************
* NAME:     sch_scheduler
*----------------------------------------------------------------------------
* PARAMS: 
* return:
*----------------------------------------------------------------------------
* PURPOSE:  Task execution scheduler 
*****************************************************************************/
void sch_scheduler(void)
{
  while (1)
  {
    Task_1_fct();
    Task_2_fct();
    Task_3_fct();            
    Task_4_fct(); 
    Task_5_fct();    
    Task_6_fct(); 
    Task_7_fct();         
    Task_8_fct();          
    Task_9_fct();          
    Task_10_fct();         

	  #ifdef  Enable_WDT
      Wdt_refresh();         // 2s �������      
    #endif     
  }
}


#if (SCH_TIMER == SCH_TIMER0)
/*F**************************************************************************
* NAME:     sch_time_init
*----------------------------------------------------------------------------
* PARAMS:
* return:
*----------------------------------------------------------------------------
* PURPOSE: Scheduler time base (timer 0) initialization
*----------------------------------------------------------------------------
* NOTE:
*   mode 16-bit Timer, Time counter
*   T0_PRIO to be defined in config.h
*   TIM_LOW & TIM_HIGH defined in scheduler.h
*****************************************************************************/
void sch_time_init(void)
{ 
  T0_stop(); 
  T0_init(T0_NOT_GATED, T0_TIMER, T0_MODE_1);
  TL0 = TIM_LOW;
  TH0 = TIM_HIGH;
  t0_set_prio(T0_PRIO);       /* set-up priority */
  T0_enable_int();            /* enable interrupt */
  T0_start();                 /* start time base */
}


/*F**************************************************************************
* NAME:     sch_timer_int
*----------------------------------------------------------------------------
* PARAMS:
* return:
*----------------------------------------------------------------------------
* PURPOSE: Timer 0 interrupt function
******************************************************************************/
void sch_timer_int(void) interrupt TF0_VECTOR using 1
{  
    T0_stop();                /* stop timer */
    TL0 = TIM_LOW;            /* reload timer */ 
    TH0 = TIM_HIGH;
    T0_start();               /* restart timer */

    /* ��ADֵ */ 
    if (ad_sample.valid == FALSE)
    { 
        //ԭ�����Ѿ�������, ����д��������
        ad_sample.val = ADC_RES;             //����8λ
        ad_sample.val = ad_sample.val << 2;         
        ad_sample.val += (ADC_RESL & 0x03);  //�õ�10bit����ֵ
        ad_sample.index = ad_index;
        ad_sample.valid = TRUE;
        if ((ad_sensor_extent == 0) && (ad_index >= 10)){   //������չ��
            ad_sample.val = 0;
        }

        //����ֵ��ȥ����ƫ��
        if (ad_sample.val > sensor_sample_offset[ad_index]) {
            ad_sample.val -= sensor_sample_offset[ad_index];
        } else {
            ad_sample.val = 0;
        }
        
        //������һͨ������     
        if (ad_index >= 13){
            ad_index = 0;
        }else{   
            ad_index ++;   
        }

        if ((ad_sensor_extent == 0) && (ad_index >= 10)){   //������չ������������չͨ��
            P5 = 9;          //��չͨ��ֻ����ͨ��9
        }else{
            P5 = ad_index;	 //ѡ��ģ������	
        }

        ADC_CONTR = 0x80;          // 1000 0000b
        ADC_CONTR |= ADC_START_;   // ����ת��  
    }  
	
    /* increment task tick counters */
    gl_delay_tick ++;      /* ͨ����ʱ��tick */
    gl_dk_tick ++;         /* �Ŵż���ʱtick */
    if (gl_ack_tick > 0)
    gl_ack_tick --;      /* Ӧ����ʱ��ʱ */

    /* beep & alarm out */
    if (beep_flag)
    { 
        //����beep
        if (beep_timer > 0){
            beep_timer--;
        }
        
        if (beep_timer == 0)
        { 
            //����ʱ�䵽������
            bBeep_Ctrl = 0;
            beep_flag = 0;
        }
    }
    
    if (alarm1C_flag)
    {	
        //�����������
        if (ld_timer > 0){
            ld_timer --;
        }
        
        if (ld_timer == 0)
        { 
            //�������ʱ�䵽��ֹͣ���
            bRelay_L1 = 0;
            alarm1C_flag = 0;	  
        }
    }
    
    if (alarm1_flag)  alarm1_timer ++;
    if (alarm2_flag)  alarm2_timer ++;
    if (alarm3_flag)  alarm3_timer ++;

    /* UART�ֽ�֮����ճ�ʱ */
    if (recv_state != FSA_INIT) 
    {
        //�ǳ�ʼ״̬����Ҫ����Ƿ�ʱ
        if (recv_timer > 0){
            recv_timer --;
        }
        if (recv_timer == 0)  {
            recv_state = FSA_INIT;   //���ճ�ʱ, �ָ�����ʼ״̬      
        }
    }  

    /* UART2�ֽ�֮����ճ�ʱ */
    if (recv2_state != FSA_INIT) 
    { 
        //�ǳ�ʼ״̬����Ҫ����Ƿ�ʱ
        if (recv2_timer > 0){
            recv2_timer --;
        }
        
        if (recv2_timer == 0){
            recv2_state = FSA_INIT;   //���ճ�ʱ, �ָ�����ʼ״̬      
        }
    }
}

#endif
