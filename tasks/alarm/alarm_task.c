/*C**************************************************************************
* NAME:   alarm_task.c
*----------------------------------------------------------------------------
* Copyright (c) 2013.
*----------------------------------------------------------------------------
* RELEASE:     2013.09.05
* REVISION:    9.4     
*----------------------------------------------------------------------------
* PURPOSE:          
* This file contains the beep/alarm task and attached routines
*****************************************************************************/

/*_____ I N C L U D E S ____________________________________________________*/
#include "config.h"                  /* system configuration */
#include "lib_mcu\c51_drv.h"         /* c51 driver definition */
#include "alarm_task.h"              /* alarm task definition */


/*_____ D E F I N I T I O N ________________________________________________*/
#define ALARM_TEMPO  (3000/SCHEDULER_TICK)    //�����źų���ʱ��


/*_____ D E C L A R A T I O N ______________________________________________*/
/* for beep */
extern bdata  bit     beep_flag;        // ������־ : 0 - ����; 1 - ���ڷ���
extern  data  Uint16  beep_timer;       // ʣ�����ʱ��, ��λtick
extern xdata  Uint16  beep_during_temp; // ����������ʱ��

/* for alarm */
extern bdata  Byte    alarm_out_flag;   //���������־��λֵ0 - �ޱ������̵����ϵ����ϣ�;  λֵ1 - ����(�ϵ�)
                                        //���������־��λֵ0 - ��������ϵ�,ʹ�ó��գ�;  λֵ1 - ���������(�̵����ϵ����ϣ���·) 
                                        //λַ76543210  ��Ӧ  X X �������� ����2 ����1 ������� X X
								                        //ZZX: �������λֵ��ʵ��Ӳ�����ƽŵ�ƽ�෴; 	�������λֵ��ʵ��Ӳ�����ƽŵ�ƽ��ͬ	
extern bdata  bit     alarm3_flag;
extern bdata  bit     alarm2_flag;
extern bdata  bit     alarm1_flag;
extern bdata  bit     alarm1C_flag;
extern  data  Uint16  alarm1_timer;   // ��ʱ����������1�ѱ���ʱ��,��λtick 
extern  data  Uint16  alarm2_timer;   // ��ʱ����������2�ѱ���ʱ��,��λtick
extern  data  Uint16  alarm3_timer;   // ��ʱ�������������ѱ���ʱ��,��λtick

/* variables for ���� */
extern  data  Uint16  ld_timer;       // ��ʱ��������ʣ�����ʱ��, ��λtick
extern xdata  Uint16  ld_during_temp; // Ԥ���һ���������ʱ��, ��λtick

extern bdata  bit     climb_alarm_flag; //����������־��0-�ޱ�����1-����
extern bdata  bit     adl_alarm_flag; //���������ϱ�����־: 0 - �ޱ���; 1 - ����ֵ������
extern bdata  bit     adr_alarm_flag; //�Ҳ�������ϱ�����־: 0 - �ޱ���; 1 - ����ֵ������
									                    //  ����ԭ�����Ϊ������������ ��̬��������Χ����																					
    
/* Doorkeep */   
extern bdata  bit     gl_dk_status;   //�Ŵſ���״̬: 1 - �պ�; 0 - ��(��Ҫ����)

/* for system */   
extern idata  Byte  system_status;    //ϵͳ״̬

extern xdata  Uint16  ad_sensor_mask_LR;  //������˳���������sensor mask: ��8 ~ 1  �� 8 ~ 1																					
			 	

/*F**************************************************************************
* NAME: alarm_task_init
*----------------------------------------------------------------------------
* PARAMS:
* return:
*----------------------------------------------------------------------------
* PURPOSE: alarm task initialization
*****************************************************************************/
void alarm_task_init(void)
{
  /* for beep */
  beep_flag = 0;       // �޷���
  //Beep��ӦӲ���ܽŵĳ�ʼ����main()��

  /* for alarm */
  alarm_out_flag = 0x00;  // ������1/2���������ޱ������/�������
  //��ӦӲ���ܽŵĳ�ʼ����main()��

  climb_alarm_flag = 0;
  adl_alarm_flag = 0;  //��ʼ�ϵ���������ϱ���
  adr_alarm_flag = 0;  //��ʼ�ϵ���Ҳ�����ϱ���
}


/*F**************************************************************************
* NAME: alarm_task
*----------------------------------------------------------------------------
* PARAMS:
* return:
*----------------------------------------------------------------------------
* PURPOSE: alarm task
*----------------------------------------------------------------------------
* REQUIREMENTS:
*****************************************************************************/
void alarm_task(void)
{ 
  Uint16 temp16;

  //1. ������1����ࣩ
  if ((!gl_dk_status || adl_alarm_flag) && (system_status == SYS_CHECK))
  {	//�ŴŴ򿪻���������쳣: �±������������
    Disable_interrupt();
    alarm1_timer = 0;     //�屨����1�ѱ���ʱ��(���ٱ���3��)
    Enable_interrupt();    
    if (alarm1_flag == 0)
    { //�±���     	  
      bRelay_A1 = 0;	   //����
      alarm1_flag = 1;   //������1����
      //beep
      if (beep_during_temp > 0)
      {	//����beep
        Disable_interrupt();
        beep_timer = beep_during_temp;
        Enable_interrupt();
        if (!beep_flag)
        { //���� -> ����
          bBeep_Ctrl = 1;
          beep_flag = 1;
        }      
      }      
    }
    //����
    if (ld_during_temp > 0)
    {	//��Ҫ�������
      Disable_interrupt();
      ld_timer = ld_during_temp;
      Enable_interrupt();
      if (!alarm1C_flag)
      { //����� -> ���
        bRelay_L1 = 1;
        alarm1C_flag = 1;
      }      
    } 
  }

  //2. ������2���Ҳࣩ
  if ((!gl_dk_status || adr_alarm_flag) && (system_status == SYS_CHECK))
  {	//�ŴŴ򿪻��Ҳ������쳣: �±������������
	  Disable_interrupt();
	  alarm2_timer = 0;    //�屨����2�ѱ���ʱ��(���ٱ���3��) 
	  Enable_interrupt();
    if (alarm2_flag == 0)
    { //�±���	  
      bRelay_A2 = 0;	   //����
      alarm2_flag = 1;   //������2����
      //beep
      if (beep_during_temp > 0)
      {	//����beep
        Disable_interrupt();
        beep_timer = beep_during_temp;
        Enable_interrupt();
        if (!beep_flag)
        { //���� -> ����
          bBeep_Ctrl = 1;
          beep_flag = 1;
        }      
      }
    }
    //����
    if (ld_during_temp > 0)
    {	//��Ҫ�������
      Disable_interrupt();
      ld_timer = ld_during_temp;
      Enable_interrupt();
      if (!alarm1C_flag)
      { //����� -> ���
        bRelay_L1 = 1;
        alarm1C_flag = 1;
      }      
    }    
  }
  
  //3.��������
  if ((climb_alarm_flag && ((ad_sensor_mask_LR >> 4) & 0x0001)) && (system_status == SYS_CHECK))
  {
    Disable_interrupt();
    alarm3_timer = 0;     //�屨����1�ѱ���ʱ��(���ٱ���3��)
    Enable_interrupt();    
    if (alarm3_flag == 0)
    { //�±���     	  
      bRelay_A1 = 0;	   //����
      bRelay_A2 = 0;	   //����
      alarm3_flag = 1;     //����
      //beep
      if (beep_during_temp > 0)
      {	//����beep
        Disable_interrupt();
        beep_timer = beep_during_temp;
        Enable_interrupt();
        if (!beep_flag)
        { //���� -> ����
          bBeep_Ctrl = 1;
          beep_flag = 1;
        }      
      }      
    }
    
    //����
    if (ld_during_temp > 0)
    {	//��Ҫ�������
      Disable_interrupt();
      ld_timer = ld_during_temp;
      Enable_interrupt();
      if (!alarm1C_flag)
      { //����� -> ���
        bRelay_L1 = 1;
        alarm1C_flag = 1;
      }      
    } 
  }

  //3. ��鱨����1����ʱ���Ƿ��ѵ�
  if (alarm1_flag)
  { //������1���ڱ���
    Disable_interrupt();
    temp16 = alarm1_timer;
    Enable_interrupt();
    if (temp16 > ALARM_TEMPO)
    { //������1�Ѿ�����󱨾�ʱ��, ֹͣ����
	    alarm1_flag = 0;
        if(alarm3_flag == 0){
            bRelay_A1 = 1;
        }
	  }
  }

  //4. ��鱨����2����ʱ���Ƿ��ѵ�
  if (alarm2_flag)
  {	//������2���ڱ���
    Disable_interrupt();
    temp16 = alarm2_timer;
    Enable_interrupt();    
    if (temp16 > ALARM_TEMPO)
    { //������2�Ѿ�����󱨾�ʱ��, ֹͣ����
	    alarm2_flag = 0;
        if(alarm3_flag == 0){
            bRelay_A2 = 1;
        }
	  }
    
  }

  //5. �����������ʱ���Ƿ��ѵ�
  if (alarm3_flag)
  {	
    //�����������ڱ���
    Disable_interrupt();
    temp16 = alarm3_timer;
    Enable_interrupt();
    if (temp16 > ALARM_TEMPO)
    { //���������Ѿ�����󱨾�ʱ��, ֹͣ����
	    alarm3_flag = 0;
        if (alarm1_flag == 0) {
            bRelay_A1 = 1;
        }
        
        if (alarm2_flag == 0) {
            bRelay_A2 = 1;
        }
	}    
  }
}//end FUNC()
