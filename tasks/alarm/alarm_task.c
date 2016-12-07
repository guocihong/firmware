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
#define ALARM_TEMPO  (3000/SCHEDULER_TICK)    //报警信号持续时间


/*_____ D E C L A R A T I O N ______________________________________________*/
/* for beep */
extern bdata  bit     beep_flag;        // 蜂鸣标志 : 0 - 禁鸣; 1 - 正在蜂鸣
extern  data  Uint16  beep_timer;       // 剩余蜂鸣时间, 单位tick
extern xdata  Uint16  beep_during_temp; // 蜂鸣最大持续时间

/* for alarm */
extern bdata  Byte    alarm_out_flag;   //报警输出标志：位值0 - 无报警（继电器上电吸合）;  位值1 - 报警(断电)
                                        //联动输出标志：位值0 - 无输出（断电,使用常闭）;  位值1 - 有联动输出(继电器上电吸合，开路) 
                                        //位址76543210  对应  X X 攀爬报警 报警2 报警1 联动输出 X X
								                        //ZZX: 报警输出位值与实际硬件控制脚电平相反; 	联动输出位值与实际硬件控制脚电平相同	
extern bdata  bit     alarm3_flag;
extern bdata  bit     alarm2_flag;
extern bdata  bit     alarm1_flag;
extern bdata  bit     alarm1C_flag;
extern  data  Uint16  alarm1_timer;   // 计时器，报警器1已报警时间,单位tick 
extern  data  Uint16  alarm2_timer;   // 计时器，报警器2已报警时间,单位tick
extern  data  Uint16  alarm3_timer;   // 计时器，攀爬报警已报警时间,单位tick

/* variables for 联动 */
extern  data  Uint16  ld_timer;       // 计时器，联动剩余输出时间, 单位tick
extern xdata  Uint16  ld_during_temp; // 预设的一次联动输出时间, 单位tick

extern bdata  bit     climb_alarm_flag; //攀爬报警标志：0-无报警；1-报警
extern bdata  bit     adl_alarm_flag; //左侧张力组合报警标志: 0 - 无报警; 1 - 超阀值，报警
extern bdata  bit     adr_alarm_flag; //右侧张力组合报警标志: 0 - 无报警; 1 - 超阀值，报警
									                    //  报警原因可能为：外力报警， 静态张力超范围报警																					
    
/* Doorkeep */   
extern bdata  bit     gl_dk_status;   //门磁开关状态: 1 - 闭合; 0 - 打开(需要报警)

/* for system */   
extern idata  Byte  system_status;    //系统状态

extern xdata  Uint16  ad_sensor_mask_LR;  //按左右顺序重排序的sensor mask: 左8 ~ 1  右 8 ~ 1																					
			 	

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
  beep_flag = 0;       // 无蜂鸣
  //Beep对应硬件管脚的初始化在main()中

  /* for alarm */
  alarm_out_flag = 0x00;  // 报警口1/2及联动均无报警输出/联动输出
  //对应硬件管脚的初始化在main()中

  climb_alarm_flag = 0;
  adl_alarm_flag = 0;  //初始上电后，左侧无组合报警
  adr_alarm_flag = 0;  //初始上电后，右侧无组合报警
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

  //1. 报警口1（左侧）
  if ((!gl_dk_status || adl_alarm_flag) && (system_status == SYS_CHECK))
  {	//门磁打开或左侧张力异常: 新报警或继续报警
    Disable_interrupt();
    alarm1_timer = 0;     //清报警器1已报警时间(至少报警3秒)
    Enable_interrupt();    
    if (alarm1_flag == 0)
    { //新报警     	  
      bRelay_A1 = 0;	   //报警
      alarm1_flag = 1;   //报警器1报警
      //beep
      if (beep_during_temp > 0)
      {	//允许beep
        Disable_interrupt();
        beep_timer = beep_during_temp;
        Enable_interrupt();
        if (!beep_flag)
        { //静音 -> 蜂鸣
          bBeep_Ctrl = 1;
          beep_flag = 1;
        }      
      }      
    }
    //联动
    if (ld_during_temp > 0)
    {	//需要联动输出
      Disable_interrupt();
      ld_timer = ld_during_temp;
      Enable_interrupt();
      if (!alarm1C_flag)
      { //无输出 -> 输出
        bRelay_L1 = 1;
        alarm1C_flag = 1;
      }      
    } 
  }

  //2. 报警口2（右侧）
  if ((!gl_dk_status || adr_alarm_flag) && (system_status == SYS_CHECK))
  {	//门磁打开或右侧张力异常: 新报警或继续报警
	  Disable_interrupt();
	  alarm2_timer = 0;    //清报警器2已报警时间(至少报警3秒) 
	  Enable_interrupt();
    if (alarm2_flag == 0)
    { //新报警	  
      bRelay_A2 = 0;	   //报警
      alarm2_flag = 1;   //报警器2报警
      //beep
      if (beep_during_temp > 0)
      {	//允许beep
        Disable_interrupt();
        beep_timer = beep_during_temp;
        Enable_interrupt();
        if (!beep_flag)
        { //静音 -> 蜂鸣
          bBeep_Ctrl = 1;
          beep_flag = 1;
        }      
      }
    }
    //联动
    if (ld_during_temp > 0)
    {	//需要联动输出
      Disable_interrupt();
      ld_timer = ld_during_temp;
      Enable_interrupt();
      if (!alarm1C_flag)
      { //无输出 -> 输出
        bRelay_L1 = 1;
        alarm1C_flag = 1;
      }      
    }    
  }
  
  //3.攀爬报警
  if ((climb_alarm_flag && ((ad_sensor_mask_LR >> 4) & 0x0001)) && (system_status == SYS_CHECK))
  {
    Disable_interrupt();
    alarm3_timer = 0;     //清报警器1已报警时间(至少报警3秒)
    Enable_interrupt();    
    if (alarm3_flag == 0)
    { //新报警     	  
      bRelay_A1 = 0;	   //报警
      bRelay_A2 = 0;	   //报警
      alarm3_flag = 1;     //报警
      //beep
      if (beep_during_temp > 0)
      {	//允许beep
        Disable_interrupt();
        beep_timer = beep_during_temp;
        Enable_interrupt();
        if (!beep_flag)
        { //静音 -> 蜂鸣
          bBeep_Ctrl = 1;
          beep_flag = 1;
        }      
      }      
    }
    
    //联动
    if (ld_during_temp > 0)
    {	//需要联动输出
      Disable_interrupt();
      ld_timer = ld_during_temp;
      Enable_interrupt();
      if (!alarm1C_flag)
      { //无输出 -> 输出
        bRelay_L1 = 1;
        alarm1C_flag = 1;
      }      
    } 
  }

  //3. 检查报警口1报警时间是否已到
  if (alarm1_flag)
  { //报警口1正在报警
    Disable_interrupt();
    temp16 = alarm1_timer;
    Enable_interrupt();
    if (temp16 > ALARM_TEMPO)
    { //报警口1已经到最大报警时间, 停止报警
	    alarm1_flag = 0;
        if(alarm3_flag == 0){
            bRelay_A1 = 1;
        }
	  }
  }

  //4. 检查报警口2报警时间是否已到
  if (alarm2_flag)
  {	//报警口2正在报警
    Disable_interrupt();
    temp16 = alarm2_timer;
    Enable_interrupt();    
    if (temp16 > ALARM_TEMPO)
    { //报警口2已经到最大报警时间, 停止报警
	    alarm2_flag = 0;
        if(alarm3_flag == 0){
            bRelay_A2 = 1;
        }
	  }
    
  }

  //5. 检查攀爬报警时间是否已到
  if (alarm3_flag)
  {	
    //攀爬报警正在报警
    Disable_interrupt();
    temp16 = alarm3_timer;
    Enable_interrupt();
    if (temp16 > ALARM_TEMPO)
    { //攀爬报警已经到最大报警时间, 停止报警
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
