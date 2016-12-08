/*C**************************************************************************
* NAME:  ad_task.c
*----------------------------------------------------------------------------
* Copyright (c) 2013.
*----------------------------------------------------------------------------
* RELEASE:      2013.09.05
* REVISION:     9.4     
*----------------------------------------------------------------------------
* PURPOSE:          
* This file contains the AD task and attached routines
*****************************************************************************/

/*_____ I N C L U D E S ____________________________________________________*/
#include <INTRINS.H>
#include <string.h>
#include "config.h"                         /* system configuration */
#include "lib_mcu\timer\timer_drv.h"        /* timer definition */
#include "lib_mcu\uart\uart_drv.h"          /* uart drive definition */
#include "lib_mcu\adc\adc_drv.h"            /* AD definition */
#include "tasks\comm\comm_task.h"           /* UART task definition */
#include "ad_task.h"                        /* ad task definition */


/*_____ D E F I N I T I O N ________________________________________________*/
#define AD_EQU_PNUM  4   //均衡去嘈声点数
                         
 
/*_____ D E C L A R A T I O N ______________________________________________*/
extern  data  Uint16   gl_delay_tick;   /* 通用延时用tick */                     

/* UART Queue */
extern  data  Byte     uart_q_index;               // 正在发送某队列项的序号：若为0xFF, 表示没有任何项进入发送流程
extern xdata  sUART_Q  uart_q[UART_QUEUE_NUM];	   // 串口队列 

/* variables for alarm output */
extern bdata  bit  climb_alarm_flag; //攀爬报警标志：0-无报警；1-报警
extern bdata  bit  adl_alarm_flag;  //左侧张力组合报警标志: 0 - 无报警; 1 - 超阀值，报警
extern bdata  bit  adr_alarm_flag;  //右侧张力组合报警标志: 0 - 无报警; 1 - 超阀值，报警
																		//  报警原因可能为：外力报警， 静态张力超范围报警
																		//  ZZX: 已经过 mask 处理, 未含门磁状态
extern  data  Uint16 alarm_led_flag;	//LED报警指示: 0 - 无报警（灭）；1 - 报警（亮）
								                      //ZZX: 没有经过mask处理，且组合外力报警和静态张力超范围报警

/* AD sample */
extern xdata  Uint16  ad_still_dn;         //静态拉力值下限
extern xdata  Uint16  ad_still_up;         //静态拉力值上限
extern xdata  Byte    ad_still_Dup[14];    //报警阀值上限

extern idata  Uint16  ad_still_dn_s;       //静态拉力值下限(单位：10bit采样值)
extern idata  Uint16  ad_still_up_s;       //静态拉力值上限(单位：10bit采样值)
extern idata  Byte    ad_still_Dup_s[14];  //报警阀值上限(单位：10bit采样值)

extern idata  Byte        ad_index;        //正在采样的通道号, 取值范围0~13
extern  data  sAD_Sample  ad_sample;       //当前采样值
extern idata  Uint16   ad_sensor_mask;     //已安装并需要判定的sensor mask: 0 - 禁止; 1 - 允许

extern idata  Uint16    ad_samp_pnum;      //采样点数(计算静态基准值时总采样点数)
extern idata  sAD_Sum   ad_samp_equ[14];   //均衡去嘈声求和
extern idata  sAD_Sum   ad_samp_sum[14];   //阶段求和
extern xdata  Union16   ad_chn_sample[14]; //最新一轮采样值（已均衡去噪声，每通道一个点，循环保存）

extern xdata  sAD_BASE  ad_chn_base[14];   //各通道静态基准值/上下限阀值
extern  data  Byte      ad_chn_over[14];   //各通道连续采样点(均衡后)的阀值判定： 0 - 范围内； 1 - 超阀值
                                           //每通道一个字节： CH0~13 对应 ad_chn_over[0~13]									
extern idata  Uint16      ad_alarm_exts;   //外力报警标志（无mask）： 位值 0 - 无； 1 - 超阀值
extern idata  Uint16      ad_alarm_base;	 //静态张力报警标志（无mask）： 位值 0 - 允许范围内； 1 - 超允许范围

/* for system */
extern idata  Byte   gl_comm_addr;     //本模块485通信地址（可能已改为规范值）																 
extern xdata  Byte   gl_addr_origin;   //地址拨码开关原值（已取反）
extern idata  Byte   system_status;    //系统状态							   
extern bdata  bit    uart_send_samp;   //基准值/采样值从UART送出标志: 0 - 不发送; 1 - 发送
									                     //  在 ad_task_init()	中初始化      
extern const  Byte   AREA_L_INDEX[7];  //左防区各道对应的板上(程序) index
extern const  Byte   AREA_R_INDEX[7];  //右防区各道对应的板上(程序) index

/* for this task: 用于基准值跟踪 */
static idata  Byte   md_point[14];     //用于基准值跟踪的计量点数

/* Doorkeep(门磁) */
extern bdata  bit    gl_dk_status;    //门磁开关状态（每1s动态检测）: 1 - 闭合; 0 - 打开(需要报警) 

//2016-12-07新增
extern xdata sAlarmDetailInfo  AlarmDetailInfo;//保存最后一次报警详细信息

/*F**************************************************************************
* NAME: ad_task_init
*----------------------------------------------------------------------------
* PARAMS:
* return:
*----------------------------------------------------------------------------
* PURPOSE:  AD task initialization
*----------------------------------------------------------------------------
* REQUIREMENTS:
*****************************************************************************/
void ad_task_init(void)
{ 
  Byte i;
  
  //初始化ADC硬件
  adc_init();
  ADC_CONTR |= ADC_POWER_;  //使能ADC电源

  //相关变量初始化
  ad_sample.valid = 0;  //空闲，可以写入新值
  ad_samp_pnum = 0;     //采样点数(计算静态基准值时总采样点数)
  for (i=0; i<14; i++)
  { 
    ad_samp_equ[i].sum = 0;	      //均衡去嘈声求和
    ad_samp_equ[i].point = 0;
    ad_samp_sum[i].sum = 0;	      //阶段求和
    ad_samp_sum[i].point = 0;
	  ad_chn_sample[i].w = 0;	      //最新一轮采样值
    ad_chn_base[i].base = 0;	    //各通道静态基准值/上下限阀值
    ad_chn_base[i].base_down = 0;
    ad_chn_base[i].base_up = 0;
	  ad_chn_over[i] = 0x00;	      //各通道连续采样点(均衡后)的阀值判定：均在范围内
    md_point[i] = 0;    //用于基准值跟踪的计量点数
  }    
  ad_alarm_exts = 0;	  //外力报警标志（无mask）: 无
  ad_alarm_base = 0;    //静态张力报警标志（无mask）：允许范围内   
  alarm_led_flag = 0;	  //所有报警LED为灭
  //P2 = 0xFF;
	//P0 = 0xFF;
                 
  //启动通道0#采样
  ad_index = 0;    
	P5 = ad_index;
  ADC_CONTR = 0x80;  // 1000 0000b
                     // 0xC0, 若CPU频率为20MHz, 则转换速率约50KHz
                     // 0x80, 若CPU频率为20MHz, 则转换速率约25KHz 
  ADC_CONTR |= ADC_START_;		  // 启动转换

/*
  //静态拉力值上/下限, 报警阀值上/下限牛顿数对应的采样值
  ad_still_dn_s  = (Uint16)(((Uint32)ad_still_dn  * SENSOR_RATIO_SAMP) >> SENSOR_RATIO_FS);   //静态拉力值下限
  ad_still_up_s  = (Uint16)(((Uint32)ad_still_up  * SENSOR_RATIO_SAMP) >> SENSOR_RATIO_FS);	  //静态拉力值上限
  for (i=0; i<8; i++)
    ad_still_Dup_s[i] = (Uint16)(((Uint32)ad_still_Dup[i] * SENSOR_RATIO_SAMP) >> SENSOR_RATIO_FS);	  //报警阀值上限差
*/

  ad_still_dn_s  = ad_still_dn;   //静态拉力值下限
  ad_still_up_s  = ad_still_up;	  //静态拉力值上限
  for (i=0; i<14; i++)
    ad_still_Dup_s[i] = ad_still_Dup[i];	//报警阀值上限差

  //是否实时发送采样值
  uart_send_samp = 0x00;   //缺省：不发送
}
						

/*F**************************************************************************
* NAME: ad_task
*----------------------------------------------------------------------------
* PARAMS:
* return:
*----------------------------------------------------------------------------
* PURPOSE: AD sample task
*----------------------------------------------------------------------------
* NOTE:	Vin = (val / 1024) * VCC
*****************************************************************************/
void ad_task(void)
{ 
  Byte    i, j;      //循环变量
  Uint16  temp16;    //临时变量
  Uint8   index;     //采样通道号
  Uint16  val_temp;  //新送入的10bit采样值,  后作临时变量
  Uint16  val;       //4点均衡后得到的平均采样值, 作为一个可进行超限判断的最小点

  if (ad_sample.valid)
  {  //有新采样数据到达
   //0.保存到临时变量
   val_temp = ad_sample.val;
   index    = ad_sample.index;     

   //1. 保存到均衡去嘈声求和中
   ad_samp_equ[index].sum += val_temp;
   ad_samp_equ[index].point ++;

   //2. 当前通道是否满去嘈声点数
   if (ad_samp_equ[index].point == AD_EQU_PNUM)
   { //已满去嘈声点数，可求出均衡后的一个点
     //2.a 求出对应通道的一个采样点
     val = ad_samp_equ[index].sum >> 2;  //除于4

     //2.b 清零当前通道的去嘈声求和结构
     ad_samp_equ[index].sum = 0;
     ad_samp_equ[index].point = 0;
        
     //2.c 保存实时采样值，且判是否需要发送
     ad_chn_sample[index].w = val;   //保存到最新一轮采样值数组中
     if (uart_send_samp && (index == 13))
     { //需要发送, 且已经完成14个通道的一次完整采样，可发送采样值
			 i = uart_get_buffer();
			 if (i < UART_QUEUE_NUM)
			 { //找到了空闲buffer, 写入data
                 uart_q[i].package_type = 1;         //设备自身的数据包
                 
				 uart_q[i].tdata[0] = FRAME_STX;
				 uart_q[i].tdata[1] = 0xFF;
				 uart_q[i].tdata[2] = 0x00;
				 //左ch1 ~ ch7				 				 		
				 for (j=0; j<7; j++)
				 {
					 uart_q[i].tdata[3 + (j<<1)] = ad_chn_sample[AREA_L_INDEX[j]].b[0];
					 uart_q[i].tdata[4 + (j<<1)] = ad_chn_sample[AREA_L_INDEX[j]].b[1];  
				 }
				 uart_q[i].tdata[17] = 0;
				 uart_q[i].tdata[18] = 0;
				 //右ch1 ~ ch7
				 for (j=0; j<7; j++)
				 {
					 uart_q[i].tdata[19 + (j<<1)] = ad_chn_sample[AREA_R_INDEX[j]].b[0];
					 uart_q[i].tdata[20 + (j<<1)] = ad_chn_sample[AREA_R_INDEX[j]].b[1];  
				 }
				 uart_q[i].tdata[33] = 0;
				 uart_q[i].tdata[34] = 0;				 				 
				 uart_q[i].len = 36;			 
			 }
			 else
			 { //无空闲buffer, 丢弃本命令
				 //检查: 若有队列项正在发送, 等待它完成
				 while (uart_q_index != 0xFF);	//若死锁,将引起 WDT 复位				           
			 }                            
     }//end if (uart_send_samp && (index ==3))

     //2.d 由系统状态决定数据的处理
     switch (system_status)
     {
       //case SYS_PowerON:   //上电
       //case SYS_B5S:       //5秒延时
       //       break;

       case SYS_SAMP_BASE: //初始上电时的静态基准值采样
              //存入阶段和
              ad_samp_sum[index].sum += val;
              ad_samp_pnum ++;
              if (ad_samp_pnum == 448)   
              { //已经满基准值采样点数（每通道32点，均衡后, 耗时约10秒) 
                //1.计算均值和上下限
                for (i=0; i<14; i++)
                { 
                  //基准
                  ad_chn_base[i].base = ad_samp_sum[i].sum >> 5;   //除于32 
                  //下限 = 基准 * （1 / 3）
                  val_temp = ad_chn_base[i].base;
                  ad_chn_base[i].base_down = (val_temp >> 1) - (val_temp >> 3) - (val_temp >> 4);   //下限 = 1/2 - 1/8 - 1/16 = 0.3125     
                  //上限
                  if ((1023 - ad_chn_base[i].base) > ad_still_Dup_s[i])
                    ad_chn_base[i].base_up = ad_chn_base[i].base + ad_still_Dup_s[i];
                  else
                    ad_chn_base[i].base_up = 1023;
                  //检查静态张力是否在允许范围内	  
                  check_still_stress(i);
                  //复位阶段和变量，准备用于自适应阀值跟踪
                  ad_samp_sum[i].sum = 0; 
                  ad_samp_sum[i].point = 0;
                }           
                //2. 级联
                //Disable_interrupt(); 
                //jl_sch_tick = JL_ACT_PRID;
                //Enable_interrupt();                                  
                //3. 状态-> 实时检测				  
                system_status = SYS_CHECK;
              }
              break;

      case SYS_CHECK: //实时检测
              //a. 判当前值
              ad_chn_over[index] = ad_chn_over[index] << 1;   //Bit0填0，因此缺省在允许范围内
              if ((val >= ad_chn_base[index].base_down) && (val <= ad_chn_base[index].base_up))
              { //在张力上/下限允许范围内
                //a. 清标志(缺省)
                //b. 计入跟踪基准值求和中
                ad_samp_sum[index].sum += val;
                ad_samp_sum[index].point ++;
                if (ad_samp_sum[index].point == 2)
                { //满2点(约需0.6秒)
                  //b.0 计算这2点均值 
                  val_temp = ad_samp_sum[index].sum >> 1;   //除于2, 得到这2点的均值
                  //b.1 更新基准值
                  if (ad_chn_base[index].base > (val_temp + 1))
                  { //至少小2, 在缓慢松弛
                    //ZZX: 立即跟踪, 跟踪差值的 1/2
                    val_temp = (ad_chn_base[index].base - val_temp) >> 1;
                    if (ad_chn_base[index].base >= val_temp)
                    {	  
                      ad_chn_base[index].base -= val_temp;
                      //同步更新上下限
                      val_temp = ad_chn_base[index].base;
                      ad_chn_base[index].base_down = (val_temp >> 1) - (val_temp >> 3) - (val_temp >> 4);   // = 1/2 - 1/8 - 1/16  
                      if ((1023 - ad_chn_base[index].base) > ad_still_Dup_s[index])
                        ad_chn_base[index].base_up = ad_chn_base[index].base + ad_still_Dup_s[index];
                      else
                        ad_chn_base[index].base_up = 1023; 
                      //检查静态张力是否在允许范围内	  
                      check_still_stress(index);                          
                    }
                    //清缓慢张紧跟踪变量
                    md_point[index] = 0;
                  }
                  else if (val_temp > (ad_chn_base[index].base + 1))
                  { //至少大2, 缓慢张紧					
                    md_point[index] ++;					  
                    if (md_point[index] >= DEF_ModiBASE_PT)
                    { //已满缓慢张紧时的连续计量点数, 进行一次跟踪
                      //1. 跟踪基准值
                      if (ad_chn_base[index].base < 1023)
                      { //可以递增1
                        ad_chn_base[index].base ++;
                        //同步更新上下限
                        val_temp = ad_chn_base[index].base;
                        ad_chn_base[index].base_down = (val_temp >> 1) - (val_temp >> 3) - (val_temp >> 4);   // = 1/2 - 1/8 - 1/16
                        if (ad_chn_base[index].base_up < 1023)
                          ad_chn_base[index].base_up ++;  
                        //检查静态张力是否在允许范围内	  
                        check_still_stress(index);							                          
                      }
                      //2. 清缓慢张紧跟踪变量
                      md_point[index] = 0;
                    }
                  }
                  //else
                  //{ //当前值在基准值的 +/- 2 之内					
                  //  //保留缓慢张紧跟踪变量值
                  //}                 
                  //b.2 复位阶段和变量 - 用于4点4点平均的求和结构
                  ad_samp_sum[index].sum = 0;
                  ad_samp_sum[index].point = 0;
                }
              }
              else
              { //超出阀值, 置标志
                ad_chn_over[index] |= 0x01; 
                //ZZX: 需要清用于跟踪的求和结构吗?         
              }
        
              //c. 计算报警标志  
              if ((ad_chn_over[index] & 0x0F) == 0x0F)
              { //连续4点超范围，此通道有报警
                //连续4点，对应持续时间为280ms X 4 = 1.1秒
                if (index == 9) {
                    climb_alarm_flag = 1;
                }
                ad_alarm_exts |= (Uint16)0x01 << index;
                
                //20110817: 立即更新基准值
                //if ( (val > (ad_chn_base[index].base + DEF_ModiBASE_TH)) ||
                //     (ad_chn_base[index].base > (val + DEF_ModiBASE_TH)) )
                //{
                ad_chn_base[index].base = val;
                val_temp = ad_chn_base[index].base;
                ad_chn_base[index].base_down = (val_temp >> 1) - (val_temp >> 3) - (val_temp >> 4);   // = 1/2 - 1/8 - 1/16    
                if ((1023 - ad_chn_base[index].base) > ad_still_Dup_s[index])
                  ad_chn_base[index].base_up = ad_chn_base[index].base + ad_still_Dup_s[index];
                else
                  ad_chn_base[index].base_up = 1023;
                //检查静态张力是否在允许范围内	  
                check_still_stress(index); 					
                //复位阶段和变量 - 用于4点4点平均的求和结构
                ad_samp_sum[index].sum = 0;
                ad_samp_sum[index].point = 0;               
                //清缓慢张紧跟踪变量
                md_point[index] = 0;   //用于基准值跟踪的计量点数
                //}
                
                //2016-12-07新增
                //保存报警详细信息
                save_alarm_detail_info();
              }
              else if ((ad_chn_over[index] & 0x0F) == 0x00)
              { //无报警
                if (index == 9) {
                    climb_alarm_flag = 0;
                }
                ad_alarm_exts &= ~((Uint16)0x01 << index);
              }  
      
              //LED指示(无mask)
              temp16 = (ad_alarm_exts | ad_alarm_base) & 0x3FFF; 
              if (alarm_led_flag != temp16)
              { //LED指示信息有变化
                alarm_led_flag = temp16;
								P2 = ~ LOW(alarm_led_flag);
								P0 = ~ HIGH(alarm_led_flag);							
              }                
              
              //更新组合报警标志(mask后)
              temp16 = ((ad_alarm_exts & 0xFDFF) | ad_alarm_base) & 0x3FFF; 
              temp16 &= ad_sensor_mask;			
              //判左侧组合报警
              if ((temp16 & 0x301F) == 0)				
                adl_alarm_flag = 0;   //无报警				
              else
                adl_alarm_flag = 1;  	//有报警								
              //判右侧组合报警
              if ((temp16 & 0x0FE0) == 0)				
                adr_alarm_flag = 0;   //无报警				
              else
                adr_alarm_flag = 1;	  //有报警							  				  								      	  					                                               
              break;
      }//end switch
    }//end 满去嘈声点数

    //3.当前采样值处理完毕，允许新的采样值输入
    ad_sample.valid = FALSE;
  }//end if (ad_sample.valid)
}//end FUNC()


/*F**************************************************************************
* NAME: check_still_stress
*----------------------------------------------------------------------------
* PARAMS: 
*         index : 通道号
* return:
*----------------------------------------------------------------------------
* PURPOSE: 检查指定通道的静态张力是否在允许范围内
*****************************************************************************/
void check_still_stress(index)
{
  if ((ad_chn_base[index].base >= ad_still_dn_s) && (ad_chn_base[index].base <= ad_still_up_s))
  { //在允许范围内, 清标志
	  ad_alarm_base &= ~((Uint16)0x01 << index);
  }
  else
  { //超出允许范围，置标志
	  ad_alarm_base |= (Uint16)0x01 << index;
  }
  
  //控制杆不存在静态报警，只存在外力报警
  ad_alarm_base &= ~(1 << 9);
}

//2016-12-07新增
//保存报警详细信息
void save_alarm_detail_info(void)
{
    Byte i;
    
    AlarmDetailInfo.DoorKeepAlarm = (Byte)gl_dk_status;
    AlarmDetailInfo.StaticAlarm = ad_alarm_base;
    AlarmDetailInfo.ExternalAlarm = ad_alarm_exts;
    
    for (i = 0; i < 14; i++) {
        AlarmDetailInfo.StaticBaseValue[i].base = ad_chn_base[i].base;
        AlarmDetailInfo.InstantSampleValue[i].w = ad_chn_sample[i].w;
    }
}