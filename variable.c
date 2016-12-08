/*C**************************************************************************
* NAME:   variable.c
*----------------------------------------------------------------------------
* Copyright (c) 2013.
*----------------------------------------------------------------------------
* RELEASE:      2013.09.04
* REVISION:     9.4 
*----------------------------------------------------------------------------
* PURPOSE:
* This file contains the definition of the global variables
*****************************************************************************/

/*_____ I N C L U D E S ____________________________________________________*/
#include "config.h"                      /* system config definition */


/*_____ D E F I N I T I O N ________________________________________________*/
                   
        				
/*_____ D E C L A R A T I O N ______________________________________________*/
/* 系统计时 */
idata  Uint16  gl_ack_tick = 0;	         /* 上位机485口应答延时计时 tick */	
 data  Uint16  gl_delay_tick;            /* 通用延时用tick */  
xdata  Byte    gl_reply_tick;            /* 设备返回延时*/

/* variables for UART */
xdata  Byte  msg_buf[MAX_RecvFrame];     // received message, used for process
bdata  bit   msg_buf_valid;	             // received valid flag
xdata  Byte  recv_buf[MAX_RecvFrame];    // receiving buffer               
idata  Byte  recv_state;                 // receive state
idata  Byte  recv_timer;                 // receive time-out, 用于字节间超时判定
idata  Byte  recv_chksum;                // computed checksum
idata  Byte  recv_ctr;                   // reveiving pointer         

xdata  Byte  trans_buf[MAX_TransFrame];  // uart transfer buffer（发送前需要填充所有字节，除校验字节外）
idata  Byte  trans_ctr;                  // transfering pointer
idata  Byte  trans_size;                 // transfered bytes number（含所有字节，即含帧头和校验）
idata  Byte  trans_chksum;               // check-sum （边发送边计算）
bdata  bit   trans_occupy;               // 发送器被占用标志，1-被占用, 0-空闲                      

 data  Byte     uart_q_index;            // 正在发送某队列项的序号：若为0xFF, 表示没有任何项在发送
xdata  sUART_Q 	uart_q[UART_QUEUE_NUM];	 // 串口队列 

/* for UART2 */
xdata  Byte  msg2_buf[MAX_RecvFrame];    // received message, used for proceding
bdata  bit   msg2_buf_valid;	           // received valid flag
xdata  Byte  recv2_buf[MAX_RecvFrame];   // receiving buffer               
idata  Byte  recv2_state;                // receive state
idata  Byte  recv2_timer;                // receive time-out, 用于字节间超时判定
idata  Byte  recv2_chksum;               // computed checksum
idata  Byte  recv2_ctr;                  // reveiving pointer 

xdata  Byte  trans2_buf[MAX_TransFrame];  // uart transfer buffer（发送前需要填充所有字节，除校验字节外）
 data  Byte  trans2_ctr;                  // transfering pointer
 data  Byte  trans2_size;                 // transfered bytes number（含所有字节，即含帧头和校验）
 data  Byte  trans2_chksum;               // check-sum （边发送边计算）
bdata  bit   trans2_occupy;               // 发送器2被占用标志，1-被占用, 0-空闲                      

 data  Byte     uart2_q_index;            // 正在发送某队列项的序号：若为0xFF, 表示没有任何项在发送
xdata  sUART_Q 	uart2_q[UART2_QUEUE_NUM]; // 串口2队列

/* variables for beep */
bdata  bit     beep_flag;           // 蜂鸣标志: 0 - 禁鸣; 1 - 正在蜂鸣
 data  Uint16  beep_timer;          // 计时器，剩余蜂鸣时间, 单位:tick
xdata  Uint16  beep_during_temp;    // 预设的一次蜂鸣持续时间, 单位:tick 

/* variables for alarm output (继电器及LED) */
bdata  Byte    alarm_out_flag;  //报警输出标志：位值0 - 无报警（继电器上电吸合）; 位值1 - 报警(断电)
                                //联动输出标志：位值0 - 无输出（断电,使用常闭）;  位值1 - 有联动输出(继电器上电吸合，开路) 
                                //位址76543210  对应  X X 攀爬报警 报警2 报警1 联动输出 X X
								                //ZZX: 报警输出位值与实际硬件控制脚电平相反; 	联动输出位值与实际硬件控制脚电平相同							
       sbit    alarm3_flag  = alarm_out_flag^5;                                                
       sbit    alarm2_flag  = alarm_out_flag^4;
       sbit    alarm1_flag  = alarm_out_flag^3;
       sbit    alarm1C_flag = alarm_out_flag^2;
 data  Uint16  alarm1_timer;    // 计时器，报警器1已报警时间,单位:tick 
 data  Uint16  alarm2_timer;    // 计时器，报警器2已报警时间,单位:tick 
 data  Uint16  alarm3_timer;    // 计时器，攀爬报警已报警时间,单位tick
 data  Uint16  ld_timer;        // 计时器，联动输出剩余时间, 单位:tick
xdata  Uint16  ld_during_temp;  // 预设的一次联动最小输出时间, 单位:tick

/* variables for alarm flag */
bdata  bit     climb_alarm_flag; //攀爬报警标志：0-无报警；1-报警
bdata  bit     adl_alarm_flag;  //主控板左侧张力组合报警标志: 0 - 无报警; 1 - 报警
bdata  bit     adr_alarm_flag;  //主控板右侧张力组合报警标志: 0 - 无报警; 1 - 报警
																//  报警原因可能为：外力报警,张力静态基准值超范围报警
																//  ZZX: 已经过 mask 处理，包含扩展模块，但不包含级联
 data  Uint16  alarm_led_flag;	//LED报警指示: 0 - 无报警（灭）；1 - 报警（亮）
								                //  ZZX: 各位值与实际硬件控制脚电平相反								 						  								   

/* AD sample */
//(用户设置的基准)静态基准值上/下限、报警阀值上/下限 (单位：目前已改为使用采样值)
xdata  Uint16  ad_still_dn;         //静态拉力值下限
xdata  Uint16  ad_still_up;         //静态拉力值上限
xdata  Byte    ad_still_Dup[14];    //报警阀值上限

//(用户设置的基准)静态基准值上/下限、报警阀值上/下限，转换为采样值
idata  Uint16  ad_still_dn_s;       //静态拉力值下限(单位：10bit采样值)
idata  Uint16  ad_still_up_s;       //静态拉力值上限(单位：10bit采样值)
idata  Byte    ad_still_Dup_s[14];  //报警阀值上限(单位：10bit采样值)

idata  Byte        ad_index;        //正在采样的通道号, 取值范围0~13
 data  sAD_Sample  ad_sample;       //当前采样值
idata  Uint16      ad_sensor_mask;  //已安装并需要判定的sensor mask
				                            // 对应位 0 - 没有安装, 不判定
						  	                    //        1 - 已经安装，需要判定
                                    // 位    13   12   11   10    9 8 7 6 5   4 3 2 1 0 
																		// 对应  左7  左6  右7  右6   右5 ~ 右1   左5 ~ 左1                                    
                                    // ZZX: 各位值与实际读到的拨码开关脚值相反
xdata  Uint16   ad_sensor_mask_LR;  //按左右顺序重排序的sensor mask: 左8 ~ 1  右 8 ~ 1
bdata  bit      ad_sensor_extent;   //是否带扩展板: 1 - 带; 0 - 不带

idata  Uint16   ad_samp_pnum;       //采样点数
idata  sAD_Sum  ad_samp_equ[14];    //原始采样值求和，用于均衡去嘈声, 此后得到单采样点
idata  sAD_Sum  ad_samp_sum[14];    //采样值阶段求和（针对均衡去嘈声后采样点求和）
xdata  Union16  ad_chn_sample[14];  //循环保存的最新一轮采样值（已均衡去噪声，每通道一个点，循环保存）
								                    //  ZZX: 用于发送给PC
xdata  sAD_BASE ad_chn_base[14];    //各通道运行时静态基准值/上下限阀值（单位：采样值）
 data  Byte     ad_chn_over[14];    //各通道连续采样点(均衡后)的阀值判定： 0 - 范围内； 1 - 超阀值
                                    //每通道一个字节： CH0~13 对应 ad_chn_over[0~13]
								                    //某字节中的每个位对应顺序采样点的阀值判定									
idata  Uint16   ad_alarm_exts;      //外力报警标志（未经mask）：位值 0 - 无； 1 - 超阀值                    
idata  Uint16   ad_alarm_base;	    //静态张力报警标志（未经mask）：位值 0 - 允许范围内； 1 - 超允许范围                                   					

/* Doorkeep(门磁) */
bdata  bit   gl_dk_status;     //门磁开关状态（每1s动态检测）: 1 - 闭合; 0 - 打开(需要报警)                    
 data  Byte  gl_dk_tick;  	   //门磁检测计时tick

/* for system */
idata  Byte   system_status;   //系统状态
                               // 0 - 初始上电
                               // 1 - 基准值采样前延时(约5秒)
                               // 2 - 基准值采样(10秒左右)
                               // 3 - 实时监测
															 
idata  Byte   gl_comm_addr;    //本模块485通信地址（可能已改为规范值）																 
xdata  Byte   gl_addr_origin;  //地址拨码开关原值（已取反）

bdata  bit    system_2or1;     //双/单防区标志: 0 - 双（缺省）; 1 - 单												 

bdata  bit    uart_send_samp;  //实时采样值从UART送出标志: 0 - 不发送（缺省）; 1 - 发送

const  Byte   AREA_L_INDEX[7] = {0, 1, 2, 3, 4, 12, 13};   //左防区各道对应的板上(程序) index
const  Byte   AREA_R_INDEX[7] = {5, 6, 7, 8, 9, 10, 11};   //右防区各道对应的板上(程序) index
	
/* 传感器采样偏差 */
xdata  Uint16   sensor_sample_offset[14];    //传感器采样偏差：没有外力时，传感器采样值不为0，大约400左右，需要矫正。瞬间张力 = 采样值 - 采样偏差
                                             //0-13分别代表：左1~左5、右1~右5、右6~右7、左6~左7


//2016-12-07新增
xdata sAlarmDetailInfo  AlarmDetailInfo;//保存最后一次报警详细信息