/*C**************************************************************************
* NAME:  comm_task.c
*----------------------------------------------------------------------------
* Copyright (c) 2013.
*----------------------------------------------------------------------------
* RELEASE:      2013.09.05
* REVISION:     9.4     
*----------------------------------------------------------------------------
* PURPOSE:          
* This file contains the communication task and attached routines
*
*****************************************************************************/

/*_____ I N C L U D E S ____________________________________________________*/
#include <INTRINS.H>
#include <string.h>
#include "config.h"                         /* system configuration */
#include "lib_mcu\wdt\wdt_drv.h"            /* WDT driver definition */
#include "lib_mcu\timer\timer_drv.h"        /* timer definition */
#include "lib_mcu\uart\uart_drv.h"          /* uart drive definition */
#include "lib_mcu\eeprom\eeprom_drv.h"      /* eeprom drive definition */
#include "comm_task.h"                      /* comm task definition */
#include "command.h"                        /* system command definition */


/*_____ D E F I N I T I O N ________________________________________________*/
#define REPLY_DLY   (100/SCHEDULER_TICK)    //收到PC命令后的应答延时

/*_____ D E C L A R A T I O N ______________________________________________*/
extern idata  Uint16 gl_ack_tick;	            /* 应答延时计时 tick */
extern xdata  Byte   gl_reply_tick;              /* 设备返回延时*/

/* UART1 */	
extern xdata  Byte  msg_buf[MAX_RecvFrame];     // received message, used for proceding
extern bdata  bit   msg_buf_valid;	            // received valid flag
extern xdata  Byte  recv_buf[MAX_RecvFrame];    // receiving buffer               
extern idata  Byte  recv_state;                 // receive state
extern idata  Byte  recv_timer;                 // receive time-out, 用于字节间超时判定
extern idata  Byte  recv_chksum;                // computed checksum
extern idata  Byte  recv_ctr;                   // reveiving pointer         

extern xdata  Byte  trans_buf[MAX_TransFrame];  // uart transfer message buffer
extern idata  Byte  trans_ctr;                  // transfer pointer
extern idata  Byte  trans_size;                 // transfer bytes number  
extern idata  Byte  trans_chksum;               // computed check-sum of already transfered message 
extern bdata  bit   trans_occupy;               // 发送器被占用标志，1-被占用, 0-空闲  

extern  data  Byte     uart_q_index;            // 正在发送某队列项的序号：若为0xFF, 表示没有任何项进入发送流程
extern xdata  sUART_Q  uart_q[UART_QUEUE_NUM];	// 串口队列

/* for UART2 */
extern xdata  Byte  msg2_buf[MAX_RecvFrame];    // received message, used for proceding
extern bdata  bit   msg2_buf_valid;	            // received valid flag
extern xdata  Byte  recv2_buf[MAX_RecvFrame];   // receiving buffer               
extern idata  Byte  recv2_state;                // receive state
extern idata  Byte  recv2_timer;                // receive time-out, 用于字节间超时判定
extern idata  Byte  recv2_chksum;               // computed checksum
extern idata  Byte  recv2_ctr;                  // reveiving pointer 

extern xdata  Byte  trans2_buf[MAX_TransFrame];    // uart transfer buffer（发送前需要填充所有字节，除校验字节外）
extern  data  Byte  trans2_ctr;                    // transfering pointer
extern  data  Byte  trans2_size;                   // transfered bytes number（含所有字节，即含帧头和校验）
extern  data  Byte  trans2_chksum;                 // check-sum （边发送边计算）
extern bdata  bit   trans2_occupy;                 // 发送器2被占用标志，1-被占用, 0-空闲                      

extern  data  Byte     uart2_q_index;              // 正在发送某队列项的序号：若为0xFF, 表示没有任何项在发送
extern xdata  sUART_Q  uart2_q[UART2_QUEUE_NUM];   // 串口2队列

/* variables for beep */
extern xdata  Uint16  beep_during_temp;   //预设的蜂鸣持续时间, 单位tick

/* variables for 联动 */
extern xdata  Uint16  ld_during_temp;   //预设的一次联动最小输出时间, 单位tick  

/* for alarm */
extern bdata  Byte  alarm_out_flag;     //报警输出标志：位值0 - 无报警（上电）;  位值1 - 报警(无电)                                      
extern bdata  bit   alarm2_flag;
extern bdata  bit   alarm1_flag;
extern bdata  bit   alarm1C_flag;	

extern bdata  bit   climb_alarm_flag; //攀爬报警标志：0-无报警；1-报警
extern bdata  bit   adl_alarm_flag;     //左侧张力组合报警标志: 0 - 无报警; 1 - 超阀值，报警
extern bdata  bit   adr_alarm_flag;     //右侧张力组合报警标志: 0 - 无报警; 1 - 超阀值，报警
extern  data  Uint16  alarm_led_flag;	  //LED报警指示: 0 - 无报警（灭）；1 - 报警（亮）

/* for AD & Sensor */
extern xdata  Uint16  ad_still_dn;        //静态拉力值下限
extern xdata  Uint16  ad_still_up;        //静态拉力值上限
extern xdata  Byte    ad_still_Dup[14];   //报警阀值上限差

extern idata  Uint16  ad_still_dn_s;      //静态拉力值下限(单位：10bit采样值)
extern idata  Uint16  ad_still_up_s;      //静态拉力值上限(单位：10bit采样值)
extern idata  Byte    ad_still_Dup_s[14]; //报警阀值上限差(单位：10bit采样值)

extern idata  Uint16     ad_sensor_mask;  //已安装并需要判定的sensor mask
				                                  // 对应位 0 - 没有安装, 不判定
						  	                          //        1 - 已经安装，需要判定
extern xdata  Uint16  ad_sensor_mask_LR;  //按左右顺序重排序的sensor mask: 左8 ~ 1  右 8 ~ 1																					
extern xdata  Union16   ad_chn_sample[14];  //最新一轮采样值（已均衡去噪声，每通道一个点，循环保存）
extern xdata  sAD_BASE  ad_chn_base[14];    //各通道静态基准值/上下限阀值
extern idata  Uint16    ad_alarm_exts;      //外力报警标志（无mask）： 位值 0 - 无； 1 - 超阀值
extern idata  Uint16    ad_alarm_base;	    //静态张力报警标志（无mask）： 位值 0 - 允许范围内； 1 - 超允许范围

/* Doorkeep(门磁) */
extern bdata  bit    gl_dk_status;    //门磁开关状态（每1s动态检测）: 1 - 闭合; 0 - 打开(需要报警)                    

/* for system */
extern idata  Byte   gl_comm_addr;     //本模块485通信地址（可能已改为规范值）																 
extern xdata  Byte   gl_addr_origin;   //地址拨码开关原值（已取反）
extern idata  Byte   system_status;    //系统状态	
extern bdata  bit    system_2or1;      //双/单防区标志: 0 - 双（缺省）; 1 - 单												 
extern bdata  bit    uart_send_samp;   //基准值/采样值从UART送出标志: 0 - 不发送; 1 - 发送
									                     //  在 ad_task_init()	中初始化      
extern const  Byte   AREA_L_INDEX[7];  //左防区各道对应的板上(程序) index
extern const  Byte   AREA_R_INDEX[7];  //右防区各道对应的板上(程序) index
                 
/* 传感器采样偏差 */
extern xdata  Uint16   sensor_sample_offset[14];    //传感器采样偏差：没有外力时，传感器采样值不为0，大约310左右，需要矫正。瞬间张力 = 采样值 - 采样偏差                 

//2016-12-07新增
extern xdata  sAlarmDetailInfo  AlarmDetailInfo;//保存最后一次报警详细信息

extern void check_still_stress(index);

/*F**************************************************************************
* NAME: comm_task_init
*----------------------------------------------------------------------------
* PARAMS:
* return:
*----------------------------------------------------------------------------
* PURPOSE: Communication task initialization
*****************************************************************************/
void comm_task_init(void)
{ 
  Byte i;

  //UART1初始化
  msg_buf_valid = 0;  
  for (i=0; i<UART_QUEUE_NUM; i++)
  {
    uart_q[i].flag = 0;            //均空闲
    //uart_q[i].external = 0;	     //485总线
    //uart_q[i].tdata[0] = FRAME_STX;
    //uart_q[i].need_wait_ack = 0; //不需应答 
  }
  uart_q_index = 0xFF;   //无队列项进入发送流程

  //UART2 初始化
  msg2_buf_valid = 0;
  for (i=0; i<UART2_QUEUE_NUM; i++)
  {
    uart2_q[i].flag = 0;                //均空闲
		//uart2_q[i].external = 0;	        //
		//uart2_q[i].tdata[0] = FRAME_STX;	//帧头
		//uart2_q[i].need_wait_ack = 0;     //不需应答 
  }
  uart2_q_index = 0xFF;   //无队列项进入发送流程  

  //UART硬件初始化
  uart_init();            //之后，已经准备好串口收发，只是还未使能全局中断
}
						

/*F**************************************************************************
* NAME: comm_task
*----------------------------------------------------------------------------
* PARAMS:
* return:
*----------------------------------------------------------------------------
* PURPOSE: communication task
*****************************************************************************/
void comm_task(void)
{ 
  Byte   i, j;
  Uint16 temp16;

  //1. 接收并处理 UART1：来自上位机的命令包
  if (msg_buf_valid)
  {	//有来自上位机的命令包
		//a.1 是否需要转发本命令
		if ((msg_buf[0] == CMD_ADDR_BC) || (msg_buf[0] != gl_comm_addr))
		{ //广播地址或非本设备, 需要转发到 UART2      
			//在UART2 队列中找空闲Buffer
			i = uart2_get_buffer();
			if (i < UART2_QUEUE_NUM)				 
			{ //找到了空闲buffer, 写入data
				uart2_q[i].tdata[0] = FRAME_STX;
				memcpy(&uart2_q[i].tdata[1], msg_buf, msg_buf[2] + 3);                    
				uart2_q[i].len = msg_buf[2] + 5;
			}
			else
			{ //无空闲buffer
				//检查: 若有队列项正在发送, 等待完成
			  while (uart2_q_index != 0xFF);	//若死锁,将引起 WDT 复位				                
			}
		}

		//a.2 是否需要执行本命令
		if ((msg_buf[0] == CMD_ADDR_BC) || (msg_buf[0] == gl_comm_addr))
		{ //广播地址或指定本设备, 需要执行 
			switch (msg_buf[3])
			{						
				case CMD_DADDR_qSTAT://询问防区状态 - 报告给上位机
						 //在UART1队列中找空闲Buffer
						 i = uart_get_buffer();
						 if (i < UART_QUEUE_NUM)
						 { //找到了空闲buffer, 准备应答
							 uart_q[i].tdata[0] = FRAME_STX;	   //帧头
							 uart_q[i].tdata[1] = msg_buf[1];	   //目的地址
							 uart_q[i].tdata[2] = gl_comm_addr;  //源地址
							 if (gl_comm_addr == CMD_ADDR_UNSOLV)
							 { //本设备无有效地址
								 //只回参数应答
								 uart_q[i].tdata[3] = 1;
								 uart_q[i].tdata[4] = CMD_DADDR_aPARA;
								 uart_q[i].len = 6;	
							 }
							 else
							 { //有有效地址,回防区状态                                 
								 if ((alarm_out_flag & 0x38) == 0x00)
								 { //2个防区均无报警
									 uart_q[i].tdata[3] = 1;
									 uart_q[i].tdata[4] = CMD_ACK_OK;
									 uart_q[i].len = 6;													
								 }
								 else
								 { //有报警
									 uart_q[i].tdata[3] = 2;
									 uart_q[i].tdata[4] = CMD_DADDR_aSTAT;
									 uart_q[i].tdata[5] = (alarm_out_flag & 0x38) >> 3;
									 uart_q[i].len = 7;
								 }
							 }							
						 }
						 else
						 { //无空闲buffer, 丢弃本命令
							 //检查: 若有队列项正在发送, 等待它完成
							 while (uart_q_index != 0xFF);	//若死锁,将引起 WDT 复位				           
						 }
						 break;													 

				case CMD_DADDR_qPARA://询问参数
						 //在UART1队列中找空闲Buffer
						 i = uart_get_buffer();
						 if (i < UART_QUEUE_NUM)
						 { //找到了空闲buffer, 准备应答
							 uart_q[i].tdata[0] = FRAME_STX;	//帧头
							 uart_q[i].tdata[1] = msg_buf[1];	//目的地址
							 uart_q[i].tdata[2] = gl_comm_addr;	    //源地址																 
							 uart_q[i].tdata[3] = 1;
							 uart_q[i].tdata[4] = CMD_DADDR_aPARA;
							 uart_q[i].len = 6;														
						 }
						 else
						 { //无空闲buffer, 丢弃本命令
							 //检查: 若有队列项正在发送, 等待它完成
							 while (uart_q_index != 0xFF);	//若死锁,将引起 WDT 复位				           
						 }
						 break;	
                         
                case 0xE3://设置延时时间
                        //1. 写入flash
                        flash_enable();                              
                        flash_erase(EEPROM_SECTOR8);                                        
                        flash_write(msg_buf[4], EEPROM_SECTOR8 + 1);  
                        flash_write(0x5a, EEPROM_SECTOR8);                                                              
                        flash_disable();
                    
                        //2. 更新变量
                        gl_reply_tick = msg_buf[4];
                    
                        break;
                
                case 0xE4://读取延时时间
						 //在UART1队列中找空闲Buffer
						 i = uart_get_buffer();
						 if (i < UART_QUEUE_NUM)
						 { //找到了空闲buffer, 准备应答
							 uart_q[i].tdata[0] = FRAME_STX;	//帧头
							 uart_q[i].tdata[1] = msg_buf[1];	//目的地址
							 uart_q[i].tdata[2] = gl_comm_addr;	    //源地址																 
							 uart_q[i].tdata[3] = 2;
							 uart_q[i].tdata[4] = 0xF4;
                             uart_q[i].tdata[5] = gl_reply_tick;
							 uart_q[i].len = 7;														
						 }
						 else
						 { //无空闲buffer, 丢弃本命令
							 //检查: 若有队列项正在发送, 等待它完成
							 while (uart_q_index != 0xFF);	//若死锁,将引起 WDT 复位				           
						 }
						 break;	

	      case CMD_ZL_PRE://张力/脉冲专用命令标志
						 switch (msg_buf[5]) 
						 {					 																							
							 case 0x10: //读配置参数
                                //在UART队列中找空闲Buffer
                                i = uart_get_buffer();
                                if (i < UART_QUEUE_NUM)
                                { //找到了空闲buffer, 写入data
                                    uart_q[i].tdata[0] = FRAME_STX;
                                  uart_q[i].tdata[1] = msg_buf[1];	    //目的地址
                                  uart_q[i].tdata[2] = gl_comm_addr;	  //源地址																 
                                  uart_q[i].tdata[3] = 0x1E;
                                  uart_q[i].tdata[4] = CMD_ZL_PRE;															
                                    uart_q[i].tdata[5] = 0x1C;
                                    uart_q[i].tdata[6] = 0x08;
                                    uart_q[i].tdata[7] = HIGH(ad_sensor_mask_LR);    
                                    uart_q[i].tdata[8] = LOW(ad_sensor_mask_LR);
                                    uart_q[i].tdata[9]  = HIGH(ad_still_dn);
                                    uart_q[i].tdata[10] = LOW(ad_still_dn);
                                    uart_q[i].tdata[11] = HIGH(ad_still_up);
                                    uart_q[i].tdata[12] = LOW(ad_still_up);
                                    uart_q[i].tdata[13]  = 66;   //ad_still_Ddn;
                                    for (j=0; j<7; j++)
                                      uart_q[i].tdata[14 + j] = ad_still_Dup[AREA_L_INDEX[j]]; 
                                    uart_q[i].tdata[21] = 0;
                                    for (j=0; j<7; j++)
                                      uart_q[i].tdata[22 + j] = ad_still_Dup[AREA_R_INDEX[j]]; 
                                    uart_q[i].tdata[29] = 0;																											
                                    uart_q[i].tdata[30] = system_2or1;
                                    uart_q[i].tdata[31] = gl_addr_origin;
                                    uart_q[i].tdata[32] = (Byte)(((Uint32)beep_during_temp * SCHEDULER_TICK) / 1000);	//声光报警输出时间
                                    uart_q[i].tdata[33] = (Byte)(((Uint32)ld_during_temp * SCHEDULER_TICK) / 1000);	  //联动输出时间              																					
                                    uart_q[i].len = 35;
                                }
                                else
                                { //无空闲buffer, 丢弃本命令
                                    //检查: 若有队列项正在发送, 等待它完成
                                    while (uart_q_index != 0xFF);	//若死锁,将引起 WDT 复位				           
                                }                 
                                break;
												
							 case 0x11: //读报警输出口状态
                                //在UART队列中找空闲Buffer
                                i = uart_get_buffer();
                                if (i < UART_QUEUE_NUM)
                                { //找到了空闲buffer, 写入data
                                    uart_q[i].tdata[0] = FRAME_STX;
                                  uart_q[i].tdata[1] = msg_buf[1];	    //目的地址
                                  uart_q[i].tdata[2] = gl_comm_addr;	  //源地址																 
                                  uart_q[i].tdata[3] = 0x05;
                                  uart_q[i].tdata[4] = CMD_ZL_PRE;
                                  uart_q[i].tdata[5] = 0x03;
                                  uart_q[i].tdata[6] = 0x19;
                                  uart_q[i].tdata[7] = system_2or1;
                                  uart_q[i].tdata[8] = (alarm_out_flag & 0x18) >> 3;														
                                    uart_q[i].len = 10;
                                } 
                                else
                                { //无空闲buffer, 丢弃本命令
                                    //检查: 若有队列项正在发送, 等待它完成
                                    while (uart_q_index != 0xFF);	//若死锁,将引起 WDT 复位				           
                                }                
                                break;
													
							 case 0x12: //读报警实时信息
                                //在UART队列中找空闲Buffer
                                i = uart_get_buffer();
                                if (i < UART_QUEUE_NUM)
                                { //找到了空闲buffer, 写入data
                                    uart_q[i].tdata[0] = FRAME_STX;
                                    uart_q[i].tdata[1] = msg_buf[1];	    //目的地址
                                    uart_q[i].tdata[2] = gl_comm_addr;	  //源地址																 
                                    uart_q[i].tdata[3] = 0x0A;
                                    uart_q[i].tdata[4] = CMD_ZL_PRE;
                                    uart_q[i].tdata[5] = 0x08;
                                    uart_q[i].tdata[6] = 0x1A;													
                                    uart_q[i].tdata[7] = HIGH(ad_sensor_mask_LR);    
                                    uart_q[i].tdata[8] = LOW(ad_sensor_mask_LR);														
                                    temp16 = change_to_LR(ad_alarm_exts);																						
                                    uart_q[i].tdata[9] = HIGH(temp16);
                                    uart_q[i].tdata[10] = LOW(temp16);
                                    temp16 = change_to_LR(ad_alarm_base);
                                    uart_q[i].tdata[11] = HIGH(temp16);
                                    uart_q[i].tdata[12] = LOW(temp16);														
                                    uart_q[i].tdata[13] = (Byte)(!gl_dk_status);                     
                                    uart_q[i].len = 15;												
                                } 
                                else
                                { //无空闲buffer, 丢弃本命令
                                    //检查: 若有队列项正在发送, 等待它完成
                                    while (uart_q_index != 0xFF);	//若死锁,将引起 WDT 复位				           
                                }                
                                break;

							 case 0x14: //读瞬态张力
                                //在UART队列中找空闲Buffer
                                i = uart_get_buffer();
                                if (i < UART_QUEUE_NUM)
                                { //找到了空闲buffer, 写入data
                                    uart_q[i].tdata[0] = FRAME_STX;
                                    uart_q[i].tdata[1] = msg_buf[1];	    //目的地址
                                    uart_q[i].tdata[2] = gl_comm_addr;	  //源地址																 
                                    uart_q[i].tdata[3] = 0x23;
                                    uart_q[i].tdata[4] = CMD_ZL_PRE;
                                    uart_q[i].tdata[5] = 0x21;
                                    uart_q[i].tdata[6] = 0x1C;																																																				
                                    for (j=0; j<7; j++)
                                    { //左7~1
                                        temp16 = ad_chn_sample[AREA_L_INDEX[j]].w;
                                        uart_q[i].tdata[7+(j<<1)] = HIGH(temp16);
                                        uart_q[i].tdata[8+(j<<1)] = LOW(temp16);
                                    }
                                    uart_q[i].tdata[21] = 0;
                                    uart_q[i].tdata[22] = 0;														
                                  for (j=0; j<7; j++)
                                    { //右7~1
                                        temp16 = ad_chn_sample[AREA_R_INDEX[j]].w;
                                        uart_q[i].tdata[23+(j<<1)] = HIGH(temp16);
                                        uart_q[i].tdata[24+(j<<1)] = LOW(temp16);
                                    }
                                    uart_q[i].tdata[37] = 0;
                                    uart_q[i].tdata[38] = 0;																																																						
                                    uart_q[i].len = 40;
                                }
                                else
                                { //无空闲buffer, 丢弃本命令
                                    //检查: 若有队列项正在发送, 等待它完成
                                    while (uart_q_index != 0xFF);	//若死锁,将引起 WDT 复位				           
                                }                
                                break;
											
							 case 0x15: //读静态张力基准
                                //在UART队列中找空闲Buffer
                                i = uart_get_buffer();
                                if (i < UART_QUEUE_NUM)
                                { //找到了空闲buffer, 写入data
                                    uart_q[i].tdata[0] = FRAME_STX;
                                    uart_q[i].tdata[1] = msg_buf[1];	    //目的地址
                                    uart_q[i].tdata[2] = gl_comm_addr;	  //源地址																 
                                    uart_q[i].tdata[3] = 0x23;
                                    uart_q[i].tdata[4] = CMD_ZL_PRE;
                                    uart_q[i].tdata[5] = 0x21;
                                    uart_q[i].tdata[6] = 0x1D;																																																				
                                    for (j=0; j<7; j++)
                                    { //左7~1
                                        temp16 = ad_chn_base[AREA_L_INDEX[j]].base;
                                        uart_q[i].tdata[7+(j<<1)] = HIGH(temp16);
                                        uart_q[i].tdata[8+(j<<1)] = LOW(temp16);
                                    }
                                    uart_q[i].tdata[21] = 0;
                                    uart_q[i].tdata[22] = 0;														
                                  for (j=0; j<7; j++)
                                    { //右7~1
                                        temp16 = ad_chn_base[AREA_R_INDEX[j]].base;
                                        uart_q[i].tdata[23+(j<<1)] = HIGH(temp16);
                                        uart_q[i].tdata[24+(j<<1)] = LOW(temp16);
                                    }
                                    uart_q[i].tdata[37] = 0;
                                    uart_q[i].tdata[38] = 0;																																																						
                                    uart_q[i].len = 40;														
                                }  
                                else
                                { //无空闲buffer, 丢弃本命令
                                    //检查: 若有队列项正在发送, 等待它完成
                                    while (uart_q_index != 0xFF);	//若死锁,将引起 WDT 复位				           
                                }                
                                break;

							 case 0x17: //通用应答
                                //在UART队列中找空闲Buffer
                                i = uart_get_buffer();
                                if (i < UART_QUEUE_NUM)
                                { //找到了空闲buffer, 写入data
                                    uart_q[i].tdata[0] = FRAME_STX;
                                    uart_q[i].tdata[1] = msg_buf[1];	    //目的地址
                                    uart_q[i].tdata[2] = gl_comm_addr;	  //源地址																 
                                    uart_q[i].tdata[3] = 0x04;
                                    uart_q[i].tdata[4] = CMD_ZL_PRE;
                                    uart_q[i].tdata[5] = 0x02;
                                    uart_q[i].tdata[6] = 0x1F;																																									
                                    uart_q[i].tdata[7] = gl_addr_origin;													
                                    uart_q[i].len = 9;
                                } 
                                else
                                { //无空闲buffer, 丢弃本命令
                                    //检查: 若有队列项正在发送, 等待它完成
                                    while (uart_q_index != 0xFF);	//若死锁,将引起 WDT 复位				           
                                }                
                                break;                             
												
							 case 0x55: //AD采样值传输控制
                                if (msg_buf[6] == 0x00)		                
                                    uart_send_samp = 0;   //停止传输				   
                                else                
                                    uart_send_samp = 1;	  //开始传输                   
                                break;
												
							 case 0x40: //设置静态张力值范围
                                //1. 写入flash
                                flash_enable();                              
                                flash_erase(EEPROM_SECTOR3);                                        
                                flash_write(msg_buf[6], EEPROM_SECTOR3 + 1);  
                                flash_write(msg_buf[7], EEPROM_SECTOR3 + 2);
                                flash_write(msg_buf[8], EEPROM_SECTOR3 + 3);  
                                flash_write(msg_buf[9], EEPROM_SECTOR3 + 4);
                                flash_write(0x5a, EEPROM_SECTOR3);                                                              
                                flash_disable();
                                //2. 更新变量
                                ad_still_dn = ((Uint16)msg_buf[6] << 8) + msg_buf[7];	 //下限
                                ad_still_up = ((Uint16)msg_buf[8] << 8) + msg_buf[9];	 //上限
                                //3. 同步更新对应采样值
                                //ad_still_dn_s = (Uint16)(((Uint32)ad_still_dn  * SENSOR_RATIO_SAMP) >> SENSOR_RATIO_FS);   //静态拉力值下限
                                //ad_still_up_s = (Uint16)(((Uint32)ad_still_up  * SENSOR_RATIO_SAMP) >> SENSOR_RATIO_FS);	 //静态拉力值上限									  
                                ad_still_dn_s = ad_still_dn;    //ZZX: 直接使用采样值
                                ad_still_up_s = ad_still_up;
                                //4. 检查当前静态张力值
                                if (system_status == SYS_CHECK)
                                {	//已开始运行检测
                                    for (i=0; i<14; i++)
                                    {
                                        check_still_stress(i);
                                    }
                                }
                                break;
																									
							 case 0x50: //设置报警阀值 (仅上限)
                                //1. 写入flash并更新变量
                                flash_enable();                              
                                flash_erase(EEPROM_SECTOR4);                                        
                                flash_write(msg_buf[6], EEPROM_SECTOR4 + 1); 	 //下限浮动值，比例
                                //ZZX: 存入 Flash 的下限比例, 目前没有被读取使用						
                                for (j=0; j<7; j++)
                                { //左1 ~7
                                    ad_still_Dup[AREA_L_INDEX[j]] = msg_buf[7 + j];	 
                                    flash_write(msg_buf[7 + j], EEPROM_SECTOR4 + 2 + j);
                                }							 
                                for (j=0; j<7; j++)
                                { //右1 ~7
                                    ad_still_Dup[AREA_R_INDEX[j]] = msg_buf[15 + j];	 
                                    flash_write(msg_buf[15 + j], EEPROM_SECTOR4 + 10 + j);
                                }				
                                flash_write(0x5a, EEPROM_SECTOR4);                                                               
                                flash_disable();													
                                //2.同步更新采样域值
                                for (j=0; j<14; j++)
                                {
                                    ad_still_Dup_s[j] = ad_still_Dup[j];
                                }													
                                //下限固定取基准值的 1/3
                                //3. 更新换算后的张力报警上限(采样值)													
                                if (system_status == SYS_CHECK)
                                {	//已开始运行检测
                                    for (i=0; i<14; i++)
                                    {					       
                                        if ((1023 - ad_chn_base[i].base) > ad_still_Dup_s[i])
                                            ad_chn_base[i].base_up = ad_chn_base[i].base + ad_still_Dup_s[i];
                                        else
                                            ad_chn_base[i].base_up = 1023; 					  
                                    }
                                }													
                                break;	
															
							 case 0x60: //设置声光报警时间	              
                                //1. 写入flash				  
                                flash_enable();                              
                                flash_erase(EEPROM_SECTOR5);                                        
                                flash_write(msg_buf[6], EEPROM_SECTOR5 + 1);  
                                flash_write(0x5a, EEPROM_SECTOR5);                                                              
                                flash_disable();
                                //2. 更新变量
                                beep_during_temp = (Uint16)(((Uint32)msg_buf[6] * 1000) / SCHEDULER_TICK);				  
                                break;

							 case 0x70: //设置联动报警输出时间	              	              
                                //1. 写入flash				  
                                flash_enable();                              
                                flash_erase(EEPROM_SECTOR6);                                        
                                flash_write(msg_buf[6], EEPROM_SECTOR6 + 1);  
                                flash_write(0x5a, EEPROM_SECTOR6);                                                              
                                flash_disable();
                                //2. 更新变量
                                ld_during_temp = (Uint16)(((Uint32)msg_buf[6] * 1000) / SCHEDULER_TICK);				  				  
                                break;	

                            case 0xF1: //设置传感器采样偏差---->消除电路上的误差                  
                                //1. 写入flash并更新变量
                                flash_enable();
                                flash_erase(EEPROM_SECTOR7);

                                for (j = 0; j < 5; j++) {
                                    //左1 ~5
                                    sensor_sample_offset[j] =
                                        ((Uint16)msg_buf[6 + (j << 1)] << 8) + msg_buf[7 + (j << 1)];
                                    flash_write(msg_buf[6 + (j << 1)], EEPROM_SECTOR7 + 1 + (j << 1));
                                    flash_write(msg_buf[7 + (j << 1)], EEPROM_SECTOR7 + 2 + (j << 1));
                                }

                                for (j = 0; j < 5; j++) {
                                    //右1 ~5
                                    sensor_sample_offset[5 + j] =
                                        ((Uint16)msg_buf[22 + (j << 1)] << 8) + msg_buf[23 + (j << 1)];
                                    flash_write(msg_buf[22 + (j << 1)], EEPROM_SECTOR7 + 11 + (j << 1));
                                    flash_write(msg_buf[23 + (j << 1)], EEPROM_SECTOR7 + 12 + (j << 1));
                                }

                                for (j = 0; j < 2; j++) {
                                    //右6 ~7
                                    sensor_sample_offset[10 + j] =
                                        ((Uint16)msg_buf[32 + (j << 1)] << 8) + msg_buf[33 + (j << 1)];
                                    flash_write(msg_buf[32 + (j << 1)], EEPROM_SECTOR7 + 21 + (j << 1));
                                    flash_write(msg_buf[33 + (j << 1)], EEPROM_SECTOR7 + 22 + (j << 1));
                                }
                                
                                for (j = 0; j < 2; j++) {
                                    //左6 ~7
                                    sensor_sample_offset[12 + j] =
                                        ((Uint16)msg_buf[16 + (j << 1)] << 8) + msg_buf[17 + (j << 1)];
                                    flash_write(msg_buf[16 + (j << 1)], EEPROM_SECTOR7 + 25 + (j << 1));
                                    flash_write(msg_buf[17 + (j << 1)], EEPROM_SECTOR7 + 26 + (j << 1));
                                }
                                
                                //杆自身
                                sensor_sample_offset[9] = 0;
                                
                                flash_write(0x5a, EEPROM_SECTOR7);
                                flash_disable(); 

                                break;     

                            //2016-12-07新增
                            case 0xF8://读取报警详细信息                                
                                get_alarm_detail_info();
                                break;
                }//end switch
				break;													
			}//end switch	命令类型
            
            //a.3 设置应答延时
            Disable_interrupt();	
            gl_ack_tick = REPLY_DLY + (gl_comm_addr - 16) * gl_reply_tick / SCHEDULER_TICK;
            Enable_interrupt(); 
		}//end if 广播地址或指定本设备, 需要执行
		
    //b. 复位标志
    msg_buf_valid = FALSE;	   
  }//end if (msg_buf_valid) 						 

  //2. 接收并处理UART2：来自下位机的命令包
  if (msg2_buf_valid)
  {	//有来自下位机的命令包
    //a. 转发此命令
		//在UART1队列中找空闲Buffer
		i = uart_get_buffer();
		if (i < UART_QUEUE_NUM)				 
		{ //找到了空闲buffer, 写入data
			uart_q[i].tdata[0] = FRAME_STX;
		  memcpy(&uart_q[i].tdata[1], msg2_buf, msg2_buf[2] + 3);                    
			uart_q[i].len = msg2_buf[2] + 5;
		}
		else
		{ //无空闲buffer
			//检查: 若有队列项正在发送, 等待完成
			while (uart_q_index != 0xFF);	//若死锁,将引起 WDT 复位				                
		}
    //b. 复位标志
    msg2_buf_valid = FALSE;	   
  }//end if (msg2_buf_valid)	  
//
  //3. UART1 队列发送
  if ((uart_q_index == 0xFF) && (recv_state == FSA_INIT) && (gl_ack_tick == 0))
  {	//UART1无进入发送流程的队列项, 找是否有等待发送的项
    for (i=0; i<UART_QUEUE_NUM; i++)
    {
			if (uart_q[i].flag == 1)
			{	//有等待发送的项，安排此项发送
				uart_q[i].flag = 2;
				uart_q_index = i;
				memcpy(trans_buf, uart_q[i].tdata, uart_q[i].len - 1);
				trans_size = uart_q[i].len;
				uart_start_trans();
				break;
			}
    }
  }
  
  //4. UART2 队列发送
  if ((uart2_q_index == 0xFF) && (recv2_state == FSA_INIT))  
  {	//UART2无进入发送流程的队列项, 找是否有等待发送的项
    for (i=0; i<UART2_QUEUE_NUM; i++)
    {
			if (uart2_q[i].flag == 1)
			{	//有等待发送的项，安排此项发送
				uart2_q[i].flag = 2;
				uart2_q_index = i;
				memcpy(trans2_buf, uart2_q[i].tdata, uart2_q[i].len - 1);
				trans2_size = uart2_q[i].len;
				uart2_start_trans();
				break;
			}
    }
  }  
}//end FUNC()


/*F**************************************************************************
* NAME: uart_get_buffer
*----------------------------------------------------------------------------
* PARAMS:          
* return: Byte
*         若返回值 >= UART_QUEUE_NUM, 则表示没有申请到空闲buffer
*----------------------------------------------------------------------------
* PURPOSE: 在串口队列中寻找空闲队列项，若找到，返回队列项序号(0 ~ (UART_QUEUE_NUM-1))
*----------------------------------------------------------------------------
* REQUIREMENTS:
*****************************************************************************/
Byte uart_get_buffer(void)
{
  Byte i, flag;

  for (i=0; i<UART_QUEUE_NUM; i++)
  { 
    Disable_interrupt();
		flag = uart_q[i].flag;
		Enable_interrupt();
		if (flag == 0)
		{ //已找到空闲Buffer
			uart_q[i].flag = 1;
			break;
		}
  }
  return i;
}


/*F**************************************************************************
* NAME: uart2_get_buffer
*----------------------------------------------------------------------------
* PARAMS:          
* return: Byte
*         若返回值 >= UART2_QUEUE_NUM, 则表示没有申请到空闲buffer
*----------------------------------------------------------------------------
* PURPOSE: 在串口2队列中寻找空闲队列项，若找到，返回队列项序号(0 ~ (UART2_QUEUE_NUM-1))
*****************************************************************************/
Byte uart2_get_buffer(void)
{
  Byte i, flag;

  for (i=0; i<UART2_QUEUE_NUM; i++)
  { 
    Disable_interrupt();
		flag = uart2_q[i].flag;
		Enable_interrupt();
		if (flag == 0)
		{ //已找到空闲Buffer
			uart2_q[i].flag = 1;
			break;
		}
  }
  return i;
}


/*F**************************************************************************
* NAME: change_to_LR
*----------------------------------------------------------------------------
* PARAMS:   
* return: 
*----------------------------------------------------------------------------
* PURPOSE: 用于将 ad_sensor_mask, ad_alarm_exts, ad_alarm_base 按左8~1,右8~1
*          的顺序返回。 
*****************************************************************************/
Uint16 change_to_LR(Uint16 val)
{
  Byte   j;
  Uint16 temp16;
	
	temp16 = 0;
	for (j=0; j<7; j++)
	{ //左7 ~ 1
		if (val & ((Uint16)0x0001 << AREA_L_INDEX[j]))
			temp16 |= 0x01 << j;
	}
	temp16 = temp16 << 8;  //左防区位移动到高字节
	for (j=0; j<7; j++)
	{ //右7 ~ 1
		if (val & ((Uint16)0x0001 << AREA_R_INDEX[j]))
			temp16 |= 0x01 << j;
	} 
	
	return temp16;	
}

//2016-12-07新增
void get_alarm_detail_info(void)
{
    Byte i,j;
    Uint16 temp16;
    
    //1、返回配置信息   
    //在UART队列中找空闲Buffer
    i = uart_get_buffer();
    if (i < UART_QUEUE_NUM)
    { 
        //找到了空闲buffer, 写入data
        uart_q[i].tdata[0] = FRAME_STX;
        uart_q[i].tdata[1] = 0x01;     	      //目的地址
        uart_q[i].tdata[2] = gl_comm_addr;	  //源地址																 
        uart_q[i].tdata[3] = 0x1E;
        uart_q[i].tdata[4] = CMD_ZL_PRE;															
        uart_q[i].tdata[5] = 0x1C;
        uart_q[i].tdata[6] = 0x08;
        uart_q[i].tdata[7] = HIGH(ad_sensor_mask_LR);    
        uart_q[i].tdata[8] = LOW(ad_sensor_mask_LR);
        uart_q[i].tdata[9]  = HIGH(ad_still_dn);
        uart_q[i].tdata[10] = LOW(ad_still_dn);
        uart_q[i].tdata[11] = HIGH(ad_still_up);
        uart_q[i].tdata[12] = LOW(ad_still_up);
        uart_q[i].tdata[13]  = 66;   //ad_still_Ddn;
        for (j=0; j<7; j++)
          uart_q[i].tdata[14 + j] = ad_still_Dup[AREA_L_INDEX[j]]; 
        uart_q[i].tdata[21] = 0;
        for (j=0; j<7; j++)
          uart_q[i].tdata[22 + j] = ad_still_Dup[AREA_R_INDEX[j]]; 
        uart_q[i].tdata[29] = 0;																											
        uart_q[i].tdata[30] = system_2or1;
        uart_q[i].tdata[31] = gl_addr_origin;
        uart_q[i].tdata[32] = (Byte)(((Uint32)beep_during_temp * SCHEDULER_TICK) / 1000);	//声光报警输出时间
        uart_q[i].tdata[33] = (Byte)(((Uint32)ld_during_temp * SCHEDULER_TICK) / 1000);	  //联动输出时间              																					
        uart_q[i].len = 35;
    }
          
    //2、读取报警信息
    //在UART队列中找空闲Buffer
    i = uart_get_buffer();
    if (i < UART_QUEUE_NUM)
    { 
        //找到了空闲buffer, 写入data
        uart_q[i].tdata[0] = FRAME_STX;
        uart_q[i].tdata[1] = 0x01;	          //目的地址
        uart_q[i].tdata[2] = gl_comm_addr;	  //源地址																 
        uart_q[i].tdata[3] = 0x0A;
        uart_q[i].tdata[4] = CMD_ZL_PRE;
        uart_q[i].tdata[5] = 0x08;
        uart_q[i].tdata[6] = 0x1A;													
        uart_q[i].tdata[7] = HIGH(ad_sensor_mask_LR);    
        uart_q[i].tdata[8] = LOW(ad_sensor_mask_LR);														
        temp16 = change_to_LR(AlarmDetailInfo.ExternalAlarm);																						
        uart_q[i].tdata[9] = HIGH(temp16);
        uart_q[i].tdata[10] = LOW(temp16);
        temp16 = change_to_LR(AlarmDetailInfo.StaticAlarm);
        uart_q[i].tdata[11] = HIGH(temp16);
        uart_q[i].tdata[12] = LOW(temp16);														
        uart_q[i].tdata[13] = !AlarmDetailInfo.DoorKeepAlarm;                     
        uart_q[i].len = 15;												
    } 
    
    //3、读瞬态张力
    //在UART队列中找空闲Buffer
    i = uart_get_buffer();
    if (i < UART_QUEUE_NUM)
    { //找到了空闲buffer, 写入data
        uart_q[i].tdata[0] = FRAME_STX;
        uart_q[i].tdata[1] = 0x01;	          //目的地址
        uart_q[i].tdata[2] = gl_comm_addr;	  //源地址																 
        uart_q[i].tdata[3] = 0x23;
        uart_q[i].tdata[4] = CMD_ZL_PRE;
        uart_q[i].tdata[5] = 0x21;
        uart_q[i].tdata[6] = 0x1C;																																																				
        for (j=0; j<7; j++)
        { //左7~1
            temp16 = AlarmDetailInfo.InstantSampleValue[AREA_L_INDEX[j]].w;
            uart_q[i].tdata[7+(j<<1)] = HIGH(temp16);
            uart_q[i].tdata[8+(j<<1)] = LOW(temp16);
        }
        uart_q[i].tdata[21] = 0;
        uart_q[i].tdata[22] = 0;														
        for (j=0; j<7; j++)
        { //右7~1
            temp16 = AlarmDetailInfo.InstantSampleValue[AREA_R_INDEX[j]].w;
            uart_q[i].tdata[23+(j<<1)] = HIGH(temp16);
            uart_q[i].tdata[24+(j<<1)] = LOW(temp16);
        }
        uart_q[i].tdata[37] = 0;
        uart_q[i].tdata[38] = 0;																																																						
        uart_q[i].len = 40;
    }
    
    //4、读静态张力基准
    //在UART队列中找空闲Buffer
    i = uart_get_buffer();
    if (i < UART_QUEUE_NUM)
    { //找到了空闲buffer, 写入data
        uart_q[i].tdata[0] = FRAME_STX;
        uart_q[i].tdata[1] = 0x01;	          //目的地址
        uart_q[i].tdata[2] = gl_comm_addr;	  //源地址																 
        uart_q[i].tdata[3] = 0x23;
        uart_q[i].tdata[4] = CMD_ZL_PRE;
        uart_q[i].tdata[5] = 0x21;
        uart_q[i].tdata[6] = 0x1D;																																																				
        for (j=0; j<7; j++)
        { //左7~1
            temp16 = AlarmDetailInfo.StaticBaseValue[AREA_L_INDEX[j]].base;
            uart_q[i].tdata[7+(j<<1)] = HIGH(temp16);
            uart_q[i].tdata[8+(j<<1)] = LOW(temp16);
        }
        uart_q[i].tdata[21] = 0;
        uart_q[i].tdata[22] = 0;														
        for (j=0; j<7; j++)
        { //右7~1
            temp16 = AlarmDetailInfo.StaticBaseValue[AREA_R_INDEX[j]].base;
            uart_q[i].tdata[23+(j<<1)] = HIGH(temp16);
            uart_q[i].tdata[24+(j<<1)] = LOW(temp16);
        }
        uart_q[i].tdata[37] = 0;
        uart_q[i].tdata[38] = 0;																																																						
        uart_q[i].len = 40;														
    }  
}