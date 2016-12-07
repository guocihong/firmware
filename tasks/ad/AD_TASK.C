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
#define AD_EQU_PNUM  4   //����ȥ��������
                         
 
/*_____ D E C L A R A T I O N ______________________________________________*/
extern  data  Uint16   gl_delay_tick;   /* ͨ����ʱ��tick */                     

/* UART Queue */
extern  data  Byte     uart_q_index;               // ���ڷ���ĳ���������ţ���Ϊ0xFF, ��ʾû���κ�����뷢������
extern xdata  sUART_Q  uart_q[UART_QUEUE_NUM];	   // ���ڶ��� 

/* variables for alarm output */
extern bdata  bit  climb_alarm_flag; //����������־��0-�ޱ�����1-����
extern bdata  bit  adl_alarm_flag;  //���������ϱ�����־: 0 - �ޱ���; 1 - ����ֵ������
extern bdata  bit  adr_alarm_flag;  //�Ҳ�������ϱ�����־: 0 - �ޱ���; 1 - ����ֵ������
																		//  ����ԭ�����Ϊ������������ ��̬��������Χ����
																		//  ZZX: �Ѿ��� mask ����, δ���Ŵ�״̬
extern  data  Uint16 alarm_led_flag;	//LED����ָʾ: 0 - �ޱ������𣩣�1 - ����������
								                      //ZZX: û�о���mask��������������������;�̬��������Χ����

/* AD sample */
extern xdata  Uint16  ad_still_dn;         //��̬����ֵ����
extern xdata  Uint16  ad_still_up;         //��̬����ֵ����
extern xdata  Byte    ad_still_Dup[14];    //������ֵ����

extern idata  Uint16  ad_still_dn_s;       //��̬����ֵ����(��λ��10bit����ֵ)
extern idata  Uint16  ad_still_up_s;       //��̬����ֵ����(��λ��10bit����ֵ)
extern idata  Byte    ad_still_Dup_s[14];  //������ֵ����(��λ��10bit����ֵ)

extern idata  Byte        ad_index;        //���ڲ�����ͨ����, ȡֵ��Χ0~13
extern  data  sAD_Sample  ad_sample;       //��ǰ����ֵ
extern idata  Uint16   ad_sensor_mask;     //�Ѱ�װ����Ҫ�ж���sensor mask: 0 - ��ֹ; 1 - ����

extern idata  Uint16    ad_samp_pnum;      //��������(���㾲̬��׼ֵʱ�ܲ�������)
extern idata  sAD_Sum   ad_samp_equ[14];   //����ȥ�������
extern idata  sAD_Sum   ad_samp_sum[14];   //�׶����
extern xdata  Union16   ad_chn_sample[14]; //����һ�ֲ���ֵ���Ѿ���ȥ������ÿͨ��һ���㣬ѭ�����棩

extern xdata  sAD_BASE  ad_chn_base[14];   //��ͨ����̬��׼ֵ/�����޷�ֵ
extern  data  Byte      ad_chn_over[14];   //��ͨ������������(�����)�ķ�ֵ�ж��� 0 - ��Χ�ڣ� 1 - ����ֵ
                                           //ÿͨ��һ���ֽڣ� CH0~13 ��Ӧ ad_chn_over[0~13]									
extern idata  Uint16      ad_alarm_exts;   //����������־����mask���� λֵ 0 - �ޣ� 1 - ����ֵ
extern idata  Uint16      ad_alarm_base;	 //��̬����������־����mask���� λֵ 0 - ������Χ�ڣ� 1 - ��������Χ

/* for system */
extern idata  Byte   gl_comm_addr;     //��ģ��485ͨ�ŵ�ַ�������Ѹ�Ϊ�淶ֵ��																 
extern xdata  Byte   gl_addr_origin;   //��ַ���뿪��ԭֵ����ȡ����
extern idata  Byte   system_status;    //ϵͳ״̬							   
extern bdata  bit    uart_send_samp;   //��׼ֵ/����ֵ��UART�ͳ���־: 0 - ������; 1 - ����
									                     //  �� ad_task_init()	�г�ʼ��      
extern const  Byte   AREA_L_INDEX[7];  //�����������Ӧ�İ���(����) index
extern const  Byte   AREA_R_INDEX[7];  //�ҷ���������Ӧ�İ���(����) index

/* for this task: ���ڻ�׼ֵ���� */
static idata  Byte   md_point[14];     //���ڻ�׼ֵ���ٵļ�������



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
  
  //��ʼ��ADCӲ��
  adc_init();
  ADC_CONTR |= ADC_POWER_;  //ʹ��ADC��Դ

  //��ر�����ʼ��
  ad_sample.valid = 0;  //���У�����д����ֵ
  ad_samp_pnum = 0;     //��������(���㾲̬��׼ֵʱ�ܲ�������)
  for (i=0; i<14; i++)
  { 
    ad_samp_equ[i].sum = 0;	      //����ȥ�������
    ad_samp_equ[i].point = 0;
    ad_samp_sum[i].sum = 0;	      //�׶����
    ad_samp_sum[i].point = 0;
	  ad_chn_sample[i].w = 0;	      //����һ�ֲ���ֵ
    ad_chn_base[i].base = 0;	    //��ͨ����̬��׼ֵ/�����޷�ֵ
    ad_chn_base[i].base_down = 0;
    ad_chn_base[i].base_up = 0;
	  ad_chn_over[i] = 0x00;	      //��ͨ������������(�����)�ķ�ֵ�ж������ڷ�Χ��
    md_point[i] = 0;    //���ڻ�׼ֵ���ٵļ�������
  }    
  ad_alarm_exts = 0;	  //����������־����mask��: ��
  ad_alarm_base = 0;    //��̬����������־����mask����������Χ��   
  alarm_led_flag = 0;	  //���б���LEDΪ��
  //P2 = 0xFF;
	//P0 = 0xFF;
                 
  //����ͨ��0#����
  ad_index = 0;    
	P5 = ad_index;
  ADC_CONTR = 0x80;  // 1000 0000b
                     // 0xC0, ��CPUƵ��Ϊ20MHz, ��ת������Լ50KHz
                     // 0x80, ��CPUƵ��Ϊ20MHz, ��ת������Լ25KHz 
  ADC_CONTR |= ADC_START_;		  // ����ת��

/*
  //��̬����ֵ��/����, ������ֵ��/����ţ������Ӧ�Ĳ���ֵ
  ad_still_dn_s  = (Uint16)(((Uint32)ad_still_dn  * SENSOR_RATIO_SAMP) >> SENSOR_RATIO_FS);   //��̬����ֵ����
  ad_still_up_s  = (Uint16)(((Uint32)ad_still_up  * SENSOR_RATIO_SAMP) >> SENSOR_RATIO_FS);	  //��̬����ֵ����
  for (i=0; i<8; i++)
    ad_still_Dup_s[i] = (Uint16)(((Uint32)ad_still_Dup[i] * SENSOR_RATIO_SAMP) >> SENSOR_RATIO_FS);	  //������ֵ���޲�
*/

  ad_still_dn_s  = ad_still_dn;   //��̬����ֵ����
  ad_still_up_s  = ad_still_up;	  //��̬����ֵ����
  for (i=0; i<14; i++)
    ad_still_Dup_s[i] = ad_still_Dup[i];	//������ֵ���޲�

  //�Ƿ�ʵʱ���Ͳ���ֵ
  uart_send_samp = 0x00;   //ȱʡ��������
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
  Byte    i, j;      //ѭ������
  Uint16  temp16;    //��ʱ����
  Uint8   index;     //����ͨ����
  Uint16  val_temp;  //�������10bit����ֵ,  ������ʱ����
  Uint16  val;       //4������õ���ƽ������ֵ, ��Ϊһ���ɽ��г����жϵ���С��

  if (ad_sample.valid)
  {  //���²������ݵ���
   //0.���浽��ʱ����
   val_temp = ad_sample.val;
   index    = ad_sample.index;     

   //1. ���浽����ȥ���������
   ad_samp_equ[index].sum += val_temp;
   ad_samp_equ[index].point ++;

   //2. ��ǰͨ���Ƿ���ȥ��������
   if (ad_samp_equ[index].point == AD_EQU_PNUM)
   { //����ȥ���������������������һ����
     //2.a �����Ӧͨ����һ��������
     val = ad_samp_equ[index].sum >> 2;  //����4

     //2.b ���㵱ǰͨ����ȥ������ͽṹ
     ad_samp_equ[index].sum = 0;
     ad_samp_equ[index].point = 0;
        
     //2.c ����ʵʱ����ֵ�������Ƿ���Ҫ����
     ad_chn_sample[index].w = val;   //���浽����һ�ֲ���ֵ������
     if (uart_send_samp && (index == 13))
     { //��Ҫ����, ���Ѿ����14��ͨ����һ�������������ɷ��Ͳ���ֵ
			 i = uart_get_buffer();
			 if (i < UART_QUEUE_NUM)
			 { //�ҵ��˿���buffer, д��data
				 uart_q[i].tdata[0] = FRAME_STX;
				 uart_q[i].tdata[1] = 0xFF;
				 uart_q[i].tdata[2] = 0x00;
				 //��ch1 ~ ch7				 				 		
				 for (j=0; j<7; j++)
				 {
					 uart_q[i].tdata[3 + (j<<1)] = ad_chn_sample[AREA_L_INDEX[j]].b[0];
					 uart_q[i].tdata[4 + (j<<1)] = ad_chn_sample[AREA_L_INDEX[j]].b[1];  
				 }
				 uart_q[i].tdata[17] = 0;
				 uart_q[i].tdata[18] = 0;
				 //��ch1 ~ ch7
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
			 { //�޿���buffer, ����������
				 //���: ���ж��������ڷ���, �ȴ������
				 while (uart_q_index != 0xFF);	//������,������ WDT ��λ				           
			 }                            
     }//end if (uart_send_samp && (index ==3))

     //2.d ��ϵͳ״̬�������ݵĴ���
     switch (system_status)
     {
       //case SYS_PowerON:   //�ϵ�
       //case SYS_B5S:       //5����ʱ
       //       break;

       case SYS_SAMP_BASE: //��ʼ�ϵ�ʱ�ľ�̬��׼ֵ����
              //����׶κ�
              ad_samp_sum[index].sum += val;
              ad_samp_pnum ++;
              if (ad_samp_pnum == 448)   
              { //�Ѿ�����׼ֵ����������ÿͨ��32�㣬�����, ��ʱԼ10��) 
                //1.�����ֵ��������
                for (i=0; i<14; i++)
                { 
                  //��׼
                  ad_chn_base[i].base = ad_samp_sum[i].sum >> 5;   //����32 
                  //���� = ��׼ * ��1 / 3��
                  val_temp = ad_chn_base[i].base;
                  ad_chn_base[i].base_down = (val_temp >> 1) - (val_temp >> 3) - (val_temp >> 4);   //���� = 1/2 - 1/8 - 1/16 = 0.3125     
                  //����
                  if ((1023 - ad_chn_base[i].base) > ad_still_Dup_s[i])
                    ad_chn_base[i].base_up = ad_chn_base[i].base + ad_still_Dup_s[i];
                  else
                    ad_chn_base[i].base_up = 1023;
                  //��龲̬�����Ƿ���������Χ��	  
                  check_still_stress(i);
                  //��λ�׶κͱ�����׼����������Ӧ��ֵ����
                  ad_samp_sum[i].sum = 0; 
                  ad_samp_sum[i].point = 0;
                }           
                //2. ����
                //Disable_interrupt(); 
                //jl_sch_tick = JL_ACT_PRID;
                //Enable_interrupt();                                  
                //3. ״̬-> ʵʱ���				  
                system_status = SYS_CHECK;
              }
              break;

      case SYS_CHECK: //ʵʱ���
              //a. �е�ǰֵ
              ad_chn_over[index] = ad_chn_over[index] << 1;   //Bit0��0�����ȱʡ��������Χ��
              if ((val >= ad_chn_base[index].base_down) && (val <= ad_chn_base[index].base_up))
              { //��������/����������Χ��
                //a. ���־(ȱʡ)
                //b. ������ٻ�׼ֵ�����
                ad_samp_sum[index].sum += val;
                ad_samp_sum[index].point ++;
                if (ad_samp_sum[index].point == 2)
                { //��2��(Լ��0.6��)
                  //b.0 ������2���ֵ 
                  val_temp = ad_samp_sum[index].sum >> 1;   //����2, �õ���2��ľ�ֵ
                  //b.1 ���»�׼ֵ
                  if (ad_chn_base[index].base > (val_temp + 1))
                  { //����С2, �ڻ����ɳ�
                    //ZZX: ��������, ���ٲ�ֵ�� 1/2
                    val_temp = (ad_chn_base[index].base - val_temp) >> 1;
                    if (ad_chn_base[index].base >= val_temp)
                    {	  
                      ad_chn_base[index].base -= val_temp;
                      //ͬ������������
                      val_temp = ad_chn_base[index].base;
                      ad_chn_base[index].base_down = (val_temp >> 1) - (val_temp >> 3) - (val_temp >> 4);   // = 1/2 - 1/8 - 1/16  
                      if ((1023 - ad_chn_base[index].base) > ad_still_Dup_s[index])
                        ad_chn_base[index].base_up = ad_chn_base[index].base + ad_still_Dup_s[index];
                      else
                        ad_chn_base[index].base_up = 1023; 
                      //��龲̬�����Ƿ���������Χ��	  
                      check_still_stress(index);                          
                    }
                    //�建���Ž����ٱ���
                    md_point[index] = 0;
                  }
                  else if (val_temp > (ad_chn_base[index].base + 1))
                  { //���ٴ�2, �����Ž�					
                    md_point[index] ++;					  
                    if (md_point[index] >= DEF_ModiBASE_PT)
                    { //���������Ž�ʱ��������������, ����һ�θ���
                      //1. ���ٻ�׼ֵ
                      if (ad_chn_base[index].base < 1023)
                      { //���Ե���1
                        ad_chn_base[index].base ++;
                        //ͬ������������
                        val_temp = ad_chn_base[index].base;
                        ad_chn_base[index].base_down = (val_temp >> 1) - (val_temp >> 3) - (val_temp >> 4);   // = 1/2 - 1/8 - 1/16
                        if (ad_chn_base[index].base_up < 1023)
                          ad_chn_base[index].base_up ++;  
                        //��龲̬�����Ƿ���������Χ��	  
                        check_still_stress(index);							                          
                      }
                      //2. �建���Ž����ٱ���
                      md_point[index] = 0;
                    }
                  }
                  //else
                  //{ //��ǰֵ�ڻ�׼ֵ�� +/- 2 ֮��					
                  //  //���������Ž����ٱ���ֵ
                  //}                 
                  //b.2 ��λ�׶κͱ��� - ����4��4��ƽ������ͽṹ
                  ad_samp_sum[index].sum = 0;
                  ad_samp_sum[index].point = 0;
                }
              }
              else
              { //������ֵ, �ñ�־
                ad_chn_over[index] |= 0x01; 
                //ZZX: ��Ҫ�����ڸ��ٵ���ͽṹ��?         
              }
        
              //c. ���㱨����־  
              if ((ad_chn_over[index] & 0x0F) == 0x0F)
              { //����4�㳬��Χ����ͨ���б���
                //����4�㣬��Ӧ����ʱ��Ϊ280ms X 4 = 1.1��
                if (index == 9) {
                    climb_alarm_flag = 1;
                }
                ad_alarm_exts |= (Uint16)0x01 << index;
                
                //20110817: �������»�׼ֵ
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
                //��龲̬�����Ƿ���������Χ��	  
                check_still_stress(index); 					
                //��λ�׶κͱ��� - ����4��4��ƽ������ͽṹ
                ad_samp_sum[index].sum = 0;
                ad_samp_sum[index].point = 0;               
                //�建���Ž����ٱ���
                md_point[index] = 0;   //���ڻ�׼ֵ���ٵļ�������
                //}
              }
              else if ((ad_chn_over[index] & 0x0F) == 0x00)
              { //�ޱ���
                if (index == 9) {
                    climb_alarm_flag = 0;
                }
                ad_alarm_exts &= ~((Uint16)0x01 << index);
              }  
      
              //LEDָʾ(��mask)
              temp16 = (ad_alarm_exts | ad_alarm_base) & 0x3FFF; 
              if (alarm_led_flag != temp16)
              { //LEDָʾ��Ϣ�б仯
                alarm_led_flag = temp16;
								P2 = ~ LOW(alarm_led_flag);
								P0 = ~ HIGH(alarm_led_flag);							
              }                
              
              //������ϱ�����־(mask��)
              temp16 = ((ad_alarm_exts & 0xFDFF) | ad_alarm_base) & 0x3FFF; 
              temp16 &= ad_sensor_mask;			
              //�������ϱ���
              if ((temp16 & 0x301F) == 0)				
                adl_alarm_flag = 0;   //�ޱ���				
              else
                adl_alarm_flag = 1;  	//�б���								
              //���Ҳ���ϱ���
              if ((temp16 & 0x0FE0) == 0)				
                adr_alarm_flag = 0;   //�ޱ���				
              else
                adr_alarm_flag = 1;	  //�б���							  				  								      	  					                                               
              break;
      }//end switch
    }//end ��ȥ��������

    //3.��ǰ����ֵ������ϣ������µĲ���ֵ����
    ad_sample.valid = FALSE;
  }//end if (ad_sample.valid)
}//end FUNC()


/*F**************************************************************************
* NAME: check_still_stress
*----------------------------------------------------------------------------
* PARAMS: 
*         index : ͨ����
* return:
*----------------------------------------------------------------------------
* PURPOSE: ���ָ��ͨ���ľ�̬�����Ƿ���������Χ��
*****************************************************************************/
void check_still_stress(index)
{
  if ((ad_chn_base[index].base >= ad_still_dn_s) && (ad_chn_base[index].base <= ad_still_up_s))
  { //��������Χ��, ���־
	  ad_alarm_base &= ~((Uint16)0x01 << index);
  }
  else
  { //����������Χ���ñ�־
	  ad_alarm_base |= (Uint16)0x01 << index;
  }
  
  //���Ƹ˲����ھ�̬������ֻ������������
  ad_alarm_base &= ~(1 << 9);
}