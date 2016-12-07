/*C**************************************************************************
* NAME:    main.c
*----------------------------------------------------------------------------
* Copyright (c) 2013.
*----------------------------------------------------------------------------
* RELEASE:      2013.09.04
* REVISION:     9.4     
*----------------------------------------------------------------------------
* PURPOSE:
* This is the main program.
*****************************************************************************/

/*_____ I N C L U D E S ____________________________________________________*/
#include <INTRINS.H>
#include "config.h"                      /* system macro definition */
#include "lib_mcu\c51_drv.h"             /* c51 driver definition */
#include "lib_mcu\wdt\wdt_drv.h"         /* WDT driver definition */
#include "lib_mcu\eeprom\eeprom_drv.h"   /* eeprom drive definition */
#include "scheduler.h"                   /* scheduler definition */
#include "tasks\comm\comm_task.h"        /* UART task definition */
#include "command.h"                     /* system command definition */
          

/*_____ D E C L A R A T I O N ______________________________________________*/
extern xdata  Byte    gl_reply_tick;     /* �豸������ʱ*/

/* variables for beep */
extern xdata  Uint16  beep_during_temp;  //Ԥ�������������ʱ��, ��λtick

/* variables for ���� */
extern xdata  Uint16  ld_during_temp;    //Ԥ���һ������������ʱ��, ��λtick  

/* AD sample */
extern idata  Uint16  ad_sensor_mask;     //sensor mask
extern xdata  Uint16  ad_sensor_mask_LR;  //������˳���������sensor mask: ��8 ~ 1  �� 8 ~ 1
extern bdata  bit     ad_sensor_extent;   //�Ƿ����չ��: 1 - ��; 0 - ����

extern xdata  Uint16  ad_still_dn;       //��̬����ֵ����
extern xdata  Uint16  ad_still_up;       //��̬����ֵ����
extern xdata  Byte    ad_still_Dup[14];  //������ֵ���޲�

/* System */
extern idata  Byte   gl_comm_addr;     //��ģ��485ͨ�ŵ�ַ�������Ѹ�Ϊ�淶ֵ��		
extern xdata  Byte   gl_addr_origin;   //��ַ���뿪��ԭֵ����ȡ����

extern bdata  bit    system_2or1;      //˫/��������־: 0 - ˫; 1 - ��												 

extern const  Byte   AREA_L_INDEX[7];  //�����������Ӧ�İ���(����) index
extern const  Byte   AREA_R_INDEX[7];  //�ҷ���������Ӧ�İ���(����) index

/* ����������ƫ�� */
extern xdata  Uint16   sensor_sample_offset[14];    //����������ƫ�û������ʱ������������ֵ��Ϊ0����Լ310���ң���Ҫ������˲������ = ����ֵ - ����ƫ��                 

static  void  get_sys_info(void);                                                                                                                                                   
static  void  main(void);
     

/*F**************************************************************************
* NAME: main
*----------------------------------------------------------------------------
* PARAMS:
* return:
*----------------------------------------------------------------------------
* PURPOSE: Main user routine 
*----------------------------------------------------------------------------
* REQUIREMENTS:
*****************************************************************************/
void main(void)
{    
  Uint32 i;
 
  //set pins values
  P0 = 0xFF;       // 1111 1111b
  P0M1 = 0x00;	   // 0000 0000b 
  P0M0 = 0xC0; 	   // 1100 0000b
	
  P1    = 0xFF;    // 1111 1111b	  
  P1M1  = 0x01;    // 0000 0001b    
  P1M0  = 0x00;	   // 0000 0000b
  P1ASF = 0x01;    // P1.0 ΪADģ������   
  
  P2 = 0xFF;       // 1111 1111b
  P2M1 = 0x00;	   // ׼˫���
  P2M0 = 0x00;			  

  P3 = 0x33;       // 0011 0011b: Beep��ֹ, ����1/2�ӵ磬�������ӵ磬RS485���ͽ�ֹ�����գ�
  P3M1 = 0x00;     // 0000 0000b
  P3M0 = 0xFC;	   // 1111 1100b
  
  P4 = 0xFF;       // 1111 1111b
  P4M1 = 0x00;     // 0000 0000b
  P4M0 = 0x00;	   // 0000 0000b   
  P4SW = 0x40;     // 0100 0000b

  P5 = 0xF0;       // 1111 0000b
	P5M1 = 0x00;     // 0000 0000b
	P5M0 = 0x0F;     // 0000 1111b

  AUXR = S1BRS_; //��ʱ��0,1Ϊ8051���ݷ�ʽ(12��Ƶ)
                 //BRTΪ12��Ƶģʽ, UART1ʹ��BRT�������ʷ�����                      
  AUXR1 = 0x10;  //0001 0000b

  //system power-on hint
  //��һ��14��LED -- ȫ�����������ڼ������LED�Ƿ����
  P2 = 0x00;   //����8��LED 
  P0 = 0xC0;   //����6��LED (1100 0000b)
  i = 150000;
  while(i>0)  i--;
  P2 = 0xFF;	 //��8��LED 
  P0 = 0xFF;	 //��6��LED 
  //��ʱ
  i = 150000;
  while(i>0)  i--;

  //��Flash�ж�ȡϵͳ����
  get_sys_info();

  //��ģ�������ʼ��
  sch_scheduler_init();

  //ʹ��ȫ���ж�
  Enable_interrupt();     //global interrupt enable

  //����WDT
  #ifdef  Enable_WDT
    Wdt_enable();         //2s �������      
  #endif      

  sch_scheduler();        //endless scheduler execution
}        


/*F**************************************************************************
* NAME: get_sys_info
*----------------------------------------------------------------------------
* PARAMS:
* return:
*----------------------------------------------------------------------------
* PURPOSE: ��ȡϵͳԤ������
*----------------------------------------------------------------------------
* REQUIREMENTS:
*****************************************************************************/
void get_sys_info(void)
{
    Uint16 i;
    Byte   j;
    Byte   temp;

    //0. ʹ��Flash����
    flash_enable();

    //1. ˫/��������־
    i = 0;
    system_2or1 = 0;  //ȱʡ: ˫����
    do 
    {
    //��ʱ
    j = 255;
    while(j > 0)  j--;	  	 
    if (system_2or1 == !bSel_2or1)
    { //������ͬ��ֵ
      i ++;
    }
    else
    { //������ͬ��ֵ
      i = 0;
      system_2or1 = !bSel_2or1;
    }
    } while (i < 8);

    //2. ��RS485ͨ�ŵ�ַ - �����뿪��ֵ
    i = 0;
    temp = 0xFF & b485_ADDR_MASK;
    do 
    {
        //��ʱ
        j = 255;
        while (j>0)  j--;	  	 
        if (temp == (CommAddr_Port & b485_ADDR_MASK))
        { //������ͬ��ֵ
            i++;
        }
        else
        { //������ͬ��ֵ
            i = 0;
            temp = CommAddr_Port & b485_ADDR_MASK;
        }
    }while (i < 8);	
    gl_addr_origin = ~temp >> 1;      //��ַ���뿪��ԭֵ����ȡ����
    gl_comm_addr = gl_addr_origin;    //��ģ��485ͨ�ŵ�ַ�������Ѹ�Ϊ�淶ֵ��		
    //�豸��ַ�Ϸ��Լ��(�����豸��ַҪ���0x10��ʼ,��<= 0xCF)
    //ʹ�� 7 λ���뿪�أ���ַ��ΧΪ 0x00 ~ 0x7F
    if ((gl_comm_addr < 0x10) || (gl_comm_addr > 0x7F))
    { //�豸��ַ����Ϊ 0x10 ~ 0x7F ֮��(��)
    gl_comm_addr = CMD_ADDR_UNSOLV;
    }	

    //3. ������������mask
    //3.a. �����뿪������
    bP2_Gate = 0;	  //ZZX����ʱmaskλ������������Ӧ��LED
    bP0_Gate = 0;

    //3.b ������ֵ
    i = 0;
    ad_sensor_mask = 0x3FFF;
    do 
    {
      //��ʱ
      j = 255;
      while (j > 0)  j--;	  	 
      if (ad_sensor_mask == ((((Uint16)(P0 & 0x3F)) << 8) + P2))
      {	//������ͬ��ֵ
        i ++;
      }
      else
      {	//������ͬ��ֵ
          i = 0;
          ad_sensor_mask = ((((Uint16)(P0 & 0x3F)) << 8) + P2);
      }
    } while (i < 15000);
    ad_sensor_mask = ~ad_sensor_mask & 0x3FFF;
    //���� ad_sensor_mask_LR
    ad_sensor_mask_LR = change_to_LR(ad_sensor_mask); 	
    //�ж��Ƿ����չ��
    if ((ad_sensor_mask & 0x3C00) == 0x0)
      ad_sensor_extent = 0;   //�Ƿ����չ��: 0 - ���� 
    else
        ad_sensor_extent = 1;   //�Ƿ����չ��: 1 - ��

    //3.c. ��ֹ���뿪�����룬������LED
    bP2_Gate = 1;		//ZZX: ��ֹ���뿪�����룬������mask��
    bP0_Gate = 1;
    
    //4. ����̬����ֵ��Χ
    temp = flash_read(EEPROM_SECTOR3);
    if (temp == 0x5A) 
    { //����Ч����
    //����
    temp = flash_read(EEPROM_SECTOR3 + 1);
      ad_still_dn = (Uint16)temp << 8;
    temp = flash_read(EEPROM_SECTOR3 + 2);    
      ad_still_dn += temp;
      //����
    temp = flash_read(EEPROM_SECTOR3 + 3);
      ad_still_up = (Uint16)temp << 8;
    temp = flash_read(EEPROM_SECTOR3 + 4);    
      ad_still_up += temp;
    //��Ч��? 		
      if ((ad_still_dn < STD_STILL_DN) || 
        (ad_still_up > STD_STILL_UP) || 
        (ad_still_dn >= ad_still_up))
      { //�޺Ϸ����ݣ�ȡȱʡֵ
        ad_still_dn = STD_STILL_DN;
      ad_still_up = STD_STILL_UP;
      }	    
    }
    else
    {	//����Ч���ã�ȡȱʡֵ
    ad_still_dn = STD_STILL_DN;
    ad_still_up = STD_STILL_UP;
    }
    
    //5.��������ֵ���� 
    temp = flash_read(EEPROM_SECTOR4);
    if (temp == 0x5A) 
    { //����Ч����
        //���������
      for (j=0; j<7; j++)
      { 
            ad_still_Dup[AREA_L_INDEX[j]] = flash_read(EEPROM_SECTOR4 + 2 + j);
        }
        //�ҷ�������
      for (j=0; j<7; j++)
      { 
            ad_still_Dup[AREA_R_INDEX[j]] = flash_read(EEPROM_SECTOR4 + 10 + j);
        }    		
        //�Ƿ���Ч�� 	
    for (j=0; j<14; j++)
      { 
        if ((ad_still_Dup[j] < STD_ALARM_MIN) || (ad_still_Dup[j] > STD_ALARM_MAX))
        ad_still_Dup[j] = STD_ALARM_DEF;    //�޺Ϸ����ݣ�ȡȱʡֵ
        }				
    }
    else
    {	//����Ч���ã�ȡȱʡֵ
      for (j=0; j<14; j++)
      ad_still_Dup[j] = STD_ALARM_DEF;
    }  

    //6. �����ⱨ��ʱ������  
    temp = flash_read(EEPROM_SECTOR5);
    if (temp == 0x5A) 
    { //����Ч����
      temp = flash_read(EEPROM_SECTOR5 + 1);
      beep_during_temp = (Uint16)(((Uint32)temp * 1000) / SCHEDULER_TICK);	
    }
    else
    {	//ȡȱʡֵ
    beep_during_temp = 0;   //��λ�� tick
    }

    //7. ���������ʱ������  
    temp = flash_read(EEPROM_SECTOR6);
    if (temp == 0x5A) 
    { //����Ч����
      temp = flash_read(EEPROM_SECTOR6 + 1);
      ld_during_temp = (Uint16)(((Uint32)temp * 1000) / SCHEDULER_TICK);	
    }
    else
    {	//ȡȱʡֵ: 5��
    ld_during_temp = (Uint16)(((Uint32)5 * 1000) / SCHEDULER_TICK);   //��λ�� tick
    }

    //8. ������������ƫ��
    temp = flash_read(EEPROM_SECTOR7);
    if (temp == 0x5A) { //����Ч����
        for (j = 0; j < 14; j++) {
            temp = flash_read(EEPROM_SECTOR7 + 1 + (j << 1));
            sensor_sample_offset[j] = ((Uint16)temp << 8);
            temp = flash_read(EEPROM_SECTOR7 + 2 + (j << 1));
            sensor_sample_offset[j] += temp;
        }

        //������
        sensor_sample_offset[9] = 0;
    } else {	//����Ч����
        for (j = 0; j < 14; j++) {
            sensor_sample_offset[j] = 0;
        }
    }
    
    //9. ��ȡ��ʱʱ��
    temp = flash_read(EEPROM_SECTOR8);
    if (temp == 0x5A) { //����Ч����
        gl_reply_tick = flash_read(EEPROM_SECTOR8 + 1);
    } else {	//����Ч����
        gl_reply_tick = 0;
    }
    
    //10. ��ֹFlash����
    flash_disable();                                            
}
