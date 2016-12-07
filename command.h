/*H**************************************************************************
* NAME:  command.h         
*----------------------------------------------------------------------------
* Copyright (c) 2012.
*----------------------------------------------------------------------------
* RELEASE:      2012.06.01
* REVISION:     1.0    
*----------------------------------------------------------------------------
* PURPOSE:
*   This file contains the system command definition.
*****************************************************************************/

#ifndef _COMMAND_H_
#define _COMMAND_H_


/* ϵͳ����� */
#define CMD_ADDR_DEF       1    //ȱʡ������ַ
#define CMD_ADDR_BC      255    //�㲥��ַ
#define CMD_ADDR_UNSOLV  254    //δ��¼��δ��ȷ���õĵ�ַ

//˫������ַģ��
#define CMD_DADDR_qSTAT	 0xE0    //ѯ�ʷ���״̬
#define CMD_DADDR_qPARA  0xE1    //ѯ�ʲ���
#define CMD_DADDR_sPARA  0xE2    //���ò���

#define CMD_ACK_OK       0x00    
#define CMD_DADDR_aSTAT  0xF0    //Ӧ�� - ��������
#define CMD_DADDR_aPARA  0xF1    //Ӧ�� - ����ѯ��/����

//����/����MODģ��
#define CMD_ZL_PRE       0xE8    //����/����ר�������־


#endif    /* _COMMAND_H_ */
