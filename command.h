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


/* 系统命令定义 */
#define CMD_ADDR_DEF       1    //缺省主机地址
#define CMD_ADDR_BC      255    //广播地址
#define CMD_ADDR_UNSOLV  254    //未烧录或未正确设置的地址

//双防区地址模块
#define CMD_DADDR_qSTAT	 0xE0    //询问防区状态
#define CMD_DADDR_qPARA  0xE1    //询问参数
#define CMD_DADDR_sPARA  0xE2    //设置参数

#define CMD_ACK_OK       0x00    
#define CMD_DADDR_aSTAT  0xF0    //应答 - 防区报警
#define CMD_DADDR_aPARA  0xF1    //应答 - 参数询问/设置

//张力/脉冲MOD模块
#define CMD_ZL_PRE       0xE8    //张力/脉冲专用命令标志


#endif    /* _COMMAND_H_ */
