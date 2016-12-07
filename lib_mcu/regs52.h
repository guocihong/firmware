/*-H-----------------------------------------------------------------------
Header file for STC12C5AXX.
Copyright (c) 2011.
All rights reserved.
Checked by ZZX at 2011.01.13
--------------------------------------------------------------------------*/

#ifndef __STC12C5AXX_H__
#define __STC12C5AXX_H__


//--------------------------------------------------------------------------
//新一代 1T 8051系列 单片机内核特殊功能寄存器 C51 Core SFRs
//                                          7     6      5       4     3    2    1     0   Reset Value
sfr ACC  = 0xE0; //Accumulator                                                              0000,0000
sfr B    = 0xF0; //B Register                                                               0000,0000
sfr PSW  = 0xD0; //Program Status Word      CY    AC     F0    RS1    RS0  OV    F1    P    0000,0000
//-----------------------------------
sbit CY  = PSW^7; //进位标志位
sbit AC  = PSW^6; //进位辅助位
sbit F0  = PSW^5; //用户标志位0
sbit RS1 = PSW^4; //RS1/0 组合用于工作寄存器组选择位
sbit RS0 = PSW^3;
sbit OV  = PSW^2; //溢出标志位
sbit F1  = PSW^1; //用户标志位1
sbit P   = PSW^0; //奇偶标志位

//                                          7     6      5       4     3    2    1     0   Reset Value
sfr SP   = 0x81;  //Stack Pointer                                                           0000,0111
sfr DPL  = 0x82;  //Data Pointer Low Byte                                                   0000,0000
sfr DPH  = 0x83;  //Data Pointer High Byte   												0000,0000
sfr16 DPTR = 0x82;
//ZZX: 8051单片机的堆栈为满增
                                                
//--------------------------------------------------------------------------------
//新一代 1T 8051系列 单片机系统管理特殊功能寄存器
//                                          7     6      5    4     3      2    1     0     Reset Value
sfr PCON   = 0x87; //Power Control        SMOD  SMOD0  LVDF  POF   GF1    GF0   PD   IDL    0011,0000
#define IDL_    0x01    /* Idle Mode Bit: 1=Active */
                        // 可由外部中断，定时器中断，低压检测中断，AD中断等任何一个中断唤醒。
						// 可用于唤醒的外部管脚有：
						//     INT0#, INT1#, T0/P3.4, T1/P3.5, RxD
						//     Timer0, Timer1 也可
						//     UART 中断也可
#define STOP_   0x02    /* Stop Mode Bit: 1=Active */
#define PD_     0x02    /* Alternate definition */
                        // 进入掉电模式后，只有外部中断工作
						// 可用于唤醒的外部管脚有：INT0#, INT1#, T0/P3.4, T1/P3.5, RxD, CCP0, CCP1, EX_LVD
#define GF0_    0x04    /* General Purpose Flag 0 */
#define GF1_    0x08    /* General Purpose Flag 1 */
#define POF_    0x10    /* PowerOn flag: 上电复位标志位 */
                        // 0 - 其它复位(热启动)，包括外部手动，WDT，软件复位等
						// 1 - 停电后上电复位（冷启动），可软件清0
#define LVDF_   0x20    /* LVD flag: 低压检测标志位，也作低压检测中断请求位, 须软件清除 */
                        // 在正常工作和空闲工作状态时，若内部VCC低于低压检测阀值，则自动置为1，须用软件清0
						// 在进入掉电工作状态前，若低压检测电路未被允许产生中断，则进入掉电工作状态后，低压检测电路不工作.
						//                       若低压检测电路被允许产生中断，则进入掉电工作状态后，低压检测电路继续工作，
						//                           并可产生低压检测中断，并唤醒CPU
#define SMOD0_  0x40    // 帧错误检测控制：当SMOD0=1时，SM0/FE用于FE功能
                        //                 当SMOD0=0时，SM0/FE用于SM0功能，与SM1一起指定UART的工作方式
#define SMOD_   0x80    // 波特率加倍控制位(适用于定时器1或BRT)

//                                         7     6       5      4     3      2      1      0    Reset Value
sfr AUXR  = 0x8E;  //Auxiliary Register  T0x12 T1x12 UART_M0x6 BRTR S2SMOD BRTx12 EXTRAM S1BRS  0000,0000
//-----------------------------------
#define T0x12_      0x80    //0 - 12分频(兼容传统8051)； 1 - 不分频，即为Fosc
#define T1x12_      0x40    //0 - 12分频(兼容传统8051)； 1 - 不分频，即为Fosc
                            //    若T1用作UART的波特率，则T1x12_也决定了UART是 12T 还是 1T
#define UART_M0x6_  0x20	  //0 - UART的模式0是传统12T的8051速度， 即12分频
                            //1 - UART的模式0是传统12T的8051速度的6倍， 即2分频 
#define BRTR_       0x10    //0 - 禁止 BRT 运行
                            //1 - 允许 BRT 运行
#define S2SMOD_     0x08    //0 - UART2 的波特率不加倍
							//1 - UART2 的波特率加倍
#define BRTx12_     0x04    //0 - BRT每 12T 计数一次
							//1 - BRT每  1T 计数一次
#define EXTRAM_     0x02    //0 - 允许使用内部 1K Byte XRAM
                            //1 - 禁止使用内部 1K Byte XRAM  
#define S1BRS_      0x01    //0 - UART1 选择 T1  为波特率发生器
                            //1 - UART1 选择 BRT 为波特率发生器 							
							 
//                                        7     6       5      4     3      2      1      0   Reset Value
sfr AUXR1 = 0xA2; //Auxiliary Register 1  -  PCA_P4  SPI_P4  S2_P4  GF2    ADRJ    -     DPS  x000,00x0
/*
PCA_P4:
    0, 缺省PCA 在P1 口
    1，PCA/PWM 从P1 口切换到P4 口: ECI 从P1.2 切换到P4.1 口，
                                   PCA0/PWM0 从P1.3 切换到P4.2 口
                                   PCA1/PWM1 从P1.4 切换到P4.3 口
SPI_P4:
    0, 缺省SPI 在P1 口
    1，SPI 从P1 口切换到P4 口: SPICLK 从P1.7 切换到P4.3 口
                               MISO 从P1.6 切换到P4.2 口
                               MOSI 从P1.5 切换到P4.1 口
                               SS 从P1.4 切换到P4.0 口
S2_P4: 
    0, 缺省UART2 在P1 口
    1，UART2 从P1 口切换到P4 口: TxD2 从P1.3 切换到P4.3 口
                                 RxD2 从P1.2 切换到P4.2 口

GF2: 通用标志位

ADRJ:
    0, 10 位A/D 转换结果的高8 位放在ADC_RES 寄存器, 低2 位放在ADC_RESL 寄存器
    1，10 位A/D 转换结果的最高2 位放在ADC_RES 寄存器的低2 位, 低8 位放在ADC_RESL 寄存器

DPS: 0, 使用缺省数据指针DPTR0
     1，使用另一个数据指针DPTR1
*/

//-----------------------------------
sfr WAKE_CLKO = 0x8F; //附加的 SFR WAKE_CLKO
/*
      7            6          5          4          3       2       1       0         Reset Value
   PCAWAKEUP  RXD_PIN_IE  T1_PIN_IE  T0_PIN_IE  LVD_WAKE BRTCLKO  T1CLKO  T0CLKO      0000,0000B
以下均： 0 - 禁止； 1 - 允许
b7 - PCAWAKEUP  : PCA上升沿/下降沿中断可唤醒 powerdown。
b6 - RXD_PIN_IE : 当 P3.0(RXD) 下降沿置位 RI 时可唤醒 powerdown(必须打开相应中断)。
b5 - T1_PIN_IE  : 当 T1 脚下降沿置位 T1 中断标志时可唤醒 powerdown(必须打开相应中断)。
b4 - T0_PIN_IE  : 当 T0 脚下降沿置位 T0 中断标志时可唤醒 powerdown(必须打开相应中断)。
b3 - LVD_WAKE   : 当 EX_LVD/P4.6 脚电平低于低压检测阀值时置位 LVD 中断标志时可唤醒 powerdown(必须打开相应中断)。
b2 - BRTCLKO : 允许 P1.0输出BRT时钟 CLKOUT2，Fck2 = BRT溢出率 / 2 
                    BRT在1T模式时为  Fck2 = SYSclk / (256 - BRT) / 2
                    BRT在12T模式时为 Fck2 = SYSclk / 12 / (256 - BRT) / 2                  
b1 - T1CLKO  : 允许 T1CKO(P3.5) 脚输出 T1 溢出脉冲，Fck1 = T1溢出率 / 2   
                    ZZX:定时器需工作在模式2 (8位自动重装载)
                    若T1工作在 Timer 方式， 1T模式时为 Fck1 = SYSclk / (256 - TH1) / 2
					                       12T模式时为 Fck1 = SYSclk / 12 / (256 - TH1) / 2
					若T1工作在 Counter 方式，Fck1 = (T1_Pin_CLK) / (256 - TH1) / 2
b0 - T0CLKO  : 允许 T0CKO(P3.4) 脚输出 T0 溢出脉冲，Fck0 = T0溢出率 / 2	  
                    ZZX:定时器需工作在模式2 (8位自动重装载)
                    若T0工作在 Timer 方式， 1T模式时为 Fck0 = SYSclk / (256 - TH0) / 2
					                       12T模式时为 Fck0 = SYSclk / 12 / (256 - TH0) / 2
					若T0工作在 Counter 方式，Fck0 = (T0_Pin_CLK) / (256 - TH0) / 2
*/

//                                     7     6      5      4      3    2     1     0     Reset Value
sfr CLK_DIV = 0x97; //Clock Divder     -     -      -       -     -  CLKS2 CLKS1 CLKS0    xxxx,x000
//-----------------------------------
#define CLKS0_   0x01
#define CLKS1_   0x02
#define CLKS2_   0x04
//ZZX:  =0x0, 为不分频，即等于外部时钟(最高速度)

//                                            7     6      5      4      3    2     1     0     Reset Value
sfr BUS_SPEED = 0xA1; //Stretch register      -     -    ALES1   ALES0   -   RWS2  RWS1  RWS0    xx10,x011
/*
ALES1 and ALES0:
00 : The P0 address setup time and hold time to ALE negative edge is one clock cycle
01 : The P0 address setup time and hold time to ALE negative edge is two clock cycles.
10 : The P0 address setup time and hold time to ALE negative edge is three clock cycles. (default)
11 : The P0 address setup time and hold time to ALE negative edge is four clock cycles.

RWS2,RWS1,RWS0:
  000 : The MOVX read/write pulse is 1 clock cycle. 
  001 : The MOVX read/write pulse is 2 clock cycles.
  010 : The MOVX read/write pulse is 3 clock cycles.
  011 : The MOVX read/write pulse is 4 clock cycles. (default)
  100 : The MOVX read/write pulse is 5 clock cycles.
  101 : The MOVX read/write pulse is 6 clock cycles.
  110 : The MOVX read/write pulse is 7 clock cycles.
  111 : The MOVX read/write pulse is 8 clock cycles.
*/

//--------------------------------------------------------------------------------
//新一代 1T 8051系列 单片机中断特殊功能寄存器
//有的中断控制、中断标志位散布在其它特殊功能寄存器中，这些位在位地址中定义
//其中有的位无位寻址能力，请参阅 新一代 1T 8051系列 单片机中文指南
//                                           7     6     5    4     3    2    1    0   Reset Value
sfr IE      = 0xA8;  //中断控制寄存器        EA  ELVD  EADC   ES   ET1  EX1  ET0  EX0  0000,0000
//-----------------------
sbit EA       = IE^7;
sbit ELVD     = IE^6;  //低压监测中断允许位
sbit EADC     = IE^5;  //ADC 中断允许位
sbit ES       = IE^4;
sbit ET1      = IE^3;
sbit EX1      = IE^2;
sbit ET0      = IE^1;
sbit EX0      = IE^0;

//                                             7     6     5    4     3    2    1    0   Reset Value
sfr IE2       = 0xAF;  //Auxiliary Interrupt   -     -     -    -     -    -  ESPI  ES2  0000,0000B
//-----------------------
#define ES2_  0x01     //UART2中断允许
#define ESPI_ 0x02	   //SPI中断允许

//                                          7     6     5    4    3    2    1    0    Reset Value
sfr IP      = 0xB8; //中断优先级低位      PPCA  PLVD  PADC  PS   PT1  PX1  PT0  PX0   0000,0000
//-----------------------------------------
sbit PPCA     = IP^7;  //PCA 模块中断优先级
sbit PLVD     = IP^6;  //低压监测中断优先级
sbit PADC     = IP^5;  //ADC 中断优先级
sbit PS       = IP^4;
sbit PT1      = IP^3;
sbit PX1      = IP^2;
sbit PT0      = IP^1;
sbit PX0      = IP^0;

//                                         7      6      5     4     3     2     1     0    Reset Value
sfr IPH   = 0xB7; //中断优先级高位       PPCAH  PLVDH  PADCH  PSH  PT1H  PX1H  PT0H  PX0H   0000,0000
//------------------------------------------------
#define PX0H_       0x01   
#define PT0H_       0x02  
#define PX1H_       0x04   
#define PT1H_       0x08   
#define PSH_        0x10   
#define PADCH_      0x20  
#define PLVDH_      0x40
#define PPCAH_      0x80

//                                         7      6      5     4     3     2     1     0    Reset Value
sfr IP2   = 0xB5; //                       -      -      -     -     -     -   PSPI   PS2   xxxx,xx00
sfr IPH2  = 0xB6; //                       -      -      -     -     -     -   PSPIH  PS2H  xxxx,xx00
sfr IP2H  = 0xB6; //别名
//-----------------------
#define PS2_    0x01  //UART2优先级低位
#define PS2H_   0x01  //UART2优先级高位
#define PSPI_   0x02  //SPI优先级低位
#define PSPIH_  0x02  //SPI优先级高位

//-----------------------
//新一代 1T 8051系列 单片机I/O 口特殊功能寄存器
//                                      7     6     5     4     3     2     1     0         Reset Value
sfr P0   = 0x80; //8 bitPort0          P0.7  P0.6  P0.5  P0.4  P0.3  P0.2  P0.1  P0.0       1111,1111
sfr P0M0 = 0x94; //                                                                         0000,0000
sfr P0M1 = 0x93; //                                                                         0000,0000
sfr P1   = 0x90; //8 bitPort1          P1.7  P1.6  P1.5  P1.4  P1.3  P1.2  P1.1  P1.0       1111,1111
sfr P1M0 = 0x92; //                                                                         0000,0000
sfr P1M1 = 0x91; //                                                                         0000,0000
sfr P1ASF = 0x9D; //P1 analog special function												0000,0000
sfr P2   = 0xA0; //8 bitPort2          P2.7  P2.6  P2.5  P2.4  P2.3  P2.2  P2.1  P2.0       1111,1111
sfr P2M0 = 0x96; //                                                                         0000,0000
sfr P2M1 = 0x95; //                                                                         0000,0000
sfr P3   = 0xB0; //8 bitPort3          P3.7  P3.6  P3.5  P3.4  P3.3  P3.2  P3.1  P3.0       1111,1111
sfr P3M0 = 0xB2; //                                                                         0000,0000
sfr P3M1 = 0xB1; //                                                                         0000,0000
sfr P4   = 0xC0; //8 bitPort4          P4.7  P4.6  P4.5  P4.4  P4.3  P4.2  P4.1  P4.0       1111,1111
sfr P4M0 = 0xB4; //                                                                         0000,0000
sfr P4M1 = 0xB3; //                                                                         0000,0000
//                                      7      6         5         4      3     2     1     0     Reset Value
sfr P4SW = 0xBB; //Port-4 switch	      -   LVD_P4.6  ALE_P4.5  NA_P4.4   -     -     -     -	   x000,xxxx
/*    
    EX_LVD/P4.6: 0, 外部低压检测脚，可使用查询方式或设置成中断来检测 
	             1,	P4.6 I/O脚
	ALE/P4.5   : 0, ALE信号，只有在用MOVX访问片外扩展器件时才有信号输出
	             1, P4.5 I/O脚
    NA_P4.4    : 0, 弱上拉，无任何功能
	             1, P4.4 I/O脚
*/

sfr P5   = 0xC8; //8 bitPort5           -     -     -     -    P5.3  P5.2  P5.1  P5.0    xxxx,1111
sfr P5M0 = 0xCA; //                                                                      0000,0000
sfr P5M1 = 0xC9; // 																	 0000,0000

/*------------------------------------------------
P0 (0x80) Bit Registers  = 1111, 1111 
------------------------------------------------*/
sbit P0_0 = 0x80;
sbit P0_1 = 0x81;
sbit P0_2 = 0x82;
sbit P0_3 = 0x83;
sbit P0_4 = 0x84;
sbit P0_5 = 0x85;
sbit P0_6 = 0x86;
sbit P0_7 = 0x87;

/*------------------------------------------------
ZZX: P1 (0x90) Bit Registers  = 1111 1111b
------------------------------------------------*/
sbit P1_0 = 0x90;
sbit P1_1 = 0x91;
sbit P1_2 = 0x92;
sbit P1_3 = 0x93;
sbit P1_4 = 0x94;
sbit P1_5 = 0x95;
sbit P1_6 = 0x96;
sbit P1_7 = 0x97;

/*------------------------------------------------
ZZX: P2 (0xA0) Bit Registers = 1111 1111b
------------------------------------------------*/
sbit P2_0 = 0xA0;
sbit P2_1 = 0xA1;
sbit P2_2 = 0xA2;
sbit P2_3 = 0xA3;
sbit P2_4 = 0xA4;
sbit P2_5 = 0xA5;
sbit P2_6 = 0xA6;
sbit P2_7 = 0xA7;

/*------------------------------------------------
ZZX: P3 (0xB0) Bit Registers (Mnemonics & Ports)
------------------------------------------------*/
sbit P3_0 = 0xB0;
sbit P3_1 = 0xB1;
sbit P3_2 = 0xB2;
sbit P3_3 = 0xB3;
sbit P3_4 = 0xB4;
sbit P3_5 = 0xB5;
sbit P3_6 = 0xB6;
sbit P3_7 = 0xB7;

/*------------------------------------------------
ZZX: P4 (0xC0) Bit Registers (Mnemonics & Ports)
------------------------------------------------*/
sbit P4_0 = 0xC0;
sbit P4_1 = 0xC1;
sbit P4_2 = 0xC2;
sbit P4_3 = 0xC3;
sbit P4_4 = 0xC4;
sbit P4_5 = 0xC5;
sbit P4_6 = 0xC6;
sbit P4_7 = 0xC7;

/*------------------------------------------------
ZZX: P5 (0xC8) Bit Registers
------------------------------------------------*/
sbit P5_0 = 0xC8;
sbit P5_1 = 0xC9;
sbit P5_2 = 0xCA;
sbit P5_3 = 0xCB;


//--------------------------------------------------------------------------------
//新一代 1T 8051系列 单片机定时器特殊功能寄存器
//                                          7     6     5     4     3     2     1     0     Reset Value
sfr TCON = 0x88; //T0/T1 Control           TF1   TR1   TF0   TR0   IE1   IT1   IE0   IT0    0000,0000
//-----------------------------------
sbit TF1 = TCON^7;	 //Timer1 中断请求标志: 响应中断时，由硬件自动清0
sbit TR1 = TCON^6;	 //Timer1 运行控制位
sbit TF0 = TCON^5;	 //Timer0 中断请求标志: 响应中断时，由硬件自动清0
sbit TR0 = TCON^4;	 //Timer0 运行控制位
sbit IE1 = TCON^3;	 //INT1 中断请求标志: 响应中断时，由硬件自动清0	(边沿触发方式时)
sbit IT1 = TCON^2;	 //0-低电平; 1-下降沿
sbit IE0 = TCON^1;	 //INT0 中断请求标志: 响应中断时，由硬件自动清0	(边沿触发方式时)
sbit IT0 = TCON^0;	 //0-低电平; 1-下降沿

//                                          7     6     5     4     3     2     1     0     Reset Value
sfr TMOD = 0x89; //T0/T1 Modes             GATE1 C/T1  M1_1  M1_0  GATE0 C/T0  M0_1  M0_0   0000,0000
//-----------------------------------
#define T0_M0_   0x01   /* Timer 0 Mode Bit 0 */
#define T0_M1_   0x02   /* Timer 0 Mode Bit 1 */
#define T0_CT_   0x04   /* Timer 0 Counter/Timer Select: 1=Counter, 0=Timer */
#define T0_GATE_ 0x08   /* Timer 0 Gate Control */

#define T1_M0_   0x10   /* Timer 1 Mode Bit 0 */
#define T1_M1_   0x20   /* Timer 1 Mode Bit 1 */
#define T1_CT_   0x40   /* Timer 1 Counter/Timer Select: 1=Counter, 0=Timer */
#define T1_GATE_ 0x80   /* Timer 1 Gate Control */

#define T0_MASK_ 0x0F   /* Mask Timer1 Control-Bits */
#define T1_MASK_ 0xF0   /* Mask Timer0 Control-Bits */

sfr TL0  = 0x8A; //T0 Low Byte                                                              0000,0000
sfr TH0  = 0x8C; //T0 High Byte                                                             0000,0000
sfr TL1  = 0x8B; //T1 Low Byte                                                              0000,0000
sfr TH1  = 0x8D; //T1 High Byte                                                             0000,0000

//--------------------------------------------------------------------------------
//新一代 1T 8051系列 单片机串行口特殊功能寄存器
//                                          7     6     5     4     3     2     1     0     Reset Value
sfr SCON = 0x98; //Serial Control         SM0/FE SM1   SM2   REN   TB8   RB8    TI    RI    0000,0000
//-----------------------------------
sbit SM0 = SCON^7;  //SM0/FE
sbit SM1 = SCON^6;
sbit SM2 = SCON^5;
sbit REN = SCON^4;	//允许/禁止UART接收
sbit TB8 = SCON^3;	//在方式2或3,为要发送的第9位数据,为校验位或地址帧/数据帧标志位. 在方式0或1中,该位不用.
sbit RB8 = SCON^2;	//在方式2或3,为接收到的第9位数据,为校验位或地址帧/数据帧标志位. 在方式0或1中,该位不用.
sbit TI  = SCON^1;	//UART1发送中断标志：由用户中断服务程序清0
sbit RI  = SCON^0;	//UART1接收中断标志：由用户中断服务程序清0

//-----------------------------------
sfr SBUF  = 0x99; //Serial Data Buffer                                                    xxxx,xxxx
sfr SADEN = 0xB9; //Slave Address Mask                                                    0000,0000
sfr SADDR = 0xA9; //Slave Address                                                         0000,0000

//                                7      6      5      4      3      2     1     0        Reset Value
sfr S2CON = 0x9A; //S2 Control  S2SM0  S2SM1  S2SM2  S2REN  S2TB8  S2RB8  S2TI  S2RI      00000000B
#define S2SM0_    0x80
#define S2SM1_    0x40
#define S2SM2_    0x20
#define S2REN_    0x10
#define S2TB8_    0x08
#define S2RB8_    0x04
#define S2TI_     0x02
#define S2RI_     0x01

sfr S2BUF = 0x9B; //S2 Serial Buffer                                                      xxxx,xxxx
sfr BRT   = 0x9C; //S2或S1 的 Baud-Rate Timer                                             0000,0000
//注：串口2只能使用 BRT 作为波特率发生器.
//    串口1可使用 定时器1(上电默认) 或 BRT 作为波特率发生器.

//--------------------------------------------------------------------------------
//新一代 1T 8051系列 单片机看门狗定时器特殊功能寄存器
sfr WDT_CONTR = 0xC1; //Watch-Dog-Timer Control register
//                                      7     6     5      4       3      2   1   0     Reset Value
//                                  WDT_FLAG  -  EN_WDT CLR_WDT IDLE_WDT PS2 PS1 PS0    xx00,0000
//-------------------------------------------
#define WDT_PS0_   0x01   //WDT定时器预分频器
#define WDT_PS1_   0x02   
#define WDT_PS2_   0x04   
#define IDLE_WDT_  0x08   /* WDT 空闲模式计数：1 - 空闲模式下计数， 0 - 空闲模式下不计数 */
#define CLR_WDT_   0x10   /* WDT清除位：置1时WDT重新计数，硬件自动清0 */
#define EN_WDT_    0x20   /* WDT允许： 1 - 允许， 0 - 禁止 */
#define WDT_FLAG_  0x80   /* WDT溢出时，该位置1，须软件清0 */


//--------------------------------------------------------------------------------
//新一代 1T 8051系列 单片机PCA/PWM 特殊功能寄存器
//                                         7     6     5     4     3     2     1     0     Reset Value
sfr CCON   = 0xD8;   //PCA 控制寄存器。    CF    CR    -     -     -     -    CCF1  CCF0   00xx,xx00
//-----------------------------------
sbit CF    = CCON^7;  //PCA计数器溢出标志,由硬件或软件置位,必须由软件清0。
sbit CR    = CCON^6;  //1:允许 PCA 计数器计数, 必须由软件清0。
sbit CCF1  = CCON^1;  //PCA 模块1 中断标志, 由硬件置位, 必须由软件清0。
sbit CCF0  = CCON^0;  //PCA 模块0 中断标志, 由硬件置位, 必须由软件清0。

//                                          7     6     5     4     3     2     1     0    Reset Value
sfr CMOD  = 0xD9; //PCA 工作模式寄存器。   CIDL   -     -     -   CPS2   CPS1  CPS0  ECF   0xxx,x000
//-----------------------------------
#define ECF_     0x01
#define CPS0_    0x02
#define CPS1_    0x04
#define CPS2_    0x08
#define CIDL_    0x80 

/*
CIDL: idle 状态时 PCA 计数器是否继续计数, 0: 继续计数, 1: 停止计数。

CPS2: PCA 计数器脉冲源选择位 2。
CPS1: PCA 计数器脉冲源选择位 1。
CPS0: PCA 计数器脉冲源选择位 0。
   CPS2   CPS1   CPS0
    0      0      0    系统时钟频率 fosc/12。
    0      0      1    系统时钟频率 fosc/2。
    0      1      0    Timer0 溢出。
    0      1      1    由 ECI/P3.4 脚输入的外部时钟，最大 fosc/2。
    1      0      0    系统时钟频率，  Fosc/1
    1      0      1    系统时钟频率/4，Fosc/4
    1      1      0    系统时钟频率/6，Fosc/6
    1      1      1    系统时钟频率/8，Fosc/8

ECF: PCA计数器溢出中断允许位, 1--允许 CF(CCON.7) 产生中断。
*/

//                                         7     6      5      4     3     2     1     0     Reset Value
sfr CL     = 0xE9; //PCA 计数器低位                                                          0000,0000
sfr CH     = 0xF9; //PCA 计数器高位                                                          0000,0000
//-----------------------

//                                         7     6      5      4     3     2     1     0     Reset Value
sfr CCAPM0 = 0xDA; //PCA 模块0 PWM 寄存器  -   ECOM0  CAPP0  CAPN0  MAT0  TOG0  PWM0  ECCF0   x000,0000
//-----------------------------------
#define ECCF0_   0x01
#define PWM0_    0x02
#define TOG0_    0x04
#define MAT0_    0x08
#define CAPN0_   0x10
#define CAPP0_   0x20
#define ECOM0_   0x40

//                                         7     6      5      4     3     2     1     0     Reset Value
sfr CCAPM1 = 0xDB; //PCA 模块1 PWM 寄存器  -   ECOM1  CAPP1  CAPN1  MAT1  TOG1  PWM1  ECCF1   x000,0000
//-----------------------------------
#define ECCF1_   0x01
#define PWM1_    0x02
#define TOG1_    0x04
#define MAT1_    0x08
#define CAPN1_   0x10
#define CAPP1_   0x20
#define ECOM1_   0x40

//ECOMn = 1:允许比较功能。
//CAPPn = 1:允许上升沿触发捕捉功能。
//CAPNn = 1:允许下降沿触发捕捉功能。
//MATn  = 1:当匹配情况发生时, 允许 CCON 中的 CCFn 置位。
//TOGn  = 1:当匹配情况发生时, CEXn 将翻转。
//PWMn  = 1:将 CEXn 设置为 PWM 输出。
//ECCFn = 1:允许 CCON 中的 CCFn 触发中断。

//ECOMn  CAPPn  CAPNn  MATn  TOGn  PWMn  ECCFn
//  0      0      0     0     0     0     0   0x00   未启用任何功能。
//  x      1      0     0     0     0     x   0x21   16位CEXn上升沿触发捕捉功能。
//  x      0      1     0     0     0     x   0x11   16位CEXn下降沿触发捕捉功能。
//  x      1      1     0     0     0     x   0x31   16位CEXn边沿(上、下沿)触发捕捉功能。
//  1      0      0     1     0     0     x   0x49   16位软件定时器。
//  1      0      0     1     1     0     x   0x4d   16位高速脉冲输出。
//  1      0      0     0     0     1     0   0x42   8位 PWM。

//ECOMn  CAPPn  CAPNn  MATn  TOGn  PWMn  ECCFn
//  0      0      0     0     0     0     0   0x00   无此操作
//  1      0      0     0     0     1     0   0x42   普通8位PWM, 无中断
//  1      1      0     0     0     1     1   0x63   PWM输出由低变高可产生中断
//  1      0      1     0     0     1     1   0x53   PWM输出由高变低可产生中断
//  1      1      1     0     0     1     1   0x73   PWM输出由低变高或由高变低都可产生中断

//-----------------------
sfr CCAP0L = 0xEA; //PCA 模块 0 的捕捉/比较寄存器低 8 位。                                    0000,0000
sfr CCAP0H = 0xFA; //PCA 模块 0 的捕捉/比较寄存器高 8 位。                                    0000,0000
sfr CCAP1L = 0xEB; //PCA 模块 1 的捕捉/比较寄存器低 8 位。                                    0000,0000
sfr CCAP1H = 0xFB; //PCA 模块 1 的捕捉/比较寄存器高 8 位。                                    0000,0000
//-----------------------
//                                                       7   6   5   4   3   2    1     0    Reset Value
sfr PCA_PWM0 = 0xF2; //PCA 模块0 PWM 寄存器。            -   -   -   -   -   -  EPC0H EPC0L   xxxx,xx00
sfr PCA_PWM1 = 0xF3; //PCA 模块1 PWM 寄存器。            -   -   -   -   -   -  EPC1H EPC1L   xxxx,xx00
#define EPC0L_   0x01
#define EPC0H_   0x02
#define EPC1L_   0x01
#define EPC1H_   0x02

//--------------------------------------------------------------------------------
//新一代 1T 8051系列 单片机 ADC 特殊功能寄存器
//                                            7        6      5       4         3      2    1    0   Reset Value
sfr ADC_CONTR = 0xBC; //A/D 转换控制寄存器 ADC_POWER SPEED1 SPEED0 ADC_FLAG ADC_START CHS2 CHS1 CHS0 0000,0000
//-----------------------------------------------------------
#define CHS0_        0x01
#define CHS1_        0x02  
#define CHS2_        0x04   
#define ADC_START_   0x08  	//ADC转换启动控制： 1-开始转换； 0-转换结束
#define ADC_FLAG_    0x10   //ADC转换结束标志（中断请求标志），须软件清除
#define SPEED0_      0x20   
#define SPEED1_      0x40    
#define ADC_POWER_   0x80   //ADC电源控制位：0-关闭； 1-打开

//zzx: 当 AUXR1.ADRJ = 0 时, ADC_RES存放转换结果高8位, ADC_RESL存放转换结果低2位
sfr ADC_RES  = 0xBD;  //A/D 转换结果高8位 ADCV.9 ADCV.8 ADCV.7 ADCV.6 ADCV.5 ADCV.4 ADCV.3 ADCV.2	 0000,0000
sfr ADC_RESL = 0xBE;  //A/D 转换结果低2位                                           ADCV.1 ADCV.0	 0000,0000

//--------------------------------------------------------------------------------
//新一代 1T 8051系列 单片机 SPI 特殊功能寄存器
//                                          7     6     5     4     3     2     1     0    Reset Value
sfr SPCTL  = 0xCE; //SPI Control Register  SSIG  SPEN  DORD  MSTR  CPOL  CPHA  SPR1  SPR0  0000,0100
sfr SPSTAT = 0xCD; //SPI Status Register   SPIF  WCOL   -     -     -     -     -     -    00xx,xxxx
sfr SPDAT  = 0xCF; //SPI Data Register                                                     0000,0000
//-----------------------------------------------
#define WCOL_   0x40
#define SPIF_   0x80

//--------------------------------------------------------------------------------
//新一代 1T 8051系列 单片机 IAP/ISP 特殊功能寄存器
//                                                7    6    5      4    3    2    1     0    Reset Value
sfr IAP_DATA    = 0xC2;	//                       											 1111,1111
sfr IAP_ADDRH   = 0xC3;	//																	 0000,0000
sfr IAP_ADDRL   = 0xC4;	//																	 0000,0000
sfr IAP_CMD     = 0xC5; //IAP Mode Table          -    -    -      -    -    -   MS1   MS0   xxxx,xx00
sfr IAP_TRIG    = 0xC6; //                                                                   xxxx,xxxx
sfr IAP_CONTR   = 0xC7; //IAP Control Register  IAPEN SWBS SWRST CFAIL  -   WT2  WT1   WT0   0000,x000
//-------------------------------------------------------------------------------- 
#define WT0_       0x01    // ISP_CONTR中的控制位： 设置等待时间
#define WT1_       0x02
#define WT2_       0x04
#define CMD_FAIL_  0x10    // ISP_CONTR中的控制位： 如果发送了触发命令，但失败，则该位置1；需由软件清零
#define SWRST_     0x20    // ISP_CONTR中的控制位： 是否产生软件系统复位及硬件自动清零, 1-是， 0-不操作
#define SWBS_      0x40    // ISP_CONTR中的控制位： 选择从用户主程序区还是ISP程序区启动， 0-用户主程序区， 1-ISP程序区 
#define IAPEN_     0x80    // ISP_CONTR中的控制位： 允许读/写/擦除 Data flash, 1-允许， 0-禁止

/*------------------------------------------------
Interrupt Vectors:
Interrupt Address = (Number * 8) + 3
------------------------------------------------*/
#define IE0_VECTOR	    0  /* 0x03 External Interrupt 0 , interrupt request flag = IE0 */
#define TF0_VECTOR	    1  /* 0x0B Timer 0 , interrupt request flag = TF0 */
#define IE1_VECTOR	    2  /* 0x13 External Interrupt 1 , interrupt request flag = IE1 */
#define TF1_VECTOR	    3  /* 0x1B Timer 1 , interrupt request flag = TF1 */
#define SIO_VECTOR	    4  /* 0x23 Serial port (UART1) , interrupt request flag = RI + TI */
#define ADC_VECTOR	    5  /* 0x2B ADC, , interrupt request flag = ADC_FLAG */
#define LVD_VECTOR      6  /* 0x33 LVD, interrupt request flag = LVDF */
#define PCA_VECTOR	    7  /* 0x3B PCA, interrupt request flag = CF + CCF0 + CCF1 */
#define SIO2_VECTOR	    8  /* 0x43 UART2, interrupt request flag = S2TI + S2RI */
#define SPI_VECTOR	    9  /* 0x4B SPI, interrupt request flag = SPIF */

#endif
