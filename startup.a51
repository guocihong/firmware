/*A**************************************************************************
* NAME:    startup.a51
*----------------------------------------------------------------------------
* Copyright (c) 2011.
*----------------------------------------------------------------------------
* RELEASE:      2011.01.13
* REVISION:     1.0   
*----------------------------------------------------------------------------
* PURPOSE:	 This file contains the start-up execution routine
* Note:      only for STC12C5Axx-serial
*****************************************************************************/

;_____ I N C L U D E S ____________________________________________________


;_____ M A C R O S ________________________________________________________

;  User-defined Power-On Initialization of Memory
;
;  With the following EQU statements the initialization of memory
;  at processor reset can be defined:
;
;  the absolute start-address of IDATA memory is always 0
IDATALEN    EQU 100H    ; the length of IDATA memory in bytes.
                        ; 256 bytes
XDATASTART  EQU 0H      ; the absolute start-address of XDATA memory
XDATALEN    EQU 400H    ; 1024 bytes
                       
PDATASTART  EQU 0H      ; the absolute start-address of PDATA memory
PDATALEN    EQU 0H      ; the length of PDATA memory in bytes.
                      
;
;  Notes:  The IDATA space overlaps physically the DATA and BIT areas of the
;          8051 CPU. At minimum the memory space occupied from the C51 
;          run-time routines must be set to zero.
;------------------------------------------------------------------------------
;
;  Reentrant Stack Initilization
;
;  The following EQU statements define the stack pointer for reentrant
;  functions and initialized it:
;
;  Stack Space for reentrant functions in the SMALL model.
IBPSTACK    EQU 0       ; set to 1 if small reentrant is used.
IBPSTACKTOP EQU 0FFH+1  ; set top of stack to highest location+1.
;
;  Stack Space for reentrant functions in the LARGE model.	
XBPSTACK    EQU 0       ; set to 1 if large reentrant is used.
XBPSTACKTOP EQU 3FFH+1  ; set top of stack to highest location+1.                
;
;  Stack Space for reentrant functions in the COMPACT model.	
PBPSTACK    EQU 0       ; set to 1 if compact reentrant is used.
PBPSTACKTOP EQU 0FFH+1  ; set top of stack to highest location+1.
                   
;
;------------------------------------------------------------------------------

; define a module name 
        NAME    ?C_STARTUP   


;_____ D E F I N I T I O N ________________________________________________

; segment location
?C_C51STARTUP   SEGMENT   CODE
?STACK          SEGMENT   IDATA

        RSEG    ?STACK
        DS      1


;_____ D E C L A R A T I O N ______________________________________________

; symbol declaration
        EXTRN CODE (?C_START)
        PUBLIC  ?C_STARTUP

;*F**************************************************************************
; NAME: C_STARTUP
;----------------------------------------------------------------------------
; PARAMS:
; return:
;----------------------------------------------------------------------------
; PURPOSE: Reset vector, 0000H
;----------------------------------------------------------------------------
; REQUIREMENTS: 
;****************************************************************************
        CSEG    AT  0

?C_STARTUP:
        LJMP    STARTUP1


;*F**************************************************************************
; NAME: STARTUP1
;----------------------------------------------------------------------------
; PARAMS:
; return:
;----------------------------------------------------------------------------
; PURPOSE: Memory initialization 
;----------------------------------------------------------------------------
; REQUIREMENTS: 
;****************************************************************************

        RSEG    ?C_C51STARTUP

STARTUP1:

IF IDATALEN <> 0
        MOV     R0,#IDATALEN - 1
        CLR     A
  IDATALOOP:
        MOV     @R0,A
        DJNZ    R0,IDATALOOP
ENDIF


IF XDATALEN <> 0
        MOV     DPTR,#XDATASTART
        MOV     R7,#LOW (XDATALEN)
  IF (LOW (XDATALEN)) <> 0
        MOV     R6,#(HIGH XDATALEN) +1
  ELSE
        MOV     R6,#HIGH (XDATALEN)
  ENDIF
        CLR     A
  XDATALOOP:
        MOVX    @DPTR,A
        INC     DPTR
        DJNZ    R7,XDATALOOP
        DJNZ    R6,XDATALOOP
ENDIF


IF PDATALEN <> 0
        MOV     R0,#PDATASTART
        MOV     R7,#LOW (PDATALEN)
        CLR     A
   PDATALOOP:
        MOVX    @R0,A
        INC     R0
        DJNZ    R7,PDATALOOP
ENDIF


IF IBPSTACK <> 0
EXTRN DATA (?C_IBP)
        MOV     ?C_IBP,#LOW IBPSTACKTOP
ENDIF

IF XBPSTACK <> 0
EXTRN DATA (?C_XBP)
        MOV     ?C_XBP,#HIGH XBPSTACKTOP
        MOV     ?C_XBP+1,#LOW XBPSTACKTOP
ENDIF

IF PBPSTACK <> 0
EXTRN DATA (?C_PBP)
        MOV     ?C_PBP,#LOW PBPSTACKTOP
ENDIF


        MOV     SP,#?STACK-1
        LJMP    ?C_START

        END

