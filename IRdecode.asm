;----------------------------------------------------------
; Code Produced by the Proton Compiler. Ver 3.5.2.7
; Copyright Rosetta Technologies/Crownhill Associates Ltd
; Written by Les Johnson. November 2011
;----------------------------------------------------------
;
#define CONFIG_REQ 1
 LIST  P = 16F73, F = INHX8M, W = 2, X = ON, R = DEC, MM = ON, N = 0, C = 255
INDF equ 0X0000
TMR0 equ 0X0001
PCL equ 0X0002
STATUS equ 0X0003
FSR equ 0X0004
PORTA equ 0X0005
PORTB equ 0X0006
PORTC equ 0X0007
PCLATH equ 0X000A
INTCON equ 0X000B
PIR1 equ 0X000C
PIR2 equ 0X000D
TMR1L equ 0X000E
TMR1LH equ 0X000F
TMR1H equ 0X000F
T1CON equ 0X0010
TMR2 equ 0X0011
T2CON equ 0X0012
SSPBUF equ 0X0013
SSPCON equ 0X0014
CCPR1L equ 0X0015
CCPR1LH equ 0X0016
CCPR1H equ 0X0016
CCP1CON equ 0X0017
RCSTA equ 0X0018
TXREG equ 0X0019
RCREG equ 0X001A
CCPR2L equ 0X001B
CCPR2LH equ 0X001C
CCPR2H equ 0X001C
CCP2CON equ 0X001D
ADRES equ 0X001E
ADCON0 equ 0X001F
OPTION_REG equ 0X0081
TRISA equ 0X0085
TRISB equ 0X0086
TRISC equ 0X0087
PIE1 equ 0X008C
PIE2 equ 0X008D
PCON equ 0X008E
PR2 equ 0X0092
SSPADD equ 0X0093
SSPSTAT equ 0X0094
TXSTA equ 0X0098
SPBRG equ 0X0099
ADCON1 equ 0X009F
EEDATL equ 0X010C
PMDATL equ 0X010C
EEDATA equ 0X010C
EEDAT equ 0X010C
PMDATA equ 0X010C
EEADR equ 0X010D
PMADR equ 0X010D
EEDATLH equ 0X010E
PMDATLH equ 0X010E
EEDATH equ 0X010E
PMDATH equ 0X010E
EEADRH equ 0X010F
PMADRH equ 0X010F
EECON1 equ 0X018C
PMCON1 equ 0X018C
_I2C_SCL_PORT=TRISC
_I2C_SCL_PIN=3
_I2C_SDA_PORT=TRISC
_I2C_SDA_PIN=4
IRP=7
RP1=6
RP0=5
NOT_TO=4
NOT_PD=3
Z=2
DC=1
C=0
GIE=7
PEIE=6
T0IE=5
INTE=4
RBIE=3
T0IF=2
INTF=1
RBIF=0
ADIF=6
RCIF=5
PP_RCIF=5
TXIF=4
PP_TXIF=4
SSPIF=3
CCP1IF=2
TMR2IF=1
TMR1IF=0
CCP2IF=0
T1CKPS1=5
T1CKPS0=4
T1OSCEN=3
NOT_T1SYNC=2
T1INSYNC=2
TMR1CS=1
TMR1ON=0
TOUTPS3=6
TOUTPS2=5
TOUTPS1=4
TOUTPS0=3
TMR2ON=2
PP_TMR2ON=2
T2CKPS1=1
PP_T2CKPS1=1
T2CKPS0=0
PP_T2CKPS0=0
WCOL=7
SSPOV=6
SSPEN=5
CKP=4
SSPM3=3
SSPM2=2
SSPM1=1
SSPM0=0
CCP1X=5
CCP1Y=4
CCP1M3=3
CCP1M2=2
CCP1M1=1
CCP1M0=0
SPEN=7
RX9=6
RC9=6
NOT_RC8=6
RC8_9=6
SREN=5
CREN=4
PP_CREN=4
FERR=2
OERR=1
PP_OERR=1
RX9D=0
RCD8=0
CCP2X=5
CCP2Y=4
CCP2M3=3
CCP2M2=2
CCP2M1=1
CCP2M0=0
ADCS1=7
ADCS0=6
CHS2=5
CHS1=4
CHS0=3
GO=2
NOT_DONE=2
GO_DONE=2
PP_GO_DONE=2
ADON=0
NOT_RBPU=7
INTEDG=6
T0CS=5
T0SE=4
PSA=3
PS2=2
PS1=1
PS0=0
ADIE=6
RCIE=5
TXIE=4
SSPIE=3
CCP1IE=2
TMR2IE=1
TMR1IE=0
CCP2IE=0
NOT_POR=1
NOT_BO=0
NOT_BOR=0
SMP=7
CKE=6
I2C_DATA=5
NOT_A=5
NOT_ADDRESS=5
D_A=5
DATA_ADDRESS=5
I2C_STOP=4
I2C_START=3
I2C_READ=2
NOT_W=2
NOT_WRITE=2
R_W=2
PP_R_W=2
READ_WRITE=2
UA=1
BF=0
PP_BF=0
CSRC=7
TX9=6
NOT_TX8=6
TX8_9=6
TXEN=5
SYNC=4
BRGH=2
TRMT=1
TX9D=0
TXD8=0
PCFG2=2
PCFG1=1
PCFG0=0
RD=0
PP_RD=0
  __MAXRAM 0X1FF
  __BADRAM 0X08-0X09
  __BADRAM 0X88-0X89, 0X8F-0X91, 0X95-0X97, 0X9A-0X9E
  __BADRAM 0X105, 0X107-0X109, 0X110-0X11F
  __BADRAM 0X185, 0X187-0X189, 0X18D-0X19F
BODEN_ON equ 0X3FFF
BODEN_OFF equ 0X3FBF
CP_ALL equ 0X3FEF
CP_OFF equ 0X3FFF
PWRTE_OFF equ 0X3FFF
PWRTE_ON equ 0X3FF7
WDT_ON equ 0X3FFF
WDT_OFF equ 0X3FFB
LP_OSC equ 0X3FFC
XT_OSC equ 0X3FFD
HS_OSC equ 0X3FFE
RC_OSC equ 0X3FFF
BOREN_OFF equ 0X3FBF
BOREN_ON equ 0X3FFF
CP_ON equ 0X3FEF
FOSC_HS equ 0X3FFE
FOSC_LP equ 0X3FFC
FOSC_RC equ 0X3FFF
FOSC_XT equ 0X3FFD
WDTE_OFF equ 0X3FFB
WDTE_ON equ 0X3FFF
#define __16F73 1
#define XTAL 8
#define _CORE 14
#define _MAXRAM 192
#define _RAM_END 0X00C0
#define _MAXMEM 4096
#define _ADC 5
#define _ADC_RES 8
#define _EEPROM 0
#define _PAGES 2
#define _BANKS 3
#define RAM_BANKS 2
#define _USART 1
#define _USB 0
#define _FLASH 2
#define _CWRITE_BLOCK 0
#define BANK0_START 0X20
#define BANK0_END 0X7F
#define BANK1_START 0X00A0
#define BANK1_END 0X00FF
#define BANK2_START 0X0110
#define BANK2_END 0X0120
#define BANK3_START 0X0190
#define BANK3_END 0X01A0
#define _SYSTEM_VARIABLE_COUNT 17
ram_bank = 0
CURRENT@PAGE = 0
DEST@PAGE = 0
#define LCD#TYPE 0
f@call macro PDEST
if(PDEST < 1)
if((PDEST & 2048) == 0)
        bcf 10,3
else
        bsf 10,3
endif
else
if(PDEST > $)
if((PDEST & 2048) == 0)
        bcf 10,3
else
        bsf 10,3
endif
else
if((PDEST & 6144) == 0)
        clrf 10
else
if((PDEST & 2048) == 0)
        bcf 10,3
else
        bsf 10,3
endif
endif
endif
endif
        call PDEST
        endm
F@JUMP macro PDEST
if(PDEST < 1)
if((PDEST & 2048) == 0)
        bcf 10,3
else
        bsf 10,3
endif
else
if(PDEST > $)
if((PDEST & 2048) == 0)
        bcf 10,3
else
        bsf 10,3
endif
else
if((PDEST & 6144) == 0)
        clrf 10
else
if((PDEST & 2048) == 0)
        bcf 10,3
else
        bsf 10,3
endif
endif
endif
endif
        goto PDEST
        endm
set@page macro PDEST
if((PDEST & 2048) == 0)
        bcf 10,3
else
        bsf 10,3
endif
        endm
s@b     macro pVarin
if((pVarin & 384) == 0)
if(ram_bank == 1)
        bcf 3,5
endif
if(ram_bank == 2)
        bcf 3,6
endif
if(ram_bank == 3)
        bcf 3,5
        bcf 3,6
endif
ram_bank = 0
endif
if((pVarin & 384) == 128)
if(ram_bank == 0)
        bsf 3,5
endif
if(ram_bank == 2)
        bsf 3,5
        bcf 3,6
endif
if(ram_bank == 3)
        bcf 3,6
endif
ram_bank = 1
endif
if((pVarin & 384) == 256)
if(ram_bank == 0)
        bsf 3,6
endif
if(ram_bank == 1)
        bcf 3,5
        bsf 3,6
endif
if(ram_bank == 3)
        bcf 3,5
endif
ram_bank = 2
endif
if((pVarin & 384) == 384)
if(ram_bank == 0)
        bsf 3,5
        bsf 3,6
endif
if(ram_bank == 1)
        bsf 3,6
endif
if(ram_bank == 2)
        bsf 3,5
endif
ram_bank = 3
endif
        endm
r@b     macro
if((ram_bank & 1) != 0)
        bcf 3,5
endif
if((ram_bank & 2) != 0)
        bcf 3,6
endif
ram_bank = 0
        endm
jump macro PLABEL
    goto PLABEL
    endm
wreg_byte macro pByteOut
    s@b pByteOut
    movwf pByteOut
    r@b
    endm
wreg_bit macro pVarOut,pBitout
    s@b pVarOut
    andlw 1
    btfsc STATUS,2
    bcf pVarOut,pBitout
    btfss STATUS,2
    bsf pVarOut,pBitout
    r@b
    endm
wreg_word macro pWordOut
    s@b pWordOut
    movwf pWordOut
    s@b pWordOut+1
    clrf pWordOut+1
    r@b
    endm
wreg_dword macro pDwordOut
    s@b pDwordOut+3
    clrf pDwordOut+3
    s@b pDwordOut+2
    clrf pDwordOut+2
    s@b pDwordOut+1
    clrf pDwordOut+1
    s@b pDwordOut
    movwf pDwordOut
    r@b
    endm
byte_wreg macro pByteIn
    s@b pByteIn
    movf pByteIn,W
    r@b
    endm
num_wreg macro pNumIn
    movlw (pNumIn & 255)
    endm
num_byte macro pNumIn,pByteOut
    s@b pByteOut
if(pNumIn == 0)
    clrf pByteOut
else
    movlw (pNumIn & 255)
    movwf pByteOut
endif
    r@b
    endm
num_bit macro pNumIn,pVarOut,pBitout
    s@b pVarOut
if((pNumIn & 1) == 1)
    bsf pVarOut,pBitout
else
    bcf pVarOut,pBitout
endif
    r@b
    endm
num_word macro pNumIn,pWordOut
if((pNumIn & 255) == 0)
    s@b pWordOut
    clrf pWordOut
else
    s@b pWordOut
    movlw low (pNumIn)
    movwf pWordOut
endif
if(((pNumIn >> 8) & 255) == 0)
    s@b pWordOut+1
    clrf pWordOut+1
else
    s@b pWordOut+1
    movlw high (pNumIn)
    movwf pWordOut+1
endif
    r@b
    endm
num_dword macro pNumIn,pDwordOut
if ((pNumIn >> 24 & 255) == 0)
    s@b pDwordOut+3
    clrf pDwordOut+3
else
    s@b pDwordOut+3
    movlw ((pNumIn >> 24) & 255)
    movwf pDwordOut+3
endif
if( ((pNumIn >> 16) & 255) == 0)
    s@b pDwordOut+2
    clrf pDwordOut+2
else
    s@b pDwordOut+2
    movlw ((pNumIn >> 16) & 255)
    movwf pDwordOut+2
endif
if( ((pNumIn >> 8) & 255) == 0)
    s@b pDwordOut+1
    clrf pDwordOut+1
else
    s@b pDwordOut+1
    movlw high (pNumIn)
    movwf pDwordOut+1
endif
if((pNumIn & 255) == 0)
    s@b pDwordOut
    clrf pDwordOut
else
    s@b pDwordOut
    movlw low (pNumIn)
    movwf pDwordOut
endif
    r@b
    endm
bit_wreg macro pVarin,pBitIn
    s@b pVarin
    clrw
    btfsc pVarin,pBitIn
    movlw 1
    r@b
    endm
bit_byte macro pVarin,pBitIn,pByteOut
    s@b pVarin
    clrw
    btfsc pVarin,pBitIn
    movlw 1
    s@b pByteOut
    movwf pByteOut
    r@b
    endm
bit_bit macro pVarin,pBitIn,pVarOut,pBitout
if((pVarin & 65408) == (pVarOut & 65408))
    s@b pVarOut
    btfsc pVarin,pBitIn
    bsf pVarOut,pBitout
    btfss pVarin,pBitIn
    bcf pVarOut,pBitout
else
    s@b pVarin
    clrdc
    btfsc pVarin,pBitIn
    setdc
    s@b pVarOut
    skpndc
    bsf pVarOut,pBitout
    skpdc
    bcf pVarOut,pBitout
endif
    endm
bit_word macro pVarin,pBitIn,pWordOut
    s@b pWordOut+1
    clrf pWordOut+1
    bit_byte pVarin,pBitIn,pWordOut
    endm
bit_dword macro pVarin,pBitIn,pDwordOut
    s@b pDwordOut+3
    clrf pDwordOut+3
    s@b pDwordOut+2
    clrf pDwordOut+2
    s@b pDwordOut+1
    clrf pDwordOut+1
    bit_byte pVarin,pBitIn,pDwordOut
    endm
word_wreg macro pWordIn
    byte_wreg pWordIn
    endm
word_byte macro pWordIn,pByteOut
    byte_byte pWordIn,pByteOut
    endm
word_bit macro pWordIn,pVarOut,pBitout
    byte_bit pWordIn, pVarOut, pBitout
    endm
word_word macro pWordIn,pWordOut
    s@b pWordIn+1
    movf pWordIn+1,W
    s@b pWordOut+1
    movwf pWordOut+1
    byte_byte pWordIn,pWordOut
    endm
word_dword macro pWordIn,pDwordOut
    s@b pDwordOut+3
    clrf pDwordOut+3
    s@b pDwordOut+2
    clrf pDwordOut+2
    byte_byte pWordIn+1,pDwordOut+1
    byte_byte pWordIn,pDwordOut
    endm
byte_byte macro pByteIn,pByteOut
    s@b pByteIn
    movf pByteIn,W
    s@b pByteOut
    movwf pByteOut
    r@b
    endm
byte_word macro pByteIn,pWordOut
    s@b pWordOut+1
    clrf pWordOut+1
    byte_byte pByteIn,pWordOut
    endm
byte_dword macro pByteIn,pDwordOut
    s@b pDwordOut+3
    clrf pDwordOut+3
    s@b pDwordOut+2
    clrf pDwordOut+2
    s@b pDwordOut+1
    clrf pDwordOut+1
    byte_byte pByteIn,pDwordOut
    endm
    byte_bit macro pByteIn,pVarOut,pBitout
if((pByteIn & 65408) == (pVarOut & 65408))
    s@b pByteIn
    btfsc pByteIn,0
    bsf pVarOut,pBitout
    btfss pByteIn,0
    bcf pVarOut,pBitout
else
    s@b pByteIn
    rrf pByteIn,W
    s@b pVarOut
    skpnc
    bsf pVarOut,pBitout
    skpc
    bcf pVarOut,pBitout
endif
    r@b
    endm
dword_wreg macro pDwordIn
    byte_wreg pDwordIn
    endm
dword_byte macro pDwordIn,pByteOut
    byte_byte pDwordIn,pByteOut
    endm
dword_word macro pDwordIn,pWordOut
    s@b pDwordIn+1
    movf pDwordIn+1,W
    s@b pWordOut+1
    movwf pWordOut+1
    byte_byte pDwordIn,pWordOut
    endm
dword_dword macro pDwordIn,pDwordOut
    byte_byte pDwordIn+3,pDwordOut+3
    byte_byte pDwordIn+2,pDwordOut+2
    byte_byte pDwordIn+1,pDwordOut+1
    byte_byte pDwordIn,pDwordOut
    endm
dword_bit macro pDwordIn,pVarOut,pBitout
    byte_bit pDwordIn,pVarOut,pBitout
    endm
variable CURRENT@PAGE = 0
variable PDESTINATION@PAGE = 0
FIND@PAGE macro pLabelIn
local CURRENT_ADDR = $
local DEST_ADDR = pLabelIn
if((CURRENT_ADDR >= 0X1800) && (CURRENT_ADDR <= 0X2000))
    CURRENT@PAGE = 3
endif
if((CURRENT_ADDR >= 0X1000) && (CURRENT_ADDR <= 0X1800))
    CURRENT@PAGE = 2
endif
if((CURRENT_ADDR >= 0X0800) && (CURRENT_ADDR <= 0X1000))
    CURRENT@PAGE = 1
endif
if((CURRENT_ADDR >= 0) && (CURRENT_ADDR <= 0X0800))
    CURRENT@PAGE = 0
endif
if((DEST_ADDR >= 0X1800) && (DEST_ADDR <= 0X2000))
    PDESTINATION@PAGE = 3
endif
if((DEST_ADDR >= 0X1000) && (DEST_ADDR <= 0X1800))
    PDESTINATION@PAGE = 2
endif
if((DEST_ADDR >= 0X0800) && (DEST_ADDR <= 0X1000))
    PDESTINATION@PAGE = 1
endif
if((DEST_ADDR >= 0) && (DEST_ADDR <= 0X0800))
    PDESTINATION@PAGE = 0
endif
    endm
NUM_FSR macro pNumIn
    num_byte pNumIn, FSR
if (((pNumIn >> 8) & 255) == 1)
    bsf STATUS,7
else
    bcf STATUS,7
endif
    endm
label_word macro pLabelIn,pWordOut
    movlw low (pLabelIn)
    s@b pWordOut
    movwf pWordOut
    movlw high (pLabelIn)
    s@b pWordOut
    movwf pWordOut+1
    r@b
    endm
BPF = 32
BPFH = 33
GEN4 = 34
GEN4H = 35
GPR = 36
PP0 = 37
PP0H = 38
PP1 = 39
PP1H = 40
PP2 = 41
PP2H = 42
PP3 = 43
PP3H = 44
PP4 = 45
PP6 = 46
PP6H = 47
SP#P9 = 48
_time = 49
_timeH = 50
_counter = 51
_cmdBin = 52
variable _cmdBin#0=52,_cmdBin#1=53,_cmdBin#2=54,_cmdBin#3=55
variable _cmdBin#4=56,_cmdBin#5=57,_cmdBin#6=58,_cmdBin#7=59
command = 60
#define IR_in PORTC,3
#define __XTAL 8
#define __HSERIAL_BAUD 9600
#define __HSERIAL_RCSTA 144
#define __HSERIAL_TXSTA 36
HSERIAL_SPBRG = 51
HSERIAL_TXSTA = 36
HSERIAL_RCSTA = 144
HSERIAL_BAUD = 9600
proton#code#start
        org 0
        goto proton#main#start
T@GT
        movwf 39
        movlw 1
        goto T@ST
T@LT
        movwf 39
        movlw 4
T@ST
        movwf 36
        movf 40,W
        subwf 38,W
        skpz
        goto $ + 3
        movf 39,W
        subwf 37,W
        movlw 4
        skpnc
        movlw 1
        skpnz
        movlw 2
        andwf 36,W
        skpz
        movlw 1
        goto I@NT
OUT@DECB
        clrf 35
OUT@DECC
        movwf 41
        clrf 42
OUT@DEC
        bcf 32,3
        movf 35,W
        skpnz
        bsf 32,3
        movlw 5
        movwf 34
        movlw 39
        movwf 40
        movlw 16
        call D@DIG
        movlw 3
        movwf 40
        movlw 232
        call D@DIG
        clrf 40
        movlw 100
        call D@DIG
        clrf 40
        movlw 10
        call D@DIG
        movf 41,W
        goto SEND@IT
D@DIG
        movwf 39
        movf 42,W
        movwf 38
        movf 41,W
        movwf 37
        call D@VD
        movf 37,W
SEND@IT
        movwf 37
        decf 34,F
        skpnz
        bcf 32,3
        movf 35,W
        skpnz
        goto $ + 4
        subwf 34,W
        skpnc
        goto EX@SEND@IT
        movf 37,W
        skpz
        bcf 32,3
        btfsc 32,3
        goto EX@SEND@IT
        addlw 48
        goto CH@SND
EX@SEND@IT
        return
HRSOUT
        btfss 12,PP_TXIF
        goto $ - 1
        movwf 25
        bsf 3,0
        goto I@NT
CH@SND
        btfsc 33,2
        goto HRSOUT
        return
D@VD
        clrf 42
        clrf 41
D@VD2
        movlw 16
        movwf 43
        rlf 38,W
        rlf 41,F
        rlf 42,F
        movf 39,W
        subwf 41,F
        movf 40,W
        skpc
        incfsz 40,W
        subwf 42,F
        skpnc
        goto $ + 8
        movf 39,W
        addwf 41,F
        movf 40,W
        skpnc
        incfsz 40,W
        addwf 42,F
        bcf 3,0
        rlf 37,F
        rlf 38,F
        decfsz 43,F
        goto $ - 21
        movf 37,W
        return
I@NT
        bcf 3,7
I@NT2
        bcf 3,5
        bcf 3,6
        return
proton#main#start
        clrf 3
        bsf 3,5
        movlw 51
        movwf 153
        movlw 36
        movwf 152
        bcf 3,5
        movlw 144
        movwf 24
        clrf 32
F2_SOF equ $ ; IRDECODE.PRP
F2_EOF equ $ ; IRDECODE.PRP
F1_SOF equ $ ; IRDECODE.BAS
F3_SOF equ $ ; IR_NEC.INC
IR_Init
F3_000003 equ $ ; IN [IR_NEC.INC] INPUT IR_IN
        bsf STATUS,5
ram_bank = 1
        bsf TRISC,3
F3_000008 equ $ ; IN [IR_NEC.INC] GOTO MAIN
        bcf STATUS,5
ram_bank = 0
        F@JUMP main
read_NEC
F3_000011 equ $ ; IN [IR_NEC.INC] GOSUB GET_ONTIME
        f@call get_ONtime
F3_000013 equ $ ; IN [IR_NEC.INC] IF _TIME > 1950 AND _TIME < 2050 THEN
        movf _timeH,W
        movwf PP0H
        movf _time,W
        movwf PP0
        movlw 7
        movwf PP1H
        movlw 158
        f@call T@GT
        movwf SP#P9
        movf _timeH,W
        movwf PP0H
        movf _time,W
        movwf PP0
        movlw 8
        movwf PP1H
        movlw 2
        f@call T@LT
        andwf SP#P9,F
        set@page BC@LL2
        btfsc STATUS,2
        goto BC@LL2
F3_000014 equ $ ; IN [IR_NEC.INC] GOSUB GET_OFFTIME
        f@call get_OFFtime
F3_000015 equ $ ; IN [IR_NEC.INC] ENDIF
BC@LL2
F3_000017 equ $ ; IN [IR_NEC.INC] IF _TIME > 950 AND _TIME < 980 THEN
        movf _timeH,W
        movwf PP0H
        movf _time,W
        movwf PP0
        movlw 3
        movwf PP1H
        movlw 182
        f@call T@GT
        movwf SP#P9
        movf _timeH,W
        movwf PP0H
        movf _time,W
        movwf PP0
        movlw 3
        movwf PP1H
        movlw 212
        f@call T@LT
        andwf SP#P9,F
        set@page BC@LL4
        btfsc STATUS,2
        goto BC@LL4
F3_000018 equ $ ; IN [IR_NEC.INC] FOR _COUNTER = 1 TO 16
        movlw 1
        movwf _counter
FR@LB6
        movlw 17
        subwf _counter,W
        set@page NX@LB7
        btfsc STATUS,0
        goto NX@LB7
F3_000019 equ $ ; IN [IR_NEC.INC] GOSUB GET_OFFTIME
        f@call get_OFFtime
CT@LB8
F3_000020 equ $ ; IN [IR_NEC.INC] NEXT
        movlw 1
        addwf _counter,F
        set@page FR@LB6
        btfss STATUS,0
        goto FR@LB6
NX@LB7
F3_000022 equ $ ; IN [IR_NEC.INC] FOR _COUNTER = 0 TO 7
        clrf _counter
FR@LB9
        movlw 8
        subwf _counter,W
        set@page NX@LB10
        btfsc STATUS,0
        goto NX@LB10
F3_000023 equ $ ; IN [IR_NEC.INC] GOSUB GET_OFFTIME
        f@call get_OFFtime
F3_000024 equ $ ; IN [IR_NEC.INC] IF _TIME > 350 AND _TIME < 370 THEN
        movf _timeH,W
        movwf PP0H
        movf _time,W
        movwf PP0
        movlw 1
        movwf PP1H
        movlw 94
        f@call T@GT
        movwf SP#P9
        movf _timeH,W
        movwf PP0H
        movf _time,W
        movwf PP0
        movlw 1
        movwf PP1H
        movlw 114
        f@call T@LT
        andwf SP#P9,F
        set@page BC@LL12
        btfsc STATUS,2
        goto BC@LL12
F3_000025 equ $ ; IN [IR_NEC.INC] _CMDBIN[_COUNTER] = 1
        movf _counter,W
        addlw _cmdBin
        movwf FSR
        movlw 1
        movwf INDF
        F@JUMP BC@LL13
BC@LL12
F3_000026 equ $ ; IN [IR_NEC.INC] ELSE
F3_000027 equ $ ; IN [IR_NEC.INC] _CMDBIN[_COUNTER] = 0
        movf _counter,W
        addlw _cmdBin
        movwf FSR
        movlw 0
        movwf INDF
F3_000028 equ $ ; IN [IR_NEC.INC] ENDIF
BC@LL13
CT@LB11
F3_000029 equ $ ; IN [IR_NEC.INC] NEXT
        movlw 1
        addwf _counter,F
        set@page FR@LB9
        btfss STATUS,0
        goto FR@LB9
NX@LB10
F3_000030 equ $ ; IN [IR_NEC.INC] COMMAND.0 = _CMDBIN[0]
        movf _cmdBin#0,W
        andlw 1
        btfsc STATUS,2
        bcf command,0
        btfss STATUS,2
        bsf command,0
F3_000031 equ $ ; IN [IR_NEC.INC] COMMAND.1 = _CMDBIN[1]
        movf _cmdBin#1,W
        andlw 1
        btfsc STATUS,2
        bcf command,1
        btfss STATUS,2
        bsf command,1
F3_000032 equ $ ; IN [IR_NEC.INC] COMMAND.2 = _CMDBIN[2]
        movf _cmdBin#2,W
        andlw 1
        btfsc STATUS,2
        bcf command,2
        btfss STATUS,2
        bsf command,2
F3_000033 equ $ ; IN [IR_NEC.INC] COMMAND.3 = _CMDBIN[3]
        movf _cmdBin#3,W
        andlw 1
        btfsc STATUS,2
        bcf command,3
        btfss STATUS,2
        bsf command,3
F3_000034 equ $ ; IN [IR_NEC.INC] COMMAND.4 = _CMDBIN[4]
        movf _cmdBin#4,W
        andlw 1
        btfsc STATUS,2
        bcf command,4
        btfss STATUS,2
        bsf command,4
F3_000035 equ $ ; IN [IR_NEC.INC] COMMAND.5 = _CMDBIN[5]
        movf _cmdBin#5,W
        andlw 1
        btfsc STATUS,2
        bcf command,5
        btfss STATUS,2
        bsf command,5
F3_000036 equ $ ; IN [IR_NEC.INC] COMMAND.6 = _CMDBIN[6]
        movf _cmdBin#6,W
        andlw 1
        btfsc STATUS,2
        bcf command,6
        btfss STATUS,2
        bsf command,6
F3_000037 equ $ ; IN [IR_NEC.INC] COMMAND.7 = _CMDBIN[7]
        movf _cmdBin#7,W
        andlw 1
        btfsc STATUS,2
        bcf command,7
        btfss STATUS,2
        bsf command,7
F3_000038 equ $ ; IN [IR_NEC.INC] HSEROUT["CMD: ", DEC COMMAND, 13]
        movlw 99
        f@call HRSOUT
        movlw 109
        f@call HRSOUT
        movlw 100
        f@call HRSOUT
        movlw 58
        f@call HRSOUT
        movlw 32
        f@call HRSOUT
        movlw 4
        movwf BPFH
        movf command,W
        f@call OUT@DECB
        movlw 13
        f@call HRSOUT
F3_000039 equ $ ; IN [IR_NEC.INC] ENDIF
BC@LL4
F3_000040 equ $ ; IN [IR_NEC.INC] RETURN
        return
get_OFFtime
F3_000043 equ $ ; IN [IR_NEC.INC] WHILE IR_IN = 1 : WEND
BC@LL14
        set@page BC@LL15
        btfss PORTC,3
        goto BC@LL15
ram_bank = 0
        F@JUMP BC@LL14
BC@LL15
F3_000044 equ $ ; IN [IR_NEC.INC] _TIME = 0
        clrf _timeH
        clrf _time
F3_000045 equ $ ; IN [IR_NEC.INC] WHILE IR_IN = 0
BC@LL16
        set@page BC@LL17
        btfsc PORTC,3
        goto BC@LL17
F3_000046 equ $ ; IN [IR_NEC.INC] INC _TIME
        incf _time,F
        btfsc STATUS,2
        incf _timeH,F
F3_000047 equ $ ; IN [IR_NEC.INC] WEND
        F@JUMP BC@LL16
BC@LL17
F3_000048 equ $ ; IN [IR_NEC.INC] RETURN
        return
get_ONtime
F3_000051 equ $ ; IN [IR_NEC.INC] WHILE IR_IN = 0 : WEND
BC@LL18
        set@page BC@LL19
        btfsc PORTC,3
        goto BC@LL19
ram_bank = 0
        F@JUMP BC@LL18
BC@LL19
F3_000052 equ $ ; IN [IR_NEC.INC] _TIME = 0
        clrf _timeH
        clrf _time
F3_000053 equ $ ; IN [IR_NEC.INC] WHILE IR_IN = 1
BC@LL20
        set@page BC@LL21
        btfss PORTC,3
        goto BC@LL21
F3_000054 equ $ ; IN [IR_NEC.INC] INC _TIME
        incf _time,F
        btfsc STATUS,2
        incf _timeH,F
F3_000055 equ $ ; IN [IR_NEC.INC] WEND
        F@JUMP BC@LL20
BC@LL21
F3_000056 equ $ ; IN [IR_NEC.INC] RETURN
        return
F3_EOF equ $ ; IR_NEC.INC
main
F1_000013 equ $ ; IN [IRDECODE.BAS] GOSUB READ_NEC
        f@call read_NEC
F1_000014 equ $ ; IN [IRDECODE.BAS] GOTO MAIN
        F@JUMP main
F1_EOF equ $ ; IRDECODE.BAS
PB@LB23
        F@JUMP PB@LB23
__EOF
__config FOSC_HS&WDTE_OFF&PWRTE_OFF&CP_ON&BOREN_OFF
        end
