Device = 16F73
Config FOSC_HS, WDTE_OFF, PWRTE_OFF, CP_ON, BOREN_OFF
Xtal = 8
Declare Hserial_Baud = 9600     
Declare Hserial_RCSTA = %10010000
Declare Hserial_TXSTA = %00100100
Declare Hserial_Clear = On

Symbol IR_in = PORTC.3
Include "IR_NEC.inc"

main:
  GoSub read_NEC
GoTo main
