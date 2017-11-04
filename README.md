# IR-NEC-decoder-ProtonIDE

This is just a demo project based on IR NEC protocol.

The source code is written in Proton Basic IDE.

How to use:
  1. simply just copy "IR_NEC.inc" file to your project folder.
  2. You MUST use 8MHz for this header file. Other oscillator might not work.
  3. Define "IR_in" to any I/O pin before including the header file.
  4. you will get pressed command in "command" variable.

<i>In this demo, calling "read_NEC" will transmit IR "command" data on UART. 
if you don't need it, just delete 38 no. line from "IR_NEC.inc" file.</i>
  
<b>Remember:</b> calling read_NEC will block your code.

Circuit Diagram:
<img src="https://github.com/AntorOfficial/IR-NEC-decoder-ProtonIDE/blob/master/diagram.PNG" width="350"/>
