
IR_Init:
  Input IR_in
  Dim _time As Word
  Dim _counter As Byte 
  Dim _cmdBin[8] As Byte
  Dim command As Byte
GoTo main

read_NEC:
  GoSub get_ONtime
  
  If _time > 1950 And _time < 2050 Then
    GoSub get_OFFtime
  EndIf
  
  If _time > 950 And _time < 980 Then
    For _counter = 1 To 16
      GoSub get_OFFtime
    Next
    
    For _counter = 0 To 7
      GoSub get_OFFtime
      If _time > 350 And _time < 370 Then 
        _cmdBin[_counter] = 1
      Else
        _cmdBin[_counter] = 0
      EndIf
    Next
    command.0 = _cmdBin[0]
    command.1 = _cmdBin[1]
    command.2 = _cmdBin[2]
    command.3 = _cmdBin[3]
    command.4 = _cmdBin[4]
    command.5 = _cmdBin[5]
    command.6 = _cmdBin[6]
    command.7 = _cmdBin[7]
    HSerOut ["cmd: ", Dec command, 13]    
  EndIf
Return

get_OFFtime:
  While IR_in = 1 : Wend
  _time = 0
  While IR_in = 0 
    Inc _time
  Wend
Return

get_ONtime:
  While IR_in = 0 : Wend
  _time = 0
  While IR_in = 1
    Inc _time
  Wend
Return

