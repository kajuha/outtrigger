1. 메시지 종류
  가. 서비스
    1) 정지
    2) 위치제어
      가) 위치(mm)
      나) 속도(mm/s)
      다) 타입(상대/절대)
    3) 호밍
      가) 일반 호밍
        1) 호밍이 되어 있지 않으면 호밍
        2) 호밍이 되어 있으면 하지 않음
      나) 강제 호밍
        1) 호밍상태와 관계없이 강제 호밍
  나. 토픽
    1) 호밍상태
    2) 위치(mm)
    3) 속도(mm/s)
    4) IO 상태

2. 로직
  가. 정지(PID : PID_COM_WATCH_DELAY(#185), PID_TQ_OFF(#5), PID_BRAKE(#6), PID_STOP_STATUS(#24))
    1) 정지 명령 송신
  나. 위치제어(PID : PID_IN_POSITION_OK(#49, PID_POSI_SS(#178), PID_POSI_SD(#179), PID_PV_GAIN(#167), PID_P_GAIN(#168), PID_V_GAIN(#169), PID_GAIN(#203), PID_PNT_POSI_VEL_CMD(#206), PID_PNT_VEL_CMD(#207)))
    1) 호밍 여부 확인
    2) 제어 명령 송신
  다. 호밍(PID : PID_INIT_SET(#35), PID_POSI_RESET(#13), PID_POSI_SET(#217))
    1) 토크호밍(또는 위치제어) 명령 송신
    2) 호밍(rpm변경)이 되는지 확인
      가) 여부에 따라서 진행
    3) 리밋센서 확인
      가) 여부에 따라서 진행
    4) 위치를 0으로 리셋(PID : )
  라. 상태(PID : )
    1) 1초에 한 번씩 IO 상태확인(PID : )
    2) 1초에 한 번씩 제어 상태확인(PID : )

3. PID 확인
  x PID_VER
  x PID_IO_MONITOR
  PID_COM_WATCH_DELAY(#185)
  x PID_TQ_OFF(#5)
  x PID_BRAKE(#6)
  PID_STOP_STATUS(#24)
  x PID_IN_POSITION_OK(#49)
  PID_POSI_SS(#178)
  PID_POSI_SD(#179)
  PID_PV_GAIN(#167)
  PID_P_GAIN(#168)
  PID_V_GAIN(#169)
  PID_GAIN(#203)
  PID_PNT_POSI_VEL_CMD(#206)
  PID_PNT_VEL_CMD(#207)
  PID_INIT_SET(#35)
  PID_POSI_RESET(#13)
  PID_POSI_SET(#217)