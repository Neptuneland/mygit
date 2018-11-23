T_Msg_FU_TO_PL:
帧号：int32 frameID
时间戳：int32 syntime
收到数据包中的惯导号：int32 navID
融合后的栅格图：uint8[2000] gridMsk
融合后的障碍物占据的栅格号：int32[1000] pObs
融合后的障碍物实际数量：int32 nObs


订阅的话题：T_FU_TO_PL