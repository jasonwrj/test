# 1st.prg Buffer 调用关系与功能/IO 摘要

> 说明：本文件依据 `1st.prg` 的 buffer 分段（#0, #1, ...）梳理。调用关系仅统计 `START/STOP/CALL/RESTART` 关键字（含带参数的 STOP/START）。

---

## Buffer #0

### 调用关系
- CALL: `Limit_Check`, `HOME_TO_TORQ_AVP`, `HOME_TO_TORQ`, `HOME_TO_SWITCH_CAR`, `HOME_TO_SWITCH_CARZ`, `HOME_TO_SWITCH`, `SET_CURRENTPOS_HOME`, `Finish`
- STOP: `self`
- START/RESTART: 无

### 功能摘要
轴 0 的电机换相与回零主流程：检查互锁、执行换相启动、根据回零方式执行力矩/开关/手动回零，并恢复限位/电流/误差门限与设定值。

### 输入输出变量表
| 类型 | 变量 | 说明 |
| --- | --- | --- |
| 输入 | `iType(0)`, `iHomingMethod(0)` | 轴类型与回零方式选择。 |
| 输入 | `iLl_Interlock_Cmd`, `iSt_Interlock_Cmd`, `iCar_Interlock_Cmd`, `iInterlockEnabled(0)` | 互锁与联锁使能。 |
| 输入 | `dHomingVel(0)`, `dHomingTorq(0)`, `dHomeOffset(0)`, `dAxisPitch(0)` | 回零速度/力矩/偏置/螺距。 |
| 输出 | `iControlProcessState(0)` | 控制流程状态（回零中/完成/报警）。 |
| 输出 | `MFLAGS(0).#HOME` | 回零完成标志。 |
| 输出 | `CERRI(0)`, `CERRV(0)` | 回零期间临时调整的 CPE 阈值。 |
| 输出 | `FPOS(0)`, `TPOS(0)` | 回零后位置清零与偏置。 |
| 输出 | `OldSP(0)`, `Sp0` | 回零完成后清零设定值。 |

---

## Buffer #1

### 调用关系
- CALL: `HOME_TO_TORQ_VLV`, `HOME_TO_TORQ_AVP`, `HOME_TO_TORQ`, `HOME_TO_SWITCH_CAR`, `HOME_TO_SWITCH_CARZ`, `HOME_TO_SWITCH`, `SET_CURRENTPOS_HOME`, `VAT_VLV_CONTROLLER`, `GETIGSIGNAL`, `ERRCALC`, `TrajPlanning`, `PIDLOOP`
- STOP: `self`
- START/RESTART: 无

### 功能摘要
轴 1 的换相与回零流程，与 #0 类似；当轴类型为 VAT 阀（如 `iType=1040`）时进入压力控制（PID/斜坡）逻辑。

### 输入输出变量表
| 类型 | 变量 | 说明 |
| --- | --- | --- |
| 输入 | `iType(1)`, `iHomingMethod(1)` | 轴类型与回零方式选择。 |
| 输入 | `dHomingVel(1)`, `dHomingTorq(1)`, `dHomeOffset(1)` | 回零参数。 |
| 输入 | `dValve_scaling_factor(1)`, `dPressureSP(1)`, `dPressureRamprate(1)` | VAT 压力控制参数。 |
| 输出 | `iControlProcessState(1)` | 控制流程状态。 |
| 输出 | `MFLAGS(1).#HOME` | 回零完成标志。 |
| 输出 | `Sp1`, `OldSP(1)` | 回零/控制循环后设定值。 |
| 输出 | `PressurePV1`, `PressureWKSP1`, `ControlProcessState1` | VAT 压力控制反馈与状态。 |

---

## Buffer #2

### 调用关系
- CALL: `Limit_Check`, `HOME_TO_TORQ_AVP`, `HOME_TO_TORQ`, `HOME_TO_SWITCH_CAR`, `HOME_TO_SWITCH_CARZ`, `HOME_TO_SWITCH`, `SET_CURRENTPOS_HOME`, `Finish`
- STOP: `self`
- START/RESTART: 无

### 功能摘要
轴 2 的换相与回零主流程，逻辑同 #0（只替换为轴 2）。

### 输入输出变量表
| 类型 | 变量 | 说明 |
| --- | --- | --- |
| 输入 | `iType(2)`, `iHomingMethod(2)` | 轴类型与回零方式。 |
| 输入 | `dHomingVel(2)`, `dHomingTorq(2)`, `dHomeOffset(2)` | 回零参数。 |
| 输出 | `iControlProcessState(2)`, `MFLAGS(2).#HOME` | 回零状态与完成标志。 |
| 输出 | `FPOS(2)`, `TPOS(2)`, `Sp2`, `OldSP(2)` | 回零后位置与设定值处理。 |

---

## Buffer #3

### 调用关系
- CALL: `Limit_Check`, `HOME_TO_TORQ_AVP`, `HOME_TO_TORQ`, `HOME_TO_SWITCH_CAR`, `HOME_TO_SWITCH_CARZ`, `HOME_TO_SWITCH`, `SET_CURRENTPOS_HOME`, `Finish`
- STOP: `self`
- START/RESTART: 无

### 功能摘要
轴 3 的换相与回零主流程，逻辑同 #0（只替换为轴 3）。

### 输入输出变量表
| 类型 | 变量 | 说明 |
| --- | --- | --- |
| 输入 | `iType(3)`, `iHomingMethod(3)` | 轴类型与回零方式。 |
| 输入 | `dHomingVel(3)`, `dHomingTorq(3)`, `dHomeOffset(3)` | 回零参数。 |
| 输出 | `iControlProcessState(3)`, `MFLAGS(3).#HOME` | 回零状态与完成标志。 |
| 输出 | `FPOS(3)`, `TPOS(3)`, `Sp3`, `OldSP(3)` | 回零后位置与设定值处理。 |

---

## Buffer #4

### 调用关系
- CALL: `Limit_Check`, `HOME_TO_TORQ_AVP`, `HOME_TO_TORQ`, `HOME_TO_SWITCH_CAR`, `HOME_TO_SWITCH_CARZ`, `HOME_TO_SWITCH`, `SET_CURRENTPOS_HOME`, `Finish`
- STOP: `self`
- START/RESTART: 无

### 功能摘要
轴 4 的换相与回零主流程，逻辑同 #0（只替换为轴 4）。

### 输入输出变量表
| 类型 | 变量 | 说明 |
| --- | --- | --- |
| 输入 | `iType(4)`, `iHomingMethod(4)` | 轴类型与回零方式。 |
| 输入 | `dHomingVel(4)`, `dHomingTorq(4)`, `dHomeOffset(4)` | 回零参数。 |
| 输出 | `iControlProcessState(4)`, `MFLAGS(4).#HOME` | 回零状态与完成标志。 |
| 输出 | `FPOS(4)`, `TPOS(4)`, `Sp4`, `OldSP(4)` | 回零后位置与设定值处理。 |

---

## Buffer #5

### 调用关系
- CALL: `Limit_Check`, `HOME_TO_TORQ_AVP`, `HOME_TO_TORQ`, `HOME_TO_SWITCH_CAR`, `HOME_TO_SWITCH_CARZ`, `HOME_TO_SWITCH`, `SET_CURRENTPOS_HOME`, `Finish`
- STOP: `self`
- START/RESTART: 无

### 功能摘要
轴 5 的换相与回零主流程，逻辑同 #0（只替换为轴 5）。

### 输入输出变量表
| 类型 | 变量 | 说明 |
| --- | --- | --- |
| 输入 | `iType(5)`, `iHomingMethod(5)` | 轴类型与回零方式。 |
| 输入 | `dHomingVel(5)`, `dHomingTorq(5)`, `dHomeOffset(5)` | 回零参数。 |
| 输出 | `iControlProcessState(5)`, `MFLAGS(5).#HOME` | 回零状态与完成标志。 |
| 输出 | `FPOS(5)`, `TPOS(5)`, `Sp5`, `OldSP(5)` | 回零后位置与设定值处理。 |

---

## Buffer #6

### 调用关系
- CALL: `Limit_Check`, `HOME_TO_TORQ_AVP`, `HOME_TO_TORQ`, `HOME_TO_SWITCH_CAR`, `HOME_TO_SWITCH_CARZ`, `HOME_TO_SWITCH`, `SET_CURRENTPOS_HOME`, `Finish`
- STOP: `self`
- START/RESTART: 无

### 功能摘要
轴 6 的换相与回零主流程，逻辑同 #0（只替换为轴 6）。

### 输入输出变量表
| 类型 | 变量 | 说明 |
| --- | --- | --- |
| 输入 | `iType(6)`, `iHomingMethod(6)` | 轴类型与回零方式。 |
| 输入 | `dHomingVel(6)`, `dHomingTorq(6)`, `dHomeOffset(6)` | 回零参数。 |
| 输出 | `iControlProcessState(6)`, `MFLAGS(6).#HOME` | 回零状态与完成标志。 |
| 输出 | `FPOS(6)`, `TPOS(6)`, `Sp6`, `OldSP(6)` | 回零后位置与设定值处理。 |

---

## Buffer #7

### 调用关系
- CALL: `Limit_Check`, `HOME_TO_TORQ_AVP`, `HOME_TO_TORQ`, `HOME_TO_SWITCH_CAR`, `HOME_TO_SWITCH_CARZ`, `HOME_TO_SWITCH`, `SET_CURRENTPOS_HOME`, `Finish`
- STOP: `self`
- START/RESTART: 无

### 功能摘要
轴 7 的换相与回零主流程，逻辑同 #0（只替换为轴 7）。

### 输入输出变量表
| 类型 | 变量 | 说明 |
| --- | --- | --- |
| 输入 | `iType(7)`, `iHomingMethod(7)` | 轴类型与回零方式。 |
| 输入 | `dHomingVel(7)`, `dHomingTorq(7)`, `dHomeOffset(7)` | 回零参数。 |
| 输出 | `iControlProcessState(7)`, `MFLAGS(7).#HOME` | 回零状态与完成标志。 |
| 输出 | `FPOS(7)`, `TPOS(7)`, `Sp7`, `OldSP(7)` | 回零后位置与设定值处理。 |

---

## Buffer #11

### 调用关系
- STOP: `self`
- START/CALL/RESTART: 无

### 功能摘要
通过 Modbus 与 Meiden PLC（Keyence）通信：周期性读取臭氧系统状态与报警，并在远程模式下写入命令与设定值。

### 输入输出变量表
| 类型 | 变量 | 说明 |
| --- | --- | --- |
| 输入 | `LC_MFC_SP`, `LC_BYPASSMODE_CMD`, `OXYGEN_BYPASS_CMD` | Molly/本地控制发往 PLC 的设定与命令。 |
| 输入 | `OZONEPRESSURE_SP`, `OZONECHAMBERPRESSURE_SP`, `CHARGEVOLUME_SP`, `POGON_CMD`, `OZONEFEED_CMD` | 臭氧相关设定与命令。 |
| 输出 | `FEED_FLOW_MODE`, `OZONE_RDY`, `FEED_RDY`, `POG_NO_FLT`, `IN_REMOTE_MODE` | PLC 状态与远程模式标志。 |
| 输出 | `POGON_CMD_RB`, `OZONEFEED_CMD_RB`, `LC_BYPASSMODE_CMD_RB`, `OXYGEN_BYPASS_CMD_RB` | 命令回读与状态合成。 |
| 输出 | `NO_POGB_FLT_TO_CLOSE_ESV` 及报警位 | 臭氧安全报警/联锁输出。 |

---

## Buffer #12

### 调用关系
- STOP: `self`
- START/CALL/RESTART: 无

### 功能摘要
臭氧系统互锁逻辑：在 Molly/Meiden 命令之间做互锁判断，维护喷嘴/主闸/转运等碰撞区域的动态限位，并向 GM 控制器输出状态。

### 输入输出变量表
| 类型 | 变量 | 说明 |
| --- | --- | --- |
| 输入 | `CommandWord`, `OzonePresureSP`, `OzoneChamberSP`, `ChargeVolumeSP`, `LCOzoneFlowRateSP` | Molly 侧命令与设定值。 |
| 输入 | `G2M_OpenESV_Status`, `G2M_SafeToMoveInj`, `G2M_MainShutterOpen`, `G2M_BfmRetracted`, `G2M_QcmLeCAR`, `G2M_PrepareForTransfer` | GM 互锁状态。 |
| 输出 | `POGON_CMD`, `OZONEFEED_CMD`, `LC_BYPASSMODE_CMD`, `OXYGEN_BYPASS_CMD` | 转发到 Buffer 11 的命令/设定。 |
| 输出 | `M2G_InjClearOfMainShutter`, `M2G_InjClearOfBFM`, `M2G_InjClearOfQCM`, `M2G_InjClearOfRobot` | 喷嘴避让状态输出。 |
| 输出 | `adjNegativeLimit(0)`, `adjPositiveLimit(0)`, `DynNegativeLimit`, `DynPositiveLimit` | 动态限位与发布给 Molly 的限制。 |
| 输出 | `iPreventMove(0)` | 对臭氧喷嘴运动的互锁阻止。 |

---

## Buffer #13

### 调用关系
- STOP: `self`
- START/CALL/RESTART: 无

### 功能摘要
与 Beckhoff CT 控制器进行 Modbus 通信：读取 CAR Z 许可/方向信号并回写 CAR Z 状态。

### 输入输出变量表
| 类型 | 变量 | 说明 |
| --- | --- | --- |
| 输入 | `M2C_CARZ`, `M2C_CARZ2`, `CARZ_PM_AxisNum`, `CARZ_PM2_AxisNum` | 是否启用 CAR Z 通信及轴号。 |
| 输入 | `SAFIN(...)`, `tmpMeasured(...)`, `MST(...)` | 轴位置/运动状态。 |
| 输出 | `C2M_OkToMoveCAR`, `C2M_CAR_UP`, `C2M_CAR_DOWN` | 从 CT 读回的控制信号。 |
| 输出 | `CARZ_PM_StatusWord`, `CARZ_PM2_StatusWord` | 写回 CT 的 CAR Z 状态字。 |

---

## Buffer #14

### 调用关系
- STOP: `self`
- START/CALL/RESTART: 无

### 功能摘要
与 Wago MTC 控制器通讯，读取 LL/ST/CAR 互锁与门/本地模式输入，同时驱动本地升降/回零动作的事件处理。

### 输入输出变量表
| 类型 | 变量 | 说明 |
| --- | --- | --- |
| 输入 | `MZ_CARAxisNum`, `MZ_LLAxisNum`, `MZ_STAxisNum` | MZ 轴号配置。 |
| 输入 | `ir_rspn(...)` | Modbus 读取的状态字。 |
| 输出 | `oLL_Door_Closed`, `oGmCarHome_Switch`, `oTIC_Local_Mode_Cmd` | 门/本地模式与开关输入。 |
| 输出 | `iLl_Interlock_Cmd`, `iSt_Interlock_Cmd`, `iCar_Interlock_Cmd` | 互锁输出。 |
| 输出 | `iLl_Stage_Up_Cmd`, `iLl_Stage_Down_Cmd`, `iSt_Stage_Up_Cmd`, `iSt_Stage_Down_Cmd`, `iCar_Stage_Home_Cmd` | 运动命令输出与事件触发。 |

---

## Buffer #15

### 调用关系
- STOP: `self`
- START/CALL/RESTART: 无

### 功能摘要
与 Wago GM 控制器通信，按 BFM/QCM 轴是否到位生成状态字并写回 GM。

### 输入输出变量表
| 类型 | 变量 | 说明 |
| --- | --- | --- |
| 输入 | `HW_Intlkd_BFMQCM_AxisNum1`, `HW_Intlkd_BFMQCM_AxisNum2` | 需要反馈的轴号。 |
| 输入 | `Measured*`, `MFLAGS(...).#HOME` | 位置与回零状态判断。 |
| 输出 | `BFM_StatusWord` | 写回 GM 的状态字（含 AtHome 位）。 |

---

## Buffer #16

### 调用关系
- STOP: `self`
- START/CALL/RESTART: 无

### 功能摘要
与 Wago GM 控制器通信：读取 QCM 互锁状态并写回本地 QCM 状态字。

### 输入输出变量表
| 类型 | 变量 | 说明 |
| --- | --- | --- |
| 输入 | `ir_rspn(...)` | GM 侧返回状态字。 |
| 输出 | `G2M_MaxSpGV`, `G2M_MaxSpCAR`, `G2M_MaxSpPosLim`, `G2M_SafeToMoveQcm` | QCM 互锁输入。 |
| 输出 | `QCM_StatusWord` | 写回 GM 的 QCM 状态字。 |

---

## Buffer #17

### 调用关系
- STOP: `self`
- START/CALL/RESTART: 无

### 功能摘要
与 Beckhoff GM 控制器通讯：读取 QCM/Ozone/CARZ 状态字并回写 MAC 侧状态字，同时维护心跳信号。

### 输入输出变量表
| 类型 | 变量 | 说明 |
| --- | --- | --- |
| 输入 | `M2G_QcmPresent`, `M2G_InjPresent`, `M2G_CARZ` | 启用哪些通道的读写。 |
| 输入 | `ir_rspn_Q/O/Z(...)` | GM 读取的状态字。 |
| 输出 | `G2M_MaxSpGV`, `G2M_MaxSpCAR`, `G2M_MaxSpPosLim`, `G2M_SafeToMoveQcm`, `G2M_InjClearOfRobot` | QCM/喷嘴互锁输入。 |
| 输出 | `G2M_OpenESV_Status`, `G2M_LeakValveAllowedOpen`, `G2M_SafeToMoveInj`, `G2M_MainShutterOpen`, `G2M_BfmRetracted`, `G2M_QcmLeCAR`, `G2M_PrepareForTransfer` | Ozone 互锁输入。 |
| 输出 | `G2M_OkToMoveCAR`, `G2M_CAR_UP`, `G2M_CAR_DOWN` | CAR Z 状态输入。 |
| 输出 | `QCM_StatusWord`, `Ozn_StatusWord`, `CARZ_StatusWord` | 写回 GM 的状态字。 |

---

## Buffer #18

### 调用关系
- STOP: `self`
- START/CALL/RESTART: 无

### 功能摘要
QCM 互锁管理：根据 GM 互锁标志动态调整 QCM 限位与状态，必要时回收设定值。

### 输入输出变量表
| 类型 | 变量 | 说明 |
| --- | --- | --- |
| 输入 | `G2M_MaxSpGV`, `G2M_MaxSpCAR`, `G2M_MaxSpPosLim`, `G2M_SafeToMoveQcm` | GM 互锁条件。 |
| 输入 | `Measured*`, `SP_Modbus*`, `dInterlockPos`, `dSecondSetpointPos` | 位置/设定值/限位。 |
| 输出 | `dQcmLimit(*)` | QCM 动态限位。 |
| 输出 | `M2G_QcmLeGV`, `M2G_QcmLeCAR`, `M2G_QcmGtCAR`, `M2G_QcmAtHome`, `M2G_QcmNotMoving`, `M2G_QcmPresent` | 反馈给 GM 的状态。 |
| 输出 | `iPreventMove(QcmAxis)` | 运动互锁输出。 |

---

## Buffer #20

### 调用关系
- STOP: `29`（停用 CAR home filter）
- START: `29`（启用 CAR home filter）
- STOP: `self`
- CALL/RESTART: 无

### 功能摘要
生成 CAR 轴的 Home Pulse 输出，并在适用机型上启动 Buffer 29 进行 CAR Home 开关滤波。

### 输入输出变量表
| 类型 | 变量 | 说明 |
| --- | --- | --- |
| 输入 | `iType(*)`, `dHomePulseDelay(*)`, `dHomePulseWidth(*)` | CAR 轴识别与脉冲参数。 |
| 输入 | `FPOS(*)`, `MST(*)` | 用于生成位置脉冲。 |
| 输出 | `OUT(0).0`, `OUT(0).1`, `OUT(0).2` | Home Pulse 与编码器位置输出。 |

---

## Buffer #22

### 调用关系
- STOP: `self`
- START/CALL/RESTART: 无

### 功能摘要
使用模拟输入对 AVP 轴（最多 4 轴）进行设定值更新，带滤波与阈值窗口。

### 输入输出变量表
| 类型 | 变量 | 说明 |
| --- | --- | --- |
| 输入 | `UseAnalogCmd(*)`, `AIN(0..3)` | 模拟指令开关、阈值、窗口与采样。 |
| 输入 | `dPositiveLimit(*)`, `AnalogChannelOffset(*)`, `iAxisGood(*)` | 设定值缩放与有效性判断。 |
| 输出 | `Sp0`, `Sp1`, `Sp2`, `Sp3` | 模拟输入转换后的设定值。 |

---

## Buffer #23

### 调用关系
- STOP: `30`, `28`, `self`
- START: `30`, `28`
- CALL/RESTART: 无

### 功能摘要
机器人自动回零流程：按顺序启动伸缩与旋转回零（Buffer 30/28），并监控超时与中止。

### 输入输出变量表
| 类型 | 变量 | 说明 |
| --- | --- | --- |
| 输入 | `RobotExtIsHoming`, `RobotRotIsHoming` | 回零进行中标志。 |
| 输出 | `FineRotHoming` | 旋转细分回零标志。 |
| 输出 | `KILL(ROBOTAXISNUMH, ROBOTAXISNUML)` | 超时或中止时强制停止运动。 |

---

## Buffer #24

### 调用关系
- STOP: `0..7`, `self`
- START: `0..7`
- CALL/RESTART: 无

### 功能摘要
按 `SP_Axis` 选择并启动对应轴的回零缓冲（#0~#7），并在 LL 门打开时阻止回零。

### 输入输出变量表
| 类型 | 变量 | 说明 |
| --- | --- | --- |
| 输入 | `SP_Axis`, `IN(0).5`, `bFoundLLELE`, `LLELEAXIS` | 回零轴选择与门互锁判断。 |
| 输出 | `START/STOP #0..#7` | 启动对应轴回零 buffer。 |

---

## Buffer #25

### 调用关系
- CALL: `Limit_Check`, `Finish`
- STOP: `self`
- START/RESTART: 无

### 功能摘要
机器人手臂轴换相启动程序：执行 detent 查找与限位规避，完成换相并更新状态。

### 输入输出变量表
| 类型 | 变量 | 说明 |
| --- | --- | --- |
| 输入 | `SP_Axis`, `SLCPRD`, `SLCNP`, `EFAC`, `MERR`, `FAULT` | 换相与限位检测参数。 |
| 输出 | `MFLAGS(SP_Axis).9` | 换相完成标志。 |
| 输出 | `SP_Fail_Code`, `SP_InCommutationStartup` | 换相状态与失败码。 |

---

## Buffer #26

### 调用关系
- CALL: `RobotCalculation`
- STOP: `self`
- START/RESTART: 无

### 功能摘要
机器人手臂（伸缩+旋转）运动解释与执行：计算几何关系、下发 PTP/速度指令并刷新机器人与轴状态到 GUI。

### 输入输出变量表
| 类型 | 变量 | 说明 |
| --- | --- | --- |
| 输入 | `Sp0`, `Sp1/Sp2`, `ramprate*`, `iType(*)` | 目标指令与速度。 |
| 输入 | `rRearArmLength`, `rFrontArmLength`, `rEndEffectorLength`, `rWristHalfWidth`, `rClearance` | 机器人几何参数。 |
| 输出 | `FRobot(*)`, `Measured*`, `WkSp*` | 机器人坐标与反馈。 |
| 输出 | `AxisInfo`, `AxisLog`, `iAxisGood(*)` | GUI/健康状态输出。 |
| 输出 | `iExtMoving`, `iRotMoving`, `ExecutingMoveExt/Rot` | 运动过程状态。 |

---

## Buffer #27

### 调用关系
- STOP: `j`（循环停止其他 buffer）, `30`, `28`, `23`, `AxisNum`, `self`
- START: `15`, `14`, `20`, `22`, `31`, `26`, `17`, `18`, `13`, `11`, `12`
- CALL/RESTART: 无

### 功能摘要
MAC 主程序：初始化参数、读取持久化设置、分配轴类型/限位/速度，并启动各子 buffer（Modbus、互锁、机器人、QCM、臭氧等）。

### 输入输出变量表
| 类型 | 变量 | 说明 |
| --- | --- | --- |
| 输入 | `READ ...` 持久化参数 | 轴类型、限位、速度、回零、互锁、臭氧碰撞区域等配置。 |
| 输入 | `SYSINFO`, `ECGETPID`, `IN(0).*` | 控制器信息与 IO 输入。 |
| 输出 | `MZ_*AxisNum`, `M2G_*`, `M2C_*` | 轴识别与互锁通道启用标志。 |
| 输出 | `VEL/XVEL/ACC/DEC/KDEC`, `FMASK` | 轴参数初始化。 |
| 输出 | `Start ...` | 启动各功能 buffer。 |

---

## Buffer #28

### 调用关系
- STOP: `23`, `self`
- START/CALL/RESTART: 无

### 功能摘要
机器人旋转回零：搜索回零开关、精细定位、设置 HOME 标志并恢复电流/误差门限。

### 输入输出变量表
| 类型 | 变量 | 说明 |
| --- | --- | --- |
| 输入 | `iHomingMethod`, `IN0.0`, `dHomingVel`, `dHomeOffset` | 回零方式/开关与参数。 |
| 输入 | `RealRobotAngle`, `MFLAGS`, `XCURI/XCURV` | 回零状态与电流门限。 |
| 输出 | `MFLAGS(ROBOTAXISNUML).#HOME`, `RobotRotIsHoming` | 回零完成与标志。 |
| 输出 | `Sp0`, `Sp1/Sp2`, `RobotROTTargetPrev`, `RobotExtTargetPrev` | 设定值清零与目标复位。 |

---

## Buffer #29

### 调用关系
- STOP: `self`
- START/CALL/RESTART: 无

### 功能摘要
CAR Home 开关滤波/整形：根据数字/模拟输入生成 `FtdCarHomeSW`，并维护模拟 CAR 轴的中值阈值。

### 输入输出变量表
| 类型 | 变量 | 说明 |
| --- | --- | --- |
| 输入 | `IN(0).*`, `AIN(0..3)`, `iType(*)` | Home 开关与模拟信号。 |
| 输出 | `FtdCarHomeSW.*` | 经过滤波后的 Home 开关状态。 |
| 输出 | `AnalogCARAxis(*)`, `AnologHomeSignalMedian(*)` | 模拟 CAR 轴映射与阈值。 |

---

## Buffer #30

### 调用关系
- STOP: `(25)`, `self`
- START: `25`
- CALL/RESTART: 无

### 功能摘要
机器人伸缩轴回零（硬止挡/偏置方式）：必要时先调用换相（#25），执行回零动作并恢复电流/误差门限。

### 输入输出变量表
| 类型 | 变量 | 说明 |
| --- | --- | --- |
| 输入 | `iType(0)`, `iHomingMethod(0)`, `dHomingVel(0)`, `dHomingTorq(0)`, `dHomeOffset(0)` | 回零条件与参数。 |
| 输入 | `ROBOTAXISNUML`, `FlipperClampDTI`, `theta0` | 机器人几何与轴号。 |
| 输出 | `MFLAGS(0).#HOME`, `RobotExtIsHoming` | 回零完成状态。 |
| 输出 | `FPOS/TPOS/RPOS/APOS`, `Sp0`, `Sp1/Sp2` | 回零后位置与设定值清零。 |
| 输出 | `iControlProcessState(0)` | 回零状态更新。 |

---

## Buffer #31

### 调用关系
- STOP: `23`, `0..7`, `self`
- START: `23`, `0..7`
- CALL/RESTART: 无

### 功能摘要
轴运动指令解释与执行：解析 Modbus 指令、校验限位/互锁条件，发起 PTP/JOG，维护回零/定义原点请求，并转发同轴回零请求到对应 buffer。

### 输入输出变量表
| 类型 | 变量 | 说明 |
| --- | --- | --- |
| 输入 | `Sp0..Sp7`, `ramprate0..7` | 运动指令与速度。 |
| 输入 | `iType(*)`, `dPositiveLimit`, `dNegtiveLimit`, `dQcmLimit`, `adjNegativeLimit`, `adjPositiveLimit` | 轴类型与限位。 |
| 输入 | `iPreventMove(*)`, `iInterlockEnabled(*)`, `bAxisInterlocked`, `iLl_Interlock_Cmd`, `iSt_Interlock_Cmd`, `iCar_Interlock_Cmd` | 互锁与安全条件。 |
| 输出 | `SP_Modbus(*)`, `OldSP(*)`, `OldRR(*)`, `ExecutingMove(*)` | 指令落地与执行状态。 |
| 输出 | `iControlProcessState(*)`, `MFLAGS(*).#HOME` | 流程状态与回零/定义原点处理。 |
| 输出 | `START/STOP #0..#7/#23` | 触发对应 buffer 回零流程。 |

---

## Buffer #A

### 调用关系
- 无（仅定义与映射）

### 功能摘要
定义轴映射、全局变量与 Modbus 映射表，是 MAC/Molly 与控制器通信的数据字典。

### 输入输出变量表
| 类型 | 变量 | 说明 |
| --- | --- | --- |
| 输出 | `axisdef ...` | 轴命名与索引映射。 |
| 输出 | `Global ...` | 全局变量与 Modbus tag 定义（如 `Sp*`, `Measured*`, `ControlProcessState*` 等）。 |

