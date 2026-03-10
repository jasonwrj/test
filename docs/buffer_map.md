# 1st.prg Buffer Map

> 说明：以下内容基于 `1st.prg` 中每个 `#<buffer>` 段落整理。每个 Buffer 给出：功能、入口条件、结束条件、关键变量修改、可能的报警/错误码。最后附 Buffer 间的 START/CALL/STOP 关系图。

---

## Buffer #0

**功能**：轴 0 换相与回零主流程（含限位检查与多种回零方式）。

**入口条件**
- 作为 Buffer 启动（`START 0,1` 或操作界面触发）。
- 若互锁满足（`iCar_Interlock_Cmd/iLl_Interlock_Cmd/iSt_Interlock_Cmd` 与 `iInterlockEnabled`）。

**结束条件**
- 正常完成回零后执行 `STOP`。
- 互锁触发直接跳转 `ENDOFHOMING` 后 `STOP`。

**关键变量修改**
- `iControlProcessState(0)`、`MFLAGS(0).#HOME`。
- `CERRI/CERRV`、`XCURI/XCURV`、`FPOS/TPOS`、`OldSP/Sp0`。

**可能的报警/错误码**
- `iControlProcessState=113`（互锁阻止）。
- `SP_Fail_Code` 在 `Limit_Check`/换相异常时置位。

---

## Buffer #1

**功能**：轴 1 换相与回零主流程；当 `iType=1040` 时进入 VAT 阀压力控制循环。

**入口条件**
- 作为 Buffer 启动（`START 1,1`）。
- 互锁允许且轴已可用。

**结束条件**
- 回零完成后 `STOP`；VAT 控制为常驻循环。

**关键变量修改**
- `iControlProcessState(1)`、`MFLAGS(1).#HOME`、`Sp1/OldSP(1)`。
- VAT 控制变量：`PressureSP1/PressurePV1/PressureWKSP1`、`ControlProcessState1`。

**可能的报警/错误码**
- `iControlProcessState=113`（互锁阻止）。
- VAT 控制内部逻辑可置状态码（如压力异常）。

---

## Buffer #2

**功能**：轴 2 换相与回零主流程（同 #0）。

**入口条件**：`START 2,1`，互锁允许。

**结束条件**：回零完成或互锁触发后 `STOP`。

**关键变量修改**：`iControlProcessState(2)`、`MFLAGS(2).#HOME`、`FPOS/TPOS`、`Sp2/OldSP(2)`。

**可能的报警/错误码**：`iControlProcessState=113`、`SP_Fail_Code`。

---

## Buffer #3

**功能**：轴 3 换相与回零主流程（同 #0）。

**入口条件**：`START 3,1`，互锁允许。

**结束条件**：回零完成或互锁触发后 `STOP`。

**关键变量修改**：`iControlProcessState(3)`、`MFLAGS(3).#HOME`、`FPOS/TPOS`、`Sp3/OldSP(3)`。

**可能的报警/错误码**：`iControlProcessState=113`、`SP_Fail_Code`。

---

## Buffer #4

**功能**：轴 4 换相与回零主流程（同 #0）。

**入口条件**：`START 4,1`，互锁允许。

**结束条件**：回零完成或互锁触发后 `STOP`。

**关键变量修改**：`iControlProcessState(4)`、`MFLAGS(4).#HOME`、`FPOS/TPOS`、`Sp4/OldSP(4)`。

**可能的报警/错误码**：`iControlProcessState=113`、`SP_Fail_Code`。

---

## Buffer #5

**功能**：轴 5 换相与回零主流程（同 #0）。

**入口条件**：`START 5,1`，互锁允许。

**结束条件**：回零完成或互锁触发后 `STOP`。

**关键变量修改**：`iControlProcessState(5)`、`MFLAGS(5).#HOME`、`FPOS/TPOS`、`Sp5/OldSP(5)`。

**可能的报警/错误码**：`iControlProcessState=113`、`SP_Fail_Code`。

---

## Buffer #6

**功能**：轴 6 换相与回零主流程（同 #0）。

**入口条件**：`START 6,1`，互锁允许。

**结束条件**：回零完成或互锁触发后 `STOP`。

**关键变量修改**：`iControlProcessState(6)`、`MFLAGS(6).#HOME`、`FPOS/TPOS`、`Sp6/OldSP(6)`。

**可能的报警/错误码**：`iControlProcessState=113`、`SP_Fail_Code`。

---

## Buffer #7

**功能**：轴 7 换相与回零主流程（同 #0）。

**入口条件**：`START 7,1`，互锁允许。

**结束条件**：回零完成或互锁触发后 `STOP`。

**关键变量修改**：`iControlProcessState(7)`、`MFLAGS(7).#HOME`、`FPOS/TPOS`、`Sp7/OldSP(7)`。

**可能的报警/错误码**：`iControlProcessState=113`、`SP_Fail_Code`。

---

## Buffer #11

**功能**：Meiden PLC（Keyence）Modbus 通讯与臭氧系统状态/命令同步。

**入口条件**：由主程序 `START 11,1` 启动。

**结束条件**：循环运行，异常才 `STOP`。

**关键变量修改**
- `POGON_CMD/POGON_CMD_RB`、`OZONEFEED_CMD/OZONEFEED_CMD_RB`。
- `OZONE_RDY/FEED_RDY/POG_NO_FLT/IN_REMOTE_MODE`。
- 多个安全报警位与 `NO_POGB_FLT_TO_CLOSE_ESV`。

**可能的报警/错误码**
- 通讯异常时 `ir_rspn` 无效（未显式设置错误码）。

---

## Buffer #12

**功能**：臭氧系统互锁与动态限位管理。

**入口条件**：由主程序 `START 12,1` 启动。

**结束条件**：循环运行，异常才 `STOP`。

**关键变量修改**
- `POGON_CMD/OZONEFEED_CMD/LC_BYPASSMODE_CMD/OXYGEN_BYPASS_CMD`。
- `M2G_InjClearOf*`、`iPreventMove(0)`。
- `adjNegativeLimit/adjPositiveLimit` 与 `DynNegativeLimit/DynPositiveLimit`。

**可能的报警/错误码**
- 互锁导致运动被阻止（`iPreventMove`）。

---

## Buffer #13

**功能**：与 Beckhoff CT 控制器通信，读/写 CARZ 状态。

**入口条件**：由主程序 `START 13,1` 启动。

**结束条件**：循环运行，异常才 `STOP`。

**关键变量修改**
- `C2M_OkToMoveCAR/C2M_CAR_UP/C2M_CAR_DOWN`。
- `CARZ_PM_StatusWord/CARZ_PM2_StatusWord`。

**可能的报警/错误码**
- 通讯失败未显式报警（依赖上层监测）。

---

## Buffer #14

**功能**：Wago MTC 互锁输入读取与本地升降/回零事件处理。

**入口条件**：主程序检测 MZ 轴存在时 `START 14,1`。

**结束条件**：循环运行，异常才 `STOP`。

**关键变量修改**
- `iLl_Interlock_Cmd/iSt_Interlock_Cmd/iCar_Interlock_Cmd`。
- `iLl_Stage_Up_Cmd/iLl_Stage_Down_Cmd/iSt_Stage_Up_Cmd/iSt_Stage_Down_Cmd`。
- `oLL_Door_Closed/oGmCarHome_Switch/oTIC_Local_Mode_Cmd`。

**可能的报警/错误码**
- 互锁触发导致 `iControlProcessState(axis)=113` 并 `KILL`。

---

## Buffer #15

**功能**：Wago GM 控制器通讯，回写 BFM/QCM 轴 AtHome 状态。

**入口条件**：主程序检测相关轴后 `START 15,1`。

**结束条件**：循环运行，异常才 `STOP`。

**关键变量修改**
- `BFM_StatusWord`、`Bit3/Bit11`。

**可能的报警/错误码**
- 无显式报警（通讯失败依赖上层）。

---

## Buffer #16

**功能**：Wago GM 控制器通讯，读取 QCM 互锁并回写 QCM 状态字。

**入口条件**：可由系统启动，但当前主流程使用 #17。

**结束条件**：循环运行，异常才 `STOP`。

**关键变量修改**
- `G2M_MaxSpGV/G2M_MaxSpCAR/G2M_MaxSpPosLim/G2M_SafeToMoveQcm`。
- `QCM_StatusWord`。

**可能的报警/错误码**
- 无显式报警。

---

## Buffer #17

**功能**：Beckhoff GM 通讯，读 QCM/Ozone/CARZ 状态并回写本地状态字与心跳。

**入口条件**：主程序按 QCM/臭氧/CARZ 需求 `START 17,1`。

**结束条件**：循环运行，异常才 `STOP`。

**关键变量修改**
- `G2M_*` 状态输入，`QCM_StatusWord/Ozn_StatusWord/CARZ_StatusWord` 输出。
- `M2G_HeartBeatON` 心跳翻转。

**可能的报警/错误码**
- 无显式报警。

---

## Buffer #18

**功能**：QCM 互锁逻辑，动态限位与状态反馈。

**入口条件**：主程序检测 `iQcmAxis` 后 `START 18,1`。

**结束条件**：循环运行，异常才 `STOP`。

**关键变量修改**
- `dQcmLimit(QcmAxis)`、`M2G_QcmLe*`、`M2G_QcmPresent`。
- `iPreventMove(QcmAxis)`。

**可能的报警/错误码**
- 互锁导致运动被阻止（`iPreventMove`）。

---

## Buffer #20

**功能**：CAR 轴 Home Pulse 输出与 Buffer #29 管理。

**入口条件**：主程序 `START 20,1`。

**结束条件**：循环运行或无 CAR 轴时 `STOP`。

**关键变量修改**
- `OUT(0).0/OUT(0).1/OUT(0).2`。
- `START/STOP 29`。

**可能的报警/错误码**
- 无显式报警。

---

## Buffer #22

**功能**：模拟输入指令处理（AVP 轴最多 4 轴）。

**入口条件**：主程序 `START 22,1` 且 `UseAnalogCmd(0)=1`。

**结束条件**：未启用时 `STOP`。

**关键变量修改**
- `Sp0..Sp3`（根据 `AIN` 与阈值更新）。

**可能的报警/错误码**
- 无显式报警。

---

## Buffer #23

**功能**：机器人自动回零流程（先伸缩 #30，再旋转 #28）。

**入口条件**：`START 23,1` 或 Buffer #31 回零请求。

**结束条件**
- 回零完成后 `STOP`。
- 超时（15min）则停止 #30/#28 并 `KILL`。

**关键变量修改**
- `RobotExtIsHoming/RobotRotIsHoming`。
- `FineRotHoming`。

**可能的报警/错误码**
- 超时触发中止（无显式错误码）。

---

## Buffer #24

**功能**：通用回零调度器（按 `SP_Axis` 触发 #0..#7）。

**入口条件**：上位机或脚本设定 `SP_Axis` 并 `START 24,1`。

**结束条件**：触发目标 Buffer 后 `STOP`。

**关键变量修改**
- 启动/停止指定 Buffer (`START/STOP 0..7`)。

**可能的报警/错误码**
- LL 门开则仅提示并不启动回零。

---

## Buffer #25

**功能**：机器人手臂轴换相启动程序。

**入口条件**：`START 25,1`（由 Buffer #30 触发）。

**结束条件**：换相完成后 `STOP`。

**关键变量修改**
- `MFLAGS(SP_Axis).9`、`SP_Fail_Code`、`SP_InCommutationStartup`。

**可能的报警/错误码**
- `SP_Fail_Code` 非 0 表示换相失败。

---

## Buffer #26

**功能**：机器人臂运动解释/执行与运动学计算。

**入口条件**：主程序在检测机器人类型后 `START 26,1`。

**结束条件**：循环运行，异常才 `STOP`。

**关键变量修改**
- `FRobot/Measured/WkSp`、`AxisInfo/AxisLog`。
- `iExtMoving/iRotMoving/ExecutingMove*`。

**可能的报警/错误码**
- `iControlProcessState` 被置为 102/108 等状态（限位/姿态不允许）。

---

## Buffer #27

**功能**：MAC 主程序初始化与系统启动调度。

**入口条件**：自动执行（`AUTOEXEC`）。

**结束条件**：常驻初始化完成后进入运行逻辑；异常才 `STOP`。

**关键变量修改**
- 读/写大量配置：`iType/d*`、`iHomingMethod`、`O3CollisionValues`。
- 启动各 Buffer：`11/12/13/14/15/17/18/20/22/26/31`。

**可能的报警/错误码**
- 初始化中若参数异常未显式报警（依赖上层监控）。

---

## Buffer #28

**功能**：机器人旋转回零。

**入口条件**：`START 28,1`（通常由 #23 触发）。

**结束条件**：回零完成后 `STOP`；超范围搜索会 `STOP 23`。

**关键变量修改**
- `RobotRotIsHoming/FineRotHoming`。
- `MFLAGS(ROBOTAXISNUML).#HOME`、`Sp0/Sp1/Sp2`。

**可能的报警/错误码**
- 发生异常搜索时置 `iControlProcessState=404`。

---

## Buffer #29

**功能**：CAR Home 开关滤波/模拟阈值处理。

**入口条件**：由 Buffer #20 `START 29,1`。

**结束条件**：循环运行，异常才 `STOP`。

**关键变量修改**
- `FtdCarHomeSW.*`、`AnalogCARAxis(*)`、`AnologHomeSignalMedian(*)`。

**可能的报警/错误码**
- 无显式报警。

---

## Buffer #30

**功能**：机器人伸缩回零（硬止挡/偏置回零）。

**入口条件**：`START 30,1`（通常由 #23 触发）。

**结束条件**：回零完成后 `STOP`。

**关键变量修改**
- `RobotExtIsHoming`、`MFLAGS(0).#HOME`。
- `FPOS/TPOS/RPOS/APOS`、`Sp0/Sp1/Sp2`。

**可能的报警/错误码**
- 回零前触发换相 `START 25`；异常未显式报警。

---

## Buffer #31

**功能**：非机器人轴的指令解释与执行（PTP/JOG/回零/定义原点）。

**入口条件**：主程序 `START 31,1`。

**结束条件**：循环运行；可触发 `STOP 0..7`/`STOP 23`。

**关键变量修改**
- `SP_Modbus/OldSP/OldRR/ExecutingMove`。
- `iControlProcessState`、`MFLAGS(*).#HOME`。
- 启动回零 `START 0..7` 或 `START 23`。

**可能的报警/错误码**
- `iControlProcessState=102`（超限）、`113`（互锁）。

---

## Buffer #A

**功能**：系统全局变量、轴映射与 Modbus tag 定义。

**入口条件**：随脚本加载。

**结束条件**：无（仅定义）。

**关键变量修改**：无运行态逻辑。

**可能的报警/错误码**：无。

---

# Buffer 关系图（START/CALL/STOP）

## START/STOP（Buffer 间）

- **#20**: `STOP 29`, `START 29`
- **#23**: `STOP 30`, `STOP 28`, `START 30`, `START 28`
- **#24**: `STOP/START 0..7`
- **#27**: `STOP j`（停止其他 buffer）, `STOP 30`, `STOP 28`, `STOP 23`, `STOP AxisNum`; `START 11/12/13/14/15/17/18/20/22/26/31`
- **#28**: `STOP 23`
- **#30**: `STOP(25)`, `START 25`
- **#31**: `STOP 23`, `STOP/START 0..7`, `START 23`

## CALL（同 Buffer 内子程序）

- **#0**: `Limit_Check`, `HOME_TO_TORQ_AVP`, `HOME_TO_TORQ`, `HOME_TO_SWITCH_CAR`, `HOME_TO_SWITCH_CARZ`, `HOME_TO_SWITCH`, `SET_CURRENTPOS_HOME`, `Finish`
- **#1**: `HOME_TO_TORQ_VLV`, `HOME_TO_TORQ_AVP`, `HOME_TO_TORQ`, `HOME_TO_SWITCH_CAR`, `HOME_TO_SWITCH_CARZ`, `HOME_TO_SWITCH`, `SET_CURRENTPOS_HOME`, `VAT_VLV_CONTROLLER`, `GETIGSIGNAL`, `ERRCALC`, `TrajPlanning`, `PIDLOOP`
- **#2/#3/#4/#5/#6/#7**: 同 #0
- **#25**: `Limit_Check`, `Finish`
- **#26**: `RobotCalculation`

