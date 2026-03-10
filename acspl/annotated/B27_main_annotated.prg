#25

! 教学式注释: Buffer #25 机器人轴换相启动程序
! 目的: 对机器人相关轴执行换相(Commutation Startup), 在 detent 点间跳转并确认电机状态。
! 输入条件: 由上层Buffer(例如#30)通过 START 25,1 启动; 轴硬件使能且无致命故障。
! 输出效果: 成功时置 MFLAGS(SP_Axis).9=1; 失败时设置 SP_Fail_Code 并结束。
!robot arm motors cummutation ini program

!Change log
! DATE		ECO#	Reason for Change			Remarks
!9/21/2015	N/A		ini release					ZW

int  SP_Axis                      ! Axis to be commutated
real SP_Direction                 ! Search direction 
real SP_Settle_Time               ! Settling time at Detent Points [msec]
real SP_Search_Vel                ! Search velocity [user-units/sec]
real SP_Drive                     ! Actuating drive command [% of maximum]
real SP_Max_Search                ! Maximum search distance [user-units]
real SP_Init_Offset               ! Initial commutation offset [elec. degrees]

SP_Axis =0          
SP_Drive = 12.5          
SP_Settle_Time = 1000    
SP_Search_Vel  = 20         
SP_Init_Offset = 0
SP_Direction =  -1
SP_Max_Search = 720

!   Output varibale:
int  SP_Fail_Code                 ! Faiure Code of the startup program

! Auxiliary variable:
real SP_Pitch                     ! Magnetic pitch [180 elec.deg. in user units]

! State Flag
int  SP_InCommutationStartup      ! Flag indicating commutation startup is in progress


! ---- 主流程循环 ----
! 目的: 依次对两根机器人轴执行换相参数设置与detent移动。
! 输入条件: SP_Axis 初始为0, 第二次循环切换为 ROBOTAXISNUML。
! 输出效果: 两轴换相状态更新。
LOOP 2


SP_Pitch=SLCPRD(SP_Axis)/SLCNP(SP_Axis)*EFAC(SP_Axis)

!******************************************************************************* 
! 目的: 清故障、禁用轴、重置换相状态并写入初始相位。
! INITIALIZE
FCLEAR(SP_Axis)   
disable(SP_Axis)

SETCONF(216,SP_Axis,0)                ! Reset commutation state
setconf(214,SP_Axis,SP_Init_Offset)   ! Set initial commutation phase
SP_Fail_Code=0                        ! Reset failure
SP_Direction=1                        ! Move in positive direction
SP_InCommutationStartup=1             ! commutation startup in progress
!******************************************************************************* 
! SET MOTION PROFILE FOR COMMUTATION STARTUP PROCESS 
!
!   WARNING: The following are the suggested motion parameters for the startup
!     process. Check that the values are suitable for your application.

!ACC(SP_Axis)=SP_Search_Vel*10.; DEC(SP_Axis)=SP_Search_Vel*10. 
!KDEC(SP_Axis)=SP_Search_Vel*50.; JERK(SP_Axis)=SP_Search_Vel*100.

!******************************************************************************* 
! STEP 1 - MOVE TO FIRST DETENT POINT 
!
! WARNING: The motor moves to a detent point by jump. 
!   The jump distance is up to one magnetic pitch in any direction.
!   The motor jumps to the closest detent point within its motion range.
!   If necessary modify initial detent point by changing the variable 
!     SP_Init_Offset between 0-360 electrical degrees.  
disp ""
disp "...Axis %i Commutation Startup Program Running...", SP_Axis
enable(SP_Axis)
wait(300)
while (DCOM(SP_Axis)+0.05 < SP_Drive); DCOM(SP_Axis) = DCOM(SP_Axis) + 0.05; end
DCOM(SP_Axis) = SP_Drive
wait SP_Settle_Time
call Limit_Check
!******************************************************************************* 
! STEP 2 - MOVE TO SECOND DETENT POINT
!
!   The program moves the motor 90 electrical degrees in order to eliminate
!      a state of unstable equilibrium.

! 目的: 执行 detent 点位移动, 避免不稳定平衡点。
! 输入条件: SP_Direction/SP_Pitch/SP_Search_Vel 已计算。
! 输出效果: 到位后再次检查限位故障。
Move_Detent:
ptp/rv (SP_Axis), SP_Direction*SP_Pitch/2.,SP_Search_Vel
till ^AST(SP_Axis).#MOVE; wait SP_Settle_Time
call Limit_Check
disable(SP_Axis)

! MFLAGS: 轴标志字。此处 .9 常用于表示换相状态(1=已换相)。
MFLAGS(SP_Axis).9=1               ! Set commutation state 
DCOM(SP_Axis) = 0

! If motor is to be left enabled after startup process delete the following line:

! 结束段: 输出成功/失败信息并清理状态标志。
Finish:
SP_InCommutationStartup=0              ! commutation startup is finished
If SP_Fail_Code=0; disp " Axis %i  Commutation Startup Finished.", SP_Axis
else disp "   Commutation Startup Failed."; disp "   Failure Code = %i",SP_Fail_Code; end

SP_Axis=ROBOTAXISNUML 
END
! 结束条件: 两轴流程完成或异常退出后停止本Buffer。
STOP

!******************************************************************************* 
!   The following routine move the motor away from limit switches 

! ---- 子程序 Limit_Check ----
! 目的: 检查MERR/限位故障, 必要时反向搜索脱离限位。
! 输入条件: 轴当前故障状态(MERR/FAULT)。
! 输出效果: 正常则 RET, 异常则置 SP_Fail_Code 并跳转 Finish。
Limit_Check:
if MERR(SP_Axis) & MERR(SP_Axis)<>5010 & MERR(SP_Axis)<>5011; SP_Fail_Code=1; DISABLE(SP_Axis); DCOM(SP_Axis)=0; goto Finish; end
!   if MERR(SP_Axis); SP_Fail_Code=1; DISABLE(SP_Axis); DCOM(SP_Axis)=0; goto Finish; end 
if (FAULT(SP_Axis).#LL)|(FAULT(SP_Axis).#RL)
   if FAULT(SP_Axis).#LL; SP_Direction=1; else SP_Direction=-1; end
   ptp/rv (SP_Axis), SP_Direction*SP_Max_Search, SP_Search_Vel 
   till ((^FAULT(SP_Axis).#LL)&(^FAULT(SP_Axis).#RL))|(^AST(SP_Axis).#MOVE)
   if (FAULT(SP_Axis).#LL)|(FAULT(SP_Axis).#RL); SP_Fail_Code=4; DISABLE(SP_Axis); DCOM(SP_Axis)=0; goto Finish; end 
   kill(SP_Axis);  wait SP_Settle_Time; goto Move_Detent
end
ret

! 事件处理: 换相过程中若出现系统/轴故障, 立即关闭驱动并走失败流程。
ON ((SP_InCommutationStartup = 1) & ((FAULT(SP_Axis)&0x30f80)>0 | (S_FAULT&0x30000000)>0))
SP_Fail_Code=1; DISABLE(SP_Axis); DCOM(SP_Axis)=0
CALL Finish 
RET
