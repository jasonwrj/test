#26

! 教学式注释: Buffer #26 机器人臂运动解释/执行主程序
! 目的: 解析外部设定值(Sp*)与速度(ramprate*), 计算机器人几何, 下发PTP运动, 并更新状态/日志。
! 入口条件: 主程序(通常Buffer #27)启动该Buffer; iType(0)>6000 表示机器人轴存在。
! 输出效果: 更新FRobot/Measured/WkSp/AxisInfo/AxisLog, 设置iControlProcessState等状态, 发起运动指令。
! robot arm axis instruction interpretation and execution including Modbus commands
! the other axis is coved by buffer 31
!The main program of the MAC application
!It starts automaticlly when start.
!It starts all boffer programs and initializes the parameers
!In the running state, it provides measured variables and Modbus registers update.

!Change log
! DATE		ECO#	Reason for Change			Remarks
!9/21/2015	N/A		ini release						ZW
!9/30/2015	N/A		change to GEN20A robot			ZW01
!11/20/2015	N/A		Swap axes 1 and 2				CR02
!12/10/2015	N/A		Try different robot equition	ZW02
! ---- 全局/共享变量 ----
! 这些变量与其他Buffer共享, 通常映射到TAG/状态字(见Buffer #A)或被上位机读写。
GLOBAL int 	RobotExtIsHoming, RobotRotIsHoming
GLOBAL real  RobotROTTargetPrev
GLOBAL real  RobotExtTargetPrev
GLOBAL real rRearArmLength, rFrontArmLength, rEndEffectorLength, rWristHalfWidth, rClearance, rRotCenterOffset
GLOBAL real rFullyRetracted, rFullyExtended, theta0
GLOBAL real FRobot(2), RealRobotAngle
Global real ArmExtMax, ArmExtMin
Global Int iAxisGood(8)
Global Int iRotAllowed, iExtMoving, iRotMoving
Global Real AxisLog(8)(3)
Global Real AxisInfo(8)(18)
GLOBAL real OldSP(8)
real rSqLengthDelta
Global real RobotArmExtHomeOffset ! ZW 1/9/16
real DirFlipperRot
real relative_angle
real extAngle, PosBeforeRot, PosBeforeExt
real tmp, theta1, theta2, lx
real x1, x2, h
GLOBAL real rSqLength
real HMotorAngle, LMotorAngle !@ZW02
real HMotorPos(3), LMotorPos(3), RobotAngle(3)
GLOBAL real SP_RobotExt, SP_RobotRot
Global int	ExecutingMoveExt, ExecutingMoveRot
Global Int iExtCommanded, iRotCommanded
Global real FlipperClampDTI
real tmpRotRampRate
RobotArmExtHomeOffset = 0.0 !inch ZW 1/9/16
iExtCommanded = 0
iRotCommanded = 0
ExecutingMoveRot = 0			!turn off moving flag
iRotMoving = 0					!used in buffer 27
ExecutingMoveExt = 0			!turn off moving flag
iExtMoving = 0					!used in buffer 27
!Sp0 = RobotArmExtHomeOffset !ArmExtMin
!Sp2 = 0	!CR02 Sp2 was Sp1
!bTeachMode=0 !ZW 03 for teaching, commented out on 11/29/2017 for not resetting by Software Reset button

rSqLengthDelta = rRearArmLength*rRearArmLength - rFrontArmLength*rFrontArmLength

real HMotorAngleBeforeExt, LMotorAngleBeforeExt
global real HMotorAngleBeforeRot, LMotorAngleBeforeRot, RotAxisAngleBeforeRot
real tmpWkSp(2)


! 调用运动学计算子程序, 更新FRobot/Measured/WkSp/AxisInfo等
CALL RobotCalculation
IF MFLAGS(ROBOTAXISNUMH).#HOME & MFLAGS(ROBOTAXISNUML).#HOME
	RobotExtTargetPrev = WkSp0
	if CONTROLLERMODELNUM=6
		RobotROTTargetPrev = WkSp1
	else
		RobotROTTargetPrev = WkSp2
	end
else
	RobotROTTargetPrev = 0
	RobotExtTargetPrev = RobotArmExtHomeOffset !ArmExtMin	ZW 1/9/16
end

Sp0 = RobotExtTargetPrev
if CONTROLLERMODELNUM=6
	Sp1 = RobotROTTargetPrev	!if rot cmd, set new extension SP back to previous
else
	Sp2 = RobotROTTargetPrev	!if rot cmd, set new extension SP back to previous
end	




! ---- 主循环入口 ----
! 入口条件: Buffer已启动且未STOP; 循环持续执行。
! 主要输入: Sp0/Sp1/Sp2, ramprate*, iType, MFLAGS/MST等状态。
! 主要输出: 运动指令(PTP/JOG), iControlProcessState, AxisInfo/AxisLog。
AA:
!BLOCK
! 仅当检测到机器人类型(轴0类型>6000)时才执行机器人运动逻辑
if iType(0)>6000					!only execute when robot exists

	
	
!	if ((iType(0)= 6001) |(iType(0)= 6003)|(iType(0)= 6005) |(iType(0)= 6007)|(iType(0)= 6009))	!this is a GEN20A Robot arm extension axis

! 调用运动学计算子程序, 更新FRobot/Measured/WkSp/AxisInfo等
CALL RobotCalculation

!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
!COMMAND
!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!	
	if((^RobotExtIsHoming)&(^RobotRotIsHoming))
		if iType(0)=6009
			if (bTeachMode=1)
				RobotArmExtHomeOffset = -2.5
			else
				RobotArmExtHomeOffset = dNegtiveLimit(0) !-2
			end
			rFullyExtended = dPositiveLimit(0)
			if (CONTROLLERMODELNUM=6)
				IF DINPUT0.5
! StatusWord0.6: 来自TAG状态字的夹持/安全标志(见Buffer #A映射), 用于触发夹持/退让流程
					StatusWord0.6=0
				END	
! StatusWord0.6: 来自TAG状态字的夹持/安全标志(见Buffer #A映射), 用于触发夹持/退让流程
				if(StatusWord0.6)&(^iRotCommanded)&(^iExtCommanded)&(^DINPUT0.5)
					FDEF(0).#CPE = 0
					FDEF(ROBOTAXISNUML).#CPE = 0
					XCURV(0) = dHomingTorq(0)
					JOG/v 0, dHomingVel(0)	!ZW03 change polar
					TILL (FAULT(0).#CPE = 1) | (FAULT(ROBOTAXISNUML).#CPE = 1)| (KillTraj0=1)|(Measured0<-0.005)  			! Wait for the critical PE
					HALT 0
					wait 1000
					DINPUT0.5 = 1
					StatusWord0.6=0
					XCURI(0) = dFaultTorq(0)
					FDEF(0).#CPE = 1
					FDEF(ROBOTAXISNUML).#CPE = 1

				end
			end	
		end
! ---- 指令检查与修正 ----
! 输入条件: Sp0/Sp1/Sp2 由上位机或Modbus设定, 需要限制到安全范围。
! 输出效果: 修正SP_RobotExt/Rot, 并在不允许时恢复上一次目标。
! EXAMINNG THE MOTION COMMANDS
		if (Sp0 <RobotArmExtHomeOffset) !rFullyRetracted) ZW04 1/9/2016 to remove offset
			Sp0 = RobotArmExtHomeOffset
		elseif ((Sp0 + rFullyRetracted) >rFullyExtended) !C-C distance
			Sp0 = RobotExtTargetPrev
			iControlProcessState(0) = 102
		end

		if ^iRotCommanded
			SP_RobotExt = Sp0			!if rotation is not commanded, take the extension cmd. only allow extension setpoint change if not cmd to rotate
			
		else
			Sp0 = RobotExtTargetPrev	!if rotation is cmded, reject extension cmd, set new extension SP back to previous
			
		end
		
		if ^iExtCommanded
			if CONTROLLERMODELNUM=6
				SP_RobotRot = Sp1			!only allow extension setpoint change if not cmd to rotate
			else
				SP_RobotRot = Sp2			!only allow extension setpoint change if not cmd to rotate
			end		
		else
			if CONTROLLERMODELNUM=6
				Sp1 = RobotROTTargetPrev	!if rot cmd, set new extension SP back to previous
			else
				Sp2 = RobotROTTargetPrev	!if rot cmd, set new extension SP back to previous
			end	
			
		end
	

! ---- 伸缩轴运动 ----
! 输入条件: 机器人轴可用(iAxisGood), 未被旋转命令锁定, 速度有效。
! 输出效果: 计算扩展角度extAngle并下发PTP, 更新iExtMoving/ExecutingMoveExt等状态。
!Extension motion 
		if iAxisGood(ROBOTAXISNUMH)&iAxisGood(ROBOTAXISNUML)	

			if (abs(SP_RobotExt-RobotExtTargetPrev)>0.00001) & (^iRotCommanded) & (abs(ramprate0) > 0.1)& MST(ROBOTAXISNUMH).#ENABLED & MST(ROBOTAXISNUML).#ENABLED	!to avoid 0 speed motion @ZW03
				if ^ExecutingMoveExt	!only HALT if already moving
					HMotorAngleBeforeExt=RPOS(ROBOTAXISNUMH)
					LMotorAngleBeforeExt=RPOS(ROBOTAXISNUML)
				end
				iExtCommanded = 1	!set move commanded flag
				iExtMoving = 1		!used in buffer 27
				if iType(0)=6009
					extAngle = SP_RobotExt/FlipperClampDTI
				else
					tmp = (SP_RobotExt + rFullyRetracted) - rEndEffectorLength	+ rRotCenterOffset!ZW 1/9/2016 to remove 14.16 offset
					if(tmp>0.0)
						theta1 = atan( rWristHalfWidth/tmp)
					elseif(tmp=0.0)
						theta1 = 3.141592654/2.0
					elseif(tmp<0.0)
						theta1 = 3.141592654- atan( rWristHalfWidth/abs(tmp))
					end
					lx = sqrt(rWristHalfWidth*rWristHalfWidth + tmp*tmp)
					real tmptest
					tmptest = (rSqLengthDelta + lx*lx)/(2*lx*rRearArmLength)
					if(tmptest>1)
						tmptest=1
					end	
					theta2 = acos (tmptest)
					extAngle = theta0 - (theta1 + theta2)/3.141592654*180.0 !@CR03 remove FPOS(0)
				end	
				
				if iType(0)= 6009
					VEL(ROBOTAXISNUMH) = ramprate0/FlipperClampDTI 
					VEL(ROBOTAXISNUML) = ramprate0/FlipperClampDTI 
					ACC(ROBOTAXISNUMH) = dAmax(ROBOTAXISNUMH)/FlipperClampDTI
					ACC(ROBOTAXISNUML) = dAmax(ROBOTAXISNUMH)/FlipperClampDTI
					DEC(ROBOTAXISNUMH) = dAmax(ROBOTAXISNUMH)/FlipperClampDTI
					DEC(ROBOTAXISNUML) = dAmax(ROBOTAXISNUMH)/FlipperClampDTI
					!JERK(ROBOTAXISNUMH)= dAmax(ROBOTAXISNUMH)/FlipperClampDTI
					!JERK(ROBOTAXISNUML)= dAmax(ROBOTAXISNUMH)/FlipperClampDTI
					KDEC(ROBOTAXISNUMH)=2*dAmax(ROBOTAXISNUMH)/FlipperClampDTI
					KDEC(ROBOTAXISNUML)=2*dAmax(ROBOTAXISNUMH)/FlipperClampDTI
				else
					VEL(ROBOTAXISNUMH)=ramprate0 
					VEL(ROBOTAXISNUML)=ramprate0 
					ACC(ROBOTAXISNUMH) = dAmax(ROBOTAXISNUMH)
					ACC(ROBOTAXISNUML) = dAmax(ROBOTAXISNUMH)
					DEC(ROBOTAXISNUMH) = dAmax(ROBOTAXISNUMH)
					DEC(ROBOTAXISNUML) = dAmax(ROBOTAXISNUMH)
					!JERK(ROBOTAXISNUMH)= ACC(ROBOTAXISNUMH)
					!JERK(ROBOTAXISNUML)= ACC(ROBOTAXISNUMH)
					KDEC(ROBOTAXISNUMH)=2*dAmax(ROBOTAXISNUMH)
					KDEC(ROBOTAXISNUML)=2*dAmax(ROBOTAXISNUMH)
				end
				
				BREAK(ROBOTAXISNUMH, ROBOTAXISNUML)
				if ((iType(0)= 6005) |(iType(0)= 6007))
					! you have to command two axis seperately to have the BREAK command working
					!PTP (ROBOTAXISNUMH, ROBOTAXISNUML), ((HMotorAngleBeforeExt+LMotorAngleBeforeExt)/2) - extAngle, ((HMotorAngleBeforeExt+LMotorAngleBeforeExt)/2) + extAngle	
					BLOCK
						PTP (ROBOTAXISNUMH), ((HMotorAngleBeforeExt+LMotorAngleBeforeExt)/2) - extAngle
						PTP (ROBOTAXISNUML), ((HMotorAngleBeforeExt+LMotorAngleBeforeExt)/2) + extAngle	
					END
				else
					PTP (ROBOTAXISNUMH), (LMotorAngleBeforeExt - extAngle)
				end
				iControlProcessState(0) = 11
			end


			if iExtCommanded							!if commanded to move
				if MST(ROBOTAXISNUMH).4 = 0 | MST(ROBOTAXISNUML).4 = 0		!if moving - 0 means motor is moving
					ExecutingMoveExt = 1			!set moving flag
					RobotExtTargetPrev = SP_RobotExt
				end
			end
				
			if ExecutingMoveExt						!if moving
				if MST(ROBOTAXISNUMH).4 = 1 & MST(ROBOTAXISNUML).4 = 1		!if stopped - 1 means move is complete
					ExecutingMoveExt = 0			!turn off moving flag
					iExtCommanded = 0				!turn off commanded flag
					iExtMoving = 0					!used in buffer 27
					iControlProcessState(0) = 10
					Sp0 = WkSp0	!Measured0					!to accept the same Sp if the previouse motion stoped
					SP_RobotExt =Sp0
					RobotExtTargetPrev = Sp0
				end 
			end

! ---- 旋转轴运动 ----
! 输入条件: 伸缩位置满足安全退让(rClearance/RobotArmExtHomeOffset), 无伸缩命令冲突。
! 输出效果: 计算最短旋转路径并下发PTP, 更新iRotMoving/ExecutingMoveRot等状态。
! rotation command

			if (FRobot(0)<RobotArmExtHomeOffset+rClearance) | (bTeachMode=1)!ZW 03 for teaching
				if iControlProcessState(ROBOTAXISNUML) = 108
					iControlProcessState(ROBOTAXISNUML) = 10
				end
				iRotAllowed = 1
				if MAXNUMOFAXIS = 2
					tmpRotRampRate = ramprate1
				else
					tmpRotRampRate = ramprate2
				end				
				
				if (abs(SP_RobotRot - RobotROTTargetPrev)>0.00001) & (^iExtCommanded) & (abs(tmpRotRampRate) > 0.01) & MST(ROBOTAXISNUMH).#ENABLED & MST(ROBOTAXISNUML).#ENABLED		!to avoid 0 speed motion @ZW03

					if ^ExecutingMoveRot	!only HALT if already moving 
						LMotorAngleBeforeRot=RPOS(ROBOTAXISNUML)
						HMotorAngleBeforeRot=RPOS(ROBOTAXISNUMH)

						if (iType(ROBOTAXISNUML)= 6002) | (iType(ROBOTAXISNUML)= 6004)
							RotAxisAngleBeforeRot = LMotorAngleBeforeRot*2 	!@ZW02
						else
							RotAxisAngleBeforeRot = (HMotorAngleBeforeRot+LMotorAngleBeforeRot)/2 	!@ZW02
						end
						if(iType(ROBOTAXISNUML)= 6010)
							!0-360 representation
							RotAxisAngleBeforeRot= LMotorAngleBeforeRot-360*FLOOR(LMotorAngleBeforeRot/360)
						else
							!?180 representation
							RotAxisAngleBeforeRot=RotAxisAngleBeforeRot+180
							RotAxisAngleBeforeRot=RotAxisAngleBeforeRot-FLOOR(RotAxisAngleBeforeRot/360)*360-180
						end
					end		
					iRotCommanded = 1	!set move commanded flag
					iRotMoving = 1		!used in Buffer 27
					if iType(0)= 6009
						relative_angle = SP_RobotRot-360*FLOOR((SP_RobotRot)/360)- RotAxisAngleBeforeRot
						if relative_angle <= -180.0					!calculate shortest path to SP
							relative_angle = relative_angle + 360
						elseif relative_angle > 180.0
							relative_angle = relative_angle - 360
						end														!end calculation for shortest path
						
						
						if VELDIR1 >0
							if relative_angle<0
								relative_angle = relative_angle +360
							end
							VELDIR1 = 0	!reset the DIR bit to default
						elseif VELDIR1 <0
							if relative_angle>0
								relative_angle = relative_angle -360
							end
							VELDIR1 = 0	!reset the DIR bit to default
						end
						
					else	
						relative_angle = (SP_RobotRot - RotAxisAngleBeforeRot)
						while (relative_angle <= -180) 
							relative_angle = relative_angle + 360
						end
						while (relative_angle > 180) !
							relative_angle = relative_angle - 360
						end
					end
					VEL(ROBOTAXISNUMH)=tmpRotRampRate !CR02 ramprate2 was ramprate0 
					VEL(ROBOTAXISNUML)=tmpRotRampRate	!	- set both motors to velocity for robot rotation
					ACC(ROBOTAXISNUMH) = dAmax(ROBOTAXISNUML)
					ACC(ROBOTAXISNUML) = dAmax(ROBOTAXISNUML)
					DEC(ROBOTAXISNUMH) = dAmax(ROBOTAXISNUML)
					DEC(ROBOTAXISNUML) = dAmax(ROBOTAXISNUML)
					!JERK(ROBOTAXISNUMH)= dAmax(ROBOTAXISNUML)
					!JERK(ROBOTAXISNUML)= dAmax(ROBOTAXISNUML)
					KDEC(ROBOTAXISNUMH)=2*dAmax(ROBOTAXISNUML)
					KDEC(ROBOTAXISNUML)=2*dAmax(ROBOTAXISNUML)

					if ((iType(0)= 6001) |(iType(0)= 6003))
						relative_angle=relative_angle/2.0
					end
					BREAK (ROBOTAXISNUMH, ROBOTAXISNUML)
					BLOCK
						PTP ROBOTAXISNUMH, HMotorAngleBeforeRot+relative_angle
						PTP ROBOTAXISNUML, LMotorAngleBeforeRot+relative_angle
					END
					iControlProcessState(ROBOTAXISNUML) = 11
		 
				end
				
				if iRotCommanded							!if commanded to move
					if MST(0).4 = 0 | MST(ROBOTAXISNUML).4 = 0		!if moving - 0 means motor is moving
						ExecutingMoveRot = 1			!set moving flag
						RobotROTTargetPrev = SP_RobotRot
					end
				end
				
				if ExecutingMoveRot						!if moving
					if MST(0).4 = 1 & MST(ROBOTAXISNUML).4 = 1		!if stopped - 1 means move is complete
						ExecutingMoveRot = 0			!turn off moving flag
						iRotCommanded = 0
						iRotMoving = 0					!used in buffer 27
						iControlProcessState(ROBOTAXISNUML) = 10
						if ROBOTAXISNUML=1
							Sp1 = WkSp1	!Measured1				!to accept the same Sp if the previouse motion stoped
							SP_RobotRot = Sp1
							RobotROTTargetPrev = Sp1
						else
							Sp2 = WkSp2	!Measured2
							SP_RobotRot = Sp2
							RobotROTTargetPrev = Sp2
						end
					end 
				end		
			else
				iRotAllowed = 0
				SP_RobotRot = RobotROTTargetPrev		!ZW 12/28/15 to avoid auto execute motion after arm retracted
				iControlProcessState(ROBOTAXISNUML) = 108
			end
		end
	else
		if((iControlProcessState(0)<>14)&(iControlProcessState(ROBOTAXISNUML)<>14))
			iControlProcessState(0) = 0
			iControlProcessState(ROBOTAXISNUML) = 0
		end	
	end !if((^RobotExtIsHoming)&(^RobotRotIsHoming))
end


!END
! 循环继续执行, 持续监测SP与状态
GOTO AA

STOP


! ---- 子程序 RobotCalculation ----
! 目的: 依据当前反馈位置(FPOS/RPOS)计算机器人笛卡尔扩展/旋转位置(FRobot)。
! 输入条件: 机器人未在对应方向运动(iExtMoving/iRotMoving=0)时更新测量。
! 输出效果: 更新Measured/WkSp/AxisInfo/AxisLog, 供上位机/TAG读出。
! MST/MFLAGS说明:
! - MST(AX).4 通常用于判断是否在运动(0=运动,1=停止到位, 依控制器定义)。
! - MFLAGS(AX).#HOME 表示回零完成标志。
! TAG说明: AxisInfo/AxisLog/Measured/WkSp等常在Buffer #A中映射到TAG用于上位机通信。
RobotCalculation:

		IF (iRotMoving= 0)& (RobotRotIsHoming = 0)	!if robot is not in rotation
			if ((iType(0)= 6005) |(iType(0)= 6007))
				tmp = (-FPOS(ROBOTAXISNUMH)+FPOS(ROBOTAXISNUML))/2 	!@ZW02
			else
				tmp = (-FPOS(ROBOTAXISNUMH)+FPOS(ROBOTAXISNUML)) 	!@ZW02
			
			end
			if(iType(0)= 6009)
				FRobot(0) = tmp*FlipperClampDTI 	!Flipper clamping pitch dia is 1"
			else
				tmp = (theta0 - tmp)*3.141592654/180.0 !@ZW02 changed from FPOS(0) to tmp
				x1 = rRearArmLength*COS(tmp)
				h = rRearArmLength*SIN(tmp) - rWristHalfWidth
				tmp = abs(rSqLength-h*h)
				x2 = SQRT(tmp)
				if FRobot(0)<0.0001 & FVEL(0)>0.001
					FRobot(0) = -(x1 + x2 + rEndEffectorLength-rFullyRetracted - rRotCenterOffset)  !ZW04 1/9/16 to remove offset
				else
					FRobot(0) = x1 + x2 + rEndEffectorLength-rFullyRetracted - rRotCenterOffset  !ZW04 1/9/16 to remove offset
				end				
			end
			
			if ((iType(0)= 6005) |(iType(0)= 6007))
				tmp = (-RPOS(ROBOTAXISNUMH)+RPOS(ROBOTAXISNUML))/2 	!@ZW02
			else
				tmp = (-RPOS(ROBOTAXISNUMH)+RPOS(ROBOTAXISNUML)) 	!@ZW02
			
			end
			if(iType(0)= 6009)
				tmpWkSp(0) = tmp
			else
				tmp = (theta0 - tmp)*3.141592654/180.0 !@ZW02 changed from FPOS(0) to tmp
				x1 = rRearArmLength*COS(tmp)
				h = rRearArmLength*SIN(tmp) - rWristHalfWidth
				tmp = abs(rSqLength-h*h)
				x2 = SQRT(tmp)
				if tmpWkSp(0)<0.0001 & RVEL(0)>0.001
					tmpWkSp(0) = -(x1 + x2 + rEndEffectorLength-rFullyRetracted - rRotCenterOffset )  !ZW04 1/9/16 to remove offset
				else
					tmpWkSp(0) = x1 + x2 + rEndEffectorLength-rFullyRetracted - rRotCenterOffset   !ZW04 1/9/16 to remove offset
				end
			end
		end
		Measured0 = FRobot(0)
		WkSp0 = tmpWkSp(0)
		MaxVel0 = XVEL(0)		!ZW03
		MinVel0 = -XVEL(0)		!ZW03
		AxisDec0 = DEC(0)
		AxisAcc0 = ACC(0)
		FaultTorq0 = XRMS(0)
		HomeTorq0 = dHomingTorq(0)			!ZW03
		MaxTorq0 =XCURV(0)
		if (abs(dPositiveLimit(0)-dNegtiveLimit(0))>0.00001)
			OUTP0= 100.0*abs(Measured0-dNegtiveLimit(0))/abs(dPositiveLimit(0)-dNegtiveLimit(0))
		else	
			OUTP0=0.0
		end
	!update axis information to GUI
		AxisLog(0)(0) = TIME
		AxisLog(0)(1) = Sp0
		AxisLog(0)(2) = Measured0
		AxisInfo(0)(0) = Measured0		!Position
		AxisInfo(0)(1) = FVEL(0)		!Velocity
		AxisInfo(0)(2) = FACC(0)		!Acc
		AxisInfo(0)(3) = MST(0).0		!0: motor is disabled 1 : motor is enabled.
		AxisInfo(0)(4) = MFLAGS(0).3	!Homed ? 0, Not done, 1, done
		AxisInfo(0)(5) = MST(0).4		!0 : Motor is not moving and has reached the target position(see variables TARGRAD and SETTLE)
												!1 : Motor is moving or is out of range
												! CR I think the correct status is: 1 = in-position; 0 = moving

		AxisInfo(0)(6) = FAULT(0).12	!POSITION ERROR
		AxisInfo(0)(7) = FAULT(0).9		!Drive error
		AxisInfo(0)(8) = FAULT(0).5		!S / W P limit
		AxisInfo(0)(9) = FAULT(0).6		!S / W N limit
		AxisInfo(0)(10) = FAULT(0).7	!Encoder disconnected
		AxisInfo(0)(11) = Sp0
		AxisInfo(0)(12) = OldSP(0)
		AxisInfo(0)(13) = WkSp0
		AxisInfo(0)(14) = ramprate0
		AxisInfo(0)(15) = dCE(0)
		AxisInfo(0)(16) = MERR(0)
		iAxisGood(0) = AxisInfo(0)(3)*AxisInfo(0)(4)*^(AxisInfo(0)(7))*^(AxisInfo(0)(10))
	
	!elseif ((iType(iAxis)= 6002) | (iType(iAxis)= 6004) | (iType(iAxis)= 6006) |(iType(iAxis)= 6008)|(iType(iAxis)= 6010))	!this is a Robot rotation axis
		IF (iExtMoving=0)&(RobotExtIsHoming=0)	!if robot is not in extension
			!tmp = FPOS(2)
			if (iType(ROBOTAXISNUML)= 6002) | (iType(ROBOTAXISNUML)= 6004)
				tmp = FPOS(ROBOTAXISNUML)*2 	!@ZW02
			else
				tmp = (FPOS(ROBOTAXISNUMH)+FPOS(ROBOTAXISNUML))/2 	!@ZW02
			end
			RealRobotAngle = tmp
			if(iType(ROBOTAXISNUML)= 6010)
				!0-360 representation
				tmp= FPOS(ROBOTAXISNUML)-360.0*FLOOR(FPOS(ROBOTAXISNUML)/360)
				if(abs(SP_RobotRot)<=0.00001) & ((abs(tmp)<=0.001)|(abs(tmp-360)<=0.001) )
					tmp = SP_RobotRot
				end
				if(abs(SP_RobotRot-360)<=0.00001) & ((abs(tmp)<=0.001)|(abs(tmp-360)<=0.001) )
					tmp = SP_RobotRot
				end	
			else
				!?180 representation
				!tmp = asin(sin(tmp/180*3.141592654))/3.141592654*180 
				
				!if (tmp <= -180) 
				!	tmp = tmp +  360*(FLOOR(tmp/360)+1)
				!end
				!if (tmp > 180) !
				!	tmp = tmp - 360*FLOOR(tmp/360)
				!end	
				
				tmp=tmp+180
				tmp=tmp-FLOOR(tmp/360)*360-180
				
				if(abs(SP_RobotRot-180)<=0.00001) & ((abs(tmp-180)<=0.001)|(abs(tmp+180)<=0.001) )
					tmp = SP_RobotRot
				end
				if(abs(SP_RobotRot+180)<=0.00001) & ((abs(tmp-180)<=0.001)|(abs(tmp+180)<=0.001) )
					tmp = SP_RobotRot
				end	

			end
			FRobot(1) = tmp
			if (iType(ROBOTAXISNUML)= 6002) | (iType(ROBOTAXISNUML)= 6004)
				tmp = RPOS(ROBOTAXISNUML)*2 	!@ZW02
			else
				tmp = (RPOS(ROBOTAXISNUMH)+RPOS(ROBOTAXISNUML))/2 	!@ZW02
			end
			if(iType(ROBOTAXISNUML)= 6010)
				!0-360 representation
				tmp= RPOS(ROBOTAXISNUML)-360*FLOOR(RPOS(ROBOTAXISNUML)/360)

			else
!				if (tmp <= -180) 
!					tmp = tmp +  360*(FLOOR(tmp/360)+1)
!				end
!				if (tmp > 180) !
!					tmp = tmp -  360*FLOOR(tmp/360)
!				end	
				
				tmp=tmp+180
				tmp=tmp-FLOOR(tmp/360)*360-180

				
			end	
			tmpWkSp(1) = tmp
			
			
		end
	
		if CONTROLLERMODELNUM=6 
			Measured1 = FRobot(1)
			WkSp1 = tmpWkSp(1) !RPOS(2)
			MaxVel1 = XVEL(ROBOTAXISNUML)		!ZW03
			MinVel1 = -XVEL(ROBOTAXISNUML)		!ZW03
			AxisDec1 = DEC(ROBOTAXISNUML)
			AxisAcc1 = ACC(ROBOTAXISNUML)
			FaultTorq1 = XRMS(ROBOTAXISNUML)
			HomeTorq1 = dHomingTorq(ROBOTAXISNUML)			!ZW03
			MaxTorq1 =XCURV(ROBOTAXISNUML)
			AxisLog(ROBOTAXISNUML)(1) = Sp1
			AxisLog(ROBOTAXISNUML)(2) = Measured1
			AxisInfo(ROBOTAXISNUML)(0) = Measured1		!Position
			AxisInfo(ROBOTAXISNUML)(11) = Sp1
			AxisInfo(ROBOTAXISNUML)(12) = OldSP(ROBOTAXISNUML)
			AxisInfo(ROBOTAXISNUML)(13) = WkSp1
			AxisInfo(ROBOTAXISNUML)(14) = ramprate1

			if (abs(dPositiveLimit(ROBOTAXISNUML)-dNegtiveLimit(ROBOTAXISNUML))>0.00001)
				OUTP1= 100.0*abs(Measured1-dNegtiveLimit(ROBOTAXISNUML))/abs(dPositiveLimit(ROBOTAXISNUML)-dNegtiveLimit(ROBOTAXISNUML))
			else	
				OUTP1=0.0
			end

		else
			Measured2 = FRobot(1)
			WkSp2 = tmpWkSp(1) !RPOS(2)
			MaxVel2 = XVEL(ROBOTAXISNUML)		!ZW03
			MinVel2 = -XVEL(ROBOTAXISNUML)		!ZW03
			AxisDec2 = DEC(ROBOTAXISNUML)
			AxisAcc2 = ACC(ROBOTAXISNUML)
			FaultTorq2 = XRMS(ROBOTAXISNUML)
			HomeTorq2 = dHomingTorq(ROBOTAXISNUML)			!ZW03
			MaxTorq2 =XCURV(ROBOTAXISNUML)
			AxisLog(ROBOTAXISNUML)(1) = Sp2
			AxisLog(ROBOTAXISNUML)(2) = Measured2
			AxisInfo(ROBOTAXISNUML)(0) = Measured2		!Position
			AxisInfo(ROBOTAXISNUML)(11) = Sp2
			AxisInfo(ROBOTAXISNUML)(12) = OldSP(ROBOTAXISNUML)
			AxisInfo(ROBOTAXISNUML)(13) = WkSp2
			AxisInfo(ROBOTAXISNUML)(14) = ramprate2

			if (abs(dPositiveLimit(ROBOTAXISNUML)-dNegtiveLimit(ROBOTAXISNUML))>0.00001)
				OUTP2= 100.0*abs(Measured2-dNegtiveLimit(ROBOTAXISNUML))/abs(dPositiveLimit(ROBOTAXISNUML)-dNegtiveLimit(ROBOTAXISNUML))
			else	
				OUTP2=0.0
			end	
		end
		
		
		!update axis information to GUI
		AxisLog(ROBOTAXISNUML)(0) = TIME
		AxisInfo(ROBOTAXISNUML)(1) = FVEL(ROBOTAXISNUML)		!Velocity
		AxisInfo(ROBOTAXISNUML)(2) = FACC(ROBOTAXISNUML)		!Acc
		AxisInfo(ROBOTAXISNUML)(3) = MST(ROBOTAXISNUML).0		!0: motor is disabled 1 : motor is enabled.
		AxisInfo(ROBOTAXISNUML)(4) = MFLAGS(ROBOTAXISNUML).3	!Homed ? 0, Not done, 1, done
		AxisInfo(ROBOTAXISNUML)(5) = MST(ROBOTAXISNUML).4		!0 : Motor is not moving and has reached the target position(see variables TARGRAD and SETTLE)
												!1 : Motor is moving or is out of range
												! CR I think the correct status is: 1 = in-position; 0 = moving

		AxisInfo(ROBOTAXISNUML)(6) = FAULT(ROBOTAXISNUML).12	!POSITION ERROR
		AxisInfo(ROBOTAXISNUML)(7) = FAULT(ROBOTAXISNUML).9		!Drive error
		AxisInfo(ROBOTAXISNUML)(8) = FAULT(ROBOTAXISNUML).5		!S / W P limit
		AxisInfo(ROBOTAXISNUML)(9) = FAULT(ROBOTAXISNUML).6		!S / W N limit
		AxisInfo(ROBOTAXISNUML)(10) = FAULT(ROBOTAXISNUML).7	!Encoder disconnected
		AxisInfo(ROBOTAXISNUML)(15) = dCE(ROBOTAXISNUML)
		AxisInfo(ROBOTAXISNUML)(16) = MERR(ROBOTAXISNUML)
		iAxisGood(ROBOTAXISNUML) = AxisInfo(ROBOTAXISNUML)(3)*AxisInfo(ROBOTAXISNUML)(4)*^(AxisInfo(ROBOTAXISNUML)(7))*^(AxisInfo(ROBOTAXISNUML)(10))
RET
