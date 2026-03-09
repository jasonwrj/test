#27
!The main program of the MAC application
!It starts automaticlly when start.
!It starts all boffer programs and initializes the parameers
!In the running state, it provides measured variables and Modbus registers update.
! DATE		ECO#	Reason for Change								Remarks
!9/21/2015	N/A		ini release										ZW01
!12/1/2015	N/A		Swapped axes 1 and 2 so robot on same module	CR
!12/8/2015	N/A		Updated modbus mapping for Molly compatibility	CR
!12/10/2015	N/A		Try different robot equition					ZW02
!12/28/2015	N/A		Axis parameters									ZW03
!3/10/2017			Internal version 2.02.012					
!10/23/2020			V2.10, changed from Measured to WkSp
!8/25/2021          V2.12


AUTOEXEC:
KILLALL
!STOP ALL Buffers Except for a VAT control buffer
INT j
j=0
loop 32
	if (j<7) 
		if (iType(j)<>1040)
			STOP j
		end
	else
		if (j<>27)	!do not stop itself	
			STOP j
		end	
	end	
	j=j+1
end

FCLEAR ALL
! initializing the robot parameters
!====ZW on 2/4/2016====
Global	int		Part_number
Global	int		Sub_part_number   !1001--8 axis controller 1002--4 axis controller
Global	real	Revision_number   !REV A
!====ZW on 2/4/2016====
real LPF_Beta
LPF_Beta = 0.05!0.025! ; // 0<?<1
int iSecondAxis
int iTmpIndex
int bInterlocActive, bSecondSP

REAL 	StartTime, ElapsedTime 
INT		TimerON
Global Int	Buffer11RunningFlag, Buffer16RunningFlag, Buffer17RunningFlag
INT		Buffer11Needed, Buffer17Needed
Global int	ExecutingMoveExt, ExecutingMoveRot
GLOBAL real rRearArmLength, rFrontArmLength, rEndEffectorLength, rWristHalfWidth, rClearance, rRotCenterOffset
GLOBAL real rFullyRetracted, rFullyExtended, theta0
GLOBAL real FlipperClampDTI
GLOBAL real SP(8)
GLOBAL real OldSP(8)
Global real RampRate(8), SP_Modbus(8)
GLOBAL real FRobot(2)
GLOBAL INT iAxisType(8), iAxisSubType(8)
Global Real AxisInfo(8)(18)
Global Real AxisLog(8)(3)
Global Int iAxisGood(8), iExtMoving, iRotMoving
Global real ArmExtMax, ArmExtMin
Global Int iSpChecked(8)
GLOBAL real SP_RobotExt, SP_RobotRot
GLOBAL real  RobotROTTargetPrev
GLOBAL real  RobotExtTargetPrev
INT iLLDoorOpened, iLLDoorOpened2
GLOBAL real rSqLength, rSqLengthDelta
Global 	Real 	dQcmLimit(8)!This is the value that is evaluated in Buffer 31 before a move is executed
GLOBAL REAL adjNegativeLimit(8) !Used for ozone injector ring position limit when crash contitions are possible with other actuators
GLOBAL REAL adjPositiveLimit(8)  !Used for ozone injector ring position limit when crash contitions are possible with other actuators
GLOBAL INT	M2G_InjPresent, M2G_CARZ, CARZ_AxisNum, M2C_CARZ, CARZ_PM_AxisNum, M2C_CARZ2, CARZ_PM2_AxisNum
Global real	mshLCZ_old, mshUCZ_old, bfmLCZ_old, bfmUCZ_old, qcmLCZ_old, qcmUCZ_old, xfrLCZ_old, xfrUCZ_old	!used to compare against changes from IOC Magellan
INT iAxis
INT A0_Enabled, A0_Homed, A0_NegLimit, A0_DriveError, A0_EncoderDisconnected
INT A2_Enabled, A2_Homed, A2_NegLimit, A2_DriveError, A2_EncoderDisconnected

INT AxisNum
INT iKillTrajVar(8)
GLOBAL INT bAxisInterlocked
GLOBAL INT HW_Intlkd_BFMQCM_AxisNum1, HW_Intlkd_BFMQCM_AxisNum2

!variables added for MZ upgrade
Global INT 	MZ_CARAxisNum, MZ_LLAxisNum, MZ_STAxisNum	!See Buffer 27, initialized
Global INT	oLL_Door_Closed		!iWord7Bit3			:= dinputs.,

HW_Intlkd_BFMQCM_AxisNum1 = -1
HW_Intlkd_BFMQCM_AxisNum2 = -1
INT bAxisAtSecondSP
!by ZW on 4/25/2016 for free up axis numbers. 
!The following vaiables are temperary to make the code automation
global real tmpMeasured(8)
real tmpWkSp(8)
real tmpMaxVel(8)
real tmpMinVel(8)
real tmpAxisDec(8)
real tmpAxisAcc(8)
real tmpFaultTorq(8)
real tmpHomeTorq(8)
real tmpMaxTorq(8)
real tmpSLVKP(8)!servo velocity loop Kp
real tmpSLVKI(8)!servo velocity loop Ki
real tmpSLPKP(8)!servo position loop Kp

real PosMeasured(4)
int tmpDINPUT

j=0
LOOP 8
	tmpDINPUT.j=0
	j=j+1
end
bAxisInterlocked=1
bAxisAtSecondSP=0
KillTraj0=0
KillTraj1=0
KillTraj2=0
KillTraj3=0
KillTraj4=0
KillTraj5=0
KillTraj6=0
KillTraj7=0

FlipperClampDTI=0.0087*2.0

Buffer11Needed=0	!initialize to NOT needed at reset or reboot
Buffer17Needed=0

!=====added by ZW on 2/4/2016====
Part_number = 1265590
Sub_part_number = 1001   !8 axis controller
!Global	int		Sub_part_number                                                =1002                    !4 axis controller
Revision_number = 2.13                      !means REV A
MAXNUMOFAXIS = SYSINFO(13)
SWVersion0 = Revision_number*100
InstrumentID0 = MAXNUMOFAXIS

IF MAXNUMOFAXIS > 8
	MAXNUMOFAXIS = 8
END
CONTROLLERMODELNUM = (ECGETPID(0)/POW(2,24))
LLELEAXIS = 0	!LL1 does not work on axis0
LLELEAXIS2 = 0	!LL2 does not work on axis0
if CONTROLLERMODELNUM=6
	ROBOTAXISNUMH=0
	ROBOTAXISNUML=1
!	LLELEAXIS = 0	!LL LE only works for 3
else
	ROBOTAXISNUMH=0
	ROBOTAXISNUML=2
!	LLELEAXIS = 3
end
MFLAGS(ROBOTAXISNUMH).#DEFCON = 1
MFLAGS(ROBOTAXISNUML).#DEFCON = 1

!=====added by ZW on 2/4/2016====

!At reset/start-up, load the user-saved values from flash.  These are not the defaults.
READ iType, iType
READ dFixedPosA, dFixedPosA
READ dFixedPosB, dFixedPosB
READ dFixedPosC, dFixedPosC
READ dFixedPosD, dFixedPosD
READ dPositiveLimit, dPositiveLimit
READ dNegtiveLimit, dNegtiveLimit
READ dVel, dVel
READ dJogVel, dJogVel			
READ dVmax,dVmax 
READ dAmax,dAmax 
READ dFaultTorq, dFaultTorq		
READ dHomingTorq, dHomingTorq 	
READ dHomingVel, dHomingVel 
READ iInterlockEnabled, iInterlockEnabled
READ iInterlockBehavior, iInterlockBehavior 	
READ dInterlockPos, dInterlockPos
READ iSecondSPEnabled, iSecondSPEnabled
READ dSecondSetpointPos, dSecondSetpointPos
READ dHomePulseWidth, dHomePulseWidth	
READ dHomePulseDelay, dHomePulseDelay
READ dAxisPitch, dAxisPitch
READ dGearRatio, dGearRatio				!added by ZW on 2/4/2016
READ iHomingMethod, iHomingMethod		!added by ZW on 2/4/2016
READ dHomeOffset, dHomeOffset		!added by ZW on 2/4/2016
READ iQcmAxis, iQcmAxis		!added by ZW on 2/4/2016
READ UseAnalogCmd, UseAnalogCmd		!ZW 7/15/2016
READ dBackLashCompensation, dBackLashCompensation
READ AnalogChannelOffset, AnalogChannelOffset
READ iInverseInterlockLogic, iInverseInterlockLogic
READ O3CollisionValues, O3CollisionValues



wait 1000
! Initializing the communication parameters
CONID = 1! Set Slave address 247
if CONTROLLERMODELNUM <> 0
	setconf(302, 2, 2)! Set COM1 for MODbus communication, Slave mode
	setconf(303, 2, 19200)! Set baud rate 19200
	setconf(304, 2, 0x18)! Set no parity 0x10
	setconf(309,1,1) !Low word first, then Hi word
END
!load Ozone injector collision values from persistent memory
mshLCZ = O3CollisionValues(0)	!Lower Crash Zone (LCZ) - main shutter
mshUCZ = O3CollisionValues(1)	!Upper Crash Zone (UCZ) - main shutter 
bfmLCZ = O3CollisionValues(2)	!Lower Crash Zone (LCZ) - BFM
bfmUCZ = O3CollisionValues(3)	!Upper Crash Zone (UCZ) - BFM
qcmLCZ = O3CollisionValues(4)	!Lower Crash Zone (LCZ) - QCM
qcmUCZ = O3CollisionValues(5)	!Upper Crash Zone (UCZ) - QCM
xfrLCZ = O3CollisionValues(6)	!Lower Crash Zone (LCZ) - Transfer (robot)
xfrUCZ = O3CollisionValues(7)	!Upper Crash Zone (UCZ) - Transfer (robot)
!copy values to "previous values" for checking changes from IOC Magellan
mshLCZ_old = mshLCZ	
mshUCZ_old = mshUCZ	
bfmLCZ_old = bfmLCZ	
bfmUCZ_old = bfmUCZ	
qcmLCZ_old = qcmLCZ	
qcmUCZ_old = qcmUCZ	
xfrLCZ_old = xfrLCZ	
xfrUCZ_old = xfrUCZ	
!check is values are all zero (new controller), and if so, update to default
IF mshLCZ=0 & mshUCZ=0 & bfmLCZ=0 & bfmUCZ=0 & qcmLCZ=0 & qcmUCZ=0 & xfrLCZ=0 & xfrUCZ=0 !a new controller will have all zeros; this initializes values to defaults.
	mshLCZ = 0.3	!Lower Crash Zone (LCZ) - main shutter
	mshUCZ = 1.6	!Upper Crash Zone (UCZ) - main shutter 
	bfmLCZ = 0.7	!Lower Crash Zone (LCZ) - BFM
	bfmUCZ = 3.7	!Upper Crash Zone (UCZ) - BFM
	qcmLCZ = 1.0	!Lower Crash Zone (LCZ) - QCM
	qcmUCZ = 3.4	!Upper Crash Zone (UCZ) - QCM
	xfrLCZ = 0.3	!Lower Crash Zone (LCZ) - Transfer (robot)
	xfrUCZ = 2.0	!Upper Crash Zone (UCZ) - Transfer (robot)
	WRITE O3CollisionValues, O3CollisionValues	!write to persistent memory
END



int iAxisIndex
!map the PID gains to MB. onetime mapping when software starts
iAxisIndex = 0
Loop MAXNUMOFAXIS
	tmpSLVKP(iAxisIndex) = SLVKP(iAxisIndex)					!servo velocity loop Kp
	tmpSLVKI(iAxisIndex) = SLVKI(iAxisIndex)					!servo velocity loop Ki
	tmpSLPKP(iAxisIndex) = SLPKP(iAxisIndex)					!servo position loop Kp
	iAxisIndex = iAxisIndex + 1
END

VKP0 = tmpSLVKP(0)					!servo velocity loop Kp
VKI0 = tmpSLVKI(0)					!servo velocity loop Ki
PKP0 = tmpSLPKP(0)					!servo position loop Kp

VKP1 = tmpSLVKP(1)					!servo velocity loop Kp
VKI1 = tmpSLVKI(1)					!servo velocity loop Ki
PKP1 = tmpSLPKP(1)					!servo position loop Kp

VKP2 = tmpSLVKP(2)					!servo velocity loop Kp
VKI2 = tmpSLVKI(2)					!servo velocity loop Ki
PKP2 = tmpSLPKP(2)					!servo position loop Kp

VKP3 = tmpSLVKP(3)					!servo velocity loop Kp
VKI3 = tmpSLVKI(3)					!servo velocity loop Ki
PKP3 = tmpSLPKP(3)					!servo position loop Kp

VKP4 = tmpSLVKP(4)					!servo velocity loop Kp
VKI4 = tmpSLVKI(4)					!servo velocity loop Ki
PKP4 = tmpSLPKP(4)					!servo position loop Kp

VKP5 = tmpSLVKP(5)					!servo velocity loop Kp
VKI5 = tmpSLVKI(5)					!servo velocity loop Ki
PKP5 = tmpSLPKP(5)					!servo position loop Kp

VKP6 = tmpSLVKP(6)					!servo velocity loop Kp
VKI6 = tmpSLVKI(6)					!servo velocity loop Ki
PKP6 = tmpSLPKP(6)					!servo position loop Kp

VKP7 = tmpSLVKP(7)					!servo velocity loop Kp
VKI7 = tmpSLVKI(7)					!servo velocity loop Ki
PKP7 = tmpSLPKP(7)					!servo position loop Kp

! seraching for special axis, LL, CARZ,
Global int bFoundLLELE, bFoundLLELE2
bFoundLLELE =0
bFoundLLELE2 =0
M2G_CARZ = 0 
M2C_CARZ = 0
M2C_CARZ2 = 0
iAxisIndex = 0

!CAR-MZ	4008
!LL-MZ	5011
!ST-MZ	5012
MZ_CARAxisNum = -1
MZ_LLAxisNum = -1
MZ_STAxisNum = -1
Loop MAXNUMOFAXIS

	if iType(iAxisIndex) = 4008	!CAR-MZ	4008
		MZ_CARAxisNum = iAxisIndex
	end	
	if iType(iAxisIndex) = 5011	!LL-MZ	5011
		MZ_LLAxisNum = iAxisIndex
	end
	if iType(iAxisIndex) = 5012	!ST-MZ	5012
		MZ_STAxisNum = iAxisIndex
	end

	! initialize the move interlock array to 0 for each axis
	iPreventMove(iAxisIndex) = 0
	! ozone delivery injector (inj) is axis type 5007. This line looks at the presence of 5007 on either axis 0 or 1 of the DMC
	IF (iType(0)= 5007 | iType(1)= 5007) M2G_InjPresent = 1 ELSE M2G_InjPresent = 0 END	!This flag has several purposes, one of which is starting buffers 11 and 12 for ozone control and interlocks.
	IF (M2G_CARZ = 0)&(iType(iAxisIndex)= 5008 | iType(iAxisIndex)= 5009 | iType(iAxisIndex)= 5010) 
		M2G_CARZ = 1
		CARZ_AxisNum = iAxisIndex
!	ELSE
!		M2G_CARZ = 0
	END	!This flag has several purposes, one of which is starting buffers 11 and 12 for ozone control and interlocks.

	IF (M2C_CARZ = 0)&(iType(iAxisIndex)= 5018 | iType(iAxisIndex)= 5019 | iType(iAxisIndex)= 5020) 
		M2C_CARZ = 1
		CARZ_PM_AxisNum = iAxisIndex
	END

	IF (M2C_CARZ2 = 0)&(iType(iAxisIndex)= 5028 | iType(iAxisIndex)= 5029 | iType(iAxisIndex)= 5030) 
		M2C_CARZ2 = 1
		CARZ_PM2_AxisNum = iAxisIndex
	END

	if (bFoundLLELE=1)			!If found the first LLELE axis, search for the second one. Two brake axis maximum 
		if ((iType(iAxisIndex) =5004)|(iType(iAxisIndex) =5006)|(iType(iAxisIndex) =5011))
	 		LLELEAXIS2 = iAxisIndex
			bFoundLLELE2 = 1
			MFLAGS(iAxisIndex).23=1			!brake
			SETCONF(29, iAxisIndex*10000+7,2)	!2nd brake is on bit 7
		end
	elseif((iType(iAxisIndex) =5004)|(iType(iAxisIndex) =5006)|(iType(iAxisIndex) =5011))	!Find the first LLELE axis 
 		LLELEAXIS = iAxisIndex
		bFoundLLELE = 1
		MFLAGS(iAxisIndex).23=1			!brake
		SETCONF(29, iAxisIndex*10000+6,2)	!1st brake is on bit 6
	end
	if(iType(iAxisIndex)=5001)|(iType(iAxisIndex)=5002)|(iType(iAxisIndex)=5003)
		MFLAGS(iAxisIndex).23=1			!brake
		if CONTROLLERMODELNUM=6
	!		SETCONF(29, iAxisIndex*10000+9,2)	!LP brake is always on bit 7, brake #2, commited out during FBH upgrade project Prep2 controller. not required
		else
			SETCONF(29, iAxisIndex*10000+7,2)	!LP brake is always on bit 7, brake #2
		end
		!record the first harware interlocked QCM/BFM arm axis number
		if (HW_Intlkd_BFMQCM_AxisNum1 = -1) & (iInterlockEnabled(iAxisIndex))
			HW_Intlkd_BFMQCM_AxisNum1 = iAxisIndex
			Start 15, 1	!start buffer 15 to send the AtHome bit to GM controller
		end
		!record the second harware interlocked QCM/BFM arm axis number
		if (HW_Intlkd_BFMQCM_AxisNum1 > -1) & (HW_Intlkd_BFMQCM_AxisNum2 = -1) & (iInterlockEnabled(iAxisIndex))
			HW_Intlkd_BFMQCM_AxisNum2 = iAxisIndex
		end
	end	
	
	iAxisGood(iAxisIndex) = 0
	!Initialize the QCM limits (for all axes) to the corresponding dPositiveLimit value
	!If there is a QCM, these limits will change based on interlock conditions (see buffer 18)
	dQcmLimit(iAxisIndex) = dPositiveLimit(iAxisIndex)
	adjNegativeLimit(iAxisIndex) = dNegtiveLimit(iAxisIndex) !!!CJR
	adjPositiveLimit(iAxisIndex) = dPositiveLimit(iAxisIndex) !!!CJR	
	!load parameters from each axis
	!iSpChecked(i) = 0
	if iType(iAxisIndex)>0
		if iType(iAxisIndex)<2000		!AVP
			VEL(iAxisIndex) = dVel(iAxisIndex)*dAxisPitch(iAxisIndex)	!mil (0.001in) to deg ZW 12/28/15
			XVEL(iAxisIndex) = dVmax(iAxisIndex)*dAxisPitch(iAxisIndex)	!mil (0.001in) to deg
			ACC(iAxisIndex) = dAmax(iAxisIndex)*dAxisPitch(iAxisIndex)
			DEC(iAxisIndex) = dAmax(iAxisIndex)*dAxisPitch(iAxisIndex)
			KDEC(iAxisIndex)=2*DEC(iAxisIndex)
			SRLIMIT(iAxisIndex) = dPositiveLimit(iAxisIndex)*dAxisPitch(iAxisIndex)	!mil
			SLLIMIT(iAxisIndex) = dNegtiveLimit(iAxisIndex)*dAxisPitch(iAxisIndex)
			FMASK(iAxisIndex).#SLL = 1
			FMASK(iAxisIndex).#SRL = 1
		elseif iType(iAxisIndex)>2000 & iType(iAxisIndex)<3000		!Index
			VEL(iAxisIndex) = dVel(iAxisIndex)	!ZW 5/14/19
			XVEL(iAxisIndex) = dVmax(iAxisIndex)
			ACC(iAxisIndex) = dAmax(iAxisIndex)
			DEC(iAxisIndex) = dAmax(iAxisIndex)
			KDEC(iAxisIndex)=2*DEC(iAxisIndex)
			SRLIMIT(iAxisIndex) = dPositiveLimit(iAxisIndex)
			SLLIMIT(iAxisIndex) = dNegtiveLimit(iAxisIndex)
			FMASK(iAxisIndex).#SLL = 1
			FMASK(iAxisIndex).#SRL = 1
		elseif iType(iAxisIndex)>3000 & iType(iAxisIndex)<5000		!GSR and GSP-ROT
			VEL(iAxisIndex) = dVel(iAxisIndex)*6		!ZW 12/28/15
			XVEL(iAxisIndex) = dVmax(iAxisIndex)*6	!rpm to deg
			ACC(iAxisIndex) = dAmax(iAxisIndex)*6		!rpm/s to deg/s^2
			DEC(iAxisIndex) = dAmax(iAxisIndex)*6
			KDEC(iAxisIndex)=2*DEC(iAxisIndex)
			FMASK(iAxisIndex).#SLL = 0
			FMASK(iAxisIndex).#SRL = 0
		elseif iType(iAxisIndex)>5000 & iType(iAxisIndex)<6000		!LE
			MFLAGS(iAxisIndex).23=1			!brake
			VEL(iAxisIndex) = dVel(iAxisIndex)*dAxisPitch(iAxisIndex)	!in to deg ZW 12/28/15
			XVEL(iAxisIndex) = dVmax(iAxisIndex)*dAxisPitch(iAxisIndex)	!in to deg
			ACC(iAxisIndex) = dAmax(iAxisIndex)*dAxisPitch(iAxisIndex)
			DEC(iAxisIndex) = dAmax(iAxisIndex)*dAxisPitch(iAxisIndex)
			KDEC(iAxisIndex)=2*DEC(iAxisIndex)
			SRLIMIT(iAxisIndex) = dPositiveLimit(iAxisIndex)*dAxisPitch(iAxisIndex)	!inch
			SLLIMIT(iAxisIndex) = dNegtiveLimit(iAxisIndex)*dAxisPitch(iAxisIndex)
			FMASK(iAxisIndex).#SLL = 1
			FMASK(iAxisIndex).#SRL = 1
		elseif iType(iAxisIndex)=6009		!Flipper clamping axis
			MFLAGS(iAxisIndex).23=1			!brake
			VEL(iAxisIndex) = dVel(iAxisIndex)/FlipperClampDTI	!in to deg ZW 12/28/15
			XVEL(iAxisIndex) = dVmax(iAxisIndex)/FlipperClampDTI	!in to deg
			ACC(iAxisIndex) = dAmax(iAxisIndex)/FlipperClampDTI
			DEC(iAxisIndex) = dAmax(iAxisIndex)/FlipperClampDTI
			KDEC(iAxisIndex)=2*DEC(iAxisIndex)
			FMASK(iAxisIndex).#SLL = 0
			FMASK(iAxisIndex).#SRL = 0
		else
			VEL(iAxisIndex) = dVel(iAxisIndex)	!ZW 12/28/15
			XVEL(iAxisIndex) = dVmax(iAxisIndex)
			ACC(iAxisIndex) = dAmax(iAxisIndex)
			DEC(iAxisIndex) = dAmax(iAxisIndex)
			KDEC(iAxisIndex)=2*DEC(iAxisIndex)
			FMASK(iAxisIndex).#SLL = 0
			FMASK(iAxisIndex).#SRL = 0

		end

		!XRMS(iAxisIndex)= dFaultTorq(iAxisIndex)		!% of full torque as max available motor torque
		XCURI(iAxisIndex) = dFaultTorq(iAxisIndex) !50% of fault torque as holding torque at idle
		XCURV(iAxisIndex) = dFaultTorq(iAxisIndex)	!100% of fault torque as driving torque in motion
		if iType(iAxisIndex)>6000
			JERK(iAxisIndex)=ACC(iAxisIndex)
		else
			JERK(iAxisIndex)=2*ACC(iAxisIndex)
		end	
	end
	iAxisIndex = iAxisIndex + 1
end

IF (IN(0).5=0) &(bFoundLLELE = 1)
	iLLDoorOpened=1
else
	iLLDoorOpened=0
end
IF (IN(0).7=0) &(bFoundLLELE2 = 1)
	iLLDoorOpened2=1
else
	iLLDoorOpened2=0
end
!Load default velocities (for now).  Eventually this will be managed with the READ and WRITE logic.
ramprate0 = dVel(0)
ramprate1 = dVel(1)
ramprate2 = dVel(2)
ramprate3 = dVel(3)
ramprate4 = dVel(4)
ramprate5 = dVel(5)
ramprate6 = dVel(6)
ramprate7 = dVel(7)

Sp0 = WkSp0	!Measured0	!@@@ set measured to SP at RESET to prevent motion after RESET.
Sp1 = WkSp1	!Measured1
Sp2 = WkSp2	!Measured2
Sp3 = WkSp3	!Measured3
Sp4 = WkSp4	!Measured4
Sp5 = WkSp5	!Measured5
Sp6 = WkSp6	!Measured6
Sp7 = WkSp7	!Measured7

OldSP(0) = Sp0	!@@@ set OldSP to SP at RESET to prevent motion after RESET.
OldSP(1) = Sp1
OldSP(2) = Sp2
OldSP(3) = Sp3
OldSP(4) = Sp4
OldSP(5) = Sp5
OldSP(6) = Sp6
OldSP(7) = Sp7

iExtMoving=0
iRotMoving=0

!if Gen2000 Arm
if iType(0) = 6001
	!ArmExtMax = 48.0
	!ArmExtMin = 0.25
	rRearArmLength = 24.25	!inch
	rFrontArmLength= 24.5	!inch
	rEndEffectorLength = 23.44!23.5! to the center of platen
	rWristHalfWidth = (5.5-5.467)/2.0 !1.5;
	rClearance = 0.5
	rFullyRetracted=20.638!23.75!0.25
	rFullyExtended=69.024!48
	rRotCenterOffset = 3.083667!3.072
	ArmExtMax = rFullyExtended + rRotCenterOffset	!convert robot rotation center to platen center distance to arm rot center to platen center distance
	ArmExtMin = rFullyRetracted + rRotCenterOffset	!convert robot rotation center to platen center distance to arm rot center to platen center distance

elseif iType(0) = 6003	!GEN200
	rRearArmLength = 15.1	!inch
	rFrontArmLength= 18.0	!inch
	rEndEffectorLength = 15.3	! to the center of platen
	rWristHalfWidth = 0
	rClearance = 0.5
	rFullyRetracted=15.13 
	rFullyExtended=45.32
	rRotCenterOffset = 3.072
	ArmExtMax = rFullyExtended + rRotCenterOffset	!convert robot rotation center to platen center distance to arm rot center to platen center distance
	ArmExtMin = rFullyRetracted + rRotCenterOffset	!convert robot rotation center to platen center distance to arm rot center to platen center distance

elseif iType(0) = 6005	!GEN20A
	rRearArmLength = 15.1	!inch
	rFrontArmLength= 14.7	!inch
	rEndEffectorLength = 14	! to the center of platen
	rWristHalfWidth = 1.5
	rClearance = 1.15
	rFullyRetracted=14.16 
	rFullyExtended=41.7
	rRotCenterOffset = 0
	ArmExtMax = rFullyExtended + rRotCenterOffset	!convert robot rotation center to platen center distance to arm rot center to platen center distance
	ArmExtMin = rFullyRetracted + rRotCenterOffset	!convert robot rotation center to platen center distance to arm rot center to platen center distance


elseif iType(0) = 6007	!GEN10
	rRearArmLength = 10.0	!inch
	rFrontArmLength= 9.0	!inch
	rEndEffectorLength = 17.35	! to the center of platen
	rWristHalfWidth = 1.5
	rClearance = 1.15
	rFullyRetracted=16.39 
	rFullyExtended=36.28
	rRotCenterOffset = 0.0
	ArmExtMax = rFullyExtended + rRotCenterOffset	!convert robot rotation center to platen center distance to arm rot center to platen center distance
	ArmExtMin = rFullyRetracted + rRotCenterOffset	!convert robot rotation center to platen center distance to arm rot center to platen center distance
elseif iType(0) = 6009	!Flipper
	rFullyRetracted=dNegtiveLimit(0) 
	rFullyExtended=dPositiveLimit(0)
	rClearance = 20
	ArmExtMax = rFullyExtended 	!convert robot rotation center to platen center distance to arm rot center to platen center distance
	ArmExtMin = rFullyRetracted	!convert robot rotation center to platen center distance to arm rot center to platen center distance
end
if (iType(0) > 6000)&(iType(0) < 6009)
	real tmpLx, tmpTheta1, tmpTheta2, tmpExtRearArmRotCenter
	tmpLx = sqrt(rWristHalfWidth*rWristHalfWidth + (rFullyRetracted-rEndEffectorLength + rRotCenterOffset)*(rFullyRetracted-rEndEffectorLength + rRotCenterOffset))
	tmpExtRearArmRotCenter = rFullyRetracted-rEndEffectorLength + rRotCenterOffset

	if (tmpExtRearArmRotCenter < 0.0)
		tmpTheta1 = 3.141592654- atan (rWristHalfWidth/abs(tmpExtRearArmRotCenter))
	elseif (tmpExtRearArmRotCenter =0.0)
		tmpTheta1 = 3.141592654/2.0
	elseif  (tmpExtRearArmRotCenter > 0.0)
		tmpTheta1 = atan (rWristHalfWidth/tmpExtRearArmRotCenter)	
	end
	tmpTheta2 = acos( (rRearArmLength*rRearArmLength + tmpLx*tmpLx  - rFrontArmLength*rFrontArmLength)/(2*rRearArmLength*tmpLx))
	theta0 = (tmpTheta1+tmpTheta2)*180/3.141592654	!angle betwen rear arm and the c-c line when fully retracted
	rSqLength = rFrontArmLength*rFrontArmLength
	dPositiveLimit(0)= rFullyExtended-rFullyRetracted-0.09	!0.09 is a safety factor added by developer	
end
!if (iType(0) = 6001) | (iType(0) = 6003)
!	rSqLengthDelta = rRearArmLength*rRearArmLength - rFrontArmLength*rFrontArmLength

!end


! if there is a MZ axis on the controller, kick off the buffer 14
If ((MZ_CARAxisNum + MZ_LLAxisNum + MZ_STAxisNum) > -3) & ( PST(14).#RUN = 0)
	Start 14, 1
END


! kick off home pulse process routine
Start 20, 1

! kick off Analog cmd process
Start 22, 1
! kick off the axis modbus commond iiie routine
Start 31, 1
! kick off robot arm modbus commond iiie routine
if iType(0)>6000
	Start 26, 1
end
! kick off QCM interlocks
if iQcmAxis(0) >= 0 & iQcmAxis(0) <= 7
	! kick off Modbus comms with GM controller
!	Start 16, 1
	Start 17, 1
	Start 18, 1
	Buffer17Needed=1
end
if	M2G_CARZ = 1
	Start 17, 1
End

IF M2C_CARZ = 1 | M2C_CARZ2 = 1
	START 13, 1
END

!Ozone injector buffers
if iType(0)= 5007
	! kick off Modbus comms with GM controller
	Start 11, 1
	Start 12, 1
	Start 17, 1
	Buffer11Needed=1
	Buffer17Needed=1
end


wait 2000
! get the initial status of iLLDoorOpened variable for MZ LL
If ((MZ_CARAxisNum + MZ_LLAxisNum + MZ_STAxisNum) > -3) & ( PST(14).#RUN = 1)
	IF (oLL_Door_Closed = 0) &(bFoundLLELE = 1)
		iLLDoorOpened=1
	else
		iLLDoorOpened=0
	end	
END

real tmp, x1, x2, h

!==================================================================
!==================================================================
! If there is a Robot extension axis, it must be on axis 0
! If there is a Robot ration axis, it must be on axis 2
! If there is no robot, axis 0 and 2 can be assigned to other type.
! If there is a LL elevator, it must be on axis 3.
! If there is a GM CAR rotation, it must be on axis 1.
!==================================================================
!==================================================================
AA:

iAxis = 0
if iInverseInterlockLogic(0)
	bInterlocActive = ^IN(0).2
	!bSecondSP = ^IN(0).4
else
	bInterlocActive = IN(0).2
	!bSecondSP = IN(0).4
end

BLOCK
LOOP MAXNUMOFAXIS

if iType(iAxis) <6000
	if (iType(iAxis)> 3000) & (iType(iAxis)< 5000) ! GSR
		real temp
		temp = FPOS(iAxis)-360*FLOOR(FPOS(iAxis)/360)	!0-360 translate
		if(abs(SP(iAxis))<=0.00001) & ((abs(temp)<=0.002)|(abs(temp-360)<=0.002) )
			tmpMeasured(iAxis)= SP(iAxis)
		elseif(abs(SP(iAxis)-360)<=0.00001) & ((abs(temp)<=0.002)|(abs(temp-360)<=0.002) )
			tmpMeasured(iAxis)= SP(iAxis)
		else
			tmpMeasured(iAxis) = temp
		end	
		tmpWkSp(iAxis) = RPOS(iAxis)-360*FLOOR(RPOS(iAxis)/360)
		tmpMaxVel(iAxis) =XVEL(iAxis)/6		!ZW03
		tmpMinVel(iAxis) = -XVEL(iAxis)/6	!ZW03
		tmpAxisDec(iAxis) = DEC(iAxis)/6
		tmpAxisAcc(iAxis) = ACC(iAxis)/6
		tmpFaultTorq(iAxis) = XRMS(iAxis)
		tmpHomeTorq(iAxis) = dHomingTorq(iAxis)			!ZW03
		tmpMaxTorq(iAxis) =XCURV(iAxis)
	elseif (iType(iAxis)> 1000) & (iType(iAxis)< 2000) !AVP
		tmpMeasured(iAxis) = FPOS(iAxis)/dAxisPitch(iAxis)
		tmpWkSp(iAxis) = RPOS(iAxis)/dAxisPitch(iAxis)
		tmpMaxVel(iAxis) =XVEL(iAxis)/dAxisPitch(iAxis)		!ZW03
		tmpMinVel(iAxis) = -XVEL(iAxis)/dAxisPitch(iAxis)	!ZW03
		tmpAxisDec(iAxis) = DEC(iAxis)/dAxisPitch(iAxis)
		tmpAxisAcc(iAxis) = ACC(iAxis)/dAxisPitch(iAxis)
		tmpFaultTorq(iAxis) = XRMS(iAxis)
		tmpHomeTorq(iAxis) = dHomingTorq(iAxis)			!ZW03
		tmpMaxTorq(iAxis) =XCURV(iAxis)
	elseif (iType(iAxis)> 5000) & (iType(iAxis)< 6000) !LE
		tmpMeasured(iAxis) = FPOS(iAxis)/dAxisPitch(iAxis)
		tmpWkSp(iAxis) = RPOS(iAxis)/dAxisPitch(iAxis)
		tmpMaxVel(iAxis) =XVEL(iAxis)/dAxisPitch(iAxis)		!ZW03
		tmpMinVel(iAxis) = -XVEL(iAxis)/dAxisPitch(iAxis)	!ZW03
		tmpAxisDec(iAxis) = DEC(iAxis)/dAxisPitch(iAxis)
		tmpAxisAcc(iAxis) = ACC(iAxis)/dAxisPitch(iAxis)
		tmpFaultTorq(iAxis) = XRMS(iAxis)
		tmpHomeTorq(iAxis) = dHomingTorq(iAxis)			!ZW03
		tmpMaxTorq(iAxis) =XCURV(iAxis)
	else
		tmpMeasured(iAxis) = FPOS(iAxis)
		tmpWkSp(iAxis) = RPOS(iAxis)
		tmpMaxVel(iAxis) =VEL(iAxis)
		tmpMinVel(iAxis) = -VEL(iAxis)
		tmpAxisDec(iAxis) = DEC(iAxis)
		tmpAxisAcc(iAxis) = ACC(iAxis)
		tmpFaultTorq(iAxis) = XRMS(iAxis)
		tmpHomeTorq(iAxis) = dHomingTorq(iAxis)			!ZW03
		tmpMaxTorq(iAxis) =XCURV(iAxis)
	end
	if (DINPUT0.5) & (iType(iAxis)= 6009)&(FRobot(0)>1.0) ! reset the bit, if flipper is unclampped
		DINPUT0.5=0
	end


!update axis information to GUI
	AxisLog(iAxis)(0) = TIME
	AxisLog(iAxis)(1) = SP_Modbus(iAxis)
	AxisLog(iAxis)(2) = tmpMeasured(iAxis)


	!AxisInfo(iAxis)(0) = AxisInfo(iAxis)(0) - (LPF_Beta * (AxisInfo(iAxis)(0) - tmpMeasured(iAxis)));

	AxisInfo(iAxis)(0) = tmpMeasured(iAxis)		!Position
	AxisInfo(iAxis)(1) = FVEL(iAxis)		!Velocity
	AxisInfo(iAxis)(2) = FACC(iAxis)		!Acc
	AxisInfo(iAxis)(3) = MST(iAxis).0		!0: motor is disabled 1 : motor is enabled.
	AxisInfo(iAxis)(4) = MFLAGS(iAxis).3	!Homed ? 0, Not done, 1, done
	AxisInfo(iAxis)(5) = MST(iAxis).4		!0 : Motor is not moving and has reached the target position(see variables TARGRAD and SETTLE)
											!1 : Motor is moving or is out of range
											! CR I think the correct status is: 1 = in-position; 0 = moving

	AxisInfo(iAxis)(6) = FAULT(iAxis).12	!POSITION ERROR
	AxisInfo(iAxis)(7) = FAULT(iAxis).9		!Drive error
	AxisInfo(iAxis)(8) = FAULT(iAxis).5|FAULT(iAxis).0		!S / W P limit
	AxisInfo(iAxis)(9) = FAULT(iAxis).6|FAULT(iAxis).1		!S / W N limit
	AxisInfo(iAxis)(10) = FAULT(iAxis).7	!Encoder disconnected
	AxisInfo(iAxis)(11) = SP_Modbus(iAxis)
	AxisInfo(iAxis)(12) = OldSP(iAxis)
	AxisInfo(iAxis)(13) = tmpWkSp(iAxis)
	AxisInfo(iAxis)(14) = RampRate(iAxis)
	AxisInfo(iAxis)(15) = dCE(iAxis)
	AxisInfo(iAxis)(16) = MERR(iAxis)
	iAxisGood(iAxis) = AxisInfo(iAxis)(3)*AxisInfo(iAxis)(4)*^(AxisInfo(iAxis)(7))*^(AxisInfo(iAxis)(10))
	if (^iAxisGood(iAxis)) & (iControlProcessState(iAxis) = 10)
		iControlProcessState(iAxis) = 0
	end
end	
	iAxis = iAxis + 1

end ! End of LOOP
	AxisInfo(0)(17) = KillTraj0
	AxisInfo(1)(17) = KillTraj1
	AxisInfo(2)(17) = KillTraj2
	AxisInfo(3)(17) = KillTraj3
	AxisInfo(4)(17) = KillTraj4
	AxisInfo(5)(17) = KillTraj5
	AxisInfo(6)(17) = KillTraj6
	AxisInfo(7)(17) = KillTraj7

! update Modbus registers below
	A0_Enabled = AxisInfo(0)(3)
	A0_Homed = AxisInfo(0)(4)
	A0_NegLimit = AxisInfo(0)(6)
	A0_DriveError = AxisInfo(0)(7)
	A0_EncoderDisconnected = AxisInfo(0)(10)
	
	A2_Enabled = AxisInfo(2)(3)
	A2_Homed = AxisInfo(2)(4)
	A2_NegLimit = AxisInfo(2)(6)
	A2_DriveError = AxisInfo(2)(7)
	A2_EncoderDisconnected = AxisInfo(2)(10)
	if iType(0) <6000
		!Measured0 = Measured0 - (LPF_Beta * (Measured0 - tmpMeasured(0)));
		Measured0 = tmpMeasured(0)
		WkSp0 = tmpWkSp(0)
		MaxVel0 = tmpMaxVel(0)
		MinVel0 = tmpMinVel(0)
		AxisDec0 = tmpAxisDec(0)
		AxisAcc0 = tmpAxisAcc(0)
		FaultTorq0 = tmpFaultTorq(0)
		HomeTorq0 = tmpHomeTorq(0)
		MaxTorq0 = tmpMaxTorq(0)
		if (abs(dPositiveLimit(0)-dNegtiveLimit(0))>0.00001)
			OUTP0= 100.0*abs(Measured0-dNegtiveLimit(0))/abs(dPositiveLimit(0)-dNegtiveLimit(0))
		else	
			OUTP0=0.0
		end
	end


	if iType(1) <6000
		if iType(1) = 1040
			if (abs(dPositiveLimit(1)-dNegtiveLimit(1))>0.00001)
				OUTP1= 100.0*abs(tmpMeasured(1)-dNegtiveLimit(1))/abs(dPositiveLimit(1)-dNegtiveLimit(1))
				WkSp1= 100.0*abs(tmpWkSp(1)-dNegtiveLimit(1))/abs(dPositiveLimit(1)-dNegtiveLimit(1))
			else	
				OUTP1=0.0
				WkSp1=0.0
			end
			Measured1 = OUTP1
		else
			Measured1 = tmpMeasured(1)
			WkSp1 = tmpWkSp(1)
			if (abs(dPositiveLimit(1)-dNegtiveLimit(1))>0.00001)
				OUTP1= 100.0*abs(Measured1-dNegtiveLimit(1))/abs(dPositiveLimit(1)-dNegtiveLimit(1))
			else	
				OUTP1=0.0
			end
		end
		MaxVel1 = tmpMaxVel(1)
		MinVel1 = tmpMinVel(1)
		AxisDec1 = tmpAxisDec(1)
		AxisAcc1 = tmpAxisAcc(1)
		FaultTorq1 = tmpFaultTorq(1)
		HomeTorq1 = tmpHomeTorq(1)
		MaxTorq1 = tmpMaxTorq(1)
	end

	if iType(2) <6000

		!Measured2 = Measured2 - (LPF_Beta * (Measured2 - tmpMeasured(2)));
		Measured2 = tmpMeasured(2)
		WkSp2 = tmpWkSp(2)
		MaxVel2 = tmpMaxVel(2)
		MinVel2 = tmpMinVel(2)
		AxisDec2 = tmpAxisDec(2)
		AxisAcc2 = tmpAxisAcc(2)
		FaultTorq2 = tmpFaultTorq(2)
		HomeTorq2 = tmpHomeTorq(2)
		MaxTorq2 = tmpMaxTorq(2)
		if (abs(dPositiveLimit(2)-dNegtiveLimit(2))>0.00001)
			OUTP2= 100.0*abs(Measured2-dNegtiveLimit(2))/abs(dPositiveLimit(2)-dNegtiveLimit(2))
		else	
			OUTP2=0.0
		end
	end
	Measured3 = tmpMeasured(3)
	WkSp3 = tmpWkSp(3)
	MaxVel3 = tmpMaxVel(3)
	MinVel3 = tmpMinVel(3)
	AxisDec3 = tmpAxisDec(3)
	AxisAcc3 = tmpAxisAcc(3)
	FaultTorq3 = tmpFaultTorq(3)
	HomeTorq3 = tmpHomeTorq(3)
	MaxTorq3 = tmpMaxTorq(3)
	if (abs(dPositiveLimit(3)-dNegtiveLimit(3))>0.00001)
		OUTP3= 100.0*abs(Measured3-dNegtiveLimit(3))/abs(dPositiveLimit(3)-dNegtiveLimit(3))
	else	
		OUTP3=0.0
	end

	Measured4 = tmpMeasured(4)
	WkSp4 = tmpWkSp(4)
	MaxVel4 = tmpMaxVel(4)
	MinVel4 = tmpMinVel(4)
	AxisDec4 = tmpAxisDec(4)
	AxisAcc4 = tmpAxisAcc(4)
	FaultTorq4 = tmpFaultTorq(4)
	HomeTorq4 = tmpHomeTorq(4)
	MaxTorq4 = tmpMaxTorq(4)
	if (abs(dPositiveLimit(4)-dNegtiveLimit(4))>0.00001)
		OUTP4= 100.0*abs(Measured4-dNegtiveLimit(4))/abs(dPositiveLimit(4)-dNegtiveLimit(4))
	else	
		OUTP4=0.0
	end


	Measured5 = tmpMeasured(5)
	WkSp5 = tmpWkSp(5)
	MaxVel5 = tmpMaxVel(5)
	MinVel5 = tmpMinVel(5)
	AxisDec5 = tmpAxisDec(5)
	AxisAcc5 = tmpAxisAcc(5)
	FaultTorq5 = tmpFaultTorq(5)
	HomeTorq5 = tmpHomeTorq(5)
	MaxTorq5 = tmpMaxTorq(5)
	if (abs(dPositiveLimit(5)-dNegtiveLimit(5))>0.00001)
		OUTP5= 100.0*abs(Measured5-dNegtiveLimit(5))/abs(dPositiveLimit(5)-dNegtiveLimit(5))
	else	
		OUTP5=0.0
	end



	Measured6 = tmpMeasured(6)
	WkSp6 = tmpWkSp(6)
	MaxVel6 = tmpMaxVel(6)
	MinVel6 = tmpMinVel(6)
	AxisDec6 = tmpAxisDec(6)
	AxisAcc6 = tmpAxisAcc(6)
	FaultTorq6 = tmpFaultTorq(6)
	HomeTorq6 = tmpHomeTorq(6)
	MaxTorq6 = tmpMaxTorq(6)
	if (abs(dPositiveLimit(6)-dNegtiveLimit(6))>0.00001)
		OUTP6= 100.0*abs(Measured6-dNegtiveLimit(6))/abs(dPositiveLimit(6)-dNegtiveLimit(6))
	else	
		OUTP6=0.0
	end



	Measured7 = tmpMeasured(7)
	WkSp7 = tmpWkSp(7)
	MaxVel7 = tmpMaxVel(7)
	MinVel7 = tmpMinVel(7)
	AxisDec7 = tmpAxisDec(7)
	AxisAcc7 = tmpAxisAcc(7)
	FaultTorq7 = tmpFaultTorq(7)
	HomeTorq7 = tmpHomeTorq(7)
	MaxTorq7 = tmpMaxTorq(7)
	if (abs(dPositiveLimit(7)-dNegtiveLimit(7))>0.00001)
		OUTP7= 100.0*abs(Measured7-dNegtiveLimit(7))/abs(dPositiveLimit(7)-dNegtiveLimit(7))
	else	
		OUTP7=0.0
	end




	
END !OF BLOCK

	!Update analog output if analog cmd enalbed
IF UseAnalogCmd(0) = 1
	BLOCK
		if dPositiveLimit(0) > 0.01
			PosMeasured(0) = 100.0*Measured0/dPositiveLimit(0)  
		else
			PosMeasured(0) = 0
		end	
		if PosMeasured(0) >100
			PosMeasured(0) =100
		elseif PosMeasured(0) <-100
			PosMeasured(0) = -100
		end
		AOUT(0) = PosMeasured(0)
	END
	BLOCK
		if dPositiveLimit(1) > 0.01
			PosMeasured(1) = 100*Measured1/dPositiveLimit(1)  
		else
			PosMeasured(1) = 0
		end	
		if PosMeasured(1) >100
			PosMeasured(1) =100
		elseif PosMeasured(1) <-100
			PosMeasured(1) = -100
		end
		AOUT(1) = PosMeasured(1)
	end
	BLOCK
		if dPositiveLimit(2) > 0.01
			PosMeasured(2) = 100*Measured2/dPositiveLimit(2)  
		else
			PosMeasured(2) = 0
		end	
		if PosMeasured(2) >100
			PosMeasured(2) =100
		elseif PosMeasured(2) <-100
			PosMeasured(2) = -100
		end
		AOUT(2) = PosMeasured(2)
	end
	BLOCK
		if dPositiveLimit(3) > 0.01
			PosMeasured(3) = 100*Measured3/dPositiveLimit(3)  
		else
			PosMeasured(3) = 0
		end	
		if PosMeasured(3) >100
			PosMeasured(3) =100
		elseif PosMeasured(3) <-100
			PosMeasured(3) = -100
		end
		AOUT(3) = PosMeasured(3)
	end
end ! if analog

	
	
	BLOCK
		!Map DINPUT bits to Lot Manager	!CR2016-01-08 ZW 10/27/2020 
		
		if(iType(0)>=6000)		!only for robot and flipper. Robot Home/Platen present sensor	1=present; 0=clear
			DINPUT2.2 = IN(0).0	
		end
		if(bFoundLLELE = 1)
			if (LLELEAXIS=0)
				DINPUT0.1 = ^IN(0).5	!LL1 door switch					In LM, 1=open; 0=closed
			elseif (LLELEAXIS=1)
				DINPUT1.1 = ^IN(0).5	!LL1 door switch					In LM, 1=open; 0=closed
			elseif (LLELEAXIS=2)
				DINPUT2.1 = ^IN(0).5	!LL1 door switch					In LM, 1=open; 0=closed
			elseif (LLELEAXIS=3)
				DINPUT3.1 = ^IN(0).5	!LL1 door switch					In LM, 1=open; 0=closed
			elseif (LLELEAXIS=4)
				DINPUT4.1 = ^IN(0).5	!LL1 door switch					In LM, 1=open; 0=closed
			elseif (LLELEAXIS=5)
				DINPUT5.1 = ^IN(0).5	!LL1 door switch					In LM, 1=open; 0=closed
			elseif (LLELEAXIS=6)
				DINPUT6.1 = ^IN(0).5	!LL1 door switch					In LM, 1=open; 0=closed
			elseif (LLELEAXIS=7)
				DINPUT7.1 = ^IN(0).5	!LL1 door switch					In LM, 1=open; 0=closed
			end
		end
		if(bFoundLLELE2 = 1)
			if (LLELEAXIS2=0)
				DINPUT0.1 = ^IN(0).7	!LL1 door switch					In LM, 1=open; 0=closed
			elseif (LLELEAXIS2=1)
				DINPUT1.1 = ^IN(0).7	!LL1 door switch					In LM, 1=open; 0=closed
			elseif (LLELEAXIS2=2)
				DINPUT2.1 = ^IN(0).7	!LL1 door switch					In LM, 1=open; 0=closed
			elseif (LLELEAXIS2=3)
				DINPUT3.1 = ^IN(0).7	!LL1 door switch					In LM, 1=open; 0=closed
			elseif (LLELEAXIS2=4)
				DINPUT4.1 = ^IN(0).7	!LL1 door switch					In LM, 1=open; 0=closed
			elseif (LLELEAXIS2=5)
				DINPUT5.1 = ^IN(0).7	!LL1 door switch					In LM, 1=open; 0=closed
			elseif (LLELEAXIS2=6)
				DINPUT6.1 = ^IN(0).7	!LL1 door switch					In LM, 1=open; 0=closed
			elseif (LLELEAXIS2=7)
				DINPUT7.1 = ^IN(0).7	!LL1 door switch					In LM, 1=open; 0=closed
			end

		end

		CARHM4.0 = FtdCarHomeSW.4
		CARHM1.0 = FtdCarHomeSW.1
		CARHM2.0 = IN(0).0
		!Map all axis DINPUT bits	!ZW2016-08-04
		if(iType(0)>=4004)&(iType(0)<=4007) !GEN20A, HVM Only
			DINPUT0.0 = FtdCarHomeSW.0
		else
			DINPUT0.0 = IN(0).0		!Axis 0 (Molly 1) home swtich	1=present; 0=clear
		end

		if(iType(1)>=4004)&(iType(1)<=4007) !GEN20A, HVL and GEN10 Only
			DINPUT1.0 = FtdCarHomeSW.1
		elseif (iType(0)=6009)	
			DINPUT1.0 = ^IN(0).0		!Axis 0 (Molly 1) home swtich	1=present; 0=clear
			DINPUT1.7 = ^IN(0).3		!Axis 0 (Molly 1) home swtich	1=present; 0=clear
		else		
			DINPUT1.0 = IN(0).1		!Axis 1 (Molly 3) home swtich	1=present; 0=clear
		end
		if(iType(2)>=4004)&(iType(2)<=4007) !GEN20A, HVL and GEN10 Only
			DINPUT2.0 = FtdCarHomeSW.2
		else	
			DINPUT2.0 = IN(0).2		!Axis 2 (Molly 2) home swtich	1=present; 0=clear
		end
		if(iType(3)>=4004)&(iType(3)<=4007) !GEN20A, HVL and GEN10 Only
			DINPUT3.0 = FtdCarHomeSW.3
		!elseif (iType(0)=6009)	
		!	DINPUT3.0 = ^IN(0).3		!Axis 0 (Molly 1) home swtich	1=present; 0=clear
		else	
			DINPUT3.0 = IN(0).3		!Axis 3 (Molly 4) home swtich	1=present; 0=clear
		end
		if(iType(4)>=4004)&(iType(4)<=4007) !GEN20A, HVL and GEN10 Only
			DINPUT4.0 = FtdCarHomeSW.4
		else	
			DINPUT4.0 = IN(0).4		!Axis 4 (Molly 5) home swtich	1=present; 0=clear
		end
		if(iType(5)>=4004)&(iType(5)<=4007) !GEN20A, HVL and GEN10 Only
			DINPUT5.0 = FtdCarHomeSW.5
		else	
			DINPUT5.0 = IN(0).5		!Axis 5 (Molly 7) home swtich	1=present; 0=clear
		end
		if(iType(6)>=4004)&(iType(6)<=4007) !GEN20A, HVL and GEN10 Only
			DINPUT6.0 = FtdCarHomeSW.6
		else	
			DINPUT6.0 = IN(0).6		!Axis 6 (Molly 6) home swtich	1=present; 0=clear
		end
		if(iType(7)>=4004)&(iType(7)<=4007) !GEN20A, HVL and GEN10 Only
			DINPUT7.0 = FtdCarHomeSW.7
		else	
			DINPUT7.0 = IN(0).7		!Axis 7 (Molly 8) home swtich	1=present; 0=clear
		end
		iTmpIndex=0
		Loop MAXNUMOFAXIS
			tmpDINPUT.iTmpIndex =MFLAGS(iTmpIndex).#HOME
			iTmpIndex=iTmpIndex+1
		end
		DINPUT0.3 = tmpDINPUT.0
		DINPUT1.3 = tmpDINPUT.1
		DINPUT2.3 = tmpDINPUT.2
		DINPUT3.3 = tmpDINPUT.3
		DINPUT4.3 = tmpDINPUT.4
		DINPUT5.3 = tmpDINPUT.5
		DINPUT6.3 = tmpDINPUT.6
		DINPUT7.3 = tmpDINPUT.7
		!2/10/2019 start
		DINPUT0.8 = ^iAxisGood(0)
		DINPUT1.8 = ^iAxisGood(1)
		DINPUT2.8 = ^iAxisGood(2)
		DINPUT3.8 = ^iAxisGood(3)
		DINPUT4.8 = ^iAxisGood(4)
		DINPUT5.8 = ^iAxisGood(5)
		DINPUT6.8 = ^iAxisGood(6)
		DINPUT7.8 = ^iAxisGood(7)

		if (iType(1)=1040)&(FPOS(1)>30)
			DINPUT0.9 = 0 !valve opened
		else	
			DINPUT0.9 = 1 !valve close
		end
		!2/10/2019 end
		iTmpIndex=0
		Loop MAXNUMOFAXIS
			tmpDINPUT.iTmpIndex =MST(iTmpIndex).4
			iTmpIndex=iTmpIndex+1
		end
		!axis interlock config
		DINPUT0.10 = iInterlockEnabled(0)
		if(iType(0)> 6000)
			if (CONTROLLERMODELNUM=6) !DMC
				DINPUT1.10 = ^bTeachMode
				DINPUT2.10 = iInterlockEnabled(2)
			else	
				DINPUT1.10 = iInterlockEnabled(1)
				DINPUT2.10 = ^bTeachMode
			end	
		else
			DINPUT1.10 = iInterlockEnabled(1)
			DINPUT2.10 = iInterlockEnabled(2)
		end
		DINPUT3.10 = iInterlockEnabled(3)
		DINPUT4.10 = iInterlockEnabled(4)
		DINPUT5.10 = iInterlockEnabled(5)
		DINPUT6.10 = iInterlockEnabled(6)
		DINPUT7.10 = iInterlockEnabled(7)

		
		if(iType(0)> 6000)
			if CONTROLLERMODELNUM=6 !DMC
				DINPUT0.4 = ^ExecutingMoveExt	!flipper clamping
				DINPUT1.4 = ^ExecutingMoveRot	!flipper rotation
				DINPUT2.4 = tmpDINPUT.2
				
			else	!MAC
				DINPUT0.4 = ^ExecutingMoveExt	!robot extension
				DINPUT1.4 = tmpDINPUT.1
				DINPUT2.4 = ^ExecutingMoveRot	!robot rotation
				DINPUT2.10 = ^bTeachMode
			end
		else
			DINPUT0.4 = tmpDINPUT.0
			DINPUT1.4 = tmpDINPUT.1
			DINPUT2.4 = tmpDINPUT.2
		end
		
		DINPUT3.4 = tmpDINPUT.3
		DINPUT4.4 = tmpDINPUT.4
		DINPUT5.4 = tmpDINPUT.5
		DINPUT6.4 = tmpDINPUT.6
		DINPUT7.4 = tmpDINPUT.7
		if(iType(0)=6009)
			DINPUT1.6 = IN(0).3	! flipper at 180? sensor
		end
	END

	ControlProcessState0 = iControlProcessState(0)
	ControlProcessState1 = iControlProcessState(1)
	ControlProcessState2 = iControlProcessState(2)
	ControlProcessState3 = iControlProcessState(3)
	ControlProcessState4 = iControlProcessState(4)
	ControlProcessState5 = iControlProcessState(5)
	ControlProcessState6 = iControlProcessState(6)
	ControlProcessState7 = iControlProcessState(7)
	
	WAIT 10

	!The next section checks if the communication buffers (11, 16, 17) are running
	!If comms is lost, the buffer freezes.  This check stops and restarts the buffer if frozen
	IF TimerON=0
		Buffer11RunningFlag=0		!Buffer 11 for Meidensha (Keyence) PLC
		Buffer16RunningFlag=0		!Buffer 16 for Wago PLC
		Buffer17RunningFlag=0		!Buffer 17 for Beckhoff PLC
		StartTime=TIME
		TimerON=1
	END

	ElapsedTime=TIME-StartTime

	IF ElapsedTime > 10000
		IF Buffer11RunningFlag=0 & Buffer11Needed=1
			!STOP 11
			!wait 2000
			!START 11,1
			!wait 2000
		END	
		IF Buffer17RunningFlag=0 & Buffer17Needed=1
			!STOP 17
			!wait 2000
			!START 17,1
			!wait 2000
		END
		TimerON=0
	END	


GOTO AA
STOP
!=========================================================================================
!=========================================================================================
!==================End of program body====================================================
!=========================================================================================
!=========================================================================================
!===begin autoroutines===
! Left limit
ON S_FAULT.#SLL=1 
BLOCK
	AxisNum = 0
	Loop MAXNUMOFAXIS
		if FAULT(AxisNum).#SLL=1 & RVEL(AxisNum)<0
			KILL(AxisNum)
			DISP "Left limit, axis: ", AxisNum
			iControlProcessState(AxisNum) = 109
		end
	AxisNum = AxisNum + 1
	end
END
RET


! right limit
ON S_FAULT.#SRL=1 
BLOCK
	AxisNum = 0
	Loop MAXNUMOFAXIS
		if FAULT(AxisNum).#SRL=1 & RVEL(AxisNum)>0
			KILL(AxisNum)
			DISP "Right limit, axis: ", AxisNum
			iControlProcessState(AxisNum) = 110
		end
	AxisNum = AxisNum + 1
	end
END	
RET


!Clear homed bit if one robot arm is disabled
ON (iType(0)>6000)& ((MST(ROBOTAXISNUMH).0=0) | (MST(ROBOTAXISNUML).0=0)) 	
	KILL (ROBOTAXISNUMH)
	KILL (ROBOTAXISNUML)
	DISABLE (ROBOTAXISNUMH)
	DISABLE (ROBOTAXISNUML)
	MFLAGS(ROBOTAXISNUMH).3 = 0
	MFLAGS(ROBOTAXISNUML).3 = 0	
	iControlProcessState(ROBOTAXISNUMH) = 0
	iControlProcessState(ROBOTAXISNUML) = 0
	DISP "Robot home bit cleared, rehoming"
RET


! Critical position error - kill and move current position into SPs

ON S_FAULT.#CPE=1 
	if (iType(ROBOTAXISNUMH)>6000)& ((FAULT(ROBOTAXISNUMH).#CPE=1) | (FAULT(ROBOTAXISNUML).#CPE=1)) 
		BLOCK
			KILL(ROBOTAXISNUMH, ROBOTAXISNUML)		
			iControlProcessState(ROBOTAXISNUMH) = 111
			iControlProcessState(ROBOTAXISNUML) = 111
			DISP "Robot stopped, positioning error"
			Sp0 = WkSp0	!Measured0
			OldSP(0) = Sp0
			SP(0) = Sp0
			SP_RobotExt =Sp0
			RobotExtTargetPrev = Sp0
			if ROBOTAXISNUML=1
				Sp1 = WkSp1	!Measured1
				OldSP(1) = Sp1
				SP(1) = Sp1
				SP_RobotRot = Sp1
				RobotROTTargetPrev = Sp1
			else
				Sp2 = WkSp2	!Measured2
				OldSP(2) = Sp2
				SP(2) = Sp2
				SP_RobotRot = Sp2
				RobotROTTargetPrev = Sp2
			end
		END
	end
	BLOCK
	AxisNum = 0
	Loop MAXNUMOFAXIS
		if (iType(AxisNum)<6000)& (FAULT(AxisNum).#CPE=1)
			KILL(AxisNum)
			iControlProcessState(AxisNum) = 111
			DISP "Critical position error, axis: ", AxisNum
			if AxisNum = 0
				Sp0 = WkSp0	!Measured0
				OldSP(0) = Sp0
				SP(0) = Sp0
			elseif AxisNum = 1
				Sp1 = WkSp1	!Measured1
				OldSP(1) = Sp1
				SP(1) = Sp1
			elseif AxisNum = 2
				Sp2 = WkSp2	!Measured2
				OldSP(2) = Sp2
				SP(2) = Sp2
			elseif AxisNum = 3
				Sp3 = WkSp3	!Measured3
				OldSP(3) = Sp3
				SP(3) = Sp3
			elseif AxisNum = 4
				Sp4 = WkSp4	!Measured4
				OldSP(4) = Sp4
				SP(4) = Sp4
			elseif AxisNum = 5
				Sp5 = WkSp5	!Measured5
				OldSP(5) = Sp5
				SP(5) = Sp5
			elseif AxisNum = 6
				Sp6 = WkSp6	!Measured6
				OldSP(6) = Sp6
				SP(6) = Sp6
			elseif AxisNum = 7
				Sp7 = WkSp7	!Measured7
				OldSP(7) = Sp7
				SP(7) = Sp7
			end
			!Sp0 = Measured0
			!OldSP(0) = Sp0
			!SP(0) = Sp0
			AxisNum = AxisNum +1
		END
	end
end	
RET

ON ((KillTraj0=1)|(KillTraj1=1)|(KillTraj2=1)|(KillTraj3=1)|(KillTraj4=1)|(KillTraj5=1)|(KillTraj6=1)|(KillTraj7=1))
!BLOCK
	iKillTrajVar(0)=KillTraj0
	iKillTrajVar(1)=KillTraj1
	iKillTrajVar(2)=KillTraj2
	iKillTrajVar(3)=KillTraj3
	iKillTrajVar(4)=KillTraj4
	iKillTrajVar(5)=KillTraj5
	iKillTrajVar(6)=KillTraj6
	iKillTrajVar(7)=KillTraj7
	
	if (iType(0)>6000)&( KillTraj0=1) ! Robot Extension axis stop command
		KILL (0, ROBOTAXISNUML)
		iControlProcessState(ROBOTAXISNUML) = 112
		STOP 30		! stop ext homing buffer if running
		STOP 28		! stop rot homing buffer if running
		STOP 23		! stop ext homing buffer if running
		KillTraj0=0
	end
	if (iType(0)>6000)&(iKillTrajVar(ROBOTAXISNUML)=1) ! Robot Rotation axis stop command
		KILL (0, ROBOTAXISNUML)
		iControlProcessState(ROBOTAXISNUMH) = 112
		STOP 30		! stop ext homing buffer if running
		STOP 28		! stop rot homing buffer if running
		STOP 23		! stop ext homing buffer if running
		if ROBOTAXISNUML=1
			KillTraj1=0
		else
			KillTraj2=0
		end
	end
	
	AxisNum = 0
	Loop MAXNUMOFAXIS
		if (iType(AxisNum)<6000)&(iKillTrajVar(AxisNum)=1)
			KILL (AxisNum)
			STOP AxisNum		! stop rot homing buffer if running
			iControlProcessState(AxisNum) = 112
		end	
		AxisNum=AxisNum+1
	end
	if (iType(0)<6000)&(KillTraj0=1)
		KillTraj0=0
	END

	if(iType(2)<6000)&(KillTraj2=1)
		KillTraj2=0
	END

	if KillTraj1=1
		KillTraj1=0
	end

	if KillTraj3=1
		KillTraj3=0
	end

	if KillTraj4=1
		KillTraj4=0
	end

	if KillTraj5=1
		KillTraj5=0
	end

	if KillTraj6=1
		KillTraj6=0
	end

	if KillTraj7=1
		KillTraj7=0
	end
	
RET






!LL1 door input (1=closed; 0=open)
ON (((IN(0).5 = 0 )&(MZ_LLAxisNum=-1))|((oLL_Door_Closed = 0)&(MZ_LLAxisNum>-1))) &(bFoundLLELE = 1) & MST(LLELEAXIS).#ENABLED
KILL (LLELEAXIS)
iControlProcessState(LLELEAXIS) = 113
DISP "LL door open!"
DISABLE (LLELEAXIS)
iLLDoorOpened=1
if (LLELEAXIS=0)
	Sp0 = WkSp0	!Measured0
	OldSP(0) = Sp0
	SP(0) = Sp0
elseif (LLELEAXIS=1)
	Sp1 = WkSp1	!Measured1
	OldSP(1) = Sp1
	SP(1) = Sp1
elseif (LLELEAXIS=2)
	Sp2 = WkSp2	!Measured2
	OldSP(2) = Sp2
	SP(2) = Sp2
elseif (LLELEAXIS=3)
	Sp3 = WkSp3	!Measured3
	OldSP(3) = Sp3
	SP(3) = Sp3
elseif (LLELEAXIS=4)
	Sp4 = WkSp4	!Measured4
	OldSP(4) = Sp4
	SP(4) = Sp4
elseif (LLELEAXIS=5)
	Sp5 = WkSp5	!Measured5
	OldSP(5) = Sp5
	SP(5) = Sp5
elseif (LLELEAXIS=6)
	Sp6 = WkSp6	!Measured6
	OldSP(6) = Sp6
	SP(6) = Sp6
elseif (LLELEAXIS=7)
	Sp7 = WkSp7	!Measured7
	OldSP(7) = Sp7
	SP(7) = Sp7
end

RET

ON (MST(LLELEAXIS).#ENABLED = 0) & ( ((IN(0).5 = 1 )&(MZ_LLAxisNum=-1))|((oLL_Door_Closed = 1)&(MZ_LLAxisNum>-1)) ) & (bFoundLLELE = 1)
if (iLLDoorOpened=1)
	ENABLE(LLELEAXIS)
	DISP "LL door closed!"
	iLLDoorOpened=0
end	
RET

!LL2 door input (1=closed; 0=open)
ON (IN(0).7=0) &(bFoundLLELE2 = 1) & MST(LLELEAXIS2).#ENABLED
KILL (LLELEAXIS2)
iControlProcessState(LLELEAXIS2) = 113
DISP "LL2 door open!"
DISABLE (LLELEAXIS2)
iLLDoorOpened2=1
if (LLELEAXIS2=0)
	Sp0 = WkSp0	!Measured0
	OldSP(0) = Sp0
	SP(0) = Sp0
elseif (LLELEAXIS2=1)
	Sp1 = WkSp1	!Measured1
	OldSP(1) = Sp1
	SP(1) = Sp1
elseif (LLELEAXIS2=2)
	Sp2 = WkSp2	!Measured2
	OldSP(2) = Sp2
	SP(2) = Sp2
elseif (LLELEAXIS2=3)
	Sp3 = WkSp3	!Measured3
	OldSP(3) = Sp3
	SP(3) = Sp3
elseif (LLELEAXIS2=4)
	Sp4 = WkSp4	!Measured4
	OldSP(4) = Sp4
	SP(4) = Sp4
elseif (LLELEAXIS2=5)
	Sp5 = WkSp5	!Measured5
	OldSP(5) = Sp5
	SP(5) = Sp5
elseif (LLELEAXIS2=6)
	Sp6 = WkSp6	!Measured6
	OldSP(6) = Sp6
	SP(6) = Sp6
elseif (LLELEAXIS2=7)
	Sp7 = WkSp7	!Measured7
	OldSP(7) = Sp7
	SP(7) = Sp7
end

RET

ON (MST(LLELEAXIS2).#ENABLED = 0) & (IN(0).7=1)&(bFoundLLELE2 = 1)
if (iLLDoorOpened2=1)
	ENABLE(LLELEAXIS2)
	DISP "LL2 door closed!"
	iLLDoorOpened2=0
end	
RET
! Interlock
On (bInterlocActive=1)&((iHomingMethod(2) <> 1)|(iType(2)>6000)) &	(bAxisInterlocked=0)	!Interlock is valid only when axis 2 is not used for homing to switch
int i
i = 0

Loop MAXNUMOFAXIS
	DISP "Interlock input activated."
	if(iInterlockEnabled(i)=1)	!1- yes, 0-no
		if(	iInterlockBehavior(i)=0)	!default 0, stop motion; 1-go to interlock position
			KILL(i)
			iControlProcessState(i) = 113
			OldSP(i) = SP(i)
		elseif(	iInterlockBehavior(i)=1)	
			if(i=0)
				Sp0 = dInterlockPos(i)
			elseif(i=1)
				Sp1 = dInterlockPos(i)
			elseif(i=2)
				Sp2 = dInterlockPos(i)
			elseif(i=3)
				Sp3 = dInterlockPos(i)
			elseif(i=4)
				Sp4 = dInterlockPos(i)
			elseif(i=5)
				Sp5 = dInterlockPos(i)
			elseif(i=6)
				Sp6 = dInterlockPos(i)
			elseif(i=7)
				Sp7 = dInterlockPos(i)
			end	
		end
	end
	i=i+1
END
wait 1000
bAxisInterlocked=1
RET


On (bInterlocActive = 0 ) & ((iHomingMethod(2) <> 1)|(iType(2)>6000)) & (bAxisInterlocked= 1)	!Interlock is valid only when axis 2 is not used for homing to switch
	bAxisInterlocked=0
RET


! 2nd Setpoint
On (IN(0).4=1)&(iHomingMethod(4) <> 1) &(bAxisAtSecondSP=0)	!2nd setpoint is valid only when axis 4 is not used for homing to switch

bAxisAtSecondSP=1
int ii
ii = 0
DISP "2nd SP input activated"
Loop MAXNUMOFAXIS
	if(	iSecondSPEnabled(ii)=1)	!1- yes, 0-no
		iControlProcessState(ii) = 114
		if(ii=0)
			Sp0 = dSecondSetpointPos(ii)
		elseif(ii=1)
			Sp1 = dSecondSetpointPos(ii)
		elseif(ii=2)
			Sp2 = dSecondSetpointPos(ii)
		elseif(ii=3)
			Sp3 = dSecondSetpointPos(ii)
		elseif(ii=4)
			Sp4 = dSecondSetpointPos(ii)
		elseif(ii=5)
			Sp5 = dSecondSetpointPos(ii)
		elseif(ii=6)
			Sp6 = dSecondSetpointPos(ii)
		elseif(ii=7)
			Sp7 = dSecondSetpointPos(ii)
		end	
	end
	ii=ii+1
END

RET

On (IN(0).4=0)&(iHomingMethod(4) <> 1) &(bAxisAtSecondSP=1)	!2nd setpoint is valid only when axis 4 is not used for homing to switch
	bAxisAtSecondSP=0
RET


