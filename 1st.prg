#/ Controller version = 3.03
#/ Date = 9/10/2024 2:19 PM
#/ User remarks = 
#0
!#/ Controller version = 2.27
!#/ Date = 12/28/2015 10:33 AM
!#/ User remarks = 
! motor cummt and axis homing routine for all types except for robot arms

!Change log
! DATE		ECO#	Reason for Change			Remarks
!9/21/2015	N/A		ini release					ZW
!12/28/15	N/A		tuning homing				ZW
!10/16/17	N/A		turn on CPE before move offset, also link CPE to %home torque
!11/17/2017 Disconnet the AVP CPE link to home torques which cause not homing
Global INT	iLl_Interlock_Cmd	!iWord10Bit1			:= doutputs.,
Global INT	iSt_Interlock_Cmd	!iWord10Bit5			:= doutputs.,
Global INT	iCar_Interlock_Cmd	!iWord10Bit9			:= doutputs.,
Global INT  bAxisInterlocked

GLOBAL real OldSP(8), Datalog(2)(1000)
global int AnalogCARAxis(4)
global real AnologHomeSignalMedian(4)
real SP_Direction                 ! Search direction 
real SP_Settle_Time               ! Settling time at Detent Points [msec]
real SP_Search_Vel                ! Search velocity [user-units/sec]
real SP_Drive                     ! Actuating drive command [% of maximum]
real SP_Max_Search                ! Maximum search distance [user-units]
real SP_Init_Offset               ! Initial commutation offset [elec. degrees]
real PrevAcc, PrevDec, PrevKDec, PrevJerk, PrevVel
real minAnalog, maxAnalog
int iNeedSLL, iNeedSRL
real store_CURI, store_CURV
int iAxisNum
real store_CERRI0, store_CERRV0

iAxisNum = 0	!set the particular axis for the buffer
If ((iType(iAxisNum)= 4008) & (iCar_Interlock_Cmd = 0)) | ((iType(iAxisNum)= 5011) & (iLl_Interlock_Cmd = 0)) | ((iType(iAxisNum)= 5012) & (iSt_Interlock_Cmd = 0)) | ((iInterlockEnabled(iAxisNum)=1)&(bAxisInterlocked=1))
	iControlProcessState(iAxisNum) = 113
	GOTO ENDOFHOMING
end

iControlProcessState(iAxisNum) = 14
CERRI(iAxisNum)=4.39
CERRV(iAxisNum)=4.39
store_CERRI0 = CERRI(iAxisNum)
store_CERRV0 = CERRV(iAxisNum)
MFLAGS(iAxisNum).#HOME = 0			
iNeedSLL=0
iNeedSRL=0

if FMASK(iAxisNum).#SLL = 1
	FMASK(iAxisNum).#SLL = 0
	iNeedSLL=1
end

if FMASK(iAxisNum).#SRL = 1
	FMASK(iAxisNum).#SRL = 0
	iNeedSRL=1
end

if MFLAGS(iAxisNum).9=0                ! not commutation

	SP_Drive = 5.475          
	SP_Settle_Time = 1000    
	SP_Search_Vel  = 6         
	SP_Init_Offset = 0
	SP_Direction =  -1
	SP_Max_Search = 144
	PrevAcc = ACC(iAxisNum)
	PrevDec = DEC(iAxisNum)
	PrevKDec=KDEC(iAxisNum)
	PrevJerk=JERK(iAxisNum)

	!   Output varibale:
	int  SP_Fail_Code                 ! Faiure Code of the startup program

	! Auxiliary variable:
	real SP_Pitch                     ! Magnetic pitch [180 elec.deg. in user units]

	! State Flag
	int  SP_InCommutationStartup      ! Flag indicating commutation startup is in progress


	SP_Pitch=SLCPRD(iAxisNum)/SLCNP(iAxisNum)*EFAC(iAxisNum)


	!******************************************************************************* 
	! INITIALIZE
	FCLEAR(iAxisNum)   
	disable(iAxisNum)                     
	SETCONF(216,(iAxisNum),0)                ! Reset commutation state
	setconf(214,(iAxisNum),SP_Init_Offset)   ! Set initial commutation phase
	SP_Fail_Code=0                        ! Reset failure
	SP_Direction=1                        ! Move in positive direction
	SP_InCommutationStartup=1             ! commutation startup in progress
	!******************************************************************************* 
	! SET MOTION PROFILE FOR COMMUTATION STARTUP PROCESS 
	!
	!   WARNING: The following are the suggested motion parameters for the startup
	!     process. Check that the values are suitable for your application.

	ACC(iAxisNum)=SP_Search_Vel*10.; DEC(iAxisNum)=SP_Search_Vel*10. 
	KDEC(iAxisNum)=SP_Search_Vel*50.; JERK(iAxisNum)=SP_Search_Vel*100.

	!******************************************************************************* 
	! STEP 1 - MOVE TO FIRST DETENT POINT 
	!
	! WARNING: The motor moves to a detent point by jump. 
	!   The jump distance is up to one magnetic pitch in any direction.
	!   The motor jumps to the closest detent point within its motion range.
	!   If necessary modify initial detent point by changing the variable 
	!     SP_Init_Offset between 0-360 electrical degrees.  
	enable(iAxisNum)
	wait(100)
	while (DCOM(iAxisNum)+0.05 < SP_Drive); DCOM(iAxisNum) = DCOM(iAxisNum) + 0.05; end
	DCOM(iAxisNum) = SP_Drive
	wait SP_Settle_Time
	call Limit_Check
	!******************************************************************************* 
	! STEP 2 - MOVE TO SECOND DETENT POINT
	!
	!   The program moves the motor 90 electrical degrees in order to eliminate
	!      a state of unstable equilibrium.

	Move_Detent:
	ptp/rv (iAxisNum), SP_Direction*SP_Pitch/2.,SP_Search_Vel
	till ^AST(iAxisNum).#MOVE; wait SP_Settle_Time
	call Limit_Check
	disable(iAxisNum)

	MFLAGS(iAxisNum).9=1               ! Set commutation state 
	DCOM(iAxisNum) = 0

	! If motor is to be left enabled after startup process delete the following line:

	Finish:
	SP_InCommutationStartup=0              ! commutation startup is finished
	If SP_Fail_Code=0; disp "   Commutation Startup Finished."
	else disp "   Commutation Startup Failed."; disp "   Failure Code = %i",SP_Fail_Code; end

	ACC(iAxisNum)=PrevAcc
	DEC(iAxisNum)=PrevDec
	KDEC(iAxisNum)=PrevKDec
	JERK(iAxisNum)=PrevJerk

end

!at this moment, the axis shall commuted. If it is, then start homing

if MFLAGS(iAxisNum).9=1
	if (iHomingMethod(iAxisNum) = 0)		!0-home to torque; 1-homing to switch; 2-manual set home;
	    if (iType(iAxisNum)>1000)&(iType(iAxisNum)<2000) 	
		!AVP home to torques - always
			call HOME_TO_TORQ_AVP	!different than LP home to torque
		else
			call HOME_TO_TORQ
		end	
	elseif (iHomingMethod(iAxisNum) = 1)&(iType(iAxisNum)<6000)
		if(iType(iAxisNum)>3000)&(iType(iAxisNum)<5000)
			CALL HOME_TO_SWITCH_CAR
		elseif(iType(iAxisNum) = 5008 | iType(iAxisNum) = 5009 | iType(iAxisNum) = 5010 | iType(iAxisNum) = 5018 | iType(iAxisNum) = 5019 | iType(iAxisNum) = 5020 | iType(iAxisNum) = 5028 | iType(iAxisNum) = 5029 | iType(iAxisNum) = 5030)
			call HOME_TO_SWITCH_CARZ
			FDEF(iAxisNum).#LL=1       ! Enable the iAxis left limit default response
			FDEF(iAxisNum).#RL=1       ! Enable the iAxis right limit default response
		else	
			CALL HOME_TO_SWITCH
			FDEF(iAxisNum).#LL=1       ! Enable the iAxis left limit default response
		end
	elseif (iHomingMethod(iAxisNum) = 2)
		CALL SET_CURRENTPOS_HOME
	end
end
iControlProcessState(iAxisNum) = 10
if iNeedSLL = 1
	FMASK(iAxisNum).#SLL = 1
end

if iNeedSRL = 1
	FMASK(iAxisNum).#SRL = 1
end
ENDOFHOMING:
STOP

!******************************************************************************* 
!   The following routine move the motor away from limit switches 

Limit_Check:
if MERR(iAxisNum) & MERR(iAxisNum)<>5010 & MERR(iAxisNum)<>5011; SP_Fail_Code=1; DISABLE(iAxisNum); DCOM(iAxisNum)=0; goto Finish; end
!   if MERR(iAxisNum); SP_Fail_Code=1; DISABLE(iAxisNum); DCOM(iAxisNum)=0; goto Finish; end 
if (FAULT(iAxisNum).#LL)|(FAULT(iAxisNum).#RL)
   if FAULT(iAxisNum).#LL; SP_Direction=1; else SP_Direction=-1; end
   ptp/rv (iAxisNum), SP_Direction*SP_Max_Search, SP_Search_Vel 
   till ((^FAULT(iAxisNum).#LL)&(^FAULT(iAxisNum).#RL))|(^AST(iAxisNum).#MOVE)
   if (FAULT(iAxisNum).#LL)|(FAULT(iAxisNum).#RL); SP_Fail_Code=4; DISABLE(iAxisNum); DCOM(iAxisNum)=0; goto Finish; end 
   kill(iAxisNum);  wait SP_Settle_Time; goto Move_Detent
end
ret

ON ((SP_InCommutationStartup = 1) & ((FAULT(iAxisNum)&0x30f80)>0 | (S_FAULT&0x30000000)>0))
SP_Fail_Code=1; DISABLE(iAxisNum); DCOM(iAxisNum)=0
CALL Finish 
RET



HOME_TO_TORQ:

MFLAGS(iAxisNum).#HOME = 0
FDEF(iAxisNum).#LL=0       ! Disable the iAxis left limit default response
FDEF(iAxisNum).#RL=0       ! Disable the iAxis right limit default response
ENABLE (iAxisNum)          ! Enable the iAxis drive
wait(1000)
JOG/v (iAxisNum), dHomingVel(iAxisNum)*dAxisPitch(iAxisNum)           ! Move to the left  
wait(1000)
KILL iAxisNum
! Reduce the motor current saturations
store_CURI = XCURI(iAxisNum)
store_CURV = XCURV(iAxisNum)
XCURI(iAxisNum) = dHomingTorq(iAxisNum)
XCURV(iAxisNum) = dHomingTorq(iAxisNum)

CERRI(iAxisNum)=0.01*dHomingTorq(iAxisNum)*CERRI(iAxisNum)
CERRV(iAxisNum)=0.01*dHomingTorq(iAxisNum)*CERRV(iAxisNum)

! Turn off the CPE response
FDEF(iAxisNum).#CPE = 0
! Jog until critical position error is triggered
JOG/v (iAxisNum), -dHomingVel(iAxisNum)*dAxisPitch(iAxisNum)           ! Move to the left  
TILL (FAULT(iAxisNum).#CPE = 1)   			! Wait for the critical PE
Kill iAxisNum
! Stop motor, let settle, set FPOS and TPOS to 0.0.  Then turn CPE mask back off
SET FPOS(iAxisNum) = 0.0
PTP/e (iAxisNum), 0.0
REAL deltaI, deltaV
deltaI = store_CURI - dHomingTorq(iAxisNum)
deltaV = store_CURV - dHomingTorq(iAxisNum)
LOOP 100
	XCURI(iAxisNum) = XCURI(iAxisNum) + 0.01*(deltaI)
	XCURV(iAxisNum) = XCURV(iAxisNum) + 0.01*(deltaV)
	SET FPOS(iAxisNum) = 0
	wait 20	!10ms x 100 = 1 sec ramp
END
!In case of rounding error, lastly set XCUR* to stored value - should be nearly identical at this point
XCURI(iAxisNum) = store_CURI
XCURV(iAxisNum) = store_CURV

CERRV(iAxisNum) = store_CERRV0
CERRI(iAxisNum) = store_CERRI0
FDEF(iAxisNum).#CPE = 1

BREAK iAxisNum
SET FPOS(iAxisNum) = 0 -dHomeOffset(iAxisNum)*dAxisPitch(iAxisNum)
PTP/ev (iAxisNum), 0.0, dHomingVel(iAxisNum)*dAxisPitch(iAxisNum)

!After homing, change Sp to 0.0 to prevent follow-on move.
OldSP(iAxisNum)=0
if 		iAxisNum = 0								
	Sp0 = 0.0
elseif 	iAxisNum = 1							
	Sp1 = 0.0
elseif 	iAxisNum = 2						
	Sp2 = 0.0
elseif 	iAxisNum = 3						
	Sp3 = 0.0
elseif 	iAxisNum = 4						
	Sp4 = 0.0
elseif 	iAxisNum = 5						
	Sp5 = 0.0
elseif 	iAxisNum = 6						
	Sp6 = 0.0
elseif 	iAxisNum = 7						
	Sp7 = 0.0
end

FDEF(iAxisNum).#CPE = 1
MFLAGS(iAxisNum).#HOME = 1
wait 500
RET


HOME_TO_SWITCH:

MFLAGS(iAxisNum).#HOME = 0
FDEF(iAxisNum).#LL=0       ! Disable the iAxis left limit default response
FDEF(iAxisNum).#RL=0       ! Disable the iAxis right limit default response
ENABLE (iAxisNum)          ! Enable the iAxis drive
wait(1000)

if  (IN(0).iAxisNum = 1) 
	JOG/v (iAxisNum), dHomingVel(iAxisNum)*dAxisPitch(iAxisNum)           ! Move to the left  
	TILL (IN(0).iAxisNum = 0)   			! Wait for home switch input
	! Stop motor, let settle, set FPOS and TPOS to 0.0.  Then turn CPE mask back off
	KILL (iAxisNum)
end
! Reduce the motor current saturations
store_CURI = XCURI(iAxisNum)
store_CURV = XCURV(iAxisNum)
XCURI(iAxisNum) = dHomingTorq(iAxisNum)
XCURV(iAxisNum) = dHomingTorq(iAxisNum)
! Do NOT Turn off the CPE response for home to switch
!FDEF(iAxisNum).#CPE = 0
! Jog until critical position error is triggered
JOG/v (iAxisNum), -dHomingVel(iAxisNum)*dAxisPitch(iAxisNum)           ! Move to the left  
   			! Wait for home switch input
TILL (IN(0).iAxisNum = 1)
! Stop motor, let settle, set FPOS and TPOS to 0.0.  Then turn CPE mask back off
KILL (iAxisNum)
wait 3000
SET FPOS(iAxisNum) = 0	!@CR2015-12-31 modified home position to simple offset - why move to index?
TPOS(iAxisNum) = 0
! FDEF(iAxisNum).#CPE = 1	!no need to turn turn CPE on because it was not turned off.
! disable motor to return to running torque, and immediately re-enable. This prevents an abrupt move with torque increase.

LOOP 100
	XCURI(iAxisNum) = XCURI(iAxisNum) + 0.01*(store_CURI - dHomingTorq(iAxisNum))
	wait 10	!10ms x 100 = 1 sec ramp
	XCURV(iAxisNum) = XCURV(iAxisNum) + 0.01*(store_CURV - dHomingTorq(iAxisNum))
	wait 10	!10ms x 100 = 1 sec ramp
END
! In case of rounding error, lastly set XCUR* to stored value - should be nearly identical at this point
XCURI(iAxisNum) = store_CURI
XCURV(iAxisNum) = store_CURV

!disable (iAxisNum)
!XCURI(iAxisNum) = store_CURI !@@@ moved here from end of if-thens
!XCURV(iAxisNum) = store_CURV	!@@@ moved here from end of if-thens
!enable (iAxisNum)
wait 1000
PTP/ev (iAxisNum), dHomeOffset(iAxisNum)*dAxisPitch(iAxisNum), dHomingVel(iAxisNum)*dAxisPitch(iAxisNum)
wait 3000
! After moving to home offset, re-zero the axis.  This is now the "working zero" for the axis
SET FPOS(iAxisNum) = 0	!@CR2015-12-31 modified home position to simple offset - why move to index?
TPOS(iAxisNum) = 0
! perform another move to zero (because without, there is residual FPOS data that shows at GUI - looks funny
PTP/ev (iAxisNum), 0.0, dHomingVel(iAxisNum)*dAxisPitch(iAxisNum)
wait 2000

!After homing, change Sp to 0.0 to prevent follow-on move.
OldSP(iAxisNum)=0
if 		iAxisNum = 0								
	Sp0 = 0.0
elseif 	iAxisNum = 1							
	Sp1 = 0.0
elseif 	iAxisNum = 2						
	Sp2 = 0.0
elseif 	iAxisNum = 3						
	Sp3 = 0.0
elseif 	iAxisNum = 4						
	Sp4 = 0.0
elseif 	iAxisNum = 5						
	Sp5 = 0.0
elseif 	iAxisNum = 6						
	Sp6 = 0.0
elseif 	iAxisNum = 7						
	Sp7 = 0.0
end

FDEF(iAxisNum).#CPE = 1
MFLAGS(iAxisNum).#HOME = 1
wait 500
RET







	
HOME_TO_SWITCH_CARZ:

MFLAGS(iAxisNum).#HOME = 0
FDEF(iAxisNum).#LL=0       ! Disable the iAxis left limit default response
FDEF(iAxisNum).#RL=0       ! Disable the iAxis right limit default response
ENABLE (iAxisNum)          ! Enable the iAxis drive
wait(1000)

if  (SAFIN(iAxisNum).#LL = 1) 
	JOG/v (iAxisNum), dHomingVel(iAxisNum)*dAxisPitch(iAxisNum)           ! Move to the left  
	TILL (SAFIN(iAxisNum).#LL  = 0)   			! Wait for home switch input
	! Stop motor, let settle, set FPOS and TPOS to 0.0.  Then turn CPE mask back off
	KILL (iAxisNum)
end
! Reduce the motor current saturations
store_CURI = XCURI(iAxisNum)
store_CURV = XCURV(iAxisNum)
XCURI(iAxisNum) = dHomingTorq(iAxisNum)
XCURV(iAxisNum) = dHomingTorq(iAxisNum)
! Do NOT Turn off the CPE response for home to switch
!FDEF(iAxisNum).#CPE = 0
! Jog until critical position error is triggered
JOG/v (iAxisNum), -dHomingVel(iAxisNum)*dAxisPitch(iAxisNum)           ! Move to the left  
   			! Wait for home switch input
TILL (SAFIN(iAxisNum).#LL = 1)
! Stop motor, let settle, set FPOS and TPOS to 0.0.  Then turn CPE mask back off
KILL (iAxisNum)
wait 3000

! @JR 2021-10-18 - Move down until upper limit switch opens
JOG/v (iAxisNum), dHomingVel(iAxisNum)*dAxisPitch(iAxisNum)
TILL (SAFIN(iAxisNum).#LL = 0)
KILL (iAxisNum)
wait 3000

SET FPOS(iAxisNum) = 0	!@CR2015-12-31 modified home position to simple offset - why move to index?
TPOS(iAxisNum) = 0
! FDEF(iAxisNum).#CPE = 1	!no need to turn turn CPE on because it was not turned off.
! disable motor to return to running torque, and immediately re-enable. This prevents an abrupt move with torque increase.

!LOOP 100
!	XCURI(iAxisNum) = XCURI(iAxisNum) + 0.01*(store_CURI - dHomingTorq(iAxisNum))
!	wait 10	!10ms x 100 = 1 sec ramp
!	XCURV(iAxisNum) = XCURV(iAxisNum) + 0.01*(store_CURV - dHomingTorq(iAxisNum))
!	wait 10	!10ms x 100 = 1 sec ramp
!END
! In case of rounding error, lastly set XCUR* to stored value - should be nearly identical at this point
XCURI(iAxisNum) = store_CURI
XCURV(iAxisNum) = store_CURV

!disable (iAxisNum)
!XCURI(iAxisNum) = store_CURI !@@@ moved here from end of if-thens
!XCURV(iAxisNum) = store_CURV	!@@@ moved here from end of if-thens
!enable (iAxisNum)
wait 1000
PTP/ev (iAxisNum), dHomeOffset(iAxisNum)*dAxisPitch(iAxisNum), dHomingVel(iAxisNum)*dAxisPitch(iAxisNum)
wait 3000
! After moving to home offset, re-zero the axis.  This is now the "working zero" for the axis
SET FPOS(iAxisNum) = 0	!@CR2015-12-31 modified home position to simple offset - why move to index?
TPOS(iAxisNum) = 0
! perform another move to zero (because without, there is residual FPOS data that shows at GUI - looks funny
PTP/ev (iAxisNum), 0.0, dHomingVel(iAxisNum)*dAxisPitch(iAxisNum)
wait 2000

!After homing, change Sp to 0.0 to prevent follow-on move.
OldSP(iAxisNum)=0
if 		iAxisNum = 0								
	Sp0 = 0.0
elseif 	iAxisNum = 1							
	Sp1 = 0.0
elseif 	iAxisNum = 2						
	Sp2 = 0.0
elseif 	iAxisNum = 3						
	Sp3 = 0.0
elseif 	iAxisNum = 4						
	Sp4 = 0.0
elseif 	iAxisNum = 5						
	Sp5 = 0.0
elseif 	iAxisNum = 6						
	Sp6 = 0.0
elseif 	iAxisNum = 7						
	Sp7 = 0.0
end

FDEF(iAxisNum).#CPE = 1
MFLAGS(iAxisNum).#HOME = 1
wait 500
RET

	
	
	
	
	

SET_CURRENTPOS_HOME:

SET FPOS(iAxisNum) = 0
TPOS(iAxisNum) = 0	!@@@ added
FDEF(iAxisNum).#CPE = 1
MFLAGS(iAxisNum).#HOME = 1
ENABLE (iAxisNum) 
! Move to 0 to clean-up residual FPOS error on GUI
PTP/ev (iAxisNum), 0, 180
wait 2000
!After homing, change Sp to 0.0 to prevent follow-on move.
OldSP(iAxisNum)=0
if 		iAxisNum = 0								
	Sp0 = 0.0
elseif 	iAxisNum = 1							
	Sp1 = 0.0
elseif 	iAxisNum = 2						
	Sp2 = 0.0
elseif 	iAxisNum = 3						
	Sp3 = 0.0
elseif 	iAxisNum = 4						
	Sp4 = 0.0
elseif 	iAxisNum = 5						
	Sp5 = 0.0
elseif 	iAxisNum = 6						
	Sp6 = 0.0
elseif 	iAxisNum = 7						
	Sp7 = 0.0
end
RET


HOME_TO_TORQ_AVP:


MFLAGS(iAxisNum).#HOME = 0
FDEF(iAxisNum).#LL=0       ! Disable the iAxis left limit default response
FDEF(iAxisNum).#RL=0       ! Disable the iAxis right limit default response
ENABLE (iAxisNum)          ! Enable the iAxis drive
wait(100)
JOG/v (iAxisNum), dHomingVel(iAxisNum)*dAxisPitch(iAxisNum)           ! Move to the left  
wait(1000)
KILL iAxisNum
!reduce the motor current saturations
store_CURI = XCURI(iAxisNum)
store_CURV = XCURV(iAxisNum)

real XCURI_actural, HmCurrent_Of_Drive, FtCurrent_Of_Drive
HmCurrent_Of_Drive = dHomingTorq(iAxisNum)*(4*XRMS(iAxisNum))/100.0
FtCurrent_Of_Drive = dFaultTorq(iAxisNum)*(4*XRMS(iAxisNum))/100.0

XCURI(iAxisNum) = HmCurrent_Of_Drive
XCURV(iAxisNum) = HmCurrent_Of_Drive
!CERRI(iAxisNum)=0.01*dHomingTorq(iAxisNum)*CERRI(iAxisNum)
!CERRV(iAxisNum)=0.01*dHomingTorq(iAxisNum)*CERRV(iAxisNum)
! Turn off the CPE response
FDEF(iAxisNum).#CPE = 0

JOG/v (iAxisNum), -dHomingVel(iAxisNum)*dAxisPitch(iAxisNum)           ! Move to the left  
TILL (FAULT(iAxisNum).#CPE = 1)   			! Wait for the critical PE
KILL iAxisNum

if (dAxisPitch(iAxisNum)>0.01)
	dCE(iAxisNum)=FPOS(iAxisNum)/dAxisPitch(iAxisNum)+dHomeOffset(iAxisNum)
end
block
SET FPOS(iAxisNum) = 0.0 -dHomeOffset(iAxisNum)*dAxisPitch(iAxisNum)
PTP/e (iAxisNum), 100!0.0
end



deltaI = FtCurrent_Of_Drive - HmCurrent_Of_Drive
deltaV = FtCurrent_Of_Drive - HmCurrent_Of_Drive
LOOP 100
	XCURI(iAxisNum) = XCURI(iAxisNum) + 0.01*(deltaI)
	XCURV(iAxisNum) = XCURV(iAxisNum) + 0.01*(deltaV)
	wait 20	!10ms x 100 = 1 sec ramp
END

XCURI(iAxisNum) = FtCurrent_Of_Drive !@@@ moved here from end of if-thens
XCURV(iAxisNum) = FtCurrent_Of_Drive	!@@@ moved here from end of if-thens
CERRV(iAxisNum) = store_CERRV0
CERRI(iAxisNum) = store_CERRI0
FDEF(iAxisNum).#CPE = 1

BREAK iAxisNum
!SET FPOS(iAxisNum) = 0.0 -dHomeOffset(iAxisNum)*dAxisPitch(iAxisNum)	!@CR2015-12-31 modified home position to simple offset - why move to index?
PTP/ev (iAxisNum), 0.0, dHomingVel(iAxisNum)*dAxisPitch(iAxisNum)


!After homing, change Sp to 0.0 to prevent follow-on move.
OldSP(iAxisNum)=0
if 		iAxisNum = 0								
	Sp0 = 0.0
elseif 	iAxisNum = 1							
	Sp1 = 0.0
elseif 	iAxisNum = 2						
	Sp2 = 0.0
elseif 	iAxisNum = 3						
	Sp3 = 0.0
elseif 	iAxisNum = 4						
	Sp4 = 0.0
elseif 	iAxisNum = 5						
	Sp5 = 0.0
elseif 	iAxisNum = 6						
	Sp6 = 0.0
elseif 	iAxisNum = 7						
	Sp7 = 0.0
end
FDEF(iAxisNum).#CPE = 1
MFLAGS(iAxisNum).#HOME = 1
RET

HOME_TO_SWITCH_CAR:
REAL rMax, rMin
INT iCARAIChannel
iCARAIChannel = 0

MFLAGS(iAxisNum).#HOME = 0
FDEF(iAxisNum).#LL=0       ! Disable the iAxis left limit default response
FDEF(iAxisNum).#RL=0       ! Disable the iAxis right limit default response
ENABLE (iAxisNum)          ! Enable the iAxis drive
wait(1000)
! Jog until critical position error is triggered
if (FtdCarHomeSW.iAxisNum = 1)
	!JOG/v (iAxisNum), dHomingVel(iAxisNum)           ! Move to the left  
	!TILL (FtdCarHomeSW(iAxisNum) = 0)   			! Wait for home switch input
	!BREAK (iAxisNum)
	PTP/er (iAxisNum), 15
end

if(iType(iAxisNum)=4007)	! if it is an analog sensor, perform sensor calibration first.
! find which analog channel is the home sensor at

	if AnalogCARAxis(0)=iAxisNum
		iCARAIChannel = 0
	elseif AnalogCARAxis(1)=iAxisNum
		iCARAIChannel = 1
	elseif AnalogCARAxis(2)=iAxisNum
		iCARAIChannel = 2
	elseif AnalogCARAxis(3)=iAxisNum
		iCARAIChannel = 3
	end
! scan the analog signal and acq 1000 points from the analog signal while spinning
	DC/S iAxisNum,Datalog,1000,100,FPOS(iAxisNum),AIN(iCARAIChannel)
	JOG/v (iAxisNum), 4 
	till (^AST(iAxisNum).#DC)	!wait DAQ to finish
	KILL iAxisNum
	WAIT 2000
! find the median in analog signal	
	rMax = MAX(Datalog, 1, 1, 0,999)
	rMin = MIN(Datalog, 1, 1, 0,999)
	if (rMax-rMin)>1.0
		AnologHomeSignalMedian(iCARAIChannel) = (rMax + rMin)/2.0
	else
		disable(iAxisNum)  
		iControlProcessState(iAxisNum) = 404
		RET
	end	
! find the position where is the peak signal
	int thepeakposIndex
	thepeakposIndex = MAXI(Datalog, 1, 1, 0,999)
! move the CAR to the neighbourhood of the peak	
	PTP/e (iAxisNum), Datalog(0)(thepeakposIndex)+15
	WAIT 3000
end





real FirstEdge, SecondEdge
!Find the leading edge of the home sensor
JOG/v (iAxisNum), -dHomingVel(iAxisNum)           ! Move to the left  
TILL (FtdCarHomeSW.iAxisNum = 1)   			! Wait for home switch input
FirstEdge = FPOS(iAxisNum)
KILL iAxisNum
WAIT 2000
!back off
PTP/e (iAxisNum), FirstEdge + 10
!low speed homing
JOG/v (iAxisNum), -dHomingVel(iAxisNum)           ! Move to the left  
TILL (FtdCarHomeSW.iAxisNum = 1)   			! Wait for home switch input
FirstEdge = FPOS(iAxisNum)
KILL iAxisNum
WAIT 2000
PTP/e (iAxisNum), FirstEdge - 15
!low speed homing
JOG/v (iAxisNum), dHomingVel(iAxisNum)           ! Move to the left  
TILL (FtdCarHomeSW.iAxisNum = 1) 
SecondEdge = FPOS(iAxisNum)
KILL iAxisNum
WAIT 2000
PTP/ev (iAxisNum), 0.5*(FirstEdge+SecondEdge)+dHomeOffset(iAxisNum), dHomingVel(iAxisNum)
wait 3000

SET FPOS(iAxisNum) = 0	!@CR2015-12-31 modified home position to simple offset - why move to index?
TPOS(iAxisNum) = 0
! After moving to home offset, re-zero the axis.  This is now the "working zero" for the axis
SET FPOS(iAxisNum) = 0	!@CR2015-12-31 modified home position to simple offset - why move to index?
TPOS(iAxisNum) = 0
! perform another move to zero (because without, there is residual FPOS data that shows at GUI - looks funny
FDEF(iAxisNum).#CPE = 1
MFLAGS(iAxisNum).#HOME = 1
wait 500

PTP/ev (iAxisNum), 0.0, dHomingVel(iAxisNum)
wait 2000

!After homing, change Sp to 0.0 to prevent follow-on move.
OldSP(iAxisNum)=0
if 		iAxisNum = 0								
	Sp0 = 0.0
elseif 	iAxisNum = 1							
	Sp1 = 0.0
elseif 	iAxisNum = 2						
	Sp2 = 0.0
elseif 	iAxisNum = 3						
	Sp3 = 0.0
elseif 	iAxisNum = 4						
	Sp4 = 0.0
elseif 	iAxisNum = 5						
	Sp5 = 0.0
elseif 	iAxisNum = 6						
	Sp6 = 0.0
elseif 	iAxisNum = 7						
	Sp7 = 0.0
end

RET
#1
!#/ Controller version = 2.27
!#/ Date = 12/28/2015 10:33 AM
!#/ User remarks = 
! motor cummt and axis homing routine for all types except for robot arms

!Change log
! DATE		ECO#	Reason for Change			Remarks
!9/21/2015	N/A		ini release					ZW
!12/28/15	N/A		tuning homing				ZW
!10/16/17	N/A		turn on CPE before move offset, also link CPE to %home torque
!11/17/2017 Disconnet the AVP CPE link to home torques which cause not homing
!2/15/2019			VAT valve control code Beta
!2/20/2019			With ramp rate auto calibration
!2/27/2019			With PID always on
!9/10/2020	added POG_NO_FLT and quit the PID loop in fault conditions
Global INT	iLl_Interlock_Cmd	!iWord10Bit1			:= doutputs.,
Global INT	iSt_Interlock_Cmd	!iWord10Bit5			:= doutputs.,
Global INT	iCar_Interlock_Cmd	!iWord10Bit9			:= doutputs.,
Global INT  bAxisInterlocked

Global INT POG_NO_FLT	!!added POG_NO_FLT on 9/10/2020
GLOBAL real OldSP(8), Datalog(2)(1000)
global int AnalogCARAxis(4)
global real AnologHomeSignalMedian(4)
real SP_Direction                 ! Search direction 
real SP_Settle_Time               ! Settling time at Detent Points [msec]
real SP_Search_Vel                ! Search velocity [user-units/sec]
real SP_Drive                     ! Actuating drive command [% of maximum]
real SP_Max_Search                ! Maximum search distance [user-units]
real SP_Init_Offset               ! Initial commutation offset [elec. degrees]
real PrevAcc, PrevDec, PrevKDec, PrevJerk, PrevVel
real minAnalog, maxAnalog
int iNeedSLL, iNeedSRL
real store_CURI, store_CURV
int iAxisNum
real store_CERRI0, store_CERRV0
ControlProcessState1 = 0	!not ready
iAxisNum=1	!set the particular axis for the buffer
If ((iType(iAxisNum)= 4008) & (iCar_Interlock_Cmd = 0)) | ((iType(iAxisNum)= 5011) & (iLl_Interlock_Cmd = 0)) | ((iType(iAxisNum)= 5012) & (iSt_Interlock_Cmd = 0)) | ((iInterlockEnabled(iAxisNum)=1)&(bAxisInterlocked=1))
	iControlProcessState(iAxisNum) = 113
	GOTO ENDOFHOMING
end

iControlProcessState(iAxisNum) = 14
CERRI(iAxisNum)=4.39
CERRV(iAxisNum)=4.39
store_CERRI0 = CERRI(iAxisNum)
store_CERRV0 = CERRV(iAxisNum)
MFLAGS(iAxisNum).#HOME = 0			
iNeedSLL=0
iNeedSRL=0

if FMASK(iAxisNum).#SLL = 1
	FMASK(iAxisNum).#SLL = 0
	iNeedSLL=1
end

if FMASK(iAxisNum).#SRL = 1
	FMASK(iAxisNum).#SRL = 0
	iNeedSRL=1
end

if MFLAGS(iAxisNum).9=0                ! not commutation
	enable iAxisNum
	wait 1000
	COMMUT iAxisNum, 5.475, 1000, 200
	till MFLAGS(iAxisNum).9=1          
end

!at this moment, the axis shall commuted. If it is, then start homing

if MFLAGS(iAxisNum).9=1
	if (iHomingMethod(iAxisNum) = 0)		!0-home to torque; 1-homing to switch; 2-manual set home;
	    if (iType(iAxisNum)>1000)&(iType(iAxisNum)<2000) 	
		!AVP home to torques - always
			
			if (iType(iAxisNum)=1040) 
				CERRI(iAxisNum) = 0.8	!Critical Position Error idle
				CERRV(iAxisNum) = 0.8	!Critical Position Error constant velocity
				CERRA(iAxisNum) = 1.0	!Critical Position Error accerleration
				call HOME_TO_TORQ_VLV
			else
				call HOME_TO_TORQ_AVP	!different than LP home to torque
			end
		else
			call HOME_TO_TORQ
		end	
	elseif (iHomingMethod(iAxisNum) = 1)&(iType(iAxisNum)<6000)
		if(iType(iAxisNum)>3000)&(iType(iAxisNum)<5000)
			CALL HOME_TO_SWITCH_CAR
		elseif(iType(iAxisNum) = 5008 | iType(iAxisNum) = 5009 | iType(iAxisNum) = 5010 | iType(iAxisNum) = 5018 | iType(iAxisNum) = 5019 | iType(iAxisNum) = 5020 | iType(iAxisNum) = 5028 | iType(iAxisNum) = 5029 | iType(iAxisNum) = 5030)
			call HOME_TO_SWITCH_CARZ
			FDEF(iAxisNum).#LL=1       ! Enable the iAxis left limit default response
			FDEF(iAxisNum).#RL=1       ! Enable the iAxis right limit default response
	    else
			CALL HOME_TO_SWITCH
			FDEF(iAxisNum).#LL=1       ! Enable the iAxis left limit default response
		end
	elseif (iHomingMethod(iAxisNum) = 2)
		CALL SET_CURRENTPOS_HOME
	end
end
iControlProcessState(iAxisNum) = 10
if iNeedSLL = 1
	FMASK(iAxisNum).#SLL = 1
end

if iNeedSRL = 1
	FMASK(iAxisNum).#SRL = 1
end
if (iType(iAxisNum)=1040) 
	call VAT_VLV_CONTROLLER
end
ENDOFHOMING:
STOP

HOME_TO_TORQ:

MFLAGS(iAxisNum).#HOME = 0
FDEF(iAxisNum).#LL=0       ! Disable the iAxis left limit default response
FDEF(iAxisNum).#RL=0       ! Disable the iAxis right limit default response
ENABLE (iAxisNum)          ! Enable the iAxis drive
wait(1000)
JOG/v (iAxisNum), dHomingVel(iAxisNum)*dAxisPitch(iAxisNum)           ! Move to the left  
wait(1000)
KILL iAxisNum
! Reduce the motor current saturations
store_CURI = XCURI(iAxisNum)
store_CURV = XCURV(iAxisNum)
XCURI(iAxisNum) = dHomingTorq(iAxisNum)
XCURV(iAxisNum) = dHomingTorq(iAxisNum)

CERRI(iAxisNum)=0.01*dHomingTorq(iAxisNum)*CERRI(iAxisNum)
CERRV(iAxisNum)=0.01*dHomingTorq(iAxisNum)*CERRV(iAxisNum)

! Turn off the CPE response
FDEF(iAxisNum).#CPE = 0
! Jog until critical position error is triggered
JOG/v (iAxisNum), -dHomingVel(iAxisNum)*dAxisPitch(iAxisNum)           ! Move to the left  
TILL (FAULT(iAxisNum).#CPE = 1)   			! Wait for the critical PE
Kill iAxisNum
! Stop motor, let settle, set FPOS and TPOS to 0.0.  Then turn CPE mask back off
SET FPOS(iAxisNum) = 0.0
PTP/e (iAxisNum), 0.0
REAL deltaI, deltaV
deltaI = store_CURI - dHomingTorq(iAxisNum)
deltaV = store_CURV - dHomingTorq(iAxisNum)
LOOP 100
	XCURI(iAxisNum) = XCURI(iAxisNum) + 0.01*(deltaI)
	XCURV(iAxisNum) = XCURV(iAxisNum) + 0.01*(deltaV)
	SET FPOS(iAxisNum) = 0
	wait 20	!10ms x 100 = 1 sec ramp
END
!In case of rounding error, lastly set XCUR* to stored value - should be nearly identical at this point
XCURI(iAxisNum) = store_CURI
XCURV(iAxisNum) = store_CURV

CERRV(iAxisNum) = store_CERRV0
CERRI(iAxisNum) = store_CERRI0
FDEF(iAxisNum).#CPE = 1

BREAK iAxisNum
SET FPOS(iAxisNum) = 0 -dHomeOffset(iAxisNum)*dAxisPitch(iAxisNum)
PTP/ev (iAxisNum), 0.0, dHomingVel(iAxisNum)*dAxisPitch(iAxisNum)

!After homing, change Sp to 0.0 to prevent follow-on move.
OldSP(iAxisNum)=0
if 		iAxisNum = 0								
	Sp0 = 0.0
elseif 	iAxisNum = 1							
	Sp1 = 0.0
elseif 	iAxisNum = 2						
	Sp2 = 0.0
elseif 	iAxisNum = 3						
	Sp3 = 0.0
elseif 	iAxisNum = 4						
	Sp4 = 0.0
elseif 	iAxisNum = 5						
	Sp5 = 0.0
elseif 	iAxisNum = 6						
	Sp6 = 0.0
elseif 	iAxisNum = 7						
	Sp7 = 0.0
end

FDEF(iAxisNum).#CPE = 1
MFLAGS(iAxisNum).#HOME = 1
wait 500
RET


HOME_TO_SWITCH:

MFLAGS(iAxisNum).#HOME = 0
FDEF(iAxisNum).#LL=0       ! Disable the iAxis left limit default response
FDEF(iAxisNum).#RL=0       ! Disable the iAxis right limit default response
ENABLE (iAxisNum)          ! Enable the iAxis drive
wait(1000)

if  (IN(0).iAxisNum = 1) 
	JOG/v (iAxisNum), dHomingVel(iAxisNum)*dAxisPitch(iAxisNum)           ! Move to the left  
	TILL (IN(0).iAxisNum = 0)   			! Wait for home switch input
	! Stop motor, let settle, set FPOS and TPOS to 0.0.  Then turn CPE mask back off
	KILL (iAxisNum)
end
! Reduce the motor current saturations
store_CURI = XCURI(iAxisNum)
store_CURV = XCURV(iAxisNum)
XCURI(iAxisNum) = dHomingTorq(iAxisNum)
XCURV(iAxisNum) = dHomingTorq(iAxisNum)
! Do NOT Turn off the CPE response for home to switch
!FDEF(iAxisNum).#CPE = 0
! Jog until critical position error is triggered
JOG/v (iAxisNum), -dHomingVel(iAxisNum)*dAxisPitch(iAxisNum)           ! Move to the left  
								  
TILL (IN(0).iAxisNum = 1)   			! Wait for home switch input
! Stop motor, let settle, set FPOS and TPOS to 0.0.  Then turn CPE mask back off
KILL (iAxisNum)
wait 3000
SET FPOS(iAxisNum) = 0	!@CR2015-12-31 modified home position to simple offset - why move to index?
TPOS(iAxisNum) = 0
! FDEF(iAxisNum).#CPE = 1	!no need to turn turn CPE on because it was not turned off.
! disable motor to return to running torque, and immediately re-enable. This prevents an abrupt move with torque increase.

LOOP 100
	XCURI(iAxisNum) = XCURI(iAxisNum) + 0.01*(store_CURI - dHomingTorq(iAxisNum))
	wait 10	!10ms x 100 = 1 sec ramp
	XCURV(iAxisNum) = XCURV(iAxisNum) + 0.01*(store_CURV - dHomingTorq(iAxisNum))
	wait 10	!10ms x 100 = 1 sec ramp
END
! In case of rounding error, lastly set XCUR* to stored value - should be nearly identical at this point
XCURI(iAxisNum) = store_CURI
XCURV(iAxisNum) = store_CURV

!disable (iAxisNum)
!XCURI(iAxisNum) = store_CURI !@@@ moved here from end of if-thens
!XCURV(iAxisNum) = store_CURV	!@@@ moved here from end of if-thens
!enable (iAxisNum)
wait 1000
PTP/ev (iAxisNum), dHomeOffset(iAxisNum)*dAxisPitch(iAxisNum), dHomingVel(iAxisNum)*dAxisPitch(iAxisNum)
wait 3000
! After moving to home offset, re-zero the axis.  This is now the "working zero" for the axis
SET FPOS(iAxisNum) = 0	!@CR2015-12-31 modified home position to simple offset - why move to index?
TPOS(iAxisNum) = 0
! perform another move to zero (because without, there is residual FPOS data that shows at GUI - looks funny
PTP/ev (iAxisNum), 0.0, dHomingVel(iAxisNum)*dAxisPitch(iAxisNum)
wait 2000

!After homing, change Sp to 0.0 to prevent follow-on move.
OldSP(iAxisNum)=0
if 		iAxisNum = 0								
	Sp0 = 0.0
elseif 	iAxisNum = 1							
	Sp1 = 0.0
elseif 	iAxisNum = 2						
	Sp2 = 0.0
elseif 	iAxisNum = 3						
	Sp3 = 0.0
elseif 	iAxisNum = 4						
	Sp4 = 0.0
elseif 	iAxisNum = 5						
	Sp5 = 0.0
elseif 	iAxisNum = 6						
	Sp6 = 0.0
elseif 	iAxisNum = 7						
	Sp7 = 0.0
end

FDEF(iAxisNum).#CPE = 1
MFLAGS(iAxisNum).#HOME = 1
wait 500
RET







 
HOME_TO_SWITCH_CARZ:

MFLAGS(iAxisNum).#HOME = 0
FDEF(iAxisNum).#LL=0       ! Disable the iAxis left limit default response
FDEF(iAxisNum).#RL=0       ! Disable the iAxis right limit default response
ENABLE (iAxisNum)          ! Enable the iAxis drive
wait(1000)

if  (SAFIN(iAxisNum).#LL = 1) 
	JOG/v (iAxisNum), dHomingVel(iAxisNum)*dAxisPitch(iAxisNum)           ! Move to the left  
	TILL (SAFIN(iAxisNum).#LL  = 0)   			! Wait for home switch input
	! Stop motor, let settle, set FPOS and TPOS to 0.0.  Then turn CPE mask back off
	KILL (iAxisNum)
end
! Reduce the motor current saturations
store_CURI = XCURI(iAxisNum)
store_CURV = XCURV(iAxisNum)
XCURI(iAxisNum) = dHomingTorq(iAxisNum)
XCURV(iAxisNum) = dHomingTorq(iAxisNum)
! Do NOT Turn off the CPE response for home to switch
!FDEF(iAxisNum).#CPE = 0
! Jog until critical position error is triggered
JOG/v (iAxisNum), -dHomingVel(iAxisNum)*dAxisPitch(iAxisNum)           ! Move to the left  
   			! Wait for home switch input
TILL (SAFIN(iAxisNum).#LL = 1)
! Stop motor, let settle, set FPOS and TPOS to 0.0.  Then turn CPE mask back off
KILL (iAxisNum)
wait 3000

! @JR 2021-10-18 - Move down until upper limit switch opens
JOG/v (iAxisNum), dHomingVel(iAxisNum)*dAxisPitch(iAxisNum)
TILL (SAFIN(iAxisNum).#LL = 0)
KILL (iAxisNum)
wait 3000

SET FPOS(iAxisNum) = 0	!@CR2015-12-31 modified home position to simple offset - why move to index?
TPOS(iAxisNum) = 0
! FDEF(iAxisNum).#CPE = 1	!no need to turn turn CPE on because it was not turned off.
! disable motor to return to running torque, and immediately re-enable. This prevents an abrupt move with torque increase.

!LOOP 100
!	XCURI(iAxisNum) = XCURI(iAxisNum) + 0.01*(store_CURI - dHomingTorq(iAxisNum))
!	wait 10	!10ms x 100 = 1 sec ramp
!	XCURV(iAxisNum) = XCURV(iAxisNum) + 0.01*(store_CURV - dHomingTorq(iAxisNum))
!	wait 10	!10ms x 100 = 1 sec ramp
!END
! In case of rounding error, lastly set XCUR* to stored value - should be nearly identical at this point
XCURI(iAxisNum) = store_CURI
XCURV(iAxisNum) = store_CURV

!disable (iAxisNum)
!XCURI(iAxisNum) = store_CURI !@@@ moved here from end of if-thens
!XCURV(iAxisNum) = store_CURV	!@@@ moved here from end of if-thens
!enable (iAxisNum)
wait 1000
PTP/ev (iAxisNum), dHomeOffset(iAxisNum)*dAxisPitch(iAxisNum), dHomingVel(iAxisNum)*dAxisPitch(iAxisNum)
wait 3000
! After moving to home offset, re-zero the axis.  This is now the "working zero" for the axis
SET FPOS(iAxisNum) = 0	!@CR2015-12-31 modified home position to simple offset - why move to index?
TPOS(iAxisNum) = 0
! perform another move to zero (because without, there is residual FPOS data that shows at GUI - looks funny
PTP/ev (iAxisNum), 0.0, dHomingVel(iAxisNum)*dAxisPitch(iAxisNum)
wait 2000

!After homing, change Sp to 0.0 to prevent follow-on move.
OldSP(iAxisNum)=0
if 		iAxisNum = 0								
	Sp0 = 0.0
elseif 	iAxisNum = 1							
	Sp1 = 0.0
elseif 	iAxisNum = 2						
	Sp2 = 0.0
elseif 	iAxisNum = 3						
	Sp3 = 0.0
elseif 	iAxisNum = 4						
	Sp4 = 0.0
elseif 	iAxisNum = 5						
	Sp5 = 0.0
elseif 	iAxisNum = 6						
	Sp6 = 0.0
elseif 	iAxisNum = 7						
	Sp7 = 0.0
end

FDEF(iAxisNum).#CPE = 1
MFLAGS(iAxisNum).#HOME = 1
wait 500
RET

	
 
 
 
 

SET_CURRENTPOS_HOME:

SET FPOS(iAxisNum) = 0
TPOS(iAxisNum) = 0	!@@@ added
FDEF(iAxisNum).#CPE = 1
MFLAGS(iAxisNum).#HOME = 1
ENABLE (iAxisNum) 
! Move to 0 to clean-up residual FPOS error on GUI
PTP/ev (iAxisNum), 0, 180
wait 2000
!After homing, change Sp to 0.0 to prevent follow-on move.
OldSP(iAxisNum)=0
if 		iAxisNum = 0								
	Sp0 = 0.0
elseif 	iAxisNum = 1							
	Sp1 = 0.0
elseif 	iAxisNum = 2						
	Sp2 = 0.0
elseif 	iAxisNum = 3						
	Sp3 = 0.0
elseif 	iAxisNum = 4						
	Sp4 = 0.0
elseif 	iAxisNum = 5						
	Sp5 = 0.0
elseif 	iAxisNum = 6						
	Sp6 = 0.0
elseif 	iAxisNum = 7						
	Sp7 = 0.0
end
RET


HOME_TO_TORQ_AVP:


MFLAGS(iAxisNum).#HOME = 0
FDEF(iAxisNum).#LL=0       ! Disable the iAxis left limit default response
FDEF(iAxisNum).#RL=0       ! Disable the iAxis right limit default response
ENABLE (iAxisNum)          ! Enable the iAxis drive
wait(100)
JOG/v (iAxisNum), dHomingVel(iAxisNum)*dAxisPitch(iAxisNum)           ! Move to the left  
wait(1000)
KILL iAxisNum
!reduce the motor current saturations
store_CURI = XCURI(iAxisNum)
store_CURV = XCURV(iAxisNum)

real XCURI_actural, HmCurrent_Of_Drive, FtCurrent_Of_Drive
HmCurrent_Of_Drive = dHomingTorq(iAxisNum)*(4*XRMS(iAxisNum))/100.0
FtCurrent_Of_Drive = dFaultTorq(iAxisNum)*(4*XRMS(iAxisNum))/100.0

XCURI(iAxisNum) = HmCurrent_Of_Drive
XCURV(iAxisNum) = HmCurrent_Of_Drive
!CERRI(iAxisNum)=0.01*dHomingTorq(iAxisNum)*CERRI(iAxisNum)
!CERRV(iAxisNum)=0.01*dHomingTorq(iAxisNum)*CERRV(iAxisNum)
! Turn off the CPE response
FDEF(iAxisNum).#CPE = 0

JOG/v (iAxisNum), -dHomingVel(iAxisNum)*dAxisPitch(iAxisNum)           ! Move to the left  
TILL (FAULT(iAxisNum).#CPE = 1)   			! Wait for the critical PE
KILL iAxisNum

if (dAxisPitch(iAxisNum)>0.01)
	dCE(iAxisNum)=FPOS(iAxisNum)/dAxisPitch(iAxisNum)+dHomeOffset(iAxisNum)
end
block
SET FPOS(iAxisNum) = 0.0 -dHomeOffset(iAxisNum)*dAxisPitch(iAxisNum)
PTP/e (iAxisNum), 100!0.0
end



deltaI = FtCurrent_Of_Drive - HmCurrent_Of_Drive
deltaV = FtCurrent_Of_Drive - HmCurrent_Of_Drive
LOOP 100
	XCURI(iAxisNum) = XCURI(iAxisNum) + 0.01*(deltaI)
	XCURV(iAxisNum) = XCURV(iAxisNum) + 0.01*(deltaV)
	wait 20	!10ms x 100 = 1 sec ramp
END

XCURI(iAxisNum) = FtCurrent_Of_Drive !@@@ moved here from end of if-thens
XCURV(iAxisNum) = FtCurrent_Of_Drive	!@@@ moved here from end of if-thens
CERRV(iAxisNum) = store_CERRV0
CERRI(iAxisNum) = store_CERRI0
FDEF(iAxisNum).#CPE = 1

BREAK iAxisNum
!SET FPOS(iAxisNum) = 0.0 -dHomeOffset(iAxisNum)*dAxisPitch(iAxisNum)	!@CR2015-12-31 modified home position to simple offset - why move to index?
PTP/ev (iAxisNum), 0.0, dHomingVel(iAxisNum)*dAxisPitch(iAxisNum)


!After homing, change Sp to 0.0 to prevent follow-on move.
OldSP(iAxisNum)=0
if 		iAxisNum = 0								
	Sp0 = 0.0
elseif 	iAxisNum = 1							
	Sp1 = 0.0
elseif 	iAxisNum = 2						
	Sp2 = 0.0
elseif 	iAxisNum = 3						
	Sp3 = 0.0
elseif 	iAxisNum = 4						
	Sp4 = 0.0
elseif 	iAxisNum = 5						
	Sp5 = 0.0
elseif 	iAxisNum = 6						
	Sp6 = 0.0
elseif 	iAxisNum = 7						
	Sp7 = 0.0
end
FDEF(iAxisNum).#CPE = 1
MFLAGS(iAxisNum).#HOME = 1
RET

HOME_TO_SWITCH_CAR:
REAL rMax, rMin
INT iCARAIChannel
iCARAIChannel = 0

MFLAGS(iAxisNum).#HOME = 0
FDEF(iAxisNum).#LL=0       ! Disable the iAxis left limit default response
FDEF(iAxisNum).#RL=0       ! Disable the iAxis right limit default response
ENABLE (iAxisNum)          ! Enable the iAxis drive
wait(1000)
! Jog until critical position error is triggered
if (FtdCarHomeSW.iAxisNum = 1)
	!JOG/v (iAxisNum), dHomingVel(iAxisNum)           ! Move to the left  
	!TILL (FtdCarHomeSW(iAxisNum) = 0)   			! Wait for home switch input
	!BREAK (iAxisNum)
	PTP/er (iAxisNum), 15
end

if(iType(iAxisNum)=4007)	! if it is an analog sensor, perform sensor calibration first.
! find which analog channel is the home sensor at

	if AnalogCARAxis(0)=iAxisNum
		iCARAIChannel = 0
	elseif AnalogCARAxis(1)=iAxisNum
		iCARAIChannel = 1
	elseif AnalogCARAxis(2)=iAxisNum
		iCARAIChannel = 2
	elseif AnalogCARAxis(3)=iAxisNum
		iCARAIChannel = 3
	end
! scan the analog signal and acq 1000 points from the analog signal while spinning
	DC/S iAxisNum,Datalog,1000,100,FPOS(iAxisNum),AIN(iCARAIChannel)
	JOG/v (iAxisNum), 4 
	till (^AST(iAxisNum).#DC)	!wait DAQ to finish
	KILL iAxisNum
	WAIT 2000
! find the median in analog signal	
	rMax = MAX(Datalog, 1, 1, 0,999)
	rMin = MIN(Datalog, 1, 1, 0,999)
	if (rMax-rMin)>1.0
		AnologHomeSignalMedian(iCARAIChannel) = (rMax + rMin)/2.0
	else
		disable(iAxisNum)  
		iControlProcessState(iAxisNum) = 404
		RET
	end	
! find the position where is the peak signal
	int thepeakposIndex
	thepeakposIndex = MAXI(Datalog, 1, 1, 0,999)
! move the CAR to the neighbourhood of the peak	
	PTP/e (iAxisNum), Datalog(0)(thepeakposIndex)+15
	WAIT 3000
end





real FirstEdge, SecondEdge
!Find the leading edge of the home sensor
JOG/v (iAxisNum), -dHomingVel(iAxisNum)           ! Move to the left  
TILL (FtdCarHomeSW.iAxisNum = 1)   			! Wait for home switch input
FirstEdge = FPOS(iAxisNum)
KILL iAxisNum
WAIT 2000
!back off
PTP/e (iAxisNum), FirstEdge + 10
!low speed homing
JOG/v (iAxisNum), -dHomingVel(iAxisNum)           ! Move to the left  
TILL (FtdCarHomeSW.iAxisNum = 1)   			! Wait for home switch input
FirstEdge = FPOS(iAxisNum)
KILL iAxisNum
WAIT 2000
PTP/e (iAxisNum), FirstEdge - 15
!low speed homing
JOG/v (iAxisNum), dHomingVel(iAxisNum)           ! Move to the left  
TILL (FtdCarHomeSW.iAxisNum = 1) 
SecondEdge = FPOS(iAxisNum)
KILL iAxisNum
WAIT 2000
PTP/ev (iAxisNum), 0.5*(FirstEdge+SecondEdge)+dHomeOffset(iAxisNum), dHomingVel(iAxisNum)
wait 3000

SET FPOS(iAxisNum) = 0	!@CR2015-12-31 modified home position to simple offset - why move to index?
TPOS(iAxisNum) = 0
! After moving to home offset, re-zero the axis.  This is now the "working zero" for the axis
SET FPOS(iAxisNum) = 0	!@CR2015-12-31 modified home position to simple offset - why move to index?
TPOS(iAxisNum) = 0
! perform another move to zero (because without, there is residual FPOS data that shows at GUI - looks funny
FDEF(iAxisNum).#CPE = 1
MFLAGS(iAxisNum).#HOME = 1
wait 500

PTP/ev (iAxisNum), 0.0, dHomingVel(iAxisNum)
wait 2000

!After homing, change Sp to 0.0 to prevent follow-on move.
OldSP(iAxisNum)=0
if 		iAxisNum = 0								
	Sp0 = 0.0
elseif 	iAxisNum = 1							
	Sp1 = 0.0
elseif 	iAxisNum = 2						
	Sp2 = 0.0
elseif 	iAxisNum = 3						
	Sp3 = 0.0
elseif 	iAxisNum = 4						
	Sp4 = 0.0
elseif 	iAxisNum = 5						
	Sp5 = 0.0
elseif 	iAxisNum = 6						
	Sp6 = 0.0
elseif 	iAxisNum = 7						
	Sp7 = 0.0
end

RET


HOME_TO_TORQ_VLV:

	MFLAGS(iAxisNum).#HOME = 0
	FDEF(iAxisNum).#LL=0       ! Disable the iAxis left limit default response
	FDEF(iAxisNum).#RL=0       ! Disable the iAxis right limit default response
	ENABLE (iAxisNum)          ! Enable the iAxis drive
	wait(100)
	JOG/v (iAxisNum), dHomingVel(iAxisNum)*dAxisPitch(iAxisNum)           ! Move to the left  
	wait(1000)
	KILL iAxisNum
	!reduce the motor current saturations
	store_CURI = XCURI(iAxisNum)
	store_CURV = XCURV(iAxisNum)
	XCURI(iAxisNum) = dHomingTorq(iAxisNum)
	XCURV(iAxisNum) = dHomingTorq(iAxisNum)
	!CERRI(iAxisNum)=0.01*dHomingTorq(iAxisNum)*CERRI(iAxisNum)
	!CERRV(iAxisNum)=0.01*dHomingTorq(iAxisNum)*CERRV(iAxisNum)
	! Turn off the CPE response
	FDEF(iAxisNum).#CPE = 0
	ENABLE (iAxisNum)          ! Enable the iAxis drive
	wait(100)
	JOG/v (iAxisNum), -dHomingVel(iAxisNum)*dAxisPitch(iAxisNum)           ! Move to the left  
	TILL (FAULT(iAxisNum).#CPE = 1)   			! Wait for the critical PE
	KILL iAxisNum

	if (dAxisPitch(iAxisNum)>0.01)
		dCE(iAxisNum)=FPOS(iAxisNum)/dAxisPitch(iAxisNum)+dHomeOffset(iAxisNum)
	end
	block
		SET FPOS(iAxisNum) = 0.0 -dHomeOffset(iAxisNum)*dAxisPitch(iAxisNum)
		!PTP/e (iAxisNum), 10!0.0
	end
	wait 1000
	deltaI = store_CURI - dHomingTorq(iAxisNum)
	deltaV = store_CURV - dHomingTorq(iAxisNum)
	LOOP 100
		XCURI(iAxisNum) = XCURI(iAxisNum) + 0.01*(deltaI)
		XCURV(iAxisNum) = XCURV(iAxisNum) + 0.01*(deltaV)
		wait 20	!10ms x 100 = 1 sec ramp
	END

	XCURI(iAxisNum) = store_CURI !@@@ moved here from end of if-thens
	XCURV(iAxisNum) = store_CURV	!@@@ moved here from end of if-thens
	CERRV(iAxisNum) = store_CERRV0
	CERRI(iAxisNum) = store_CERRI0
	
	
	
	
	
	BREAK iAxisNum
	!SET FPOS(iAxisNum) = 0.0 -dHomeOffset(iAxisNum)*dAxisPitch(iAxisNum)	!@CR2015-12-31 modified home position to simple offset - why move to index?
	PTP/ev (iAxisNum), 0.0, dHomingVel(iAxisNum)*dAxisPitch(iAxisNum)


	!After homing, change Sp to 0.0 to prevent follow-on move.
	OldSP(iAxisNum)=0
	if 		iAxisNum = 0								
		Sp0 = 0.0
	elseif 	iAxisNum = 1							
		Sp1 = 0.0
	elseif 	iAxisNum = 2						
		Sp2 = 0.0
	elseif 	iAxisNum = 3						
		Sp3 = 0.0
	elseif 	iAxisNum = 4						
		Sp4 = 0.0
	elseif 	iAxisNum = 5						
		Sp5 = 0.0
	elseif 	iAxisNum = 6						
		Sp6 = 0.0
	elseif 	iAxisNum = 7						
		Sp7 = 0.0
	end
	FDEF(iAxisNum).#CPE = 1
	MFLAGS(iAxisNum).#HOME = 1

RET



VAT_VLV_CONTROLLER:


! VAT VLV CONTROL PROG
real PresWorkingSP_v, PresWorkingSP_v_prev, Valve_Scaling_Factor_Prev, Pres_min_Prev, Pres_max_Prev, SafetyShutDownPressure_Prev
real pressure_sp_v, pressure_measured_V, pressure_measured_V_prev, PressureSP_Prev, PressurePV_Prev, Pressure_Ramp_rate_Prev
real pressurePID_Loop_Time	!ms
global real	IG_SLOP, IG_INTERCEPT, FilteredIGSignal, IGSignals(20)
real PressureControllerPositionCmd
real FB_error, FB_ERR_DELTA, pressure_sp_v_prev
int IGValid, InRamping, iNumOfRampPoints, iCurrentRampingPoint, iRampingDir, iSPChanged, iSwitchedToPressControlMode, iControlModePrev
real jogVLVvelocity, RampingVel
REAL KP_OUTPUT, KI_OUTPUT, KD_OUTPUT, PID_OUTPUT_LIMIT
REAL ANALOG_OFFSET
real tmptime, tmptimed, tmpIG, tmpRampCycleStartTime, tmpRampCycleTime, tmpRamprate, tmpStartPressure, RampRateRatio
real PressureChangeThreshold


READ dValve_scaling_factor, dValve_scaling_factor		!A calibratable parameter. Value range 6.0~20, default 6.0.
READ dPressureSP, dPressureSP							!Torr, pressure setpoint, 5e-11 to 1e-4
READ dPressureRamprate, dPressureRamprate				!Pressure ramp rate expressed at Torr/min
READ dPres_min, dPres_min								!Min Pressure setpoint, 5e-11
READ dPres_max, dPres_max								!Max pressure setpoint, 5e-5
READ dSafetyShutDownPressure, dSafetyShutDownPressure	!Defines a pressure threshold for safety.  (5e-11 to 1e-4) 8e-5 is default




PressureChangeThreshold = 0.1 !v
IGValid = 1
iSPChanged = 0
ANALOG_OFFSET = 0.0 !-1.0!0.4
XCURI(iAxisNum) = 40!5.5!3.3! 47.5	!Max current at Idle
XCURV(iAxisNum) = 40 !5!3.6! 95	!Max current at moving
XRMS(iAxisNum) =  27!5!2.9! 27	!RMS current limit
CERRI(iAxisNum) = 1.0!0.8	!Critical Position Error idle
CERRV(iAxisNum) = 1.0!0.8	!Critical Position Error constant velocity
CERRA(iAxisNum) = 1.2!1.0	!Critical Position Error accerleration
VEL(iAxisNum) = 100
ACC(iAxisNum) = 100
DEC(iAxisNum) = 100
KDEC(iAxisNum)= 100
JERK(iAxisNum)= 100
!====================USER CHANGEABLE==============================
!dHomingTorq(iAxisNum) = 3.8 !3.67 !4.8
!dHomingVel(iAxisNum) = 10
tmptimed = 50
OutPressure_KP1 = 1.0
pressurePID_KP1 = 0.01!0.035!0.02!0.0006!0.01!0.0045 !0.003 !6000 !100000!3200000!9357253!630000 !935725!0.1          !PROPORTINAL GAIN
pressurePID_KI1 = 0.00001!0.00292!1e-14!0!1e-7!0.000001 !30! 60!0.00000000003     !INTEGRAL GAIN
pressurePID_KD1 = 0!1e-16!0!300!0.0          !DERIVATIVE GAIN
PID_OUTPUT_LIMIT =  4 !0.2
iControlMode1 = 0	!0=position mode; 1=pressure mode
iControlModePrev = 0
iSwitchedToPressControlMode = 0
InRamping = 0
!MaxSP1 = 320 !260!61.2	!?
IG_SLOP = 1.0
IG_INTERCEPT = 11.0

!================================================================
pressurePID_Loop_Time = 50		!ms DO NOT CHANGE
KI_OUTPUT = 0.0		!DO NOT CHANGE
!Start analog data acquisition
STOPDC
DC/C IGSignals, 20, 5, AIN(iAxisNum)
!DC/C IGSignals, 20, 5, RunningAverage
ENABLE (iAxisNum)          ! Enable the iAxis drive
wait 1000
PTP/e iAxisNum, 10
!=======INI Setpoint==================================

!SafetyShutDownPressure1 = 8e-5
!Pressure_Ramp_rate1 = 1e-9*60 !1.0e-8*0.02	!20.		!Torr/s
!Pres_max1 = 5e-5
!Pres_min1 = 5e-11
!Valve_Scaling_Factor1 = 10.813!6.0

! initialize the valve scale factor
IF dValve_scaling_factor(iAxisNum) = 0.0
	dValve_scaling_factor(iAxisNum) = 10.813
	WRITE dValve_scaling_factor, dValve_scaling_factor
END
Valve_Scaling_Factor1 = dValve_scaling_factor(iAxisNum)
Valve_Scaling_Factor_Prev = Valve_Scaling_Factor1

! initialize the pressure setpoint
CALL GETIGSIGNAL

if PressureSP1<=3e-11
BLOCK
	PressureSP1=PressurePV1
	PressureSP_Prev = PressureSP1
	dPressureSP(iAxisNum) = PressurePV1
END	
	WRITE dPressureSP, dPressureSP
end
PressureSP1 = dPressureSP(iAxisNum)
PressureSP_Prev = PressureSP1
pressure_sp_v = (log10(PressureSP1) + IG_INTERCEPT)/IG_SLOP
pressure_sp_v_prev = pressure_sp_v

! initialize the working setpoint
PressureWKSP1 = PressurePV1
PresWorkingSP_v = pressure_measured_V

! initialize the pressure ramp rate
IF dPressureRamprate(iAxisNum) = 0.0
	dPressureRamprate(iAxisNum) = 6e-8	!Torr/s
	WRITE dPressureRamprate, dPressureRamprate	
END
Pressure_Ramp_rate1 = dPressureRamprate(iAxisNum)
Pressure_Ramp_rate_Prev = Pressure_Ramp_rate1
tmpRamprate = Pressure_Ramp_rate1/60.0

! initialize the MIN pressure setpoint
IF dPres_min(iAxisNum) = 0.0
	dPres_min(iAxisNum) = 5e-11
	WRITE dPres_min, dPres_min	
END
Pres_min1 = dPres_min(iAxisNum)
Pres_min_Prev = Pres_min1

! initialize the MAX pressure setpoint
IF dPres_max(iAxisNum) = 0.0
	dPres_max(iAxisNum) = 5e-5
	WRITE dPres_max, dPres_max	
END
Pres_max1 = dPres_max(iAxisNum)
Pres_max_Prev = Pres_max1
! initialize the safety shutdown pressure
IF dSafetyShutDownPressure(iAxisNum) = 0.0
	dSafetyShutDownPressure(iAxisNum) = 8e-5
	WRITE dSafetyShutDownPressure, dSafetyShutDownPressure	
END
SafetyShutDownPressure1 = dSafetyShutDownPressure(iAxisNum)
SafetyShutDownPressure_Prev = SafetyShutDownPressure1

!=======INI controller================================
CALL ERRCALC
FB_ERR_DELTA = FB_error
ControlProcessState1 = 10	!initialized
!=======Controller Loop===============================
iCurrentRampingPoint = 0
WHILE 1!; BLOCK
	LOOPSTART:
	tmptime = TIME
	if IGValid = 1
		if iControlMode1 = 1	!1 = pressure control mode; 0 = position mode
			
			if iSwitchedToPressControlMode = 1	!	!Drive the valve out of the deadband
				ControlProcessState1 = 1		!moving out from safety zone to work zone
				CALL GETIGSIGNAL
				tmpIG = pressure_measured_V !AIN(iAxisNum)
				if FPOS(iAxisNum) >= 2	
					kill iAxisNum
					break iAxisNum
					PTP/re iAxisNum, -2.0
				end
				CALL GETIGSIGNAL
				if pressure_measured_V - tmpIG < PressureChangeThreshold !(AIN(iAxisNum)- tmpIG) < 1
					!tmpIG = AIN(iAxisNum)
					CALL GETIGSIGNAL
					tmpIG = pressure_measured_V !AIN(iAxisNum)
					JOG/v iAxisNum, 2.0
					!till (AIN(iAxisNum)- tmpIG) > 1	! seeing 1e-11 changes from the ION gauge
					while (pressure_measured_V - tmpIG < PressureChangeThreshold)&iControlMode1	! seeing 1e-11 changes from the ION gauge
						CALL GETIGSIGNAL
					end
					kill iAxisNum
					break iAxisNum
					PTP/re iAxisNum, -2.0
					wait 2000
				end
				Call TrajPlanning
				iSwitchedToPressControlMode = 0		
			end	
			
			CALL GETIGSIGNAL
			tmpStartPressure = PressurePV1
			PresWorkingSP_v_prev = PresWorkingSP_v
			if (InRamping = 1 )&(iNumOfRampPoints>=1 )&(iCurrentRampingPoint < iNumOfRampPoints-2  )	!ramping loop
				if (iRampingDir = 1)
					ControlProcessState1 = 2	!ramping up
					PressureWKSP1 = PressureWKSP1 + tmpRamprate
					if (PressureWKSP1 > (PressureSP1 - tmpRamprate)) !| (abs (PressureSP1 - PressurePV1) < tmpRamprate))
						PressureWKSP1 = PressureSP1
						PresWorkingSP_v = (log10(PressureWKSP1) + IG_INTERCEPT)/IG_SLOP
						KI_OUTPUT = 0
						InRamping = 0
						FB_ERR_DELTA = FB_error
						goto LOOPMID
					end	
					
					PresWorkingSP_v = (log10(PressureWKSP1) + IG_INTERCEPT)/IG_SLOP
					jogVLVvelocity =  abs(PresWorkingSP_v - PresWorkingSP_v_prev)*Valve_Scaling_Factor1*10!3*pressure_measured_V
					if jogVLVvelocity > 0.8
						jogVLVvelocity = 0.8
					end	
					JOG/v iAxisNum, jogVLVvelocity				
					
					tmpRampCycleStartTime = TIME
					while (((TIME - tmpRampCycleStartTime) < 1000 ))! | (PressurePV1 < PressureWKSP1))
						CALL GETIGSIGNAL
						if (iControlMode1 = 0) | (iSPChanged = 1)
							if IGValid = 1
								kill iAxisNum
							end	
							if (iSPChanged = 1)
								iSPChanged = 0
							end						
							goto LOOPSTART
						end
						if (PressurePV1 >= PressureWKSP1)
							kill iAxisNum
						end

						!CALL PIDLOOP
					end
				else
					ControlProcessState1 = 3	!ramping down
					PressureWKSP1 = PressureWKSP1 - tmpRamprate
					if (PressureWKSP1 < (PressureSP1 + tmpRamprate)) ! | (abs (PressureSP1 - PressurePV1) < tmpRamprate))
						PressureWKSP1 = PressureSP1
						PresWorkingSP_v = (log10(PressureWKSP1) + IG_INTERCEPT)/IG_SLOP
						KI_OUTPUT = 0
						InRamping = 0
						FB_ERR_DELTA = FB_error
						goto LOOPMID
					end	
					PresWorkingSP_v = (log10(PressureWKSP1) + IG_INTERCEPT)/IG_SLOP
					jogVLVvelocity =  abs(PresWorkingSP_v - PresWorkingSP_v_prev)*Valve_Scaling_Factor1*10!3*pressure_measured_V
					if jogVLVvelocity > 0.8
						jogVLVvelocity = 0.8
					end	
					JOG/v iAxisNum, -jogVLVvelocity				
					
					tmpRampCycleStartTime = TIME
					while (((TIME - tmpRampCycleStartTime) < 1000 )) !	| (PressurePV1 > PressureWKSP1))			
						CALL GETIGSIGNAL
						if (iControlMode1 = 0) | (iSPChanged = 1)
							if IGValid = 1
								kill iAxisNum
							end	
							if (iSPChanged = 1)
								iSPChanged = 0
							end						
							goto LOOPSTART
						end
						if (PressurePV1 <= PressureWKSP1)
							kill iAxisNum
						end

	!					CALL PIDLOOP
					end
				end
				LOOPMID:
				iCurrentRampingPoint = iCurrentRampingPoint + 1
			else	!Pressure PID Loop
				ControlProcessState1 = 4	!holding pressure Setpoint
				if InRamping = 1
					PressureWKSP1 = PressureSP1
					PresWorkingSP_v = (log10(PressureWKSP1) + IG_INTERCEPT)/IG_SLOP				
					KI_OUTPUT = 0
					InRamping = 0
				end	
				CALL PIDLOOP
			end
		else
			!if (ControlProcessState1 <> 5)&(IGValid = 1)
			!	kill iAxisNum
			!	if (abs(dPositiveLimit(iAxisNum)-dNegtiveLimit(iAxisNum))>0.00001)
			!		Sp1= 100.0*abs(RPOS(iAxisNum) - dNegtiveLimit(iAxisNum))/abs(dPositiveLimit(iAxisNum)-dNegtiveLimit(iAxisNum))
			!	else	
			!		Sp1=0.0
			!	end
			!end
			ControlProcessState1 = 5	!in position mode
			CALL GETIGSIGNAL
			PressureWKSP1 = PressurePV1
		end	!if pressure or position mode
	end	!if IGValid
	!wait pressurePID_Loop_Time-8
	tmptimed = TIME - tmptime
END!; END
RET



PIDLOOP:
BLOCK
	int PIDTime
	PIDTime = TIME
	ControlProcessState1 = 10
	if StatusWord1.10 = 0
		if 	InRamping = 1
			!Pressure_Ramp_rate1 = 1e-9
			pressurePID_KP1 = 0.02!0.035!0.02!0.0006!0.01!0.0045 !0.003 !6000 !100000!3200000!9357253!630000 !935725!0.1          !PROPORTINAL GAIN
			pressurePID_KI1 = 0.00001!0.00292!1e-14!0!1e-7!0.000001 !30! 60!0.00000000003     !INTEGRAL GAIN
			pressurePID_KD1 = 0!1e-16!0!300!0.0          !DERIVATIVE GAIN
		else
			if PressureSP1 <5e-7
				!Pressure_Ramp_rate1 = 1e-9
				pressurePID_KP1 = 0.001!0.035!0.02!0.0006!0.01!0.0045 !0.003 !6000 !100000!3200000!9357253!630000 !935725!0.1          !PROPORTINAL GAIN
				pressurePID_KI1 = 0.00!0.00292!1e-14!0!1e-7!0.000001 !30! 60!0.00000000003     !INTEGRAL GAIN
				pressurePID_KD1 = 0!1e-16!0!300!0.0          !DERIVATIVE GAIN
			else
				!Pressure_Ramp_rate1 = 5e-9
				pressurePID_KP1 = 0.01!0.035!0.02!0.0006!0.01!0.0045 !0.003 !6000 !100000!3200000!9357253!630000 !935725!0.1          !PROPORTINAL GAIN
				pressurePID_KI1 = 0.00001!0.00292!1e-14!0!1e-7!0.000001 !30! 60!0.00000000003     !INTEGRAL GAIN
				pressurePID_KD1 = 0!1e-16!0!300!0.0          !DERIVATIVE GAIN
			end
		end
	end
	CALL ERRCALC
	if IGValid = 1
		!CALCULATE PROPORTIONAL OUTPUT
		KP_OUTPUT = pressurePID_KP1 * FB_error
		!CALCULATE DERIVATIVE OUTPUT
		KD_OUTPUT = pressurePID_KD1 * ((FB_error - FB_ERR_DELTA)/(pressurePID_Loop_Time*0.001))
		!CALCULATE INTEGRAL OUTPUT. BASED ON TRAPIZOIDAL RIEMANN SUM
		KI_OUTPUT = KI_OUTPUT + (pressurePID_KI1 *(pressurePID_Loop_Time*0.001/2) * (FB_error + FB_ERR_DELTA))
		FB_ERR_DELTA = FB_error
		!Intergral limits
		
		PressureControllerPositionCmd = (KP_OUTPUT + KD_OUTPUT + KI_OUTPUT)*Valve_Scaling_Factor1
		if (PressureControllerPositionCmd > PID_OUTPUT_LIMIT)
			 PressureControllerPositionCmd = PID_OUTPUT_LIMIT
		end
		if  (PressureControllerPositionCmd < -PID_OUTPUT_LIMIT )
			 PressureControllerPositionCmd = -PID_OUTPUT_LIMIT
		end
		if iControlMode1 = 1	!added to jump out the PID loop when fault happens 9/10/2020
			break iAxisNum
			PTP/r iAxisNum, PressureControllerPositionCmd!*Valve_Scaling_Factor2!, PressureControllerPositionCmd/0.02
			till ((TIME-PIDTime)>= pressurePID_Loop_Time) | (iControlMode1 = 0)	!!changed to jump out the PID loop when fault happens 9/10/2020
		end
!	else
!		break iAxisNum
!		PTP/e iAxisNum, 0
	end
	
END
RET

TrajPlanning:
	!CALL MeasurePressure
	real currentPV
	CALL GETIGSIGNAL
	currentPV = PressurePV1
	PressureWKSP1 = currentPV
	if (PressureSP1 > currentPV)!PressureWKSP1)	!increase
		iNumOfRampPoints = FLOOR( (PressureSP1 - currentPV)/tmpRamprate)
		iRampingDir = 1 !Ramping up
	else
		iNumOfRampPoints = FLOOR( (currentPV - PressureSP1)/tmpRamprate)
		iRampingDir = -1 !Ramping Down
	end
	InRamping = 1
	iCurrentRampingPoint = 0	
RET

MeasurePressure:
BLOCK
	pressure_measured_V_prev = pressure_measured_V
	pressure_measured_V = 0.1*(AIN(iAxisNum)+ANALOG_OFFSET)
	PressurePV_Prev = PressurePV1
	PressurePV1 = pow(10, (IG_SLOP*pressure_measured_V-IG_INTERCEPT))
END
ret

ERRCALC:
	CALL GETIGSIGNAL
	!CALL MeasurePressure
	!FB_error = pressure_sp_v - pressure_measured_V
	FB_error = OutPressure_KP1*(PresWorkingSP_v - pressure_measured_V) - (pressure_measured_V - pressure_measured_V_prev)

RET

GETIGSIGNAL:
	BLOCK
		real sum
		sum = (IGSignals(0)+IGSignals(1)+IGSignals(2)+IGSignals(3)+IGSignals(4)+IGSignals(5)+IGSignals(6)+IGSignals(7)+IGSignals(8)+IGSignals(9)+IGSignals(10)+IGSignals(11)+IGSignals(12)+IGSignals(13)+IGSignals(14)+IGSignals(15)+IGSignals(16)+IGSignals(17)+IGSignals(18)+IGSignals(19))/20
		pressure_measured_V_prev = pressure_measured_V
		pressure_measured_V = 0.1*(sum + ANALOG_OFFSET)
		PressurePV_Prev = PressurePV1
		PressurePV1 = pow(10, (IG_SLOP*pressure_measured_V-IG_INTERCEPT))
	END
RET



ON (iType(iAxisNum)=1040)& ((PressurePV1 > SafetyShutDownPressure1)|(POG_NO_FLT=0))	!added POG_NO_FLT on 9/10/2020
BLOCK
	ControlProcessState1 = 100	!Safety Shutdown
	iControlMode1 = 0
	kill iAxisNum
	break iAxisNum
	PTP/e iAxisNum, 0
END	
RET

ON (iType(iAxisNum)=1040)& (AIN(iAxisNum) < 14.8)!6.9897)
BLOCK
	ControlProcessState1 = 101	!ION GAUGE FAULT
	iControlMode1 = 0
	IGValid = 0
	kill iAxisNum
	break iAxisNum
	PTP/e iAxisNum, 0
END	
RET


!on (iType(iAxisNum)=1040)& (FPOS(iAxisNum)>MaxSP1) !| (FPOS(iAxisNum)<0.0)
!BLOCK
!	ControlProcessState1 = 102	!valve position out of range
!	kill iAxisNum
!	break iAxisNum
!	PTP/e iAxisNum,0
!END
!ret

!on (iType(iAxisNum)=1040)& (FPOS(iAxisNum)<MinSP1)
!	KILL iAxisNum
!	PTP/e iAxisNum,0
!ret



on (iType(iAxisNum)=1040)&(ControlProcessState1 > 0)& (abs(PressureSP1 - PressureSP_Prev)>1e-10)
	ControlProcessState1 = 6	!Pressure SP changed
	If (PressureSP1<Pres_min1) | (PressureSP1>Pres_max1)
		PressureSP1 = PressureSP_Prev
		ControlProcessState1 = 102
	else
		pressure_sp_v_prev = pressure_sp_v
		pressure_sp_v = (log10(PressureSP1) + IG_INTERCEPT)/IG_SLOP
		PressureSP_Prev = PressureSP1
		tmpRamprate = Pressure_Ramp_rate1/60.0
		call TrajPlanning
		iSPChanged = 1
		dPressureSP(1) = PressureSP1 
		WRITE dPressureSP, dPressureSP
	END	
RET


on (iType(iAxisNum)=1040)&(ControlProcessState1 > 0)&  (abs(Pressure_Ramp_rate1 - Pressure_Ramp_rate_Prev)>1e-10) 
	ControlProcessState1 = 7	!Ramp rate changed
	if (Pressure_Ramp_rate1 < 1e-10) | ( Pressure_Ramp_rate1 > 5e-4)
		Pressure_Ramp_rate1 = Pressure_Ramp_rate_Prev
		ControlProcessState1 = 103
	else
		tmpRamprate = Pressure_Ramp_rate1/60.0
		Pressure_Ramp_rate_Prev = Pressure_Ramp_rate1
		call TrajPlanning
		iSPChanged = 1
		dPressureRamprate(1) = Pressure_Ramp_rate1
		WRITE dPressureRamprate, dPressureRamprate
	END	
RET



on (iType(iAxisNum)=1040)&(ControlProcessState1 > 0)&  (abs(Valve_Scaling_Factor1 - Valve_Scaling_Factor_Prev)>1e-10) 
	if (Valve_Scaling_Factor1 < 0.1 ) | ( Valve_Scaling_Factor1 > 100)
		Valve_Scaling_Factor1 = Valve_Scaling_Factor_Prev
		ControlProcessState1 = 104
	else
		Valve_Scaling_Factor_Prev = Valve_Scaling_Factor1
		dValve_scaling_factor(1) = Valve_Scaling_Factor1
		WRITE dValve_scaling_factor, dValve_scaling_factor
	END	
RET
		
on (iType(iAxisNum)=1040)&(ControlProcessState1 > 0)&  (abs(Pres_min1 - Pres_min_Prev)>1e-12) 
	if (Pres_min1 < 5e-11 ) | ( Pres_min1 >= Pres_max1 )
		Pres_min1 = Pres_min_Prev
		ControlProcessState1 = 105
	else
		Pres_min_Prev = Pres_min1
		dPres_min(1) = Pres_min1
		WRITE dPres_min, dPres_min
	END	
RET
		
on (iType(iAxisNum)=1040)&(ControlProcessState1 > 0)&  (abs(Pres_max1 - Pres_max_Prev)>1e-12) 
	if (Pres_max1 <= Pres_min1 ) | ( Pres_max1 > 5e-5)
		ControlProcessState1 = 106	
		Pres_max1 = Pres_max_Prev
	else
		Pres_max_Prev = Pres_max1
		dPres_max(1) = Pres_max1
		WRITE dPres_max, dPres_max
	END	
RET
		

on (iType(iAxisNum)=1040)&(ControlProcessState1 > 0)&  (abs(SafetyShutDownPressure1 - SafetyShutDownPressure_Prev)>1e-12) 
	if (SafetyShutDownPressure1 < 5e-11 ) | ( SafetyShutDownPressure1 > 1e-4)
		SafetyShutDownPressure1 = SafetyShutDownPressure_Prev
		ControlProcessState1 = 107
	else
		SafetyShutDownPressure_Prev = SafetyShutDownPressure1
		dSafetyShutDownPressure(1) = SafetyShutDownPressure1
		WRITE dSafetyShutDownPressure, dSafetyShutDownPressure
	END	
RET
		
		



!on (iType(iAxisNum)=1040)&(ControlProcessState1 > 0)& (abs(PressureWKSP1 - PressurePV1) > Pressure_Ramp_rate1) 
!	ControlProcessState1 = 8	!Large deviation
!	If (PressureSP1<Pres_min1) | (PressureSP1>Pres_max1)
!		PressureSP1 = PressureSP_Prev
!	else
!		call CHANGERAMPING
!	END	
!RET


on (iType(iAxisNum) = 1040)& (iControlModePrev = 0)& (iControlMode1 = 1)
	iSwitchedToPressControlMode = 1
	iControlModePrev = iControlMode1
ret

on (iType(iAxisNum) = 1040)& (iControlModePrev = 1)& (iControlMode1 = 0)
	kill iAxisNum
	iControlModePrev = iControlMode1
ret

#2
!#/ Controller version = 2.27
!#/ Date = 12/28/2015 10:33 AM
!#/ User remarks = 
! motor cummt and axis homing routine for all types except for robot arms

!Change log
! DATE		ECO#	Reason for Change			Remarks
!9/21/2015	N/A		ini release					ZW
!12/28/15	N/A		tuning homing				ZW
!10/16/17	N/A		turn on CPE before move offset, also link CPE to %home torque
!11/17/2017 Disconnet the AVP CPE link to home torques which cause not homing
Global INT	iLl_Interlock_Cmd	!iWord10Bit1			:= doutputs.,
Global INT	iSt_Interlock_Cmd	!iWord10Bit5			:= doutputs.,
Global INT	iCar_Interlock_Cmd	!iWord10Bit9			:= doutputs.,
Global INT  bAxisInterlocked

GLOBAL real OldSP(8), Datalog(2)(1000)
global int AnalogCARAxis(4)
global real AnologHomeSignalMedian(4)
real SP_Direction                 ! Search direction 
real SP_Settle_Time               ! Settling time at Detent Points [msec]
real SP_Search_Vel                ! Search velocity [user-units/sec]
real SP_Drive                     ! Actuating drive command [% of maximum]
real SP_Max_Search                ! Maximum search distance [user-units]
real SP_Init_Offset               ! Initial commutation offset [elec. degrees]
real PrevAcc, PrevDec, PrevKDec, PrevJerk, PrevVel
real minAnalog, maxAnalog
int iNeedSLL, iNeedSRL
real store_CURI, store_CURV
int iAxisNum
real store_CERRI0, store_CERRV0

iAxisNum = 2	!set the particular axis for the buffer
If ((iType(iAxisNum)= 4008) & (iCar_Interlock_Cmd = 0)) | ((iType(iAxisNum)= 5011) & (iLl_Interlock_Cmd = 0)) | ((iType(iAxisNum)= 5012) & (iSt_Interlock_Cmd = 0)) | ((iInterlockEnabled(iAxisNum)=1)&(bAxisInterlocked=1))
	iControlProcessState(iAxisNum) = 113
	GOTO ENDOFHOMING
end

iControlProcessState(iAxisNum) = 14
CERRI(iAxisNum)=4.39
CERRV(iAxisNum)=4.39
store_CERRI0 = CERRI(iAxisNum)
store_CERRV0 = CERRV(iAxisNum)
MFLAGS(iAxisNum).#HOME = 0			
iNeedSLL=0
iNeedSRL=0

if FMASK(iAxisNum).#SLL = 1
	FMASK(iAxisNum).#SLL = 0
	iNeedSLL=1
end

if FMASK(iAxisNum).#SRL = 1
	FMASK(iAxisNum).#SRL = 0
	iNeedSRL=1
end

if MFLAGS(iAxisNum).9=0                ! not commutation

	SP_Drive = 5.475          
	SP_Settle_Time = 1000    
	SP_Search_Vel  = 6         
	SP_Init_Offset = 0
	SP_Direction =  -1
	SP_Max_Search = 144
	PrevAcc = ACC(iAxisNum)
	PrevDec = DEC(iAxisNum)
	PrevKDec=KDEC(iAxisNum)
	PrevJerk=JERK(iAxisNum)

	!   Output varibale:
	int  SP_Fail_Code                 ! Faiure Code of the startup program

	! Auxiliary variable:
	real SP_Pitch                     ! Magnetic pitch [180 elec.deg. in user units]

	! State Flag
	int  SP_InCommutationStartup      ! Flag indicating commutation startup is in progress


	SP_Pitch=SLCPRD(iAxisNum)/SLCNP(iAxisNum)*EFAC(iAxisNum)


	!******************************************************************************* 
	! INITIALIZE
	FCLEAR(iAxisNum)   
	disable(iAxisNum)                     
	SETCONF(216,(iAxisNum),0)                ! Reset commutation state
	setconf(214,(iAxisNum),SP_Init_Offset)   ! Set initial commutation phase
	SP_Fail_Code=0                        ! Reset failure
	SP_Direction=1                        ! Move in positive direction
	SP_InCommutationStartup=1             ! commutation startup in progress
	!******************************************************************************* 
	! SET MOTION PROFILE FOR COMMUTATION STARTUP PROCESS 
	!
	!   WARNING: The following are the suggested motion parameters for the startup
	!     process. Check that the values are suitable for your application.

	ACC(iAxisNum)=SP_Search_Vel*10.; DEC(iAxisNum)=SP_Search_Vel*10. 
	KDEC(iAxisNum)=SP_Search_Vel*50.; JERK(iAxisNum)=SP_Search_Vel*100.

	!******************************************************************************* 
	! STEP 1 - MOVE TO FIRST DETENT POINT 
	!
	! WARNING: The motor moves to a detent point by jump. 
	!   The jump distance is up to one magnetic pitch in any direction.
	!   The motor jumps to the closest detent point within its motion range.
	!   If necessary modify initial detent point by changing the variable 
	!     SP_Init_Offset between 0-360 electrical degrees.  
	enable(iAxisNum)
	wait(100)
	while (DCOM(iAxisNum)+0.05 < SP_Drive); DCOM(iAxisNum) = DCOM(iAxisNum) + 0.05; end
	DCOM(iAxisNum) = SP_Drive
	wait SP_Settle_Time
	call Limit_Check
	!******************************************************************************* 
	! STEP 2 - MOVE TO SECOND DETENT POINT
	!
	!   The program moves the motor 90 electrical degrees in order to eliminate
	!      a state of unstable equilibrium.

	Move_Detent:
	ptp/rv (iAxisNum), SP_Direction*SP_Pitch/2.,SP_Search_Vel
	till ^AST(iAxisNum).#MOVE; wait SP_Settle_Time
	call Limit_Check
	disable(iAxisNum)

	MFLAGS(iAxisNum).9=1               ! Set commutation state 
	DCOM(iAxisNum) = 0

	! If motor is to be left enabled after startup process delete the following line:

	Finish:
	SP_InCommutationStartup=0              ! commutation startup is finished
	If SP_Fail_Code=0; disp "   Commutation Startup Finished."
	else disp "   Commutation Startup Failed."; disp "   Failure Code = %i",SP_Fail_Code; end

	ACC(iAxisNum)=PrevAcc
	DEC(iAxisNum)=PrevDec
	KDEC(iAxisNum)=PrevKDec
	JERK(iAxisNum)=PrevJerk

end

!at this moment, the axis shall commuted. If it is, then start homing

if MFLAGS(iAxisNum).9=1
	if (iHomingMethod(iAxisNum) = 0)		!0-home to torque; 1-homing to switch; 2-manual set home;
	    if (iType(iAxisNum)>1000)&(iType(iAxisNum)<2000) 	
		!AVP home to torques - always
			call HOME_TO_TORQ_AVP	!different than LP home to torque
		else
			call HOME_TO_TORQ
		end	
	elseif (iHomingMethod(iAxisNum) = 1)&(iType(iAxisNum)<6000)
		if(iType(iAxisNum)>3000)&(iType(iAxisNum)<5000)
			CALL HOME_TO_SWITCH_CAR
		elseif(iType(iAxisNum) = 5008 | iType(iAxisNum) = 5009 | iType(iAxisNum) = 5010 | iType(iAxisNum) = 5018 | iType(iAxisNum) = 5019 | iType(iAxisNum) = 5020 | iType(iAxisNum) = 5028 | iType(iAxisNum) = 5029 | iType(iAxisNum) = 5030)
			call HOME_TO_SWITCH_CARZ
			FDEF(iAxisNum).#LL=1       ! Enable the iAxis left limit default response
			FDEF(iAxisNum).#RL=1       ! Enable the iAxis right limit default response
		else	
			CALL HOME_TO_SWITCH
			FDEF(iAxisNum).#LL=1       ! Enable the iAxis left limit default response
		end
	elseif (iHomingMethod(iAxisNum) = 2)
		CALL SET_CURRENTPOS_HOME
	end
end
iControlProcessState(iAxisNum) = 10
if iNeedSLL = 1
	FMASK(iAxisNum).#SLL = 1
end

if iNeedSRL = 1
	FMASK(iAxisNum).#SRL = 1
end
ENDOFHOMING:
STOP

!******************************************************************************* 
!   The following routine move the motor away from limit switches 

Limit_Check:
if MERR(iAxisNum) & MERR(iAxisNum)<>5010 & MERR(iAxisNum)<>5011; SP_Fail_Code=1; DISABLE(iAxisNum); DCOM(iAxisNum)=0; goto Finish; end
!   if MERR(iAxisNum); SP_Fail_Code=1; DISABLE(iAxisNum); DCOM(iAxisNum)=0; goto Finish; end 
if (FAULT(iAxisNum).#LL)|(FAULT(iAxisNum).#RL)
   if FAULT(iAxisNum).#LL; SP_Direction=1; else SP_Direction=-1; end
   ptp/rv (iAxisNum), SP_Direction*SP_Max_Search, SP_Search_Vel 
   till ((^FAULT(iAxisNum).#LL)&(^FAULT(iAxisNum).#RL))|(^AST(iAxisNum).#MOVE)
   if (FAULT(iAxisNum).#LL)|(FAULT(iAxisNum).#RL); SP_Fail_Code=4; DISABLE(iAxisNum); DCOM(iAxisNum)=0; goto Finish; end 
   kill(iAxisNum);  wait SP_Settle_Time; goto Move_Detent
end
ret

ON ((SP_InCommutationStartup = 1) & ((FAULT(iAxisNum)&0x30f80)>0 | (S_FAULT&0x30000000)>0))
SP_Fail_Code=1; DISABLE(iAxisNum); DCOM(iAxisNum)=0
CALL Finish 
RET



HOME_TO_TORQ:

MFLAGS(iAxisNum).#HOME = 0
FDEF(iAxisNum).#LL=0       ! Disable the iAxis left limit default response
FDEF(iAxisNum).#RL=0       ! Disable the iAxis right limit default response
ENABLE (iAxisNum)          ! Enable the iAxis drive
wait(1000)
JOG/v (iAxisNum), dHomingVel(iAxisNum)*dAxisPitch(iAxisNum)           ! Move to the left  
wait(1000)
KILL iAxisNum
! Reduce the motor current saturations
store_CURI = XCURI(iAxisNum)
store_CURV = XCURV(iAxisNum)
XCURI(iAxisNum) = dHomingTorq(iAxisNum)
XCURV(iAxisNum) = dHomingTorq(iAxisNum)

CERRI(iAxisNum)=0.01*dHomingTorq(iAxisNum)*CERRI(iAxisNum)
CERRV(iAxisNum)=0.01*dHomingTorq(iAxisNum)*CERRV(iAxisNum)

! Turn off the CPE response
FDEF(iAxisNum).#CPE = 0
! Jog until critical position error is triggered
JOG/v (iAxisNum), -dHomingVel(iAxisNum)*dAxisPitch(iAxisNum)           ! Move to the left  
TILL (FAULT(iAxisNum).#CPE = 1)   			! Wait for the critical PE
Kill iAxisNum
! Stop motor, let settle, set FPOS and TPOS to 0.0.  Then turn CPE mask back off
SET FPOS(iAxisNum) = 0.0
PTP/e (iAxisNum), 0.0
REAL deltaI, deltaV
deltaI = store_CURI - dHomingTorq(iAxisNum)
deltaV = store_CURV - dHomingTorq(iAxisNum)
LOOP 100
	XCURI(iAxisNum) = XCURI(iAxisNum) + 0.01*(deltaI)
	XCURV(iAxisNum) = XCURV(iAxisNum) + 0.01*(deltaV)
	SET FPOS(iAxisNum) = 0
	wait 20	!10ms x 100 = 1 sec ramp
END
!In case of rounding error, lastly set XCUR* to stored value - should be nearly identical at this point
XCURI(iAxisNum) = store_CURI
XCURV(iAxisNum) = store_CURV

CERRV(iAxisNum) = store_CERRV0
CERRI(iAxisNum) = store_CERRI0
FDEF(iAxisNum).#CPE = 1

BREAK iAxisNum
SET FPOS(iAxisNum) = 0 -dHomeOffset(iAxisNum)*dAxisPitch(iAxisNum)
PTP/ev (iAxisNum), 0.0, dHomingVel(iAxisNum)*dAxisPitch(iAxisNum)

!After homing, change Sp to 0.0 to prevent follow-on move.
OldSP(iAxisNum)=0
if 		iAxisNum = 0								
	Sp0 = 0.0
elseif 	iAxisNum = 1							
	Sp1 = 0.0
elseif 	iAxisNum = 2						
	Sp2 = 0.0
elseif 	iAxisNum = 3						
	Sp3 = 0.0
elseif 	iAxisNum = 4						
	Sp4 = 0.0
elseif 	iAxisNum = 5						
	Sp5 = 0.0
elseif 	iAxisNum = 6						
	Sp6 = 0.0
elseif 	iAxisNum = 7						
	Sp7 = 0.0
end

FDEF(iAxisNum).#CPE = 1
MFLAGS(iAxisNum).#HOME = 1
wait 500
RET


HOME_TO_SWITCH:

MFLAGS(iAxisNum).#HOME = 0
FDEF(iAxisNum).#LL=0       ! Disable the iAxis left limit default response
FDEF(iAxisNum).#RL=0       ! Disable the iAxis right limit default response
ENABLE (iAxisNum)          ! Enable the iAxis drive
wait(1000)

if  (IN(0).iAxisNum = 1) 
	JOG/v (iAxisNum), dHomingVel(iAxisNum)*dAxisPitch(iAxisNum)           ! Move to the left  
	TILL (IN(0).iAxisNum = 0)   			! Wait for home switch input
	! Stop motor, let settle, set FPOS and TPOS to 0.0.  Then turn CPE mask back off
	KILL (iAxisNum)
end
! Reduce the motor current saturations
store_CURI = XCURI(iAxisNum)
store_CURV = XCURV(iAxisNum)
XCURI(iAxisNum) = dHomingTorq(iAxisNum)
XCURV(iAxisNum) = dHomingTorq(iAxisNum)
! Do NOT Turn off the CPE response for home to switch
!FDEF(iAxisNum).#CPE = 0
! Jog until critical position error is triggered
JOG/v (iAxisNum), -dHomingVel(iAxisNum)*dAxisPitch(iAxisNum)           ! Move to the left  
   			! Wait for home switch input
TILL (IN(0).iAxisNum = 1)
! Stop motor, let settle, set FPOS and TPOS to 0.0.  Then turn CPE mask back off
KILL (iAxisNum)
wait 3000
SET FPOS(iAxisNum) = 0	!@CR2015-12-31 modified home position to simple offset - why move to index?
TPOS(iAxisNum) = 0
! FDEF(iAxisNum).#CPE = 1	!no need to turn turn CPE on because it was not turned off.
! disable motor to return to running torque, and immediately re-enable. This prevents an abrupt move with torque increase.

LOOP 100
	XCURI(iAxisNum) = XCURI(iAxisNum) + 0.01*(store_CURI - dHomingTorq(iAxisNum))
	wait 10	!10ms x 100 = 1 sec ramp
	XCURV(iAxisNum) = XCURV(iAxisNum) + 0.01*(store_CURV - dHomingTorq(iAxisNum))
	wait 10	!10ms x 100 = 1 sec ramp
END
! In case of rounding error, lastly set XCUR* to stored value - should be nearly identical at this point
XCURI(iAxisNum) = store_CURI
XCURV(iAxisNum) = store_CURV

!disable (iAxisNum)
!XCURI(iAxisNum) = store_CURI !@@@ moved here from end of if-thens
!XCURV(iAxisNum) = store_CURV	!@@@ moved here from end of if-thens
!enable (iAxisNum)
wait 1000
PTP/ev (iAxisNum), dHomeOffset(iAxisNum)*dAxisPitch(iAxisNum), dHomingVel(iAxisNum)*dAxisPitch(iAxisNum)
wait 3000
! After moving to home offset, re-zero the axis.  This is now the "working zero" for the axis
SET FPOS(iAxisNum) = 0	!@CR2015-12-31 modified home position to simple offset - why move to index?
TPOS(iAxisNum) = 0
! perform another move to zero (because without, there is residual FPOS data that shows at GUI - looks funny
PTP/ev (iAxisNum), 0.0, dHomingVel(iAxisNum)*dAxisPitch(iAxisNum)
wait 2000

!After homing, change Sp to 0.0 to prevent follow-on move.
OldSP(iAxisNum)=0
if 		iAxisNum = 0								
	Sp0 = 0.0
elseif 	iAxisNum = 1							
	Sp1 = 0.0
elseif 	iAxisNum = 2						
	Sp2 = 0.0
elseif 	iAxisNum = 3						
	Sp3 = 0.0
elseif 	iAxisNum = 4						
	Sp4 = 0.0
elseif 	iAxisNum = 5						
	Sp5 = 0.0
elseif 	iAxisNum = 6						
	Sp6 = 0.0
elseif 	iAxisNum = 7						
	Sp7 = 0.0
end

FDEF(iAxisNum).#CPE = 1
MFLAGS(iAxisNum).#HOME = 1
wait 500
RET







	
HOME_TO_SWITCH_CARZ:

MFLAGS(iAxisNum).#HOME = 0
FDEF(iAxisNum).#LL=0       ! Disable the iAxis left limit default response
FDEF(iAxisNum).#RL=0       ! Disable the iAxis right limit default response
ENABLE (iAxisNum)          ! Enable the iAxis drive
wait(1000)

if  (SAFIN(iAxisNum).#LL = 1) 
	JOG/v (iAxisNum), dHomingVel(iAxisNum)*dAxisPitch(iAxisNum)           ! Move to the left  
	TILL (SAFIN(iAxisNum).#LL  = 0)   			! Wait for home switch input
	! Stop motor, let settle, set FPOS and TPOS to 0.0.  Then turn CPE mask back off
	KILL (iAxisNum)
end
! Reduce the motor current saturations
store_CURI = XCURI(iAxisNum)
store_CURV = XCURV(iAxisNum)
XCURI(iAxisNum) = dHomingTorq(iAxisNum)
XCURV(iAxisNum) = dHomingTorq(iAxisNum)
! Do NOT Turn off the CPE response for home to switch
!FDEF(iAxisNum).#CPE = 0
! Jog until critical position error is triggered
JOG/v (iAxisNum), -dHomingVel(iAxisNum)*dAxisPitch(iAxisNum)           ! Move to the left  
   			! Wait for home switch input
TILL (SAFIN(iAxisNum).#LL = 1)
! Stop motor, let settle, set FPOS and TPOS to 0.0.  Then turn CPE mask back off
KILL (iAxisNum)
wait 3000

! @JR 2021-10-18 - Move down until upper limit switch opens
JOG/v (iAxisNum), dHomingVel(iAxisNum)*dAxisPitch(iAxisNum)
TILL (SAFIN(iAxisNum).#LL = 0)
KILL (iAxisNum)
wait 3000

SET FPOS(iAxisNum) = 0	!@CR2015-12-31 modified home position to simple offset - why move to index?
TPOS(iAxisNum) = 0
! FDEF(iAxisNum).#CPE = 1	!no need to turn turn CPE on because it was not turned off.
! disable motor to return to running torque, and immediately re-enable. This prevents an abrupt move with torque increase.

!LOOP 100
!	XCURI(iAxisNum) = XCURI(iAxisNum) + 0.01*(store_CURI - dHomingTorq(iAxisNum))
!	wait 10	!10ms x 100 = 1 sec ramp
!	XCURV(iAxisNum) = XCURV(iAxisNum) + 0.01*(store_CURV - dHomingTorq(iAxisNum))
!	wait 10	!10ms x 100 = 1 sec ramp
!END
! In case of rounding error, lastly set XCUR* to stored value - should be nearly identical at this point
XCURI(iAxisNum) = store_CURI
XCURV(iAxisNum) = store_CURV

!disable (iAxisNum)
!XCURI(iAxisNum) = store_CURI !@@@ moved here from end of if-thens
!XCURV(iAxisNum) = store_CURV	!@@@ moved here from end of if-thens
!enable (iAxisNum)
wait 1000
PTP/ev (iAxisNum), dHomeOffset(iAxisNum)*dAxisPitch(iAxisNum), dHomingVel(iAxisNum)*dAxisPitch(iAxisNum)
wait 3000
! After moving to home offset, re-zero the axis.  This is now the "working zero" for the axis
SET FPOS(iAxisNum) = 0	!@CR2015-12-31 modified home position to simple offset - why move to index?
TPOS(iAxisNum) = 0
! perform another move to zero (because without, there is residual FPOS data that shows at GUI - looks funny
PTP/ev (iAxisNum), 0.0, dHomingVel(iAxisNum)*dAxisPitch(iAxisNum)
wait 2000

!After homing, change Sp to 0.0 to prevent follow-on move.
OldSP(iAxisNum)=0
if 		iAxisNum = 0								
	Sp0 = 0.0
elseif 	iAxisNum = 1							
	Sp1 = 0.0
elseif 	iAxisNum = 2						
	Sp2 = 0.0
elseif 	iAxisNum = 3						
	Sp3 = 0.0
elseif 	iAxisNum = 4						
	Sp4 = 0.0
elseif 	iAxisNum = 5						
	Sp5 = 0.0
elseif 	iAxisNum = 6						
	Sp6 = 0.0
elseif 	iAxisNum = 7						
	Sp7 = 0.0
end

FDEF(iAxisNum).#CPE = 1
MFLAGS(iAxisNum).#HOME = 1
wait 500
RET

	
	
	
	
	

SET_CURRENTPOS_HOME:

SET FPOS(iAxisNum) = 0
TPOS(iAxisNum) = 0	!@@@ added
FDEF(iAxisNum).#CPE = 1
MFLAGS(iAxisNum).#HOME = 1
ENABLE (iAxisNum) 
! Move to 0 to clean-up residual FPOS error on GUI
PTP/ev (iAxisNum), 0, 180
wait 2000
!After homing, change Sp to 0.0 to prevent follow-on move.
OldSP(iAxisNum)=0
if 		iAxisNum = 0								
	Sp0 = 0.0
elseif 	iAxisNum = 1							
	Sp1 = 0.0
elseif 	iAxisNum = 2						
	Sp2 = 0.0
elseif 	iAxisNum = 3						
	Sp3 = 0.0
elseif 	iAxisNum = 4						
	Sp4 = 0.0
elseif 	iAxisNum = 5						
	Sp5 = 0.0
elseif 	iAxisNum = 6						
	Sp6 = 0.0
elseif 	iAxisNum = 7						
	Sp7 = 0.0
end
RET


HOME_TO_TORQ_AVP:


MFLAGS(iAxisNum).#HOME = 0
FDEF(iAxisNum).#LL=0       ! Disable the iAxis left limit default response
FDEF(iAxisNum).#RL=0       ! Disable the iAxis right limit default response
ENABLE (iAxisNum)          ! Enable the iAxis drive
wait(100)
JOG/v (iAxisNum), dHomingVel(iAxisNum)*dAxisPitch(iAxisNum)           ! Move to the left  
wait(1000)
KILL iAxisNum
!reduce the motor current saturations
store_CURI = XCURI(iAxisNum)
store_CURV = XCURV(iAxisNum)

real XCURI_actural, HmCurrent_Of_Drive, FtCurrent_Of_Drive
HmCurrent_Of_Drive = dHomingTorq(iAxisNum)*(4*XRMS(iAxisNum))/100.0
FtCurrent_Of_Drive = dFaultTorq(iAxisNum)*(4*XRMS(iAxisNum))/100.0

XCURI(iAxisNum) = HmCurrent_Of_Drive
XCURV(iAxisNum) = HmCurrent_Of_Drive
!CERRI(iAxisNum)=0.01*dHomingTorq(iAxisNum)*CERRI(iAxisNum)
!CERRV(iAxisNum)=0.01*dHomingTorq(iAxisNum)*CERRV(iAxisNum)
! Turn off the CPE response
FDEF(iAxisNum).#CPE = 0

JOG/v (iAxisNum), -dHomingVel(iAxisNum)*dAxisPitch(iAxisNum)           ! Move to the left  
TILL (FAULT(iAxisNum).#CPE = 1)   			! Wait for the critical PE
KILL iAxisNum

if (dAxisPitch(iAxisNum)>0.01)
	dCE(iAxisNum)=FPOS(iAxisNum)/dAxisPitch(iAxisNum)+dHomeOffset(iAxisNum)
end
block
SET FPOS(iAxisNum) = 0.0 -dHomeOffset(iAxisNum)*dAxisPitch(iAxisNum)
PTP/e (iAxisNum), 100!0.0
end



deltaI = FtCurrent_Of_Drive - HmCurrent_Of_Drive
deltaV = FtCurrent_Of_Drive - HmCurrent_Of_Drive
LOOP 100
	XCURI(iAxisNum) = XCURI(iAxisNum) + 0.01*(deltaI)
	XCURV(iAxisNum) = XCURV(iAxisNum) + 0.01*(deltaV)
	wait 20	!10ms x 100 = 1 sec ramp
END

XCURI(iAxisNum) = FtCurrent_Of_Drive !@@@ moved here from end of if-thens
XCURV(iAxisNum) = FtCurrent_Of_Drive	!@@@ moved here from end of if-thens
CERRV(iAxisNum) = store_CERRV0
CERRI(iAxisNum) = store_CERRI0
FDEF(iAxisNum).#CPE = 1

BREAK iAxisNum
!SET FPOS(iAxisNum) = 0.0 -dHomeOffset(iAxisNum)*dAxisPitch(iAxisNum)	!@CR2015-12-31 modified home position to simple offset - why move to index?
PTP/ev (iAxisNum), 0.0, dHomingVel(iAxisNum)*dAxisPitch(iAxisNum)


!After homing, change Sp to 0.0 to prevent follow-on move.
OldSP(iAxisNum)=0
if 		iAxisNum = 0								
	Sp0 = 0.0
elseif 	iAxisNum = 1							
	Sp1 = 0.0
elseif 	iAxisNum = 2						
	Sp2 = 0.0
elseif 	iAxisNum = 3						
	Sp3 = 0.0
elseif 	iAxisNum = 4						
	Sp4 = 0.0
elseif 	iAxisNum = 5						
	Sp5 = 0.0
elseif 	iAxisNum = 6						
	Sp6 = 0.0
elseif 	iAxisNum = 7						
	Sp7 = 0.0
end
FDEF(iAxisNum).#CPE = 1
MFLAGS(iAxisNum).#HOME = 1
RET

HOME_TO_SWITCH_CAR:
REAL rMax, rMin
INT iCARAIChannel
iCARAIChannel = 0

MFLAGS(iAxisNum).#HOME = 0
FDEF(iAxisNum).#LL=0       ! Disable the iAxis left limit default response
FDEF(iAxisNum).#RL=0       ! Disable the iAxis right limit default response
ENABLE (iAxisNum)          ! Enable the iAxis drive
wait(1000)
! Jog until critical position error is triggered
if (FtdCarHomeSW.iAxisNum = 1)
	!JOG/v (iAxisNum), dHomingVel(iAxisNum)           ! Move to the left  
	!TILL (FtdCarHomeSW(iAxisNum) = 0)   			! Wait for home switch input
	!BREAK (iAxisNum)
	PTP/er (iAxisNum), 15
end

if(iType(iAxisNum)=4007)	! if it is an analog sensor, perform sensor calibration first.
! find which analog channel is the home sensor at

	if AnalogCARAxis(0)=iAxisNum
		iCARAIChannel = 0
	elseif AnalogCARAxis(1)=iAxisNum
		iCARAIChannel = 1
	elseif AnalogCARAxis(2)=iAxisNum
		iCARAIChannel = 2
	elseif AnalogCARAxis(3)=iAxisNum
		iCARAIChannel = 3
	end
! scan the analog signal and acq 1000 points from the analog signal while spinning
	DC/S iAxisNum,Datalog,1000,100,FPOS(iAxisNum),AIN(iCARAIChannel)
	JOG/v (iAxisNum), 4 
	till (^AST(iAxisNum).#DC)	!wait DAQ to finish
	KILL iAxisNum
	WAIT 2000
! find the median in analog signal	
	rMax = MAX(Datalog, 1, 1, 0,999)
	rMin = MIN(Datalog, 1, 1, 0,999)
	if (rMax-rMin)>1.0
		AnologHomeSignalMedian(iCARAIChannel) = (rMax + rMin)/2.0
	else
		disable(iAxisNum)  
		iControlProcessState(iAxisNum) = 404
		RET
	end	
! find the position where is the peak signal
	int thepeakposIndex
	thepeakposIndex = MAXI(Datalog, 1, 1, 0,999)
! move the CAR to the neighbourhood of the peak	
	PTP/e (iAxisNum), Datalog(0)(thepeakposIndex)+15
	WAIT 3000
end





real FirstEdge, SecondEdge
!Find the leading edge of the home sensor
JOG/v (iAxisNum), -dHomingVel(iAxisNum)           ! Move to the left  
TILL (FtdCarHomeSW.iAxisNum = 1)   			! Wait for home switch input
FirstEdge = FPOS(iAxisNum)
KILL iAxisNum
WAIT 2000
!back off
PTP/e (iAxisNum), FirstEdge + 10
!low speed homing
JOG/v (iAxisNum), -dHomingVel(iAxisNum)           ! Move to the left  
TILL (FtdCarHomeSW.iAxisNum = 1)   			! Wait for home switch input
FirstEdge = FPOS(iAxisNum)
KILL iAxisNum
WAIT 2000
PTP/e (iAxisNum), FirstEdge - 15
!low speed homing
JOG/v (iAxisNum), dHomingVel(iAxisNum)           ! Move to the left  
TILL (FtdCarHomeSW.iAxisNum = 1) 
SecondEdge = FPOS(iAxisNum)
KILL iAxisNum
WAIT 2000
PTP/ev (iAxisNum), 0.5*(FirstEdge+SecondEdge)+dHomeOffset(iAxisNum), dHomingVel(iAxisNum)
wait 3000

SET FPOS(iAxisNum) = 0	!@CR2015-12-31 modified home position to simple offset - why move to index?
TPOS(iAxisNum) = 0
! After moving to home offset, re-zero the axis.  This is now the "working zero" for the axis
SET FPOS(iAxisNum) = 0	!@CR2015-12-31 modified home position to simple offset - why move to index?
TPOS(iAxisNum) = 0
! perform another move to zero (because without, there is residual FPOS data that shows at GUI - looks funny
FDEF(iAxisNum).#CPE = 1
MFLAGS(iAxisNum).#HOME = 1
wait 500

PTP/ev (iAxisNum), 0.0, dHomingVel(iAxisNum)
wait 2000

!After homing, change Sp to 0.0 to prevent follow-on move.
OldSP(iAxisNum)=0
if 		iAxisNum = 0								
	Sp0 = 0.0
elseif 	iAxisNum = 1							
	Sp1 = 0.0
elseif 	iAxisNum = 2						
	Sp2 = 0.0
elseif 	iAxisNum = 3						
	Sp3 = 0.0
elseif 	iAxisNum = 4						
	Sp4 = 0.0
elseif 	iAxisNum = 5						
	Sp5 = 0.0
elseif 	iAxisNum = 6						
	Sp6 = 0.0
elseif 	iAxisNum = 7						
	Sp7 = 0.0
end

RET
#3
!#/ Controller version = 2.27
!#/ Date = 12/28/2015 10:33 AM
!#/ User remarks = 
! motor cummt and axis homing routine for all types except for robot arms

!Change log
! DATE		ECO#	Reason for Change			Remarks
!9/21/2015	N/A		ini release					ZW
!12/28/15	N/A		tuning homing				ZW
!10/16/17	N/A		turn on CPE before move offset, also link CPE to %home torque
!11/17/2017 Disconnet the AVP CPE link to home torques which cause not homing
Global INT	iLl_Interlock_Cmd	!iWord10Bit1			:= doutputs.,
Global INT	iSt_Interlock_Cmd	!iWord10Bit5			:= doutputs.,
Global INT	iCar_Interlock_Cmd	!iWord10Bit9			:= doutputs.,
Global INT  bAxisInterlocked

GLOBAL real OldSP(8), Datalog(2)(1000)
global int AnalogCARAxis(4)
global real AnologHomeSignalMedian(4)
real SP_Direction                 ! Search direction 
real SP_Settle_Time               ! Settling time at Detent Points [msec]
real SP_Search_Vel                ! Search velocity [user-units/sec]
real SP_Drive                     ! Actuating drive command [% of maximum]
real SP_Max_Search                ! Maximum search distance [user-units]
real SP_Init_Offset               ! Initial commutation offset [elec. degrees]
real PrevAcc, PrevDec, PrevKDec, PrevJerk, PrevVel
real minAnalog, maxAnalog
int iNeedSLL, iNeedSRL
real store_CURI, store_CURV
int iAxisNum
real store_CERRI0, store_CERRV0

iAxisNum = 3	!set the particular axis for the buffer
If ((iType(iAxisNum)= 4008) & (iCar_Interlock_Cmd = 0)) | ((iType(iAxisNum)= 5011) & (iLl_Interlock_Cmd = 0)) | ((iType(iAxisNum)= 5012) & (iSt_Interlock_Cmd = 0)) | ((iInterlockEnabled(iAxisNum)=1)&(bAxisInterlocked=1))
	iControlProcessState(iAxisNum) = 113
	GOTO ENDOFHOMING
end

iControlProcessState(iAxisNum) = 14
CERRI(iAxisNum)=4.39
CERRV(iAxisNum)=4.39
store_CERRI0 = CERRI(iAxisNum)
store_CERRV0 = CERRV(iAxisNum)
MFLAGS(iAxisNum).#HOME = 0			
iNeedSLL=0
iNeedSRL=0

if FMASK(iAxisNum).#SLL = 1
	FMASK(iAxisNum).#SLL = 0
	iNeedSLL=1
end

if FMASK(iAxisNum).#SRL = 1
	FMASK(iAxisNum).#SRL = 0
	iNeedSRL=1
end

if MFLAGS(iAxisNum).9=0                ! not commutation

	SP_Drive = 5.475          
	SP_Settle_Time = 1000    
	SP_Search_Vel  = 6         
	SP_Init_Offset = 0
	SP_Direction =  -1
	SP_Max_Search = 144
	PrevAcc = ACC(iAxisNum)
	PrevDec = DEC(iAxisNum)
	PrevKDec=KDEC(iAxisNum)
	PrevJerk=JERK(iAxisNum)

	!   Output varibale:
	int  SP_Fail_Code                 ! Faiure Code of the startup program

	! Auxiliary variable:
	real SP_Pitch                     ! Magnetic pitch [180 elec.deg. in user units]

	! State Flag
	int  SP_InCommutationStartup      ! Flag indicating commutation startup is in progress


	SP_Pitch=SLCPRD(iAxisNum)/SLCNP(iAxisNum)*EFAC(iAxisNum)


	!******************************************************************************* 
	! INITIALIZE
	FCLEAR(iAxisNum)   
	disable(iAxisNum)                     
	SETCONF(216,(iAxisNum),0)                ! Reset commutation state
	setconf(214,(iAxisNum),SP_Init_Offset)   ! Set initial commutation phase
	SP_Fail_Code=0                        ! Reset failure
	SP_Direction=1                        ! Move in positive direction
	SP_InCommutationStartup=1             ! commutation startup in progress
	!******************************************************************************* 
	! SET MOTION PROFILE FOR COMMUTATION STARTUP PROCESS 
	!
	!   WARNING: The following are the suggested motion parameters for the startup
	!     process. Check that the values are suitable for your application.

	ACC(iAxisNum)=SP_Search_Vel*10.; DEC(iAxisNum)=SP_Search_Vel*10. 
	KDEC(iAxisNum)=SP_Search_Vel*50.; JERK(iAxisNum)=SP_Search_Vel*100.

	!******************************************************************************* 
	! STEP 1 - MOVE TO FIRST DETENT POINT 
	!
	! WARNING: The motor moves to a detent point by jump. 
	!   The jump distance is up to one magnetic pitch in any direction.
	!   The motor jumps to the closest detent point within its motion range.
	!   If necessary modify initial detent point by changing the variable 
	!     SP_Init_Offset between 0-360 electrical degrees.  
	enable(iAxisNum)
	wait(100)
	while (DCOM(iAxisNum)+0.05 < SP_Drive); DCOM(iAxisNum) = DCOM(iAxisNum) + 0.05; end
	DCOM(iAxisNum) = SP_Drive
	wait SP_Settle_Time
	call Limit_Check
	!******************************************************************************* 
	! STEP 2 - MOVE TO SECOND DETENT POINT
	!
	!   The program moves the motor 90 electrical degrees in order to eliminate
	!      a state of unstable equilibrium.

	Move_Detent:
	ptp/rv (iAxisNum), SP_Direction*SP_Pitch/2.,SP_Search_Vel
	till ^AST(iAxisNum).#MOVE; wait SP_Settle_Time
	call Limit_Check
	disable(iAxisNum)

	MFLAGS(iAxisNum).9=1               ! Set commutation state 
	DCOM(iAxisNum) = 0

	! If motor is to be left enabled after startup process delete the following line:

	Finish:
	SP_InCommutationStartup=0              ! commutation startup is finished
	If SP_Fail_Code=0; disp "   Commutation Startup Finished."
	else disp "   Commutation Startup Failed."; disp "   Failure Code = %i",SP_Fail_Code; end

	ACC(iAxisNum)=PrevAcc
	DEC(iAxisNum)=PrevDec
	KDEC(iAxisNum)=PrevKDec
	JERK(iAxisNum)=PrevJerk

end

!at this moment, the axis shall commuted. If it is, then start homing

if MFLAGS(iAxisNum).9=1
	if (iHomingMethod(iAxisNum) = 0)		!0-home to torque; 1-homing to switch; 2-manual set home;
	    if (iType(iAxisNum)>1000)&(iType(iAxisNum)<2000) 	
		!AVP home to torques - always
			call HOME_TO_TORQ_AVP	!different than LP home to torque
		else
			call HOME_TO_TORQ
		end	
	elseif (iHomingMethod(iAxisNum) = 1)&(iType(iAxisNum)<6000)
		if(iType(iAxisNum)>3000)&(iType(iAxisNum)<5000)
			CALL HOME_TO_SWITCH_CAR
		elseif(iType(iAxisNum) = 5008 | iType(iAxisNum) = 5009 | iType(iAxisNum) = 5010 | iType(iAxisNum) = 5018 | iType(iAxisNum) = 5019 | iType(iAxisNum) = 5020 | iType(iAxisNum) = 5028 | iType(iAxisNum) = 5029 | iType(iAxisNum) = 5030)
			call HOME_TO_SWITCH_CARZ
			FDEF(iAxisNum).#LL=1       ! Enable the iAxis left limit default response
			FDEF(iAxisNum).#RL=1       ! Enable the iAxis right limit default response
		else	
			CALL HOME_TO_SWITCH
			FDEF(iAxisNum).#LL=1       ! Enable the iAxis left limit default response
		end
	elseif (iHomingMethod(iAxisNum) = 2)
		CALL SET_CURRENTPOS_HOME
	end
end
iControlProcessState(iAxisNum) = 10
if iNeedSLL = 1
	FMASK(iAxisNum).#SLL = 1
end

if iNeedSRL = 1
	FMASK(iAxisNum).#SRL = 1
end
ENDOFHOMING:
STOP

!******************************************************************************* 
!   The following routine move the motor away from limit switches 

Limit_Check:
if MERR(iAxisNum) & MERR(iAxisNum)<>5010 & MERR(iAxisNum)<>5011; SP_Fail_Code=1; DISABLE(iAxisNum); DCOM(iAxisNum)=0; goto Finish; end
!   if MERR(iAxisNum); SP_Fail_Code=1; DISABLE(iAxisNum); DCOM(iAxisNum)=0; goto Finish; end 
if (FAULT(iAxisNum).#LL)|(FAULT(iAxisNum).#RL)
   if FAULT(iAxisNum).#LL; SP_Direction=1; else SP_Direction=-1; end
   ptp/rv (iAxisNum), SP_Direction*SP_Max_Search, SP_Search_Vel 
   till ((^FAULT(iAxisNum).#LL)&(^FAULT(iAxisNum).#RL))|(^AST(iAxisNum).#MOVE)
   if (FAULT(iAxisNum).#LL)|(FAULT(iAxisNum).#RL); SP_Fail_Code=4; DISABLE(iAxisNum); DCOM(iAxisNum)=0; goto Finish; end 
   kill(iAxisNum);  wait SP_Settle_Time; goto Move_Detent
end
ret

ON ((SP_InCommutationStartup = 1) & ((FAULT(iAxisNum)&0x30f80)>0 | (S_FAULT&0x30000000)>0))
SP_Fail_Code=1; DISABLE(iAxisNum); DCOM(iAxisNum)=0
CALL Finish 
RET



HOME_TO_TORQ:

MFLAGS(iAxisNum).#HOME = 0
FDEF(iAxisNum).#LL=0       ! Disable the iAxis left limit default response
FDEF(iAxisNum).#RL=0       ! Disable the iAxis right limit default response
ENABLE (iAxisNum)          ! Enable the iAxis drive
wait(1000)
JOG/v (iAxisNum), dHomingVel(iAxisNum)*dAxisPitch(iAxisNum)           ! Move to the left  
wait(1000)
KILL iAxisNum
! Reduce the motor current saturations
store_CURI = XCURI(iAxisNum)
store_CURV = XCURV(iAxisNum)
XCURI(iAxisNum) = dHomingTorq(iAxisNum)
XCURV(iAxisNum) = dHomingTorq(iAxisNum)

CERRI(iAxisNum)=0.01*dHomingTorq(iAxisNum)*CERRI(iAxisNum)
CERRV(iAxisNum)=0.01*dHomingTorq(iAxisNum)*CERRV(iAxisNum)

! Turn off the CPE response
FDEF(iAxisNum).#CPE = 0
! Jog until critical position error is triggered
JOG/v (iAxisNum), -dHomingVel(iAxisNum)*dAxisPitch(iAxisNum)           ! Move to the left  
TILL (FAULT(iAxisNum).#CPE = 1)   			! Wait for the critical PE
Kill iAxisNum
! Stop motor, let settle, set FPOS and TPOS to 0.0.  Then turn CPE mask back off
SET FPOS(iAxisNum) = 0.0
PTP/e (iAxisNum), 0.0
REAL deltaI, deltaV
deltaI = store_CURI - dHomingTorq(iAxisNum)
deltaV = store_CURV - dHomingTorq(iAxisNum)
LOOP 100
	XCURI(iAxisNum) = XCURI(iAxisNum) + 0.01*(deltaI)
	XCURV(iAxisNum) = XCURV(iAxisNum) + 0.01*(deltaV)
	SET FPOS(iAxisNum) = 0
	wait 20	!10ms x 100 = 1 sec ramp
END
!In case of rounding error, lastly set XCUR* to stored value - should be nearly identical at this point
XCURI(iAxisNum) = store_CURI
XCURV(iAxisNum) = store_CURV

CERRV(iAxisNum) = store_CERRV0
CERRI(iAxisNum) = store_CERRI0
FDEF(iAxisNum).#CPE = 1

BREAK iAxisNum
SET FPOS(iAxisNum) = 0 -dHomeOffset(iAxisNum)*dAxisPitch(iAxisNum)
PTP/ev (iAxisNum), 0.0, dHomingVel(iAxisNum)*dAxisPitch(iAxisNum)

!After homing, change Sp to 0.0 to prevent follow-on move.
OldSP(iAxisNum)=0
if 		iAxisNum = 0								
	Sp0 = 0.0
elseif 	iAxisNum = 1							
	Sp1 = 0.0
elseif 	iAxisNum = 2						
	Sp2 = 0.0
elseif 	iAxisNum = 3						
	Sp3 = 0.0
elseif 	iAxisNum = 4						
	Sp4 = 0.0
elseif 	iAxisNum = 5						
	Sp5 = 0.0
elseif 	iAxisNum = 6						
	Sp6 = 0.0
elseif 	iAxisNum = 7						
	Sp7 = 0.0
end

FDEF(iAxisNum).#CPE = 1
MFLAGS(iAxisNum).#HOME = 1
wait 500
RET


HOME_TO_SWITCH:

MFLAGS(iAxisNum).#HOME = 0
FDEF(iAxisNum).#LL=0       ! Disable the iAxis left limit default response
FDEF(iAxisNum).#RL=0       ! Disable the iAxis right limit default response
ENABLE (iAxisNum)          ! Enable the iAxis drive
wait(1000)

if  (IN(0).iAxisNum = 1) 
	JOG/v (iAxisNum), dHomingVel(iAxisNum)*dAxisPitch(iAxisNum)           ! Move to the left  
	TILL (IN(0).iAxisNum = 0)   			! Wait for home switch input
	! Stop motor, let settle, set FPOS and TPOS to 0.0.  Then turn CPE mask back off
	KILL (iAxisNum)
end
! Reduce the motor current saturations
store_CURI = XCURI(iAxisNum)
store_CURV = XCURV(iAxisNum)
XCURI(iAxisNum) = dHomingTorq(iAxisNum)
XCURV(iAxisNum) = dHomingTorq(iAxisNum)
! Do NOT Turn off the CPE response for home to switch
!FDEF(iAxisNum).#CPE = 0
! Jog until critical position error is triggered
JOG/v (iAxisNum), -dHomingVel(iAxisNum)*dAxisPitch(iAxisNum)           ! Move to the left  
   			! Wait for home switch input
TILL (IN(0).iAxisNum = 1)
! Stop motor, let settle, set FPOS and TPOS to 0.0.  Then turn CPE mask back off
KILL (iAxisNum)
wait 3000
SET FPOS(iAxisNum) = 0	!@CR2015-12-31 modified home position to simple offset - why move to index?
TPOS(iAxisNum) = 0
! FDEF(iAxisNum).#CPE = 1	!no need to turn turn CPE on because it was not turned off.
! disable motor to return to running torque, and immediately re-enable. This prevents an abrupt move with torque increase.

LOOP 100
	XCURI(iAxisNum) = XCURI(iAxisNum) + 0.01*(store_CURI - dHomingTorq(iAxisNum))
	wait 10	!10ms x 100 = 1 sec ramp
	XCURV(iAxisNum) = XCURV(iAxisNum) + 0.01*(store_CURV - dHomingTorq(iAxisNum))
	wait 10	!10ms x 100 = 1 sec ramp
END
! In case of rounding error, lastly set XCUR* to stored value - should be nearly identical at this point
XCURI(iAxisNum) = store_CURI
XCURV(iAxisNum) = store_CURV

!disable (iAxisNum)
!XCURI(iAxisNum) = store_CURI !@@@ moved here from end of if-thens
!XCURV(iAxisNum) = store_CURV	!@@@ moved here from end of if-thens
!enable (iAxisNum)
wait 1000
PTP/ev (iAxisNum), dHomeOffset(iAxisNum)*dAxisPitch(iAxisNum), dHomingVel(iAxisNum)*dAxisPitch(iAxisNum)
wait 3000
! After moving to home offset, re-zero the axis.  This is now the "working zero" for the axis
SET FPOS(iAxisNum) = 0	!@CR2015-12-31 modified home position to simple offset - why move to index?
TPOS(iAxisNum) = 0
! perform another move to zero (because without, there is residual FPOS data that shows at GUI - looks funny
PTP/ev (iAxisNum), 0.0, dHomingVel(iAxisNum)*dAxisPitch(iAxisNum)
wait 2000

!After homing, change Sp to 0.0 to prevent follow-on move.
OldSP(iAxisNum)=0
if 		iAxisNum = 0								
	Sp0 = 0.0
elseif 	iAxisNum = 1							
	Sp1 = 0.0
elseif 	iAxisNum = 2						
	Sp2 = 0.0
elseif 	iAxisNum = 3						
	Sp3 = 0.0
elseif 	iAxisNum = 4						
	Sp4 = 0.0
elseif 	iAxisNum = 5						
	Sp5 = 0.0
elseif 	iAxisNum = 6						
	Sp6 = 0.0
elseif 	iAxisNum = 7						
	Sp7 = 0.0
end

FDEF(iAxisNum).#CPE = 1
MFLAGS(iAxisNum).#HOME = 1
wait 500
RET







	
HOME_TO_SWITCH_CARZ:

MFLAGS(iAxisNum).#HOME = 0
FDEF(iAxisNum).#LL=0       ! Disable the iAxis left limit default response
FDEF(iAxisNum).#RL=0       ! Disable the iAxis right limit default response
ENABLE (iAxisNum)          ! Enable the iAxis drive
wait(1000)

if  (SAFIN(iAxisNum).#LL = 1) 
	JOG/v (iAxisNum), dHomingVel(iAxisNum)*dAxisPitch(iAxisNum)           ! Move to the left  
	TILL (SAFIN(iAxisNum).#LL  = 0)   			! Wait for home switch input
	! Stop motor, let settle, set FPOS and TPOS to 0.0.  Then turn CPE mask back off
	KILL (iAxisNum)
end
! Reduce the motor current saturations
store_CURI = XCURI(iAxisNum)
store_CURV = XCURV(iAxisNum)
XCURI(iAxisNum) = dHomingTorq(iAxisNum)
XCURV(iAxisNum) = dHomingTorq(iAxisNum)
! Do NOT Turn off the CPE response for home to switch
!FDEF(iAxisNum).#CPE = 0
! Jog until critical position error is triggered
JOG/v (iAxisNum), -dHomingVel(iAxisNum)*dAxisPitch(iAxisNum)           ! Move to the left  
   			! Wait for home switch input
TILL (SAFIN(iAxisNum).#LL = 1)
! Stop motor, let settle, set FPOS and TPOS to 0.0.  Then turn CPE mask back off
KILL (iAxisNum)
wait 3000

! @JR 2021-10-18 - Move down until upper limit switch opens
JOG/v (iAxisNum), dHomingVel(iAxisNum)*dAxisPitch(iAxisNum)
TILL (SAFIN(iAxisNum).#LL = 0)
KILL (iAxisNum)
wait 3000

SET FPOS(iAxisNum) = 0	!@CR2015-12-31 modified home position to simple offset - why move to index?
TPOS(iAxisNum) = 0
! FDEF(iAxisNum).#CPE = 1	!no need to turn turn CPE on because it was not turned off.
! disable motor to return to running torque, and immediately re-enable. This prevents an abrupt move with torque increase.

!LOOP 100
!	XCURI(iAxisNum) = XCURI(iAxisNum) + 0.01*(store_CURI - dHomingTorq(iAxisNum))
!	wait 10	!10ms x 100 = 1 sec ramp
!	XCURV(iAxisNum) = XCURV(iAxisNum) + 0.01*(store_CURV - dHomingTorq(iAxisNum))
!	wait 10	!10ms x 100 = 1 sec ramp
!END
! In case of rounding error, lastly set XCUR* to stored value - should be nearly identical at this point
XCURI(iAxisNum) = store_CURI
XCURV(iAxisNum) = store_CURV

!disable (iAxisNum)
!XCURI(iAxisNum) = store_CURI !@@@ moved here from end of if-thens
!XCURV(iAxisNum) = store_CURV	!@@@ moved here from end of if-thens
!enable (iAxisNum)
wait 1000
PTP/ev (iAxisNum), dHomeOffset(iAxisNum)*dAxisPitch(iAxisNum), dHomingVel(iAxisNum)*dAxisPitch(iAxisNum)
wait 3000
! After moving to home offset, re-zero the axis.  This is now the "working zero" for the axis
SET FPOS(iAxisNum) = 0	!@CR2015-12-31 modified home position to simple offset - why move to index?
TPOS(iAxisNum) = 0
! perform another move to zero (because without, there is residual FPOS data that shows at GUI - looks funny
PTP/ev (iAxisNum), 0.0, dHomingVel(iAxisNum)*dAxisPitch(iAxisNum)
wait 2000

!After homing, change Sp to 0.0 to prevent follow-on move.
OldSP(iAxisNum)=0
if 		iAxisNum = 0								
	Sp0 = 0.0
elseif 	iAxisNum = 1							
	Sp1 = 0.0
elseif 	iAxisNum = 2						
	Sp2 = 0.0
elseif 	iAxisNum = 3						
	Sp3 = 0.0
elseif 	iAxisNum = 4						
	Sp4 = 0.0
elseif 	iAxisNum = 5						
	Sp5 = 0.0
elseif 	iAxisNum = 6						
	Sp6 = 0.0
elseif 	iAxisNum = 7						
	Sp7 = 0.0
end

FDEF(iAxisNum).#CPE = 1
MFLAGS(iAxisNum).#HOME = 1
wait 500
RET

	
	
	
	
	

SET_CURRENTPOS_HOME:

SET FPOS(iAxisNum) = 0
TPOS(iAxisNum) = 0	!@@@ added
FDEF(iAxisNum).#CPE = 1
MFLAGS(iAxisNum).#HOME = 1
ENABLE (iAxisNum) 
! Move to 0 to clean-up residual FPOS error on GUI
PTP/ev (iAxisNum), 0, 180
wait 2000
!After homing, change Sp to 0.0 to prevent follow-on move.
OldSP(iAxisNum)=0
if 		iAxisNum = 0								
	Sp0 = 0.0
elseif 	iAxisNum = 1							
	Sp1 = 0.0
elseif 	iAxisNum = 2						
	Sp2 = 0.0
elseif 	iAxisNum = 3						
	Sp3 = 0.0
elseif 	iAxisNum = 4						
	Sp4 = 0.0
elseif 	iAxisNum = 5						
	Sp5 = 0.0
elseif 	iAxisNum = 6						
	Sp6 = 0.0
elseif 	iAxisNum = 7						
	Sp7 = 0.0
end
RET


HOME_TO_TORQ_AVP:


MFLAGS(iAxisNum).#HOME = 0
FDEF(iAxisNum).#LL=0       ! Disable the iAxis left limit default response
FDEF(iAxisNum).#RL=0       ! Disable the iAxis right limit default response
ENABLE (iAxisNum)          ! Enable the iAxis drive
wait(100)
JOG/v (iAxisNum), dHomingVel(iAxisNum)*dAxisPitch(iAxisNum)           ! Move to the left  
wait(1000)
KILL iAxisNum
!reduce the motor current saturations
store_CURI = XCURI(iAxisNum)
store_CURV = XCURV(iAxisNum)

real XCURI_actural, HmCurrent_Of_Drive, FtCurrent_Of_Drive
HmCurrent_Of_Drive = dHomingTorq(iAxisNum)*(4*XRMS(iAxisNum))/100.0
FtCurrent_Of_Drive = dFaultTorq(iAxisNum)*(4*XRMS(iAxisNum))/100.0

XCURI(iAxisNum) = HmCurrent_Of_Drive
XCURV(iAxisNum) = HmCurrent_Of_Drive
!CERRI(iAxisNum)=0.01*dHomingTorq(iAxisNum)*CERRI(iAxisNum)
!CERRV(iAxisNum)=0.01*dHomingTorq(iAxisNum)*CERRV(iAxisNum)
! Turn off the CPE response
FDEF(iAxisNum).#CPE = 0

JOG/v (iAxisNum), -dHomingVel(iAxisNum)*dAxisPitch(iAxisNum)           ! Move to the left  
TILL (FAULT(iAxisNum).#CPE = 1)   			! Wait for the critical PE
KILL iAxisNum

if (dAxisPitch(iAxisNum)>0.01)
	dCE(iAxisNum)=FPOS(iAxisNum)/dAxisPitch(iAxisNum)+dHomeOffset(iAxisNum)
end
block
SET FPOS(iAxisNum) = 0.0 -dHomeOffset(iAxisNum)*dAxisPitch(iAxisNum)
PTP/e (iAxisNum), 100!0.0
end



deltaI = FtCurrent_Of_Drive - HmCurrent_Of_Drive
deltaV = FtCurrent_Of_Drive - HmCurrent_Of_Drive
LOOP 100
	XCURI(iAxisNum) = XCURI(iAxisNum) + 0.01*(deltaI)
	XCURV(iAxisNum) = XCURV(iAxisNum) + 0.01*(deltaV)
	wait 20	!10ms x 100 = 1 sec ramp
END

XCURI(iAxisNum) = FtCurrent_Of_Drive !@@@ moved here from end of if-thens
XCURV(iAxisNum) = FtCurrent_Of_Drive	!@@@ moved here from end of if-thens
CERRV(iAxisNum) = store_CERRV0
CERRI(iAxisNum) = store_CERRI0
FDEF(iAxisNum).#CPE = 1

BREAK iAxisNum
!SET FPOS(iAxisNum) = 0.0 -dHomeOffset(iAxisNum)*dAxisPitch(iAxisNum)	!@CR2015-12-31 modified home position to simple offset - why move to index?
PTP/ev (iAxisNum), 0.0, dHomingVel(iAxisNum)*dAxisPitch(iAxisNum)


!After homing, change Sp to 0.0 to prevent follow-on move.
OldSP(iAxisNum)=0
if 		iAxisNum = 0								
	Sp0 = 0.0
elseif 	iAxisNum = 1							
	Sp1 = 0.0
elseif 	iAxisNum = 2						
	Sp2 = 0.0
elseif 	iAxisNum = 3						
	Sp3 = 0.0
elseif 	iAxisNum = 4						
	Sp4 = 0.0
elseif 	iAxisNum = 5						
	Sp5 = 0.0
elseif 	iAxisNum = 6						
	Sp6 = 0.0
elseif 	iAxisNum = 7						
	Sp7 = 0.0
end
FDEF(iAxisNum).#CPE = 1
MFLAGS(iAxisNum).#HOME = 1
RET

HOME_TO_SWITCH_CAR:
REAL rMax, rMin
INT iCARAIChannel
iCARAIChannel = 0

MFLAGS(iAxisNum).#HOME = 0
FDEF(iAxisNum).#LL=0       ! Disable the iAxis left limit default response
FDEF(iAxisNum).#RL=0       ! Disable the iAxis right limit default response
ENABLE (iAxisNum)          ! Enable the iAxis drive
wait(1000)
! Jog until critical position error is triggered
if (FtdCarHomeSW.iAxisNum = 1)
	!JOG/v (iAxisNum), dHomingVel(iAxisNum)           ! Move to the left  
	!TILL (FtdCarHomeSW(iAxisNum) = 0)   			! Wait for home switch input
	!BREAK (iAxisNum)
	PTP/er (iAxisNum), 15
end

if(iType(iAxisNum)=4007)	! if it is an analog sensor, perform sensor calibration first.
! find which analog channel is the home sensor at

	if AnalogCARAxis(0)=iAxisNum
		iCARAIChannel = 0
	elseif AnalogCARAxis(1)=iAxisNum
		iCARAIChannel = 1
	elseif AnalogCARAxis(2)=iAxisNum
		iCARAIChannel = 2
	elseif AnalogCARAxis(3)=iAxisNum
		iCARAIChannel = 3
	end
! scan the analog signal and acq 1000 points from the analog signal while spinning
	DC/S iAxisNum,Datalog,1000,100,FPOS(iAxisNum),AIN(iCARAIChannel)
	JOG/v (iAxisNum), 4 
	till (^AST(iAxisNum).#DC)	!wait DAQ to finish
	KILL iAxisNum
	WAIT 2000
! find the median in analog signal	
	rMax = MAX(Datalog, 1, 1, 0,999)
	rMin = MIN(Datalog, 1, 1, 0,999)
	if (rMax-rMin)>1.0
		AnologHomeSignalMedian(iCARAIChannel) = (rMax + rMin)/2.0
	else
		disable(iAxisNum)  
		iControlProcessState(iAxisNum) = 404
		RET
	end	
! find the position where is the peak signal
	int thepeakposIndex
	thepeakposIndex = MAXI(Datalog, 1, 1, 0,999)
! move the CAR to the neighbourhood of the peak	
	PTP/e (iAxisNum), Datalog(0)(thepeakposIndex)+15
	WAIT 3000
end





real FirstEdge, SecondEdge
!Find the leading edge of the home sensor
JOG/v (iAxisNum), -dHomingVel(iAxisNum)           ! Move to the left  
TILL (FtdCarHomeSW.iAxisNum = 1)   			! Wait for home switch input
FirstEdge = FPOS(iAxisNum)
KILL iAxisNum
WAIT 2000
!back off
PTP/e (iAxisNum), FirstEdge + 10
!low speed homing
JOG/v (iAxisNum), -dHomingVel(iAxisNum)           ! Move to the left  
TILL (FtdCarHomeSW.iAxisNum = 1)   			! Wait for home switch input
FirstEdge = FPOS(iAxisNum)
KILL iAxisNum
WAIT 2000
PTP/e (iAxisNum), FirstEdge - 15
!low speed homing
JOG/v (iAxisNum), dHomingVel(iAxisNum)           ! Move to the left  
TILL (FtdCarHomeSW.iAxisNum = 1) 
SecondEdge = FPOS(iAxisNum)
KILL iAxisNum
WAIT 2000
PTP/ev (iAxisNum), 0.5*(FirstEdge+SecondEdge)+dHomeOffset(iAxisNum), dHomingVel(iAxisNum)
wait 3000

SET FPOS(iAxisNum) = 0	!@CR2015-12-31 modified home position to simple offset - why move to index?
TPOS(iAxisNum) = 0
! After moving to home offset, re-zero the axis.  This is now the "working zero" for the axis
SET FPOS(iAxisNum) = 0	!@CR2015-12-31 modified home position to simple offset - why move to index?
TPOS(iAxisNum) = 0
! perform another move to zero (because without, there is residual FPOS data that shows at GUI - looks funny
FDEF(iAxisNum).#CPE = 1
MFLAGS(iAxisNum).#HOME = 1
wait 500

PTP/ev (iAxisNum), 0.0, dHomingVel(iAxisNum)
wait 2000

!After homing, change Sp to 0.0 to prevent follow-on move.
OldSP(iAxisNum)=0
if 		iAxisNum = 0								
	Sp0 = 0.0
elseif 	iAxisNum = 1							
	Sp1 = 0.0
elseif 	iAxisNum = 2						
	Sp2 = 0.0
elseif 	iAxisNum = 3						
	Sp3 = 0.0
elseif 	iAxisNum = 4						
	Sp4 = 0.0
elseif 	iAxisNum = 5						
	Sp5 = 0.0
elseif 	iAxisNum = 6						
	Sp6 = 0.0
elseif 	iAxisNum = 7						
	Sp7 = 0.0
end

RET
#4
!#/ Controller version = 2.27
!#/ Date = 12/28/2015 10:33 AM
!#/ User remarks = 
! motor cummt and axis homing routine for all types except for robot arms

!Change log
! DATE		ECO#	Reason for Change			Remarks
!9/21/2015	N/A		ini release					ZW
!12/28/15	N/A		tuning homing				ZW
!10/16/17	N/A		turn on CPE before move offset, also link CPE to %home torque
!11/17/2017 Disconnet the AVP CPE link to home torques which cause not homing
Global INT	iLl_Interlock_Cmd	!iWord10Bit1			:= doutputs.,
Global INT	iSt_Interlock_Cmd	!iWord10Bit5			:= doutputs.,
Global INT	iCar_Interlock_Cmd	!iWord10Bit9			:= doutputs.,
Global INT  bAxisInterlocked

GLOBAL real OldSP(8), Datalog(2)(1000)
global int AnalogCARAxis(4)
global real AnologHomeSignalMedian(4)
real SP_Direction                 ! Search direction 
real SP_Settle_Time               ! Settling time at Detent Points [msec]
real SP_Search_Vel                ! Search velocity [user-units/sec]
real SP_Drive                     ! Actuating drive command [% of maximum]
real SP_Max_Search                ! Maximum search distance [user-units]
real SP_Init_Offset               ! Initial commutation offset [elec. degrees]
real PrevAcc, PrevDec, PrevKDec, PrevJerk, PrevVel
real minAnalog, maxAnalog
int iNeedSLL, iNeedSRL
real store_CURI, store_CURV
int iAxisNum
real store_CERRI0, store_CERRV0

iAxisNum = 4	!set the particular axis for the buffer
If ((iType(iAxisNum)= 4008) & (iCar_Interlock_Cmd = 0)) | ((iType(iAxisNum)= 5011) & (iLl_Interlock_Cmd = 0)) | ((iType(iAxisNum)= 5012) & (iSt_Interlock_Cmd = 0)) | ((iInterlockEnabled(iAxisNum)=1)&(bAxisInterlocked=1))
	iControlProcessState(iAxisNum) = 113
	GOTO ENDOFHOMING
end

iControlProcessState(iAxisNum) = 14
CERRI(iAxisNum)=4.39
CERRV(iAxisNum)=4.39
store_CERRI0 = CERRI(iAxisNum)
store_CERRV0 = CERRV(iAxisNum)
MFLAGS(iAxisNum).#HOME = 0			
iNeedSLL=0
iNeedSRL=0

if FMASK(iAxisNum).#SLL = 1
	FMASK(iAxisNum).#SLL = 0
	iNeedSLL=1
end

if FMASK(iAxisNum).#SRL = 1
	FMASK(iAxisNum).#SRL = 0
	iNeedSRL=1
end

if MFLAGS(iAxisNum).9=0                ! not commutation

	SP_Drive = 5.475          
	SP_Settle_Time = 1000    
	SP_Search_Vel  = 6         
	SP_Init_Offset = 0
	SP_Direction =  -1
	SP_Max_Search = 144
	PrevAcc = ACC(iAxisNum)
	PrevDec = DEC(iAxisNum)
	PrevKDec=KDEC(iAxisNum)
	PrevJerk=JERK(iAxisNum)

	!   Output varibale:
	int  SP_Fail_Code                 ! Faiure Code of the startup program

	! Auxiliary variable:
	real SP_Pitch                     ! Magnetic pitch [180 elec.deg. in user units]

	! State Flag
	int  SP_InCommutationStartup      ! Flag indicating commutation startup is in progress


	SP_Pitch=SLCPRD(iAxisNum)/SLCNP(iAxisNum)*EFAC(iAxisNum)


	!******************************************************************************* 
	! INITIALIZE
	FCLEAR(iAxisNum)   
	disable(iAxisNum)                     
	SETCONF(216,(iAxisNum),0)                ! Reset commutation state
	setconf(214,(iAxisNum),SP_Init_Offset)   ! Set initial commutation phase
	SP_Fail_Code=0                        ! Reset failure
	SP_Direction=1                        ! Move in positive direction
	SP_InCommutationStartup=1             ! commutation startup in progress
	!******************************************************************************* 
	! SET MOTION PROFILE FOR COMMUTATION STARTUP PROCESS 
	!
	!   WARNING: The following are the suggested motion parameters for the startup
	!     process. Check that the values are suitable for your application.

	ACC(iAxisNum)=SP_Search_Vel*10.; DEC(iAxisNum)=SP_Search_Vel*10. 
	KDEC(iAxisNum)=SP_Search_Vel*50.; JERK(iAxisNum)=SP_Search_Vel*100.

	!******************************************************************************* 
	! STEP 1 - MOVE TO FIRST DETENT POINT 
	!
	! WARNING: The motor moves to a detent point by jump. 
	!   The jump distance is up to one magnetic pitch in any direction.
	!   The motor jumps to the closest detent point within its motion range.
	!   If necessary modify initial detent point by changing the variable 
	!     SP_Init_Offset between 0-360 electrical degrees.  
	enable(iAxisNum)
	wait(100)
	while (DCOM(iAxisNum)+0.05 < SP_Drive); DCOM(iAxisNum) = DCOM(iAxisNum) + 0.05; end
	DCOM(iAxisNum) = SP_Drive
	wait SP_Settle_Time
	call Limit_Check
	!******************************************************************************* 
	! STEP 2 - MOVE TO SECOND DETENT POINT
	!
	!   The program moves the motor 90 electrical degrees in order to eliminate
	!      a state of unstable equilibrium.

	Move_Detent:
	ptp/rv (iAxisNum), SP_Direction*SP_Pitch/2.,SP_Search_Vel
	till ^AST(iAxisNum).#MOVE; wait SP_Settle_Time
	call Limit_Check
	disable(iAxisNum)

	MFLAGS(iAxisNum).9=1               ! Set commutation state 
	DCOM(iAxisNum) = 0

	! If motor is to be left enabled after startup process delete the following line:

	Finish:
	SP_InCommutationStartup=0              ! commutation startup is finished
	If SP_Fail_Code=0; disp "   Commutation Startup Finished."
	else disp "   Commutation Startup Failed."; disp "   Failure Code = %i",SP_Fail_Code; end

	ACC(iAxisNum)=PrevAcc
	DEC(iAxisNum)=PrevDec
	KDEC(iAxisNum)=PrevKDec
	JERK(iAxisNum)=PrevJerk

end

!at this moment, the axis shall commuted. If it is, then start homing

if MFLAGS(iAxisNum).9=1
	if (iHomingMethod(iAxisNum) = 0)		!0-home to torque; 1-homing to switch; 2-manual set home;
	    if (iType(iAxisNum)>1000)&(iType(iAxisNum)<2000) 	
		!AVP home to torques - always
			call HOME_TO_TORQ_AVP	!different than LP home to torque
		else
			call HOME_TO_TORQ
		end	
	elseif (iHomingMethod(iAxisNum) = 1)&(iType(iAxisNum)<6000)
		if(iType(iAxisNum)>3000)&(iType(iAxisNum)<5000)
			CALL HOME_TO_SWITCH_CAR
		elseif(iType(iAxisNum) = 5008 | iType(iAxisNum) = 5009 | iType(iAxisNum) = 5010 | iType(iAxisNum) = 5018 | iType(iAxisNum) = 5019 | iType(iAxisNum) = 5020 | iType(iAxisNum) = 5028 | iType(iAxisNum) = 5029 | iType(iAxisNum) = 5030)
			call HOME_TO_SWITCH_CARZ
			FDEF(iAxisNum).#LL=1       ! Enable the iAxis left limit default response
			FDEF(iAxisNum).#RL=1       ! Enable the iAxis right limit default response
		else	
			CALL HOME_TO_SWITCH
			FDEF(iAxisNum).#LL=1       ! Enable the iAxis left limit default response
		end
	elseif (iHomingMethod(iAxisNum) = 2)
		CALL SET_CURRENTPOS_HOME
	end
end
iControlProcessState(iAxisNum) = 10
if iNeedSLL = 1
	FMASK(iAxisNum).#SLL = 1
end

if iNeedSRL = 1
	FMASK(iAxisNum).#SRL = 1
end
ENDOFHOMING:
STOP

!******************************************************************************* 
!   The following routine move the motor away from limit switches 

Limit_Check:
if MERR(iAxisNum) & MERR(iAxisNum)<>5010 & MERR(iAxisNum)<>5011; SP_Fail_Code=1; DISABLE(iAxisNum); DCOM(iAxisNum)=0; goto Finish; end
!   if MERR(iAxisNum); SP_Fail_Code=1; DISABLE(iAxisNum); DCOM(iAxisNum)=0; goto Finish; end 
if (FAULT(iAxisNum).#LL)|(FAULT(iAxisNum).#RL)
   if FAULT(iAxisNum).#LL; SP_Direction=1; else SP_Direction=-1; end
   ptp/rv (iAxisNum), SP_Direction*SP_Max_Search, SP_Search_Vel 
   till ((^FAULT(iAxisNum).#LL)&(^FAULT(iAxisNum).#RL))|(^AST(iAxisNum).#MOVE)
   if (FAULT(iAxisNum).#LL)|(FAULT(iAxisNum).#RL); SP_Fail_Code=4; DISABLE(iAxisNum); DCOM(iAxisNum)=0; goto Finish; end 
   kill(iAxisNum);  wait SP_Settle_Time; goto Move_Detent
end
ret

ON ((SP_InCommutationStartup = 1) & ((FAULT(iAxisNum)&0x30f80)>0 | (S_FAULT&0x30000000)>0))
SP_Fail_Code=1; DISABLE(iAxisNum); DCOM(iAxisNum)=0
CALL Finish 
RET



HOME_TO_TORQ:

MFLAGS(iAxisNum).#HOME = 0
FDEF(iAxisNum).#LL=0       ! Disable the iAxis left limit default response
FDEF(iAxisNum).#RL=0       ! Disable the iAxis right limit default response
ENABLE (iAxisNum)          ! Enable the iAxis drive
wait(1000)
JOG/v (iAxisNum), dHomingVel(iAxisNum)*dAxisPitch(iAxisNum)           ! Move to the left  
wait(1000)
KILL iAxisNum
! Reduce the motor current saturations
store_CURI = XCURI(iAxisNum)
store_CURV = XCURV(iAxisNum)
XCURI(iAxisNum) = dHomingTorq(iAxisNum)
XCURV(iAxisNum) = dHomingTorq(iAxisNum)

CERRI(iAxisNum)=0.01*dHomingTorq(iAxisNum)*CERRI(iAxisNum)
CERRV(iAxisNum)=0.01*dHomingTorq(iAxisNum)*CERRV(iAxisNum)

! Turn off the CPE response
FDEF(iAxisNum).#CPE = 0
! Jog until critical position error is triggered
JOG/v (iAxisNum), -dHomingVel(iAxisNum)*dAxisPitch(iAxisNum)           ! Move to the left  
TILL (FAULT(iAxisNum).#CPE = 1)   			! Wait for the critical PE
Kill iAxisNum
! Stop motor, let settle, set FPOS and TPOS to 0.0.  Then turn CPE mask back off
SET FPOS(iAxisNum) = 0.0
PTP/e (iAxisNum), 0.0
REAL deltaI, deltaV
deltaI = store_CURI - dHomingTorq(iAxisNum)
deltaV = store_CURV - dHomingTorq(iAxisNum)
LOOP 100
	XCURI(iAxisNum) = XCURI(iAxisNum) + 0.01*(deltaI)
	XCURV(iAxisNum) = XCURV(iAxisNum) + 0.01*(deltaV)
	SET FPOS(iAxisNum) = 0
	wait 20	!10ms x 100 = 1 sec ramp
END
!In case of rounding error, lastly set XCUR* to stored value - should be nearly identical at this point
XCURI(iAxisNum) = store_CURI
XCURV(iAxisNum) = store_CURV

CERRV(iAxisNum) = store_CERRV0
CERRI(iAxisNum) = store_CERRI0
FDEF(iAxisNum).#CPE = 1

BREAK iAxisNum
SET FPOS(iAxisNum) = 0 -dHomeOffset(iAxisNum)*dAxisPitch(iAxisNum)
PTP/ev (iAxisNum), 0.0, dHomingVel(iAxisNum)*dAxisPitch(iAxisNum)

!After homing, change Sp to 0.0 to prevent follow-on move.
OldSP(iAxisNum)=0
if 		iAxisNum = 0								
	Sp0 = 0.0
elseif 	iAxisNum = 1							
	Sp1 = 0.0
elseif 	iAxisNum = 2						
	Sp2 = 0.0
elseif 	iAxisNum = 3						
	Sp3 = 0.0
elseif 	iAxisNum = 4						
	Sp4 = 0.0
elseif 	iAxisNum = 5						
	Sp5 = 0.0
elseif 	iAxisNum = 6						
	Sp6 = 0.0
elseif 	iAxisNum = 7						
	Sp7 = 0.0
end

FDEF(iAxisNum).#CPE = 1
MFLAGS(iAxisNum).#HOME = 1
wait 500
RET


HOME_TO_SWITCH:

MFLAGS(iAxisNum).#HOME = 0
FDEF(iAxisNum).#LL=0       ! Disable the iAxis left limit default response
FDEF(iAxisNum).#RL=0       ! Disable the iAxis right limit default response
ENABLE (iAxisNum)          ! Enable the iAxis drive
wait(1000)

if  (IN(0).iAxisNum = 1) 
	JOG/v (iAxisNum), dHomingVel(iAxisNum)*dAxisPitch(iAxisNum)           ! Move to the left  
	TILL (IN(0).iAxisNum = 0)   			! Wait for home switch input
	! Stop motor, let settle, set FPOS and TPOS to 0.0.  Then turn CPE mask back off
	KILL (iAxisNum)
end
! Reduce the motor current saturations
store_CURI = XCURI(iAxisNum)
store_CURV = XCURV(iAxisNum)
XCURI(iAxisNum) = dHomingTorq(iAxisNum)
XCURV(iAxisNum) = dHomingTorq(iAxisNum)
! Do NOT Turn off the CPE response for home to switch
!FDEF(iAxisNum).#CPE = 0
! Jog until critical position error is triggered
JOG/v (iAxisNum), -dHomingVel(iAxisNum)*dAxisPitch(iAxisNum)           ! Move to the left  
   			! Wait for home switch input
TILL (IN(0).iAxisNum = 1)
! Stop motor, let settle, set FPOS and TPOS to 0.0.  Then turn CPE mask back off
KILL (iAxisNum)
wait 3000
SET FPOS(iAxisNum) = 0	!@CR2015-12-31 modified home position to simple offset - why move to index?
TPOS(iAxisNum) = 0
! FDEF(iAxisNum).#CPE = 1	!no need to turn turn CPE on because it was not turned off.
! disable motor to return to running torque, and immediately re-enable. This prevents an abrupt move with torque increase.

LOOP 100
	XCURI(iAxisNum) = XCURI(iAxisNum) + 0.01*(store_CURI - dHomingTorq(iAxisNum))
	wait 10	!10ms x 100 = 1 sec ramp
	XCURV(iAxisNum) = XCURV(iAxisNum) + 0.01*(store_CURV - dHomingTorq(iAxisNum))
	wait 10	!10ms x 100 = 1 sec ramp
END
! In case of rounding error, lastly set XCUR* to stored value - should be nearly identical at this point
XCURI(iAxisNum) = store_CURI
XCURV(iAxisNum) = store_CURV

!disable (iAxisNum)
!XCURI(iAxisNum) = store_CURI !@@@ moved here from end of if-thens
!XCURV(iAxisNum) = store_CURV	!@@@ moved here from end of if-thens
!enable (iAxisNum)
wait 1000
PTP/ev (iAxisNum), dHomeOffset(iAxisNum)*dAxisPitch(iAxisNum), dHomingVel(iAxisNum)*dAxisPitch(iAxisNum)
wait 3000
! After moving to home offset, re-zero the axis.  This is now the "working zero" for the axis
SET FPOS(iAxisNum) = 0	!@CR2015-12-31 modified home position to simple offset - why move to index?
TPOS(iAxisNum) = 0
! perform another move to zero (because without, there is residual FPOS data that shows at GUI - looks funny
PTP/ev (iAxisNum), 0.0, dHomingVel(iAxisNum)*dAxisPitch(iAxisNum)
wait 2000

!After homing, change Sp to 0.0 to prevent follow-on move.
OldSP(iAxisNum)=0
if 		iAxisNum = 0								
	Sp0 = 0.0
elseif 	iAxisNum = 1							
	Sp1 = 0.0
elseif 	iAxisNum = 2						
	Sp2 = 0.0
elseif 	iAxisNum = 3						
	Sp3 = 0.0
elseif 	iAxisNum = 4						
	Sp4 = 0.0
elseif 	iAxisNum = 5						
	Sp5 = 0.0
elseif 	iAxisNum = 6						
	Sp6 = 0.0
elseif 	iAxisNum = 7						
	Sp7 = 0.0
end

FDEF(iAxisNum).#CPE = 1
MFLAGS(iAxisNum).#HOME = 1
wait 500
RET







	
HOME_TO_SWITCH_CARZ:

MFLAGS(iAxisNum).#HOME = 0
FDEF(iAxisNum).#LL=0       ! Disable the iAxis left limit default response
FDEF(iAxisNum).#RL=0       ! Disable the iAxis right limit default response
ENABLE (iAxisNum)          ! Enable the iAxis drive
wait(1000)

if  (SAFIN(iAxisNum).#LL = 1) 
	JOG/v (iAxisNum), dHomingVel(iAxisNum)*dAxisPitch(iAxisNum)           ! Move to the left  
	TILL (SAFIN(iAxisNum).#LL  = 0)   			! Wait for home switch input
	! Stop motor, let settle, set FPOS and TPOS to 0.0.  Then turn CPE mask back off
	KILL (iAxisNum)
end
! Reduce the motor current saturations
store_CURI = XCURI(iAxisNum)
store_CURV = XCURV(iAxisNum)
XCURI(iAxisNum) = dHomingTorq(iAxisNum)
XCURV(iAxisNum) = dHomingTorq(iAxisNum)
! Do NOT Turn off the CPE response for home to switch
!FDEF(iAxisNum).#CPE = 0
! Jog until critical position error is triggered
JOG/v (iAxisNum), -dHomingVel(iAxisNum)*dAxisPitch(iAxisNum)           ! Move to the left  
   			! Wait for home switch input
TILL (SAFIN(iAxisNum).#LL = 1)
! Stop motor, let settle, set FPOS and TPOS to 0.0.  Then turn CPE mask back off
KILL (iAxisNum)
wait 3000

! @JR 2021-10-18 - Move down until upper limit switch opens
JOG/v (iAxisNum), dHomingVel(iAxisNum)*dAxisPitch(iAxisNum)
TILL (SAFIN(iAxisNum).#LL = 0)
KILL (iAxisNum)
wait 3000

SET FPOS(iAxisNum) = 0	!@CR2015-12-31 modified home position to simple offset - why move to index?
TPOS(iAxisNum) = 0
! FDEF(iAxisNum).#CPE = 1	!no need to turn turn CPE on because it was not turned off.
! disable motor to return to running torque, and immediately re-enable. This prevents an abrupt move with torque increase.

!LOOP 100
!	XCURI(iAxisNum) = XCURI(iAxisNum) + 0.01*(store_CURI - dHomingTorq(iAxisNum))
!	wait 10	!10ms x 100 = 1 sec ramp
!	XCURV(iAxisNum) = XCURV(iAxisNum) + 0.01*(store_CURV - dHomingTorq(iAxisNum))
!	wait 10	!10ms x 100 = 1 sec ramp
!END
! In case of rounding error, lastly set XCUR* to stored value - should be nearly identical at this point
XCURI(iAxisNum) = store_CURI
XCURV(iAxisNum) = store_CURV

!disable (iAxisNum)
!XCURI(iAxisNum) = store_CURI !@@@ moved here from end of if-thens
!XCURV(iAxisNum) = store_CURV	!@@@ moved here from end of if-thens
!enable (iAxisNum)
wait 1000
PTP/ev (iAxisNum), dHomeOffset(iAxisNum)*dAxisPitch(iAxisNum), dHomingVel(iAxisNum)*dAxisPitch(iAxisNum)
wait 3000
! After moving to home offset, re-zero the axis.  This is now the "working zero" for the axis
SET FPOS(iAxisNum) = 0	!@CR2015-12-31 modified home position to simple offset - why move to index?
TPOS(iAxisNum) = 0
! perform another move to zero (because without, there is residual FPOS data that shows at GUI - looks funny
PTP/ev (iAxisNum), 0.0, dHomingVel(iAxisNum)*dAxisPitch(iAxisNum)
wait 2000

!After homing, change Sp to 0.0 to prevent follow-on move.
OldSP(iAxisNum)=0
if 		iAxisNum = 0								
	Sp0 = 0.0
elseif 	iAxisNum = 1							
	Sp1 = 0.0
elseif 	iAxisNum = 2						
	Sp2 = 0.0
elseif 	iAxisNum = 3						
	Sp3 = 0.0
elseif 	iAxisNum = 4						
	Sp4 = 0.0
elseif 	iAxisNum = 5						
	Sp5 = 0.0
elseif 	iAxisNum = 6						
	Sp6 = 0.0
elseif 	iAxisNum = 7						
	Sp7 = 0.0
end

FDEF(iAxisNum).#CPE = 1
MFLAGS(iAxisNum).#HOME = 1
wait 500
RET

	
	
	
	
	

SET_CURRENTPOS_HOME:

SET FPOS(iAxisNum) = 0
TPOS(iAxisNum) = 0	!@@@ added
FDEF(iAxisNum).#CPE = 1
MFLAGS(iAxisNum).#HOME = 1
ENABLE (iAxisNum) 
! Move to 0 to clean-up residual FPOS error on GUI
PTP/ev (iAxisNum), 0, 180
wait 2000
!After homing, change Sp to 0.0 to prevent follow-on move.
OldSP(iAxisNum)=0
if 		iAxisNum = 0								
	Sp0 = 0.0
elseif 	iAxisNum = 1							
	Sp1 = 0.0
elseif 	iAxisNum = 2						
	Sp2 = 0.0
elseif 	iAxisNum = 3						
	Sp3 = 0.0
elseif 	iAxisNum = 4						
	Sp4 = 0.0
elseif 	iAxisNum = 5						
	Sp5 = 0.0
elseif 	iAxisNum = 6						
	Sp6 = 0.0
elseif 	iAxisNum = 7						
	Sp7 = 0.0
end
RET


HOME_TO_TORQ_AVP:


MFLAGS(iAxisNum).#HOME = 0
FDEF(iAxisNum).#LL=0       ! Disable the iAxis left limit default response
FDEF(iAxisNum).#RL=0       ! Disable the iAxis right limit default response
ENABLE (iAxisNum)          ! Enable the iAxis drive
wait(100)
JOG/v (iAxisNum), dHomingVel(iAxisNum)*dAxisPitch(iAxisNum)           ! Move to the left  
wait(1000)
KILL iAxisNum
!reduce the motor current saturations
store_CURI = XCURI(iAxisNum)
store_CURV = XCURV(iAxisNum)

real XCURI_actural, HmCurrent_Of_Drive, FtCurrent_Of_Drive
HmCurrent_Of_Drive = dHomingTorq(iAxisNum)*(4*XRMS(iAxisNum))/100.0
FtCurrent_Of_Drive = dFaultTorq(iAxisNum)*(4*XRMS(iAxisNum))/100.0

XCURI(iAxisNum) = HmCurrent_Of_Drive
XCURV(iAxisNum) = HmCurrent_Of_Drive
!CERRI(iAxisNum)=0.01*dHomingTorq(iAxisNum)*CERRI(iAxisNum)
!CERRV(iAxisNum)=0.01*dHomingTorq(iAxisNum)*CERRV(iAxisNum)
! Turn off the CPE response
FDEF(iAxisNum).#CPE = 0

JOG/v (iAxisNum), -dHomingVel(iAxisNum)*dAxisPitch(iAxisNum)           ! Move to the left  
TILL (FAULT(iAxisNum).#CPE = 1)   			! Wait for the critical PE
KILL iAxisNum

if (dAxisPitch(iAxisNum)>0.01)
	dCE(iAxisNum)=FPOS(iAxisNum)/dAxisPitch(iAxisNum)+dHomeOffset(iAxisNum)
end
block
SET FPOS(iAxisNum) = 0.0 -dHomeOffset(iAxisNum)*dAxisPitch(iAxisNum)
PTP/e (iAxisNum), 100!0.0
end



deltaI = FtCurrent_Of_Drive - HmCurrent_Of_Drive
deltaV = FtCurrent_Of_Drive - HmCurrent_Of_Drive
LOOP 100
	XCURI(iAxisNum) = XCURI(iAxisNum) + 0.01*(deltaI)
	XCURV(iAxisNum) = XCURV(iAxisNum) + 0.01*(deltaV)
	wait 20	!10ms x 100 = 1 sec ramp
END

XCURI(iAxisNum) = FtCurrent_Of_Drive !@@@ moved here from end of if-thens
XCURV(iAxisNum) = FtCurrent_Of_Drive	!@@@ moved here from end of if-thens
CERRV(iAxisNum) = store_CERRV0
CERRI(iAxisNum) = store_CERRI0
FDEF(iAxisNum).#CPE = 1

BREAK iAxisNum
!SET FPOS(iAxisNum) = 0.0 -dHomeOffset(iAxisNum)*dAxisPitch(iAxisNum)	!@CR2015-12-31 modified home position to simple offset - why move to index?
PTP/ev (iAxisNum), 0.0, dHomingVel(iAxisNum)*dAxisPitch(iAxisNum)


!After homing, change Sp to 0.0 to prevent follow-on move.
OldSP(iAxisNum)=0
if 		iAxisNum = 0								
	Sp0 = 0.0
elseif 	iAxisNum = 1							
	Sp1 = 0.0
elseif 	iAxisNum = 2						
	Sp2 = 0.0
elseif 	iAxisNum = 3						
	Sp3 = 0.0
elseif 	iAxisNum = 4						
	Sp4 = 0.0
elseif 	iAxisNum = 5						
	Sp5 = 0.0
elseif 	iAxisNum = 6						
	Sp6 = 0.0
elseif 	iAxisNum = 7						
	Sp7 = 0.0
end
FDEF(iAxisNum).#CPE = 1
MFLAGS(iAxisNum).#HOME = 1
RET

HOME_TO_SWITCH_CAR:
REAL rMax, rMin
INT iCARAIChannel
iCARAIChannel = 0

MFLAGS(iAxisNum).#HOME = 0
FDEF(iAxisNum).#LL=0       ! Disable the iAxis left limit default response
FDEF(iAxisNum).#RL=0       ! Disable the iAxis right limit default response
ENABLE (iAxisNum)          ! Enable the iAxis drive
wait(1000)
! Jog until critical position error is triggered
if (FtdCarHomeSW.iAxisNum = 1)
	!JOG/v (iAxisNum), dHomingVel(iAxisNum)           ! Move to the left  
	!TILL (FtdCarHomeSW(iAxisNum) = 0)   			! Wait for home switch input
	!BREAK (iAxisNum)
	PTP/er (iAxisNum), 15
end

if(iType(iAxisNum)=4007)	! if it is an analog sensor, perform sensor calibration first.
! find which analog channel is the home sensor at

	if AnalogCARAxis(0)=iAxisNum
		iCARAIChannel = 0
	elseif AnalogCARAxis(1)=iAxisNum
		iCARAIChannel = 1
	elseif AnalogCARAxis(2)=iAxisNum
		iCARAIChannel = 2
	elseif AnalogCARAxis(3)=iAxisNum
		iCARAIChannel = 3
	end
! scan the analog signal and acq 1000 points from the analog signal while spinning
	DC/S iAxisNum,Datalog,1000,100,FPOS(iAxisNum),AIN(iCARAIChannel)
	JOG/v (iAxisNum), 4 
	till (^AST(iAxisNum).#DC)	!wait DAQ to finish
	KILL iAxisNum
	WAIT 2000
! find the median in analog signal	
	rMax = MAX(Datalog, 1, 1, 0,999)
	rMin = MIN(Datalog, 1, 1, 0,999)
	if (rMax-rMin)>1.0
		AnologHomeSignalMedian(iCARAIChannel) = (rMax + rMin)/2.0
	else
		disable(iAxisNum)  
		iControlProcessState(iAxisNum) = 404
		RET
	end	
! find the position where is the peak signal
	int thepeakposIndex
	thepeakposIndex = MAXI(Datalog, 1, 1, 0,999)
! move the CAR to the neighbourhood of the peak	
	PTP/e (iAxisNum), Datalog(0)(thepeakposIndex)+15
	WAIT 3000
end





real FirstEdge, SecondEdge
!Find the leading edge of the home sensor
JOG/v (iAxisNum), -dHomingVel(iAxisNum)           ! Move to the left  
TILL (FtdCarHomeSW.iAxisNum = 1)   			! Wait for home switch input
FirstEdge = FPOS(iAxisNum)
KILL iAxisNum
WAIT 2000
!back off
PTP/e (iAxisNum), FirstEdge + 10
!low speed homing
JOG/v (iAxisNum), -dHomingVel(iAxisNum)           ! Move to the left  
TILL (FtdCarHomeSW.iAxisNum = 1)   			! Wait for home switch input
FirstEdge = FPOS(iAxisNum)
KILL iAxisNum
WAIT 2000
PTP/e (iAxisNum), FirstEdge - 15
!low speed homing
JOG/v (iAxisNum), dHomingVel(iAxisNum)           ! Move to the left  
TILL (FtdCarHomeSW.iAxisNum = 1) 
SecondEdge = FPOS(iAxisNum)
KILL iAxisNum
WAIT 2000
PTP/ev (iAxisNum), 0.5*(FirstEdge+SecondEdge)+dHomeOffset(iAxisNum), dHomingVel(iAxisNum)
wait 3000

SET FPOS(iAxisNum) = 0	!@CR2015-12-31 modified home position to simple offset - why move to index?
TPOS(iAxisNum) = 0
! After moving to home offset, re-zero the axis.  This is now the "working zero" for the axis
SET FPOS(iAxisNum) = 0	!@CR2015-12-31 modified home position to simple offset - why move to index?
TPOS(iAxisNum) = 0
! perform another move to zero (because without, there is residual FPOS data that shows at GUI - looks funny
FDEF(iAxisNum).#CPE = 1
MFLAGS(iAxisNum).#HOME = 1
wait 500

PTP/ev (iAxisNum), 0.0, dHomingVel(iAxisNum)
wait 2000

!After homing, change Sp to 0.0 to prevent follow-on move.
OldSP(iAxisNum)=0
if 		iAxisNum = 0								
	Sp0 = 0.0
elseif 	iAxisNum = 1							
	Sp1 = 0.0
elseif 	iAxisNum = 2						
	Sp2 = 0.0
elseif 	iAxisNum = 3						
	Sp3 = 0.0
elseif 	iAxisNum = 4						
	Sp4 = 0.0
elseif 	iAxisNum = 5						
	Sp5 = 0.0
elseif 	iAxisNum = 6						
	Sp6 = 0.0
elseif 	iAxisNum = 7						
	Sp7 = 0.0
end

RET
#5
!#/ Controller version = 2.27
!#/ Date = 12/28/2015 10:33 AM
!#/ User remarks = 
! motor cummt and axis homing routine for all types except for robot arms

!Change log
! DATE		ECO#	Reason for Change			Remarks
!9/21/2015	N/A		ini release					ZW
!12/28/15	N/A		tuning homing				ZW
!10/16/17	N/A		turn on CPE before move offset, also link CPE to %home torque
!11/17/2017 Disconnet the AVP CPE link to home torques which cause not homing
Global INT	iLl_Interlock_Cmd	!iWord10Bit1			:= doutputs.,
Global INT	iSt_Interlock_Cmd	!iWord10Bit5			:= doutputs.,
Global INT	iCar_Interlock_Cmd	!iWord10Bit9			:= doutputs.,
Global INT  bAxisInterlocked

GLOBAL real OldSP(8), Datalog(2)(1000)
global int AnalogCARAxis(4)
global real AnologHomeSignalMedian(4)
real SP_Direction                 ! Search direction 
real SP_Settle_Time               ! Settling time at Detent Points [msec]
real SP_Search_Vel                ! Search velocity [user-units/sec]
real SP_Drive                     ! Actuating drive command [% of maximum]
real SP_Max_Search                ! Maximum search distance [user-units]
real SP_Init_Offset               ! Initial commutation offset [elec. degrees]
real PrevAcc, PrevDec, PrevKDec, PrevJerk, PrevVel
real minAnalog, maxAnalog
int iNeedSLL, iNeedSRL
real store_CURI, store_CURV
int iAxisNum
real store_CERRI0, store_CERRV0

iAxisNum=5	!set the particular axis for the buffer
If ((iType(iAxisNum)= 4008) & (iCar_Interlock_Cmd = 0)) | ((iType(iAxisNum)= 5011) & (iLl_Interlock_Cmd = 0)) | ((iType(iAxisNum)= 5012) & (iSt_Interlock_Cmd = 0)) | ((iInterlockEnabled(iAxisNum)=1)&(bAxisInterlocked=1))
	iControlProcessState(iAxisNum) = 113
	GOTO ENDOFHOMING
end

iControlProcessState(iAxisNum) = 14
CERRI(iAxisNum)=4.39
CERRV(iAxisNum)=4.39
store_CERRI0 = CERRI(iAxisNum)
store_CERRV0 = CERRV(iAxisNum)
MFLAGS(iAxisNum).#HOME = 0			
iNeedSLL=0
iNeedSRL=0

if FMASK(iAxisNum).#SLL = 1
	FMASK(iAxisNum).#SLL = 0
	iNeedSLL=1
end

if FMASK(iAxisNum).#SRL = 1
	FMASK(iAxisNum).#SRL = 0
	iNeedSRL=1
end

if MFLAGS(iAxisNum).9=0                ! not commutation

	SP_Drive = 5.475          
	SP_Settle_Time = 1000    
	SP_Search_Vel  = 6         
	SP_Init_Offset = 0
	SP_Direction =  -1
	SP_Max_Search = 144
	PrevAcc = ACC(iAxisNum)
	PrevDec = DEC(iAxisNum)
	PrevKDec=KDEC(iAxisNum)
	PrevJerk=JERK(iAxisNum)

	!   Output varibale:
	int  SP_Fail_Code                 ! Faiure Code of the startup program

	! Auxiliary variable:
	real SP_Pitch                     ! Magnetic pitch [180 elec.deg. in user units]

	! State Flag
	int  SP_InCommutationStartup      ! Flag indicating commutation startup is in progress


	SP_Pitch=SLCPRD(iAxisNum)/SLCNP(iAxisNum)*EFAC(iAxisNum)


	!******************************************************************************* 
	! INITIALIZE
	FCLEAR(iAxisNum)   
	disable(iAxisNum)                     
	SETCONF(216,(iAxisNum),0)                ! Reset commutation state
	setconf(214,(iAxisNum),SP_Init_Offset)   ! Set initial commutation phase
	SP_Fail_Code=0                        ! Reset failure
	SP_Direction=1                        ! Move in positive direction
	SP_InCommutationStartup=1             ! commutation startup in progress
	!******************************************************************************* 
	! SET MOTION PROFILE FOR COMMUTATION STARTUP PROCESS 
	!
	!   WARNING: The following are the suggested motion parameters for the startup
	!     process. Check that the values are suitable for your application.

	ACC(iAxisNum)=SP_Search_Vel*10.; DEC(iAxisNum)=SP_Search_Vel*10. 
	KDEC(iAxisNum)=SP_Search_Vel*50.; JERK(iAxisNum)=SP_Search_Vel*100.

	!******************************************************************************* 
	! STEP 1 - MOVE TO FIRST DETENT POINT 
	!
	! WARNING: The motor moves to a detent point by jump. 
	!   The jump distance is up to one magnetic pitch in any direction.
	!   The motor jumps to the closest detent point within its motion range.
	!   If necessary modify initial detent point by changing the variable 
	!     SP_Init_Offset between 0-360 electrical degrees.  
	enable(iAxisNum)
	wait(100)
	while (DCOM(iAxisNum)+0.05 < SP_Drive); DCOM(iAxisNum) = DCOM(iAxisNum) + 0.05; end
	DCOM(iAxisNum) = SP_Drive
	wait SP_Settle_Time
	call Limit_Check
	!******************************************************************************* 
	! STEP 2 - MOVE TO SECOND DETENT POINT
	!
	!   The program moves the motor 90 electrical degrees in order to eliminate
	!      a state of unstable equilibrium.

	Move_Detent:
	ptp/rv (iAxisNum), SP_Direction*SP_Pitch/2.,SP_Search_Vel
	till ^AST(iAxisNum).#MOVE; wait SP_Settle_Time
	call Limit_Check
	disable(iAxisNum)

	MFLAGS(iAxisNum).9=1               ! Set commutation state 
	DCOM(iAxisNum) = 0

	! If motor is to be left enabled after startup process delete the following line:

	Finish:
	SP_InCommutationStartup=0              ! commutation startup is finished
	If SP_Fail_Code=0; disp "   Commutation Startup Finished."
	else disp "   Commutation Startup Failed."; disp "   Failure Code = %i",SP_Fail_Code; end

	ACC(iAxisNum)=PrevAcc
	DEC(iAxisNum)=PrevDec
	KDEC(iAxisNum)=PrevKDec
	JERK(iAxisNum)=PrevJerk

end

!at this moment, the axis shall commuted. If it is, then start homing

if MFLAGS(iAxisNum).9=1
	if (iHomingMethod(iAxisNum) = 0)		!0-home to torque; 1-homing to switch; 2-manual set home;
	    if (iType(iAxisNum)>1000)&(iType(iAxisNum)<2000) 	
		!AVP home to torques - always
			call HOME_TO_TORQ_AVP	!different than LP home to torque
		else
			call HOME_TO_TORQ
		end	
	elseif (iHomingMethod(iAxisNum) = 1)&(iType(iAxisNum)<6000)
		if(iType(iAxisNum)>3000)&(iType(iAxisNum)<5000)
			CALL HOME_TO_SWITCH_CAR
		elseif(iType(iAxisNum) = 5008 | iType(iAxisNum) = 5009 | iType(iAxisNum) = 5010 | iType(iAxisNum) = 5018 | iType(iAxisNum) = 5019 | iType(iAxisNum) = 5020 | iType(iAxisNum) = 5028 | iType(iAxisNum) = 5029 | iType(iAxisNum) = 5030)
			call HOME_TO_SWITCH_CARZ
			FDEF(iAxisNum).#LL=1       ! Enable the iAxis left limit default response
			FDEF(iAxisNum).#RL=1       ! Enable the iAxis right limit default response
		else	
			CALL HOME_TO_SWITCH
			FDEF(iAxisNum).#LL=1       ! Enable the iAxis left limit default response
		end
	elseif (iHomingMethod(iAxisNum) = 2)
		CALL SET_CURRENTPOS_HOME
	end
end
iControlProcessState(iAxisNum) = 10
if iNeedSLL = 1
	FMASK(iAxisNum).#SLL = 1
end

if iNeedSRL = 1
	FMASK(iAxisNum).#SRL = 1
end
ENDOFHOMING:
STOP

!******************************************************************************* 
!   The following routine move the motor away from limit switches 

Limit_Check:
if MERR(iAxisNum) & MERR(iAxisNum)<>5010 & MERR(iAxisNum)<>5011; SP_Fail_Code=1; DISABLE(iAxisNum); DCOM(iAxisNum)=0; goto Finish; end
!   if MERR(iAxisNum); SP_Fail_Code=1; DISABLE(iAxisNum); DCOM(iAxisNum)=0; goto Finish; end 
if (FAULT(iAxisNum).#LL)|(FAULT(iAxisNum).#RL)
   if FAULT(iAxisNum).#LL; SP_Direction=1; else SP_Direction=-1; end
   ptp/rv (iAxisNum), SP_Direction*SP_Max_Search, SP_Search_Vel 
   till ((^FAULT(iAxisNum).#LL)&(^FAULT(iAxisNum).#RL))|(^AST(iAxisNum).#MOVE)
   if (FAULT(iAxisNum).#LL)|(FAULT(iAxisNum).#RL); SP_Fail_Code=4; DISABLE(iAxisNum); DCOM(iAxisNum)=0; goto Finish; end 
   kill(iAxisNum);  wait SP_Settle_Time; goto Move_Detent
end
ret

ON ((SP_InCommutationStartup = 1) & ((FAULT(iAxisNum)&0x30f80)>0 | (S_FAULT&0x30000000)>0))
SP_Fail_Code=1; DISABLE(iAxisNum); DCOM(iAxisNum)=0
CALL Finish 
RET



HOME_TO_TORQ:

MFLAGS(iAxisNum).#HOME = 0
FDEF(iAxisNum).#LL=0       ! Disable the iAxis left limit default response
FDEF(iAxisNum).#RL=0       ! Disable the iAxis right limit default response
ENABLE (iAxisNum)          ! Enable the iAxis drive
wait(1000)
JOG/v (iAxisNum), dHomingVel(iAxisNum)*dAxisPitch(iAxisNum)           ! Move to the left  
wait(1000)
KILL iAxisNum
! Reduce the motor current saturations
store_CURI = XCURI(iAxisNum)
store_CURV = XCURV(iAxisNum)
XCURI(iAxisNum) = dHomingTorq(iAxisNum)
XCURV(iAxisNum) = dHomingTorq(iAxisNum)

CERRI(iAxisNum)=0.01*dHomingTorq(iAxisNum)*CERRI(iAxisNum)
CERRV(iAxisNum)=0.01*dHomingTorq(iAxisNum)*CERRV(iAxisNum)

! Turn off the CPE response
FDEF(iAxisNum).#CPE = 0
! Jog until critical position error is triggered
JOG/v (iAxisNum), -dHomingVel(iAxisNum)*dAxisPitch(iAxisNum)           ! Move to the left  
TILL (FAULT(iAxisNum).#CPE = 1)   			! Wait for the critical PE
Kill iAxisNum
! Stop motor, let settle, set FPOS and TPOS to 0.0.  Then turn CPE mask back off
SET FPOS(iAxisNum) = 0.0
PTP/e (iAxisNum), 0.0
REAL deltaI, deltaV
deltaI = store_CURI - dHomingTorq(iAxisNum)
deltaV = store_CURV - dHomingTorq(iAxisNum)
LOOP 100
	XCURI(iAxisNum) = XCURI(iAxisNum) + 0.01*(deltaI)
	XCURV(iAxisNum) = XCURV(iAxisNum) + 0.01*(deltaV)
	SET FPOS(iAxisNum) = 0
	wait 20	!10ms x 100 = 1 sec ramp
END
!In case of rounding error, lastly set XCUR* to stored value - should be nearly identical at this point
XCURI(iAxisNum) = store_CURI
XCURV(iAxisNum) = store_CURV

CERRV(iAxisNum) = store_CERRV0
CERRI(iAxisNum) = store_CERRI0
FDEF(iAxisNum).#CPE = 1

BREAK iAxisNum
SET FPOS(iAxisNum) = 0 -dHomeOffset(iAxisNum)*dAxisPitch(iAxisNum)
PTP/ev (iAxisNum), 0.0, dHomingVel(iAxisNum)*dAxisPitch(iAxisNum)

!After homing, change Sp to 0.0 to prevent follow-on move.
OldSP(iAxisNum)=0
if 		iAxisNum = 0								
	Sp0 = 0.0
elseif 	iAxisNum = 1							
	Sp1 = 0.0
elseif 	iAxisNum = 2						
	Sp2 = 0.0
elseif 	iAxisNum = 3						
	Sp3 = 0.0
elseif 	iAxisNum = 4						
	Sp4 = 0.0
elseif 	iAxisNum = 5						
	Sp5 = 0.0
elseif 	iAxisNum = 6						
	Sp6 = 0.0
elseif 	iAxisNum = 7						
	Sp7 = 0.0
end

FDEF(iAxisNum).#CPE = 1
MFLAGS(iAxisNum).#HOME = 1
wait 500
RET


HOME_TO_SWITCH:

MFLAGS(iAxisNum).#HOME = 0
FDEF(iAxisNum).#LL=0       ! Disable the iAxis left limit default response
FDEF(iAxisNum).#RL=0       ! Disable the iAxis right limit default response
ENABLE (iAxisNum)          ! Enable the iAxis drive
wait(1000)

if  (IN(0).iAxisNum = 1) 
	JOG/v (iAxisNum), dHomingVel(iAxisNum)*dAxisPitch(iAxisNum)           ! Move to the left  
	TILL (IN(0).iAxisNum = 0)   			! Wait for home switch input
	! Stop motor, let settle, set FPOS and TPOS to 0.0.  Then turn CPE mask back off
	KILL (iAxisNum)
end
! Reduce the motor current saturations
store_CURI = XCURI(iAxisNum)
store_CURV = XCURV(iAxisNum)
XCURI(iAxisNum) = dHomingTorq(iAxisNum)
XCURV(iAxisNum) = dHomingTorq(iAxisNum)
! Do NOT Turn off the CPE response for home to switch
!FDEF(iAxisNum).#CPE = 0
! Jog until critical position error is triggered
JOG/v (iAxisNum), -dHomingVel(iAxisNum)*dAxisPitch(iAxisNum)           ! Move to the left  
   			! Wait for home switch input
TILL (IN(0).iAxisNum = 1)
! Stop motor, let settle, set FPOS and TPOS to 0.0.  Then turn CPE mask back off
KILL (iAxisNum)
wait 3000
SET FPOS(iAxisNum) = 0	!@CR2015-12-31 modified home position to simple offset - why move to index?
TPOS(iAxisNum) = 0
! FDEF(iAxisNum).#CPE = 1	!no need to turn turn CPE on because it was not turned off.
! disable motor to return to running torque, and immediately re-enable. This prevents an abrupt move with torque increase.

LOOP 100
	XCURI(iAxisNum) = XCURI(iAxisNum) + 0.01*(store_CURI - dHomingTorq(iAxisNum))
	wait 10	!10ms x 100 = 1 sec ramp
	XCURV(iAxisNum) = XCURV(iAxisNum) + 0.01*(store_CURV - dHomingTorq(iAxisNum))
	wait 10	!10ms x 100 = 1 sec ramp
END
! In case of rounding error, lastly set XCUR* to stored value - should be nearly identical at this point
XCURI(iAxisNum) = store_CURI
XCURV(iAxisNum) = store_CURV

!disable (iAxisNum)
!XCURI(iAxisNum) = store_CURI !@@@ moved here from end of if-thens
!XCURV(iAxisNum) = store_CURV	!@@@ moved here from end of if-thens
!enable (iAxisNum)
wait 1000
PTP/ev (iAxisNum), dHomeOffset(iAxisNum)*dAxisPitch(iAxisNum), dHomingVel(iAxisNum)*dAxisPitch(iAxisNum)
wait 3000
! After moving to home offset, re-zero the axis.  This is now the "working zero" for the axis
SET FPOS(iAxisNum) = 0	!@CR2015-12-31 modified home position to simple offset - why move to index?
TPOS(iAxisNum) = 0
! perform another move to zero (because without, there is residual FPOS data that shows at GUI - looks funny
PTP/ev (iAxisNum), 0.0, dHomingVel(iAxisNum)*dAxisPitch(iAxisNum)
wait 2000

!After homing, change Sp to 0.0 to prevent follow-on move.
OldSP(iAxisNum)=0
if 		iAxisNum = 0								
	Sp0 = 0.0
elseif 	iAxisNum = 1							
	Sp1 = 0.0
elseif 	iAxisNum = 2						
	Sp2 = 0.0
elseif 	iAxisNum = 3						
	Sp3 = 0.0
elseif 	iAxisNum = 4						
	Sp4 = 0.0
elseif 	iAxisNum = 5						
	Sp5 = 0.0
elseif 	iAxisNum = 6						
	Sp6 = 0.0
elseif 	iAxisNum = 7						
	Sp7 = 0.0
end

FDEF(iAxisNum).#CPE = 1
MFLAGS(iAxisNum).#HOME = 1
wait 500
RET







	
HOME_TO_SWITCH_CARZ:

MFLAGS(iAxisNum).#HOME = 0
FDEF(iAxisNum).#LL=0       ! Disable the iAxis left limit default response
FDEF(iAxisNum).#RL=0       ! Disable the iAxis right limit default response
ENABLE (iAxisNum)          ! Enable the iAxis drive
wait(1000)

if  (SAFIN(iAxisNum).#LL = 1) 
	JOG/v (iAxisNum), dHomingVel(iAxisNum)*dAxisPitch(iAxisNum)           ! Move to the left  
	TILL (SAFIN(iAxisNum).#LL  = 0)   			! Wait for home switch input
	! Stop motor, let settle, set FPOS and TPOS to 0.0.  Then turn CPE mask back off
	KILL (iAxisNum)
end
! Reduce the motor current saturations
store_CURI = XCURI(iAxisNum)
store_CURV = XCURV(iAxisNum)
XCURI(iAxisNum) = dHomingTorq(iAxisNum)
XCURV(iAxisNum) = dHomingTorq(iAxisNum)
! Do NOT Turn off the CPE response for home to switch
!FDEF(iAxisNum).#CPE = 0
! Jog until critical position error is triggered
JOG/v (iAxisNum), -dHomingVel(iAxisNum)*dAxisPitch(iAxisNum)           ! Move to the left  
   			! Wait for home switch input
TILL (SAFIN(iAxisNum).#LL = 1)
! Stop motor, let settle, set FPOS and TPOS to 0.0.  Then turn CPE mask back off
KILL (iAxisNum)
wait 3000

! @JR 2021-10-18 - Move down until upper limit switch opens
JOG/v (iAxisNum), dHomingVel(iAxisNum)*dAxisPitch(iAxisNum)
TILL (SAFIN(iAxisNum).#LL = 0)
KILL (iAxisNum)
wait 3000

SET FPOS(iAxisNum) = 0	!@CR2015-12-31 modified home position to simple offset - why move to index?
TPOS(iAxisNum) = 0
! FDEF(iAxisNum).#CPE = 1	!no need to turn turn CPE on because it was not turned off.
! disable motor to return to running torque, and immediately re-enable. This prevents an abrupt move with torque increase.

!LOOP 100
!	XCURI(iAxisNum) = XCURI(iAxisNum) + 0.01*(store_CURI - dHomingTorq(iAxisNum))
!	wait 10	!10ms x 100 = 1 sec ramp
!	XCURV(iAxisNum) = XCURV(iAxisNum) + 0.01*(store_CURV - dHomingTorq(iAxisNum))
!	wait 10	!10ms x 100 = 1 sec ramp
!END
! In case of rounding error, lastly set XCUR* to stored value - should be nearly identical at this point
XCURI(iAxisNum) = store_CURI
XCURV(iAxisNum) = store_CURV

!disable (iAxisNum)
!XCURI(iAxisNum) = store_CURI !@@@ moved here from end of if-thens
!XCURV(iAxisNum) = store_CURV	!@@@ moved here from end of if-thens
!enable (iAxisNum)
wait 1000
PTP/ev (iAxisNum), dHomeOffset(iAxisNum)*dAxisPitch(iAxisNum), dHomingVel(iAxisNum)*dAxisPitch(iAxisNum)
wait 3000
! After moving to home offset, re-zero the axis.  This is now the "working zero" for the axis
SET FPOS(iAxisNum) = 0	!@CR2015-12-31 modified home position to simple offset - why move to index?
TPOS(iAxisNum) = 0
! perform another move to zero (because without, there is residual FPOS data that shows at GUI - looks funny
PTP/ev (iAxisNum), 0.0, dHomingVel(iAxisNum)*dAxisPitch(iAxisNum)
wait 2000

!After homing, change Sp to 0.0 to prevent follow-on move.
OldSP(iAxisNum)=0
if 		iAxisNum = 0								
	Sp0 = 0.0
elseif 	iAxisNum = 1							
	Sp1 = 0.0
elseif 	iAxisNum = 2						
	Sp2 = 0.0
elseif 	iAxisNum = 3						
	Sp3 = 0.0
elseif 	iAxisNum = 4						
	Sp4 = 0.0
elseif 	iAxisNum = 5						
	Sp5 = 0.0
elseif 	iAxisNum = 6						
	Sp6 = 0.0
elseif 	iAxisNum = 7						
	Sp7 = 0.0
end

FDEF(iAxisNum).#CPE = 1
MFLAGS(iAxisNum).#HOME = 1
wait 500
RET

	
	
	
	
	

SET_CURRENTPOS_HOME:

SET FPOS(iAxisNum) = 0
TPOS(iAxisNum) = 0	!@@@ added
FDEF(iAxisNum).#CPE = 1
MFLAGS(iAxisNum).#HOME = 1
ENABLE (iAxisNum) 
! Move to 0 to clean-up residual FPOS error on GUI
PTP/ev (iAxisNum), 0, 180
wait 2000
!After homing, change Sp to 0.0 to prevent follow-on move.
OldSP(iAxisNum)=0
if 		iAxisNum = 0								
	Sp0 = 0.0
elseif 	iAxisNum = 1							
	Sp1 = 0.0
elseif 	iAxisNum = 2						
	Sp2 = 0.0
elseif 	iAxisNum = 3						
	Sp3 = 0.0
elseif 	iAxisNum = 4						
	Sp4 = 0.0
elseif 	iAxisNum = 5						
	Sp5 = 0.0
elseif 	iAxisNum = 6						
	Sp6 = 0.0
elseif 	iAxisNum = 7						
	Sp7 = 0.0
end
RET


HOME_TO_TORQ_AVP:


MFLAGS(iAxisNum).#HOME = 0
FDEF(iAxisNum).#LL=0       ! Disable the iAxis left limit default response
FDEF(iAxisNum).#RL=0       ! Disable the iAxis right limit default response
ENABLE (iAxisNum)          ! Enable the iAxis drive
wait(100)
JOG/v (iAxisNum), dHomingVel(iAxisNum)*dAxisPitch(iAxisNum)           ! Move to the left  
wait(1000)
KILL iAxisNum
!reduce the motor current saturations
store_CURI = XCURI(iAxisNum)
store_CURV = XCURV(iAxisNum)

real XCURI_actural, HmCurrent_Of_Drive, FtCurrent_Of_Drive
HmCurrent_Of_Drive = dHomingTorq(iAxisNum)*(4*XRMS(iAxisNum))/100.0
FtCurrent_Of_Drive = dFaultTorq(iAxisNum)*(4*XRMS(iAxisNum))/100.0

XCURI(iAxisNum) = HmCurrent_Of_Drive
XCURV(iAxisNum) = HmCurrent_Of_Drive
!CERRI(iAxisNum)=0.01*dHomingTorq(iAxisNum)*CERRI(iAxisNum)
!CERRV(iAxisNum)=0.01*dHomingTorq(iAxisNum)*CERRV(iAxisNum)
! Turn off the CPE response
FDEF(iAxisNum).#CPE = 0

JOG/v (iAxisNum), -dHomingVel(iAxisNum)*dAxisPitch(iAxisNum)           ! Move to the left  
TILL (FAULT(iAxisNum).#CPE = 1)   			! Wait for the critical PE
KILL iAxisNum

if (dAxisPitch(iAxisNum)>0.01)
	dCE(iAxisNum)=FPOS(iAxisNum)/dAxisPitch(iAxisNum)+dHomeOffset(iAxisNum)
end
block
SET FPOS(iAxisNum) = 0.0 -dHomeOffset(iAxisNum)*dAxisPitch(iAxisNum)
PTP/e (iAxisNum), 100!0.0
end



deltaI = FtCurrent_Of_Drive - HmCurrent_Of_Drive
deltaV = FtCurrent_Of_Drive - HmCurrent_Of_Drive
LOOP 100
	XCURI(iAxisNum) = XCURI(iAxisNum) + 0.01*(deltaI)
	XCURV(iAxisNum) = XCURV(iAxisNum) + 0.01*(deltaV)
	wait 20	!10ms x 100 = 1 sec ramp
END

XCURI(iAxisNum) = FtCurrent_Of_Drive !@@@ moved here from end of if-thens
XCURV(iAxisNum) = FtCurrent_Of_Drive	!@@@ moved here from end of if-thens
CERRV(iAxisNum) = store_CERRV0
CERRI(iAxisNum) = store_CERRI0
FDEF(iAxisNum).#CPE = 1

BREAK iAxisNum
!SET FPOS(iAxisNum) = 0.0 -dHomeOffset(iAxisNum)*dAxisPitch(iAxisNum)	!@CR2015-12-31 modified home position to simple offset - why move to index?
PTP/ev (iAxisNum), 0.0, dHomingVel(iAxisNum)*dAxisPitch(iAxisNum)


!After homing, change Sp to 0.0 to prevent follow-on move.
OldSP(iAxisNum)=0
if 		iAxisNum = 0								
	Sp0 = 0.0
elseif 	iAxisNum = 1							
	Sp1 = 0.0
elseif 	iAxisNum = 2						
	Sp2 = 0.0
elseif 	iAxisNum = 3						
	Sp3 = 0.0
elseif 	iAxisNum = 4						
	Sp4 = 0.0
elseif 	iAxisNum = 5						
	Sp5 = 0.0
elseif 	iAxisNum = 6						
	Sp6 = 0.0
elseif 	iAxisNum = 7						
	Sp7 = 0.0
end
FDEF(iAxisNum).#CPE = 1
MFLAGS(iAxisNum).#HOME = 1
RET

HOME_TO_SWITCH_CAR:
REAL rMax, rMin
INT iCARAIChannel
iCARAIChannel = 0

MFLAGS(iAxisNum).#HOME = 0
FDEF(iAxisNum).#LL=0       ! Disable the iAxis left limit default response
FDEF(iAxisNum).#RL=0       ! Disable the iAxis right limit default response
ENABLE (iAxisNum)          ! Enable the iAxis drive
wait(1000)
! Jog until critical position error is triggered
if (FtdCarHomeSW.iAxisNum = 1)
	!JOG/v (iAxisNum), dHomingVel(iAxisNum)           ! Move to the left  
	!TILL (FtdCarHomeSW(iAxisNum) = 0)   			! Wait for home switch input
	!BREAK (iAxisNum)
	PTP/er (iAxisNum), 15
end

if(iType(iAxisNum)=4007)	! if it is an analog sensor, perform sensor calibration first.
! find which analog channel is the home sensor at

	if AnalogCARAxis(0)=iAxisNum
		iCARAIChannel = 0
	elseif AnalogCARAxis(1)=iAxisNum
		iCARAIChannel = 1
	elseif AnalogCARAxis(2)=iAxisNum
		iCARAIChannel = 2
	elseif AnalogCARAxis(3)=iAxisNum
		iCARAIChannel = 3
	end
! scan the analog signal and acq 1000 points from the analog signal while spinning
	DC/S iAxisNum,Datalog,1000,100,FPOS(iAxisNum),AIN(iCARAIChannel)
	JOG/v (iAxisNum), 4 
	till (^AST(iAxisNum).#DC)	!wait DAQ to finish
	KILL iAxisNum
	WAIT 2000
! find the median in analog signal	
	rMax = MAX(Datalog, 1, 1, 0,999)
	rMin = MIN(Datalog, 1, 1, 0,999)
	if (rMax-rMin)>1.0
		AnologHomeSignalMedian(iCARAIChannel) = (rMax + rMin)/2.0
	else
		disable(iAxisNum)  
		iControlProcessState(iAxisNum) = 404
		RET
	end	
! find the position where is the peak signal
	int thepeakposIndex
	thepeakposIndex = MAXI(Datalog, 1, 1, 0,999)
! move the CAR to the neighbourhood of the peak	
	PTP/e (iAxisNum), Datalog(0)(thepeakposIndex)+15
	WAIT 3000
end





real FirstEdge, SecondEdge
!Find the leading edge of the home sensor
JOG/v (iAxisNum), -dHomingVel(iAxisNum)           ! Move to the left  
TILL (FtdCarHomeSW.iAxisNum = 1)   			! Wait for home switch input
FirstEdge = FPOS(iAxisNum)
KILL iAxisNum
WAIT 2000
!back off
PTP/e (iAxisNum), FirstEdge + 10
!low speed homing
JOG/v (iAxisNum), -dHomingVel(iAxisNum)           ! Move to the left  
TILL (FtdCarHomeSW.iAxisNum = 1)   			! Wait for home switch input
FirstEdge = FPOS(iAxisNum)
KILL iAxisNum
WAIT 2000
PTP/e (iAxisNum), FirstEdge - 15
!low speed homing
JOG/v (iAxisNum), dHomingVel(iAxisNum)           ! Move to the left  
TILL (FtdCarHomeSW.iAxisNum = 1) 
SecondEdge = FPOS(iAxisNum)
KILL iAxisNum
WAIT 2000
PTP/ev (iAxisNum), 0.5*(FirstEdge+SecondEdge)+dHomeOffset(iAxisNum), dHomingVel(iAxisNum)
wait 3000

SET FPOS(iAxisNum) = 0	!@CR2015-12-31 modified home position to simple offset - why move to index?
TPOS(iAxisNum) = 0
! After moving to home offset, re-zero the axis.  This is now the "working zero" for the axis
SET FPOS(iAxisNum) = 0	!@CR2015-12-31 modified home position to simple offset - why move to index?
TPOS(iAxisNum) = 0
! perform another move to zero (because without, there is residual FPOS data that shows at GUI - looks funny
FDEF(iAxisNum).#CPE = 1
MFLAGS(iAxisNum).#HOME = 1
wait 500

PTP/ev (iAxisNum), 0.0, dHomingVel(iAxisNum)
wait 2000

!After homing, change Sp to 0.0 to prevent follow-on move.
OldSP(iAxisNum)=0
if 		iAxisNum = 0								
	Sp0 = 0.0
elseif 	iAxisNum = 1							
	Sp1 = 0.0
elseif 	iAxisNum = 2						
	Sp2 = 0.0
elseif 	iAxisNum = 3						
	Sp3 = 0.0
elseif 	iAxisNum = 4						
	Sp4 = 0.0
elseif 	iAxisNum = 5						
	Sp5 = 0.0
elseif 	iAxisNum = 6						
	Sp6 = 0.0
elseif 	iAxisNum = 7						
	Sp7 = 0.0
end

RET
#6
!#/ Controller version = 2.27
!#/ Date = 12/28/2015 10:33 AM
!#/ User remarks = 
! motor cummt and axis homing routine for all types except for robot arms

!Change log
! DATE		ECO#	Reason for Change			Remarks
!9/21/2015	N/A		ini release					ZW
!12/28/15	N/A		tuning homing				ZW
!10/16/17	N/A		turn on CPE before move offset, also link CPE to %home torque
!11/17/2017 Disconnet the AVP CPE link to home torques which cause not homing
Global INT	iLl_Interlock_Cmd	!iWord10Bit1			:= doutputs.,
Global INT	iSt_Interlock_Cmd	!iWord10Bit5			:= doutputs.,
Global INT	iCar_Interlock_Cmd	!iWord10Bit9			:= doutputs.,
Global INT  bAxisInterlocked

GLOBAL real OldSP(8), Datalog(2)(1000)
global int AnalogCARAxis(4)
global real AnologHomeSignalMedian(4)
real SP_Direction                 ! Search direction 
real SP_Settle_Time               ! Settling time at Detent Points [msec]
real SP_Search_Vel                ! Search velocity [user-units/sec]
real SP_Drive                     ! Actuating drive command [% of maximum]
real SP_Max_Search                ! Maximum search distance [user-units]
real SP_Init_Offset               ! Initial commutation offset [elec. degrees]
real PrevAcc, PrevDec, PrevKDec, PrevJerk, PrevVel
real minAnalog, maxAnalog
int iNeedSLL, iNeedSRL
real store_CURI, store_CURV
int iAxisNum
real store_CERRI0, store_CERRV0

iAxisNum = 6	!set the particular axis for the buffer
If ((iType(iAxisNum)= 4008) & (iCar_Interlock_Cmd = 0)) | ((iType(iAxisNum)= 5011) & (iLl_Interlock_Cmd = 0)) | ((iType(iAxisNum)= 5012) & (iSt_Interlock_Cmd = 0)) | ((iInterlockEnabled(iAxisNum)=1)&(bAxisInterlocked=1))
	iControlProcessState(iAxisNum) = 113
	GOTO ENDOFHOMING
end

iControlProcessState(iAxisNum) = 14
CERRI(iAxisNum)=4.39
CERRV(iAxisNum)=4.39
store_CERRI0 = CERRI(iAxisNum)
store_CERRV0 = CERRV(iAxisNum)
MFLAGS(iAxisNum).#HOME = 0			
iNeedSLL=0
iNeedSRL=0

if FMASK(iAxisNum).#SLL = 1
	FMASK(iAxisNum).#SLL = 0
	iNeedSLL=1
end

if FMASK(iAxisNum).#SRL = 1
	FMASK(iAxisNum).#SRL = 0
	iNeedSRL=1
end

if MFLAGS(iAxisNum).9=0                ! not commutation

	SP_Drive = 5.475          
	SP_Settle_Time = 1000    
	SP_Search_Vel  = 6         
	SP_Init_Offset = 0
	SP_Direction =  -1
	SP_Max_Search = 144
	PrevAcc = ACC(iAxisNum)
	PrevDec = DEC(iAxisNum)
	PrevKDec=KDEC(iAxisNum)
	PrevJerk=JERK(iAxisNum)

	!   Output varibale:
	int  SP_Fail_Code                 ! Faiure Code of the startup program

	! Auxiliary variable:
	real SP_Pitch                     ! Magnetic pitch [180 elec.deg. in user units]

	! State Flag
	int  SP_InCommutationStartup      ! Flag indicating commutation startup is in progress


	SP_Pitch=SLCPRD(iAxisNum)/SLCNP(iAxisNum)*EFAC(iAxisNum)


	!******************************************************************************* 
	! INITIALIZE
	FCLEAR(iAxisNum)   
	disable(iAxisNum)                     
	SETCONF(216,(iAxisNum),0)                ! Reset commutation state
	setconf(214,(iAxisNum),SP_Init_Offset)   ! Set initial commutation phase
	SP_Fail_Code=0                        ! Reset failure
	SP_Direction=1                        ! Move in positive direction
	SP_InCommutationStartup=1             ! commutation startup in progress
	!******************************************************************************* 
	! SET MOTION PROFILE FOR COMMUTATION STARTUP PROCESS 
	!
	!   WARNING: The following are the suggested motion parameters for the startup
	!     process. Check that the values are suitable for your application.

	ACC(iAxisNum)=SP_Search_Vel*10.; DEC(iAxisNum)=SP_Search_Vel*10. 
	KDEC(iAxisNum)=SP_Search_Vel*50.; JERK(iAxisNum)=SP_Search_Vel*100.

	!******************************************************************************* 
	! STEP 1 - MOVE TO FIRST DETENT POINT 
	!
	! WARNING: The motor moves to a detent point by jump. 
	!   The jump distance is up to one magnetic pitch in any direction.
	!   The motor jumps to the closest detent point within its motion range.
	!   If necessary modify initial detent point by changing the variable 
	!     SP_Init_Offset between 0-360 electrical degrees.  
	enable(iAxisNum)
	wait(100)
	while (DCOM(iAxisNum)+0.05 < SP_Drive); DCOM(iAxisNum) = DCOM(iAxisNum) + 0.05; end
	DCOM(iAxisNum) = SP_Drive
	wait SP_Settle_Time
	call Limit_Check
	!******************************************************************************* 
	! STEP 2 - MOVE TO SECOND DETENT POINT
	!
	!   The program moves the motor 90 electrical degrees in order to eliminate
	!      a state of unstable equilibrium.

	Move_Detent:
	ptp/rv (iAxisNum), SP_Direction*SP_Pitch/2.,SP_Search_Vel
	till ^AST(iAxisNum).#MOVE; wait SP_Settle_Time
	call Limit_Check
	disable(iAxisNum)

	MFLAGS(iAxisNum).9=1               ! Set commutation state 
	DCOM(iAxisNum) = 0

	! If motor is to be left enabled after startup process delete the following line:

	Finish:
	SP_InCommutationStartup=0              ! commutation startup is finished
	If SP_Fail_Code=0; disp "   Commutation Startup Finished."
	else disp "   Commutation Startup Failed."; disp "   Failure Code = %i",SP_Fail_Code; end

	ACC(iAxisNum)=PrevAcc
	DEC(iAxisNum)=PrevDec
	KDEC(iAxisNum)=PrevKDec
	JERK(iAxisNum)=PrevJerk

end

!at this moment, the axis shall commuted. If it is, then start homing

if MFLAGS(iAxisNum).9=1
	if (iHomingMethod(iAxisNum) = 0)		!0-home to torque; 1-homing to switch; 2-manual set home;
	    if (iType(iAxisNum)>1000)&(iType(iAxisNum)<2000) 	
		!AVP home to torques - always
			call HOME_TO_TORQ_AVP	!different than LP home to torque
		else
			call HOME_TO_TORQ
		end	
	elseif (iHomingMethod(iAxisNum) = 1)&(iType(iAxisNum)<6000)
		if(iType(iAxisNum)>3000)&(iType(iAxisNum)<5000)
			CALL HOME_TO_SWITCH_CAR
		elseif(iType(iAxisNum) = 5008 | iType(iAxisNum) = 5009 | iType(iAxisNum) = 5010 | iType(iAxisNum) = 5018 | iType(iAxisNum) = 5019 | iType(iAxisNum) = 5020 | iType(iAxisNum) = 5028 | iType(iAxisNum) = 5029 | iType(iAxisNum) = 5030)
			call HOME_TO_SWITCH_CARZ
			FDEF(iAxisNum).#LL=1       ! Enable the iAxis left limit default response
			FDEF(iAxisNum).#RL=1       ! Enable the iAxis right limit default response
		else	
			CALL HOME_TO_SWITCH
			FDEF(iAxisNum).#LL=1       ! Enable the iAxis left limit default response
		end
	elseif (iHomingMethod(iAxisNum) = 2)
		CALL SET_CURRENTPOS_HOME
	end
end
iControlProcessState(iAxisNum) = 10
if iNeedSLL = 1
	FMASK(iAxisNum).#SLL = 1
end

if iNeedSRL = 1
	FMASK(iAxisNum).#SRL = 1
end
ENDOFHOMING:
STOP

!******************************************************************************* 
!   The following routine move the motor away from limit switches 

Limit_Check:
if MERR(iAxisNum) & MERR(iAxisNum)<>5010 & MERR(iAxisNum)<>5011; SP_Fail_Code=1; DISABLE(iAxisNum); DCOM(iAxisNum)=0; goto Finish; end
!   if MERR(iAxisNum); SP_Fail_Code=1; DISABLE(iAxisNum); DCOM(iAxisNum)=0; goto Finish; end 
if (FAULT(iAxisNum).#LL)|(FAULT(iAxisNum).#RL)
   if FAULT(iAxisNum).#LL; SP_Direction=1; else SP_Direction=-1; end
   ptp/rv (iAxisNum), SP_Direction*SP_Max_Search, SP_Search_Vel 
   till ((^FAULT(iAxisNum).#LL)&(^FAULT(iAxisNum).#RL))|(^AST(iAxisNum).#MOVE)
   if (FAULT(iAxisNum).#LL)|(FAULT(iAxisNum).#RL); SP_Fail_Code=4; DISABLE(iAxisNum); DCOM(iAxisNum)=0; goto Finish; end 
   kill(iAxisNum);  wait SP_Settle_Time; goto Move_Detent
end
ret

ON ((SP_InCommutationStartup = 1) & ((FAULT(iAxisNum)&0x30f80)>0 | (S_FAULT&0x30000000)>0))
SP_Fail_Code=1; DISABLE(iAxisNum); DCOM(iAxisNum)=0
CALL Finish 
RET



HOME_TO_TORQ:

MFLAGS(iAxisNum).#HOME = 0
FDEF(iAxisNum).#LL=0       ! Disable the iAxis left limit default response
FDEF(iAxisNum).#RL=0       ! Disable the iAxis right limit default response
ENABLE (iAxisNum)          ! Enable the iAxis drive
wait(1000)
JOG/v (iAxisNum), dHomingVel(iAxisNum)*dAxisPitch(iAxisNum)           ! Move to the left  
wait(1000)
KILL iAxisNum
! Reduce the motor current saturations
store_CURI = XCURI(iAxisNum)
store_CURV = XCURV(iAxisNum)
XCURI(iAxisNum) = dHomingTorq(iAxisNum)
XCURV(iAxisNum) = dHomingTorq(iAxisNum)

CERRI(iAxisNum)=0.01*dHomingTorq(iAxisNum)*CERRI(iAxisNum)
CERRV(iAxisNum)=0.01*dHomingTorq(iAxisNum)*CERRV(iAxisNum)

! Turn off the CPE response
FDEF(iAxisNum).#CPE = 0
! Jog until critical position error is triggered
JOG/v (iAxisNum), -dHomingVel(iAxisNum)*dAxisPitch(iAxisNum)           ! Move to the left  
TILL (FAULT(iAxisNum).#CPE = 1)   			! Wait for the critical PE
Kill iAxisNum
! Stop motor, let settle, set FPOS and TPOS to 0.0.  Then turn CPE mask back off
SET FPOS(iAxisNum) = 0.0
PTP/e (iAxisNum), 0.0
REAL deltaI, deltaV
deltaI = store_CURI - dHomingTorq(iAxisNum)
deltaV = store_CURV - dHomingTorq(iAxisNum)
LOOP 100
	XCURI(iAxisNum) = XCURI(iAxisNum) + 0.01*(deltaI)
	XCURV(iAxisNum) = XCURV(iAxisNum) + 0.01*(deltaV)
	SET FPOS(iAxisNum) = 0
	wait 20	!10ms x 100 = 1 sec ramp
END
!In case of rounding error, lastly set XCUR* to stored value - should be nearly identical at this point
XCURI(iAxisNum) = store_CURI
XCURV(iAxisNum) = store_CURV

CERRV(iAxisNum) = store_CERRV0
CERRI(iAxisNum) = store_CERRI0
FDEF(iAxisNum).#CPE = 1

BREAK iAxisNum
SET FPOS(iAxisNum) = 0 -dHomeOffset(iAxisNum)*dAxisPitch(iAxisNum)
PTP/ev (iAxisNum), 0.0, dHomingVel(iAxisNum)*dAxisPitch(iAxisNum)

!After homing, change Sp to 0.0 to prevent follow-on move.
OldSP(iAxisNum)=0
if 		iAxisNum = 0								
	Sp0 = 0.0
elseif 	iAxisNum = 1							
	Sp1 = 0.0
elseif 	iAxisNum = 2						
	Sp2 = 0.0
elseif 	iAxisNum = 3						
	Sp3 = 0.0
elseif 	iAxisNum = 4						
	Sp4 = 0.0
elseif 	iAxisNum = 5						
	Sp5 = 0.0
elseif 	iAxisNum = 6						
	Sp6 = 0.0
elseif 	iAxisNum = 7						
	Sp7 = 0.0
end

FDEF(iAxisNum).#CPE = 1
MFLAGS(iAxisNum).#HOME = 1
wait 500
RET


HOME_TO_SWITCH:

MFLAGS(iAxisNum).#HOME = 0
FDEF(iAxisNum).#LL=0       ! Disable the iAxis left limit default response
FDEF(iAxisNum).#RL=0       ! Disable the iAxis right limit default response
ENABLE (iAxisNum)          ! Enable the iAxis drive
wait(1000)

if  (IN(0).iAxisNum = 1) 
	JOG/v (iAxisNum), dHomingVel(iAxisNum)*dAxisPitch(iAxisNum)           ! Move to the left  
	TILL (IN(0).iAxisNum = 0)   			! Wait for home switch input
	! Stop motor, let settle, set FPOS and TPOS to 0.0.  Then turn CPE mask back off
	KILL (iAxisNum)
end
! Reduce the motor current saturations
store_CURI = XCURI(iAxisNum)
store_CURV = XCURV(iAxisNum)
XCURI(iAxisNum) = dHomingTorq(iAxisNum)
XCURV(iAxisNum) = dHomingTorq(iAxisNum)
! Do NOT Turn off the CPE response for home to switch
!FDEF(iAxisNum).#CPE = 0
! Jog until critical position error is triggered
JOG/v (iAxisNum), -dHomingVel(iAxisNum)*dAxisPitch(iAxisNum)           ! Move to the left  
   			! Wait for home switch input
TILL (IN(0).iAxisNum = 1)
! Stop motor, let settle, set FPOS and TPOS to 0.0.  Then turn CPE mask back off
KILL (iAxisNum)
wait 3000
SET FPOS(iAxisNum) = 0	!@CR2015-12-31 modified home position to simple offset - why move to index?
TPOS(iAxisNum) = 0
! FDEF(iAxisNum).#CPE = 1	!no need to turn turn CPE on because it was not turned off.
! disable motor to return to running torque, and immediately re-enable. This prevents an abrupt move with torque increase.

LOOP 100
	XCURI(iAxisNum) = XCURI(iAxisNum) + 0.01*(store_CURI - dHomingTorq(iAxisNum))
	wait 10	!10ms x 100 = 1 sec ramp
	XCURV(iAxisNum) = XCURV(iAxisNum) + 0.01*(store_CURV - dHomingTorq(iAxisNum))
	wait 10	!10ms x 100 = 1 sec ramp
END
! In case of rounding error, lastly set XCUR* to stored value - should be nearly identical at this point
XCURI(iAxisNum) = store_CURI
XCURV(iAxisNum) = store_CURV

!disable (iAxisNum)
!XCURI(iAxisNum) = store_CURI !@@@ moved here from end of if-thens
!XCURV(iAxisNum) = store_CURV	!@@@ moved here from end of if-thens
!enable (iAxisNum)
wait 1000
PTP/ev (iAxisNum), dHomeOffset(iAxisNum)*dAxisPitch(iAxisNum), dHomingVel(iAxisNum)*dAxisPitch(iAxisNum)
wait 3000
! After moving to home offset, re-zero the axis.  This is now the "working zero" for the axis
SET FPOS(iAxisNum) = 0	!@CR2015-12-31 modified home position to simple offset - why move to index?
TPOS(iAxisNum) = 0
! perform another move to zero (because without, there is residual FPOS data that shows at GUI - looks funny
PTP/ev (iAxisNum), 0.0, dHomingVel(iAxisNum)*dAxisPitch(iAxisNum)
wait 2000

!After homing, change Sp to 0.0 to prevent follow-on move.
OldSP(iAxisNum)=0
if 		iAxisNum = 0								
	Sp0 = 0.0
elseif 	iAxisNum = 1							
	Sp1 = 0.0
elseif 	iAxisNum = 2						
	Sp2 = 0.0
elseif 	iAxisNum = 3						
	Sp3 = 0.0
elseif 	iAxisNum = 4						
	Sp4 = 0.0
elseif 	iAxisNum = 5						
	Sp5 = 0.0
elseif 	iAxisNum = 6						
	Sp6 = 0.0
elseif 	iAxisNum = 7						
	Sp7 = 0.0
end

FDEF(iAxisNum).#CPE = 1
MFLAGS(iAxisNum).#HOME = 1
wait 500
RET







	
HOME_TO_SWITCH_CARZ:

MFLAGS(iAxisNum).#HOME = 0
FDEF(iAxisNum).#LL=0       ! Disable the iAxis left limit default response
FDEF(iAxisNum).#RL=0       ! Disable the iAxis right limit default response
ENABLE (iAxisNum)          ! Enable the iAxis drive
wait(1000)

if  (SAFIN(iAxisNum).#LL = 1) 
	JOG/v (iAxisNum), dHomingVel(iAxisNum)*dAxisPitch(iAxisNum)           ! Move to the left  
	TILL (SAFIN(iAxisNum).#LL  = 0)   			! Wait for home switch input
	! Stop motor, let settle, set FPOS and TPOS to 0.0.  Then turn CPE mask back off
	KILL (iAxisNum)
end
! Reduce the motor current saturations
store_CURI = XCURI(iAxisNum)
store_CURV = XCURV(iAxisNum)
XCURI(iAxisNum) = dHomingTorq(iAxisNum)
XCURV(iAxisNum) = dHomingTorq(iAxisNum)
! Do NOT Turn off the CPE response for home to switch
!FDEF(iAxisNum).#CPE = 0
! Jog until critical position error is triggered
JOG/v (iAxisNum), -dHomingVel(iAxisNum)*dAxisPitch(iAxisNum)           ! Move to the left  
   			! Wait for home switch input
TILL (SAFIN(iAxisNum).#LL = 1)
! Stop motor, let settle, set FPOS and TPOS to 0.0.  Then turn CPE mask back off
KILL (iAxisNum)
wait 3000

! @JR 2021-10-18 - Move down until upper limit switch opens
JOG/v (iAxisNum), dHomingVel(iAxisNum)*dAxisPitch(iAxisNum)
TILL (SAFIN(iAxisNum).#LL = 0)
KILL (iAxisNum)
wait 3000

SET FPOS(iAxisNum) = 0	!@CR2015-12-31 modified home position to simple offset - why move to index?
TPOS(iAxisNum) = 0
! FDEF(iAxisNum).#CPE = 1	!no need to turn turn CPE on because it was not turned off.
! disable motor to return to running torque, and immediately re-enable. This prevents an abrupt move with torque increase.

!LOOP 100
!	XCURI(iAxisNum) = XCURI(iAxisNum) + 0.01*(store_CURI - dHomingTorq(iAxisNum))
!	wait 10	!10ms x 100 = 1 sec ramp
!	XCURV(iAxisNum) = XCURV(iAxisNum) + 0.01*(store_CURV - dHomingTorq(iAxisNum))
!	wait 10	!10ms x 100 = 1 sec ramp
!END
! In case of rounding error, lastly set XCUR* to stored value - should be nearly identical at this point
XCURI(iAxisNum) = store_CURI
XCURV(iAxisNum) = store_CURV

!disable (iAxisNum)
!XCURI(iAxisNum) = store_CURI !@@@ moved here from end of if-thens
!XCURV(iAxisNum) = store_CURV	!@@@ moved here from end of if-thens
!enable (iAxisNum)
wait 1000
PTP/ev (iAxisNum), dHomeOffset(iAxisNum)*dAxisPitch(iAxisNum), dHomingVel(iAxisNum)*dAxisPitch(iAxisNum)
wait 3000
! After moving to home offset, re-zero the axis.  This is now the "working zero" for the axis
SET FPOS(iAxisNum) = 0	!@CR2015-12-31 modified home position to simple offset - why move to index?
TPOS(iAxisNum) = 0
! perform another move to zero (because without, there is residual FPOS data that shows at GUI - looks funny
PTP/ev (iAxisNum), 0.0, dHomingVel(iAxisNum)*dAxisPitch(iAxisNum)
wait 2000

!After homing, change Sp to 0.0 to prevent follow-on move.
OldSP(iAxisNum)=0
if 		iAxisNum = 0								
	Sp0 = 0.0
elseif 	iAxisNum = 1							
	Sp1 = 0.0
elseif 	iAxisNum = 2						
	Sp2 = 0.0
elseif 	iAxisNum = 3						
	Sp3 = 0.0
elseif 	iAxisNum = 4						
	Sp4 = 0.0
elseif 	iAxisNum = 5						
	Sp5 = 0.0
elseif 	iAxisNum = 6						
	Sp6 = 0.0
elseif 	iAxisNum = 7						
	Sp7 = 0.0
end

FDEF(iAxisNum).#CPE = 1
MFLAGS(iAxisNum).#HOME = 1
wait 500
RET

	
	
	
	
	

SET_CURRENTPOS_HOME:

SET FPOS(iAxisNum) = 0
TPOS(iAxisNum) = 0	!@@@ added
FDEF(iAxisNum).#CPE = 1
MFLAGS(iAxisNum).#HOME = 1
ENABLE (iAxisNum) 
! Move to 0 to clean-up residual FPOS error on GUI
PTP/ev (iAxisNum), 0, 180
wait 2000
!After homing, change Sp to 0.0 to prevent follow-on move.
OldSP(iAxisNum)=0
if 		iAxisNum = 0								
	Sp0 = 0.0
elseif 	iAxisNum = 1							
	Sp1 = 0.0
elseif 	iAxisNum = 2						
	Sp2 = 0.0
elseif 	iAxisNum = 3						
	Sp3 = 0.0
elseif 	iAxisNum = 4						
	Sp4 = 0.0
elseif 	iAxisNum = 5						
	Sp5 = 0.0
elseif 	iAxisNum = 6						
	Sp6 = 0.0
elseif 	iAxisNum = 7						
	Sp7 = 0.0
end
RET


HOME_TO_TORQ_AVP:


MFLAGS(iAxisNum).#HOME = 0
FDEF(iAxisNum).#LL=0       ! Disable the iAxis left limit default response
FDEF(iAxisNum).#RL=0       ! Disable the iAxis right limit default response
ENABLE (iAxisNum)          ! Enable the iAxis drive
wait(100)
JOG/v (iAxisNum), dHomingVel(iAxisNum)*dAxisPitch(iAxisNum)           ! Move to the left  
wait(1000)
KILL iAxisNum
!reduce the motor current saturations
store_CURI = XCURI(iAxisNum)
store_CURV = XCURV(iAxisNum)

real XCURI_actural, HmCurrent_Of_Drive, FtCurrent_Of_Drive
HmCurrent_Of_Drive = dHomingTorq(iAxisNum)*(4*XRMS(iAxisNum))/100.0
FtCurrent_Of_Drive = dFaultTorq(iAxisNum)*(4*XRMS(iAxisNum))/100.0

XCURI(iAxisNum) = HmCurrent_Of_Drive
XCURV(iAxisNum) = HmCurrent_Of_Drive
!CERRI(iAxisNum)=0.01*dHomingTorq(iAxisNum)*CERRI(iAxisNum)
!CERRV(iAxisNum)=0.01*dHomingTorq(iAxisNum)*CERRV(iAxisNum)
! Turn off the CPE response
FDEF(iAxisNum).#CPE = 0

JOG/v (iAxisNum), -dHomingVel(iAxisNum)*dAxisPitch(iAxisNum)           ! Move to the left  
TILL (FAULT(iAxisNum).#CPE = 1)   			! Wait for the critical PE
KILL iAxisNum

if (dAxisPitch(iAxisNum)>0.01)
	dCE(iAxisNum)=FPOS(iAxisNum)/dAxisPitch(iAxisNum)+dHomeOffset(iAxisNum)
end
block
SET FPOS(iAxisNum) = 0.0 -dHomeOffset(iAxisNum)*dAxisPitch(iAxisNum)
PTP/e (iAxisNum), 100!0.0
end



deltaI = FtCurrent_Of_Drive - HmCurrent_Of_Drive
deltaV = FtCurrent_Of_Drive - HmCurrent_Of_Drive
LOOP 100
	XCURI(iAxisNum) = XCURI(iAxisNum) + 0.01*(deltaI)
	XCURV(iAxisNum) = XCURV(iAxisNum) + 0.01*(deltaV)
	wait 20	!10ms x 100 = 1 sec ramp
END

XCURI(iAxisNum) = FtCurrent_Of_Drive !@@@ moved here from end of if-thens
XCURV(iAxisNum) = FtCurrent_Of_Drive	!@@@ moved here from end of if-thens
CERRV(iAxisNum) = store_CERRV0
CERRI(iAxisNum) = store_CERRI0
FDEF(iAxisNum).#CPE = 1

BREAK iAxisNum
!SET FPOS(iAxisNum) = 0.0 -dHomeOffset(iAxisNum)*dAxisPitch(iAxisNum)	!@CR2015-12-31 modified home position to simple offset - why move to index?
PTP/ev (iAxisNum), 0.0, dHomingVel(iAxisNum)*dAxisPitch(iAxisNum)


!After homing, change Sp to 0.0 to prevent follow-on move.
OldSP(iAxisNum)=0
if 		iAxisNum = 0								
	Sp0 = 0.0
elseif 	iAxisNum = 1							
	Sp1 = 0.0
elseif 	iAxisNum = 2						
	Sp2 = 0.0
elseif 	iAxisNum = 3						
	Sp3 = 0.0
elseif 	iAxisNum = 4						
	Sp4 = 0.0
elseif 	iAxisNum = 5						
	Sp5 = 0.0
elseif 	iAxisNum = 6						
	Sp6 = 0.0
elseif 	iAxisNum = 7						
	Sp7 = 0.0
end
FDEF(iAxisNum).#CPE = 1
MFLAGS(iAxisNum).#HOME = 1
RET

HOME_TO_SWITCH_CAR:
REAL rMax, rMin
INT iCARAIChannel
iCARAIChannel = 0

MFLAGS(iAxisNum).#HOME = 0
FDEF(iAxisNum).#LL=0       ! Disable the iAxis left limit default response
FDEF(iAxisNum).#RL=0       ! Disable the iAxis right limit default response
ENABLE (iAxisNum)          ! Enable the iAxis drive
wait(1000)
! Jog until critical position error is triggered
if (FtdCarHomeSW.iAxisNum = 1)
	!JOG/v (iAxisNum), dHomingVel(iAxisNum)           ! Move to the left  
	!TILL (FtdCarHomeSW(iAxisNum) = 0)   			! Wait for home switch input
	!BREAK (iAxisNum)
	PTP/er (iAxisNum), 15
end

if(iType(iAxisNum)=4007)	! if it is an analog sensor, perform sensor calibration first.
! find which analog channel is the home sensor at

	if AnalogCARAxis(0)=iAxisNum
		iCARAIChannel = 0
	elseif AnalogCARAxis(1)=iAxisNum
		iCARAIChannel = 1
	elseif AnalogCARAxis(2)=iAxisNum
		iCARAIChannel = 2
	elseif AnalogCARAxis(3)=iAxisNum
		iCARAIChannel = 3
	end
! scan the analog signal and acq 1000 points from the analog signal while spinning
	DC/S iAxisNum,Datalog,1000,100,FPOS(iAxisNum),AIN(iCARAIChannel)
	JOG/v (iAxisNum), 4 
	till (^AST(iAxisNum).#DC)	!wait DAQ to finish
	KILL iAxisNum
	WAIT 2000
! find the median in analog signal	
	rMax = MAX(Datalog, 1, 1, 0,999)
	rMin = MIN(Datalog, 1, 1, 0,999)
	if (rMax-rMin)>1.0
		AnologHomeSignalMedian(iCARAIChannel) = (rMax + rMin)/2.0
	else
		disable(iAxisNum)  
		iControlProcessState(iAxisNum) = 404
		RET
	end	
! find the position where is the peak signal
	int thepeakposIndex
	thepeakposIndex = MAXI(Datalog, 1, 1, 0,999)
! move the CAR to the neighbourhood of the peak	
	PTP/e (iAxisNum), Datalog(0)(thepeakposIndex)+15
	WAIT 3000
end





real FirstEdge, SecondEdge
!Find the leading edge of the home sensor
JOG/v (iAxisNum), -dHomingVel(iAxisNum)           ! Move to the left  
TILL (FtdCarHomeSW.iAxisNum = 1)   			! Wait for home switch input
FirstEdge = FPOS(iAxisNum)
KILL iAxisNum
WAIT 2000
!back off
PTP/e (iAxisNum), FirstEdge + 10
!low speed homing
JOG/v (iAxisNum), -dHomingVel(iAxisNum)           ! Move to the left  
TILL (FtdCarHomeSW.iAxisNum = 1)   			! Wait for home switch input
FirstEdge = FPOS(iAxisNum)
KILL iAxisNum
WAIT 2000
PTP/e (iAxisNum), FirstEdge - 15
!low speed homing
JOG/v (iAxisNum), dHomingVel(iAxisNum)           ! Move to the left  
TILL (FtdCarHomeSW.iAxisNum = 1) 
SecondEdge = FPOS(iAxisNum)
KILL iAxisNum
WAIT 2000
PTP/ev (iAxisNum), 0.5*(FirstEdge+SecondEdge)+dHomeOffset(iAxisNum), dHomingVel(iAxisNum)
wait 3000

SET FPOS(iAxisNum) = 0	!@CR2015-12-31 modified home position to simple offset - why move to index?
TPOS(iAxisNum) = 0
! After moving to home offset, re-zero the axis.  This is now the "working zero" for the axis
SET FPOS(iAxisNum) = 0	!@CR2015-12-31 modified home position to simple offset - why move to index?
TPOS(iAxisNum) = 0
! perform another move to zero (because without, there is residual FPOS data that shows at GUI - looks funny
FDEF(iAxisNum).#CPE = 1
MFLAGS(iAxisNum).#HOME = 1
wait 500

PTP/ev (iAxisNum), 0.0, dHomingVel(iAxisNum)
wait 2000

!After homing, change Sp to 0.0 to prevent follow-on move.
OldSP(iAxisNum)=0
if 		iAxisNum = 0								
	Sp0 = 0.0
elseif 	iAxisNum = 1							
	Sp1 = 0.0
elseif 	iAxisNum = 2						
	Sp2 = 0.0
elseif 	iAxisNum = 3						
	Sp3 = 0.0
elseif 	iAxisNum = 4						
	Sp4 = 0.0
elseif 	iAxisNum = 5						
	Sp5 = 0.0
elseif 	iAxisNum = 6						
	Sp6 = 0.0
elseif 	iAxisNum = 7						
	Sp7 = 0.0
end

RET
#7
!#/ Controller version = 2.27
!#/ Date = 12/28/2015 10:33 AM
!#/ User remarks = 
! motor cummt and axis homing routine for all types except for robot arms

!Change log
! DATE		ECO#	Reason for Change			Remarks
!9/21/2015	N/A		ini release					ZW
!12/28/15	N/A		tuning homing				ZW
!10/16/17	N/A		turn on CPE before move offset, also link CPE to %home torque
!11/17/2017 Disconnet the AVP CPE link to home torques which cause not homing
Global INT	iLl_Interlock_Cmd	!iWord10Bit1			:= doutputs.,
Global INT	iSt_Interlock_Cmd	!iWord10Bit5			:= doutputs.,
Global INT	iCar_Interlock_Cmd	!iWord10Bit9			:= doutputs.,
Global INT  bAxisInterlocked

GLOBAL real OldSP(8), Datalog(2)(1000)
global int AnalogCARAxis(4)
global real AnologHomeSignalMedian(4)
real SP_Direction                 ! Search direction 
real SP_Settle_Time               ! Settling time at Detent Points [msec]
real SP_Search_Vel                ! Search velocity [user-units/sec]
real SP_Drive                     ! Actuating drive command [% of maximum]
real SP_Max_Search                ! Maximum search distance [user-units]
real SP_Init_Offset               ! Initial commutation offset [elec. degrees]
real PrevAcc, PrevDec, PrevKDec, PrevJerk, PrevVel
real minAnalog, maxAnalog
int iNeedSLL, iNeedSRL
real store_CURI, store_CURV
int iAxisNum
real store_CERRI0, store_CERRV0

iAxisNum=7	!set the particular axis for the buffer
If ((iType(iAxisNum)= 4008) & (iCar_Interlock_Cmd = 0)) | ((iType(iAxisNum)= 5011) & (iLl_Interlock_Cmd = 0)) | ((iType(iAxisNum)= 5012) & (iSt_Interlock_Cmd = 0)) | ((iInterlockEnabled(iAxisNum)=1)&(bAxisInterlocked=1))
	iControlProcessState(iAxisNum) = 113
	GOTO ENDOFHOMING
end

iControlProcessState(iAxisNum) = 14
CERRI(iAxisNum)=4.39
CERRV(iAxisNum)=4.39
store_CERRI0 = CERRI(iAxisNum)
store_CERRV0 = CERRV(iAxisNum)
MFLAGS(iAxisNum).#HOME = 0			
iNeedSLL=0
iNeedSRL=0

if FMASK(iAxisNum).#SLL = 1
	FMASK(iAxisNum).#SLL = 0
	iNeedSLL=1
end

if FMASK(iAxisNum).#SRL = 1
	FMASK(iAxisNum).#SRL = 0
	iNeedSRL=1
end

if MFLAGS(iAxisNum).9=0                ! not commutation

	SP_Drive = 5.475          
	SP_Settle_Time = 1000    
	SP_Search_Vel  = 6         
	SP_Init_Offset = 0
	SP_Direction =  -1
	SP_Max_Search = 144
	PrevAcc = ACC(iAxisNum)
	PrevDec = DEC(iAxisNum)
	PrevKDec=KDEC(iAxisNum)
	PrevJerk=JERK(iAxisNum)

	!   Output varibale:
	int  SP_Fail_Code                 ! Faiure Code of the startup program

	! Auxiliary variable:
	real SP_Pitch                     ! Magnetic pitch [180 elec.deg. in user units]

	! State Flag
	int  SP_InCommutationStartup      ! Flag indicating commutation startup is in progress


	SP_Pitch=SLCPRD(iAxisNum)/SLCNP(iAxisNum)*EFAC(iAxisNum)


	!******************************************************************************* 
	! INITIALIZE
	FCLEAR(iAxisNum)   
	disable(iAxisNum)                     
	SETCONF(216,(iAxisNum),0)                ! Reset commutation state
	setconf(214,(iAxisNum),SP_Init_Offset)   ! Set initial commutation phase
	SP_Fail_Code=0                        ! Reset failure
	SP_Direction=1                        ! Move in positive direction
	SP_InCommutationStartup=1             ! commutation startup in progress
	!******************************************************************************* 
	! SET MOTION PROFILE FOR COMMUTATION STARTUP PROCESS 
	!
	!   WARNING: The following are the suggested motion parameters for the startup
	!     process. Check that the values are suitable for your application.

	ACC(iAxisNum)=SP_Search_Vel*10.; DEC(iAxisNum)=SP_Search_Vel*10. 
	KDEC(iAxisNum)=SP_Search_Vel*50.; JERK(iAxisNum)=SP_Search_Vel*100.

	!******************************************************************************* 
	! STEP 1 - MOVE TO FIRST DETENT POINT 
	!
	! WARNING: The motor moves to a detent point by jump. 
	!   The jump distance is up to one magnetic pitch in any direction.
	!   The motor jumps to the closest detent point within its motion range.
	!   If necessary modify initial detent point by changing the variable 
	!     SP_Init_Offset between 0-360 electrical degrees.  
	enable(iAxisNum)
	wait(100)
	while (DCOM(iAxisNum)+0.05 < SP_Drive); DCOM(iAxisNum) = DCOM(iAxisNum) + 0.05; end
	DCOM(iAxisNum) = SP_Drive
	wait SP_Settle_Time
	call Limit_Check
	!******************************************************************************* 
	! STEP 2 - MOVE TO SECOND DETENT POINT
	!
	!   The program moves the motor 90 electrical degrees in order to eliminate
	!      a state of unstable equilibrium.

	Move_Detent:
	ptp/rv (iAxisNum), SP_Direction*SP_Pitch/2.,SP_Search_Vel
	till ^AST(iAxisNum).#MOVE; wait SP_Settle_Time
	call Limit_Check
	disable(iAxisNum)

	MFLAGS(iAxisNum).9=1               ! Set commutation state 
	DCOM(iAxisNum) = 0

	! If motor is to be left enabled after startup process delete the following line:

	Finish:
	SP_InCommutationStartup=0              ! commutation startup is finished
	If SP_Fail_Code=0; disp "   Commutation Startup Finished."
	else disp "   Commutation Startup Failed."; disp "   Failure Code = %i",SP_Fail_Code; end

	ACC(iAxisNum)=PrevAcc
	DEC(iAxisNum)=PrevDec
	KDEC(iAxisNum)=PrevKDec
	JERK(iAxisNum)=PrevJerk

end

!at this moment, the axis shall commuted. If it is, then start homing

if MFLAGS(iAxisNum).9=1
	if (iHomingMethod(iAxisNum) = 0)		!0-home to torque; 1-homing to switch; 2-manual set home;
	    if (iType(iAxisNum)>1000)&(iType(iAxisNum)<2000) 	
		!AVP home to torques - always
			call HOME_TO_TORQ_AVP	!different than LP home to torque
		else
			call HOME_TO_TORQ
		end	
	elseif (iHomingMethod(iAxisNum) = 1)&(iType(iAxisNum)<6000)
		if(iType(iAxisNum)>3000)&(iType(iAxisNum)<5000)
			CALL HOME_TO_SWITCH_CAR
		elseif(iType(iAxisNum) = 5008 | iType(iAxisNum) = 5009 | iType(iAxisNum) = 5010 | iType(iAxisNum) = 5018 | iType(iAxisNum) = 5019 | iType(iAxisNum) = 5020 | iType(iAxisNum) = 5028 | iType(iAxisNum) = 5029 | iType(iAxisNum) = 5030)
			call HOME_TO_SWITCH_CARZ
			FDEF(iAxisNum).#LL=1       ! Enable the iAxis left limit default response
			FDEF(iAxisNum).#RL=1       ! Enable the iAxis right limit default response
		else	
			CALL HOME_TO_SWITCH
			FDEF(iAxisNum).#LL=1       ! Enable the iAxis left limit default response
		end
	elseif (iHomingMethod(iAxisNum) = 2)
		CALL SET_CURRENTPOS_HOME
	end
end
iControlProcessState(iAxisNum) = 10
if iNeedSLL = 1
	FMASK(iAxisNum).#SLL = 1
end

if iNeedSRL = 1
	FMASK(iAxisNum).#SRL = 1
end
ENDOFHOMING:
STOP

!******************************************************************************* 
!   The following routine move the motor away from limit switches 

Limit_Check:
if MERR(iAxisNum) & MERR(iAxisNum)<>5010 & MERR(iAxisNum)<>5011; SP_Fail_Code=1; DISABLE(iAxisNum); DCOM(iAxisNum)=0; goto Finish; end
!   if MERR(iAxisNum); SP_Fail_Code=1; DISABLE(iAxisNum); DCOM(iAxisNum)=0; goto Finish; end 
if (FAULT(iAxisNum).#LL)|(FAULT(iAxisNum).#RL)
   if FAULT(iAxisNum).#LL; SP_Direction=1; else SP_Direction=-1; end
   ptp/rv (iAxisNum), SP_Direction*SP_Max_Search, SP_Search_Vel 
   till ((^FAULT(iAxisNum).#LL)&(^FAULT(iAxisNum).#RL))|(^AST(iAxisNum).#MOVE)
   if (FAULT(iAxisNum).#LL)|(FAULT(iAxisNum).#RL); SP_Fail_Code=4; DISABLE(iAxisNum); DCOM(iAxisNum)=0; goto Finish; end 
   kill(iAxisNum);  wait SP_Settle_Time; goto Move_Detent
end
ret

ON ((SP_InCommutationStartup = 1) & ((FAULT(iAxisNum)&0x30f80)>0 | (S_FAULT&0x30000000)>0))
SP_Fail_Code=1; DISABLE(iAxisNum); DCOM(iAxisNum)=0
CALL Finish 
RET



HOME_TO_TORQ:

MFLAGS(iAxisNum).#HOME = 0
FDEF(iAxisNum).#LL=0       ! Disable the iAxis left limit default response
FDEF(iAxisNum).#RL=0       ! Disable the iAxis right limit default response
ENABLE (iAxisNum)          ! Enable the iAxis drive
wait(1000)
JOG/v (iAxisNum), dHomingVel(iAxisNum)*dAxisPitch(iAxisNum)           ! Move to the left  
wait(1000)
KILL iAxisNum
! Reduce the motor current saturations
store_CURI = XCURI(iAxisNum)
store_CURV = XCURV(iAxisNum)
XCURI(iAxisNum) = dHomingTorq(iAxisNum)
XCURV(iAxisNum) = dHomingTorq(iAxisNum)

CERRI(iAxisNum)=0.01*dHomingTorq(iAxisNum)*CERRI(iAxisNum)
CERRV(iAxisNum)=0.01*dHomingTorq(iAxisNum)*CERRV(iAxisNum)

! Turn off the CPE response
FDEF(iAxisNum).#CPE = 0
! Jog until critical position error is triggered
JOG/v (iAxisNum), -dHomingVel(iAxisNum)*dAxisPitch(iAxisNum)           ! Move to the left  
TILL (FAULT(iAxisNum).#CPE = 1)   			! Wait for the critical PE
Kill iAxisNum
! Stop motor, let settle, set FPOS and TPOS to 0.0.  Then turn CPE mask back off
SET FPOS(iAxisNum) = 0.0
PTP/e (iAxisNum), 0.0
REAL deltaI, deltaV
deltaI = store_CURI - dHomingTorq(iAxisNum)
deltaV = store_CURV - dHomingTorq(iAxisNum)
LOOP 100
	XCURI(iAxisNum) = XCURI(iAxisNum) + 0.01*(deltaI)
	XCURV(iAxisNum) = XCURV(iAxisNum) + 0.01*(deltaV)
	SET FPOS(iAxisNum) = 0
	wait 20	!10ms x 100 = 1 sec ramp
END
!In case of rounding error, lastly set XCUR* to stored value - should be nearly identical at this point
XCURI(iAxisNum) = store_CURI
XCURV(iAxisNum) = store_CURV

CERRV(iAxisNum) = store_CERRV0
CERRI(iAxisNum) = store_CERRI0
FDEF(iAxisNum).#CPE = 1

BREAK iAxisNum
SET FPOS(iAxisNum) = 0 -dHomeOffset(iAxisNum)*dAxisPitch(iAxisNum)
PTP/ev (iAxisNum), 0.0, dHomingVel(iAxisNum)*dAxisPitch(iAxisNum)

!After homing, change Sp to 0.0 to prevent follow-on move.
OldSP(iAxisNum)=0
if 		iAxisNum = 0								
	Sp0 = 0.0
elseif 	iAxisNum = 1							
	Sp1 = 0.0
elseif 	iAxisNum = 2						
	Sp2 = 0.0
elseif 	iAxisNum = 3						
	Sp3 = 0.0
elseif 	iAxisNum = 4						
	Sp4 = 0.0
elseif 	iAxisNum = 5						
	Sp5 = 0.0
elseif 	iAxisNum = 6						
	Sp6 = 0.0
elseif 	iAxisNum = 7						
	Sp7 = 0.0
end

FDEF(iAxisNum).#CPE = 1
MFLAGS(iAxisNum).#HOME = 1
wait 500
RET


HOME_TO_SWITCH:

MFLAGS(iAxisNum).#HOME = 0
FDEF(iAxisNum).#LL=0       ! Disable the iAxis left limit default response
FDEF(iAxisNum).#RL=0       ! Disable the iAxis right limit default response
ENABLE (iAxisNum)          ! Enable the iAxis drive
wait(1000)

if  (IN(0).iAxisNum = 1) 
	JOG/v (iAxisNum), dHomingVel(iAxisNum)*dAxisPitch(iAxisNum)           ! Move to the left  
	TILL (IN(0).iAxisNum = 0)   			! Wait for home switch input
	! Stop motor, let settle, set FPOS and TPOS to 0.0.  Then turn CPE mask back off
	KILL (iAxisNum)
end
! Reduce the motor current saturations
store_CURI = XCURI(iAxisNum)
store_CURV = XCURV(iAxisNum)
XCURI(iAxisNum) = dHomingTorq(iAxisNum)
XCURV(iAxisNum) = dHomingTorq(iAxisNum)
! Do NOT Turn off the CPE response for home to switch
!FDEF(iAxisNum).#CPE = 0
! Jog until critical position error is triggered
JOG/v (iAxisNum), -dHomingVel(iAxisNum)*dAxisPitch(iAxisNum)           ! Move to the left  
   			! Wait for home switch input
TILL (IN(0).iAxisNum = 1)
! Stop motor, let settle, set FPOS and TPOS to 0.0.  Then turn CPE mask back off
KILL (iAxisNum)
wait 3000
SET FPOS(iAxisNum) = 0	!@CR2015-12-31 modified home position to simple offset - why move to index?
TPOS(iAxisNum) = 0
! FDEF(iAxisNum).#CPE = 1	!no need to turn turn CPE on because it was not turned off.
! disable motor to return to running torque, and immediately re-enable. This prevents an abrupt move with torque increase.

LOOP 100
	XCURI(iAxisNum) = XCURI(iAxisNum) + 0.01*(store_CURI - dHomingTorq(iAxisNum))
	wait 10	!10ms x 100 = 1 sec ramp
	XCURV(iAxisNum) = XCURV(iAxisNum) + 0.01*(store_CURV - dHomingTorq(iAxisNum))
	wait 10	!10ms x 100 = 1 sec ramp
END
! In case of rounding error, lastly set XCUR* to stored value - should be nearly identical at this point
XCURI(iAxisNum) = store_CURI
XCURV(iAxisNum) = store_CURV

!disable (iAxisNum)
!XCURI(iAxisNum) = store_CURI !@@@ moved here from end of if-thens
!XCURV(iAxisNum) = store_CURV	!@@@ moved here from end of if-thens
!enable (iAxisNum)
wait 1000
PTP/ev (iAxisNum), dHomeOffset(iAxisNum)*dAxisPitch(iAxisNum), dHomingVel(iAxisNum)*dAxisPitch(iAxisNum)
wait 3000
! After moving to home offset, re-zero the axis.  This is now the "working zero" for the axis
SET FPOS(iAxisNum) = 0	!@CR2015-12-31 modified home position to simple offset - why move to index?
TPOS(iAxisNum) = 0
! perform another move to zero (because without, there is residual FPOS data that shows at GUI - looks funny
PTP/ev (iAxisNum), 0.0, dHomingVel(iAxisNum)*dAxisPitch(iAxisNum)
wait 2000

!After homing, change Sp to 0.0 to prevent follow-on move.
OldSP(iAxisNum)=0
if 		iAxisNum = 0								
	Sp0 = 0.0
elseif 	iAxisNum = 1							
	Sp1 = 0.0
elseif 	iAxisNum = 2						
	Sp2 = 0.0
elseif 	iAxisNum = 3						
	Sp3 = 0.0
elseif 	iAxisNum = 4						
	Sp4 = 0.0
elseif 	iAxisNum = 5						
	Sp5 = 0.0
elseif 	iAxisNum = 6						
	Sp6 = 0.0
elseif 	iAxisNum = 7						
	Sp7 = 0.0
end

FDEF(iAxisNum).#CPE = 1
MFLAGS(iAxisNum).#HOME = 1
wait 500
RET







	
HOME_TO_SWITCH_CARZ:

MFLAGS(iAxisNum).#HOME = 0
FDEF(iAxisNum).#LL=0       ! Disable the iAxis left limit default response
FDEF(iAxisNum).#RL=0       ! Disable the iAxis right limit default response
ENABLE (iAxisNum)          ! Enable the iAxis drive
wait(1000)

if  (SAFIN(iAxisNum).#LL = 1) 
	JOG/v (iAxisNum), dHomingVel(iAxisNum)*dAxisPitch(iAxisNum)           ! Move to the left  
	TILL (SAFIN(iAxisNum).#LL  = 0)   			! Wait for home switch input
	! Stop motor, let settle, set FPOS and TPOS to 0.0.  Then turn CPE mask back off
	KILL (iAxisNum)
end
! Reduce the motor current saturations
store_CURI = XCURI(iAxisNum)
store_CURV = XCURV(iAxisNum)
XCURI(iAxisNum) = dHomingTorq(iAxisNum)
XCURV(iAxisNum) = dHomingTorq(iAxisNum)
! Do NOT Turn off the CPE response for home to switch
!FDEF(iAxisNum).#CPE = 0
! Jog until critical position error is triggered
JOG/v (iAxisNum), -dHomingVel(iAxisNum)*dAxisPitch(iAxisNum)           ! Move to the left  
   			! Wait for home switch input
TILL (SAFIN(iAxisNum).#LL = 1)
! Stop motor, let settle, set FPOS and TPOS to 0.0.  Then turn CPE mask back off
KILL (iAxisNum)
wait 3000

! @JR 2021-10-18 - Move down until upper limit switch opens
JOG/v (iAxisNum), dHomingVel(iAxisNum)*dAxisPitch(iAxisNum)
TILL (SAFIN(iAxisNum).#LL = 0)
KILL (iAxisNum)
wait 3000

SET FPOS(iAxisNum) = 0	!@CR2015-12-31 modified home position to simple offset - why move to index?
TPOS(iAxisNum) = 0
! FDEF(iAxisNum).#CPE = 1	!no need to turn turn CPE on because it was not turned off.
! disable motor to return to running torque, and immediately re-enable. This prevents an abrupt move with torque increase.

!LOOP 100
!	XCURI(iAxisNum) = XCURI(iAxisNum) + 0.01*(store_CURI - dHomingTorq(iAxisNum))
!	wait 10	!10ms x 100 = 1 sec ramp
!	XCURV(iAxisNum) = XCURV(iAxisNum) + 0.01*(store_CURV - dHomingTorq(iAxisNum))
!	wait 10	!10ms x 100 = 1 sec ramp
!END
! In case of rounding error, lastly set XCUR* to stored value - should be nearly identical at this point
XCURI(iAxisNum) = store_CURI
XCURV(iAxisNum) = store_CURV

!disable (iAxisNum)
!XCURI(iAxisNum) = store_CURI !@@@ moved here from end of if-thens
!XCURV(iAxisNum) = store_CURV	!@@@ moved here from end of if-thens
!enable (iAxisNum)
wait 1000
PTP/ev (iAxisNum), dHomeOffset(iAxisNum)*dAxisPitch(iAxisNum), dHomingVel(iAxisNum)*dAxisPitch(iAxisNum)
wait 3000
! After moving to home offset, re-zero the axis.  This is now the "working zero" for the axis
SET FPOS(iAxisNum) = 0	!@CR2015-12-31 modified home position to simple offset - why move to index?
TPOS(iAxisNum) = 0
! perform another move to zero (because without, there is residual FPOS data that shows at GUI - looks funny
PTP/ev (iAxisNum), 0.0, dHomingVel(iAxisNum)*dAxisPitch(iAxisNum)
wait 2000

!After homing, change Sp to 0.0 to prevent follow-on move.
OldSP(iAxisNum)=0
if 		iAxisNum = 0								
	Sp0 = 0.0
elseif 	iAxisNum = 1							
	Sp1 = 0.0
elseif 	iAxisNum = 2						
	Sp2 = 0.0
elseif 	iAxisNum = 3						
	Sp3 = 0.0
elseif 	iAxisNum = 4						
	Sp4 = 0.0
elseif 	iAxisNum = 5						
	Sp5 = 0.0
elseif 	iAxisNum = 6						
	Sp6 = 0.0
elseif 	iAxisNum = 7						
	Sp7 = 0.0
end

FDEF(iAxisNum).#CPE = 1
MFLAGS(iAxisNum).#HOME = 1
wait 500
RET

	
	
	
	
	

SET_CURRENTPOS_HOME:

SET FPOS(iAxisNum) = 0
TPOS(iAxisNum) = 0	!@@@ added
FDEF(iAxisNum).#CPE = 1
MFLAGS(iAxisNum).#HOME = 1
ENABLE (iAxisNum) 
! Move to 0 to clean-up residual FPOS error on GUI
PTP/ev (iAxisNum), 0, 180
wait 2000
!After homing, change Sp to 0.0 to prevent follow-on move.
OldSP(iAxisNum)=0
if 		iAxisNum = 0								
	Sp0 = 0.0
elseif 	iAxisNum = 1							
	Sp1 = 0.0
elseif 	iAxisNum = 2						
	Sp2 = 0.0
elseif 	iAxisNum = 3						
	Sp3 = 0.0
elseif 	iAxisNum = 4						
	Sp4 = 0.0
elseif 	iAxisNum = 5						
	Sp5 = 0.0
elseif 	iAxisNum = 6						
	Sp6 = 0.0
elseif 	iAxisNum = 7						
	Sp7 = 0.0
end
RET


HOME_TO_TORQ_AVP:


MFLAGS(iAxisNum).#HOME = 0
FDEF(iAxisNum).#LL=0       ! Disable the iAxis left limit default response
FDEF(iAxisNum).#RL=0       ! Disable the iAxis right limit default response
ENABLE (iAxisNum)          ! Enable the iAxis drive
wait(100)
JOG/v (iAxisNum), dHomingVel(iAxisNum)*dAxisPitch(iAxisNum)           ! Move to the left  
wait(1000)
KILL iAxisNum
!reduce the motor current saturations
store_CURI = XCURI(iAxisNum)
store_CURV = XCURV(iAxisNum)

real XCURI_actural, HmCurrent_Of_Drive, FtCurrent_Of_Drive
HmCurrent_Of_Drive = dHomingTorq(iAxisNum)*(4*XRMS(iAxisNum))/100.0
FtCurrent_Of_Drive = dFaultTorq(iAxisNum)*(4*XRMS(iAxisNum))/100.0

XCURI(iAxisNum) = HmCurrent_Of_Drive
XCURV(iAxisNum) = HmCurrent_Of_Drive
!CERRI(iAxisNum)=0.01*dHomingTorq(iAxisNum)*CERRI(iAxisNum)
!CERRV(iAxisNum)=0.01*dHomingTorq(iAxisNum)*CERRV(iAxisNum)
! Turn off the CPE response
FDEF(iAxisNum).#CPE = 0

JOG/v (iAxisNum), -dHomingVel(iAxisNum)*dAxisPitch(iAxisNum)           ! Move to the left  
TILL (FAULT(iAxisNum).#CPE = 1)   			! Wait for the critical PE
KILL iAxisNum

if (dAxisPitch(iAxisNum)>0.01)
	dCE(iAxisNum)=FPOS(iAxisNum)/dAxisPitch(iAxisNum)+dHomeOffset(iAxisNum)
end
block
SET FPOS(iAxisNum) = 0.0 -dHomeOffset(iAxisNum)*dAxisPitch(iAxisNum)
PTP/e (iAxisNum), 100!0.0
end



deltaI = FtCurrent_Of_Drive - HmCurrent_Of_Drive
deltaV = FtCurrent_Of_Drive - HmCurrent_Of_Drive
LOOP 100
	XCURI(iAxisNum) = XCURI(iAxisNum) + 0.01*(deltaI)
	XCURV(iAxisNum) = XCURV(iAxisNum) + 0.01*(deltaV)
	wait 20	!10ms x 100 = 1 sec ramp
END

XCURI(iAxisNum) = FtCurrent_Of_Drive !@@@ moved here from end of if-thens
XCURV(iAxisNum) = FtCurrent_Of_Drive	!@@@ moved here from end of if-thens
CERRV(iAxisNum) = store_CERRV0
CERRI(iAxisNum) = store_CERRI0
FDEF(iAxisNum).#CPE = 1

BREAK iAxisNum
!SET FPOS(iAxisNum) = 0.0 -dHomeOffset(iAxisNum)*dAxisPitch(iAxisNum)	!@CR2015-12-31 modified home position to simple offset - why move to index?
PTP/ev (iAxisNum), 0.0, dHomingVel(iAxisNum)*dAxisPitch(iAxisNum)


!After homing, change Sp to 0.0 to prevent follow-on move.
OldSP(iAxisNum)=0
if 		iAxisNum = 0								
	Sp0 = 0.0
elseif 	iAxisNum = 1							
	Sp1 = 0.0
elseif 	iAxisNum = 2						
	Sp2 = 0.0
elseif 	iAxisNum = 3						
	Sp3 = 0.0
elseif 	iAxisNum = 4						
	Sp4 = 0.0
elseif 	iAxisNum = 5						
	Sp5 = 0.0
elseif 	iAxisNum = 6						
	Sp6 = 0.0
elseif 	iAxisNum = 7						
	Sp7 = 0.0
end
FDEF(iAxisNum).#CPE = 1
MFLAGS(iAxisNum).#HOME = 1
RET

HOME_TO_SWITCH_CAR:
REAL rMax, rMin
INT iCARAIChannel
iCARAIChannel = 0

MFLAGS(iAxisNum).#HOME = 0
FDEF(iAxisNum).#LL=0       ! Disable the iAxis left limit default response
FDEF(iAxisNum).#RL=0       ! Disable the iAxis right limit default response
ENABLE (iAxisNum)          ! Enable the iAxis drive
wait(1000)
! Jog until critical position error is triggered
if (FtdCarHomeSW.iAxisNum = 1)
	!JOG/v (iAxisNum), dHomingVel(iAxisNum)           ! Move to the left  
	!TILL (FtdCarHomeSW(iAxisNum) = 0)   			! Wait for home switch input
	!BREAK (iAxisNum)
	PTP/er (iAxisNum), 15
end

if(iType(iAxisNum)=4007)	! if it is an analog sensor, perform sensor calibration first.
! find which analog channel is the home sensor at

	if AnalogCARAxis(0)=iAxisNum
		iCARAIChannel = 0
	elseif AnalogCARAxis(1)=iAxisNum
		iCARAIChannel = 1
	elseif AnalogCARAxis(2)=iAxisNum
		iCARAIChannel = 2
	elseif AnalogCARAxis(3)=iAxisNum
		iCARAIChannel = 3
	end
! scan the analog signal and acq 1000 points from the analog signal while spinning
	DC/S iAxisNum,Datalog,1000,100,FPOS(iAxisNum),AIN(iCARAIChannel)
	JOG/v (iAxisNum), 4 
	till (^AST(iAxisNum).#DC)	!wait DAQ to finish
	KILL iAxisNum
	WAIT 2000
! find the median in analog signal	
	rMax = MAX(Datalog, 1, 1, 0,999)
	rMin = MIN(Datalog, 1, 1, 0,999)
	if (rMax-rMin)>1.0
		AnologHomeSignalMedian(iCARAIChannel) = (rMax + rMin)/2.0
	else
		disable(iAxisNum)  
		iControlProcessState(iAxisNum) = 404
		RET
	end	
! find the position where is the peak signal
	int thepeakposIndex
	thepeakposIndex = MAXI(Datalog, 1, 1, 0,999)
! move the CAR to the neighbourhood of the peak	
	PTP/e (iAxisNum), Datalog(0)(thepeakposIndex)+15
	WAIT 3000
end





real FirstEdge, SecondEdge
!Find the leading edge of the home sensor
JOG/v (iAxisNum), -dHomingVel(iAxisNum)           ! Move to the left  
TILL (FtdCarHomeSW.iAxisNum = 1)   			! Wait for home switch input
FirstEdge = FPOS(iAxisNum)
KILL iAxisNum
WAIT 2000
!back off
PTP/e (iAxisNum), FirstEdge + 10
!low speed homing
JOG/v (iAxisNum), -dHomingVel(iAxisNum)           ! Move to the left  
TILL (FtdCarHomeSW.iAxisNum = 1)   			! Wait for home switch input
FirstEdge = FPOS(iAxisNum)
KILL iAxisNum
WAIT 2000
PTP/e (iAxisNum), FirstEdge - 15
!low speed homing
JOG/v (iAxisNum), dHomingVel(iAxisNum)           ! Move to the left  
TILL (FtdCarHomeSW.iAxisNum = 1) 
SecondEdge = FPOS(iAxisNum)
KILL iAxisNum
WAIT 2000
PTP/ev (iAxisNum), 0.5*(FirstEdge+SecondEdge)+dHomeOffset(iAxisNum), dHomingVel(iAxisNum)
wait 3000

SET FPOS(iAxisNum) = 0	!@CR2015-12-31 modified home position to simple offset - why move to index?
TPOS(iAxisNum) = 0
! After moving to home offset, re-zero the axis.  This is now the "working zero" for the axis
SET FPOS(iAxisNum) = 0	!@CR2015-12-31 modified home position to simple offset - why move to index?
TPOS(iAxisNum) = 0
! perform another move to zero (because without, there is residual FPOS data that shows at GUI - looks funny
FDEF(iAxisNum).#CPE = 1
MFLAGS(iAxisNum).#HOME = 1
wait 500

PTP/ev (iAxisNum), 0.0, dHomingVel(iAxisNum)
wait 2000

!After homing, change Sp to 0.0 to prevent follow-on move.
OldSP(iAxisNum)=0
if 		iAxisNum = 0								
	Sp0 = 0.0
elseif 	iAxisNum = 1							
	Sp1 = 0.0
elseif 	iAxisNum = 2						
	Sp2 = 0.0
elseif 	iAxisNum = 3						
	Sp3 = 0.0
elseif 	iAxisNum = 4						
	Sp4 = 0.0
elseif 	iAxisNum = 5						
	Sp5 = 0.0
elseif 	iAxisNum = 6						
	Sp6 = 0.0
elseif 	iAxisNum = 7						
	Sp7 = 0.0
end

RET
#11

!Buffer 11 - This buffer communicates with the Meiden PLC (Keyence) over Modbus

!Declaration - READ/WRITE Meiden PLC
!READ
!Global      Int   K2M_POG_Ready, K2M_Feed_Ready, K2M_POG_Fault
!Global      Int   K2M_O3PressurePV  !For CBr4 type only 

!Modbus addresses
INT MODBUSADD_OFFSET
INT MODBUSADD_LC_MFC_SP					!8000
INT MODBUSADD_LC_BYPASSMODE_CMD			!8001 
INT MODBUSADD_OXYGEN_BYPASSMODE_CMD		!8002
INT MODBUSADD_OZONEPRESSURE_SP			!8003
INT MODBUSADD_OZONECHAMBERPRESSURE_SP	!8004
INT MODBUSADD_CHARGEVOLUME_SP			!8005
INT MODBUSADD_POGON_CMD					!8006
INT MODBUSADD_OZONEFEED_CMD				!8007
!INT MODBUSADD_ALARMSTOP_CMD
INT MODBUSADD_POG_STATUS1				!8028
INT MODBUSADD_FEED_FLOW_MODE			!8051
INT MODBUSADD_OZONE_RDY					!8052
INT	MODBUSADD_FEED_RDY					!8053
INT MODBUSADD_POG_NO_FLT				!8054
INT MODBUSADD_IN_REMOTE_MODE			!8060
INT MODBUSADD_MEIDENSHA_ALARMS			!starting 8032, qty 6, ZW 10/16/2020
!These are read back (RB) from the Keyence to update Molly
INT POGON_CMD_RB
INT OZONEFEED_CMD_RB
INT LC_BYPASSMODE_CMD_RB
INT OXYGEN_BYPASS_CMD_RB
INT POG_STATUS1
!Local variables derived from POG_STATUS1
INT POGON_FROM_STATUS1 
INT FEED_ON_FROM_STATUS1
INT StatusCapturePOGON
INT StatusCaptureFEED_ON
INT OkToTogglePOG

!INT MODBUSADD_OutletPressurePV

!variables M->P (MBE to Meiden) - Generated in Buffer 12
Global INT LC_MFC_SP
Global INT LC_BYPASSMODE_CMD
Global INT OXYGEN_BYPASS_CMD
Global INT OZONEPRESSURE_SP
Global INT OZONECHAMBERPRESSURE_SP
Global INT CHARGEVOLUME_SP
Global INT POGON_CMD
Global INT OZONEFEED_CMD
!Global INT ALARMSTOP_CMD		This one to be hardwired
!Status from Meiden for Buffer 12 interlocks
Global INT FEED_FLOW_MODE
Global INT OZONE_RDY
Global INT FEED_RDY
Global INT POG_NO_FLT
Global INT IN_REMOTE_MODE
! BEGIN 10/16/2020 by ZW for Ozone safety alarms
Global INT NO_POGB_FLT_TO_CLOSE_ESV
Global INT S0_EMO_activated != ir_rspn(3).0						!8110.0
Global INT S01_Power_failure != ir_rspn(3).1					!8110.1
Global INT S08_PCBox_Communications_Failure != ir_rspn(3).8		!8110.8
Global INT S09_MC_Trouble != ir_rspn(3).9						!8110.9
Global INT S14_WaterOZ_leakage_detected != ir_rspn(3).14		!8110.14

Global INT S16_Refrigerator_failure != ir_rspn(4).0				!8111.0
Global INT S19_Exhaust_down != ir_rspn(4).3						!8111.3
Global INT S20_Pneumatic_down != ir_rspn(4).4					!8111.4
Global INT S23_Water_flow_down != ir_rspn(4).7					!8111.7

Global INT S36_External_Alarm_Signal_Detect != ir_rspn(5).5		!8112.5
Global INT S42_Ozone_excessive_pressure != ir_rspn(5).11		!8112.11


Global INT L01_Heater2_failure != ir_rspn(7).1					!8114.1
Global INT L03_Cooling_time_up != ir_rspn(7).3					!8114.3
Global INT L05_Dry_pump_failure != ir_rspn(7).5					!8114.5
Global INT L06_Vacuum_pressure_High != ir_rspn(7).6				!8114.6
Global INT L07_Overtime_alarm_chamber_pumping != ir_rspn(7).7	!8114.7
Global INT L08_Overtime_alarm_HICpumping != ir_rspn(7).8		!8114.8

Global INT L20_Ozone_temperature_overrange != ir_rspn(8).4		!8115.4
Global INT L21_Temperature_controller_failure != ir_rspn(8).5	!8115.5
Global INT L22_Discharge_pressure_failure != ir_rspn(8).6		!8115.6
Global INT L23_Ozone_pressure_overrange != ir_rspn(8).7			!8115.7
Global INT L24_Chamber_Pressure_SensorFault1 != ir_rspn(8).8	!8115.8
Global INT L25_Chamber_Pressure_SensorFault2 != ir_rspn(8).9	!8115.9
! END 10/16/2020 by ZW for Ozone safety alarms

!Modbus communication variables
INT ir_rqst(100) ! Request array
INT ir_rspn(100) ! Response array
INT iw_rqst(100) !Request array
INT iw_rspn(100) !Response array
!real r_rqst(100) !Request array
!real r_rspn(100) !Response array
INT r_reg!Slave 32-bit register
INT w_reg!Slave 32-bit register
INT result1, result2 ! Function result
INT slave_IP, slave_channel, slave_id, Timeout
INT Direction_Write, Direction_Read

!Local variables - I DON'T THINK WE NEED THESE.  BELOW, MAYBE WRITE DIRECTLY TO GLOBAL VARIABLES?
!INT FeedFlowMode
!INT Ozone_Ready
!INT Feed_Ready
!INT POG_No_Fault
!INT OutletPressurePV

!initialize the Modbus address
MODBUSADD_OFFSET = 8000
MODBUSADD_LC_MFC_SP = MODBUSADD_OFFSET + 0
MODBUSADD_LC_BYPASSMODE_CMD = MODBUSADD_OFFSET + 1
MODBUSADD_OXYGEN_BYPASSMODE_CMD = MODBUSADD_OFFSET + 2
MODBUSADD_OZONEPRESSURE_SP = MODBUSADD_OFFSET + 3
MODBUSADD_OZONECHAMBERPRESSURE_SP = MODBUSADD_OFFSET + 4
MODBUSADD_CHARGEVOLUME_SP = MODBUSADD_OFFSET + 5
MODBUSADD_POGON_CMD = MODBUSADD_OFFSET + 6
MODBUSADD_OZONEFEED_CMD = MODBUSADD_OFFSET + 7
!MODBUSADD_ALARMSTOP_CMD = MODBUSADD_OFFSET + 8
MODBUSADD_POG_STATUS1 = MODBUSADD_OFFSET + 28
MODBUSADD_FEED_FLOW_MODE = MODBUSADD_OFFSET + 51
MODBUSADD_OZONE_RDY = MODBUSADD_OFFSET + 52
MODBUSADD_FEED_RDY = MODBUSADD_OFFSET + 53
MODBUSADD_POG_NO_FLT = MODBUSADD_OFFSET + 54
MODBUSADD_IN_REMOTE_MODE = MODBUSADD_OFFSET + 60
!MODBUSADD_OutletPressurePV = MODBUSADD_OFFSET + 24
MODBUSADD_MEIDENSHA_ALARMS = MODBUSADD_OFFSET + 32	!110	! for error decoding starting from 8110, by ZW 10/16/2020
! setup Bodbus communication
Direction_Write = 16
Direction_Read = 3
Timeout = 500
slave_channel = 19 ! Can be 17, 18, or 19
slave_IP = 192 + 168*POW(2, 8) + 1*POW(2,16) + 101*POW(2,24) ! = 192.168.1.101
slave_id = 1 ! 1 to 247 for MODBUS Slave IDs
! Open the Modbus TCP connection with slave device
SETCONF(308, slave_channel, 0) ! Close
wait 2000
SETCONF(308, slave_channel, slave_IP) ! Open
wait 2000
!SETCONF(309,slave_channel,1)
INP(slave_channel) ! Purges off all characters received before in the channel

!WRITE
!Global Int        M2K_POG_On, M2K_Feed_On, M2K_LC_Bypass_On, M2K_O2_Bypass_On
!Global Int        M2K_LC_FlowSP, M2K_O3PressureSP, M2K_O3ChamberPresSP, M2K_ChargeVolSP       !O3Pressure for CBr4 type only
!Global Int        Word8001, Word8002, Word8006, Word8007    !placeholders 
!Global int 	      O3_CommandWord    !combined M2G states; bit0 is GV, bit 1 is CAR, bit 2 is PosLim



!Initialize Logic - READ/WRITE Meiden PLC
!READ
ir_rqst(0)=slave_id !Slave ID
ir_rqst(1)=Direction_Read !3-Read Holding Register; 16-write holding register; 
!ir_rqst(2)=274 !Starting Address, STATUS WORD
ir_rqst(3)=1 !Quantity of Registers
ir_rqst(4)=2 !Byte Count
ir_rqst(5)=0.0 !Value to be written, used with 16-write holding register

!WRITE
iw_rqst(0)=slave_id !Slave ID
iw_rqst(1)=Direction_Write !3-Read Holding Register;; 16-write holding register; 
!iw_rqst(2)=786 !Starting Address, MEASURED
iw_rqst(3)=1 !Quantity of Registers
iw_rqst(4)=2 !Byte Count
iw_rqst(5)=0 !Value to be written, used with 16-write holding register

INT WriteIsAllowed      !for development and testing use
WriteIsAllowed=1




AA:

!READ always
!8028
ir_rqst(2) = MODBUSADD_POG_STATUS1
result1 = OUTP(slave_channel, ir_rqst, 0, 4) ! Send request through channel 17
result2 = INP(slave_channel, ir_rspn, 0, 5, Timeout) ! Receive Slave response - CJR changed 10 to 4 to match OUTP
POG_STATUS1 = ir_rspn(3)
wait 20
!Evaluate Status1 for POGON and FEED_ON (not FEEDING)
!POG_ON
IF POG_STATUS1=11 | POG_STATUS1=12 | POG_STATUS1=21 | POG_STATUS1=22 | POG_STATUS1=31 | POG_STATUS1=32 | POG_STATUS1=33 | POG_STATUS1=34
	POGON_FROM_STATUS1 = 1
ELSE
	POGON_FROM_STATUS1 = 0
END
!FEED_ON
IF POG_STATUS1=31 | POG_STATUS1=32 | POG_STATUS1=33
	FEED_ON_FROM_STATUS1 = 1
ELSE
	FEED_ON_FROM_STATUS1 = 0
END

!8051
ir_rqst(2) = MODBUSADD_FEED_FLOW_MODE
result1 = OUTP(slave_channel, ir_rqst, 0, 4) ! Send request through channel 17
wait 20
result2 = INP(slave_channel, ir_rspn, 0, 5, Timeout) ! Receive Slave response - CJR changed 10 to 4 to match OUTP
FEED_FLOW_MODE = ir_rspn(3)

!8052
ir_rqst(2) = MODBUSADD_OZONE_RDY
result1 = OUTP(slave_channel, ir_rqst, 0, 4) ! Send request through channel 17
wait 20
result2 = INP(slave_channel, ir_rspn, 0, 5, Timeout) ! Receive Slave response - CJR changed 10 to 4 to match OUTP
OZONE_RDY = ir_rspn(3).0

!8053
ir_rqst(2) = MODBUSADD_FEED_RDY
result1 = OUTP(slave_channel, ir_rqst, 0, 4) ! Send request through channel 17
wait 20
result2 = INP(slave_channel, ir_rspn, 0, 5, Timeout) ! Receive Slave response - CJR changed 10 to 4 to match OUTP
FEED_RDY = ir_rspn(3).0

!8054
ir_rqst(2) = MODBUSADD_POG_NO_FLT
result1 = OUTP(slave_channel, ir_rqst, 0, 4) ! Send request through channel 17
wait 20
result2 = INP(slave_channel, ir_rspn, 0, 5, Timeout) ! Receive Slave response - CJR changed 10 to 4 to match OUTP
POG_NO_FLT = ir_rspn(3).0

!8060
ir_rqst(2) = MODBUSADD_IN_REMOTE_MODE
result1 = OUTP(slave_channel, ir_rqst, 0, 4) ! Send request through channel 17
wait 20
result2 = INP(slave_channel, ir_rspn, 0, 5, Timeout) ! Receive Slave response - CJR changed 10 to 4 to match OUTP
IN_REMOTE_MODE = ir_rspn(3).0

!8024 - OutletPressurePV
!ir_rqst(2) = MODBUSADD_OutletPressurePV
!result1 = OUTP(slave_channel, ir_rqst, 0, 4) ! Send request through channel 17
!result2 = INP(slave_channel, ir_rspn, 0, 5, Timeout) ! Receive Slave response - CJR changed 10 to 4 to match OUTP
!OutletPressurePV = ir_rspn(3)
!wait 20

!8110 qty 6
ir_rqst(2) = MODBUSADD_MEIDENSHA_ALARMS
ir_rqst(3) = 6 !Quantity of Registers
ir_rqst(4) = 12 !Byte Count

result1 = OUTP(slave_channel, ir_rqst, 0, 4) ! Send request through channel 17
wait 20
result2 = INP(slave_channel, ir_rspn, 0, 10, Timeout) ! Receive Slave response - CJR changed 10 to 4 to match OUTP

S0_EMO_activated = ir_rspn(3).0						!8032.0
S01_Power_failure = ir_rspn(3).1					!8032.1
S08_PCBox_Communications_Failure = ir_rspn(3).8		!8032.8
S09_MC_Trouble = ir_rspn(3).9						!8032.9
S14_WaterOZ_leakage_detected = ir_rspn(3).14		!8032.14

S16_Refrigerator_failure = ir_rspn(4).0				!8033.0
S19_Exhaust_down = ir_rspn(4).3						!8033.3
S20_Pneumatic_down = ir_rspn(4).4					!8033.4
S23_Water_flow_down = ir_rspn(4).7					!8033.7

S36_External_Alarm_Signal_Detect = ir_rspn(5).4		!8034.4
S42_Ozone_excessive_pressure = ir_rspn(5).10		!8034.10


L01_Heater2_failure = ir_rspn(7).1					!8036.1
L03_Cooling_time_up = ir_rspn(7).3					!8036.3
L05_Dry_pump_failure = ir_rspn(7).5					!8036.5
L06_Vacuum_pressure_High = ir_rspn(7).6				!8036.6
L07_Overtime_alarm_chamber_pumping = ir_rspn(7).7	!8036.7
L08_Overtime_alarm_HICpumping = ir_rspn(7).8		!8036.8

L20_Ozone_temperature_overrange = ir_rspn(8).4		!8037.4
L21_Temperature_controller_failure = ir_rspn(8).5	!8037.5
L22_Discharge_pressure_failure = ir_rspn(8).6		!8037.6
L23_Ozone_pressure_overrange = ir_rspn(8).7			!8037.7
L24_Chamber_Pressure_SensorFault1 = ir_rspn(8).8	!8037.8
L25_Chamber_Pressure_SensorFault2 = ir_rspn(8).9	!8037.9
!L32	POGB Communication Failure	
!close ESV signal 1=happy, 0=close ESV
NO_POGB_FLT_TO_CLOSE_ESV = ^(S0_EMO_activated | S01_Power_failure | S08_PCBox_Communications_Failure | S09_MC_Trouble | S14_WaterOZ_leakage_detected | S16_Refrigerator_failure|S19_Exhaust_down|S20_Pneumatic_down|S23_Water_flow_down|S36_External_Alarm_Signal_Detect|S42_Ozone_excessive_pressure|L01_Heater2_failure|L03_Cooling_time_up|L05_Dry_pump_failure|L06_Vacuum_pressure_High|L07_Overtime_alarm_chamber_pumping|L08_Overtime_alarm_HICpumping|L20_Ozone_temperature_overrange|L21_Temperature_controller_failure|L22_Discharge_pressure_failure|L23_Ozone_pressure_overrange|L24_Chamber_Pressure_SensorFault1|L25_Chamber_Pressure_SensorFault2)
!close leak valve signal 1=happy, 0=close leak valve
POG_NO_FLT = POG_NO_FLT | NO_POGB_FLT_TO_CLOSE_ESV






!end READ always  !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!


ir_rqst(3)=1 !Quantity of Registers
ir_rqst(4)=2 !Byte Count


!WRITE always
!8006
ir_rqst(2) = MODBUSADD_POGON_CMD
result1 = OUTP(slave_channel, ir_rqst, 0, 4) ! Send request through channel 17
wait 20
result2 = INP(slave_channel, ir_rspn, 0, 5, Timeout) ! Receive Slave response - CJR changed 10 to 4 to match OUTP
!if the SP is changed, write it to Meiden controller
if (WriteIsAllowed=1) & (ir_rspn(3) <> POGON_CMD) 
      iw_rqst(2) = MODBUSADD_POGON_CMD
      iw_rqst(5) = POGON_CMD
	  !iw_rqst(5) = 1
      OUTP(slave_channel, iw_rqst, 0, 6) ! Send request through channel 17
      INP(slave_channel, iw_rspn, 0, 6, Timeout) ! Receive Slave response - CJR changed 10 to 4 to match OUTP
      wait 20
end
!end WRITE always  !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!




!REMOTE MODE - Molly controls
IF (IN_REMOTE_MODE=1)

!8000
ir_rqst(2) = MODBUSADD_LC_MFC_SP
result1 = OUTP(slave_channel, ir_rqst, 0, 4) ! Send request through channel 17
wait 20
result2 = INP(slave_channel, ir_rspn, 0, 5, Timeout) ! Receive Slave response - CJR changed 10 to 4 to match OUTP
!if the SP is changed, write it to Meiden controller
if (WriteIsAllowed=1) & (IN_REMOTE_MODE=1) & (ir_rspn(3) <> LC_MFC_SP) 
!if (ir_rspn(3) <> LC_MFC_SP)
      iw_rqst(2) = MODBUSADD_LC_MFC_SP
      iw_rqst(5) = LC_MFC_SP
      OUTP(slave_channel, iw_rqst, 0, 6) ! Send request through channel 17
      INP(slave_channel, iw_rspn, 0, 6, Timeout) ! Receive Slave response - CJR changed 10 to 4 to match OUTP
      wait 20
end

!8001
ir_rqst(2) = MODBUSADD_LC_BYPASSMODE_CMD
result1 = OUTP(slave_channel, ir_rqst, 0, 4) ! Send request through channel 17
wait 20
result2 = INP(slave_channel, ir_rspn, 0, 5, Timeout) ! Receive Slave response - CJR changed 10 to 4 to match OUTP
!if the SP is changed, write it to Meiden controller
if (WriteIsAllowed=1) & (IN_REMOTE_MODE=1) & (ir_rspn(3).0 <> LC_BYPASSMODE_CMD) 
      iw_rqst(2) = MODBUSADD_LC_BYPASSMODE_CMD
      !iw_rqst(5).0 = LC_BYPASSMODE_CMD
	  iw_rqst(5) = LC_BYPASSMODE_CMD
      OUTP(slave_channel, iw_rqst, 0, 6) ! Send request through channel 17
      INP(slave_channel, iw_rspn, 0, 6, Timeout) ! Receive Slave response - CJR changed 10 to 4 to match OUTP
      wait 20
end

!8002
ir_rqst(2) = MODBUSADD_OXYGEN_BYPASSMODE_CMD
result1 = OUTP(slave_channel, ir_rqst, 0, 4) ! Send request through channel 17
wait 20
result2 = INP(slave_channel, ir_rspn, 0, 5, Timeout) ! Receive Slave response - CJR changed 10 to 4 to match OUTP
!if the SP is changed, write it to Meiden controller
if (WriteIsAllowed=1) & (IN_REMOTE_MODE=1) & (ir_rspn(3).0 <> OXYGEN_BYPASS_CMD)   
      iw_rqst(2) = MODBUSADD_OXYGEN_BYPASSMODE_CMD
      iw_rqst(5) = OXYGEN_BYPASS_CMD
      OUTP(slave_channel, iw_rqst, 0, 6) ! Send request through channel 17
      INP(slave_channel, iw_rspn, 0, 6, Timeout) ! Receive Slave response - CJR changed 10 to 4 to match OUTP
      wait 20
end

!8003
ir_rqst(2) = MODBUSADD_OZONEPRESSURE_SP
result1 = OUTP(slave_channel, ir_rqst, 0, 4) ! Send request through channel 17
wait 20
result2 = INP(slave_channel, ir_rspn, 0, 5, Timeout) ! Receive Slave response - CJR changed 10 to 4 to match OUTP
!if the SP is changed, write it to Meiden controller
if (WriteIsAllowed=1) & (IN_REMOTE_MODE=1) & (ir_rspn(3) <> OZONEPRESSURE_SP)    
      iw_rqst(2) = MODBUSADD_OZONEPRESSURE_SP
      iw_rqst(5) = OZONEPRESSURE_SP
      OUTP(slave_channel, iw_rqst, 0, 6) ! Send request through channel 17
      INP(slave_channel, iw_rspn, 0, 6, Timeout) ! Receive Slave response - CJR changed 10 to 4 to match OUTP
      wait 20
end

!8004
ir_rqst(2) = MODBUSADD_OZONECHAMBERPRESSURE_SP
result1 = OUTP(slave_channel, ir_rqst, 0, 4) ! Send request through channel 17
wait 20
result2 = INP(slave_channel, ir_rspn, 0, 5, Timeout) ! Receive Slave response - CJR changed 10 to 4 to match OUTP
!if the SP is changed, write it to Meiden controller
if (WriteIsAllowed=1) & (IN_REMOTE_MODE=1) & (ir_rspn(3) <> OZONECHAMBERPRESSURE_SP)   
      iw_rqst(2) = MODBUSADD_OZONECHAMBERPRESSURE_SP
      iw_rqst(5) = OZONECHAMBERPRESSURE_SP
      OUTP(slave_channel, iw_rqst, 0, 6) ! Send request through channel 17
      INP(slave_channel, iw_rspn, 0, 6, Timeout) ! Receive Slave response - CJR changed 10 to 4 to match OUTP
      wait 20
end

!8005
ir_rqst(2) = MODBUSADD_CHARGEVOLUME_SP
result1 = OUTP(slave_channel, ir_rqst, 0, 4) ! Send request through channel 17
wait 20
result2 = INP(slave_channel, ir_rspn, 0, 5, Timeout) ! Receive Slave response - CJR changed 10 to 4 to match OUTP
!if the SP is changed, write it to Meiden controller
if (WriteIsAllowed=1) & (IN_REMOTE_MODE=1) & (ir_rspn(3) <> CHARGEVOLUME_SP)     
      iw_rqst(2) = MODBUSADD_CHARGEVOLUME_SP
      iw_rqst(5) = CHARGEVOLUME_SP
      OUTP(slave_channel, iw_rqst, 0, 6) ! Send request through channel 17
      INP(slave_channel, iw_rspn, 0, 6, Timeout) ! Receive Slave response - CJR changed 10 to 4 to match OUTP
      wait 20
end

!8007
ir_rqst(2) = MODBUSADD_OZONEFEED_CMD
result1 = OUTP(slave_channel, ir_rqst, 0, 4) ! Send request through channel 17
wait 20
result2 = INP(slave_channel, ir_rspn, 0, 5, Timeout) ! Receive Slave response - CJR changed 10 to 4 to match OUTP
!if the SP is changed, write it to Meiden controller
if (WriteIsAllowed=1)    
      iw_rqst(2) = MODBUSADD_OZONEFEED_CMD
      iw_rqst(5) = OZONEFEED_CMD
      OUTP(slave_channel, iw_rqst, 0, 6) ! Send request through channel 17
      INP(slave_channel, iw_rspn, 0, 6, Timeout) ! Receive Slave response - CJR changed 10 to 4 to match OUTP
      wait 20
end

!8008
!ir_rqst(2) = MODBUSADD_ALARMSTOP_CMD
!result1 = OUTP(slave_channel, ir_rqst, 0, 4) ! Send request through channel 17
!result2 = INP(slave_channel, ir_rspn, 0, 5, Timeout) ! Receive Slave response - CJR changed 10 to 4 to match OUTP
!wait 20
!if the SP is changed, write it to Meiden controller
!if (WriteIsAllowed=1) & (ir_rspn(3).0 <> ALARMSTOP_CMD)     
!      iw_rqst(2) = MODBUSADD_ALARMSTOP_CMD
!      iw_rqst(5).0 = ALARMSTOP_CMD
!      OUTP(slave_channel, iw_rqst, 0, 6) ! Send request through channel 17
!      INP(slave_channel, iw_rspn, 0, 6, Timeout) ! Receive Slave response - CJR changed 10 to 4 to match OUTP
!      wait 20
!end

END	!REMOTE (Molly) MODE  !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!





!LOCAL MODE - Meiden (Keyence) controls
IF (IN_REMOTE_MODE=0)	!In LOCAL mode, read these value from Meidensha and write to Molly registers for polling

!8000
ir_rqst(2) = MODBUSADD_LC_MFC_SP 
result1 = OUTP(slave_channel, ir_rqst, 0, 4) ! Send request through channel 17
wait 20
result2 = INP(slave_channel, ir_rspn, 0, 5, Timeout) ! Receive Slave response - CJR changed 10 to 4 to match OUTP
LCOzoneFlowRateSP = ir_rspn(3)
!if the SP is changed, write it to Meiden controller

!8003
ir_rqst(2) = MODBUSADD_OZONEPRESSURE_SP
result1 = OUTP(slave_channel, ir_rqst, 0, 4) ! Send request through channel 17
wait 20
result2 = INP(slave_channel, ir_rspn, 0, 5, Timeout) ! Receive Slave response - CJR changed 10 to 4 to match OUTP
OzonePresureSP = ir_rspn(3)

!8004
ir_rqst(2) = MODBUSADD_OZONECHAMBERPRESSURE_SP
result1 = OUTP(slave_channel, ir_rqst, 0, 4) ! Send request through channel 17
wait 20
result2 = INP(slave_channel, ir_rspn, 0, 5, Timeout) ! Receive Slave response - CJR changed 10 to 4 to match OUTP
OzoneChamberSP = ir_rspn(3)

!8005
ir_rqst(2) = MODBUSADD_CHARGEVOLUME_SP
result1 = OUTP(slave_channel, ir_rqst, 0, 4) ! Send request through channel 17
wait 20
result2 = INP(slave_channel, ir_rspn, 0, 5, Timeout) ! Receive Slave response - CJR changed 10 to 4 to match OUTP
ChargeVolumeSP = ir_rspn(3)

!Status bits put into the CommandWord for Molly to read
!8001
ir_rqst(2) = MODBUSADD_LC_BYPASSMODE_CMD
result1 = OUTP(slave_channel, ir_rqst, 0, 4) ! Send request through channel 17
wait 20
result2 = INP(slave_channel, ir_rspn, 0, 5, Timeout) ! Receive Slave response - CJR changed 10 to 4 to match OUTP
LC_BYPASSMODE_CMD_RB = ir_rspn(3).0

!8002
ir_rqst(2) = MODBUSADD_OXYGEN_BYPASSMODE_CMD
result1 = OUTP(slave_channel, ir_rqst, 0, 4) ! Send request through channel 17
wait 20
result2 = INP(slave_channel, ir_rspn, 0, 5, Timeout) ! Receive Slave response - CJR changed 10 to 4 to match OUTP
OXYGEN_BYPASS_CMD_RB = ir_rspn(3).0

!8006
ir_rqst(2) = MODBUSADD_POGON_CMD
result1 = OUTP(slave_channel, ir_rqst, 0, 4) ! Send request through channel 17
wait 20
result2 = INP(slave_channel, ir_rspn, 0, 5, Timeout) ! Receive Slave response - CJR changed 10 to 4 to match OUTP
POGON_CMD_RB = ir_rspn(3).0

!8007
ir_rqst(2) = MODBUSADD_OZONEFEED_CMD
result1 = OUTP(slave_channel, ir_rqst, 0, 4) ! Send request through channel 17
wait 20
result2 = INP(slave_channel, ir_rspn, 0, 5, Timeout) ! Receive Slave response - CJR changed 10 to 4 to match OUTP
OZONEFEED_CMD_RB = ir_rspn(3).0

!CommandWord.0 = POGON_CMD_RB		!RB = read back from Meidensha
!CommandWord.1 = OZONEFEED_CMD_RB
CommandWord.0 = POGON_FROM_STATUS1
CommandWord.1 = FEED_ON_FROM_STATUS1
CommandWord.2 = LC_BYPASSMODE_CMD_RB
CommandWord.3 = OXYGEN_BYPASS_CMD_RB

END !LOCAL (Meidensha) MODE  !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!




!These are for Buffer 12 interlocks
!OZONE_RDY = Ozone_Ready
!FEED_RDY = Feed_Ready
!POG_NO_FLT = POG_No_Fault

!Send alarm signal to Veeco facility
!OUT(0).2 = ^POG_NO_FLT

goto AA


STOP


#12
!Buffer 12 - This buffer contains the interlock logic for the Ozone system


!NOTES:
!Axis 0 - Ozone Injector Nozzle
!Axis 1 - QCM

!***** DECLARATIONS *****
!Buffer 11 variables
GLOBAL	INT	OZONEPRESSURE_SP, OZONECHAMBERPRESSURE_SP, CHARGEVOLUME_SP, LC_MFC_SP
GLOBAL	INT	POGON_CMD, OZONEFEED_CMD, LC_BYPASSMODE_CMD, OXYGEN_BYPASS_CMD
!From Buffer 11, used in Buffer 12 interlocks
GLOBAL	INT OZONE_RDY
GLOBAL	INT FEED_RDY
GLOBAL	INT POG_NO_FLT
!From Buffer 11, used in Buffer 12 to end status to Molly
Global INT IN_REMOTE_MODE
!Ozone injector postions that cause motion to be prevented
INT mshPreventMove, bfmPreventMove, qcmPreventMove, xfrPreventMove
!D-Buffer variables don't need declaring here
! REALs:	OzonePressureSP, OzoneChamberSP, ChargeVolumeSP, LCOzoneFlowRateSP
! INTs:		CommandWord

!Buffer 17 variables
INT			InjBlockingMainShutter, InjBlockingBFM, InjBlockingQCM, InjBlockingRobot
GLOBAL INT	M2G_InjClearOfMainShutter, M2G_InjClearOfBFM, M2G_InjClearOfQCM, M2G_InjClearOfRobot, M2G_InjPresent
GLOBAL INT	M2G_OpenESV, M2G_LeakValveClosed
Global Int	G2M_OpenESV_Status, G2M_SafeToMoveInj, G2M_MainShutterOpen, G2M_BfmRetracted, G2M_QcmLeCAR, G2M_PrepareForTransfer

!Local variables:
INT	ConditionsOkForPogON, ConditionsOkForFeedON
INT O3_RunValve1, O3_RunValve2, O3_RunValve3, O3_RunValve4
INT SomethingChanged

GLOBAL REAL adjNegativeLimit(8)  !Used for ozone injector ring position limit when crash contitions are possible with other actuators
GLOBAL REAL adjPositiveLimit(8)  !Used for ozone injector ring position limit when crash contitions are possible with other actuators
Global real	mshLCZ_old, mshUCZ_old, bfmLCZ_old, bfmUCZ_old, qcmLCZ_old, qcmUCZ_old, xfrLCZ_old, xfrUCZ_old	!used to compare against changes from IOC Magellan

GLOBAL real OldSP(8), SP(8), SP_Modbus(8)

!***** INITIALATION *****

AA:

ConditionsOkForPogON 	= 1		!The 1 will be replaced as we determine requirements for POG_ON
ConditionsOkForFeedON 	= 1		!The 1 will be replaced as we determine requirements for FEED_ON
!*** PART 1 - Copy Molly commands to Meiden PLC (Keyence)
!INTs in Buffer 11		= REALs in D-Buffer
OZONEPRESSURE_SP 		= OzonePresureSP	!Can I save REAL to INT?
OZONECHAMBERPRESSURE_SP = OzoneChamberSP	!Can I save REAL to INT?
CHARGEVOLUME_SP 		= ChargeVolumeSP	!Can I save REAL to INT?
LC_MFC_SP 				= LCOzoneFlowRateSP	!Can I save REAL to INT?
!LC_MFC_SP 				= 123	!Can I save REAL to INT?
!INTs in Buffer 11		= BITs in D-Buffer
POGON_CMD				=	CommandWord.0 & ConditionsOkForPogON
OZONEFEED_CMD			=	CommandWord.1 & ConditionsOkForFeedON
LC_BYPASSMODE_CMD		=	CommandWord.2
OXYGEN_BYPASS_CMD		=	CommandWord.3


!*** PART 2 - Ozone Run Valve Interlocks - move interlocks to GM PLC
M2G_OpenESV				=	CommandWord.4

!StatusWord for Molly
StatusWord.0 = G2M_OpenESV_Status	!This comes from the GM controller where the ESV interlocks now reside.
StatusWord.1 = G2M_OpenESV_Status & ^M2G_LeakValveClosed	!Both valves open (ESV and Leak Valve) cause a "GM1_TransferValveIntlk"

!*** PART 3 - Ozone Injector Nozzle Interlocks
!The default interlock collision values are loaded (new controller from defaults; reset controller from persistent memory) in buffer 27. After that
!changes from IOC Magellan are monitored here, and written to persistent memory.

IF mshLCZ <> mshLCZ_old 
	O3CollisionValues(0)=mshLCZ
	mshLCZ_old = mshLCZ	
	SomethingChanged=1
END
IF mshUCZ <> mshUCZ_old 
	O3CollisionValues(1)=mshUCZ
	mshUCZ_old = mshUCZ	
	SomethingChanged=1
END
	
IF bfmLCZ <> bfmLCZ_old 
	O3CollisionValues(2)=bfmLCZ
	bfmLCZ_old = bfmLCZ	
	SomethingChanged=1
END
IF bfmUCZ <> bfmUCZ_old 
	O3CollisionValues(3)=bfmUCZ
	bfmUCZ_old = bfmUCZ	
	SomethingChanged=1
END

IF qcmLCZ <> qcmLCZ_old 
	O3CollisionValues(4)=qcmLCZ
	qcmLCZ_old = qcmLCZ
	SomethingChanged=1
END
IF qcmUCZ <> qcmUCZ_old 
	O3CollisionValues(5)=qcmUCZ
	qcmUCZ_old = qcmUCZ
	SomethingChanged=1
END

IF xfrLCZ <> xfrLCZ_old 
	O3CollisionValues(6)=xfrLCZ
	xfrLCZ_old = xfrLCZ	
	SomethingChanged=1
END
IF xfrUCZ <> xfrUCZ_old 
	O3CollisionValues(7)=xfrUCZ
	xfrUCZ_old = xfrUCZ
	SomethingChanged=1
END	

! If any value changed, write new collision values to persistent memory
IF SomethingChanged=1
	WRITE O3CollisionValues, O3CollisionValues
	SomethingChanged=0
END	


IF MFLAGS(0).#HOME = 0 | (Measured0 > mshLCZ+0.05 & Measured0 < mshUCZ-0.05) | (Sp0 > mshLCZ & Sp0 < mshUCZ) 	InjBlockingMainShutter = 1 		ELSE InjBlockingMainShutter = 0	END
!BFM
IF MFLAGS(0).#HOME = 0 | (Measured0 > bfmLCZ+0.05 & Measured0 < bfmUCZ-0.05) | (Sp0 > bfmLCZ & Sp0 < bfmUCZ) 	InjBlockingBFM = 1 				ELSE InjBlockingBFM = 0			END
!QCM 
IF MFLAGS(0).#HOME = 0 | (Measured0 > qcmLCZ+0.05 & Measured0 < qcmUCZ-0.05) | (Sp0 > qcmLCZ & Sp0 < qcmUCZ) 	InjBlockingQCM = 1 				ELSE InjBlockingQCM = 0			END
!Robot
IF MFLAGS(0).#HOME = 0 | (Measured0 > xfrLCZ+0.05 & Measured0 < xfrUCZ-0.05) | (Sp0 > xfrLCZ & Sp0 < xfrUCZ) 	InjBlockingRobot = 1 			ELSE InjBlockingRobot = 0		END


M2G_InjClearOfMainShutter	=	^InjBlockingMainShutter
M2G_InjClearOfBFM			=	^InjBlockingBFM
M2G_InjClearOfQCM			=	^InjBlockingQCM
M2G_InjClearOfRobot			=	^InjBlockingRobot

!IF (iType(0)= 5007) InjOnAxis0 = 1 ELSE InjOnAxis0 = 0 END
!IF (iType(1)= 5007) InjOnAxis1 = 1 ELSE InjOnAxis1 = 0 END
		
!IF (iType(0)= 5007 | iType(1)= 5007) M2G_InjPresent = 1 ELSE M2G_InjPresent = 0 END

!Check if injector is blocking an actuator, and somehow the actuator is not retracted - prevent motion
IF InjBlockingMainShutter 	& G2M_MainShutterOpen=0 	mshPreventMove=1 ELSE mshPreventMove=0 END
IF InjBlockingBFM 			& G2M_BfmRetracted=0 		bfmPreventMove=1 ELSE bfmPreventMove=0 END
IF InjBlockingQCM 			& G2M_QcmLeCAR=0 			qcmPreventMove=1 ELSE qcmPreventMove=0 END
IF InjBlockingRobot 		& G2M_PrepareForTransfer=1 	xfrPreventMove=1 ELSE xfrPreventMove=0 END

IF M2G_InjPresent=1 & mshPreventMove=0 & bfmPreventMove=0 & qcmPreventMove=0! & xfrPreventMove=0
	iPreventMove(0) = 0
ELSE
	iPreventMove(0) = 1
END

!Set the adjustable NegativeLimit based on actuators that are not retracted:
!NOTE - the order of the next 4 lines is critical! Must be in order of smallest to greatest UCZ
IF G2M_PrepareForTransfer=1 	& Measured0 <= xfrUCZ adjNegativeLimit(0)=0.0 END	!0.0 - special case where RETRACT/LOCK sends injector to 0.0
IF G2M_MainShutterOpen=0 		& Measured0 >= mshUCZ adjNegativeLimit(0)=mshUCZ END	!1.6
IF G2M_PrepareForTransfer=1 	& Measured0 >= xfrUCZ adjNegativeLimit(0)=xfrUCZ END	!2.0
IF G2M_QcmLeCAR=0  				& Measured0 >= qcmUCZ adjNegativeLimit(0)=qcmUCZ END	!3.4
IF G2M_BfmRetracted=0 			& Measured0 >= bfmUCZ adjNegativeLimit(0)=bfmUCZ END	!3.7
IF G2M_SafeToMoveInj=1 adjNegativeLimit(0)= dNegtiveLimit(0) END !SafeToMoveInj is only ON from GM when ALL actuators are out of the way

!Set the adjustable PositiveLimit based on actuators that are not retracted:
!NOTE - the order of the next 4 lines is critical! Must be in order of greatest to smallest LCZ
IF G2M_QcmLeCAR=0  				& Measured0 <= qcmLCZ adjPositiveLimit(0)=qcmLCZ END	!1.0
IF G2M_BfmRetracted=0 			& Measured0 <= bfmLCZ adjPositiveLimit(0)=bfmLCZ END	!0.7
IF G2M_MainShutterOpen=0 		& Measured0 <= mshLCZ adjPositiveLimit(0)=mshLCZ END	!0.3
IF G2M_PrepareForTransfer=1 	& Measured0 <= xfrLCZ adjPositiveLimit(0)=xfrLCZ END	!0.3

IF G2M_SafeToMoveInj=1 adjNegativeLimit(0)= dNegtiveLimit(0) END !SafeToMoveInj is only ON from GM when ALL actuators are out of the way
IF G2M_SafeToMoveInj=1 adjPositiveLimit(0)= dPositiveLimit(0) END !SafeToMoveInj is only ON from GM when ALL actuators are out of the way

!When lot manager sets "prepare for transfer" flag, wait for clear injector path, and then send injector to 0.0
IF G2M_PrepareForTransfer=1 & G2M_MainShutterOpen=1 & G2M_BfmRetracted=1 & G2M_QcmLeCAR=1 Sp0=0.0 END

DynNegativeLimit = adjNegativeLimit(0)	!DynNegativeLimit is the variable name sent to Molly - see D Buffer
DynPositiveLimit = adjPositiveLimit(0)	!DynPositiveLimit is the variable name sent to Molly - see D Buffer

goto AA


STOP
#13
!Buffer 13
!For READ and WRITE to the Beckhoff CT controller

GLOBAL INT M2C_CARZ, CARZ_PM_AxisNum, M2C_CARZ2, CARZ_PM2_AxisNum
GLOBAL INT C2M_OkToMoveCAR, C2M_CAR_UP, C2M_CAR_DOWN, C2M_OkToMoveCAR2, C2M_CAR2_UP, C2M_CAR2_DOWN
GLOBAL INT CARZ_PM_StatusWord, CARZ_PM2_StatusWord
GLOBAL REAL tmpMeasured(8)

INT SlaveChannel, SlaveIp, SlaveId
INT ReadRequest(100), ReadResponse(100)
INT WriteRequest(100), WriteResponse(100)
INT ReadFunctionCode, WriteFunctionCode
INT StartingReadRegister, StartingWriteRegister
INT	Bit0, Bit1

SlaveChannel = 17 ! Can be 17, 18, or 19
SlaveIp = 192 + 168 * POW(2, 8) + 1 * POW(2, 16) + 45 * POW(2,24) ! 192.168.1.45
SlaveId = 1 ! 1 to 247 for Modbus slave ids
ReadFunctionCode = 3
StartingReadRegister = 296 ! Word 40 in CT controller
WriteFunctionCode = 16
StartingWriteRegister = 808 ! Word 40 in CT controller

! Modbus read request.
ReadRequest(0) = SlaveId
ReadRequest(1) = ReadFunctionCode
ReadRequest(2) = StartingReadRegister
ReadRequest(3) = 2  ! Quantity of registers

! Modbus write request.
WriteRequest(0) = SlaveId
WriteRequest(1) = WriteFunctionCode
WriteRequest(2) = StartingWriteRegister
WriteRequest(3) = 2 ! Quantity of registers
WriteRequest(4) = 4 ! Byte Count
WriteRequest(5) = 0 ! Value to be written
WriteRequest(6) = 0 ! Value to be written

! Open the Modbus TCP connection with the CT controller.
SETCONF(308, SlaveChannel, 0) ! Close
WAIT 2000
SETCONF(308, SlaveChannel, SlaveIp) ! Open
WAIT 2000
INP(SlaveChannel) ! Purge all characters

MAIN_LOOP:

! Read the CAR Z signals from the CT controller.
IF M2C_CARZ = 1 | M2C_CARZ2 = 1
    OUTP(SlaveChannel, ReadRequest, 0, 4) ! Send the request
    INP(SlaveChannel, ReadResponse, 0, 5) ! Receive the response

    IF ReadResponse(1) = ReadFunctionCode
        C2M_OkToMoveCAR  = ReadResponse(3).0
        C2M_CAR_UP       = ReadResponse(3).1
        C2M_CAR_DOWN     = ReadResponse(3).2
        C2M_OkToMoveCAR2 = ReadResponse(4).0
        C2M_CAR2_UP      = ReadResponse(4).1
        C2M_CAR2_DOWN    = ReadResponse(4).2
    END
END

WAIT 100

! Write the CAR Z signals to the CT controller.
IF M2C_CARZ = 1 | M2C_CARZ2 = 1
    WriteRequest(5) = 0
    IF M2C_CARZ = 1
        Bit0 = SAFIN(CARZ_PM_AxisNum).#LL = 1 | (tmpMeasured(CARZ_PM_AxisNum) < 0.05) ! CAR is at home
        Bit1 = MST(CARZ_PM_AxisNum).4                                                 ! CAR is not moving
        CARZ_PM_StatusWord = Bit0 + (Bit1 * 2)
        WriteRequest(5) = CARZ_PM_StatusWord
    END
    WriteRequest(6) = 0
    IF M2C_CARZ2 = 1
        Bit0 = SAFIN(CARZ_PM2_AxisNum).#LL = 1 | (tmpMeasured(CARZ_PM2_AxisNum) < 0.05) ! CAR is at home
        Bit1 = MST(CARZ_PM2_AxisNum).4                                                  ! CAR is not moving
        CARZ_PM2_StatusWord = Bit0 + (Bit1 * 2)
        WriteRequest(6) = CARZ_PM2_StatusWord
    END
    OUTP(SlaveChannel, WriteRequest, 0, 7) ! Send the request
    WAIT 50
    INP(SlaveChannel, WriteResponse, 0, 4) ! Receive the response
END

WAIT 100

GOTO MAIN_LOOP

STOP

#14
!For READ Modbus from Wago MTC controller at IP 45

!READ - declaration
Global INT 	MZ_CARAxisNum, MZ_LLAxisNum, MZ_STAxisNum	!See Buffer 27, initialized
Global INT	oLL_Door_Closed		!iWord7Bit3			:= dinputs.,
Global INT	oGmCarHome_Switch	!iWord7Bit4			:= NOT dinputs.,
!Global INT	oLLElevator_Input1	!iWord10Bit0			:= dinputs.,
Global INT	iLl_Interlock_Cmd	!iWord10Bit1			:= doutputs.,
Global INT	iLl_Stage_Up_Cmd	!iWord10Bit2			:= doutputs.,
Global INT	iLl_Stage_Down_Cmd	!iWord10Bit3			:= doutputs.,
!	(*Storage Elevator Controller Items*)
!Global INT	oSTElevator_Input1	!iWord10Bit4			:= dinputs.,
Global INT	iSt_Interlock_Cmd	!iWord10Bit5			:= doutputs.,
Global INT	iSt_Stage_Up_Cmd	!iWord10Bit6			:= doutputs.,
Global INT	iSt_Stage_Down_Cmd	!iWord10Bit7			:= doutputs.,
!	(*CAR Controller Items*)
!Global INT	oGmCarHome_Encoder_Ind	!iWord10Bit8			:= dinputs.,
Global INT	iCar_Interlock_Cmd	!iWord10Bit9			:= doutputs.,
Global INT	iCar_Stage_Home_Cmd	!iWord10Bit10			:= doutputs.,
Global INT  oTIC_Local_Mode_Cmd !Word9Bit9

Global real RampRate(8)
Global INT iAxisGood(8)
Global Real OldSP(8), SP(8)
INT LLJogging_Command_Old, LLJogging_Command
INT STJogging_Command_Old, STJogging_Command

INT ir_rqst(100) ! for read word 7 at %QW263
INT ir_rspn(100) ! Response array

int r_reg!Slave 32-bit register
INT result1, result2 ! Function result
INT slave_IP, slave_channel, slave_id, Timeout
INT Direction_Write, Direction_Read

LLJogging_Command = 0
STJogging_Command = 0
LLJogging_Command_Old = 0
STJogging_Command_Old = 0

Direction_Write = 16
Direction_Read = 3
Timeout = 1500
slave_channel = 17 ! Can be 17, 18, or 19
!slave_IP = 192 + 168*POW(2, 8) + 1*POW(2,16) + 65*POW(2,24) ! = 192.168.1.45
slave_IP = 192 + 168*POW(2, 8) + 1*POW(2,16) + 45*POW(2,24) ! = 192.168.1.45
slave_id = 1 ! 1 to 247 for MODBUS Slave IDs
! Open the Modbus TCP connection with slave device
SETCONF(308, slave_channel, 0) ! Close
wait 2000
SETCONF(308, slave_channel, slave_IP) ! Open
wait 2000
!SETCONF(309,slave_channel,1)
INP(slave_channel) ! Purges off all characters received before in the channel

!WRITE - declaration
!Global Int 	M2G_QcmLeGV, M2G_QcmLeCAR, M2G_QcmLePosLim, M2G_QcmAtHome, M2G_QcmNotMoving, M2G_QcmPresent	!See Buffer 18
!Int	Bit0, Bit1, Bit2, Bit3, Bit4, Bit5	!Bit placeholders for conversion to integer
!Global int	QCM_StatusWord	!combined M2G states; bit0 is GV, bit 1 is CAR, bit 2 is PosLim
!int iw_rqst(100) !Request array
!int iw_rspn(100) !Response array
!int w_reg!Slave 32-bit register

!READ
ir_rqst(0)=slave_id !Slave ID
ir_rqst(1)=Direction_Read !3-Read Holding Register; 16-write holding register; 
ir_rqst(2)=263 !Starting Address, STATUS WORD
ir_rqst(3)=8 !Quantity 4 of Registers for 263(Word7), 264, 265, 266(Word10)
!r_rqst7(4)=4 !Byte Count
!r_rqst7(5)=0.0 !Value to be written, used with 16-write holding register




!WRITE
!iw_rqst(0)=1 !Slave ID
!iw_rqst(1)=16 !3-Read Holding Register;; 16-write holding register; 
!iw_rqst(2)=786 !Starting Address, MEASURED
!iw_rqst(3)=2 !Quantity of Registers
!iw_rqst(4)=4 !Byte Count
!iw_rqst(5)=5.126 !Value to be written, used with 16-write holding register

AA:

!READ
!outp(slave_channel, r_rqst, 0, 4) !send request to Slave
!reg = inp(slave_channel, r_rspn, 0, 5, 500) !receive Slave response

result1 = OUTP(slave_channel, ir_rqst, 0, 7) ! Send request through channel 17
wait 50
result2 = INP(slave_channel, ir_rspn, 0, 7, 500) ! Receive Slave respon
if ir_rspn(1)=3
	oLL_Door_Closed		= ir_rspn(3).3	!iWord7Bit3			:= dinputs.,
	oGmCarHome_Switch	= ir_rspn(3).4	!iWord7Bit4			:= dinputs.,
	
	oTIC_Local_Mode_Cmd	= 1!ir_rspn(5).9	!iWord9Bit9			:= dinputs.,
	
	iLl_Interlock_Cmd	= ir_rspn(6).1	!iWord10Bit1			:= doutputs.,

	
	iLl_Stage_Up_Cmd	= ir_rspn(6).2	!iWord10Bit2			:= doutputs.,
	iLl_Stage_Down_Cmd	= ir_rspn(6).3	!iWord10Bit3			:= doutputs.,
	LLJogging_Command_Old = LLJogging_Command
	if (iLl_Stage_Up_Cmd = 1) & (iLl_Stage_Down_Cmd = 0)
		LLJogging_Command = 1
	elseif (iLl_Stage_Up_Cmd = 0) & (iLl_Stage_Down_Cmd = 1)
		LLJogging_Command = -1
	else
		LLJogging_Command = 0
	end	
	!	(*Storage Elevator Controller Items*)
	iSt_Interlock_Cmd	= ir_rspn(6).5	!iWord10Bit5			:= doutputs.,
	
	
	iSt_Stage_Up_Cmd	= ir_rspn(6).6	!iWord10Bit6			:= doutputs.,
	iSt_Stage_Down_Cmd	= ir_rspn(6).7	!iWord10Bit7			:= doutputs.,
	STJogging_Command_Old = STJogging_Command
	if (iSt_Stage_Up_Cmd = 1) & (iSt_Stage_Down_Cmd = 0)
		STJogging_Command = 1
	elseif (iSt_Stage_Up_Cmd = 0) & (iSt_Stage_Down_Cmd = 1)
		STJogging_Command = -1
	else
		STJogging_Command = 0
	end	

	
	
	!	(*CAR Controller Items*)
	iCar_Interlock_Cmd	= ir_rspn(6).9	!iWord10Bit9			:= doutputs.,
	iCar_Stage_Home_Cmd	= ir_rspn(6).10	!iWord10Bit10			:= doutputs.,
	
	!Will capture command change
end

!if result2 > 0
!if i_rspn(3)&0x8000; i_rspn(3)=i_rspn(3)|0xFFFF0000; end
!result2=i_rspn(3)*256*256 + i_rspn(4)
!end

wait 50

!WRITE
!create the status word by combining the individual state integers
!QCM_StatusWord = M2G_QcmLePosA + M2G_QcmLePosB * POW(10,M2G_QcmLePosB) + M2G_QcmLePosC * POW(100,M2G_QcmLePosC)
!Bit0 = M2G_QcmLeGV		!In GM controller, oWord18.0
!Bit1 = M2G_QcmLeCAR		!In GM controller, oWord18.1
!Bit2 = M2G_QcmLePosLim	!In GM controller, oWord18.2
!Bit3 = M2G_QcmAtHome	!In GM controller, oWord18.3	!IN(0).6
!Bit4 = M2G_QcmNotMoving	!In GM controller, oWord18.4	!may or may not use
!Bit5 = M2G_QcmPresent	!In GM controller, oWord18.5	!from Buffer 18
!QCM_StatusWord = Bit0 + (Bit1*2) + (Bit2*4) + (Bit3*8) + (Bit4*16) + (Bit5*32)
!iw_rqst(5) = QCM_StatusWord !Value to be written, used with 16-write holding register
!outp(17, iw_rqst, 0, 6) !send request to Slave
!INP(17, iw_rspn, 0, 6, 500) ! Receive Slave response - CJR changed 10 to 4 to match OUTP



goto AA

STOP

!	oLL_Door_Closed = 1 event is handled in Buffer 27, same as a regular LL



On	(LLJogging_Command <> LLJogging_Command_Old) & (MZ_LLAxisNum > -1) & (iLl_Interlock_Cmd = 1) & (oTIC_Local_Mode_Cmd = 1)
	Break MZ_LLAxisNum
	if (LLJogging_Command = 1)& (iAxisGood(MZ_LLAxisNum) = 1)
	
		jog/v  MZ_LLAxisNum, RampRate(MZ_LLAxisNum)*dAxisPitch(MZ_LLAxisNum)
	elseif (LLJogging_Command = -1)& (iAxisGood(MZ_LLAxisNum) = 1)
		jog/v  MZ_LLAxisNum, -RampRate(MZ_LLAxisNum)*dAxisPitch(MZ_LLAxisNum)
	else
		halt MZ_LLAxisNum
	end
	
RET

On	(STJogging_Command <> STJogging_Command_Old) & (MZ_STAxisNum > -1) & (iSt_Interlock_Cmd = 1) & (oTIC_Local_Mode_Cmd = 1)
	Break MZ_STAxisNum
	if (STJogging_Command = 1) &(iAxisGood(MZ_STAxisNum) = 1)
		jog/v  MZ_STAxisNum, RampRate(MZ_STAxisNum)*dAxisPitch(MZ_STAxisNum)
	elseif (STJogging_Command = -1) &(iAxisGood(MZ_STAxisNum) = 1)
		jog/v  MZ_STAxisNum, -RampRate(MZ_STAxisNum)*dAxisPitch(MZ_STAxisNum)
	else
		halt MZ_STAxisNum
	end
	
RET




!On (iLl_Stage_Up_Cmd = 1) & (MZ_LLAxisNum > -1)
	!Break MZ_LLAxisNum
	!jog/v  MZ_LLAxisNum, RampRate(MZ_LLAxisNum)*dAxisPitch(MZ_LLAxisNum)
!ret

!On (iLl_Stage_Up_Cmd = 0) & (iLl_Stage_Down_Cmd = 0) & (MZ_LLAxisNum > -1)
!	halt MZ_LLAxisNum
!ret

!On (iLl_Stage_Down_Cmd = 1) & (MZ_LLAxisNum > -1)
!	Break MZ_LLAxisNum 
	!jog/v  MZ_LLAxisNum, -RampRate(MZ_LLAxisNum)*dAxisPitch(MZ_LLAxisNum)
!ret
	


!On iSt_Stage_Up_Cmd = 1
	!Break MZ_STAxisNum
	!jog/v  iAxisNum, RampRate(iAxisNum)*dAxisPitch(iAxisNum)
!ret

!On iSt_Stage_Up_Cmd = 0 & iSt_Stage_Down_Cmd = 0
	!halt
!ret
!On iSt_Stage_Down_Cmd = 1
	!Break
	!jog/v  iAxisNum, -RampRate(iAxisNum)*dAxisPitch(iAxisNum)
!ret


ON iCar_Stage_Home_Cmd & (MZ_CARAxisNum > -1) & (iCar_Interlock_Cmd = 1) & (oTIC_Local_Mode_Cmd = 1)

	if  (iAxisGood(MZ_CARAxisNum) = 1)
	
		if MZ_CARAxisNum = 0
			Sp0 = 0
		elseif MZ_CARAxisNum = 1
			Sp1 = 0
		elseif MZ_CARAxisNum = 2
			Sp3 = 0
		elseif MZ_CARAxisNum = 3
			Sp3 = 0
		elseif MZ_CARAxisNum = 4
			Sp4 = 0
		elseif MZ_CARAxisNum = 5
			Sp5 = 0
		elseif MZ_CARAxisNum = 6
			Sp6 = 0
		elseif MZ_CARAxisNum = 7
			Sp7 = 0
		end			
!	elseif  MFLAGS(iAxis).3 = 0
	end
RET


On (iLl_Interlock_Cmd = 0) & (MZ_LLAxisNum > -1)	!Interlock to Kill CAR motion
	KILL MZ_LLAxisNum
	iControlProcessState(MZ_LLAxisNum) = 113
	OldSP(MZ_LLAxisNum) = SP(MZ_LLAxisNum)
RET


On (iSt_Interlock_Cmd = 0) & (MZ_STAxisNum > -1)	!Interlock to Kill CAR motion
	KILL MZ_STAxisNum
	iControlProcessState(MZ_STAxisNum) = 113
	OldSP(MZ_STAxisNum) = SP(MZ_STAxisNum)
RET

On (iCar_Interlock_Cmd = 0) & (MZ_CARAxisNum > -1)	!Interlock to Kill CAR motion
	KILL MZ_CARAxisNum
	iControlProcessState(MZ_CARAxisNum) = 113
	OldSP(MZ_CARAxisNum) = SP(MZ_CARAxisNum)
RET
#15
!For READ and WRITE to the Wago GM controller on Sandia 1240 
!For BFM ARM interlock to mainshutter

!READ - declaration
real tmpMeasured(8)
GLOBAL INT HW_Intlkd_BFMQCM_AxisNum1, HW_Intlkd_BFMQCM_AxisNum2
!Global 	Int 	G2M_MaxSpGV, G2M_MaxSpCAR, G2M_MaxSpPosLim, G2M_SafeToMoveQcm	!See Buffer 18 
INT ir_rqst(100) ! Request array
INT ir_rspn(100) ! Response array
!real r_rqst(100) !Request array
!real r_rspn(100) !Response array
int r_reg!Slave 32-bit register
INT result1, result2 ! Function result
INT slave_IP, slave_channel, slave_id, Timeout
INT Direction_Write, Direction_Read
Direction_Write = 16
Direction_Read = 3
Timeout = 1500
slave_channel = 17 ! Can be 17, 18, or 19
slave_IP = 192 + 168*POW(2, 8) + 1*POW(2,16) + 13*POW(2,24) ! = 10.0.0.100
slave_id = 1 ! 1 to 247 for MODBUS Slave IDs
! Open the Modbus TCP connection with slave device
SETCONF(308, slave_channel, 0) ! Close
wait 2000
SETCONF(308, slave_channel, slave_IP) ! Open
wait 2000
!SETCONF(309,slave_channel,1)
INP(slave_channel) ! Purges off all characters received before in the channel

!WRITE - declaration
!Global Int 	M2G_QcmLeGV, M2G_QcmLeCAR, M2G_QcmLePosLim, M2G_QcmAtHome, M2G_QcmNotMoving, M2G_QcmPresent	!See Buffer 18

Int	Bit0, Bit1, Bit2, Bit3, Bit4, Bit5, Bit11	!Bit placeholders for conversion to integer
Int	BFM_StatusWord	!combined M2G states; bit0 is GV, bit 1 is CAR, bit 2 is PosLim
int iw_rqst(100) !Request array
int iw_rspn(100) !Response array
int w_reg!Slave 32-bit register

!READ
ir_rqst(0)=slave_id !Slave ID
ir_rqst(1)=Direction_Read !3-Read Holding Register; 16-write holding register; 
ir_rqst(2)=274 !Starting Address, STATUS WORD Word32
ir_rqst(3)=2 !Quantity of Registers
!r_rqst(0)=slave_id !Slave ID
!r_rqst(1)=Direction_Read !3-Read Holding Register;; 16-write holding register; 
!r_rqst(2)=100 !Starting Address, MEASURED
!r_rqst(3)=2 !Quantity of Registers
!r_rqst(4)=4 !Byte Count
!r_rqst(5)=0.0 !Value to be written, used with 16-write holding register

!WRITE
iw_rqst(0)=1 !Slave ID
iw_rqst(1)=16 !3-Read Holding Register;; 16-write holding register; 
iw_rqst(2)=800 !Starting Address, MEASURED, Word32
iw_rqst(3)=1!2 !Quantity of Registers
iw_rqst(4)=2!4 !Byte Count
iw_rqst(5)=5.126 !Value to be written, used with 16-write holding register


Bit0 = 0	!M2G_QcmLeGV		!In GM controller, oWord18.0
Bit1 = 0	!M2G_QcmLeCAR		!In GM controller, oWord18.1
Bit2 = 0	!M2G_QcmLePosLim	!In GM controller, oWord18.2
Bit3 = 0	!M2G_QcmAtHome	!In GM controller, oWord18.3	!IN(0).6
Bit4 = 0	!M2G_QcmNotMoving	!In GM controller, oWord18.4	!may or may not use
Bit5 = 0	!M2G_QcmPresent	!In GM controller, oWord18.5	!from Buffer 18


AA:

tmpMeasured(0) = Measured0
tmpMeasured(1) = Measured1
tmpMeasured(2) = Measured2
tmpMeasured(3) = Measured3
tmpMeasured(4) = Measured4
tmpMeasured(5) = Measured5
tmpMeasured(6) = Measured6
tmpMeasured(7) = Measured7
if HW_Intlkd_BFMQCM_AxisNum1 > -1
	if tmpMeasured(HW_Intlkd_BFMQCM_AxisNum1) <= 0.1 & MFLAGS(HW_Intlkd_BFMQCM_AxisNum1).#HOME = 1
		!WRITE
		Bit3 = 1	!M2G_QcmAtHome	!In GM controller, oWord32.3
	else
		!WRITE
		Bit3 = 0	!In GM controller, oWord32.3	
	end
end
if HW_Intlkd_BFMQCM_AxisNum2 > -1
	if tmpMeasured(HW_Intlkd_BFMQCM_AxisNum2) <= 0.1 & MFLAGS(HW_Intlkd_BFMQCM_AxisNum2).#HOME = 1
		!WRITE
		Bit11 = 1	!M2G_QcmAtHome	!In GM controller, oWord32.11	!IN(0).6
	else
		!WRITE
		Bit11 = 0	!In GM controller, oWord32.11	
	end
end


!BFM_StatusWord = (Bit3*8) + (Bit11*2048)
BFM_StatusWord.3 = Bit3
BFM_StatusWord.11 = Bit11
iw_rqst(5) = BFM_StatusWord !Value to be written, used with 16-write holding register
outp(slave_channel, iw_rqst, 0, 6) !send request to Slave
INP(slave_channel, iw_rspn, 0, 6, 500) ! Receive Slave response - CJR changed 10 to 4 to match OUTP


wait 1000

goto AA

STOP
#16
!For READ and WRITE to the Wago GM controller

!READ - declaration
Global 	Int 	G2M_MaxSpGV, G2M_MaxSpCAR, G2M_MaxSpPosLim, G2M_SafeToMoveQcm	!See Buffer 18 
INT ir_rqst(100) ! Request array
INT ir_rspn(100) ! Response array
!real r_rqst(100) !Request array
!real r_rspn(100) !Response array
int r_reg!Slave 32-bit register
INT result1, result2 ! Function result
INT slave_IP, slave_channel, slave_id, Timeout
INT Direction_Write, Direction_Read
Direction_Write = 16
Direction_Read = 3
Timeout = 1500
slave_channel = 17 ! Can be 17, 18, or 19
slave_IP = 192 + 168*POW(2, 8) + 1*POW(2,16) + 13*POW(2,24) ! = 10.0.0.100
slave_id = 1 ! 1 to 247 for MODBUS Slave IDs
! Open the Modbus TCP connection with slave device
SETCONF(308, slave_channel, 0) ! Close
wait 2000
SETCONF(308, slave_channel, slave_IP) ! Open
wait 2000
!SETCONF(309,slave_channel,1)
INP(slave_channel) ! Purges off all characters received before in the channel

!WRITE - declaration
Global Int 	M2G_QcmLeGV, M2G_QcmLeCAR, M2G_QcmLePosLim, M2G_QcmAtHome, M2G_QcmNotMoving, M2G_QcmPresent	!See Buffer 18
Int	Bit0, Bit1, Bit2, Bit3, Bit4, Bit5	!Bit placeholders for conversion to integer
Global int	QCM_StatusWord	!combined M2G states; bit0 is GV, bit 1 is CAR, bit 2 is PosLim
int iw_rqst(100) !Request array
int iw_rspn(100) !Response array
int w_reg!Slave 32-bit register

!READ
ir_rqst(0)=slave_id !Slave ID
ir_rqst(1)=Direction_Read !3-Read Holding Register; 16-write holding register; 
ir_rqst(2)=274 !Starting Address, STATUS WORD
ir_rqst(3)=2 !Quantity of Registers
!r_rqst(0)=slave_id !Slave ID
!r_rqst(1)=Direction_Read !3-Read Holding Register;; 16-write holding register; 
!r_rqst(2)=100 !Starting Address, MEASURED
!r_rqst(3)=2 !Quantity of Registers
!r_rqst(4)=4 !Byte Count
!r_rqst(5)=0.0 !Value to be written, used with 16-write holding register

!WRITE
iw_rqst(0)=1 !Slave ID
iw_rqst(1)=16 !3-Read Holding Register;; 16-write holding register; 
iw_rqst(2)=786 !Starting Address, MEASURED
iw_rqst(3)=2 !Quantity of Registers
iw_rqst(4)=4 !Byte Count
iw_rqst(5)=5.126 !Value to be written, used with 16-write holding register

AA:

!READ
!outp(slave_channel, r_rqst, 0, 4) !send request to Slave
!reg = inp(slave_channel, r_rspn, 0, 5, 500) !receive Slave response

result1 = OUTP(slave_channel, ir_rqst, 0, 4) ! Send request through channel 17
!result2 = INP(slave_channel, ir_rspn, 0, 10, 500) ! Receive Slave response
result2 = INP(slave_channel, ir_rspn, 0, 4, 500) ! Receive Slave response - CJR changed 10 to 4 to match OUTP

!if result2 > 0
!if i_rspn(3)&0x8000; i_rspn(3)=i_rspn(3)|0xFFFF0000; end
!result2=i_rspn(3)*256*256 + i_rspn(4)
!end
if ir_rspn(1)=3
G2M_MaxSpGV = ir_rspn(3).0
G2M_MaxSpCAR = ir_rspn(3).1
G2M_MaxSpPosLim = ir_rspn(3).2
G2M_SafeToMoveQcm = ir_rspn(3).3
end

wait 1000

!WRITE
!create the status word by combining the individual state integers
!QCM_StatusWord = M2G_QcmLePosA + M2G_QcmLePosB * POW(10,M2G_QcmLePosB) + M2G_QcmLePosC * POW(100,M2G_QcmLePosC)
Bit0 = M2G_QcmLeGV		!In GM controller, oWord18.0
Bit1 = M2G_QcmLeCAR		!In GM controller, oWord18.1
Bit2 = M2G_QcmLePosLim	!In GM controller, oWord18.2
Bit3 = M2G_QcmAtHome	!In GM controller, oWord18.3	!IN(0).6
Bit4 = M2G_QcmNotMoving	!In GM controller, oWord18.4	!may or may not use
Bit5 = M2G_QcmPresent	!In GM controller, oWord18.5	!from Buffer 18
QCM_StatusWord = Bit0 + (Bit1*2) + (Bit2*4) + (Bit3*8) + (Bit4*16) + (Bit5*32)
iw_rqst(5) = QCM_StatusWord !Value to be written, used with 16-write holding register
outp(17, iw_rqst, 0, 6) !send request to Slave
INP(17, iw_rspn, 0, 6, 500) ! Receive Slave response - CJR changed 10 to 4 to match OUTP

wait 1000

goto AA

STOP
#17
!Buffer 17
!For READ and WRITE to the Beckhoff GM controller

!READ - declaration
Global 	Int 	G2M_MaxSpGV, G2M_MaxSpCAR, G2M_MaxSpPosLim, G2M_SafeToMoveQcm	!See Buffer 18 
Global Int		G2M_OpenESV_Status, G2M_LeakValveAllowedOpen, G2M_SafeToMoveInj, G2M_MainShutterOpen, G2M_BfmRetracted, G2M_QcmLeCAR, G2M_PrepareForTransfer	!See Buffer 12
Global Int		G2M_InjClearOfRobot, M2G_CARZ, CARZ_AxisNum; 	
Global int		G2M_OkToMoveCAR,	G2M_CAR_UP, 	G2M_CAR_DOWN
Global INT		NO_POGB_FLT_TO_CLOSE_ESV

INT ir_rqst_Q(100) ! Request array
INT ir_rspn_Q(100) ! Response array
INT ir_rqst_O(100) ! Request array
INT ir_rspn_O(100) ! Response array
INT ir_rqst_Z(100) ! Request array
INT ir_rspn_Z(100) ! Response array
!real r_rqst(100) !Request array
!real r_rspn(100) !Response array
int r_reg!Slave 32-bit register
INT result1_Q, result2_Q ! Function result
INT result1_O, result2_O ! Function result
INT result1_Z, result2_Z ! Function result
INT slave_IP, slave_channel, slave_id, Timeout
INT Direction_Write, Direction_Read
Direction_Write = 16
Direction_Read = 3
Timeout = 1500
slave_channel = 17 ! Can be 17, 18, or 19
slave_IP = 192 + 168*POW(2, 8) + 1*POW(2,16) + 13*POW(2,24) ! = 192.168.1.13
slave_id = 1 ! 1 to 247 for MODBUS Slave IDs
! Open the Modbus TCP connection with slave device
SETCONF(308, slave_channel, 0) ! Close
wait 2000
SETCONF(308, slave_channel, slave_IP) ! Open
wait 2000
!SETCONF(309,slave_channel,1)
INP(slave_channel) ! Purges off all characters received before in the channel
global real tmpMeasured(8)
!WRITE - declaration
INT M2G_HeartBeatON
GLOBAL INT	M2G_OpenESV, M2G_LeakValveClosed
Global Int 	M2G_QcmLeGV, M2G_QcmLeCAR, M2G_QcmGtCAR, M2G_QcmAtHome, M2G_QcmNotMoving, M2G_QcmPresent	!See Buffer 18
Global Int	M2G_InjClearOfMainShutter, M2G_InjClearOfBFM, M2G_InjClearOfRobot, M2G_InjClearOfQCM, M2G_InjPresent	!See Buffer 12
Int	Bit0, Bit1, Bit2, Bit3, Bit4, Bit5, Bit6, Bit7, Bit8, Bit9, Bit10	!Bit placeholders for conversion to integer
Global int	QCM_StatusWord, Ozn_StatusWord, CARZ_StatusWord	!combined M2G states; bit0 is GV, bit 1 is CAR, bit 2 is PosLim
int iw_rqst_Q(100) !Request array
int iw_rspn_Q(100) !Response array
int iw_rqst_O(100) !Request array
int iw_rspn_O(100) !Response array
int iw_rqst_Z(100) !Request array
int iw_rspn_Z(100) !Response array
int w_reg!Slave 32-bit register

!READ
ir_rqst_Q(0)=slave_id !Slave ID
ir_rqst_Q(1)=Direction_Read !3-Read Holding Register; 16-write holding register; 
ir_rqst_Q(2)=288 !Starting Address, STATUS WORD. This is Word32 in Beckhoff PRG program - WORD 32
ir_rqst_Q(3)=2 !Quantity of Registers

ir_rqst_O(0)=slave_id !Slave ID
ir_rqst_O(1)=Direction_Read !3-Read Holding Register; 16-write holding register; 
ir_rqst_O(2)=289 !Starting Address, STATUS WORD. This is Word32 in Beckhoff PRG program - WORD 33
ir_rqst_O(3)=2 !Quantity of Registers


ir_rqst_Z(0)=slave_id !Slave ID
ir_rqst_Z(1)=Direction_Read !3-Read Holding Register; 16-write holding register; 
ir_rqst_Z(2)=290 !Starting Address, STATUS WORD. This is Word32 in Beckhoff PRG program - WORD 33
ir_rqst_Z(3)=2 !Quantity of Registers


!r_rqst(0)=slave_id !Slave ID
!r_rqst(1)=Direction_Read !3-Read Holding Register;; 16-write holding register; 
!r_rqst(2)=100 !Starting Address, MEASURED
!r_rqst(3)=2 !Quantity of Registers
!r_rqst(4)=4 !Byte Count
!r_rqst(5)=0.0 !Value to be written, used with 16-write holding register

!WRITE
iw_rqst_Q(0)=1 !Slave ID
iw_rqst_Q(1)=16 !3-Read Holding Register;; 16-write holding register; 
iw_rqst_Q(2)=800 !Starting Address, MEASURED. This is Word32 in Beckhoff usrcmds - WORD 32
iw_rqst_Q(3)=1 !Quantity of Registers
iw_rqst_Q(4)=2 !Byte Count
iw_rqst_Q(5)=5.126 !Value to be written, used with 16-write holding register

iw_rqst_O(0)=1 !Slave ID
iw_rqst_O(1)=16 !3-Read Holding Register;; 16-write holding register; 
iw_rqst_O(2)=801 !Starting Address, MEASURED. This is Word32 in Beckhoff usrcmds - WORD 33
iw_rqst_O(3)=1 !Quantity of Registers
iw_rqst_O(4)=2 !Byte Count
iw_rqst_O(5)=5.126 !Value to be written, used with 16-write holding register

iw_rqst_Z(0)=1 !Slave ID
iw_rqst_Z(1)=16 !3-Read Holding Register;; 16-write holding register; 
iw_rqst_Z(2)=802 !Starting Address, MEASURED. This is Word32 in Beckhoff usrcmds - WORD 33
iw_rqst_Z(3)=1 !Quantity of Registers
iw_rqst_Z(4)=2 !Byte Count
iw_rqst_Z(5)=5.126 !Value to be written, used with 16-write holding register

AA:
! Establish heartbeat to GM controller
IF M2G_HeartBeatON = 0 
	M2G_HeartBeatON = 1		! Turn ON the heart beat signal, then turn OFF after 2nd (wait 100) to create 200 mS pulse to be monitored by GM PLC
ELSE
	M2G_HeartBeatON = 0
END

!Leak valve closed status
M2G_LeakValveClosed = DINPUT0.9 !1 = leak valve closed - see D Buffer

!READ from GM PLC for QCM signals (word 32 in PLC)
IF M2G_QcmPresent=1
!outp(slave_channel, r_rqst, 0, 4) !send request to Slave
!reg = inp(slave_channel, r_rspn, 0, 5, 500) !receive Slave response

result1_Q = OUTP(slave_channel, ir_rqst_Q, 0, 4) ! Send request through channel 17
result2_Q = INP(slave_channel, ir_rspn_Q, 0, 4) ! Receive Slave response - CJR changed 10 to 4 to match OUTP
!if result2 > 0
!if i_rspn(3)&0x8000; i_rspn(3)=i_rspn(3)|0xFFFF0000; end
!result2=i_rspn(3)*256*256 + i_rspn(4)
!end
if ir_rspn_Q(1)=3									! in GM PLC_PRG, status word 32
	G2M_MaxSpGV = ir_rspn_Q(3).0
	G2M_MaxSpCAR = ir_rspn_Q(3).1
	G2M_MaxSpPosLim = ir_rspn_Q(3).2
	G2M_SafeToMoveQcm = ir_rspn_Q(3).3
	G2M_InjClearOfRobot = ir_rspn_Q(3).4
end
END ! if qcm present

wait 100

!READ from GM PLC for OZONE signals (word 33 in PLC)
IF M2G_InjPresent=1
result1_O = OUTP(slave_channel, ir_rqst_O, 0, 4) ! Send request through channel 17
result2_O = INP(slave_channel, ir_rspn_O, 0, 4) ! Receive Slave response - CJR changed 10 to 4 to match OUTP
!if result2 > 0
!if i_rspn(3)&0x8000; i_rspn(3)=i_rspn(3)|0xFFFF0000; end
!result2=i_rspn(3)*256*256 + i_rspn(4)
!end
if ir_rspn_O(1)=3									! in GM PLC_PRG, status word 33
	G2M_OpenESV_Status 			= ir_rspn_O(3).1
	G2M_LeakValveAllowedOpen	= ir_rspn_O(3).2
	G2M_SafeToMoveInj 			= ir_rspn_O(3).4
	G2M_MainShutterOpen 		= ir_rspn_O(3).5
	G2M_BfmRetracted 			= ir_rspn_O(3).6
	G2M_QcmLeCAR 				= ir_rspn_O(3).7
	G2M_PrepareForTransfer 		= ir_rspn_O(3).8
end
END ! if inj present

wait 100

!READ from GM PLC for CAR Z signals (word 34 in PLC)
IF M2G_CARZ=1
result1_Z = OUTP(slave_channel, ir_rqst_Z, 0, 4) ! Send request through channel 17
result2_Z = INP(slave_channel, ir_rspn_Z, 0, 4) ! Receive Slave response - CJR changed 10 to 4 to match OUTP
!if result2 > 0
!if i_rspn(3)&0x8000; i_rspn(3)=i_rspn(3)|0xFFFF0000; end
!result2=i_rspn(3)*256*256 + i_rspn(4)
!end
if ir_rspn_Z(1)=3									! in GM PLC_PRG, status word 34

	G2M_OkToMoveCAR	= ir_rspn_Z(3).8
	G2M_CAR_UP		= ir_rspn_Z(3).9
	G2M_CAR_DOWN	= ir_rspn_Z(3).10
end
END ! if inj present

wait 100
!WRITE to GM PLC (MAC QCM signals)
IF M2G_QcmPresent=1
!create the status word by combining the individual state integers
!QCM_StatusWord = M2G_QcmLePosA + M2G_QcmLePosB * POW(10,M2G_QcmLePosB) + M2G_QcmLePosC * POW(100,M2G_QcmLePosC)
Bit0 = M2G_QcmLeGV		!In GM controller (Beckhoff), oWord32.0
Bit1 = M2G_QcmLeCAR		!In GM controller (Beckhoff), oWord32.1
Bit2 = M2G_QcmGtCAR		!In GM controller (Beckhoff), oWord32.2
Bit3 = M2G_QcmAtHome	!In GM controller (Beckhoff), oWord32.3	- NOT home switch because switch not on if offset is used; this is Measured < 0.1
Bit4 = M2G_QcmNotMoving	!In GM controller (Beckhoff), oWord32.4	!may or may not use
Bit5 = M2G_QcmPresent	!In GM controller (Beckhoff), oWord32.5	
Bit6 = 0				!In GM controller (Beckhoff), oWord32.6 
Bit7 = 0				!In GM controller (Beckhoff), oWord32.7 
Bit8 = 0				!In GM controller (Beckhoff), oWord32.8 
Bit9 = 0				!In GM controller (Beckhoff), oWord32.9 

QCM_StatusWord = Bit0 + (Bit1*2) + (Bit2*4) + (Bit3*8) + (Bit4*16) + (Bit5*32) + (Bit6*64) + (Bit7*128) + (Bit8*256) + (Bit9*512)
iw_rqst_Q(5) = QCM_StatusWord !Value to be written, used with 16-write holding register
OUTP(17, iw_rqst_Q, 0, 6) !send request to Slave
INP(17, iw_rspn_Q, 0, 6) ! Receive Slave response - CJR changed 10 to 4 to match OUTP
END ! if qcm present

wait 100

!WRITE to GM PLC (DMC Ozone signals)
IF M2G_InjPresent=1
!create the status word by combining the individual state integers
!QCM_StatusWord = M2G_QcmLePosA + M2G_QcmLePosB * POW(10,M2G_QcmLePosB) + M2G_QcmLePosC * POW(100,M2G_QcmLePosC)
Bit0 = M2G_HeartBeatON									!In GM controller (Beckhoff), oWord33.0
!Bit0 = 1									!In GM controller (Beckhoff), oWord33.0
Bit1 = M2G_OpenESV									!In GM controller (Beckhoff), oWord33.1
Bit2 = M2G_LeakValveClosed								!In GM controller (Beckhoff), oWord33.2
Bit3 = 0												!In GM controller (Beckhoff), oWord33.3	!IN(0).6
Bit4 = 0												!In GM controller (Beckhoff), oWord33.4	!may or may not use
Bit5 = M2G_InjClearOfQCM & MFLAGS(0).#HOME = 1 			!In GM controller (Beckhoff), oWord33.5	!from Buffer 18
Bit6 = M2G_InjClearOfMainShutter & MFLAGS(0).#HOME = 1	!In GM controller (Beckhoff), oWord33.6 !from Buffer 12
Bit7 = M2G_InjClearOfBFM & MFLAGS(0).#HOME = 1			!In GM controller (Beckhoff), oWord33.7 !from Buffer 12
Bit8 = M2G_InjClearOfRobot & MFLAGS(0).#HOME = 1		!In GM controller (Beckhoff), oWord33.8 !from Buffer 12
Bit9 = M2G_InjPresent									!In GM controller (Beckhoff), oWord33.9 !from Buffer 12
Bit10= NO_POGB_FLT_TO_CLOSE_ESV							!In GM controller (Beckhoff), oWord33.10 !from Buffer 11. 1=happy, 0=close ESV, ZW 10/16/2020
Ozn_StatusWord = Bit0 + (Bit1*2) + (Bit2*4) + (Bit3*8) + (Bit4*16) + (Bit5*32) + (Bit6*64) + (Bit7*128) + (Bit8*256) + (Bit9*512) + (Bit10*1024)
iw_rqst_O(5) = Ozn_StatusWord !Value to be written, used with 16-write holding register
OUTP(17, iw_rqst_O, 0, 6) !send request to Slave
INP(17, iw_rspn_O, 0, 6) ! Receive Slave response - CJR changed 10 to 4 to match OUTP
END ! if inj present
wait 100


!WRITE to GM PLC (car z MOTION signals)
IF M2G_CARZ=1
!create the status word by combining the individual state integers
!QCM_StatusWord = M2G_QcmLePosA + M2G_QcmLePosB * POW(10,M2G_QcmLePosB) + M2G_QcmLePosC * POW(100,M2G_QcmLePosC)
Bit0 = 0									!In GM controller (Beckhoff), oWord34.0
!Bit0 = 1									!In GM controller (Beckhoff), oWord34.0
Bit1 = 0									!In GM controller (Beckhoff), oWord34.1
Bit2 = 0									!In GM controller (Beckhoff), oWord34.2
Bit3 = 0									!In GM controller (Beckhoff), oWord34.3	
Bit4 = 0									!In GM controller (Beckhoff), oWord34.4	
Bit5 = 0  									!In GM controller (Beckhoff), oWord34.5	!from Buffer 18
Bit6 = 0									!In GM controller (Beckhoff), oWord34.6 !from Buffer 12
If ((tmpMeasured(1)<5.0)&(tmpMeasured(1)>-5.0))|(tmpMeasured(1)<-175.0)|(tmpMeasured(1)>175.0)
	if (tmpMeasured(0)<10)
		Bit7 = 1								!Ok to move down the heater
	else
		Bit7 = 0
	end
else
	Bit7 = 1
end

Bit8 = SAFIN(CARZ_AxisNum).#LL = 1 | (tmpMeasured (CARZ_AxisNum)<0.05)		!CAR is Up In GM controller (Beckhoff), oWord34.8 !from Buffer 12
Bit9 = MST(CARZ_AxisNum).4 									!CAR is not moving In GM controller (Beckhoff), oWord34.9 !from Buffer 12

CARZ_StatusWord = Bit0 + (Bit1*2) + (Bit2*4) + (Bit3*8) + (Bit4*16) + (Bit5*32) + (Bit6*64) + (Bit7*128) + (Bit8*256) + (Bit9*512)
iw_rqst_Z(5) = CARZ_StatusWord !Value to be written, used with 16-write holding register
OUTP(17, iw_rqst_Z, 0, 6) !send request to Slave
wait 50
INP(17, iw_rspn_Z, 0, 6) ! Receive Slave response - CJR changed 10 to 4 to match OUTP
END ! if inj present
wait 100

goto AA


STOP
#18
! This buffer is exclusively for managing the interlocks for the QCM.
! Change log
! DATE		 ECO#	Reason for Change			Remarks
! 2/11/2016	 N/A	Initial release				CJR
! 


Global 	Int 	G2M_MaxSpGV, G2M_MaxSpCAR, G2M_MaxSpPosLim, G2M_SafeToMoveQcm	!Flags from GM controller indicating current QCM limit position 
Global 	Int 	M2G_QcmLeGV, M2G_QcmLeCAR, M2G_QcmGtCAR, M2G_QcmAtHome, M2G_QcmNotMoving, M2G_QcmPresent	!Flags back to the GM controller indicating QCM position status
Global 	Real 	dQcmLimit(8)!This is the value that is evaluated in Buffer 31 before a move is executed
Global 	Int		QcmAxis		!Set at the GUI; can only be 0 through 7 (only 1 QCM per MAC); -1 = No QCM
		Int 	i
Global	Real	Measured(8), SP(8), SP_Modbus(8)

!Initialize the QCM limits (for all axes) to the corresponding dPositiveLimit value
i=0
Loop 8
dQcmLimit(i) = dPositiveLimit(i)
i=i+1
end

AA:

QcmAxis = iQcmAxis(0)	!iQcmAxis is an array so it can be written to flash

if (QcmAxis >= 0) & (QcmAxis <= 7)
	M2G_QcmPresent = 1
else
	M2G_QcmPresent = 0
end

iPreventMove(QcmAxis) = ^G2M_SafeToMoveQcm	!this is used to take the "safe to move" signal from GM and interlock motion in buffer 31

Measured(0)=Measured0	!make an array of measured values to keep code modular
Measured(1)=Measured1
Measured(2)=Measured2
Measured(3)=Measured3
Measured(4)=Measured4
Measured(5)=Measured5              
Measured(6)=Measured6
Measured(7)=Measured7

!Set the QCM limit based on the interlock flags from the GM controller
!NOTE - We don't want this to execute if there is no QCM (setting limit to 0.0 is a problem in buffer 31)
if 		G2M_MaxSpGV & ^G2M_MaxSpCAR & ^G2M_MaxSpPosLim		!The GM controller limits QCM travel to behind the gate valve
	dQcmLimit(QcmAxis) = dInterlockPos(QcmAxis)				!The user entered value for PosA is assigned to the QCM limit
elseif 	G2M_MaxSpCAR & ^G2M_MaxSpGV & ^G2M_MaxSpPosLim		!The GM controller limits QCM travel to outside the CAR location
	dQcmLimit(QcmAxis) = dSecondSetpointPos(QcmAxis)				!The user entered value for PosB is assigned to the QCM limit
elseif 	G2M_MaxSpPosLim & ^G2M_MaxSpGV & ^G2M_MaxSpCAR		!The GM controller limits QCM travel to the center of the CAR
	dQcmLimit(QcmAxis) = dPositiveLimit(QcmAxis)				!The user entered value for PosC is assigned to the QCM limit
else
	!dQcmLimit(QcmAxis) = 0.0								!If there is no signal from GM, restrict QCM to home position
end

!Look for the QCM measured position to be beyond the QCM limit (may happen when a more restrictive limit is imposed).
!If this is found, then set the Modbus Sp value to the limit so the QCM is retracted to this safe location. Change the Modbus
!variable so that Molly is updated.
if (Measured(QcmAxis) > dQcmLimit(QcmAxis)) & (M2G_QcmPresent=1)		!only check the axis associated with the QCM
	if 		QcmAxis = 0								!change Sp0 if the QCM is on axis 0
		Sp0 = dQcmLimit(QcmAxis)
	elseif 	QcmAxis = 1							!and so on...
		Sp1 = dQcmLimit(QcmAxis)
	elseif 	QcmAxis = 2						
		Sp2 = dQcmLimit(QcmAxis)
	elseif 	QcmAxis = 3						
		Sp3 = dQcmLimit(QcmAxis)
	elseif 	QcmAxis = 4						
		Sp4 = dQcmLimit(QcmAxis)
	elseif 	QcmAxis = 5						
		Sp5 = dQcmLimit(QcmAxis)
	elseif 	QcmAxis = 6						
		Sp6 = dQcmLimit(QcmAxis)
	elseif 	QcmAxis = 7						
		Sp7 = dQcmLimit(QcmAxis)
	end
end

!compare the measured position of the QCM axis to the limits, and set flags accordingly.  These are sent to the GM controller
!check measured against Pos A
if Measured(QcmAxis) <= dInterlockPos(QcmAxis) & SP_Modbus(QcmAxis) <= dInterlockPos(QcmAxis) & MFLAGS(QcmAxis).#HOME = 1
	M2G_QcmLeGV = 1
else
	M2G_QcmLeGV = 0
end
!check measured against Pos B	
if Measured(QcmAxis) <= dSecondSetpointPos(QcmAxis) & SP_Modbus(QcmAxis) <= dSecondSetpointPos(QcmAxis) & MFLAGS(QcmAxis).#HOME = 1
	M2G_QcmLeCAR = 1
else
	M2G_QcmLeCAR = 0
end
!check measured against Pos C
if Measured(QcmAxis) > dSecondSetpointPos(QcmAxis) | SP_Modbus(QcmAxis) > dSecondSetpointPos(QcmAxis) & MFLAGS(QcmAxis).#HOME = 1
	M2G_QcmGtCAR = 1
else
	M2G_QcmGtCAR = 0
end

!check for QCM motor moving
if MST(QcmAxis).4 = 1	!@@@ = 1 means motor is not moving
	M2G_QcmNotMoving = 1
else
	M2G_QcmNotMoving = 0
end
	
!check for QCM at home switch
!M2G_QcmAtHome = IN(0).QcmAxis		! it was 5 on Qatar 
if Measured(QcmAxis) <= 0.1 & MFLAGS(QcmAxis).#HOME = 1
	M2G_QcmAtHome = 1
else
	M2G_QcmAtHome = 0
end


GOTO AA

STOP




#20
! service prog for home pulse output on DOUT 0.
!Change log
! DATE		ECO#	Reason for Change			Remarks
!9/21/2015	N/A		ini release					ZW

int iCARAxis(2), iNumOfCARs

int bFoundCAR(2)
!Find the first CAR axis to output home pulse
int k
k = 0
bFoundCAR(0) = 0
bFoundCAR(1) = 0
iCARAxis(0)=0
iCARAxis(1)=0
iNumOfCARs=0
LOOP MAXNUMOFAXIS
	if((iType(k) >3000)&(iType(k)<5000))
 		iCARAxis(iNumOfCARs) = k
		bFoundCAR(iNumOfCARs) = 1
		iNumOfCARs = iNumOfCARs + 1
		if(iNumOfCARs=2)
			GOTO SS
		end	
	END
	k=k+1
END

SS:
if (iNumOfCARs > 0)
	if(iType(iCARAxis(0))>=4004)&(iType(iCARAxis(0))<=4008) !GEN20A, MZ, HVM and GEN10 Only
		STOP 29
		Start 29, 1
	end
	real tmp, tmpstart(2), tmpend(2)
	k=0
	LOOP iNumOfCARs
		tmpstart(k) = 3.6*dHomePulseDelay(iCARAxis(k))
		tmpend(k) = 3.6*(dHomePulseDelay(iCARAxis(k))+dHomePulseWidth(iCARAxis(k)))
		k=k+1
	end

	AA:
		wait(100)
	GOTO AA
END
STOP

!1st CAR==========================
on	(bFoundCAR(0)=1) & (MST(iCARAxis(0)).4 = 0)	& ((FPOS(iCARAxis(0))-360*FLOOR(FPOS(iCARAxis(0))/360))>=tmpstart(0)) & ((FPOS(iCARAxis(0))-360*FLOOR(FPOS(iCARAxis(0))/360))<tmpend(0)) 
! 0 = moving; 1 = in-position
	if (OUT(0).0=0)
		OUT(0).0=1
	end	
RET

on	(bFoundCAR(0)=1) & (MST(iCARAxis(0)).4 = 0)	& ((FPOS(iCARAxis(0))-360*FLOOR(FPOS(iCARAxis(0))/360))>=tmpend(0)) 
! 0 = moving; 1 = in-position
	if (OUT(0).0=1)
		OUT(0).0=0
	end	
RET
!2ns CAR==========================
on	(bFoundCAR(1)=1) & (MST(iCARAxis(1)).4 = 0)	& ((FPOS(iCARAxis(1))-360*FLOOR(FPOS(iCARAxis(1))/360))>=tmpstart(1)) & ((FPOS(iCARAxis(1))-360*FLOOR(FPOS(iCARAxis(1))/360))<tmpend(1)) 
! 0 = moving; 1 = in-position
	if (OUT(0).1=0)
		OUT(0).1=1
	end	
RET

on	(bFoundCAR(1)=1) & (MST(iCARAxis(1)).4 = 0)	& ((FPOS(iCARAxis(1))-360*FLOOR(FPOS(iCARAxis(1))/360))>=tmpend(1)) 
! 0 = moving; 1 = in-position
	if (OUT(0).1=1)
		OUT(0).1=0
	end	
RET

on	(bFoundCAR(0)=1) & (iType(iCARAxis(0)) = 4008)	& ((abs( FPOS(iCARAxis(0)) - 360*FLOOR(FPOS(iCARAxis(0))/360))<=0.3) | (abs( FPOS(iCARAxis(0)) - 360*FLOOR(FPOS(iCARAxis(0))/360))>=359.7))
! output encoder 0 position for GEN20MZ
	if (OUT(0).2=0)
		OUT(0).2=1
	end	
RET


on	(bFoundCAR(0)=1) & (iType(iCARAxis(0)) = 4008)	& ((abs( FPOS(iCARAxis(0)) - 360*FLOOR(FPOS(iCARAxis(0))/360))>0.3) & (abs( FPOS(iCARAxis(0)) - 360*FLOOR(FPOS(iCARAxis(0))/360))<359.7))
! output encoder 0 position for GEN20MZ
	if (OUT(0).2=1)
		OUT(0).2=0
	end	
RET
#22
! use analog command for AVP axis. Upto 4 axis
!MFLAGS(2).#DEFCON = 1
!CONNECT RPOS(2) = APOS(2) + 0.001*dsign(RVEL(2), 0, 0)
IF UseAnalogCmd(0) = 1
	Global Int iAxisGood(8)
	int AvgFilterPole
	real PosCmd(4)
	real NewCmd(4)	!for diag
	real PrePosCmd(4)
	real CmdThreshold
	real SamplingTime	!ms
	real CmdWindowTimeLength	!ms
	CmdThreshold = UseAnalogCmd(1)
	AvgFilterPole = 40
	SamplingTime = 10
	CmdWindowTimeLength = UseAnalogCmd(2)
	!initialize the Analog commands
	PosCmd(0)=0
	PosCmd(1)=0
	PosCmd(2)=0
	PosCmd(3)=0
	BLOCK
	Loop AvgFilterPole
		PosCmd(0)= PosCmd(0) + AIN(0)
		PosCmd(1)= PosCmd(1) + AIN(1)
		PosCmd(2)= PosCmd(2) + AIN(2)
		PosCmd(3)= PosCmd(3) + AIN(3)
		wait SamplingTime
	end
	END
	PosCmd(0)= PosCmd(0)/AvgFilterPole
	PosCmd(1)= PosCmd(1)/AvgFilterPole
	PosCmd(2)= PosCmd(2)/AvgFilterPole
	PosCmd(3)= PosCmd(3)/AvgFilterPole
	PrePosCmd(0)= PosCmd(0)
	PrePosCmd(1)= PosCmd(1)
	PrePosCmd(2)= PosCmd(2)
	PrePosCmd(3)= PosCmd(3)
	
	
	
	
	AA:
		BLOCK
			PosCmd(0)=0
			PosCmd(1)=0
			PosCmd(2)=0
			PosCmd(3)=0
			Loop AvgFilterPole
				PosCmd(0)= PosCmd(0) + AIN(0)
				PosCmd(1)= PosCmd(1) + AIN(1)
				PosCmd(2)= PosCmd(2) + AIN(2)
				PosCmd(3)= PosCmd(3) + AIN(3)
				wait SamplingTime
			end
		
			PosCmd(0)= PosCmd(0)/AvgFilterPole
			PosCmd(1)= PosCmd(1)/AvgFilterPole
			PosCmd(2)= PosCmd(2)/AvgFilterPole
			PosCmd(3)= PosCmd(3)/AvgFilterPole
			NewCmd(0) = PosCmd(0)
			NewCmd(1) = PosCmd(1)
			NewCmd(2) = PosCmd(2)
			NewCmd(3) = PosCmd(3)
		END	
		BLOCK
			if (iType(0)>1000) & (iType(0) <2000) & (abs(PosCmd(0)-PrePosCmd(0))>CmdThreshold)& iAxisGood(0)
				wait CmdWindowTimeLength
				PosCmd(0)=0
				Loop AvgFilterPole
					PosCmd(0)= PosCmd(0) + AIN(0)
					wait SamplingTime
				end
				PosCmd(0)= PosCmd(0)/AvgFilterPole
				NewCmd(0) = PosCmd(0)
				Sp0 = 0.01 * PosCmd(0) * dPositiveLimit(0)- AnalogChannelOffset(0)
				PrePosCmd(0)=PosCmd(0)
			end
			
		END
		BLOCK
			if (iType(1)>1000) & (iType(1) <2000) & (abs(PosCmd(1)-PrePosCmd(1))>CmdThreshold)& iAxisGood(1)
				wait CmdWindowTimeLength
				PosCmd(1)=0
				Loop AvgFilterPole
					PosCmd(1)= PosCmd(1) + AIN(1)
					wait SamplingTime
				end
				PosCmd(1)= PosCmd(1)/AvgFilterPole
				NewCmd(1) = PosCmd(1)
				Sp1 = 0.01 * PosCmd(1) * dPositiveLimit(1)- AnalogChannelOffset(1)
				PrePosCmd(1)=PosCmd(1)

			end

		end
		BLOCK
			if (iType(2)>1000) & (iType(2) <2000)& (abs(PosCmd(2)-PrePosCmd(2))>CmdThreshold)& iAxisGood(2)
				wait CmdWindowTimeLength
				PosCmd(2)=0
				Loop AvgFilterPole
					PosCmd(2)= PosCmd(2) + AIN(2)
					wait SamplingTime
				end
				PosCmd(2)= PosCmd(2)/AvgFilterPole
				NewCmd(2) = PosCmd(2)

				Sp2 = 0.01 * PosCmd(2) * dPositiveLimit(2)- AnalogChannelOffset(2)
				PrePosCmd(2)=PosCmd(2)
			end
		end
		BLOCK
			if (iType(3)>1000) & (iType(3) <2000) & (abs(PosCmd(3)-PrePosCmd(3))>CmdThreshold)& iAxisGood(3)
				wait CmdWindowTimeLength
				PosCmd(3)=0
				Loop AvgFilterPole
					PosCmd(3)= PosCmd(3) + AIN(3)
					wait SamplingTime
				end
				PosCmd(3)= PosCmd(3)/AvgFilterPole
				NewCmd(3) = PosCmd(3)
				Sp3 = 0.01 * PosCmd(3) * dPositiveLimit(3)- AnalogChannelOffset(3)
				PrePosCmd(3)=PosCmd(3)
			end
		end
	!wait 1000
	GOTO AA

END

STOP



#23
! service prog for robot auto home
!Change log
! DATE		ECO#	Reason for Change					Remarks
!12/21/2015	N/A		move function from CUI to Buffer	ZW

Global int RobotExtIsHoming
Global int RobotRotIsHoming
Global int FineRotHoming
int iTimeout
		
STOP 30		! stop ext homing buffer if running
STOP 28		! stop rot homing buffer if running
KILL (ROBOTAXISNUMH, ROBOTAXISNUML) ! stop ,otor 0 and 2 motion if any

START 30,1 ! start buffer 30 to home extension 


iTimeout = 1;
while RobotExtIsHoming
	WAIT 1000
	iTimeout=iTimeout +1
	if iTimeout > 900 !15 minutes time out
		STOP 30		! stop ext homing buffer if running
		STOP 28		! stop rot homing buffer if running
		KILL (ROBOTAXISNUMH, ROBOTAXISNUML) ! stop motor 0 and 2 motion if any
		GOTO EXITING 
	end
end	

START 28,1 ! start buffer 28 to home rotation 

iTimeout = 1;
while RobotRotIsHoming
	WAIT 1000
	FineRotHoming = 0
	iTimeout=iTimeout +1
	if iTimeout > 900 !15 minutes time out
		STOP 30		! stop ext homing buffer if running
		STOP 28		! stop rot homing buffer if running
		KILL (ROBOTAXISNUMH, ROBOTAXISNUML) ! stop motor 0 and 2 motion if any
		GOTO EXITING 
	end
end	


START 30,1 ! start buffer 30 to home extension 


iTimeout = 1;
while RobotExtIsHoming
	WAIT 1000
	iTimeout=iTimeout +1
	if iTimeout > 900 !15 minutes time out
		STOP 30		! stop ext homing buffer if running
		STOP 28		! stop rot homing buffer if running
		KILL (ROBOTAXISNUMH, ROBOTAXISNUML) ! stop motor 0 and 2 motion if any
		GOTO EXITING 
	end
end	

START 28,1 ! start buffer 28 to home rotation 





EXITING:

STOP
#24
!#/ Controller version = 2.27
!#/ Date = 12/28/2015 10:33 AM
!#/ User remarks = 
!#24
! motor cummt and axis homing routine for all types except for robot arms

!Change log
! DATE		ECO#	Reason for Change			Remarks
!9/21/2015	N/A		ini release					ZW
!12/28/15	N/A		tuning homing				ZW


global int SP_Axis                      ! Axis to be commutated
Global int bFoundLLELE
if (IN(0).5=0) &(bFoundLLELE = 1)&(LLELEAXIS = SP_Axis)
	DISP "LL door open!"
else
	if SP_Axis = 0
		STOP 0
		START 0,1
	elseif SP_Axis = 1
		STOP 1
		START 1,1
	elseif SP_Axis = 2
		STOP 2
		START 2,1
	elseif SP_Axis = 3
		STOP 3
		START 3,1
	elseif SP_Axis = 4
		STOP 4
		START 4,1
	elseif SP_Axis = 5
		STOP 5
		START 5,1
	elseif SP_Axis = 6
		STOP 6
		START 6,1
	elseif SP_Axis = 7
		STOP 7
		START 7,1
	end
end

STOP
#25
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


LOOP 2


SP_Pitch=SLCPRD(SP_Axis)/SLCNP(SP_Axis)*EFAC(SP_Axis)

!******************************************************************************* 
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

Move_Detent:
ptp/rv (SP_Axis), SP_Direction*SP_Pitch/2.,SP_Search_Vel
till ^AST(SP_Axis).#MOVE; wait SP_Settle_Time
call Limit_Check
disable(SP_Axis)

MFLAGS(SP_Axis).9=1               ! Set commutation state 
DCOM(SP_Axis) = 0

! If motor is to be left enabled after startup process delete the following line:

Finish:
SP_InCommutationStartup=0              ! commutation startup is finished
If SP_Fail_Code=0; disp " Axis %i  Commutation Startup Finished.", SP_Axis
else disp "   Commutation Startup Failed."; disp "   Failure Code = %i",SP_Fail_Code; end

SP_Axis=ROBOTAXISNUML 
END
STOP

!******************************************************************************* 
!   The following routine move the motor away from limit switches 

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

ON ((SP_InCommutationStartup = 1) & ((FAULT(SP_Axis)&0x30f80)>0 | (S_FAULT&0x30000000)>0))
SP_Fail_Code=1; DISABLE(SP_Axis); DCOM(SP_Axis)=0
CALL Finish 
RET
#26
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



AA:
!BLOCK
if iType(0)>6000					!only execute when robot exists

	
	
!	if ((iType(0)= 6001) |(iType(0)= 6003)|(iType(0)= 6005) |(iType(0)= 6007)|(iType(0)= 6009))	!this is a GEN20A Robot arm extension axis

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
					StatusWord0.6=0
				END	
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
GOTO AA

STOP

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


#28
! Robot Rotation Homing Program.prg
!Change log
! DATE		ECO#	Reason for Change			Remarks
!9/21/2015	N/A		ini release					ZW01
!12/21/2015	N/A		debug yellow					ZW02

GLOBAL real  RobotROTTargetPrev, RealRobotAngle
GLOBAL real  RobotExtTargetPrev
GLOBAL int RobotNormalMode
Global int RobotRotIsHoming
Global int FineRotHoming
Global Int iExtCommanded, iRotCommanded, ExecutingMoveRot, iRotMoving, ExecutingMoveExt, iExtMoving
real HomeFlagWidth, HomingStartPoint

real home_velocity, RotHomeOffset
real store_CURI0, store_CURV0, store_CURI2, store_CURV2
real distance, flagpos, flagpos0
RobotRotIsHoming = 1
FineRotHoming=1
ExecutingMoveRot = 0			!turn off moving flag
iRotCommanded = 0
iRotMoving = 0					!used in buffer 27
ExecutingMoveExt = 0			!turn off moving flag
iExtCommanded = 0				!turn off commanded flag
iExtMoving = 0					!used in buffer 27

home_velocity = 10
RotHomeOffset = 0.55!-12.12
iControlProcessState(ROBOTAXISNUML) = 14

MFLAGS(ROBOTAXISNUML).#HOME = 0


if ((MFLAGS(0).#HOME = 1)&(FPOS(0)<1))

	MFLAGS(ROBOTAXISNUML).#HOME = 0

	!reduce the motor current saturations
	store_CURI0 = XCURI(0)
	store_CURV0 = XCURV(0)
	store_CURI2 = XCURI(ROBOTAXISNUML)
	store_CURV2 = XCURV(ROBOTAXISNUML)
	XCURI(0) = dHomingTorq(0)
	XCURV(0) = dHomingTorq(0)
	XCURI(ROBOTAXISNUML) = dHomingTorq(ROBOTAXISNUML)
	XCURV(ROBOTAXISNUML) = dHomingTorq(ROBOTAXISNUML)

	ENABLE (ROBOTAXISNUML)          ! Enable the iAxis drive
	ENABLE (0)          ! Enable the iAxis drive
!	SET FPOS(0) = 0
!	SET FPOS(ROBOTAXISNUML) = 0 !FPOS(iAxis) - (IND(iAxis) - home_offset)
!	TPOS(0) = 0!added by ZW02
!	TPOS(ROBOTAXISNUML) = 0!added by ZW02
	MFLAGS(0).#DEFCON = 1
	MFLAGS(ROBOTAXISNUML).#DEFCON = 1
	wait 1000
	MFLAGS(0).#DEFCON = 0
	CONNECT RPOS(0) = APOS(ROBOTAXISNUML)
	DEPENDS 0, ROBOTAXISNUML

	if iHomingMethod(ROBOTAXISNUML)=1

		!If flag is on, back off
		if (iType(0)=6009)	
			IF  ( IN0.0 = 0)
				PTP/erv (ROBOTAXISNUML), 5, dHomingVel(ROBOTAXISNUML)*3             
			end
		else	
			IF  ( IN0.0 = 1)
				PTP/erv (ROBOTAXISNUML), 5, dHomingVel(ROBOTAXISNUML)*3             
			end
		end

		!Searching the home flag, pulse rising edge
		JOG/v ROBOTAXISNUML, -dHomingVel(ROBOTAXISNUML)             
		HomingStartPoint = RealRobotAngle
		SEARCHFLAG:
		if abs(HomingStartPoint - RealRobotAngle) > 360
			iControlProcessState(ROBOTAXISNUML) = 404
			iControlProcessState(ROBOTAXISNUMH) = 404
			HALT (ROBOTAXISNUMH, ROBOTAXISNUML)
			MFLAGS(ROBOTAXISNUMH).3 = 0
			MFLAGS(ROBOTAXISNUML).3 = 0
			STOP 23
			GOTO EXITROTHM
		end
		if (iType(0)=6009)	
			TILL ( IN0.0 = 0) | (abs(HomingStartPoint - RealRobotAngle) > 360)
		else
			TILL ( IN0.0 = 1) | (abs(HomingStartPoint - RealRobotAngle) > 360)
		end
		flagpos0 = FPOS(ROBOTAXISNUML)

		
		if (iType(0)=6009)	
			TILL ( IN0.0 = 1) | (abs(HomingStartPoint - RealRobotAngle) > 360)
		else
			TILL ( IN0.0 = 0) | (abs(HomingStartPoint - RealRobotAngle) > 360)
		end
		flagpos = FPOS(ROBOTAXISNUML)

		HomeFlagWidth=abs(flagpos0-flagpos)
		if (iType(0)= 6003)	!GEN200
			if (HomeFlagWidth>2.0)|(HomeFlagWidth<1.39)	!changed from 1.6 to 2.0 10/20/2020
				GOTO SEARCHFLAG
			end
		elseif (iType(0)= 6001)	!GEN2000
			if (HomeFlagWidth>1.1)|(HomeFlagWidth<0.9)
				GOTO SEARCHFLAG
			end
		elseif (iType(0)= 6005)	!GEN20A
!			if (HomeFlagWidth>0.8)|(HomeFlagWidth<0.5)	!Sandia GEN20A #1 Special
!			if (HomeFlagWidth>1.6)|(HomeFlagWidth<1.30)
			if (HomeFlagWidth>1.8)|(HomeFlagWidth<1.30)
				GOTO SEARCHFLAG
			end
		end


		! get ready to search the leading edge
		PTP/ev (ROBOTAXISNUML), flagpos-1, dHomingVel(ROBOTAXISNUML)
		! fine search the other edge
		JOG/v ROBOTAXISNUML, 0.11             
		if (iType(0)=6009)	
			TILL ( IN0.0 = 0)
		else
			TILL ( IN0.0 = 1)
		end
		flagpos = FPOS(ROBOTAXISNUML)


		! get ready to search the other edge
		PTP/ev (ROBOTAXISNUML), flagpos0+1, dHomingVel(ROBOTAXISNUML)
		! fine search the other edge
		JOG/v ROBOTAXISNUML, -0.11             
		if (iType(0)=6009)	
			TILL ( IN0.0 = 0)
		else
			TILL ( IN0.0 = 1)
		end
		flagpos0 = FPOS(ROBOTAXISNUML)


		if (iType(0)= 6007) !GEN10
			if FineRotHoming
				PTP/ev (ROBOTAXISNUML), 0.5*(flagpos+flagpos0)+180.0+dHomeOffset(ROBOTAXISNUML), dHomingVel(ROBOTAXISNUML)*3
			else
				PTP/ev (ROBOTAXISNUML), 0.5*(flagpos+flagpos0)+dHomeOffset(ROBOTAXISNUML), dHomingVel(ROBOTAXISNUML)*3
			end
		else
			PTP/ev (ROBOTAXISNUML), 0.5*(flagpos+flagpos0)+dHomeOffset(ROBOTAXISNUML), dHomingVel(ROBOTAXISNUML)
		end
		!End of iHomingMethod(ROBOTAXISNUML)=1, home to switch
	else
		MFLAGS(ROBOTAXISNUML).#HOME = 0
	end

	!!!!!!!!!!!!!!!!!!!!!!!!!
	wait 1000

	SET FPOS(ROBOTAXISNUML) = 0 !FPOS(iAxis) - (IND(iAxis) - home_offset)
	SET FPOS(0) = 0
	TPOS(0) = 0!added by ZW02
	TPOS(ROBOTAXISNUML) = 0!added by ZW02

	MFLAGS(0).#DEFCON = 1
	XCURI(0) = store_CURI0 
	XCURV(0) = store_CURV0
	XCURI(ROBOTAXISNUML) = store_CURI2 
	XCURV(ROBOTAXISNUML) = store_CURV2
	FDEF(0).#CPE = 1
	FDEF(ROBOTAXISNUML).#CPE = 1
	if FineRotHoming
		MFLAGS(ROBOTAXISNUML).#HOME = 1
		if  (MST(0).#ENABLED)&( MST(ROBOTAXISNUML).#ENABLED)
			PTP/e (0, ROBOTAXISNUML), 0, 0  !@ZW03	!CR2016-01-12 PTP to 0.0 so the robot servos.
		end
	else
		MFLAGS(ROBOTAXISNUML).#HOME = 0
	end
	Sp0=0.0	!@@@ set Modbus register to 0.0 so no move happens after home.
	if CONTROLLERMODELNUM = 6
		Sp1=0.0	!@@@ set Modbus register to 0.0 so no move happens after home.
	else
		Sp2=0.0	!@@@ set Modbus register to 0.0 so no move happens after home.
	end
	RobotROTTargetPrev=0.0
	RobotExtTargetPrev=0.0
	! if both robot axes are homed, set status flag ON for normal running.
	! without this (turned OFF in buffer 27 if either motor is disabled) then robot is in safe mode
	! safe mode is when velocity and step size are a fraction of normal. This is to allow recovery from crash.
	if (MFLAGS(0).#HOME = 1) &  (MFLAGS(ROBOTAXISNUML).#HOME = 1)
		RobotNormalMode = 1
		iControlProcessState(ROBOTAXISNUML) = 10
		iControlProcessState(ROBOTAXISNUMH) = 10
	end
end 
EXITROTHM:
RobotRotIsHoming = 0
iExtCommanded=0
iRotCommanded=0

STOP             
#29
! Duty Cycle Filter for Car Home Switch, threshold is set to 3/30=10%
! kick off by buffer 20
!10/27/2016
int iIndex, tmpOutput, NumAnalogCAR
global int AnalogCARAxis(4)
global real AnologHomeSignalMedian(4)
global int oGmCarHome_Switch
iIndex=0
LOOP 4
	AnalogCARAxis(iIndex)= 8
	AnologHomeSignalMedian(iIndex) = 6.2
	iIndex = iIndex +1
end

iIndex=0
NumAnalogCAR = 0
Loop MAXNUMOFAXIS
	if iType(iIndex)=4007
		AnalogCARAxis(NumAnalogCAR)=iIndex
		NumAnalogCAR =	NumAnalogCAR+1
	end
	iIndex=iIndex+1
end

AA:
	iIndex=0

	Loop MAXNUMOFAXIS
		if(iType(iIndex)=4005) !HVM Only 12/16
			BLOCK
				tmpOutput=0
				LOOP 100
					tmpOutput=tmpOutput+IN(0).iIndex
					wait 1
				end
			END
			if(tmpOutput>3)
				FtdCarHomeSW.iIndex=1
			end
			if(tmpOutput=0)
				FtdCarHomeSW.iIndex=0
			end
		elseif(iType(iIndex)=4004)	!20A
			FtdCarHomeSW.iIndex= ^IN(0).iIndex
		elseif(iType(iIndex)=4006)	!GEN10
			FtdCarHomeSW.iIndex= IN(0).iIndex
		elseif(iType(iIndex)=4008)	!20MZ
			FtdCarHomeSW.iIndex= oGmCarHome_Switch
		end
		
		iIndex= iIndex+1
	end
	wait 20
GOTO AA

STOP

on  (AnalogCARAxis(0) <8 ) & ((AIN(0) > (AnologHomeSignalMedian(0)+0.2)) & (FtdCarHomeSW.AnalogCARAxis(0) = 0))
	FtdCarHomeSW.AnalogCARAxis(0) =1
ret
on  (AnalogCARAxis(0) <8 ) & ((AIN(0) < (AnologHomeSignalMedian(0)-0.2)) & (FtdCarHomeSW.AnalogCARAxis(0) = 1))
	FtdCarHomeSW.AnalogCARAxis(0) =0
ret

on  (AnalogCARAxis(1) <8 ) & ((AIN(1) > (AnologHomeSignalMedian(1)+0.2)) & (FtdCarHomeSW.AnalogCARAxis(1) = 0))
	FtdCarHomeSW.AnalogCARAxis(1) =1
ret
on  (AnalogCARAxis(1) <8 ) & ((AIN(1) < (AnologHomeSignalMedian(1)-0.2)) & (FtdCarHomeSW.AnalogCARAxis(1) = 1))
	FtdCarHomeSW.AnalogCARAxis(1) =0
ret


on  (AnalogCARAxis(2) <8 ) & ((AIN(2) > (AnologHomeSignalMedian(2)+0.2)) & (FtdCarHomeSW.AnalogCARAxis(2) = 0))
	FtdCarHomeSW.AnalogCARAxis(2) =1
ret
on  (AnalogCARAxis(2) <8 ) & ((AIN(2) < (AnologHomeSignalMedian(2)-0.2)) & (FtdCarHomeSW.AnalogCARAxis(2) = 1))
	FtdCarHomeSW.AnalogCARAxis(2) =0
ret


on  (AnalogCARAxis(3) <8 ) & ((AIN(3) > (AnologHomeSignalMedian(3)+0.2)) & (FtdCarHomeSW.AnalogCARAxis(3) = 0))
	FtdCarHomeSW.AnalogCARAxis(3) =1
ret
on  (AnalogCARAxis(3) <8 ) & ((AIN(3) < (AnologHomeSignalMedian(3)-0.2)) & (FtdCarHomeSW.AnalogCARAxis(3) = 1))
	FtdCarHomeSW.AnalogCARAxis(3) =0
ret


#30
!/ User remarks = Home to hard stop--RobotArm-extension-gen2k
!Change log
! DATE		ECO#	Reason for Change			Remarks
!9/21/2015	N/A		ini release					ZW
!10/05/15,  N/A		changed to 20A Ext Homign	ZW
!10/16/2017, N/A	change made to turn on the CPE alarm and increase CPE threshold before go to home offset


real home_velocity, home_offset
real store_CURI0, store_CURV0, store_CURI2, store_CURV2
real store_CERRI0, store_CERRV0, store_CERRI2, store_CERRV2
global int RobotExtIsHoming
Global Int iExtCommanded, iRotCommanded, ExecutingMoveRot, iRotMoving, ExecutingMoveExt, iExtMoving
global real theta0
GLOBAL real  RobotROTTargetPrev
GLOBAL real  RobotExtTargetPrev
Global real FlipperClampDTI
RobotExtIsHoming = 1
ExecutingMoveRot = 0			!turn off moving flag
iRotCommanded = 0
iRotMoving = 0					!used in buffer 27
ExecutingMoveExt = 0			!turn off moving flag
iExtCommanded = 0				!turn off commanded flag
iExtMoving = 0					!used in buffer 27

FCLEAR 0
FCLEAR ROBOTAXISNUML
! commutation check
if MFLAGS(0).9=0 | MFLAGS(ROBOTAXISNUML).9=0 
	STOP(25)
	START 25, 1
	wait (10000)	!@@@ ask ZW why the 10 sec wait. Is this to allow commut to finish? Should make deterministic?
end
iControlProcessState(0) = 14
MFLAGS(0).#HOME = 0
MFLAGS(ROBOTAXISNUML).#HOME = 0
FDEF(0).#LL=0       ! Disable the iAxis left limit default response
FDEF(0).#RL=0       ! Disable the iAxis right limit default response
FDEF(ROBOTAXISNUML).#LL=0       ! Disable the iAxis left limit default response
FDEF(ROBOTAXISNUML).#RL=0       ! Disable the iAxis right limit default response

!reduce the motor current saturations
store_CURI0 = XCURI(0)
store_CURV0 = XCURV(0)
store_CURI2 = XCURI(ROBOTAXISNUML)
store_CURV2 = XCURV(ROBOTAXISNUML)

store_CERRI0 = CERRI(0)
store_CERRV0 = CERRV(0)
store_CERRI2 = CERRI(ROBOTAXISNUML)
store_CERRV2 = CERRV(ROBOTAXISNUML)

XCURI(0) = dHomingTorq(0)
XCURV(0) = dHomingTorq(0)
if ((iType(0)= 6005) |(iType(0)= 6007))
	XCURI(ROBOTAXISNUML) = dHomingTorq(ROBOTAXISNUML)
	XCURV(ROBOTAXISNUML) = dHomingTorq(ROBOTAXISNUML)
end

if (iType(0)=6009)	
	CERRI(0)=0.3
	CERRI(ROBOTAXISNUML)=0.3
	CERRV(0)=0.3
	CERRV(ROBOTAXISNUML)=0.3
end


!SET FPOS(0) = 0
!SET FPOS(ROBOTAXISNUML) = 0
!TPOS(0) = 0!added by ZW02
!TPOS(ROBOTAXISNUML) = 0!added by ZW02


! Turn off the CPE response

FDEF(0).#CPE = 0
FDEF(ROBOTAXISNUML).#CPE = 0
MFLAGS(0).#DEFCON = 1
MFLAGS(ROBOTAXISNUML).#DEFCON = 1


if ((iType(0)= 6005) |(iType(0)= 6007))
	MFLAGS(ROBOTAXISNUML).#DEFCON = 0
	CONNECT RPOS(ROBOTAXISNUML) = -APOS(0)
	DEPENDS ROBOTAXISNUML, 0
end
ENABLE (0)          ! Enable the iAxis drive
ENABLE (ROBOTAXISNUML)          ! Enable the iAxis drive
if (iType(0)=6009)	
	home_velocity = dHomingVel(0)/FlipperClampDTI
else
	home_velocity = dHomingVel(0)
end
if iHomingMethod(0) = 0
	JOG/v 0, home_velocity	!ZW03 change polar
	TILL (FAULT(0).#CPE = 1) | (FAULT(ROBOTAXISNUML).#CPE = 1)   			! Wait for the critical PE
	HALT (ROBOTAXISNUMH, ROBOTAXISNUML)
	
	wait 2000
end ! of if iHomingMethod =0, home to hard stop

SET FPOS(0) =  0
SET FPOS(ROBOTAXISNUML) = 0
SET RPOS(0) = 0		!@@@ why setting to 0 again?
SET RPOS(ROBOTAXISNUML) = 0		!@@@ why setting to 0 again?
SET APOS(0)=0		!@@@ what is APOS? why only setting to 0 here? 
SET APOS(ROBOTAXISNUML)=0		!@@@ what is APOS? why only setting to 0 here? 
TPOS(0) = 0!added by ZW02
TPOS(ROBOTAXISNUML) = 0!added by ZW02


!SET FPOS(1) = FPOS(1) + (IND(0) - home_offset)




! Set to higher CPE threshold to move out from the homed position
CERRI(0)=1
CERRI(ROBOTAXISNUML)=1
CERRV(0)=1
CERRV(ROBOTAXISNUML)=1

wait 1000

! Turn on CPE alarm
FDEF(0).#CPE = 1
FDEF(ROBOTAXISNUML).#CPE = 1

	
!Move to home offset position
if (iType(0)=6009)	
	PTP/e (0), -dHomeOffset(0)/FlipperClampDTI
else
	if iType(0) = 6005 !gen20A
		PTP (0), -2.06 !-dHomeOffset(0)
	else
		PTP (0), theta0-180 !-dHomeOffset(0)
	end	
	
	till (MST(0).4 = 1 ) & (MST(ROBOTAXISNUML).4 = 1 )
end

wait 1000

SET FPOS(0) =  0				
SET FPOS(ROBOTAXISNUML) = 0				
TPOS(0) = 0!added by ZW02		
TPOS(ROBOTAXISNUML) = 0!added by ZW02		
SET RPOS(ROBOTAXISNUML) = 0		!@@@ why setting to 0 again?
SET APOS(ROBOTAXISNUML)=0		!@@@ what is APOS? why only setting to 0 here? 
!Sp0 = 0.0	!@@@ comment out because set to 0.0 above
Sp0=0.0	!@@@ set Modbus register to 0.0 so no move happens after home.
RobotExtTargetPrev=0.0
if CONTROLLERMODELNUM = 6
	Sp1=0.0	!@@@ set Modbus register to 0.0 so no move happens after home.
else
	Sp2=0.0	!@@@ set Modbus register to 0.0 so no move happens after home.
end

!Reset THE CPE threshold
CERRI(0) = store_CERRI0 
CERRV(0) = store_CERRV0 
CERRI(ROBOTAXISNUML) = store_CERRI2
CERRV(ROBOTAXISNUML) = store_CERRV2

! Recover higher current to move to the home offset
XCURI(0) = store_CURI0 
XCURV(0) = store_CURV0
XCURI(ROBOTAXISNUML) = store_CURI2 
XCURV(ROBOTAXISNUML) = store_CURV2

! Set home flag to 1.
MFLAGS(ROBOTAXISNUML).#DEFCON = 1
!JERK(0) = JERK(0)/2
!JERK(ROBOTAXISNUML) = JERK(ROBOTAXISNUML)/2
MFLAGS(0).#HOME = 1
RobotExtIsHoming = 0
iControlProcessState(0) = 10
STOP             
#31
! instruction interpretation and execution including Modbus commands
!Change log
! DATE		ECO#	Reason for Change			Remarks
!9/21/2015	N/A		ini release					ZW
!1/5/16				Implement "define CAR home"	CR2016-01-05
!7/11/2018	N/A		Add estimate of CAR stopping distance to avoid moving back from continue turning to a fixed position.
Global INT 	MZ_CARAxisNum, MZ_LLAxisNum, MZ_STAxisNum	!See Buffer 27, initialized
Global INT	iLl_Interlock_Cmd	!iWord10Bit1			:= doutputs.,
Global INT	iSt_Interlock_Cmd	!iWord10Bit5			:= doutputs.,
Global INT	iCar_Interlock_Cmd	!iWord10Bit9			:= doutputs.,

GLOBAL INT bAxisInterlocked, G2M_OkToMoveCAR, M2G_CARZ, CARZ_AxisNum, C2M_OkToMoveCAR, M2C_CARZ, CARZ_PM_AxisNum, C2M_OkToMoveCAR2, M2C_CARZ2, CARZ_PM2_AxisNum
GLOBAL real OldSP(8), SP(8), SP_Modbus(8), RampRate(8), OldRR(8)
Global Int iAxisGood(8)
Global Int iSpChecked(8)
Global 	Real 	dQcmLimit(8)
GLOBAL REAL adjNegativeLimit(8)  !Used for ozone injector ring position limit when crash contitions are possible with other actuators
GLOBAL REAL adjPositiveLimit(8)  !Used for ozone injector ring position limit when crash contitions are possible with other actuators

Int ExecutingMove(8)
Int MoveCommanded(8)
int iAxisNum
real tmpVar, currentPos, overshootturns
int DefineHomeFlag
int Halted(8)
int HomingCMDBit(8)
int DefineHomeBit(8)
StatusWord0.5=0 !initialize the DefineHome request from LM to 0
StatusWord1.5=0 !initialize the DefineHome request from LM to 0
StatusWord2.5=0 !initialize the DefineHome request from LM to 0
StatusWord3.5=0 !initialize the DefineHome request from LM to 0
StatusWord4.5=0 !initialize the DefineHome request from LM to 0
StatusWord5.5=0 !initialize the DefineHome request from LM to 0
StatusWord6.5=0 !initialize the DefineHome request from LM to 0
StatusWord7.5=0 !initialize the DefineHome request from LM to 0

StatusWord0.6=0 !clamping command from LM to 0

StatusWord0.7=0 !initialize the Homing Command request from LM to 0
StatusWord1.7=0 !initialize the Homing Command request from LM to 0
StatusWord2.7=0 !initialize the Homing Command request from LM to 0
StatusWord3.7=0 !initialize the Homing Command request from LM to 0
StatusWord4.7=0 !initialize the Homing Command request from LM to 0
StatusWord5.7=0 !initialize the Homing Command request from LM to 0
StatusWord6.7=0 !initialize the Homing Command request from LM to 0
StatusWord7.7=0 !initialize the Homing Command request from LM to 0
StatusWord0.7=0 !initialize the Homing Command request from LM to 0

!real delete		!delete this line
!set FPOS(1)=0  	!delete this line

ExecutingMove(0) = 0
ExecutingMove(1) = 0
ExecutingMove(2) = 0
ExecutingMove(3) = 0
ExecutingMove(4) = 0
ExecutingMove(5) = 0
ExecutingMove(6) = 0
ExecutingMove(7) = 0
MoveCommanded(0) = 0
MoveCommanded(1) = 0
MoveCommanded(2) = 0
MoveCommanded(3) = 0
MoveCommanded(4) = 0
MoveCommanded(5) = 0
MoveCommanded(6) = 0
MoveCommanded(7) = 0
RampRate(0)= ramprate0	
RampRate(1)= ramprate1
RampRate(2)= ramprate2
RampRate(3)= ramprate3
RampRate(4)= ramprate4
RampRate(5)= ramprate5
RampRate(6)= ramprate6
RampRate(7)= ramprate7
int k
k = 0
LOOP MAXNUMOFAXIS
	Halted(k) = 0	!initialize to "not halted"
	OldRR(k)= RampRate(k)
	k=k+1
END

AA:
BLOCK
!==========ZW on 2/4/2016===========
!Add SP_Modbus variable to keep the code generic for different configurations
SP_Modbus(0) = Sp0	!@@@ moved below; update conditional on NOT moving
!changed for VAT valve
if iType(1) = 1040
	SP_Modbus(1) = 0.01*Sp1*abs(dPositiveLimit(1)-dNegtiveLimit(1)) + dNegtiveLimit(1)
else
	SP_Modbus(1) = Sp1
end
SP_Modbus(2) = Sp2
SP_Modbus(3) = Sp3
SP_Modbus(4) = Sp4
SP_Modbus(5) = Sp5
SP_Modbus(6) = Sp6
SP_Modbus(7) = Sp7
!==========ZW on 2/4/2016===========

RampRate(0)= ramprate0	!@@@ RampRate can change "on the fly"
RampRate(1)= ramprate1
RampRate(2)= ramprate2
RampRate(3)= ramprate3
RampRate(4)= ramprate4
RampRate(5)= ramprate5
RampRate(6)= ramprate6
RampRate(7)= ramprate7

iAxisNum = 0


LOOP MAXNUMOFAXIS
if iType(iAxisNum)<6000	!robot command is handled by Buffer 26
	if (RampRate(iAxisNum) >= dVmax(iAxisNum)) | ((iAxisNum = CARZ_AxisNum)& (G2M_OkToMoveCAR = 0) & (M2G_CARZ = 1)) | ((iAxisNum = CARZ_PM_AxisNum) & (C2M_OkToMoveCAR = 0) & (M2C_CARZ = 1)) | ((iAxisNum = CARZ_PM2_AxisNum) & (C2M_OkToMoveCAR2 = 0) & (M2C_CARZ2 = 1)) !verify velocity is not over max limit.  If so, set back to previous
		iControlProcessState(iAxisNum) = 103
		RampRate(iAxisNum) = OldRR(iAxisNum)
		SP(iAxisNum) = OldSP(iAxisNum)
		SP_Modbus(iAxisNum) = OldSP(iAxisNum)
		if iAxisNum = 0		
			Sp0 = SP_Modbus(iAxisNum)
			ramprate0 = RampRate(iAxisNum)
		elseif iAxisNum = 1
			if iType(1) = 1040
				if (abs(dPositiveLimit(1)-dNegtiveLimit(1))>0.00001)
					Sp1= 100.0*abs(SP_Modbus(1)-dNegtiveLimit(1))/abs(dPositiveLimit(1)-dNegtiveLimit(1))
				else	
					Sp1=0.0
				end
			else
				Sp1 = SP_Modbus(iAxisNum)
			end	
			ramprate1 = RampRate(iAxisNum)
		elseif iAxisNum = 2
			Sp2 = SP_Modbus(iAxisNum)
			ramprate2 = RampRate(iAxisNum)
		elseif iAxisNum = 3
			Sp3 = SP_Modbus(iAxisNum)
			ramprate3 = RampRate(iAxisNum)
		elseif iAxisNum = 4
			Sp4 = SP_Modbus(iAxisNum)
			ramprate4 = RampRate(iAxisNum)
		elseif iAxisNum = 5
			Sp5 = SP_Modbus(iAxisNum)
			ramprate5 = RampRate(iAxisNum)
		elseif iAxisNum = 6
			Sp6 = SP_Modbus(iAxisNum)
			ramprate6 = RampRate(iAxisNum)
		elseif iAxisNum = 7
			Sp7 = SP_Modbus(iAxisNum)
			ramprate7 = RampRate(iAxisNum)
		end

	end


!if 	(iAxisGood(iAxisNum))&(SP_Modbus(iAxisNum)<=dPositiveLimit(iAxisNum))&(SP_Modbus(iAxisNum)>=dNegtiveLimit(iAxisNum))&(SP_Modbus(iAxisNum)<=dQcmLimit(iAxisNum))&(^iPreventMove(iAxisNum))
! removed the position limit check since it does not allow you to move out from a fault condition.

	if 	(iAxisGood(iAxisNum))&(SP_Modbus(iAxisNum)<=dQcmLimit(iAxisNum))&(SP_Modbus(iAxisNum)>=adjNegativeLimit(iAxisNum))&(SP_Modbus(iAxisNum)<=adjPositiveLimit(iAxisNum))&(^iPreventMove(iAxisNum)&^((iInterlockEnabled(iAxisNum)=1)&(bAxisInterlocked=1)))
		!conditions are good for axis
		!move request inside positive and negative limits
		!move request inside QCM limit, and move not prevented by GM controller
	!==========ZW on 2/4/2016===========
		If ((iType(iAxisNum)= 4008) & (iCar_Interlock_Cmd = 0)) | ((iType(iAxisNum)= 5011) & (iLl_Interlock_Cmd = 0)) | ((iType(iAxisNum)= 5012) & (iSt_Interlock_Cmd = 0))
			! interlocked for MZ axes, doing nothing
			! = 0, interlocked
			! = 1, not intlocked
			iControlProcessState(iAxisNum) = 113
			SP_Modbus(iAxisNum) = OldSP(iAxisNum)
			if iAxisNum = 0		!CR2016-01-13 commented out because caused "Stop" button to turn yellow (incomplete move)
				Sp0 = SP_Modbus(iAxisNum)
			elseif iAxisNum = 1
				Sp1 = SP_Modbus(iAxisNum)
			elseif iAxisNum = 2
				Sp2 = SP_Modbus(iAxisNum)
			elseif iAxisNum = 3
				Sp3 = SP_Modbus(iAxisNum)
			elseif iAxisNum = 4
				Sp4 = SP_Modbus(iAxisNum)
			elseif iAxisNum = 5
				Sp5 = SP_Modbus(iAxisNum)
			elseif iAxisNum = 6
				Sp6 = SP_Modbus(iAxisNum)
			elseif iAxisNum = 7
				Sp7 = SP_Modbus(iAxisNum)
			end
		else
			if (iControlProcessState(iAxisNum) = 0) 
				iControlProcessState(iAxisNum) = 10
			end	
		!START 2016-02-29
			SP(iAxisNum)= SP_Modbus(iAxisNum)	!ZW 2/4/2016
				
			if (iType(iAxisNum)>3000) & (iType(iAxisNum)<5000)	!GSR and GSP-ROT !CAR needs to take SPs even when moving.
				SP(iAxisNum) = SP_Modbus(iAxisNum)
			end
		!==========ZW on 2/4/2016===========	

			if ((SP(iAxisNum) <> OldSP(iAxisNum)) | (RampRate(iAxisNum) <> OldRR(iAxisNum)))& MST(iAxisNum).#ENABLED
				
				if (iType(iAxisNum)>1000) & (iType(iAxisNum)<2000)  !AVP
					if((iType(iAxisNum)=1040)&(iControlMode1 <> 0))
					
					
					else
						iControlProcessState(iAxisNum) = 11
						BREAK(iAxisNum)
						if (OldSP(iAxisNum) < SP(iAxisNum))&((SP(iAxisNum)+dBackLashCompensation(iAxisNum))<=dPositiveLimit(iAxisNum))
							MPTP/wv iAxisNum
							POINT iAxisNum, (SP(iAxisNum)+dBackLashCompensation(iAxisNum))*dAxisPitch(iAxisNum), RampRate(iAxisNum)*dAxisPitch(iAxisNum)		!pitch = user linear unit to degrees
							POINT iAxisNum, SP(iAxisNum)*dAxisPitch(iAxisNum), 20*dAxisPitch(iAxisNum)		!pitch = user linear unit to degrees
							ENDS iAxisNum
							GO iAxisNum
						else
							PTP/v iAxisNum, SP(iAxisNum)*dAxisPitch(iAxisNum), RampRate(iAxisNum)*dAxisPitch(iAxisNum)		!pitch = user linear unit to degrees
						end
						OldSP(iAxisNum) = SP(iAxisNum)				!zw 12/28/15 MOVED INTO LOOP
						OldRR(iAxisNum) = RampRate(iAxisNum)
						MoveCommanded(iAxisNum) = 1
					end
				
				elseif  (iType(iAxisNum)>2000) & (iType(iAxisNum)<3000)	!Index
					BREAK(iAxisNum)
					PTP/v iAxisNum, SP(iAxisNum), RampRate(iAxisNum)
					OldSP(iAxisNum) = SP(iAxisNum)
					OldRR(iAxisNum) = RampRate(iAxisNum)
					MoveCommanded(iAxisNum) = 1
				
				
				elseif  (iType(iAxisNum)>3000) & (iType(iAxisNum)<5000)	!GSR and GSP-ROT
					! adding MZ_CAR code here for interlocked condition
					if abs(SP(iAxisNum)-361)<0.01
						jog/v  iAxisNum, RampRate(iAxisNum)*6	!rpm to deg/s
					elseif abs(SP(iAxisNum)+1)<0.01
						jog/v  iAxisNum, -RampRate(iAxisNum)*6	!rpm to deg/s
					else
						BREAK(iAxisNum)
						overshootturns = RVEL(iAxisNum)*RVEL(iAxisNum)/(dAmax(iAxisNum)*6)/2
						if RVEL(iAxisNum)<0.0
							currentPos = RPOS(iAxisNum) - overshootturns
						else
							currentPos = RPOS(iAxisNum) + overshootturns
						end	
						tmpVar = SP(iAxisNum) + 360*FLOOR(currentPos/360)	!0-360 translate
						if (tmpVar - currentPos) <= -180.0					!calculate shortest path to SP
							tmpVar = tmpVar + 360
						elseif (tmpVar - currentPos) >= 180.0
							tmpVar = tmpVar - 360
						end														!end calculation for shortest path
						PTP/v iAxisNum, tmpVar, RampRate(iAxisNum)*6	!CJR added *
					end
					OldSP(iAxisNum) = SP(iAxisNum)
					OldRR(iAxisNum) = RampRate(iAxisNum)
					MoveCommanded(iAxisNum) = 1
								
				elseif  (iType(iAxisNum)>5000) & (iType(iAxisNum)<6000)	!LP&LE
					! adding MZ_LL, MZ_ST code here for interlocked condition

					BREAK(iAxisNum)
					PTP/v iAxisNum, SP(iAxisNum)*dAxisPitch(iAxisNum), RampRate(iAxisNum)*dAxisPitch(iAxisNum)		!pitch = user linear unit to degrees
					OldSP(iAxisNum) = SP(iAxisNum)
					OldRR(iAxisNum) = RampRate(iAxisNum)
					MoveCommanded(iAxisNum) = 1
				else
					! other cases not hanlded here
				end
			end
			
			
			if MoveCommanded(iAxisNum)
				if MST(iAxisNum).4 = 0	!@@@ =0 means motor is moving
					ExecutingMove(iAxisNum) = 1
				end
			end
				
			if ExecutingMove(iAxisNum)
				if MST(iAxisNum).4 = 1	!@@@ =1 means move is complete
					ExecutingMove(iAxisNum) = 0
					MoveCommanded(iAxisNum) = 0
					Halted(iAxisNum) = 0
					iControlProcessState(iAxisNum) = 10
					
					if iAxisNum = 0
						Sp0 = WkSp0	!Measured0
						OldSP(0) = Sp0
						SP(0) = Sp0
					end
					if iAxisNum = 1
						Sp1 = WkSp1	!Measured1
						OldSP(1) = Sp1
						SP(1) = Sp1
					end
					if iAxisNum = 2
						Sp2 = WkSp2	!Measured2
						OldSP(2) = Sp2
						SP(2) = Sp2
					end
					if iAxisNum = 3
						Sp3 = WkSp3	!Measured3
						OldSP(3) = Sp3
						SP(3) = Sp3
					end
					if iAxisNum = 4
						Sp4 = WkSp4	!Measured4
						OldSP(4) = Sp4
						SP(4) = Sp4
					end
					if iAxisNum = 5
						Sp5 = WkSp5	!Measured5
						OldSP(5) = Sp5
						SP(5) = Sp5
					end
					if iAxisNum = 6
						Sp6 = WkSp6	!Measured6
						OldSP(6) = Sp6
						SP(6) = Sp6
					end
					if iAxisNum = 7
						Sp7 = WkSp7	!Measured7
						OldSP(7) = Sp7
						SP(7) = Sp7
					end
				end 
			end	
		end	! of MZ interlock condition
	else
		if(SP_Modbus(iAxisNum)<adjNegativeLimit(iAxisNum))|(SP_Modbus(iAxisNum)>adjPositiveLimit(iAxisNum))
			iControlProcessState(iAxisNum) = 102
		end

		if iPreventMove(iAxisNum)
			iControlProcessState(iAxisNum) = 113
		end
		
		SP_Modbus(iAxisNum) = OldSP(iAxisNum)
		if iAxisNum = 0		!CR2016-01-13 commented out because caused "Stop" button to turn yellow (incomplete move)
			Sp0 = SP_Modbus(iAxisNum)
		elseif iAxisNum = 1
			Sp1 = SP_Modbus(iAxisNum)
		elseif iAxisNum = 2
			Sp2 = SP_Modbus(iAxisNum)
		elseif iAxisNum = 3
			Sp3 = SP_Modbus(iAxisNum)
		elseif iAxisNum = 4
			Sp4 = SP_Modbus(iAxisNum)
		elseif iAxisNum = 5
			Sp5 = SP_Modbus(iAxisNum)
		elseif iAxisNum = 6
			Sp6 = SP_Modbus(iAxisNum)
		elseif iAxisNum = 7
			Sp7 = SP_Modbus(iAxisNum)
		end

	end
end	! of if type <6000
iAxisNum=iAxisNum+1
end

END !OF BLOCK


BLOCK

DefineHomeBit(0)= StatusWord0.5
DefineHomeBit(1)= StatusWord1.5
DefineHomeBit(2)= StatusWord2.5
DefineHomeBit(3)= StatusWord3.5
DefineHomeBit(4)= StatusWord4.5
DefineHomeBit(5)= StatusWord5.5
DefineHomeBit(6)= StatusWord6.5
DefineHomeBit(7)= StatusWord7.5
iAxisNum = 0
LOOP MAXNUMOFAXIS
	if DefineHomeBit(iAxisNum) !& ^DefineHomeFlag	
		SET FPOS(iAxisNum) = 0
		!FDEF(iAxisNum).#CPE = 1
		MFLAGS(iAxisNum).#HOME = 1
		if iAxisNum =0
			Sp0=0	
		elseif iAxisNum =1
			Sp1=0	
		elseif iAxisNum =2
			Sp2=0	
		elseif iAxisNum =3
			Sp3=0	
		elseif iAxisNum =4
			Sp4=0	
		elseif iAxisNum =5
			Sp5=0	
		elseif iAxisNum =6
			Sp6=0	
		elseif iAxisNum =7
			Sp7=0	
		end	
		ENABLE (iAxisNum)
		PTP/rv (iAxisNum), 2, RampRate(iAxisNum)	!@@@ relative move to get away from 359.nnn which takes absolute move from 359.nnn around to 0.0
		PTP/v (iAxisNum), 0, RampRate(iAxisNum)
		DefineHomeBit(iAxisNum)=0	!Reset the Modbus register to 0 after reading it from LM
		iControlProcessState(iAxisNum) = 13

	end
	iAxisNum=iAxisNum+1
end
StatusWord0.5 = DefineHomeBit(0)
StatusWord1.5 = DefineHomeBit(1)
StatusWord2.5 = DefineHomeBit(2)
StatusWord3.5 = DefineHomeBit(3)
StatusWord4.5 = DefineHomeBit(4)
StatusWord5.5 = DefineHomeBit(5)
StatusWord6.5 = DefineHomeBit(6)
StatusWord7.5 = DefineHomeBit(7)

END


WAIT 20	!@@@ was 200 - maybe eliminate altogether?
!delete = delete + 1		!delete this line
!made these next 11 lines specific to CAR home from lot manager.	!CR2016-01-05
!if StatusWord1.5 !& ^DefineHomeFlag	
!	DefineHomeFlag = 1
!	SET FPOS(1) = 0
!	FDEF(1).#CPE = 1
!	MFLAGS(1).#HOME = 1
!	Sp1=0	!added by ZW 4/27/2016
!	ENABLE (1)
!	PTP/rv (1), 5, RampRate(1)	!@@@ relative move to get away from 359.nnn which takes absolute move from 359.nnn around to 0.0
!	PTP/v (1), 0, RampRate(1)
!	wait 5000	!@@@ comment out - this wait stops all other parallel code execution
!	StatusWord1.5=0	!Reset the Modbus register to 0 after reading it from LM
!end
!if ^StatusWord1.5
!	DefineHomeFlag = 0
!end
! end CR2016-01-05

GOTO AA

STOP
!need to set the fault protection for LL RL and Interlock

!ON FAULT(2).#RL | FAULT(2).#LL.
!When there is a right limit or left limit fault in the 2 axis motor.
!DISABLE 2
!Disable axis 2
!RET



!ON IN0.0! When input#0=0
!OUT0.4=1! Set output#4 to 1
!disp ?Activates motor?
!RET ! Ends the autoroutine







On StatusWord0.7 = 1	
	if iType(0)>6000
		STOP 23
		START 23,1
	else
		STOP 0
		START 0,1
	end	
	StatusWord0.7 = 0	!HomingCMDBit(iAxisNum)=0	!Reset the Modbus register to 0 after reading it from LM
	iControlProcessState(0) = 12
ret

On StatusWord1.7 = 1	
	if iType(1)>6000
		STOP 23
		START 23,1
	else
		STOP 1
		START 1,1
	end	
	StatusWord1.7 = 0	!HomingCMDBit(iAxisNum)=0	!Reset the Modbus register to 0 after reading it from LM
	iControlProcessState(1) = 12
ret


On StatusWord2.7 = 1	
	if iType(2)>6000
		STOP 23
		START 23,1
	else
		STOP 2
		START 2,1
	end	
	StatusWord2.7 = 0	!HomingCMDBit(iAxisNum)=0	!Reset the Modbus register to 0 after reading it from LM
	iControlProcessState(2) = 12
ret

On StatusWord3.7 = 1	
	if iType(3)>6000
		STOP 23
		START 23,1
	else
		STOP 3
		START 3,1
	end	
	StatusWord3.7 = 0	!HomingCMDBit(iAxisNum)=0	!Reset the Modbus register to 0 after reading it from LM
	iControlProcessState(3) = 12
ret


On StatusWord4.7 = 1	
	if iType(4)>6000
		STOP 23
		START 23,1
	else
		STOP 4
		START 4,1
	end	
	StatusWord4.7 = 0	!HomingCMDBit(iAxisNum)=0	!Reset the Modbus register to 0 after reading it from LM
	iControlProcessState(4) = 12
	
ret

On StatusWord5.7 = 1	
	if iType(5)>6000
		STOP 23
		START 23,1
	else
		STOP 5
		START 5,1
	end	
	StatusWord5.7 = 0	!HomingCMDBit(iAxisNum)=0	!Reset the Modbus register to 0 after reading it from LM
	iControlProcessState(5) = 12
	
ret

On StatusWord6.7 = 1	
	if iType(6)>6000
		STOP 23
		START 23,1
	else
		STOP 6
		START 6,1
	end	
	StatusWord6.7 = 0	!HomingCMDBit(iAxisNum)=0	!Reset the Modbus register to 0 after reading it from LM
	iControlProcessState(6) = 12

ret

On StatusWord7.7 = 1	
	if iType(7)>6000
		STOP 23
		START 23,1
	else
		STOP 7
		START 7,1
	end	
	StatusWord7.7 = 0	!HomingCMDBit(iAxisNum)=0	!Reset the Modbus register to 0 after reading it from LM
	iControlProcessState(7) = 12
ret




#A
!Molly Axis:	1	2	3	4	5	6	7	8	- used at Molly interface
!ACS Axis:		0	2	1	3	4	6	5	7	- used in the ACS application
!Connection:	X	A	Y	B	Z	C	T	D	- as labeled by ACS
!Labels			1	2	3	4	5	6	7	8	- connections labeled by Veeco


axisdef X=0,Y=1,Z=4,T=5,A=2,B=3,C=6,D=7
axisdef x=0,y=1,z=4,t=5,a=2,b=3,c=6,d=7

global int I(100),I0,I1,I2,I3,I4,I5,I6,I7,I8,I9,I90,I91,I92,I93,I94,I95,I96,I97,I98,I99
global real V(100),V0,V1,V2,V3,V4,V5,V6,V7,V8,V9,V90,V91,V92,V93,V94,V95,V96,V97,V98,V99

Global int	FtdCarHomeSW
Global Int bTeachMode
global int MAXNUMOFAXIS
global int ROBOTAXISNUMH
global int ROBOTAXISNUML
global int	CONTROLLERMODELNUM
global	int	LLELEAXIS, LLELEAXIS2
global	int iControlProcessState(8)
!Axis define
Global INT AxisType(8)
Global	int		iType(8)
Global	real	dFixedPosA(8)
Global	real	dFixedPosB(8)
Global	real	dFixedPosC(8)
Global	real	dFixedPosD(8)
Global	real	dPositiveLimit(8)
Global	real	dNegtiveLimit(8)
Global	real	dVel(8)				!ZW 12/28/15
Global	real	dJogVel(8)			!Zw 12/28/15
Global	real	dVmax(8)
Global	real	dAmax(8)
Global	real	dFaultTorq(8)		!to be converted to Motor current limit at controller
Global	real	dHomingTorq(8)	!to be converted to Motor current limit at controller
Global	real	dHomingVel(8)
Global	int		iInterlockEnabled(8)	!1- yes, 0-no
Global	int		iSecondSPEnabled(8)
Global	int		iInterlockBehavior(8)	!default 0, stop motion; 1-go to interlock position
Global	real	dInterlockPos(8)
Global	real	dSecondSetpointPos(8)
Global	real	dHomePulseWidth(8)	
Global	real	dHomePulseDelay(8)
Global	real	dAxisPitch(8)
Global	real	dGearRatio(8)
!added by ZW on 2/4/2016
Global	int		iHomingMethod(8)		!0-home to torque; 1-homing to switch; 2-manual set home;
!added by CJR pm 2/17/2016
Global	real	dHomeOffset(8)			!to replace home_offset, given in user units
Global	int		iQcmAxis(1)				!0-7 if QCM on 0-7; -1 = no QCM 
Global	int		iPreventMove(8)			!this is an interlock to prevent motion
Global	real	UseAnalogCmd(3)			!0-ena-analog; 
										!1--analog cmd threshold (default 0.1); 
										!2--AnalogCmdTimeWindow default 1000ms
Global	real	AnalogChannelOffset(4)
Global	real	dBackLashCompensation(8)
Global	real	dCE(8)					!this is used to track home position "drift" for AVPs
Global	int		iInverseInterlockLogic(1)
Global real		O3CollisionValues(8)
Global real		dValve_scaling_factor(8)		!A calibratable parameter. Value range 6.0~20, default 6.0.
Global real		dPressureSP(8)				!Torr, pressure setpoint, 5e-11 to 1e-4
Global real		dPressureRamprate(8)			!Pressure ramp rate expressed at Torr/min
Global real		dPres_min(8)					!Min Pressure setpoint, 5e-11
Global real		dPres_max(8)					!Max pressure setpoint, 5e-5
Global real		dSafetyShutDownPressure(8)	!Defines a pressure threshold for safety.  (5e-11 to 1e-4) 8e-5 is default



! Modbus Mapping
!The 1st 4 INTs are repeated for each axis because Molly needs flexibility for any 
!controller on any axis. There should be an "internal INT where the SW version is
!assigned, and each axis SWVersion is set to this value. Regarding the DINPUT values,
!the plan is to make them configurable through the MAC GUI, so part of the configuration
!will be linking these inputs to the appropriate modbus bit for Molly to read. For example,
!if MAC input 2 is assigned as interlock for ACS axis 4, then input 2 would be mapped to
!DINPUT4.
!global int tag 1000	InputStatus
!global int tag 1001	InstrumentID
!global int tag 1002	SWVersion
!global int tag 1003	DINPUT

!GLOBAL REAL TAG 134	Time

!Molly Axis 1 = ACS Axis0 - Robot Extension
!				Molly Axis# add		MAC Axis# add
global int 	tag 1000	InputStatus0
global int 	tag 1001	InstrumentID0
global int 	tag 1002	SWVersion0
global int 	tag 1003	DINPUT0
global real tag 1004	OUTP0
global real tag 1005	CARHM0
global int 	tag 1006	iControlMode0
global real tag 1007	pressurePID_KP0
global real tag 1008	pressurePID_KI0
global real tag 1009	pressurePID_KD0
global real tag 1010	OutPressure_KP0
global real tag 1011	Valve_Scaling_Factor0
global real tag 1012	PressureSP0
global real tag 1013	Pressure_Ramp_rate0		!Pressure ramp rate expressed at Torr/min
global real tag 1014	Pres_min0				!Min Pressure setpoint, 5e-11
global real tag 1015	Pres_max0				!Max pressure setpoint, 5e-5
global real tag 1016	SafetyShutDownPressure0	!Defines a pressure threshold for safety.  (5e-11 to 1e-4) 8e-5 is default
global int 	tag 1017	StatusWord0
global int 	tag 1018	KillTraj0
global real tag 1019	PressurePV0
global real tag 1020	PressureWKSP0
global int 	tag 1021	ControlProcessState0
global real tag 1050 	Measured0
global real tag 1051	WkSp0
global real tag 1067	Sp0	!command
global real tag 1068	ramprate0
global real tag 1070	VKP0					!servo velocity loop Kp
global real tag 1071	VKI0					!servo velocity loop Ki
global real tag 1072	PKP0					!servo position loop Kp
global real tag 1073	MaxSP0
global real tag 1074	MinSP0
global real tag 1075	MaxVel0
global real tag 1076	MinVel0
global real tag 1084	AxisDec0
global real tag 1085	AxisAcc0
global real tag 1086	FaultTorq0
global real tag 1087	HomeTorq0
global real tag 1088	MaxTorq0


!Molly Axis 2 = ACS Axis2 - Robot Rotation
global int 	tag 1100	InputStatus2
global int 	tag 1101	InstrumentID2
global int 	tag 1102	SWVersion2
global int 	tag 1103	DINPUT2
global real tag 1104	OUTP2
global int 	tag 1105	CARHM2
global int 	tag 1106	iControlMode2
global real tag 1107	pressurePID_KP2
global real tag 1108	pressurePID_KI2
global real tag 1109	pressurePID_KD2
global real tag 1110	OutPressure_KP2
global real tag 1111	Valve_Scaling_Factor2
global real tag 1112	PressureSP2
global real tag 1113	Pressure_Ramp_rate2		!Pressure ramp rate expressed at Torr/min
global real tag 1114	Pres_min2				!Min Pressure setpoint, 5e-11
global real tag 1115	Pres_max2				!Max pressure setpoint, 5e-5
global real tag 1116	SafetyShutDownPressure2	!Defines a pressure threshold for safety.  (5e-11 to 1e-4) 8e-5 is default
global int 	tag 1117	StatusWord2
global int 	tag 1118	KillTraj2
global real tag 1119	PressurePV2
global real tag 1120	PressureWKSP2
global int 	tag 1121	ControlProcessState2
global real tag 1150 	Measured2
global real tag 1151	WkSp2
global real tag 1167	Sp2	!command
global real tag 1168	ramprate2
global real tag 1170	VKP2					!servo velocity loop Kp
global real tag 1171	VKI2					!servo velocity loop Ki
global real tag 1172	PKP2					!servo position loop Kp
global real tag 1173	MaxSP2
global real tag 1174	MinSP2
global real tag 1175	MaxVel2
global real tag 1176	MinVel2
global real tag 1184	AxisDec2
global real tag 1185	AxisAcc2
global real tag 1186	FaultTorq2
global real tag 1187	HomeTorq2
global real tag 1188	MaxTorq2
global int 	tag 1189	VELDIR2			!direction of flipper rotation

!Molly Axis 3 = ACS Axis 1 - CAR
global int 	tag 1200	InputStatus1
global int 	tag 1201	InstrumentID1
global int 	tag 1202	SWVersion1
global int 	tag 1203	DINPUT1
global real tag 1204	OUTP1
global int 	tag 1205	CARHM1
global int 	tag 1206	iControlMode1
global real tag 1207	pressurePID_KP1
global real tag 1208	pressurePID_KI1
global real tag 1209	pressurePID_KD1
global real tag 1210	OutPressure_KP1
global real tag 1211	Valve_Scaling_Factor1
global real tag 1212	PressureSP1
global real tag 1213	Pressure_Ramp_rate1		!Pressure ramp rate expressed at Torr/min
global real tag 1214	Pres_min1				!Min Pressure setpoint, 5e-11
global real tag 1215	Pres_max1				!Max pressure setpoint, 5e-5
global real tag 1216	SafetyShutDownPressure1	!Defines a pressure threshold for safety.  (5e-11 to 1e-4) 8e-5 is default
global int 	tag 1217	StatusWord1
global int 	tag 1218	KillTraj1
global real tag 1219	PressurePV1
global real tag 1220	PressureWKSP1
global int 	tag 1221	ControlProcessState1
global real tag 1250 	Measured1
global real tag 1251	WkSp1
global real tag 1267	Sp1	!command
global real tag 1268	ramprate1
global real tag 1270	VKP1					!servo velocity loop Kp
global real tag 1271	VKI1					!servo velocity loop Ki
global real tag 1272	PKP1					!servo position loop Kp
global real tag 1273	MaxSP1
global real tag 1274	MinSP1
global real tag 1275	MaxVel1
global real tag 1276	MinVel1
global real tag 1284	AxisDec1
global real tag 1285	AxisAcc1
global real tag 1286	FaultTorq1
global real tag 1287	HomeTorq1
global real tag 1288	MaxTorq1
global int 	tag 1289	VELDIR1				!direction of flipper rotation

!Molly Axis 4 = ACS Axis 3 - Spare 4
global int 	tag 1300	InputStatus3
global int 	tag 1301	InstrumentID3
global int 	tag 1302	SWVersion3
global int 	tag 1303	DINPUT3
global real tag 1304	OUTP3
global int 	tag 1306	iControlMode3
global real tag 1307	pressurePID_KP3
global real tag 1308	pressurePID_KI3
global real tag 1309	pressurePID_KD3
global real tag 1310	OutPressure_KP3
global real tag 1311	Valve_Scaling_Factor3
global real tag 1312	PressureSP3
global real tag 1313	Pressure_Ramp_rate3		!Pressure ramp rate expressed at Torr/min
global real tag 1314	Pres_min3				!Min Pressure setpoint, 5e-11
global real tag 1315	Pres_max3				!Max pressure setpoint, 5e-5
global real tag 1316	SafetyShutDownPressure3	!Defines a pressure threshold for safety.  (5e-11 to 1e-4) 8e-5 is default
global int 	tag 1317	StatusWord3
global int 	tag 1318	KillTraj3
global real tag 1319	PressurePV3
global real tag 1320	PressureWKSP3
global int 	tag 1321	ControlProcessState3
global real tag 1350 	Measured3
global real tag 1351	WkSp3
global real tag 1367	Sp3	!command
global real tag 1368	ramprate3
global real tag 1370	VKP3					!servo velocity loop Kp
global real tag 1371	VKI3					!servo velocity loop Ki
global real tag 1372	PKP3					!servo position loop Kp
global real tag 1373	MaxSP3
global real tag 1374	MinSP3
global real tag 1375	MaxVel3
global real tag 1376	MinVel3
global real tag 1384	AxisDec3
global real tag 1385	AxisAcc3
global real tag 1386	FaultTorq3
global real tag 1387	HomeTorq3
global real tag 1388	MaxTorq3

!Molly Axis 5 = ACS Axis 4 - Spare 5
global int 	tag 1400	InputStatus4
global int 	tag 1401	InstrumentID4
global int 	tag 1402	SWVersion4
global int 	tag 1403	DINPUT4
global real tag 1404	OUTP4
global int 	tag 1405	CARHM4
global int 	tag 1406	iControlMode4
global real tag 1407	pressurePID_KP4
global real tag 1408	pressurePID_KI4
global real tag 1409	pressurePID_KD4
global real tag 1410	OutPressure_KP4
global real tag 1411	Valve_Scaling_Factor4
global real tag 1412	PressureSP4
global real tag 1413	Pressure_Ramp_rate4		!Pressure ramp rate expressed at Torr/min
global real tag 1414	Pres_min4				!Min Pressure setpoint, 5e-11
global real tag 1415	Pres_max4				!Max pressure setpoint, 5e-5
global real tag 1416	SafetyShutDownPressure4	!Defines a pressure threshold for safety.  (5e-11 to 1e-4) 8e-5 is default
global int 	tag 1417	StatusWord4
global int 	tag 1418	KillTraj4
global real tag 1419	PressurePV4
global real tag 1420	PressureWKSP4
global int 	tag 1421	ControlProcessState4
global real tag 1450 	Measured4
global real tag 1451	WkSp4
global real tag 1467	Sp4	!command
global real tag 1468	ramprate4
global real tag 1470	VKP4					!servo velocity loop Kp
global real tag 1471	VKI4					!servo velocity loop Ki
global real tag 1472	PKP4					!servo position loop Kp
global real tag 1473	MaxSP4
global real tag 1474	MinSP4
global real tag 1475	MaxVel4
global real tag 1476	MinVel4
global real tag 1484	AxisDec4
global real tag 1485	AxisAcc4
global real tag 1486	FaultTorq4
global real tag 1487	HomeTorq4
global real tag 1488	MaxTorq4

!Molly Axis 6 = ACS Axis 6 - Spare 6
global int 	tag 1500	InputStatus6
global int 	tag 1501	InstrumentID6
global int 	tag 1502	SWVersion6
global int 	tag 1503	DINPUT6
global real tag 1504	OUTP6
global int 	tag 1506	iControlMode6
global real tag 1507	pressurePID_KP6
global real tag 1508	pressurePID_KI6
global real tag 1509	pressurePID_KD6
global real tag 1510	OutPressure_KP6
global real tag 1511	Valve_Scaling_Factor6
global real tag 1512	PressureSP6
global real tag 1513	Pressure_Ramp_rate6		!Pressure ramp rate expressed at Torr/min
global real tag 1514	Pres_min6				!Min Pressure setpoint, 5e-11
global real tag 1515	Pres_max6				!Max pressure setpoint, 5e-5
global real tag 1516	SafetyShutDownPressure6	!Defines a pressure threshold for safety.  (5e-11 to 1e-4) 8e-5 is default
global int 	tag 1517	StatusWord6
global int 	tag 1518	KillTraj6
global real tag 1519	PressurePV6
global real tag 1520	PressureWKSP6
global int 	tag 1521	ControlProcessState6
global real tag 1550 	Measured6
global real tag 1551	WkSp6
global real tag 1567	Sp6	!command
global real tag 1568	ramprate6
global real tag 1570	VKP6					!servo velocity loop Kp
global real tag 1571	VKI6					!servo velocity loop Ki
global real tag 1572	PKP6					!servo position loop Kp
global real tag 1573	MaxSP6
global real tag 1574	MinSP6
global real tag 1575	MaxVel6
global real tag 1576	MinVel6
global real tag 1584	AxisDec6
global real tag 1585	AxisAcc6
global real tag 1586	FaultTorq6
global real tag 1587	HomeTorq6
global real tag 1588	MaxTorq6

!Molly Axis 7 = ACS Axis 5 - Spare 7
global int 	tag 1600	InputStatus5
global int 	tag 1601	InstrumentID5
global int 	tag 1602	SWVersion5
global int 	tag 1603	DINPUT5
global real tag 1604	OUTP5
global int 	tag 1606	iControlMode5
global real tag 1607	pressurePID_KP5
global real tag 1608	pressurePID_KI5
global real tag 1609	pressurePID_KD5
global real tag 1610	OutPressure_KP5
global real tag 1611	Valve_Scaling_Factor5
global real tag 1612	PressureSP5
global real tag 1613	Pressure_Ramp_rate5		!Pressure ramp rate expressed at Torr/min
global real tag 1614	Pres_min5				!Min Pressure setpoint, 5e-11
global real tag 1615	Pres_max5				!Max pressure setpoint, 5e-5
global real tag 1616	SafetyShutDownPressure5	!Defines a pressure threshold for safety.  (5e-11 to 1e-4) 8e-5 is default
global int 	tag 1617	StatusWord5
global int 	tag 1618	KillTraj5
global real tag 1619	PressurePV5
global real tag 1620	PressureWKSP5
global int 	tag 1621	ControlProcessState5
global real tag 1650 	Measured5
global real tag 1651	WkSp5
global real tag 1667	Sp5	!command
global real tag 1668	ramprate5
global real tag 1670	VKP5					!servo velocity loop Kp
global real tag 1671	VKI5					!servo velocity loop Ki
global real tag 1672	PKP5					!servo position loop Kp
global real tag 1673	MaxSP5
global real tag 1674	MinSP5
global real tag 1675	MaxVel5
global real tag 1676	MinVel5
global real tag 1684	AxisDec5
global real tag 1685	AxisAcc5
global real tag 1686	FaultTorq5
global real tag 1687	HomeTorq5
global real tag 1688	MaxTorq5

!Molly Axis 8 = ACS Axis 7 - Spare 8
global int 	tag 1700	InputStatus7
global int 	tag 1701	InstrumentID7
global int 	tag 1702	SWVersion7
global int 	tag 1703	DINPUT7
global real tag 1704	OUTP7
global int 	tag 1706	iControlMode7
global real tag 1707	pressurePID_KP7
global real tag 1708	pressurePID_KI7
global real tag 1709	pressurePID_KD7
global real tag 1710	OutPressure_KP7
global real tag 1711	Valve_Scaling_Factor7
global real tag 1712	PressureSP7
global real tag 1713	Pressure_Ramp_rate7		!Pressure ramp rate expressed at Torr/min
global real tag 1714	Pres_min7				!Min Pressure setpoint, 5e-11
global real tag 1715	Pres_max7				!Max pressure setpoint, 5e-5
global real tag 1716	SafetyShutDownPressure7	!Defines a pressure threshold for safety.  (5e-11 to 1e-4) 8e-5 is default
global int 	tag 1717	StatusWord7
global int 	tag 1718	KillTraj7
global real tag 1719	PressurePV7
global real tag 1720	PressureWKSP7
global int 	tag 1721	ControlProcessState7
global real tag 1750 	Measured7
global real tag 1751	WkSp7
global real tag 1767	Sp7						!command
global real tag 1768	ramprate7
global real tag 1770	VKP7					!servo velocity loop Kp
global real tag 1771	VKI7					!servo velocity loop Ki
global real tag 1772	PKP7					!servo position loop Kp
global real tag 1773	MaxSP7
global real tag 1774	MinSP7
global real tag 1775	MaxVel7
global real tag 1776	MinVel7
global real tag 1784	AxisDec7
global real tag 1785	AxisAcc7
global real tag 1786	FaultTorq7
global real tag 1787	HomeTorq7
global real tag 1788	MaxTorq7


!Molly Modbus registers for communication with Meidensha ozone controller through DMC
!Meiden controller variables on Modbus
!register range: 1825-1844
global int tag 1826 CommandWord
!bit 0      POG_ON                             
!bit 1      POG_Feed_On
!bit 2      LC_O3_Bipass_ON
!bit 3      O2_Bipass_ON
!bit 4      OpenRunValve
!bit 5		O3_RunValve1
!bit 6		O3_RunValve2
!bit 7		O3_RunValve3
!bit 8		O3_RunValve4
global real tag 1827 OzonePresureSP
global real tag 1828 OzoneChamberSP
global real tag 1829 ChargeVolumeSP
global real tag 1830 LCOzoneFlowRateSP
global int 	tag 1836 StatusWord
!bit 0      RunValveOpen
global real tag	1837 DynNegativeLimit
global real tag	1838 DynPositiveLimit
global real tag	1839 mshLCZ	!Lower Crash Zone (LCZ) - main shutter
global real tag	1840 mshUCZ	!Upper Crash Zone (UCZ) - main shutter 
global real tag	1841 bfmLCZ !Lower Crash Zone (LCZ) - BFM
global real tag	1842 bfmUCZ !Upper Crash Zone (UCZ) - BFM
global real tag	1843 qcmLCZ !Lower Crash Zone (LCZ) - QCM
global real tag	1844 qcmUCZ !Upper Crash Zone (UCZ) - QCM
global real tag	1845 xfrLCZ !Lower Crash Zone (LCZ) - Transfer (robot)
global real tag	1846 xfrUCZ !Upper Crash Zone (UCZ) - Transfer (robot)
