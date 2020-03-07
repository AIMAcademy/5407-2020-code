package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.cscore.HttpCamera;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Spark;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonFX;


public class Shooter {


	// create your variables

	ShooterVelocity shootvel = null;
	TalonFX  motCANShooterMotorLeft = null;
	TalonFX  motCANShooterMotorRight = null;

	//  PID Closed Loop Settings for shooter
	static final int kSlotIdx = 0;  		// PID Slot 0 or 1 
	static final int kPIDLoopIdx = 0;		// PID loop. We only need one
	static final int kTimeoutMs = 30;		// Set to zero to skip waiting for confirmation
	double dPid_Proportional = 7.0;
	double dPid_Integral = 0.0008;
	double dPid_Derivative = 0.7;
	double dPid_FeedForward = 0.04447826;  // estimate of how many ticks we will see at any velocity in 100 ms.
	int iPid_IntegralZone = 100;
	double d75PctVelocity = 17250;
	double dRequestedPct = 0.0;
	double dVelocitySensitivity = 5.0;
	double dLastRequestedVelocity = 0.0;
	double dCLError = 0.0;
	double dCLErrorThreshold = 50.0;
	String sCLStatus = "****";
	boolean bInClosedLoopMode = false;
	boolean bUpToSpeed = false;


	Limelight limelight = null;
 	private final String LimelightHostname = "limelight";   // Limelight http camera feeds
	private HttpCamera limelightFeed;
	

	//TalonSRX motCANTurretMotor = null;
	AnalogInput anaTurretPos  = null;
	int iTurretLeftStop = 300;		// check config file
	int iTurretRigthStop = 3400;	// check config file
	String sTurretState = "";
	boolean bTurretOnTarget = false;
	boolean bTurretRequestedToPosition = false;
	int iTurretPosition = 0;

	Solenoid    solBallPusherUpper = null;

	Spark 		motPWMMouthMotor = null;
	public double dMouth_IntakePower = .50; 
	public double dMouth_ReversePower = -.50; 
	double dPWMMouthMotorPower = 0.0;

	FireSequence2 fireseq = null;

	DigitalInput digBallInPlace = null;
	boolean bBallInPlace = false;

	Servo    motShooterHood = null;
	AnalogInput anaShooterHood = null;
	public static final double kShooterHood_Stop = .5; 
	double dShooterHoodPower = .5;
	int iShooterHood = 0;
	int iShooterHoodTop = 3240;
	int iShooterHoodBottom = 2850;
	int iShooterHoodPosition = 0;
	String sHoodStatus = "---";
	int iDiff = 0;
	int iTurretTicksPerDegree = 5;


	boolean bLastShooterLaunch = false;             // what was the launch value last cycle.

	
    /**
     * This function is run when this class is first created used for any initialization code.
     */
    public Shooter(final Config config) {
		System.out.println("Shooter constructor init...");

		fireseq = new FireSequence2(this);  // do this here to be sure it is ready when loadConfig is called. 
		fireseq.reset();

		loadConfig(config); // do this here to be sure we have the values updated before we used them.

		// add limelight
		limelight = new Limelight(LimelightHostname);
		limelightFeed = new HttpCamera(LimelightHostname, "http://limelight.local:5800/stream.mjpg");

		motPWMMouthMotor = new Spark(RobotMap.kPWMPort_Mouth);

		digBallInPlace = new DigitalInput(RobotMap.kDigitalInPort_BallInPlace);

		solBallPusherUpper = new Solenoid(RobotMap.kPCMPort_BallPusherUpper);
		solBallPusherUpper.set(false);

		motCANShooterMotorRight = new TalonFX(RobotMap.kCANId_ShooterMotorRight);
		motCANShooterMotorRight.configFactoryDefault();
		/* Config sensor used for Primary PID [Velocity] */
		motCANShooterMotorRight.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, kPIDLoopIdx, kTimeoutMs);
		motCANShooterMotorRight.setSensorPhase(true);
		motCANShooterMotorRight.configClosedloopRamp(0);  	// ramp up time so we do not cause a huge surge in current 
		motCANShooterMotorRight.configNominalOutputForward(0, kTimeoutMs);
		motCANShooterMotorRight.configNominalOutputReverse(0, kTimeoutMs);
		motCANShooterMotorRight.configPeakOutputForward(1.0, kTimeoutMs);
		motCANShooterMotorRight.configPeakOutputReverse(-1.0, kTimeoutMs);
		motCANShooterMotorRight.set(ControlMode.Velocity, 0.0);

		motCANShooterMotorLeft = new TalonFX(RobotMap.kCANId_ShooterMotorLeft);
		motCANShooterMotorLeft.configFactoryDefault();
		/* Config sensor used for Primary PID [Velocity] */
		//motCANShooterMotorLeft.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, kPIDLoopIdx, kTimeoutMs);
		//motCANShooterMotorLeft.setSensorPhase(true);
		//motCANShooterMotorLeft.configClosedloopRamp(0);  	// ramp up time so we do not cause a huge surge in current 
		//motCANShooterMotorLeft.configNominalOutputForward(0, kTimeoutMs);
		//motCANShooterMotorLeft.configNominalOutputReverse(0, kTimeoutMs);
		//motCANShooterMotorLeft.set(ControlMode.Velocity, 0.0);
		//motCANShooterMotorLeft.configPeakOutputForward(1.0, kTimeoutMs);
		//motCANShooterMotorLeft.configPeakOutputReverse(-1.0, kTimeoutMs);
		motCANShooterMotorLeft.set(ControlMode.PercentOutput, 0.0);


		//motCANTurretMotor = new TalonSRX(RobotMap.kCANId_ShooterTurretMotor);
		//motCANTurretMotor.setInverted(false); // invert direction to match gearing

		motShooterHood = new Servo(RobotMap.kPWMPort_ShooterHoodMotor);
		anaShooterHood = new AnalogInput(RobotMap.kAnalogPort_ShooterHood);

		anaTurretPos = new AnalogInput(RobotMap.kAnalogPort_TurretPos);
		
		System.out.println("Shooter constructor end...");

	}

	/****public void reloadTables(){

		//shootvel.loadTable();

	}
	****/



	public void update(final Inputs inputs, final Config config) {

		dPWMMouthMotorPower = 0.0;									// kill this an below it will be reset
		this.updateShooterVelocity(inputs);							// spinn the shootr wheel
		// read the sensors so we all have them now
		//iTurretPosition = anaTurretPos.getAverageValue(); 
		iShooterHoodPosition = anaShooterHood.getAverageValue();

		if(inputs.bTargetting == true){
			this.Targetting(inputs);
		}

		//TurretPosition(inputs);
		HoodPosition(inputs);

		if (digBallInPlace.get() == false) {		//Device is set for normally closed
			bBallInPlace = false;   			//We have a connection so it's set to false
		}else {
			bBallInPlace = true;				//Indicates open circut, means ball inplace or wiring failure
		}
		
		if( inputs.bIntakeIn == true)
			LoadABall();
			

		/**
		 * turret mouth processing
		 **/
	
		if (inputs.bShooterLaunch == true){
			if( bLastShooterLaunch == false ){
				fireseq.reset();
			}

			fireseq.execute(inputs); //  this refers to the shooter itself. 
		}

		//motPWMMouthMotor.set(dPWMMouthMotorPower);
		//SmartDashboard.putNumber("Sh Mouth Power", dPWMMouthMotorPower);
		bLastShooterLaunch = inputs.bShooterLaunch;

	}

	public void loadConfig(final Config config) {

		dPid_Proportional = config.getDouble("shooter.dPID_P", 0.731);
		dPid_Integral = config.getDouble("shooter.dPID_I", 0.0008);
		dPid_Derivative = config.getDouble("shooter.dPID_D", 7.0);
		dPid_FeedForward = config.getDouble("shooter.dPID_F", 0.04447826);
		dVelocitySensitivity = config.getDouble("shooter.dVelocitySensitivity", 25.0);
		iPid_IntegralZone = config.getInt("shooter.iPID_IntegralZone", 100);
		dCLErrorThreshold = config.getDouble("shooter.dCLErrorThreshold", 50.0);
		iTurretLeftStop = config.getInt("shooter.iTurretLeftStop", 300);
		iTurretRigthStop = config.getInt("shooter.iTurretRigthStop", 3400);
		
		iShooterHoodBottom = config.getInt("shooter.iShooterHoodBottom", 500);
		iShooterHoodTop = config.getInt("shooter.iShooterHoodTop", 1000);

		fireseq.loadConfig(config);

	}

	public boolean LoadABall(){

		if( this.bBallInPlace == true ){
			dPWMMouthMotorPower = dMouth_ReversePower;
			return this.bBallInPlace;
		}

		dPWMMouthMotorPower = dMouth_IntakePower;
		return false;

	}

	public void updateShooterSettings() {
		/* Config the Velocity closed loop gains in slot0 */
		motCANShooterMotorRight.config_kP(kPIDLoopIdx, dPid_Proportional, kTimeoutMs);
		motCANShooterMotorRight.config_kI(kPIDLoopIdx, dPid_Integral, kTimeoutMs);
		motCANShooterMotorRight.config_kD(kPIDLoopIdx, dPid_Derivative, kTimeoutMs);
		motCANShooterMotorRight.config_kF(kPIDLoopIdx, dPid_FeedForward, kTimeoutMs);
		motCANShooterMotorRight.config_IntegralZone(kPIDLoopIdx, iPid_IntegralZone);
	}

	public void updateShooterVelocity(final Inputs inputs) {
		dCLError = motCANShooterMotorRight.getClosedLoopError();

		dRequestedPct = (.75 * inputs.dRequestedVelocity)/d75PctVelocity;

		sCLStatus = "Stopped";
		bUpToSpeed = false;
	
		if( inputs.dRequestedVelocity  < 8000) {
			bInClosedLoopMode = false;
			sCLStatus = "PCTMode";
			motCANShooterMotorRight.set(ControlMode.PercentOutput, dRequestedPct);
		} else { 

			bInClosedLoopMode = true;

			if(inputs.dRequestedVelocity != dLastRequestedVelocity){
				motCANShooterMotorRight.set(ControlMode.Velocity, inputs.dRequestedVelocity);
				double dRightPCT = motCANShooterMotorRight.getMotorOutputPercent();
				motCANShooterMotorLeft.set(ControlMode.PercentOutput, -dRightPCT);

				updateShooterSettings();
			}
			else {
				if( Math.abs(dCLError) < dCLErrorThreshold ){
					sCLStatus = "PID Ready";
					bUpToSpeed = true;
				} else {
					sCLStatus = "PID Adjust";
					bUpToSpeed = false;
				}
			}

		}

		dLastRequestedVelocity = inputs.dRequestedVelocity;

	
	}

	private void Targetting( Inputs inputs){

		double ty = limelight.getTy();
		int iRequestedTicks = 0;

		if( Math.abs(ty) > .5){
			iRequestedTicks = (int) ( ty * (double) iTurretTicksPerDegree);
			inputs.iTurretRequestedToPosition = iTurretPosition - iRequestedTicks; 
		}

	}

	private void HoodPosition( Inputs inputs){

		iShooterHoodPosition = anaShooterHood.getAverageValue();
		inputs.iHoodRequestedToPosition = iShooterHoodBottom + 40;

		if( Math.abs(inputs.dHoodPower) < .2 ){					// dead band to prevent accidental hits
			inputs.dHoodPower = 0.0;
			sHoodStatus = "Stopped";
		}

 		if( inputs.dHoodPower == 0.0 ){
			int iHoodDiff = -1;
			// negative power is ??, positive power is ??
			if(inputs.iHoodRequestedToPosition > -1) {
				sHoodStatus = "Reposit";
				iHoodDiff = inputs.iHoodRequestedToPosition - iShooterHoodPosition;
				SmartDashboard.putNumber("sh Hood Diff", iHoodDiff);
				if( Math.abs(iDiff) < 5 ){
					sHoodStatus = "Hit Pos";
					inputs.dHoodPower = 0.0;
				} else { 
					inputs.dHoodPower =  (double) iDiff / 15.0;  // proportional response
					if( Math.abs(inputs.dHoodPower  ) < .10){
						if( inputs.dHoodPower < 0.0 ){
							inputs.dHoodPower = -.15;
						} else {
							inputs.dHoodPower = .15;
						}
					}
				}

			} 
		}

		// test for the hood stops
		if( inputs.dHoodPower > 0.0 &&					// going up 
				iShooterHoodPosition > iShooterHoodTop){
			sHoodStatus = "Full Up";
			inputs.dHoodPower = 0.0;
		} else if( inputs.dHoodPower < 0.0 && 			// going down
				iShooterHoodPosition > iShooterHoodTop){
			sHoodStatus = "Full Down";
			inputs.dHoodPower = 0.0;
		}

		/** Convert requested power in range 1.0 to -1.0 to the servo range 
		 * This Hood motor is a continuois turn servo
		 * deadband we set at .2/-.2 up above 
		 * Range greater than .5 is up
		 * 		  .5 =  stop
		 * 		 less than .5 is down 
		 * We will pass the joystick power and modify the fit the
		 * new range for the servo.  
		 * */

		
		double temp = inputs.dShooterHoodPower;	// save current power
		if( Math.abs(temp) < .2 ){					// dead band to prevent accidental hits
			temp = 0.0;
		}
		temp = temp * Math.abs(temp*temp);			// dsensitizie the lower end .5 stop.

		temp += 1.0;								// shift from 1.0 / -1.0 to 2.0 / 0.0
		temp /= 2;									// shift from 2.0 / 0.0 to 1.0 / 0.0, 
		this.dShooterHoodPower = temp;			// save result to local variable to save in telem and display
	
		/** Range is now set 0.0 to 1.0.  .5 is stop.
		 * But 0.0 is up and 1.0 is down. 
		 * We need to flip it so 1.0 is up and 0.0 is down
		 * It is a servo so we cannot just use motor invert 
		 **/
		this.dShooterHoodPower -= .5;  	// subtract the mid point .5. Convert from 0.0/1.0 to -.5/.5
		this.dShooterHoodPower *= -1;	// now we can invert. Converts from -.5/5 to .5/-.5 	
		this.dShooterHoodPower += .5;	// now add back .5   .5/-.5 becomes 1.0/0.0 which is what we want.

		motShooterHood.set(this.dShooterHoodPower);


	}

	private void TurretPosition( Inputs inputs){

		sTurretState = "Stopped"; 

		if( Math.abs(inputs.dTurretPower) < .10 ){  // deadband
			sTurretState = "Stopped"; 
			inputs.dTurretPower = 0.0;				// set input to 0 to stop it. 
		}

		if( Math.abs(inputs.dTurretPower) == 0.0 ){  // if no one is trying to turn procees position requests

			if( inputs.dTurretPOV == 0){
				inputs.iTurretRequestedToPosition = 1291;
				inputs.dRequestedBearing = 0.0;
			} else if(inputs.dTurretPOV == 90){
				inputs.iTurretRequestedToPosition = 500;
				inputs.dRequestedBearing = 90.0;
			} else if(inputs.dTurretPOV == 270){
				inputs.dRequestedBearing = 270;
				inputs.iTurretRequestedToPosition = 2190;
			}


			iDiff = -1;
			// negative power is left, positive power is right
			if(inputs.iTurretRequestedToPosition > -1) {
				sTurretState = "Reposit";
				iDiff = inputs.iTurretRequestedToPosition - iTurretPosition;
				if( Math.abs(iDiff) < 5 ){
					sTurretState = "Hit Pos";
					inputs.dTurretPower = 0.0;
				} else { 
					inputs.dTurretPower =  (double) iDiff / 400.0;  // proportional response
					if( Math.abs(inputs.dTurretPower  ) < .15){
						if( inputs.dTurretPower < 0.0 ){
							inputs.dTurretPower = -.15;
						} else {
							inputs.dTurretPower = .15;
						}
					}
				}

			} 
		}

		// now process turret requests from either position requests or operator

		if( inputs.dTurretPower < 0.0 && 			// asking to go left (negative)
			iTurretPosition <= iTurretLeftStop){	// we are past the left soft stop
			inputs.dTurretPower = 0.0;				// set input to 0 to stop it.
			sTurretState = "Full Left"; 
		}
		
		if( inputs.dTurretPower > 0.0 && 			// asking to go right (positive)
			iTurretPosition >= iTurretRigthStop  ){ // we are at or past the right soft stop
			inputs.dTurretPower = 0.0;				// set input to 0 to stop it. 
			sTurretState = "Full Right"; 
		}

		if(inputs.dTurretPower < 0.0){					// negative wants to go left
			sTurretState = "<<<<<"; 

		} else if(inputs.dTurretPower > 0.0){			// positive want to go right
			sTurretState = ">>>>>";
		}


		//if( inputs.dTurretPower >  .20) inputs.dTurretPower =  .20;
		//if( inputs.dTurretPower < -.20) inputs.dTurretPower = -.20;
		SmartDashboard.putNumber("Sh Turret Req Pow", inputs.dTurretPower);

		//inputs.dTurretPower = 0.0;
		//motCANTurretMotor.set(ControlMode.PercentOutput, -inputs.dTurretPower);	// invert for direction 

	}

    public void addTelemetryHeaders(final LCTelemetry telem ){
		telem.addColumn("Sh CL PID P");
		telem.addColumn("Sh CL PID I");
		telem.addColumn("Sh CL PID D");
		telem.addColumn("Sh CL PID F");

		telem.addColumn("Sh CL Error");
		telem.addColumn("Sh CL Sen Vel");
		telem.addColumn("Sh CL Target Vel");
		telem.addColumn("IN Shooter Launch");
		telem.addColumn("Sh Turret State");
		telem.addColumn("Sh Hood");
		telem.addColumn("Sh Hood Power");
		telem.addColumn("Sh On Target");
		telem.addColumn("Sh Up To Speed");
		telem.addColumn("Sh Ball In Place");
		telem.addColumn("Sh Turr Pos");
		telem.addColumn("Sh Turr Req Pos");

		
		fireseq.addTelemetryHeaders(telem);		// do these here as we have access to telem
		limelight.addTelemetryHeaders(telem);

	}

    public void writeTelemetryValues(final LCTelemetry telem, Inputs inputs ){
		telem.saveDouble("Sh CL PID P", dPid_Proportional, 6 );
		telem.saveDouble("Sh CL PID I", dPid_Integral, 6 );  // need more decimals here, default = 2
		telem.saveDouble("Sh CL PID D", dPid_Derivative, 6 );
		telem.saveDouble("Sh CL PID F", dPid_FeedForward, 6 );

		if(bInClosedLoopMode){
			telem.saveDouble("Sh CL Error", motCANShooterMotorRight.getClosedLoopError() );
			telem.saveDouble("Sh CL Sen Vel", motCANShooterMotorRight.getSelectedSensorVelocity() );
			telem.saveDouble("Sh CL Target Vel", motCANShooterMotorRight.getClosedLoopTarget());
		}

		telem.saveDouble("Sh Hood Power", this.dShooterHoodPower);
		telem.saveDouble("Sh Hood Posit", this.iShooterHoodPosition);
		telem.saveTrueBoolean("In Shooter Launch", inputs.bShooterLaunch);
		telem.saveString("Sh Turret State", sTurretState);
		telem.saveTrueBoolean("Sh Turret On Target", bTurretOnTarget);
		telem.saveInteger("Sh Turr Pos", iTurretPosition);
		telem.saveInteger("Sh Turr Req Pos", inputs.iTurretRequestedToPosition);
		telem.saveDouble("Sh Turr Power", inputs.dTurretPower);
		telem.saveTrueBoolean("Sh Up To Speed", this.bUpToSpeed);
		telem.saveTrueBoolean("Sh Ball In Place", bBallInPlace);

		fireseq.writeTelemetryValues(telem);			// do these here as we have access to telem
		limelight.writeTelemetryValues(telem);
    }
	
	public void outputToDashboard(final boolean b_MinDisplay)  {
		
		SmartDashboard.putNumber("Sh CL Sen Vel", motCANShooterMotorRight.getSelectedSensorVelocity() );
		if(bInClosedLoopMode) SmartDashboard.putNumber("Sh CL Target", motCANShooterMotorRight.getClosedLoopTarget() );
		SmartDashboard.putString("Sh CL Status", sCLStatus );
		SmartDashboard.putBoolean("Sh Turret On Target", bTurretOnTarget);
		SmartDashboard.putBoolean("Sh Up To Speed", bUpToSpeed );
		SmartDashboard.putBoolean("Sh Ball In Place", bBallInPlace);
		SmartDashboard.putString("Sh Turret State", sTurretState );
		SmartDashboard.putNumber("Sh Hood Calc Power", dShooterHoodPower);
		SmartDashboard.putString("Sh Hood Status", sHoodStatus);
		SmartDashboard.putNumber("Sh Turret Pos", iTurretPosition);
		SmartDashboard.putNumber("Sh iDiff", iDiff);

		
		if( b_MinDisplay) return;						// minimum display to save bandwidth

		SmartDashboard.putNumber("Sh CL PID P", dPid_Proportional );
		SmartDashboard.putNumber("Sh CL PID I", dPid_Integral );  // need more decimals here, default = 2
		SmartDashboard.putNumber("Sh CL PID D", dPid_Derivative );
		SmartDashboard.putNumber("Sh CL PID F", dPid_FeedForward );
		SmartDashboard.putNumber("Sh CL Error", dCLError );
		SmartDashboard.putNumber("Sh CL Req Pct", dRequestedPct);
		SmartDashboard.putNumber("Sh Turret Pos", anaTurretPos.getAverageValue());
		SmartDashboard.putNumber("Sh Hood Pos", anaShooterHood.getAverageValue() );
		//SmartDashboard.putNumber("Sh Hood Pow", motS );
	}
}


class FireSequence2{

	Shooter shooter = null;
	int iStep = 0;
	int iNextStep = 0;
	int iLastStep = -1;
	boolean bStepIsSetUp = false;
	Timer timStepTimer = null;
	String sState = "Reset";
	double dClearBallTime = .15;			// time to allow ingested ball to settle in shooter
	double dLaunchResetTime = .10;			// time to allow launched ball to clear the shooter

	FireSequence2(Shooter mPassedShooter){
		shooter = mPassedShooter;
		timStepTimer = new Timer();
		timStepTimer.start();
	}

	void loadConfig(Config config){
		dLaunchResetTime = config.getDouble("firesequence.dLaunchResetTime", .10);
		dClearBallTime = config.getDouble("firesequence.dClearBallTime", .15);
	}

	void reset(){
		iStep = 0;
		iNextStep = 0;
		iLastStep = -1;
		sState = "Init";
	}

	void execute(Inputs inputs){

		/**
		 * We need iNextStep as a way to indicate in a step where we are going next.
		 * Prefer this to changing iStep in a step before we record telemetry. 
		 * Without this we see iStep change and not reflect correct state when 
		 * recorded in telemetry.
		 * 
		 * Now iStep changes to iNextStep below before the actual next step section and 
		 * after the previous iStep is completed and recorded. 
		 */
		iStep = iNextStep;					// do these here to change to the next step. 

		if( iLastStep != iStep){			// we have move to a new step. Get ready. 
			timStepTimer.reset();			// reset the "step" timer so we know how long we have been in the step
			bStepIsSetUp = false;			// used to set up contitiones within a step before it starts
		}

		iLastStep = iStep;					// save this for the next loop

		shooter.dPWMMouthMotorPower = 0.0;  // set the default for this motor to keep it from running
											// this value, if unchanged below, will be applied outside this 
											// class in shooter.

		SmartDashboard.putNumber("FS2 iStep", iStep);
		SmartDashboard.putNumber("FS2 Timer", timStepTimer.get() );
		SmartDashboard.putString("FS2 State", sState );

		switch (iStep) {

			case 0:			// Initilaize any thing you need to 
				sState = "Init";
				shooter.solBallPusherUpper.set(false);
				iNextStep = iStep+1;
				break;

			case 1:									// see if we have a ball in place to shoot
				sState = "Reset";					// test to see if we can remove this step. Saves .10 sec
				iNextStep = iStep + 1;	  			// no ball, go get one 

				/**
				shooter.solBallPusherUpper.set(false);
				shooter.dPWMMouthMotorPower = shooter.dMouth_ReversePower;

				if(timStepTimer.get() > .10){
					iNextStep = iStep + 1;	  			// no ball, go get one 
				}
				**/

				break;


			case 2:										// see if we have a ball in place to shoot
				sState = "Ball In Place";
				if(shooter.bBallInPlace == true){
					iNextStep=10;							// ball ready to fire
				}else{
					iNextStep=3;							// no ball, go get one 
				}

				break;

			case 3:
				sState = "Load Ball";
				shooter.LoadABall(); // pull in the next ball

				if(shooter.bBallInPlace == true){				// we see a ball
					shooter.dPWMMouthMotorPower = 0.0;	// stop pull
					iNextStep = iStep + 1;							
				}

				break;

			case 4:
				sState = "Settle Ball";

				if(shooter.bBallInPlace == false){			// no ball go back to step 2
					iNextStep = iStep - 1;					//previous step
					break;
				}

				shooter.dPWMMouthMotorPower = 
							shooter.dMouth_ReversePower;	// this will pull back the nexf ball if there is one. 

				if( timStepTimer.get() > .10 && shooter.bBallInPlace == true  ){  	// do this for .15 seconds and ball in place
					shooter.dPWMMouthMotorPower = 0.0;						  		// times up, stop motor. 
					iNextStep = 10;													// step 10
				}

				break;

			case 10:
				sState = "Final Check";
				if(shooter.bBallInPlace == false){			// final check if we have a ball
					iNextStep = 3;							// look for ball
				} else {
					iNextStep = 20;
				}

				break;

			case 20:
				sState = "Speed?";
				if( shooter.bUpToSpeed == true){
					iNextStep = 21;
				}

				break;

			case 21:
				sState = "Launching";
				shooter.solBallPusherUpper.set(true);

				if( timStepTimer.get() > .15 ){		// let ball get out 
					shooter.solBallPusherUpper.set(false);
					iNextStep = 1;									// restart
				}	

				break;

		}
	}

	public void addTelemetryHeaders(final LCTelemetry telem ){
		telem.addColumn("Sh FS2 Step Time");
		telem.addColumn("Sh FS2 Step");
		telem.addColumn("Sh FS2 State");
		telem.addColumn("Sh FS2 Next Step");
	}

    public void writeTelemetryValues(final LCTelemetry telem ){
		telem.saveDouble("Sh FS2 Step Time", timStepTimer.get(), 2 ); 
		telem.saveInteger("Sh FS2 Step", iStep );
		telem.saveInteger("Sh F2S NextStep", iNextStep );
		telem.saveString("Sh FS2 State", sState );
    }
	
}
