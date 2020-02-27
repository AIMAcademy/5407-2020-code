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
	double dPid_Proportional = 7.0;
	double dPid_Integral = 0.0008;
	double dPid_Derivative = 0.7;
	double dPid_FeedForward = 0.04447826;  // estimate of how many ticks we will see at any velocity in 100 ms.
	int iPid_IntegralZone = 100;
	double d75PctVelocity = 17250;
	double dRequestedPct = 0.0;
	double dVelocitySensitivity = 5.0;
	//double dRequestedVelocity = 0.0;
	double dLastRequestedVelocity = 0.0;
	//double dCLAdjust = 0.0;
	double dCLError = 0.0;
	//double dCLErrorCorrectionFactor = 0.0000001;
	//boolean bCLAdjustedError = false;
	double dCLErrorThreshold = 50.0;
	String sCLStatus = "****";
	boolean bInClosedLoopMode = false;
	boolean bUpToSpeed = false;

	Limelight limelight = null;
  
	// Limelight http camera feeds
	private final String LimelightHostname = "limelight";
	private HttpCamera limelightFeed;
	
	/**
	 * Which PID slot to pull gains from. Starting 2018, you can choose from
	 * 0,1,2 or 3. Only the first two (0,1) are visible in web-based
	 * configuration.
	 */
	static final int kSlotIdx = 0;

	/**
	 * Talon SRX/ Victor SPX will supported multiple (cascaded) PID loops. For
	 * now we just want the primary one.
	 */
	static final int kPIDLoopIdx = 0;

	/**
	 * Set to zero to skip waiting for confirmation, set to nonzero to wait and
	 * report to DS if action fails.
	 */
	static final int kTimeoutMs = 30;
	

	
	int 	i_PositionDelta;
	boolean b_InPosition;
	boolean b_IsClose;
	Timer 	tim_FireTimer;
	boolean b_StepIsSetup = false;
	int		i_Step = 0;

	TalonFX  motCANShooterMotor = null;
	Timer    timAdjust		  = null;

	TalonSRX motCANTurretMotor = null;
	AnalogInput anaTurretPos  = null;

	Solenoid    solBallPusherUpper = null;
	Spark 		motPWMMouthMotor = null;
	FireSequence2 fireseq = null;
	public double dMouth_IntakePower = .50; 
	public double dMouth_ReversePower = -.50; 



	DigitalInput digBallInPlace = null;
	boolean bBallInPlace = false;

	Servo    motShooterHeight = null;
	AnalogInput anaShooterHeight = null;
	public static final double kShooterHeight_Up = 1.0; 
	public static final double kShooterHeight_Stop = .5; 
	public static final double kShooterHeight_Down = 0.0;

	double dShooterHeightPower= kShooterHeight_Stop;


	int iShooterHeight = 0;
	int iShooterHeightTop =0;
	int iShooterHeightBottom = 0;

	int iTurretLeftStop = 300;		// check config file
	int iTurretRigthStop = 3400;	// check config file
	String sTurretState = "";

	double dPWMMouthMotorPower = 0.0;

	boolean bLastShooterLaunch = false;
	
    /**
     * This function is run when this class is first created used for any initialization code.
     */
    public Shooter(final Config config) {
		System.out.println("Shooter constructor init...");

		//mFireSequence = new FireSequence(this);  // do this here to be sure it is ready when loadConfig is called. 
		//mFireSequence.reset();

		fireseq = new FireSequence2(this);  // do this here to be sure it is ready when loadConfig is called. 
		fireseq.reset();

		loadConfig(config); // do this here to be sure we have the values updated before we used them.

		// add limelight
		limelight = new Limelight(LimelightHostname);
		limelightFeed = new HttpCamera(LimelightHostname, "http://limelight.local:5800/stream.mjpg");

		timAdjust = new Timer();
		timAdjust.stop();

		motPWMMouthMotor = new Spark(RobotMap.kPWMPort_Mouth);

		digBallInPlace = new DigitalInput(RobotMap.kDigitalInPort_BallInPlace);

		solBallPusherUpper = new Solenoid(RobotMap.kPCMPort_BallPusherUpper);
		solBallPusherUpper.set(false);

		motCANShooterMotor = new TalonFX(RobotMap.kCANId_ShooterMotor);
		motCANShooterMotor.configFactoryDefault();
		/* Config sensor used for Primary PID [Velocity] */
		motCANShooterMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, kPIDLoopIdx, kTimeoutMs);
		motCANShooterMotor.setSensorPhase(true);
		motCANShooterMotor.configClosedloopRamp(3);  	// ramp up time so we do not cause a huge surge in current 
		motCANShooterMotor.configNominalOutputForward(0, kTimeoutMs);
		motCANShooterMotor.configNominalOutputReverse(0, kTimeoutMs);
		motCANShooterMotor.set(ControlMode.Velocity, 0.0);
		motCANShooterMotor.configPeakOutputForward(1.0, kTimeoutMs);
		motCANShooterMotor.configPeakOutputReverse(-1.0, kTimeoutMs);
		motCANShooterMotor.set(ControlMode.Velocity, 0.0);
	
		motCANTurretMotor = new TalonSRX(RobotMap.kCANId_ShooterTurretMotor);
		motCANTurretMotor.setInverted(false); // invert direction to match gearing

		motShooterHeight = new Servo(RobotMap.kPWMPort_ShooterHeight);
		anaShooterHeight = new AnalogInput(RobotMap.kAnalogPort_ShooterHeight);

		anaTurretPos = new AnalogInput(RobotMap.kAnalogPort_TurretPos);
		
		System.out.println("Shooter constructor end...");

	}

	/****public void reloadTables(){

		//shootvel.loadTable();

	}
	****/



	public void update(final Inputs inputs, final Config config) {

		this.updateShooterVelocity(inputs);							// spinn the shootr wheel

		// motCANTurretMotor.set(ControlMode.PercentOutput, inputs.dTurretPower);

		if (digBallInPlace.get() == false) {		//Device is set for normally closed
			bBallInPlace = false;   			//We have a connection so it's set to false
		}else {
			bBallInPlace = true;				//Indicate open circut, means ball inplace or wiring failure
		}
		

		dShooterHeightPower= kShooterHeight_Stop;
		if (inputs.bShooterHeightRaise == true)
			dShooterHeightPower = kShooterHeight_Up;
		else if (inputs.bShooterHeightLower == true)
			dShooterHeightPower = kShooterHeight_Down;
  
		if(  dShooterHeightPower == kShooterHeight_Down && anaShooterHeight.getAverageValue() < 567 )
			dShooterHeightPower= kShooterHeight_Stop;

		if(  dShooterHeightPower == kShooterHeight_Up && anaShooterHeight.getAverageValue() < 2000 )
			dShooterHeightPower= kShooterHeight_Stop;
	
		motShooterHeight.set(dShooterHeightPower);
		//turret position
		sTurretState = "Stopped"; 

		// negative power is left, positive power is right
		if(inputs.dTurretPower <= 0.0){					// negative wants to go left
			sTurretState = "<<<<<"; 

			if(anaTurretPos.getAverageValue() <= iTurretLeftStop){
				inputs.dTurretPower = 0.0;				// set input to 0 to stop it.
				sTurretState = "Full Left"; 
			} 
		} else if(inputs.dTurretPower >= 0.0){			// positive want to go right
			sTurretState = ">>>>>";

			if(anaTurretPos.getAverageValue() >= iTurretRigthStop ){
				inputs.dTurretPower = 0.0;				// set input to 0 to stop it. 
				sTurretState = "Full Right"; 
			}
		} else {
			sTurretState = "Stopped"; 
			inputs.dTurretPower = 0.0;				// set input to 0 to stop it. 
		}

		motCANTurretMotor.set(ControlMode.PercentOutput, -inputs.dTurretPower);	// invert for direction 

		/**
		 * turret mouth processing
		 **/
		dPWMMouthMotorPower = 0.0;					

		if (inputs.bShooterLaunch == true){
			if( bLastShooterLaunch == false ){
				fireseq.reset();
			}

			fireseq.execute(inputs); //  this refers to the shooter itself. 
		}

		bLastShooterLaunch = inputs.bShooterLaunch;
		motPWMMouthMotor.set(dPWMMouthMotorPower);
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
		
		iShooterHeightBottom = config.getInt("shooter.iShooterHeightBottom", 500);
		iShooterHeightTop = config.getInt("shooter.iShooterHeightTop", 1000);

		fireseq.loadConfig(config);

	}



	public void updateShooterSettings() {
		/* Config the Velocity closed loop gains in slot0 */
		motCANShooterMotor.config_kP(kPIDLoopIdx, dPid_Proportional, kTimeoutMs);
		motCANShooterMotor.config_kI(kPIDLoopIdx, dPid_Integral, kTimeoutMs);
		motCANShooterMotor.config_kD(kPIDLoopIdx, dPid_Derivative, kTimeoutMs);
		motCANShooterMotor.config_kF(kPIDLoopIdx, dPid_FeedForward, kTimeoutMs);
		motCANShooterMotor.config_IntegralZone(kPIDLoopIdx, iPid_IntegralZone);
	}

	public void updateShooterVelocity(final Inputs inputs) {
		dCLError = motCANShooterMotor.getClosedLoopError();

		dRequestedPct = (.75 * inputs.dRequestedVelocity)/d75PctVelocity;

		//final double dAdjustTime = timAdjust.get();
		sCLStatus = "Stopped";
		bUpToSpeed = false;
	
		if( inputs.dRequestedVelocity  < 8000) {
			bInClosedLoopMode = false;
			sCLStatus = "PCTMode";
			motCANShooterMotor.set(ControlMode.PercentOutput, dRequestedPct);
		} else { 

			bInClosedLoopMode = true;

			if(inputs.dRequestedVelocity != dLastRequestedVelocity){
				motCANShooterMotor.set(ControlMode.Velocity, inputs.dRequestedVelocity);
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

	private boolean isShooterReadyToFire(Inputs inputs) {

		if( bUpToSpeed == true)
			return true;

    	return false;
    }


    public void addTelemetryHeaders(final LCTelemetry telem ){
		telem.addColumn("Sh CL PID P");
		telem.addColumn("Sh CL PID I");
		telem.addColumn("Sh CL PID D");
		telem.addColumn("Sh CL PID F");
		telem.addColumn("Sh CL Error");
		telem.addColumn("Sh CL Sen Vel");
		telem.addColumn("Sh CL Target Vel");
		telem.addColumn("Sh Turret State");
		telem.addColumn("Sh Turret State");
		telem.addColumn("Sh Up To Speed");
		telem.addColumn("Sh Ball In Place");

		fireseq.addTelemetryHeaders(telem);		// do these here as we have access to telem

	}

    public void writeTelemetryValues(final LCTelemetry telem ){
		telem.saveDouble("Sh CL PID P", dPid_Proportional, 6 );
		telem.saveDouble("Sh CL PID I", dPid_Integral, 6 );  // need more decimals here, default = 2
		telem.saveDouble("Sh CL PID D", dPid_Derivative, 6 );
		telem.saveDouble("Sh CL PID F", dPid_FeedForward, 6 );
		if(bInClosedLoopMode){
			telem.saveDouble("Sh CL Error", motCANShooterMotor.getClosedLoopError() );
			telem.saveDouble("Sh CL Sen Vel", motCANShooterMotor.getSelectedSensorVelocity() );
			telem.saveDouble("Sh CL Target Vel", motCANShooterMotor.getClosedLoopTarget());
		}
		telem.saveString("Sh Turret State", sTurretState);
		telem.saveTrueBoolean("Sh Up To Speed", this.bUpToSpeed);
		telem.saveBoolean("Sh Ball In Place", bBallInPlace);

		fireseq.writeTelemetryValues(telem);			// do these here as we have access to telem
    }
	
	public void outputToDashboard(final boolean b_MinDisplay)  {
		
		SmartDashboard.putNumber("Sh CL Sen Vel", motCANShooterMotor.getSelectedSensorVelocity() );
		if(bInClosedLoopMode) SmartDashboard.putNumber("Sh CL Target", motCANShooterMotor.getClosedLoopTarget() );
		SmartDashboard.putString("Sh CL Status", sCLStatus );
		SmartDashboard.putBoolean("Sh Up To Speed", bUpToSpeed );
		SmartDashboard.putBoolean("Sh Ball In Place", bBallInPlace);
		SmartDashboard.putString("Sh Turret State", sTurretState );

		if( b_MinDisplay) return;						// minimum display to save bandwidth

		SmartDashboard.putNumber("Sh CL PID P", dPid_Proportional );
		SmartDashboard.putNumber("Sh CL PID I", dPid_Integral );  // need more decimals here, default = 2
		SmartDashboard.putNumber("Sh CL PID D", dPid_Derivative );
		SmartDashboard.putNumber("Sh CL PID F", dPid_FeedForward );
		SmartDashboard.putNumber("Sh CL Error", dCLError );
		SmartDashboard.putNumber("Sh CL Req Pct", dRequestedPct);
		SmartDashboard.putNumber("Sh Turret Pos", anaTurretPos.getAverageValue());
		SmartDashboard.putNumber("Sh Height Pos", anaShooterHeight.getAverageValue() );
		SmartDashboard.putNumber("Sh Height Pow", dShooterHeightPower );
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
		 * We need INextStep as a way to indicate in a step where we are going next.
		 * Prefer this to changing iStep in a step before we record telemetry. 
		 * Without this we see iStep change and not reflect correct state when 
		 * recorded in telemetry.
		 * 
		 * Now iStep changes to the Next Step here before the next step and 
		 * after the previous iStep is completed and recorded. 
		 */
		iStep = iNextStep;					// do these here to change step. 

		if( iLastStep != iStep){
			timStepTimer.reset();
			bStepIsSetUp = false;			// used to set up contitiones within a step before it starts
		}

		iLastStep = iStep;					// save this for the next loop

		shooter.dPWMMouthMotorPower = 0.0;
		SmartDashboard.putNumber("FS2 iStep", iStep);
		SmartDashboard.putNumber("FS2 Timer", timStepTimer.get() );
		SmartDashboard.putString("FS2 State", sState );

		switch (iStep) {

			case 0:			// Initilaize any thing you need to 
				sState = "Init";
				shooter.solBallPusherUpper.set(false);
				iNextStep = iStep+1;
				break;

			case 1:										// see if we have a ball in place to shoot
				sState = "Reset";
				shooter.solBallPusherUpper.set(false);
				shooter.dPWMMouthMotorPower = shooter.dMouth_ReversePower;

				if(timStepTimer.get() > .10){
					iNextStep = iStep + 1;	  			// no ball, go get one 
				}

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
				sState = "Get Ball";
				shooter.dPWMMouthMotorPower  = shooter.dMouth_IntakePower; // pull in the next ball

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

				if( timStepTimer.get() > .20 ){		// let ball get out 
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
