package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.cscore.HttpCamera;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DigitalInput;
//import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Spark;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
//import com.ctre.phoenix.sensors.CANCoder;


public class Shooter {


	// create your variables

	ShooterVelocity shootvel = null;
	double dPid_Proportional = 0.2;
	double dPid_Integral = 0.0008;
	double dPid_Derivative = 2.0;
	double dPid_FeedForward = 0.04447826;  // estimate of how many ticks we will see at any velocity in 100 ms.
	double d75PctVelocity = 17250;
	double dRequestedPct = 0.0;
	double dVelocitySensitivity = 5.0;
	//double dRequestedVelocity = 0.0;
	double dLastRequestedVelocity = 0.0;
	//double dCLAdjust = 0.0;
	double dCLError = 0.0;
	//double dCLErrorCorrectionFactor = 0.0000001;
	//boolean bCLAdjustedError = false;
	double dCLErrorThreshold = 15;
	String sCLStatus = "****";
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
	TalonSRX motCANTurretMotor = null;
	Timer    timAdjust		  = null;
	AnalogPotentiometer anaTurretPos  = null;
	Solenoid    solBallPusherUpper = null;
	Spark motPWMMouthMotor = null;

	DigitalInput digBallInPlace = null;
	boolean bBallInPlace = false;

	Servo    motShooterHeight = null;
	AnalogInput anaShooterHeight = null;
	public static final double kShooterHeight_Up = 1.0; 
	public static final double kShooterHeight_Down = 0.0;
	public static final double kShooterHeight_Stop = .5; 

	double dShooterHeightPower= kShooterHeight_Stop;


	int iShooterHeight = 0;
	int iShooterHeight_Top =0;
	int iShooterHeight_Bottom = 0;

	double dTurretLeftStop = .9;
	double dTurretRigthStop = .1;
	String sTurretState = "";
	
    /**
     * This function is run when this class is first created used for any initialization code.
     */
    public Shooter(final Config config) {
		System.out.println("Shooter constructor init...");

		loadConfig(config); // do this here to be sure we have the values updated before we used them.

		//shootvel = new ShooterVelocity("/home/lvuser", "shootervelocity.csv");
		//shootvel.loadTable();

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
		motCANShooterMotor.set(ControlMode.Velocity, 0);
		motCANShooterMotor.configPeakOutputForward(1.0, kTimeoutMs);
		motCANShooterMotor.configPeakOutputReverse(-1.0, kTimeoutMs);
		updateShooterSettings();

		motCANTurretMotor = new TalonSRX(RobotMap.kCANId_ShooterTurretMotor);
		motCANTurretMotor.setInverted(true); // invert direction to match gearing

		motShooterHeight = new Servo(RobotMap.kPWMPort_ShooterHeight);

		anaShooterHeight = new AnalogInput(RobotMap.kAnalogPort_ShooterHeight);
		anaTurretPos = new AnalogPotentiometer(RobotMap.kAnalogPort_TurretPos);
		
		System.out.println("Shooter constructor end...");

	}

	/****public void reloadTables(){

		//shootvel.loadTable();

	}
	****/



	public void update(final Inputs inputs, final Config config) {

		//if( inputs.dRequestedVelocity != dLastRequestedVelocity){
		//	dPid_FeedForward = shootvel.getFeedForwardRange(Integer.valueOf( (int) inputs.dRequestedVelocity));
		//	this.updateShooterSettings();
		//}
		//dLastRequestedVelocity = inputs.dRequestedVelocity;

		//if(inputs.bUpdateShooterPID == true){
		//	adjustPIDError();
		//	this.updateShooterSettings();
		//}

		//if( inputs.bShooterVelocitySaveSetting ){
		//	shootvel.saveVelocity(Integer.valueOf( 
		//							(int) motCANShooterMotor.getClosedLoopTarget() ), 
		//							Double.valueOf(dPid_FeedForward) );
		//}

		this.updateShooterVelocity(inputs);

		this.solBallPusherUpper.set(inputs.bShooterLaunch);			// fire ball into shooter

		// motCANTurretMotor.set(ControlMode.PercentOutput, inputs.dTurretPower);

		if (digBallInPlace.get() == true) {		//Device is set for normally closed

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
		//turret posistion
		sTurretState = "Stopped"; 

		if(anaTurretPos.get() >= dTurretLeftStop  && inputs.dTurretPower <= 0.0 ){	// and we ara skign to go left
			inputs.dTurretPower = 0.0;		// set input to 0 to stop it.
			sTurretState = "Full Left"; 
		} else if(anaTurretPos.get() <= dTurretRigthStop && inputs.dTurretPower >= 0.0){
			inputs.dTurretPower = 0.0;		// set input to 0 to stop it. 
			sTurretState = "Full Right"; 
		} else if(inputs.dTurretPower <= 0.0){
			sTurretState = "<<<<<"; 
		} else if(inputs.dTurretPower >= 0.0){
			sTurretState = ">>>>>"; 
		} else {
			sTurretState = "Stopped"; 
		}

		motCANTurretMotor.set(ControlMode.PercentOutput, inputs.dTurretPower);

		//turret mouth 

		if (isShooterReadyToFire(inputs) == true) {

			if (inputs.bShooterLaunch == true) {

				solBallPusherUpper.set(true);

			} 

		}

	}

	public void loadConfig(final Config config) {

		dPid_Proportional = config.getDouble("shooter.PID_P", 0.2);
		dPid_Integral = config.getDouble("shooter.PID_I", 0.0008);
		dPid_Derivative = config.getDouble("shooter.PID_D", 2.0);
		dPid_FeedForward = config.getDouble("shooter.PID_F", 0.04447826);
		dVelocitySensitivity = config.getDouble("shooter.VelocitySensitivity", 5.0);

		dTurretLeftStop = config.getDouble("shooter.dTurretLeftStop", 0.9);
		dTurretRigthStop = config.getDouble("shooter.dTurretRigthStop", 0.1);
		
		iShooterHeight_Bottom = config.getInt("shooter.ShooterHeight_Bottom", 500);
		iShooterHeight_Top = config.getInt("shooter.ShooterHeight_top", 1000);

	}



	public void updateShooterSettings() {
		/* Config the Velocity closed loop gains in slot0 */
		motCANShooterMotor.config_kP(kPIDLoopIdx, dPid_Proportional, kTimeoutMs);
		motCANShooterMotor.config_kI(kPIDLoopIdx, dPid_Integral, kTimeoutMs);
		motCANShooterMotor.config_kD(kPIDLoopIdx, dPid_Derivative, kTimeoutMs);
		motCANShooterMotor.config_kF(kPIDLoopIdx, dPid_FeedForward, kTimeoutMs);
	}

	public void updateShooterVelocity(final Inputs inputs) {
		dCLError = motCANShooterMotor.getClosedLoopError();

		dRequestedPct = (.75 * inputs.dRequestedVelocity)/d75PctVelocity;

		//final double dAdjustTime = timAdjust.get();
		sCLStatus = "Stopped";
		bUpToSpeed = false;
	
		if( inputs.dRequestedVelocity  < 8000) {
			sCLStatus = "PCTMode";
			motCANShooterMotor.set(ControlMode.PercentOutput, dRequestedPct);
		} else if( Math.abs(dCLError) < dCLErrorThreshold ){
			sCLStatus = "PID Ready";
			bUpToSpeed = true;
			motCANShooterMotor.set(ControlMode.Velocity, inputs.dRequestedVelocity);
		} else {
			sCLStatus = "PID Adjust";
			motCANShooterMotor.set(ControlMode.Velocity, inputs.dRequestedVelocity);
		}

	
	}

	private boolean isShooterReadyToFire(Inputs inputs) {

		if( bBallInPlace == true && bUpToSpeed == true)
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
		telem.addColumn("Sh RTF");	
	}

    public void writeTelemetryValues(final LCTelemetry telem ){
		telem.saveDouble("Sh CL PID P", dPid_Proportional, 3 );
		telem.saveDouble("Sh CL PID I", dPid_Integral, 6 );  // need more decimals here, default = 2
		telem.saveDouble("Sh CL PID D", dPid_Derivative, 6 );
		telem.saveDouble("Sh CL PID F", dPid_FeedForward, 6 );
		telem.saveDouble("Sh CL Error", motCANShooterMotor.getClosedLoopError() );
		telem.saveDouble("Sh CL Sen Vel", motCANShooterMotor.getSelectedSensorVelocity() );
		telem.saveDouble("Sh CL Target Vel", motCANShooterMotor.getClosedLoopTarget());
		telem.saveTrueBoolean("Sh Up To Speed", this.bUpToSpeed);
		telem.saveString("Sh Turret State", sTurretState);
    }
	
	public void outputToDashboard(final boolean b_MinDisplay)  {
		
		//SmartDashboard.putNumber("Sh CL Req Vel", dRequestedVelocity );
		SmartDashboard.putNumber("Sh CL PID Fx", dPid_FeedForward);
		SmartDashboard.putNumber("Sh CL Error", dCLError );
		//SmartDashboard.putString("Sh CL Factor",String.valueOf(dCLErrorCorrectionFactor) );
		//SmartDashboard.putNumber("Sh CL Adjust", dCLAdjust );
		//SmartDashboard.putNumber("Sh CL AdjTim", timAdjust.get() );
		SmartDashboard.putNumber("Sh CL Sen Vel", motCANShooterMotor.getSelectedSensorVelocity() );
		SmartDashboard.putNumber("Sh CL Target", motCANShooterMotor.getClosedLoopTarget() );
		SmartDashboard.putNumber("Sh CL Req Pct", dRequestedPct);
		SmartDashboard.putString("Sh CL Status", sCLStatus );
		SmartDashboard.putBoolean("Sh Up To Speed", bUpToSpeed );
		SmartDashboard.putBoolean("Sh Ball In Place", bBallInPlace);
		
		if( b_MinDisplay == false){
		}
		
	}


	
}
