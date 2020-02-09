package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.AnalogInput;
//import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Solenoid;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
//import com.ctre.phoenix.sensors.CANCoder;


public class Shooter {


	// create your variables

	double dPid_Proportional = 0.2;
	double dPid_Integral = 0.000;
	double dPid_Derivative = 0.0;
	double dPid_FeedForward = 0.0446;  // estimate of how many ticks we will see at any velocity in 100 ms.
	double dVelocitySensitivity = 5.0;
	double dRequestedVelocity = 0.0;
	double dVelocityAdjust = 0.0;
	double dCLAdjust = 0.0;
	double dCLError = 0.0;
	boolean bCLAdjustedError = false;
	double dCLErrorThreshold = 10;
	String sCLStatus = "****";
	boolean bReadyToShoot = false;
	
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
	Servo    motShooterHeight = null;
	Timer    timAdjust		  = null;
	AnalogInput anaTurretPos  = null;
	AnalogInput anaShooterHeight = null;
	Solenoid    solBallPusherUpper = null;

	public static final double kShooterHeight_Up = 1.0; 
	public static final double kShooterHeight_Down = 0.0;
	public static final double kShooterHeight_Stop = .5; 

	double dShooterHeightPower= kShooterHeight_Stop;


	int iTurretPos =0;
	int iTurretPos_FullRight =0;
	int iTurretPos_FullLeft=0;

	int iShooterHeight = 0;
	int iShooterHeight_Top =0;
	int iShooterHeight_Bottom = 0;
	
    /**
     * This function is run when this class is first created used for any initialization code.
     */
    public Shooter(final Config config) {

		System.out.println("Shooter constructor init...");
		timAdjust = new Timer();
		timAdjust.stop();

		solBallPusherUpper = new Solenoid(RobotMap.kPCMPort_BallPusherUpper);
		solBallPusherUpper.set(false);

		motCANShooterMotor = new TalonFX(RobotMap.kCANId_ShooterMotor);

		motCANTurretMotor = new TalonSRX(RobotMap.kCANId_ShooterTurretMotor);
		motCANTurretMotor.setInverted(true); // invert direction to match gearing


		loadConfig(config); // do this here to be sure we have the values updated before we used them.

		motCANShooterMotor.configFactoryDefault();

		motShooterHeight = new Servo(RobotMap.kPWMPort_ShooterHeight);

		/* Config sensor used for Primary PID [Velocity] */
		motCANShooterMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, kPIDLoopIdx, kTimeoutMs);

		/**
		 * Phase sensor accordingly. Positive Sensor Reading should match Green
		 * (blinking) Leds on Talon
		 */
		motCANShooterMotor.setSensorPhase(true);

		// ramp up time so we do not cause a huge surge in current 
		motCANShooterMotor.configClosedloopRamp(3);

		/* Config the peak and nominal outputs */
		// motCANShooterMotor.setInverted(true);

		motCANShooterMotor.configNominalOutputForward(0, kTimeoutMs);
		motCANShooterMotor.configNominalOutputReverse(0, kTimeoutMs);

		motCANShooterMotor.configPeakOutputForward(1.0, kTimeoutMs);
		motCANShooterMotor.configPeakOutputReverse(-1.0, kTimeoutMs);

		updateShooterSettings();

		motCANShooterMotor.set(ControlMode.Velocity, 0);
		// motCANTurretMotor.set(ControlMode.Velocity, 0);

		anaShooterHeight = new AnalogInput(RobotMap.kAnalogPort_ShooterHeight);
		anaTurretPos = new AnalogInput(RobotMap.kAnalogPort_TurretPos);

		
		System.out.println("Shooter constructor end...");

	}

	public void update(final Inputs inputs, final Config config) {

		// Control Shooter Height
		if (inputs.bUpdateShooterPID == true) {

			// config.load(); // force a reload of the config
			// loadConfig(config); // reload from the nex config file

			this.updateShooterSettings();
		}

		if( inputs.bShooterVelocity_Lower == true){
			dVelocityAdjust -= 1.0;
		}

		if( inputs.bShooterVelocity_Raise == true){
			dVelocityAdjust += 1.0;
		}

		if(inputs.mDriverControl.getRawButtonPressed(3) == true){
			adjustPIDError();
			this.updateShooterSettings();
		}

		this.updateShooterVelocity(inputs);

		this.solBallPusherUpper.set(inputs.bShooterLaunch);			// fire ball into shooter

		// motCANTurretMotor.set(ControlMode.PercentOutput, inputs.dTurretPower);

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

	}

	public void loadConfig(final Config config) {

		dPid_Proportional = config.getDouble("shooter.PID_P", .20);
		dPid_Integral = config.getDouble("shooter.PID_I", 0.000);
		dPid_Derivative = config.getDouble("shooter.PID_D", 0.0);
		dPid_FeedForward = config.getDouble("shooter.PID_F", 0.0446);
		dVelocitySensitivity = config.getDouble("shooter.VelocitySensitivity", 5.0);

		iTurretPos_FullLeft = config.getInt("shooter.TurretPos_FullLeft", 500);
		iTurretPos_FullRight = config.getInt("shooter.TurretPos_FullRight", 1000);
		iShooterHeight_Bottom = config.getInt("shooter.ShooterHeight_Bottom", 500);
		iShooterHeight_Top = config.getInt("shooter.ShooterHeight_top", 1000);

	}


	public void adjustPIDError() {

		final double dRequestedVelocity = motCANShooterMotor.getClosedLoopTarget();

		if (dRequestedVelocity > 7900)
			dCLAdjust *= .25;
		else if (dRequestedVelocity > 6900)
			dCLAdjust *= .5;

		if (dCLError < 0.0)
			dPid_FeedForward -= dCLAdjust;
		else
			dPid_FeedForward += dCLAdjust;

	}

	public void updateShooterSettings() {
		// this.motCANShooterMotor.reverseOutput(true);
		// this.motCANShooterMotor.setPID(0.2, 0, 0);
		/* Config the Velocity closed loop gains in slot0 */
		motCANShooterMotor.config_kP(kPIDLoopIdx, dPid_Proportional, kTimeoutMs);
		motCANShooterMotor.config_kI(kPIDLoopIdx, dPid_Integral, kTimeoutMs);
		motCANShooterMotor.config_kD(kPIDLoopIdx, dPid_Derivative, kTimeoutMs);
		motCANShooterMotor.config_kF(kPIDLoopIdx, dPid_FeedForward, kTimeoutMs);

		// motCANShooterMotor.enable();
	}

	public void updateShooterVelocity(final Inputs inputs) {
		// motCANShooterMotor.set(ControlMode.PercentOutput, inputs.dShooterPower);
		dCLError = motCANShooterMotor.getClosedLoopError();
		dCLAdjust = Math.abs(dCLError * .00001);

		final double dAdjustTime = timAdjust.get();
		sCLStatus = "Stopped";
		bReadyToShoot = false;
	
		dRequestedVelocity = inputs.dRequestedVelocity + dVelocityAdjust; 

		if( inputs.bShooterLaunch == false){
			if( dRequestedVelocity  < 5000) {
				sCLStatus = "Slow";

				if(dAdjustTime > 0.0){
					timAdjust.stop();
					timAdjust.reset();
				}
			} else if( Math.abs(dCLError) < dCLErrorThreshold ){
				sCLStatus = "Ready";
				timAdjust.reset();
				timAdjust.start();
				bReadyToShoot = true;
			} else {
				sCLStatus = "Run";

				if(dAdjustTime <= 0.0 ){
					timAdjust.reset();
					timAdjust.start();
				}
				else if( dAdjustTime > .5 ){
					adjustPIDError();
					updateShooterSettings();
					timAdjust.reset();
					timAdjust.start();
					sCLStatus = "Adjust";
				}
			}
		}

		motCANShooterMotor.set(ControlMode.Velocity, dRequestedVelocity);

	}

	private boolean isShooterReadyToFire(final Inputs inputs) {

		final boolean returnValue = false;
    	
    	/*** test to see if normal forward and desired normal forward are the same.
    	if( inputs.b_DriveNormalForward == this.b_DesiredForwardNormal)
    		if(inputs.i_BalistaPowerLevel == this.i_DesiredShooterLevel){

    			final int sensitivity = 20;
		    	final int high_point = this.i_DesiredShooterPosition + sensitivity; 
		    	final int low_point = this.i_DesiredShooterPosition - sensitivity; 
		    	
		    	final int cur_Position = this.motCANShooterMotor.getAnalogInPosition();
		    	
		    	if( cur_Position > low_point && cur_Position < high_point  )
		    		returnValue = true;
    		}
    	
		this.b_InPosition = returnValue;
		***/

    	return returnValue;
    }


    public void addTelemetryHeaders(final LCTelemetry telem ){
		telem.addColumn("Sh CL PID P");
		telem.addColumn("Sh CL PID I");
		telem.addColumn("Sh CL PID D");
		telem.addColumn("Sh CL PID F");
		telem.addColumn("Sh CL Req Vel");
		telem.addColumn("Sh CL Error");
		telem.addColumn("Sh CL Sen Vel");
		telem.addColumn("Sh CL Target");
		telem.addColumn("Sh Ready");	
	}

    public void writeTelemetryValues(final LCTelemetry telem ){
		telem.saveDouble("Sh CL PID P", dPid_Proportional );
		telem.saveDouble("Sh CL PID I", dPid_Integral, 3 );  // need more decimals here, default = 2
		telem.saveDouble("Sh CL PID D", dPid_Derivative );
		telem.saveDouble("Sh CL PID F", dPid_FeedForward, 6 );
		telem.saveDouble("Sh CL Error", motCANShooterMotor.getClosedLoopError() );
		telem.saveDouble("Sh CL Sen Vel", motCANShooterMotor.getSelectedSensorVelocity() );
		telem.saveDouble("Sh CL Target", motCANShooterMotor.getClosedLoopTarget());
		telem.saveTrueBoolean("Sh Ready", this.bReadyToShoot);
    }
	
	public void outputToDashboard(final boolean b_MinDisplay)  {
		
		SmartDashboard.putNumber("Sh CL Req Vel", dRequestedVelocity );
		SmartDashboard.putNumber("Sh CL PID F", dPid_FeedForward);
		SmartDashboard.putNumber("Sh CL Error", dCLError );
		SmartDashboard.putNumber("Sh CL Adjust", dCLAdjust );
		SmartDashboard.putNumber("Sh CL AdjTim", timAdjust.get() );
		SmartDashboard.putNumber("Sh CL Sen Vel", motCANShooterMotor.getSelectedSensorVelocity() );
		SmartDashboard.putNumber("Sh CL Target", motCANShooterMotor.getClosedLoopTarget() );
		SmartDashboard.putString("Sh CL Status", sCLStatus );
		SmartDashboard.putBoolean("Sh Nominal", bReadyToShoot );
		
		if( b_MinDisplay == false){
		}
		
	}


	
}
