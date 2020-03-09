package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
//import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

public class RobotBase {

	// create your variables
	Double dLeftDrivePower = 0.0;
	Double dRightDrivePower = 0.0;

	Inputs inputs = null;
	Config config = null;
	ApplyPower applyPower = null;
	//Limelight limelight = null;

	TalonFX motLeftDriveMotorA  = null;
	TalonFX motLeftDriveMotorB  = null;
	TalonFX motRightDriveMotorA = null;
	TalonFX motRightDriveMotorB = null;

	Spark motIntake = null;
	double dIntakePower = 0.0;

	
	CANSparkMax motWinchLeftMotor = null;
	CANSparkMax motWinchRightMotor = null;

	Compressor mCompressor = null;
	String sCompressorCLState = "UNK";

	//Compressor motCompressor = null;
	Solenoid solShifter = null;
	Solenoid solTeainator = null;

	public boolean bLastTeainatorToggle = false;	 
	public boolean bTeainatorState = false;	 

	boolean bDev_StopCompressor = false;
	boolean bDev_StopDriveWheels = false;

	boolean bInEndGameState = false;
	
	public Gyro gyro = null;
	double dRelativeGyroBearing = 0.0;
	double dAnglePorportion = -0.015;
	boolean bIsOnGyroBearing = false;
	boolean bIsOnCloseToBearing = false;
	

	double dEncoderPosition = 0.0;
	double dEncoderDistance = 0.0;
	public static final String k_sBaseMotorEncoderKey = "BaseMotorEncoder";

    /**
     * This function is run when this class is first created used for any initialization code.
     */
    public RobotBase(Config mPassedConfig, Inputs mPassedInputs) {
		config = mPassedConfig;
		loadConfig();

		inputs = mPassedInputs;
		//limelight = mPassedLimelight;

		applyPower = new ApplyPower();			// get our own copy of this class
						
		motLeftDriveMotorA  = new TalonFX(RobotMap.kCANId_RightDriveMotorA);
		motLeftDriveMotorB  = new TalonFX(RobotMap.kCANId_RightDriveMotorB);
		motRightDriveMotorA = new TalonFX(RobotMap.kCANId_LeftDriveMotorA);
		motRightDriveMotorB = new TalonFX(RobotMap.kCANId_LeftDriveMotorB);

		// Make sure motors are in break mode
		motLeftDriveMotorA.setNeutralMode(NeutralMode.Brake);
		motLeftDriveMotorB.setNeutralMode(NeutralMode.Brake);
		motRightDriveMotorA.setNeutralMode(NeutralMode.Brake);
		motRightDriveMotorB.setNeutralMode(NeutralMode.Brake);
		
		// Make sure motors are stopped
		motLeftDriveMotorA.set(ControlMode.PercentOutput, 0.0);
		motLeftDriveMotorB.set(ControlMode.PercentOutput, 0.0);
		motRightDriveMotorA.set(ControlMode.PercentOutput, 0.0);
		motRightDriveMotorB.set(ControlMode.PercentOutput, 0.0);

		motLeftDriveMotorA.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);


		motWinchLeftMotor = new CANSparkMax(RobotMap.kCANId_WinchLeftMotor,  CANSparkMaxLowLevel.MotorType.kBrushless );
		motWinchLeftMotor.setInverted(false);
		motWinchRightMotor = new CANSparkMax(RobotMap.kCANId_WinchRightMotor, CANSparkMaxLowLevel.MotorType.kBrushless);
		motWinchRightMotor.setInverted(false);

		motWinchLeftMotor.set(0.0);
		motWinchRightMotor.set(0.0);


		mCompressor = new Compressor( RobotMap.kCANId_PCM );
		mCompressor.enabled();
		mCompressor.setClosedLoopControl(true);
		sCompressorCLState = "Normal";

		SetDevModes();

		solShifter = new Solenoid(RobotMap.kCANId_PCM, RobotMap.kPCMPort_DriveShifter);
		solTeainator = new Solenoid(RobotMap.kCANId_PCM, RobotMap.kPCMPort_Teainator);

		motIntake = new Spark(RobotMap.kPWMPort_IntakeMoter);
		motIntake.set(0.0);

		gyro = new Gyro();

	}

	public void loadConfig(){

		bDev_StopCompressor = config.getBoolean("robotbase.bDev_StopCompressor", false);
		bDev_StopDriveWheels = config.getBoolean("robotbase.bDev_StopDriveWheels", false); 

	}

	public void SetDevModes(){

		if(bDev_StopCompressor == true){
			mCompressor.stop();
			sCompressorCLState = "Dev Stopped";
		} else {
			mCompressor.start();
			sCompressorCLState = "Normal";
		}
	}





	public void GyroToAngle(Inputs inputs, Gyro gyro){
		double dRelativeGyroBearing = gyro.getGyroRelativeBearing();
		double dDiff = dRelativeGyroBearing - inputs.dRequestedBearing;

		SmartDashboard.putNumber("RB Gyro Rel Bear",  dRelativeGyroBearing);
		SmartDashboard.putNumber("RB Bearing dDiff", dDiff);
		SmartDashboard.putNumber("RB Req Bearing", inputs.dRequestedBearing);

		if( Math.abs(dDiff) < 1.0){
			bIsOnGyroBearing = true;
		} else {
			bIsOnGyroBearing = false;
		}

		
		if( bIsOnGyroBearing == false && inputs.dRequestedBearing > -1.0 || inputs.bGyroNavigate == true ){

			if( inputs.dRequestedBearing == 270.0 ){
				inputs.dRequestedBearing = -90.0;
			}

			double dMax = .35;
			double dMin = .1;

			inputs.dDriverTurn = dDiff * dAnglePorportion;	// turn porportionally
			if(inputs.dDriverTurn > 0.0){
				if(inputs.dDriverTurn > dMax ){
					inputs.dDriverTurn = dMax; 
				} else {
					inputs.dDriverTurn = dMin;
				} 
			} else if(inputs.dDriverTurn < 0.0) {
				if(inputs.dDriverTurn < -dMax){
					inputs.dDriverTurn = -dMax; 
				} else {
					inputs.dDriverTurn = -dMin;
				}
			}

		}
	}

	public void SaveEncoderPosition(){
		applyPower.saveEncoderLocation(k_sBaseMotorEncoderKey, dEncoderPosition);
	}

    /**
     * This function is run to update the output objects with data. 
     */
    public void update(Inputs inputs){
    		
    	/* Motors on one side are flipped over (inverted) so that if we apply the + power the robot goes in what you consider forward.  
    	 * In the case below we flip the Right. If it turns out that your robot is going backwards then you
    	 * would remove * -1 from right and put it on left. Just negating  as in -d_LeftFrontDrivePower does the same thing.      
    	 */

		// calculate the motor power base dupon the driver power and turn. This is from inputs
		// ApplyPower has a way to figure out what to send the motors Arcade Power  
		// The same power is applyed to the 2 left motors and anothe calculation is for the rigth motors. 
		// For arcade we only pass power and turn.


		dEncoderPosition = (double) motLeftDriveMotorA.getSelectedSensorPosition();
		dRelativeGyroBearing = gyro.getGyroRelativeBearing();

		if( inputs.bSaveEncoderPosition ==  true){
			applyPower.saveEncoderLocation(k_sBaseMotorEncoderKey, dEncoderPosition);
		}

		dEncoderDistance = applyPower.getEncoderDistance(k_sBaseMotorEncoderKey, dEncoderPosition);


		if( inputs.bTargetting == false && inputs.bShooterLaunch == false){ // override gyro turning
			GyroToAngle(inputs, gyro);
		}

		if(bDev_StopDriveWheels == false ){	// used durign dev to keep robot from killing someone

		
			//if( inputs.bRampPower == true){
			//	inputs.dDriverPower = applyPower.RampPower(inputs.dDriverPower, k_sBaseMotorEncoderKey, 
			//							dEncoderPosition, inputs.dTargetDistanceUsingEncoder, .15);
			//}
	

			dLeftDrivePower  = applyPower.getWheelPower(ApplyPower.k_iLeftRearDrive, inputs.dDriverPower, inputs.dDriverTurn);
			dRightDrivePower = applyPower.getWheelPower(ApplyPower.k_iRightRearDrive, inputs.dDriverPower, inputs.dDriverTurn);


			motLeftDriveMotorA.set(ControlMode.PercentOutput, dLeftDrivePower );
			motLeftDriveMotorB.set(ControlMode.PercentOutput, dLeftDrivePower );
			motRightDriveMotorA.set(ControlMode.PercentOutput, -dRightDrivePower ); // invert on final output, oppisite side of robot
			motRightDriveMotorB.set(ControlMode.PercentOutput, -dRightDrivePower ); // invert on final output
		}

		solShifter.set(inputs.bShiftBaseToHigh);

		motWinchLeftMotor.set(inputs.dLeftWinchPower);
		motWinchRightMotor.set(inputs.dRightWinchPower);

		// process the intake motors
		dIntakePower = 0.0;							// set to default

		// Powering Intake/Runway Motors			// apply the filtering
		if (inputs.bIntakeIn == true) {											// Forward
			dIntakePower = .5;
		} else if (inputs.bIntakeOut == true) { 									// Backwards
			dIntakePower = -.5;
		}

		motIntake.set(dIntakePower);				// assign the resulting power settings 

		//Setting Intake Soloniod to true/false
		if( inputs.bTeainatorDown == true)
			solTeainator.set(true);

		if( inputs.bTeainatorUp == true)
			solTeainator.set(false);

	}

	

	public void allowCompressorToRun(boolean bDesiredState){

		if(bDesiredState == false){
			mCompressor.stop();
			sCompressorCLState = "Manual Stop";
		} else {
			mCompressor.start();
			sCompressorCLState = "Manual Start";
		}

	}
    public void addTelemetryHeaders(LCTelemetry telem ){
		telem.addColumn("Dev Stop Compressor");
		telem.addColumn("Dev Stop Drive Wheels");

		telem.addColumn("IN Driver Power"); 
		telem.addColumn("IN Driver Turn"); 
		telem.addColumn("RB Left Drive Motor A"); 
		telem.addColumn("RB Left Drive Motor B"); 
		telem.addColumn("RB Rite Drive Motor A"); 
		telem.addColumn("RB Rite Drive Motor B");

		telem.addColumn("RB Compres Current");
		telem.addColumn("RB Compres CL State");

		telem.addColumn("IN Tea In");
		telem.addColumn("IN Tea Out");
		telem.addColumn("RB Tea Motor Power");
		telem.addColumn("RB Tea Status");

		telem.addColumn("RB Ramp Power");
		telem.addColumn("RB Enco Save Pos");
		telem.addColumn("RB Enco Pos");
		telem.addColumn("RB Enco Dist");
		telem.addColumn("RB Enco Desired Distance");

		gyro.addTelemetryHeaders(telem);
		applyPower.addTelemetryHeaders(telem);

    }

    public void writeTelemetryValues(LCTelemetry telem, Inputs inputs ){

		telem.saveDouble("IN Driver Power", inputs.dDriverPower); 
		telem.saveDouble("IN Driver Turn", inputs.dDriverTurn); 
		telem.saveDouble("RB Left Drive Motor A", this.motLeftDriveMotorA.getMotorOutputPercent()); 
		telem.saveDouble("RB Left Drive Motor B", this.motLeftDriveMotorB.getMotorOutputPercent()); 
		telem.saveDouble("RB Rite Drive Motor A", this.motRightDriveMotorA.getMotorOutputPercent()); 
		telem.saveDouble("RB Rite Drive Motor B", this.motRightDriveMotorB.getMotorOutputPercent());

		telem.saveDouble("RB Compres Current", this.mCompressor.getCompressorCurrent(),2);
		telem.saveString("RB Compres CL State", sCompressorCLState);

		telem.saveTrueBoolean("Dev Stop Compressor", bDev_StopCompressor);
		telem.saveTrueBoolean("Dev Stop Drive Wheels", bDev_StopDriveWheels);

		telem.saveTrueBoolean("IN Tea In", inputs.bIntakeIn);
		telem.saveTrueBoolean("IN Tea Out", inputs.bIntakeOut);
		telem.saveDouble("RB Tea Motor Power", this.motIntake.getSpeed());
		telem.saveTrueBoolean("RB Tea State", bTeainatorState);

		telem.saveTrueBoolean("RB Ramp Power", inputs.bRampPower);
		telem.saveTrueBoolean("RB Enco Save Pos", inputs.bSaveEncoderPosition);
		telem.saveDouble("RB Enco Desired Distance", inputs.dTargetDistanceUsingEncoder);
		telem.saveDouble("RB Enco Pos", dEncoderPosition);
		telem.saveDouble("RB Enco Dist", dEncoderDistance);

		gyro.writeTelemetryValues(telem);
		applyPower.writeTelemetryValues(telem);

	}

    
	// Show what variables we want to the SmartDashboard
	public void outputToDashboard(boolean b_MinDisplay, Inputs inputs)  {

		gyro.outputToDashboard(b_MinDisplay);

		SmartDashboard.putNumber("RB <<<Motors", dLeftDrivePower);
		SmartDashboard.putNumber("RB >>>Motors", dRightDrivePower);
		SmartDashboard.putBoolean("RB Gyro On Bearing", bIsOnGyroBearing);
		SmartDashboard.putNumber("RB Enc Posit", dEncoderPosition);

		if( b_MinDisplay == true ) return;

		SmartDashboard.putNumber("RB Intake Speed", this.motIntake.getSpeed());
		SmartDashboard.putBoolean("RB Tea State", this.solTeainator.get());
		SmartDashboard.putString("RB Comp CL State", sCompressorCLState);
		SmartDashboard.putBoolean("Dev Stop Compressor", bDev_StopCompressor);
		SmartDashboard.putBoolean("Dev Stop Drive Wheels", bDev_StopDriveWheels);

		SmartDashboard.putBoolean("RB Enco Save Pos", inputs.bSaveEncoderPosition);
		SmartDashboard.putNumber("RB Enco Desired Distance", inputs.dTargetDistanceUsingEncoder);
		SmartDashboard.putNumber("RB Enco Pos", dEncoderPosition);
		SmartDashboard.putNumber("RB Enco Dist", dEncoderDistance);
	}

   
    
}
