package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.ControlMode;
//import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class RobotBase {

	// create your variables
	Double dLeftDrivePower = 0.0;
	Double dRightDrivePower = 0.0;

	Config config = null;
	ApplyPower applyPower = null; 
	TalonFX motLeftDriveMotorA  = null;
	TalonFX motLeftDriveMotorB  = null;
	TalonFX motRightDriveMotorA = null;
	TalonFX motRightDriveMotorB = null;
	Spark motIntake = null;

	Compressor mCompressor = null;
	String sCompressorCLState = "UNK";

	//Compressor motCompressor = null;
	Solenoid solShifter = null;
	Solenoid solTeainator = null;

	public boolean bLastTeainatorToggle = false;	 
	public boolean bTeainatorState = false;	 

	boolean bDev_StopCompressor = false;
	boolean bDev_StopDriveWheels = false;
  

    /**
     * This function is run when this class is first created used for any initialization code.
     */
    public RobotBase(Config mPassedConfig) {
		config = mPassedConfig;
		loadConfig();

		applyPower = new ApplyPower();			// get our own copy of this class
						
		motLeftDriveMotorA  = new TalonFX(RobotMap.kCANId_RightDriveMotorA);
		motLeftDriveMotorB  = new TalonFX(RobotMap.kCANId_RightDriveMotorB);
		motRightDriveMotorA = new TalonFX(RobotMap.kCANId_LeftDriveMotorA);
		motRightDriveMotorB = new TalonFX(RobotMap.kCANId_LeftDriveMotorB);

		mCompressor = new Compressor( RobotMap.kCANId_PCM );
		mCompressor.enabled();
		mCompressor.setClosedLoopControl(true);
		sCompressorCLState = "Normal";

		if(bDev_StopCompressor == true){
			mCompressor.stop();
			sCompressorCLState = "Dev Stopped";
		}

		solShifter = new Solenoid(RobotMap.kCANId_PCM, RobotMap.kPCMPort_DriveShifter);
		solTeainator = new Solenoid(RobotMap.kCANId_PCM, RobotMap.kPCMPort_Teainator);


		motIntake = new Spark(RobotMap.kPWMPort_IntakeMoter);

		// Make sure motors are stopped
		motLeftDriveMotorA.set(ControlMode.PercentOutput, 0.0);
		motLeftDriveMotorB.set(ControlMode.PercentOutput, 0.0);
		motRightDriveMotorA.set(ControlMode.PercentOutput, 0.0);
		motRightDriveMotorB.set(ControlMode.PercentOutput, 0.0);
		motIntake.set(0.0);

	}

	public void loadConfig(){

		bDev_StopCompressor = config.getBoolean("robotbase.bDev_StopCompressor", false);
		bDev_StopDriveWheels = config.getBoolean("robotbase.bDev_StopDriveWheels", false); 

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

		if(bDev_StopDriveWheels == false ){	// used durign dev to keep robot from killing someone
			dLeftDrivePower  = applyPower.getWheelPower(ApplyPower.k_iLeftRearDrive, inputs.dDriverPower, inputs.dDriverTurn);
			dRightDrivePower = applyPower.getWheelPower(ApplyPower.k_iRightRearDrive, inputs.dDriverPower, inputs.dDriverTurn);

			motLeftDriveMotorA.set(ControlMode.PercentOutput, dLeftDrivePower );
			motLeftDriveMotorB.set(ControlMode.PercentOutput, dLeftDrivePower );
			motRightDriveMotorA.set(ControlMode.PercentOutput, -dRightDrivePower ); // invert on final output, oppisite side of robot
			motRightDriveMotorB.set(ControlMode.PercentOutput, -dRightDrivePower ); // invert on final output
		}

		solShifter.set(inputs.bShiftBaseToHigh);

		// Powering Intake Motors
		if (inputs.bIntakeIn == true) {											// Forward
			motIntake.set(.5);
		}
		else if (inputs.bIntakeOut == true) { 									// Backwards
			motIntake.set(-.5);			
		}
		else {
			motIntake.set(0.0);
		}


		/**
		if(inputs.bMouthIn == true) {
			motMouth.set(1); 
		}
		else if(inputs.bMouthOut == true)
		{
			motMouth.set(-1); 
		}
		else 
		{
			motMouth.set(0); 
		}
		**/
		//Setting Intake Soloniod to true/false

		bTeainatorState = solTeainator.get();							// capture here so we can see it in telemetry 
		if( inputs.bTeainatorToggle == true && bLastTeainatorToggle == false) {

			if( bTeainatorState == true){
				bTeainatorState = false;
			} else if(bTeainatorState == false){
				bTeainatorState = true;
			}
		} 
		solTeainator.set(bTeainatorState);
		bLastTeainatorToggle = inputs.bTeainatorToggle;	 

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
		telem.addColumn("RB Left Drive Motor A"); 
		telem.addColumn("RB Left Drive Motor B"); 
		telem.addColumn("RB Rite Drive Motor A"); 
		telem.addColumn("RB Rite Drive Motor B");
		telem.addColumn("RB Intake Motor");
		telem.addColumn("RB Tea Status");
		telem.addColumn("RB Comp CL State");

    }

    public void writeTelemetryValues(LCTelemetry telem ){
		telem.saveDouble("RB Left Drive Motor A", this.motLeftDriveMotorA.getMotorOutputPercent()); 
		telem.saveDouble("RB Left Drive Motor B", this.motLeftDriveMotorB.getMotorOutputPercent()); 
		telem.saveDouble("RB Rite Drive Motor A", this.motRightDriveMotorA.getMotorOutputPercent()); 
		telem.saveDouble("RB Rite Drive Motor B", this.motRightDriveMotorB.getMotorOutputPercent());
		telem.saveDouble("RB Intake Motor", this.motIntake.getSpeed());
		telem.saveTrueBoolean("RB Tea State", bTeainatorState);
		telem.saveString("RB Comp CL State", sCompressorCLState);
    }

    
	// Show what variables we want to the SmartDashboard
	public void outputToDashboard(boolean b_MinDisplay)  {

		if( b_MinDisplay == true ){
			SmartDashboard.putNumber("O_<<<Motors", dLeftDrivePower);
			SmartDashboard.putNumber("O_>>>Motors", dRightDrivePower);
			return;
    	}

		SmartDashboard.putNumber("RB Intake Speed", this.motIntake.getSpeed());
		SmartDashboard.putBoolean("RB Tea State", this.solTeainator.get());
		SmartDashboard.putString("RB Comp CL State", sCompressorCLState);
		SmartDashboard.putBoolean("Dev Stop Compressor", bDev_StopCompressor);
		SmartDashboard.putBoolean("Dev Stop Drive Wheels", bDev_StopDriveWheels);
	}

   
    
}
