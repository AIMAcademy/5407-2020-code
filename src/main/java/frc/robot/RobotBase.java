package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.ControlMode;
//import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class RobotBase {

	// create your variables
	Double dLeftDrivePower = 0.0;
	Double dRightDrivePower = 0.0;

	ApplyPower applyPower = null; 
	TalonFX motLeftDriveMotorA  = null;
	TalonFX motLeftDriveMotorB  = null;
	TalonFX motRightDriveMotorA = null;
	TalonFX motRightDriveMotorB = null;

	//Compressor motCompressor = null;
	Solenoid solShifter = null;
	

    /**
     * This function is run when this class is first created used for any initialization code.
     */
    public RobotBase() {
		
		applyPower = new ApplyPower();			// get our own copy of this class
						
		motLeftDriveMotorA  = new TalonFX(RobotMap.kCANId_RightDriveMotorA);
		motLeftDriveMotorB  = new TalonFX(RobotMap.kCANId_RightDriveMotorB);
		motRightDriveMotorA = new TalonFX(RobotMap.kCANId_LeftDriveMotorA);
		motRightDriveMotorB = new TalonFX(RobotMap.kCANId_LeftDriveMotorB);

		//motCompressor = new Compressor( RobotMap.kCANId_PCM );
		//motCompressor.enabled();
		solShifter = new Solenoid(RobotMap.kCANId_PCM, RobotMap.kPCMPort_DriveShifter);


		// Make sure motors are stopped
		
		
    	// Make sure motors are stopped
		motLeftDriveMotorA.set(ControlMode.PercentOutput, 0.0);
		motLeftDriveMotorB.set(ControlMode.PercentOutput, 0.0);
		motRightDriveMotorA.set(ControlMode.PercentOutput, 0.0);
		motRightDriveMotorB.set(ControlMode.PercentOutput, 0.0);
  	
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
		dLeftDrivePower  = applyPower.getWheelPower(ApplyPower.k_iLeftRearDrive, inputs.dDriverPower, inputs.dDriverTurn);
		dRightDrivePower = applyPower.getWheelPower(ApplyPower.k_iRightRearDrive, inputs.dDriverPower, inputs.dDriverTurn);

		motLeftDriveMotorA.set(ControlMode.PercentOutput, dLeftDrivePower );
		motLeftDriveMotorB.set(ControlMode.PercentOutput, dLeftDrivePower );
		motRightDriveMotorA.set(ControlMode.PercentOutput, -dRightDrivePower ); // invert on final output, oppisite side of robot
		motRightDriveMotorB.set(ControlMode.PercentOutput, -dRightDrivePower ); // invert on final output

		solShifter.set(inputs.bShiftBaseToHigh);


    }

    public void addTelemetryHeaders(LCTelemetry telem ){
		telem.addColumn("RB Left Drive Motor A"); 
		telem.addColumn("RB Left Drive Motor B"); 
		telem.addColumn("RB Rite Drive Motor A"); 
		telem.addColumn("RB Rite Drive Motor B");
    }

    public void writeTelemetryValues(LCTelemetry telem ){
		telem.saveDouble("RB Left Drive Motor A", this.motLeftDriveMotorA.getMotorOutputPercent()); 
		telem.saveDouble("RB Left Drive Motor B", this.motLeftDriveMotorB.getMotorOutputPercent()); 
		telem.saveDouble("RB Rite Drive Motor A", this.motRightDriveMotorA.getMotorOutputPercent()); 
		telem.saveDouble("RB Rite Drive Motor B", this.motRightDriveMotorB.getMotorOutputPercent());
    }


    
    
    
	// Show what variables we want to the SmartDashboard
	public void outputToDashboard(boolean b_MinDisplay)  {

    	if( b_MinDisplay == false ){
			SmartDashboard.putNumber("O_<<<Motors", dLeftDrivePower);
	    	SmartDashboard.putNumber("O_>>>Motors", dRightDrivePower);
    	}

	}

   
    
}
