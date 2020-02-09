package frc.robot;

import edu.wpi.first.wpilibj.Joystick;

/**********************************************************************
 * Inputs Class -- This is used to collect all human input or what is considered input for the robot.
 *<p>
 *Here we will document how the joysticks accept the human input
 *
 * <pre>
 *     Source     Range     Usage
 * 
 * Driver Turn     Axis     -1 to 1
 * 
 * </pre>  
 */


//import edu.wpi.first.wpilibj.XboxController;
//import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Inputs {

	// declare you variable up here
	//XboxController mOpertorControl = null;
	//XboxController mDriverControl = null;
	Joystick mDriverControl = null;


	public double dDriverPower = 0.0;
	public double dDriverTurn = 0.0;
	public double dShooterPower = 0.0;
	public double dTurretPower = 0.0;
	public double dTestValue = 0.0;
	public double dRequestedVelocity = 0.0;

	
	public boolean bShooterLaunch = false;

	public boolean bUpdateShooterPID = false;
	public boolean bShiftBaseToHigh = false;

	public boolean bShooterHeightRaise = false;
	public boolean bShooterHeightLower = false;

	public boolean bShooterVelocity_Raise = false;
	public boolean bShooterVelocity_Lower = false;


	// class Constructor initialize your variables here
    public Inputs() {
    	
		mDriverControl = new Joystick( RobotMap.kUSBPort_DriverControl );
		//gamepadDriver  = new XboxController(RobotMap.kUSBPort_DriverControl );
    	//gamepadOperator = new XboxController(RobotMap.kUSBPort_OperatorControl );
    	zeroInputs();      				// this will init many variables
    
    }
     
    // this is the order they will be in the spread sheet. 
    public void addTelemetryHeaders(LCTelemetry telem ){
		telem.addColumn("I Driver Power");
		telem.addColumn("I Driver Turn");
		telem.addColumn("I Shooter Power");
		telem.addColumn("I Shooter Launch");
		telem.addColumn("I Turret Power");
		telem.addColumn("I Base Shift");
		telem.addColumn("I Update PID");
		telem.addColumn("I Req Vel");
		telem.addColumn("I PID F Lower");
		telem.addColumn("I PID F Raise");
		
		//telem.addColumn("I Fire By Camera");
    }

    // the order does not matter here 
    public void writeTelemetryValues(LCTelemetry telem ){
		telem.saveDouble("I Driver Power", this.dDriverPower );
		telem.saveDouble("I Driver Turn", this.dDriverTurn );
		telem.saveDouble("I Shooter Power", this.dShooterPower );
		telem.saveTrueBoolean("I Shooter Launch", this.bShooterLaunch );
		telem.saveDouble("I Turret Power", this.dTurretPower );
		telem.saveDouble("I Req Vel", this.dRequestedVelocity );
		telem.saveTrueBoolean("I Base Shift", this.bShiftBaseToHigh );
		telem.saveTrueBoolean("I Update PID", this.bUpdateShooterPID);
		telem.saveTrueBoolean("I PID F Lower", this.bShooterVelocity_Lower );
		telem.saveTrueBoolean("I PID F Raise", this.bShooterVelocity_Raise );
		//telem.saveDouble("I Driver Arch Power", this.d_DriverArchadePower );
		//telem.saveTrueBoolean("I Fire By Camera", this.b_FireByCamera);
		
    }

    
    // This will read all the inputs, cook them and save them to the appropriate variables.
    public void readValues() {   

    	// you can overload the inputs to test different ideas. 
		//double temp  = mDriverControl.getY(Hand.kLeft) ;	    //  we cook this down as full is too fast
		double temp  = mDriverControl.getY() ;	    			// we cook this down as full is too fast
		dDriverPower = temp * Math.abs(temp * temp * temp);     // quad it to desensatize the turn power 

		temp  = mDriverControl.getX() ;	    					// we cook this down as full is too fast
		dDriverTurn = temp * Math.abs(temp * temp * temp);      // quad it to desensatize the robot turn power 
		
		temp  = mDriverControl.getZ() ;	    					// we cook this down as full is too fast
		dTurretPower = temp * Math.abs(temp * temp * temp);     // quad it to desensatize the turret turn power 

		dShooterPower = convertJoystickAxisToValueRange( mDriverControl.getTwist(), 1.0 ) ; // force to + value only
		dRequestedVelocity = convertJoystickAxisToValueRange(  mDriverControl.getTwist(), 15000.0 ) ;    // force to + value only

		// Please put all buttons in ID order from the drive controller are placed here
		bUpdateShooterPID = mDriverControl.getRawButtonPressed(11);  // only when it is pressed 

		bShiftBaseToHigh= mDriverControl.getTop();

		bShooterHeightLower = mDriverControl.getRawButton(RobotMap.kButton_ShooterHeightLower);
		bShooterHeightRaise = mDriverControl.getRawButton(RobotMap.kButton_ShooterHeightRaise);

		bShooterVelocity_Lower = mDriverControl.getRawButtonPressed(RobotMap.kButton_ShooterVelocity_Lower);
		bShooterVelocity_Raise = mDriverControl.getRawButtonPressed(RobotMap.kButton_ShooterVelocity_Raise);

		bShooterLaunch = mDriverControl.getTrigger();
		
    }

    
	public int convertJoystickAxisToValueRange( double d_InputValue, int i_MaxValue )  {

		// Author Matt Hoffman LC2010 Alum.
		// use this to take an axis and convert into a range. 
		// Axis today it is 1.0 to -1.0, they have to invert outside when they pass in. 
		double d_temp = d_InputValue;						// get the current value    				   range: 1.0 to -1.0
		d_temp = d_temp + 1.0; 								// change range to positive only    		   range: 0.0 to 2.0
		d_temp = d_temp / 2.0;								// divide by 2 to get an average multiplier    range: 0.0 to 1.0  
		d_temp = (int) (d_temp * (double)i_MaxValue);	  	// multiply by highest, Example: 1000		   range:  0  to 1000 (int)
		return (int) d_temp;								// convert to int and return
		
															/* Truth table
															 *    getThrottle  change range	   convert to   mult by 
															 *                 positive +1     average /2   highest
															 *Down    -1.0        0.0             0.0            0            
															 *        -0.5        0.5             0.25         250            
															 *         0.0        1.0              .5          500            
															 *         0.5        1.5              .75         750            
															 *up       1.0        2.0             1.0         1000            
															 */
	}
    
	public double convertJoystickAxisToValueRange( double d_InputValue, double d_MaxValue )  {

		// Author Matt Hoffman LC2010 Alum.
		// use this to take an axis and convert into a range. 
		// Axis today it is 1.0 to -1.0, they have to invert outside when they pass in. 
		double d_temp = d_InputValue;						// get the current value    				   range: 1.0 to -1.0
		d_temp = d_temp + 1.0; 								// change range to positive only    		   range: 0.0 to 2.0
		d_temp = d_temp / 2.0;								// divide by 2 to get an average multiplier    range: 0.0 to 1.0  
		d_temp = (d_temp * (double)d_MaxValue);	  			// multiply by highest, Example: 1,0		   range:  0  to 1000 (int)
		return d_temp;										// convert to int and return
		
															/* Truth table
															 *    getThrottle  change range	   convert to   mult by 
															 *                 positive +1     average /2   highest
															 *Down    -1.0        0.0             0.0            0            
															 *        -0.5        0.5             0.25         250            
															 *         0.0        1.0              .5          500            
															 *         0.5        1.5              .75         750            
															 *up       1.0        2.0             1.0         1000            
															 */
	}
    
	// Show what variables we want to the SmartDashboard
	public void outputToDashboard(boolean b_MinDisplay)  {

		SmartDashboard.putNumber("I_DriverPower",this.dDriverPower);
		SmartDashboard.putNumber("I_DriverTurn",this.dDriverTurn);
		SmartDashboard.putNumber("I_ShooterPower",this.dShooterPower);
		SmartDashboard.putBoolean("I_UpdateShooterPID",bUpdateShooterPID);
		SmartDashboard.putNumber("I_TestValue",dTestValue);
		
		if ( b_MinDisplay == false ){
		}
		
	}


	
	public void loadConfig(Config config)  {

        //bp_FastOperation = config.getBoolean("b_FastOperation", true);	// ****  we do not zero this **** 
//		b_IsTestMode = Preferences.getInstance().getBoolean("I_IsTestMode", false);
		//b_CameraTestMode = Preferences.getInstance().getBoolean("I_CameraTestMode", false);
	
	}


	

    public void zeroInputs() {					// reset all variables to stop or off state
     }
    

    
}
