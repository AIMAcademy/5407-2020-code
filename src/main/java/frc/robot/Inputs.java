package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;

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


import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Inputs {

	// declare you variable up here
	private XboxController gamepadOperator = null;
	private XboxController gamepadDriver = null;
	public Joystick joyTestController = null;


	public double dDriverPower = 0.0;
	public double dDriverTurn = 0.0;
	public double dShooterPower = 0.0;
	public double dTurretPower = 0.0;
	public double dHoodPower = 0.0;
	public double dTestValue = 0.0;
	public double dRequestedVelocity = 0.0;
	
	public boolean bShooterLaunch = false;

	public boolean bIntakeIn = false;
	public boolean bIntakeOut = false;

	public boolean bTeainatorToggle = false;
	public boolean bTeainatorLastToggle = false;	 

	public boolean bUpdateShooterPID = false;
	public boolean bShiftBaseToHigh = false;

	public boolean bShooterHeightRaise = false;
	public boolean bShooterHeightLower = false;

	private boolean bUseTestController = false;
	public boolean bShooterVelocity_Raise = false;
	public boolean bShooterVelocity_Lower = false;
	public boolean bShooterVelocitySaveSetting = false;

	// class Constructor initialize your variables here
    public Inputs() {
		
		//try {
		joyTestController = new Joystick( RobotMap.kUSBPort_TestJoyStick );
		//	bUseTestController = true;
		//} 
		//catch(Exception e) {
		//	bUseTestController = false;
		//}
		//bUseTestController = false;
		
		gamepadDriver  = new XboxController(RobotMap.kUSBPort_DriverControl );
    	gamepadOperator = new XboxController(RobotMap.kUSBPort_OperatorControl );
    	zeroInputs();      				// this will init many variables
    
    }
     
    // this is the order they will be in the spread sheet. 
    public void addTelemetryHeaders(LCTelemetry telem ){
		telem.addColumn("I Driver Power");
		telem.addColumn("I Driver Turn");
		telem.addColumn("I Shooter Power");
		telem.addColumn("I Shooter Launch");
		telem.addColumn("I Sh Hood Power");
		telem.addColumn("I Turret Power");
		telem.addColumn("I Intake In");
		telem.addColumn("I Intake Out");
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
		telem.saveDouble("I Sh Hood Power", this.dHoodPower );
		telem.saveTrueBoolean("I Shooter Launch", this.bShooterLaunch );
		telem.saveDouble("I Turret Power", this.dTurretPower );
		telem.saveTrueBoolean("I Intake In", this.bIntakeIn);
		telem.saveTrueBoolean("I Intake Out", this.bIntakeOut);
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
		//double temp  = F.getY(Hand.kLeft) ;	    //  we cook this down as full is too fast
		double temp  = gamepadDriver.getY(Hand.kRight) ;	    			// we cook this down as full is too fast
		dDriverPower = temp * Math.abs(temp * temp * temp);     // quad it to desensatize the turn power 

		temp  = gamepadDriver.getX(Hand.kLeft) ;	    					// we cook this down as full is too fast
		dDriverTurn = temp * Math.abs(temp * temp * temp);      // quad it to desensatize the robot turn power 
		
		temp  = gamepadOperator.getX(Hand.kLeft) ;	    					// we cook this down as full is too fast
		dTurretPower = temp * Math.abs(temp * temp);     // quad it to desensatize the turret turn power 

		//dShooterPower = convertJoystickAxisToValueRange( gamepadDriver.getTwist(), 1.0 ) ; // force to + value only
		
		temp = convertJoystickAxisToValueRange(  joyTestController.getTwist(), 100 ) ;    // force to + value only
		if( temp < 2 ){
			dRequestedVelocity = 0.0;
		}else {
			dRequestedVelocity = 6000 + temp * 100;
		}
		
		bShooterVelocitySaveSetting = joyTestController.getRawButtonPressed(11);

	
												
		bIntakeIn = false;
		bIntakeOut = false;
									
		// give priority to the operator in these operations
		if( gamepadOperator.getBumper(Hand.kLeft) == true){
			bIntakeIn = true;
		} else if( gamepadOperator.getBumper(Hand.kRight) == true){
			bIntakeOut = true;
		} else if (gamepadDriver.getTriggerAxis(Hand.kLeft) > 0.7 ){  //Prevent accident Hits
			bIntakeIn = true;			
		} else if (gamepadDriver.getTriggerAxis(Hand.kRight) > 0.7 ){ //Prevent accident Hits
			bIntakeOut = true;									
		}
		bTeainatorToggle = false;
		if(gamepadOperator.getYButtonPressed() == true || gamepadDriver.getYButtonPressed() == true) { //If operator or driver presses Y intake will be put out
			if(bTeainatorLastToggle == false){ 						//If the last toggle was false, set it to true
				bTeainatorToggle = true; 
			}
			else {
				bTeainatorToggle = false;							//If last toggle was true, set it to false
			}
		}

		bTeainatorLastToggle = bTeainatorToggle;					//Setting LastToggle to Current Toggle


		// Please put all buttons in ID order from the drive controller are placed here
		if( gamepadOperator.getAButtonPressed() == true ){
			bUpdateShooterPID = true;  // only when it is pressed
		}else if(joyTestController.getTopPressed() == true){
			bUpdateShooterPID = true;  // only when it is pressed
		} else { 
			bUpdateShooterPID = false;  // only when it is pressed
		}

		bShiftBaseToHigh= gamepadDriver.getBumper(Hand.kLeft);

		dHoodPower = gamepadOperator.getY(Hand.kRight);
		if (dHoodPower < Math.abs(.5)) 								// dead zone
			dHoodPower = 0.0; 										// kill to ensure no accidents

//		if (bUseTestController == true) {
			bShooterVelocity_Lower = joyTestController.getRawButtonPressed(RobotMap.kButton_ShooterVelocity_Lower);
			bShooterVelocity_Raise = joyTestController.getRawButtonPressed(RobotMap.kButton_ShooterVelocity_Raise);
//		}

		if (gamepadOperator.getTriggerAxis(Hand.kRight) > 0.7)		// Prevent accidental presses
			bShooterLaunch = true;
		else
			bShooterLaunch = false;		
	
	
	
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
		SmartDashboard.putBoolean("I_TestController",bUseTestController);
		
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
