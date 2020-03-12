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
	Servo    svoCamera = null;
	Inputs inputs = null;

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


	Spark motPWMEPCCarousel = null;
	double dFastCarouselPower = 0.7;
	double dSlowCarouselPower = 0.2;

	Spark motPWMEPCLifter = null;


	FireSequence2 fireseq = null;
	boolean bFireSequenceIsComplete = false;

	DigitalInput digEPCInTheWay = null;
	boolean bEPCInTheWay = false;

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

	double dEPCLifterPower = 0.0;
	double dEPCLifterSpeed = 0.0;

	double dCamera_TopStop = .401;
	double dCamera_Store = .535;
	double dCamera_Bar = .535;
	double dCamera_CloseTargets = .472;
	double dCamera_FarTargets = .456;
	double dCamera_EPCView = .732;

	boolean bLastShooterLaunch = false;             // what was the launch value last cycle.
	
    /**
     * This function is run when this class is first created used for any initialization code.
     */
    public Shooter(final Config config, Inputs  mPassedInputs) {
		System.out.println("Shooter constructor init...");
		inputs = mPassedInputs;
		loadConfig(config); // do this here to be sure we have the values updated before we used them.

		fireseq = new FireSequence2(this);  // do this here to be sure it is ready when loadConfig is called. 
		fireseq.reset();


		motPWMEPCCarousel = new Spark(RobotMap.kPWMPort_EPCCarousel);
		motPWMEPCCarousel.set(0.0);

		motPWMEPCLifter = new Spark(RobotMap.kPWMPort_EPCLifter);
		motPWMEPCLifter.set(0.0);
	
		digEPCInTheWay = new DigitalInput(RobotMap.kDigitalInPort_EPCInTheWay);

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


		svoCamera = new Servo(RobotMap.kPWMPort_CameraServo);

		motShooterHood = new Servo(RobotMap.kPWMPort_ShooterHoodMotor);
		anaShooterHood = new AnalogInput(RobotMap.kAnalogPort_ShooterHood);

		System.out.println("Shooter constructor end...");

	}

	/****public void reloadTables(){

		//shootvel.loadTable();

	}
	****/



	public void update(final Inputs inputs, final Config config) {

		dEPCLifterPower = 0.0;

		if(inputs.dRequestedCameraPosition > 0.0){
			if(inputs.dRequestedCameraPosition < dCamera_TopStop ){			// camera stops
				inputs.dRequestedCameraPosition = dCamera_TopStop;
			} else if( inputs.dRequestedCameraPosition > dCamera_EPCView ){  // flipped around to look for EPC (Balls)
				inputs.dRequestedCameraPosition = dCamera_EPCView;
			}
			
			svoCamera.set(inputs.dRequestedCameraPosition);

		} else {
			if(inputs.bCloseTargets == true){
				svoCamera.set(dCamera_CloseTargets);
			} else if(inputs.bFarTargets == true) {
				svoCamera.set(dCamera_FarTargets);
			} else if(inputs.bInEndGame == true) {
				svoCamera.set(dCamera_Bar);
			} else if(inputs.bIntakeIn == true) {
				svoCamera.set(dCamera_EPCView);
			} 
		}
		
		if(inputs.bTargetting == false && bLastShooterLaunch == false ){
			inputs.dRequestedVelocity = 0.0;
			this.updateShooterVelocity(inputs);							// spinn the shootr wheel
		}

		if(inputs.bTargetting == true && inputs.bShooterLaunch == false){
			if( inputs.dRequestedVelocity == 0.0 ){
				SetShooterSpeed(inputs);
			}
			this.updateShooterVelocity(inputs);							// spinn the shootr wheel
		}

		// Shooter processing
		if( bLastShooterLaunch == false ){
				fireseq.reset();
		}

		if(inputs.bShooterLaunch == true){

			if( inputs.dRequestedVelocity == 0.0 ){
				SetShooterSpeed(inputs);
			}

			this.updateShooterVelocity(inputs);							// spinn the shootr whee

			fireseq.execute(inputs); //  this refers to the shooter itself. 
		} else {

			fireseq.reset();

		}

		motPWMEPCCarousel.set(inputs.dRequestedCarouselPower);


			// read the sensors so we all have them now
			//iShooterHoodPosition = anaShooterHood.getAverageValue();

		return;

		//HoodPosition(inputs);

		//if (digEPCInTheWay.get() == false) {	//Device is set for normally closed
		//	bEPCInTheWay = false;   			//We have a connection so it's set to false
		//}else {
		//	bEPCInTheWay = true;				//Indicates open circut, means EPC in th way
		//}
		
		//if( inputs.bIntakeIn == true)
		//	LoadABall();
			

		//motPWMMouthMotor.set(dPWMMouthMotorPower);
		//SmartDashboard.putNumber("Sh Mouth Power", dPWMMouthMotorPower);
		//bLastShooterLaunch = inputs.bShooterLaunch;

	}

	public void loadConfig(Config config) {

		dPid_Proportional = config.getDouble("shooter.dPID_P", 0.731);
		dPid_Integral = config.getDouble("shooter.dPID_I", 0.0008);
		dPid_Derivative = config.getDouble("shooter.dPID_D", 7.0);
		dPid_FeedForward = config.getDouble("shooter.dPID_F", 0.04447826);
		dVelocitySensitivity = config.getDouble("shooter.dVelocitySensitivity", 25.0);
		iPid_IntegralZone = config.getInt("shooter.iPID_IntegralZone", 100);
		dCLErrorThreshold = config.getDouble("shooter.dCLErrorThreshold", 50.0);
		
		iShooterHoodBottom = config.getInt("shooter.iShooterHoodBottom", 500);
		iShooterHoodTop = config.getInt("shooter.iShooterHoodTop", 1000);

		dCamera_TopStop 	 = config.getDouble("shooter.dCamera_TopStop", .401);
		dCamera_Store 		 = config.getDouble("shooter.dCamera_Store", .535);
		dCamera_Bar 		 = config.getDouble("shooter.dCamera_BarView", .535);
		dCamera_CloseTargets = config.getDouble("shooter.dCamera_CloseTargets", .472);
		dCamera_FarTargets 	 = config.getDouble("shooter.dCamera_FarTargets", .456);
		dCamera_EPCView 	 = config.getDouble("shooter.dCamera_EPCView", .732);
	
		dEPCLifterSpeed      = config.getDouble("shooter.dEPCLifterSpeed", .7);
		dSlowCarouselPower	 = config.getDouble("shooter.dSlowCarouselPower", .2);
		dFastCarouselPower	 = config.getDouble("shooter.dFastCarouselPower", .7);

		fireseq.loadConfig(config);

	}

	public boolean ClearTheEPCLifter(Inputs inputs){

		if( this.bEPCInTheWay == true ){		// EPC (ball) is in the way
			inputs.dRequestedCarouselPower = .15;
			return this.bEPCInTheWay;			// return true
		}

		inputs.dRequestedCarouselPower = 0.0;
		return this.bEPCInTheWay;				// return false

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
	
		if( inputs.dRequestedVelocity  < 15000) {
			bInClosedLoopMode = false;
			sCLStatus = "PCTMode";
			motCANShooterMotorRight.set(ControlMode.PercentOutput, -dRequestedPct);
			double dRightPCT = motCANShooterMotorRight.getMotorOutputPercent();
			motCANShooterMotorLeft.set(ControlMode.PercentOutput, -dRightPCT);
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

	private void SetShooterSpeed( Inputs inputs){
		if( inputs.bCloseTargets == true){
			inputs.dRequestedVelocity = 10000;
		}
		if( inputs.bFarTargets == true){
			inputs.dRequestedVelocity = 14000;
		}
	}



	private void Targetting( Inputs inputs){

		//int iRequestedTicks = 0;


		//if( Math.abs(ty) > .5){
		//	iRequestedTicks = (int) ( ty * (double) iTurretTicksPerDegree);
		//	inputs.iTurretRequestedToPosition = iTurretPosition - iRequestedTicks; 
		//}

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


    public void addTelemetryHeaders(final LCTelemetry telem ){
		telem.addColumn("Sh CL PID P");
		telem.addColumn("Sh CL PID I");
		telem.addColumn("Sh CL PID D");
		telem.addColumn("Sh CL PID F");

		telem.addColumn("Sh CL Error");
		telem.addColumn("Sh CL Sen Vel");
		telem.addColumn("Sh CL Target Vel");
		telem.addColumn("IN Shooter Launch");
		telem.addColumn("Sh Hood");
		telem.addColumn("Sh Hood Power");
		telem.addColumn("Sh On Target");
		telem.addColumn("Sh Up To Speed");
		telem.addColumn("Sh EPC In Way");

		
		fireseq.addTelemetryHeaders(telem);		// do these here as we have access to telem

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
		telem.saveTrueBoolean("Sh Up To Speed", this.bUpToSpeed);
		telem.saveTrueBoolean("Sh EPC In Way", bEPCInTheWay);

		fireseq.writeTelemetryValues(telem);			// do these here as we have access to telem
	}
	
	public void outputToDashboard(final boolean b_MinDisplay)  {
		
		SmartDashboard.putNumber("Sh CL Sen Vel", motCANShooterMotorRight.getSelectedSensorVelocity() );
		if(bInClosedLoopMode) SmartDashboard.putNumber("Sh CL Target", motCANShooterMotorRight.getClosedLoopTarget() );
		SmartDashboard.putString("Sh CL Status", sCLStatus );
		SmartDashboard.putBoolean("Sh Up To Speed", bUpToSpeed );
		SmartDashboard.putBoolean("Sh Ball In Place", bEPCInTheWay);
		SmartDashboard.putNumber("Sh Hood Calc Power", dShooterHoodPower);
		SmartDashboard.putString("Sh Hood Status", sHoodStatus);
		SmartDashboard.putNumber("Sh iDiff", iDiff);

		
		if( b_MinDisplay) return;						// minimum display to save bandwidth

		SmartDashboard.putNumber("Sh CL PID P", dPid_Proportional );
		SmartDashboard.putNumber("Sh CL PID I", dPid_Integral );  // need more decimals here, default = 2
		SmartDashboard.putNumber("Sh CL PID D", dPid_Derivative );
		SmartDashboard.putNumber("Sh CL PID F", dPid_FeedForward );
		SmartDashboard.putNumber("Sh CL Error", dCLError );
		SmartDashboard.putNumber("Sh CL Req Pct", dRequestedPct);
		SmartDashboard.putNumber("Sh Hood Pos", anaShooterHood.getAverageValue() );
		SmartDashboard.putNumber("Sh EPC Lifter", motPWMEPCLifter.get() );
		
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
	double dClearEPCTime = .15;			// time to allow ingested ball to settle in shooter
	double dLaunchResetTime = .10;			// time to allow launched ball to clear the shooter
	

	FireSequence2(Shooter mPassedShooter){
		shooter = mPassedShooter;
		timStepTimer = new Timer();
		timStepTimer.start();
	}

	void loadConfig(Config config){
		dLaunchResetTime = config.getDouble("firesequence.dLaunchResetTime", .10);
		dClearEPCTime = config.getDouble("firesequence.dClearEPCTime", .15);
	}

	void reset(){
		iStep = 0;
		iNextStep = 0;
		iLastStep = -1;
		shooter.bFireSequenceIsComplete = false;
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


		SmartDashboard.putNumber("FS2 iStep", iStep);
		SmartDashboard.putNumber("FS2 Timer", timStepTimer.get() );
		SmartDashboard.putString("FS2 State", sState );

		switch (iStep) {

			case 0:			// Initilaize any thing you need to 
				sState = "Init";
				iNextStep = iStep+1;
				break;

			case 1:										// check that there isn't an epc below the lifter
				sState = "Clear EPCLifter";
				if(shooter.bEPCInTheWay == true){
					shooter.ClearTheEPCLifter(inputs);	
				}else{
					iNextStep= iStep + 1;				// no ball, go get one 
				}

				break;

			case 2:
				sState = "Spin EPCLifter";
				//shooter.LoadABall(); // pull in the next ball
				shooter.dEPCLifterPower = shooter.dEPCLifterSpeed;   	//Start wheel to push balls up
				if(timStepTimer.get() > this.dClearEPCTime){			// we see a ball
					iNextStep = iStep + 1;							
				}

				break;

			case 3:
				sState = "Spin Carousel";
				if( inputs.bCloseTargets == true){
					inputs.dRequestedCarouselPower = shooter.dFastCarouselPower;
				} else if( inputs.bFarTargets == true ){
					inputs.dRequestedCarouselPower = shooter.dSlowCarouselPower;
				}

				if( inputs.bCloseTargets == true){
					if( timStepTimer.get() > 1.5 ){
						this.iNextStep = this.iStep +1;  
					} else {
						if( shooter.bEPCInTheWay == true) { // if we see a ball
							timStepTimer.reset();
						}			

						if( timStepTimer.get() > 3.0 ){
							this.iNextStep = this.iStep +1;  
						}
					}
				}

				break;

			case 4:
			default:
				shooter.bFireSequenceIsComplete = true;

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
