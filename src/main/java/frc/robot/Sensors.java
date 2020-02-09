package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.SPI;




public class Sensors {
	// declare your variable up here
    AHRS ahrs; 												//multi point sensor board
	AnalogInput ana_PressureTransducer;
	AnalogInput ana_IRLeftSensor;
	AnalogInput ana_IRRightSensor;
	BuiltInAccelerometer bac_BuiltInAccel;

	
	//int 	i_IRLeftDistance = 0;
	//int 	i_IRRightDistance = 0;

	boolean b_PressureIsGood 	= false;
	int 	i_PressureTransducer = 0;
	double 	d_PressureInPSI 	= 0.0;
	//int		ip_PSI0				= 478;
	//int		ip_PSI60			= 1000;
	
	
	
	double d_BACC_XAxis, d_BACC_YAxis, d_BACC_ZAxis;
	
	// Values returned by Navx-MXP
	boolean b_isMoving, b_isRotating;
	double d_navxAngle, d_navxPitch, d_navxRoll, d_navxYaw;
	double d_navxDisplacementX, d_navxDisplacementY, d_navxDisplacementZ;
	
	// Values computed by Target class
	boolean b_haveTarget;
	double d_targetX;
	double d_targetY;
	
	boolean b_IRCloseEnough = false;

//	double d_targetDistance;
	
	// class Constructor initialize you variables here
    public Sensors(int ANAConnector_IRLeftSensor,
		   		   int ANAConnector_IRRightSensor,
		   		   int ANAConnector_PressureTransducer) {

    	d_BACC_XAxis = 
    	d_BACC_YAxis = 
    	d_BACC_ZAxis = 0.0;
    	
    	b_isMoving = 
    	b_isRotating = false;
    	d_navxAngle =
    	d_navxPitch =
    	d_navxRoll  =
    	d_navxYaw = 
    	d_navxDisplacementX =
    	d_navxDisplacementY =
    	d_navxDisplacementZ = 0.0;
    	
    	b_haveTarget = false;
    	d_targetX = 0.0;
    	d_targetY = 0.0;
//    	d_targetDistance = 0.0;

    	this.ana_IRLeftSensor = 				new AnalogInput(ANAConnector_IRLeftSensor);
    	this.ana_IRRightSensor = 			new AnalogInput(ANAConnector_IRRightSensor);
    	this.ana_PressureTransducer = 	new AnalogInput(ANAConnector_PressureTransducer);
    	bac_BuiltInAccel = new BuiltInAccelerometer( );
    	
        try {
            /* Communicate w/navX MXP via the MXP SPI Bus.                                     */
            /* Alternatively:  I2C.Port.kMXP, SerialPort.Port.kMXP or SerialPort.Port.kUSB     */
            /* See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface/ for details. */
            ahrs = new AHRS(SPI.Port.kMXP); 
        } catch (RuntimeException ex ) {
            DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
        }

    }

    
    // This will read all the inputs, cook them and save them to the appropriate variables.
    public void readValues() {
    	
    	//this.d_BACC_XAxis = this.ana_IRLeftSensor.getAverageValue();
		//this.i_IRRightDistance = this.ana_IRRightSensor.getAverageValue();
		//this.i_PressureTransducer = this.ana_PressureTransducer.getAverageValue();

		//this.d_PressureInPSI = this.getPresssurePSI();

		//if (this.d_PressureInPSI > 60)
		//	this.b_PressureIsGood = true;
		//else
		//	this.b_PressureIsGood = false;

		// Internal Accelerometer
		d_BACC_XAxis = this.bac_BuiltInAccel.getX();
		d_BACC_YAxis = this.bac_BuiltInAccel.getY();
		d_BACC_ZAxis = this.bac_BuiltInAccel.getZ();

		// navxMXP gyro, accelerometer, compass
		if (ahrs.isConnected() && !ahrs.isCalibrating()) {
			b_isMoving = ahrs.isMoving();
			b_isRotating = ahrs.isRotating();
			d_navxAngle = ahrs.getAngle();
			d_navxPitch = ahrs.getPitch();
			d_navxRoll = ahrs.getRoll();
			d_navxYaw = ahrs.getYaw();
			d_navxDisplacementX = ahrs.getDisplacementX();
			d_navxDisplacementY = ahrs.getDisplacementY();
			d_navxDisplacementZ = ahrs.getDisplacementZ();
		}

		// Target coordinates derived from RoboRealm image processing
		// target.getTargetCoordinates();
		// b_haveTarget = target.b_HaveTarget;
		// d_targetX = target.getD_TargetX();
		// d_targetY = target.getD_TargetY();
		// d_targetDistance = target.getD_TargetDistance();

		// if (i_IRRightDistance > 350 - 60 && i_IRRightDistance < 350 + 60) {
		// 	b_IRCloseEnough = true;
		// } else {
		// 	b_IRCloseEnough = false;
		// }
	}

	/**
	 * GetGyroRelativeBearing Here we will cook the value of the gyro. If we are
	 * turning right we go positive.<br>
	 * Turn left and we go negative. Gyro returns 360 on down the more you turn
	 * left. We cannot use that as it is.
	 * <p>
	 * Here we convert to relative angle. Positive we are going right. Negative
	 * left.
	 * 
	 * <pre>
	 * Angle Value | Compass Value | Relative Value No Turn 0 deg | 0 | 0 Right 5
	 * degrees | 5.0 | 5.0 Left 5 degrees | 355.0 | -5.0 (355-360)
	 * 
	 * @return
	 */

	public double getGyroRelativeBearing() {

		double d_CurrYaw = this.ahrs.getAngle(); // you have to read the pitch from the gyro

		if (d_CurrYaw <= 180.0) // return positive if < 180
			return d_CurrYaw;
		else
			return (d_CurrYaw - 360.00); // return value offset from 360 if > 180

	}

	/**
	 * ZeroGyroBearing() Call this to reset the gyro to 0 degrees. You may need to
	 * experiment to get the value correct.
	 * 
	 * @return
	 */

	public void zeroGyroBearing() {

		this.ahrs.zeroYaw();

	}

	/**
	 * ZeroGyroBearing() Call this to reset the gyro to 0 degrees. You may need to
	 * experiment to get the value correct.
	 * 
	 * @return
	 */

	public double getPitch() {

		return this.ahrs.getRoll();

	}

	// public double getPresssurePSI() {
	// 	double d_CurrValue = ((double) (this.ana_PressureTransducer.getAverageValue() - this.ip_PSI0));
	// 	double d_OffsetPerLB = ((double) (this.ip_PSI60 - this.ip_PSI0)) / 60.0;
	// 	return d_CurrValue / d_OffsetPerLB;
	// }

	public void addTelemetryHeaders(LCTelemetry telem) {
		telem.addColumn("S AHRS Pitch");
		telem.addColumn("S Bearing");
		// telem.addColumn("S Pres Trans");
		// telem.addColumn("S IR Left Dist");
		// telem.addColumn("S IR Right Dist");
		telem.addColumn("S Have Target");
		telem.addColumn("S Target X");
		telem.addColumn("S Target Y");
		// telem.addColumn("S Target Dist");

		//target.addTelemetryHeaders(telem);
	}

	public void writeTelemetryValues(LCTelemetry telem) {
		telem.saveDouble("S AHRS Pitch", this.ahrs.getPitch());
		telem.saveDouble("S Bearing", this.getGyroRelativeBearing());
		// telem.saveInteger("S Pres Trans", this.i_PressureTransducer);
		// telem.saveInteger("S IR Left Dist", this.d_BACC_XAxis);
		// telem.saveInteger("S IR Right Dist", this.i_IRRightDistance);
		telem.saveBoolean("S Have Target", this.b_haveTarget);
		telem.saveDouble("S Target X", this.d_targetX);
		telem.saveDouble("S Target Y", this.d_targetY);
		// telem.saveDouble("S Target Dist", this.d_targetDistance);
		// target.writeTelemetryValues(telem);
	}

	// Show what variables we want to the SmartDashboard
	public void outputToDashboard(boolean b_MinDisplay) {

		if (b_MinDisplay == false) {
			SmartDashboard.putBoolean("S_IsMoving", b_isMoving);
			SmartDashboard.putBoolean("S_IsRotating", b_isRotating);
			SmartDashboard.putBoolean("S_HaveTarget", b_haveTarget);
			SmartDashboard.putNumber("S_Target_X", this.d_targetX);
			SmartDashboard.putNumber("S_Target_Y", this.d_targetY);
			// SmartDashboard.putNumber("S_TargetDistance", this.d_targetDistance);
		}
		SmartDashboard.putNumber("S_Roll", this.ahrs.getRoll());
		SmartDashboard.putNumber("S_Pitch", this.ahrs.getPitch());
		SmartDashboard.putNumber("S_Bearing", getGyroRelativeBearing());

		// SmartDashboard.putNumber("S_Pres_Trans", this.i_PressureTransducer);
		// SmartDashboard.putNumber("S Pres PSI", this.getPresssurePSI());
		// SmartDashboard.putNumber("S IRLeftGripper", this.d_BACC_XAxis);
		// SmartDashboard.putNumber("S IRRightGripper",this.i_IRRightDistance);
		// SmartDashboard.putBoolean("S_Pressure Is Good",this.b_PressureIsGood);
		
		
		// target.outputToDashboard(false);
	}
	
	// Load config file
	public void loadConfig(Config config) {
		// Read these from config file
		// target.loadConfig(config);
	}

}
