package frc.robot;

import java.util.HashMap;

public class ApplyPower {

    private static final int k_iArchade = 1;
    private static final int k_iMecannum = 2;
    private static final int k_iZDrive = 3;

    public static final int k_iLeftFrontDrive = 1;
    public static final int k_iRightFrontDrive = 2;
    public static final int k_iLeftRearDrive = 3;
    public static final int k_iRightRearDrive = 4;

    public static final int k_iFrontZDrive = 1;
    public static final int k_iRearZDrive = 2;
    public static final int k_iLeftZDrive = 3;
    public static final int k_iRightZDrive = 4;

    public static final double k_dCalculatedError = -10000000000.0;

    public HashMap<String,Double> mapEncoderValues = new HashMap<String,Double>();

    public Double dSavedStartLocation = 0.0;

    Double dPctOfDistance = 0.0;           // pct of how far we traveled
    double dPCTOfPower = 0.0;                       // pct power over full distance. 
    double dCalculatedPower = 0.0;      // default this in case of no changes 
    double dRampUpPoint = .20;
    double dRampDownPoint = .80;
    double dTicksToInches = 2727.272727;


    public ApplyPower(){
		
        // nothing to initialize for this class
    }

    public void saveEncoderLocation(String sEncoderName, Double dStartLocation){

        dSavedStartLocation = dStartLocation;
        //mapEncoderValues.put(sEncoderName, dStartLocation);

    }

	public void loadConfig(Config config){
		dRampUpPoint = config.getDouble("applypower.dRampUpPoint", .2);
		dRampDownPoint = config.getDouble("applypower.dRampDownPoint",.8);

	}
		
    public double getEncoderDistance(String sEncoderName, Double dCurrentLocation){

        //Double dStartLocation = mapEncoderValues.getOrDefault(sEncoderName, Double.valueOf(k_dCalculatedError));

        return (dCurrentLocation - dSavedStartLocation); 

    }

    public double RampPowerToEncoder( double dPower, String sEncoderName, Double dCurrentLocation, 
                    Double dTargetDistance, Double dMinPower){

        Double dCurrentDistance = getEncoderDistance(sEncoderName, dCurrentLocation);

        //if( dCurrentDistance == k_dCalculatedError){
        //    return k_dCalculatedError;
        //}

        dCalculatedPower = dPower;                              // default this in case of no changes 
        //System.out.println(">>>>>AP IN  dCalculatedPower: " + String.valueOf(dCalculatedPower));
        dPctOfDistance = dCurrentDistance/dTargetDistance;      // pct of how far we traveled, always positive

        //if(dPctOfDistance > .95){                               // are we in the start up part
        //    dCalculatedPower = 0.0;
        //}
        //else 
        /**
        if(dPctOfDistance < dRampUpPoint){                // are we in the start up part   
            dPCTOfPower = dPower * dPctOfDistance;              //      pct power over full distance. 
            dCalculatedPower = dPCTOfPower/dRampUpPoint;        //      from beginning to ramp up point
        }else if(Math.abs(dPctOfDistance) >= dRampDownPoint){            // are we in the end part
            dCalculatedPower = (1-dPCTOfPower)/dRampDownPoint;        //      we are now at ramp down, subtrack overall pct from full power
        }
        //System.out.println(">>>>>AP MID dCalculatedPower: " + String.valueOf(dCalculatedPower));

        if( Math.abs(dCalculatedPower) < dMinPower){
            if( dCalculatedPower > 0.0  ){ 
                dCalculatedPower = dMinPower;
            } else if( dCalculatedPower < 0.0 ){
            dCalculatedPower = -dMinPower;
            }
        }
        **/
        //System.out.println(">>>>>AP OUT dCalculatedPower: " + String.valueOf(dCalculatedPower));
        return dCalculatedPower;

    }

	public double inchesToTicks( double dInches ){
		return dInches * dTicksToInches;
	}

	public double ticksToInches( double dTicks ){
		return dTicks / dTicksToInches;
	}

    public double calcZWheelPower(int iWheel, double dPower, double dTurn, double dCrab ) {

        double dWheelPower = 0.0;

        // Calculate the the zdrive power and crab

        //        Front(1)
        //          ===
        //     ||          ||
        //Left ||          ||  Right
        // (3) ||          ||   (4)
        //          ===
        //        Rear (2)

        // Apply the drive power (front/back) and crab power (left/right)
        switch(iWheel){
            case k_iFrontZDrive:     // these make the robot go left and right so they get crab power
            case k_iRearZDrive:
                dWheelPower = dCrab; // in inputs decide with direct positive is vs. negative.
                break;

            case k_iLeftZDrive:      // these make the robot go forward and back so they get driver power
            case k_iRightZDrive:
                dWheelPower = dPower;
                break;

            default:
                // Tell user that the wheel is not valid.
                //telemetry.addData("ERROR: ApplyWheelPower: getWheelPower", "iWheel is not known.");
                //telemetry.update();
                return 0.0;
        }

        switch(iWheel){              // now add in the turn power
            case k_iFrontZDrive:
                dWheelPower += dTurn;
                break;

            case k_iRearZDrive:
                dWheelPower -= dTurn;
                break;

            case k_iLeftZDrive:      // these make the robot go forward and back so they get driver power
                dWheelPower += dTurn;
                break;

            case k_iRightZDrive:
                dWheelPower -= dTurn;
                break;

            default:
                // Tell user that the wheel is not valid.
                //telemetry.addData("ERROR: ApplyWheelPower: getWheelPower", "iWheel is not known.");
                //telemetry.update();
                return 0.0;
        }


        return dWheelPower;

    }

    private double calcWheelPower(int iDriveType, int iWheel, double dPower, double dTurn, double dCrab ) {

        double dWheelPower = 0.0;

        if (iDriveType != k_iMecannum && iDriveType != k_iArchade){
            // Tell user that the wheel is not valid.
            //telemetry.addData("ERROR: ApplyWheelPower: getWheelPower", "iType is not mecannum or arcade.");
            //telemetry.update();
            return 0.0;
        }

        // Calculate the archade power
        switch(iWheel){
            case k_iLeftFrontDrive:
                    dWheelPower = dPower + dTurn;
                    break;
            case k_iRightFrontDrive:
                    dWheelPower = dPower - dTurn;
                    break;
            case k_iLeftRearDrive:
                    dWheelPower = dPower + dTurn;
                    break;
            case k_iRightRearDrive:
                    dWheelPower = dPower - dTurn;
                    break;

            default:
                    // Tell user that the wheel is not valid.
                    //telemetry.addData("ERROR: ApplyWheelPower: getWheelPower", "iWheel is not known.");
                    //telemetry.update();
                    return 0.0;

        }

        if( iDriveType == k_iArchade)
            return dWheelPower;


        // We are Mecannum so now apply the crab power
        switch( iWheel){
            case k_iLeftFrontDrive:
                dWheelPower += dCrab;
                break;
            case k_iRightFrontDrive:
                dWheelPower -= dCrab;
                break;
            case k_iLeftRearDrive:
                dWheelPower -= dCrab;
                break;
            case k_iRightRearDrive:
                dWheelPower += dCrab;
                break;
        }

        return dWheelPower;

    }

    // This is an example fo Java over loading. Same function name different parameters.
    public double getWheelPower(int iWheel, double dPower, double dTurn ) {
        return calcWheelPower(k_iArchade, iWheel, dPower, dTurn, 0.0 );
    }

    public double getWheelPower(int iWheel, double dPower, double dTurn, double dCrab ) {
        return calcWheelPower(k_iMecannum, iWheel, dPower, dTurn, dCrab );
    }

    public void addTelemetryHeaders(LCTelemetry telem ){
        telem.addColumn("AP Ramp Up"); 
        telem.addColumn("AP Ramp Down"); 
        telem.addColumn("AP Calc Power"); 
        telem.addColumn("AP PCT Power" );
        telem.addColumn("AP PCT Dist");
    
    }

  public void writeTelemetryValues(LCTelemetry telem ){

    telem.saveDouble("AP Ramp Up", this.dRampUpPoint); 
    telem.saveDouble("AP Ramp Down", this.dRampDownPoint); 
    telem.saveDouble("AP Calc Power", this.dCalculatedPower); 
    telem.saveDouble("AP PCT Power", this.dPCTOfPower );
    telem.saveDouble("AP PCT Dist", this.dPctOfDistance);
    

 }

};