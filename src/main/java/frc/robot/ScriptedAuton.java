/**------------------------------------------------------------------------------
 * ScriptedAuton
 * This allows the user to script their autonimous moves.
 * This has to be updated for each year's code.
 * It relies on booleans in the inputs class that signal an action to be taken
 * line as in pressing a button (boolean) or requesting a position or speed (double).
 * 
 * The file is a simple text file, made up of statements. The statement can be
 * tabbed in for readability. Right now each action requires a description.
 * You can include empty lines and comments starting with # for readability. 
 *  
 * Example:
 * # basic auton description
 * auton 1
 *    step 1
 *        bearing, -90  ,  set the robot gyro bearing to -90 degrees (left).
 *        spin   ,    0  ,  spin up the shooter wheel to get ready to shoot. 
 *        timer  ,  3.5 ,  execte this step for 3.5 seconds
 *        power  ,  .75 ,  run at .75 power
 * 
 *    step 2
 *        bearing, 0     ,  turn back to target 0.0 degrees
 *        spin   ,    0  ,  spin up the shooter wheel to get ready to shoot. 
 *        timer  ,  1.0. ,  execute this step for 3.5 seconds
 *        power  ,  .75 ,  run at .75 power
 * 
 *    step 3
 *        shoot, 0     ,  start shooting process
 *
 * # next auton
 * auton 2
 *    step 1
 * 
 ***************************************************************************     
 * Key words
 * auton #  - defines the start of a new autonomous program.
 * step #   - The step in that auton program. 
 * 
 * The actions, defined in AutonStep class, for the 2020 season are below.
 * bearing   -- set the gyro bearing for the robot
 * distance  -- drive a certain distance as defined in code using encoder
 * high      -- shift drive to high gear
 * ingest    -- start / stop ingesting 0 = stop, non 0 = start
 * power     -- drive forward (+), backward (-) using gyro keep straight.
 * timer     -- run the step for a certain amount of time 
 * shoot     -- press and hold the shoot trigger on driver's control
 * spin      -- pre-spin up the shooter so it is moving when you get there to shoot.   
/----------------------------------------------------------------------------**/

package frc.robot;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.Map;
import java.util.TreeMap;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ScriptedAuton{

  public String sFileName = null;

  private Integer iAutonNumber = 0;
  private Integer iStepNumber = 0;
  private Integer iNextStepNumber = 0;
  private Integer iLastStepNumber = 0;
  private Integer iActionNumber = 0;               //This is an Action within a step
  boolean bStepIsSetup = false;
  boolean bStepIsComplete = false;

  private String sAction = "";
  private String sValue = "";
  private String sDesc = "";
  
  
  Timer timStepTimer = null;
  boolean bAutonComplete = false;
  
  public TreeMap<String,AutonStep> mapValues = new TreeMap<String,AutonStep>();

  public  ScriptedAuton(String sPassedFilePath, String sPassedFileName) {
  	sFileName = sPassedFilePath + File.separator + sPassedFileName;
    loadScript(sFileName);
    timStepTimer = new Timer();
    timStepTimer.start();
 }

  void loadScript(String sFileName){

    BufferedReader bufr = null;
	  
    try {
      FileReader fr = new FileReader(sFileName);
      bufr = new BufferedReader(fr);
            
      while(true){
        String sLine = bufr.readLine();
        if( sLine == null){               // nothing more from the file
          break;                          // get out of while loop
        }

        sLine = sLine.strip();

        if( sLine.startsWith("#") == true){
          continue;
        } else if(sLine.length() == 0){
          continue;
        }

        String[] sParms = sLine.split(",");	   								// break up line into Key and Value Strings
        String sParam = sParms[0].strip().toLowerCase();
        String[] sWords = sParam.split(" ");

		if (sParam.startsWith("auton") == true){							// example auton 3, descrition

          iAutonNumber = Integer.valueOf(sWords[1]);
          iStepNumber = 0;
          iActionNumber = 0;
          iLastStepNumber = 0;

        } else if (sParam.startsWith("step") == true){						// example step 2, bla bla

          iStepNumber = Integer.valueOf(sWords[1].strip());

        } else {                // everything below here is an action
          iActionNumber += 1;
          saveAction(sParms[0], sParms[1], sParms[2]);
        }
      }
    } catch(IOException e) {
      e.printStackTrace();
    }

    try{
      bufr.close();
    } catch(IOException e) {
      e.printStackTrace();
    }

    reset();                  // all done reset
  }

  void saveAction(String sAction, String sValue, String sDesc){

    if( iStepNumber != iLastStepNumber){
      iActionNumber = 1;
    }
  
    iLastStepNumber = iStepNumber;


    AutonStep mAutonStep = new AutonStep(
      iAutonNumber,   		// Auton Number
      iStepNumber,   		// Auton Step
      iActionNumber,   		// Action Step
      sAction.toLowerCase(),// Action
      sValue, 				// Param
      sDesc);				// description

    String sKey = String.format("%02d|%02d|%02d", iAutonNumber, iStepNumber,iActionNumber); 

    mapValues.put(sKey, mAutonStep);								// save to map as Distance, TargetData class

  }

  public void dumpMap(){

  // using for-each loop for iteration over TreeMap.entrySet() 
    for (Map.Entry<String, AutonStep> entry : mapValues.entrySet()){
        String sKey = entry.getKey();         // return the key 
        AutonStep autonStep = entry.getValue();   // get the associated AutonStep class value
        System.out.println( sKey + " : " + autonStep.getCSVLine() ); 
    } 

  }

  public void reset(){

    iStepNumber = 1;
    iActionNumber = 0;
    iLastStepNumber = 0;
    iNextStepNumber = 1;

  }
  
  public void execute(Integer iRunningAutonId, Inputs inputs, RobotBase robotbase){

    SmartDashboard.putNumber("SA Auton Id", iRunningAutonId);

    if( this.iStepNumber == 0 ){
      reset();
    }

    if( iStepNumber != iNextStepNumber){
      iStepNumber = iNextStepNumber;
    }
    
    if( this.iStepNumber != this.iLastStepNumber){				// set conditions for a new step to process.
      bStepIsSetup = false;
      bStepIsComplete = false;
      timStepTimer.reset();
      //robotbase.encoderReset();
      //
    }
    
    this.iLastStepNumber = this.iStepNumber;

    if( bStepIsSetup == true ){               // force us to go through at least 1 pass
      testCompletion(iRunningAutonId, inputs, robotbase);

      if(bAutonComplete == true){
        stopAll(inputs, robotbase);
        return;
      }
        
    }
	
  	setAutonActions(iRunningAutonId, inputs, robotbase);

		if(this.bStepIsComplete == true){
			this.iNextStepNumber = this.iStepNumber += 1;
    }
	
  	this.bStepIsSetup = true;                   // force at least 1 pass

  }  

  public void stopAll( Inputs inputs, RobotBase robotbase ){
    inputs.dDriverPower = 0.0;
    inputs.dDriverTurn = 0.0;
    inputs.dRequestedBearing = 0.0;
    inputs.bShooterLaunch = false;
    inputs.bTeainatorUp = true;
    inputs.bIntakeIn = false;
    inputs.bIntakeOut = false;
    inputs.bSpinUpShooter = false;
  }

  private void setAutonActions(Integer iRunningAutonId, Inputs inputs, RobotBase robotbase ){

    String sStartKey = String.format( "%02d|%02d|01", iRunningAutonId, this.iStepNumber);

    // here we iterate through only the actions for this step.
    for (Map.Entry<String, AutonStep> entry : mapValues.tailMap(sStartKey).entrySet()) {
          String sKey = entry.getKey();         // return the key 
          AutonStep autonStep = entry.getValue();   // get the associated AutonStep class value
      
      // we know we are done here when autonStep.iAutonNumber or 
      // the autonStep.iAutonStep on the action we just read from the map 
      // is not the current one being processed 
      if( autonStep.iAutonNumber != iRunningAutonId ||              // not the current Auton 
                        autonStep.iAutonStep != this.iStepNumber){  // not the current step
          break;
        }

        if( autonStep.sAction.equals("distance")){
                                                        // do nothing here, will be used to test we are done. 
        } else if(autonStep.sAction.equals("fire")) {   // start the shooter start machine
          inputs.bShooterLaunch = true;                     //    hold the trigger 
        } else if( autonStep.sAction.equals("bearing")){// change the gyro bearing
          inputs.dRequestedBearing = autonStep.dValue;      //    set a Requested bearing for the gyro.
          inputs.bGyroNavigate = true;                      //    tell gyro to go there
        } else if( autonStep.sAction.equals("high")){   // shift to high on transmission
          inputs.bShiftBaseToHigh = true;                   //    hold the button
        } else if( autonStep.sAction.equals("ingest")){	// test conditions for ingest
          if( autonStep.dValue == 0.0) {                    //    0.0 indicates to turn it off. 
            inputs.bTeainatorUp = true;                     //        hold the button to lift it up
          } else {                                          //    not 0.0 drop it and turn on ingest.
            inputs.bTeainatorDown = true;                   //        hold the button put it down
            inputs.bIntakeIn = true;                        //        hold the button to run it to intake
          }

        } else if( autonStep.sAction.equals("power")){      // update the variable for driver power
          inputs.dDriverPower = autonStep.dValue;           //    set the driver power
        } else if( autonStep.sAction.equals("spin")){	      // if this is not in the step injest remains unchanged
          inputs.bSpinUpShooter = true;                 //    spin the shooter to a predetermined value
        } else if( autonStep.sAction.equals("settle")){	      // if this is not in the step injest remains unchanged
          inputs.dDriverPower = 0.0;              
          inputs.dDriverTurn = 0.0;               
        } else if( autonStep.sAction.equals("timer")){
                                                        // do nothing, will be used later to test we are done. 
        } else if( autonStep.sAction.equals("turnto")){
          inputs.dRequestedBearing = autonStep.dValue;      //    set a Requested bearing for the gyro.
          inputs.bGyroNavigate = true;                      //    tell gyro to go there
        }
    }
  }

  private void testCompletion(Integer iRunningAutonId, Inputs inputs, RobotBase robotbase ){

    String sStartKey = String.format( "%02d|%02d|01", iRunningAutonId, this.iStepNumber);

    for (Map.Entry<String, AutonStep> entry : mapValues.tailMap(sStartKey).entrySet()) {
          String sKey = entry.getKey();         // return the key 
          AutonStep autonStep = entry.getValue();   // get the associated AutonStep class value
      
      if( autonStep.iAutonNumber != iRunningAutonId || autonStep.iAutonStep != this.iStepNumber){
        break;
      }
      
      if( autonStep.sAction.equals("distance")){
          this.bStepIsComplete = true;                  // TODO: Need to put the real test in here
      } else if(autonStep.sAction.equals("fire")) {
          this.bStepIsComplete = true;                  // TODO: Need to put the real test in here
        } else if( autonStep.sAction.equals("settle")){
          if(timStepTimer.get() > autonStep.dValue){      // is the step timer up 
            this.bStepIsComplete = true;
          }
      } else if(autonStep.sAction.equals("stop")) {
        if(timStepTimer.get() > autonStep.dValue ){
          this.bStepIsComplete = true;             
        }
      } else if( autonStep.sAction.equals("turnto")){
          if(robotbase.bIsOnGyroBearing == false){
            timStepTimer.reset();
          } else if( robotbase.bIsOnGyroBearing == true && timStepTimer.get() >.20){ 
            this.bStepIsComplete = true;  // are we on the right bearing
            System.out.println("Gyro On bearing: bStepIsComplete = true"); 
          }
      } else if( autonStep.sAction.equals("timer")){
          if(timStepTimer.get() > autonStep.dValue){      // is the step timer up 
            this.bStepIsComplete = true;
          }
      }
	  
    }

    //this.bAutonComplete = true;

	  
  }

  public void outputToDashboard(boolean b_MinDisplay)  {
		SmartDashboard.putNumber("SA I Step Number",this.iStepNumber);
		SmartDashboard.putNumber("SA I Next Step",this.iNextStepNumber);
    SmartDashboard.putBoolean("SA I Auton Comp",this.bAutonComplete);
    SmartDashboard.putNumber("SA I Step Timer",this.timStepTimer.get() );

    if ( b_MinDisplay == false ) return;
	
		
	}



}

/**
public AutonStep getAutonStep(Double dDistance) {
	
	Double dKey = mapValues.floorKey(dDistance);

	return mapValues.get(dKey);

	//return mAutonStep;
    //System.out.println("Key = " + String.valueOf(iVelocity) + " | " + 
    //                         "floor: " + String.valueOf(floorKey) + " , " + mapValues.get(floorKey) +
	//						 "  next: " + String.valueOf(ceilingKey)  + " , " + mapValues.get(ceilingKey) );
	  
  }
**/



