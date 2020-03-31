package frc.robot;

public class AutonStep{ 

    public Integer iAutonNumber = 0;
    public Integer iAutonStep = 0;
    public Integer iActionNumber = 0;
    public String sDescription = "";
    public String sAction = "";
    public Double dValue = 0.0;
    
    public String sCSVLine = "";

  public AutonStep(Integer iAutonNumber, Integer iAutonStep, Integer iActionNumber,
                   String sAction, String sValue, String sDescription  ){

    this.iAutonNumber = iAutonNumber;
    this.iAutonStep = iAutonStep;
    this.iActionNumber = iActionNumber;    
    this.sAction = sAction.strip().toLowerCase();
    this.dValue = Double.valueOf(sValue.strip());
    this.sDescription = sDescription.strip();

	if( isValidAction(this.sAction) == false ){
		this.sDescription = "******ERROR!!!! Invalid action [" + this.sAction +"]!!!!!!";
		System.out.println("******************************************************************"); 
		System.out.println("ERROR!!!!! Action: " + this.sAction + " is not a valid action!!!!!"); 
		System.out.println("******************************************************************"); 
	}


	/**
    sCSVLine =  String.valueOf(iAutonNumber)  + "," +
        String.valueOf(iStep)  + "," +
        String.valueOf(iActionNumber)  + "," +
        String.valueOf(sAction)  + "," +
        String.valueOf(dValue) + "," +
        String.valueOf(sDescription);
	**/
	
    sCSVLine =  String.format( "%3d, %3d, %3d, %-12s, %12.2f, %s", 
						this.iAutonNumber,this.iAutonStep,this.iActionNumber,
						this.sAction,this.dValue,this.sDescription);

	System.out.println(sCSVLine);
	
  }

  public boolean isValidAction( String sAction ){
	  
	  //System.out.println("sAction: [" + sAction + "]");
	  if( sAction.equals("distance"))	return  true;
	  if( sAction.equals("bearing")) 	return  true;
	  if( sAction.equals("high")) 		return  true;
	  if( sAction.equals("ingest" )) 	return  true;
	  if( sAction.equals("power"))  	return  true;
	  if( sAction.equals("settle" ))  return  true;
	  if( sAction.equals("shoot")) 		return  true;
	  if( sAction.equals("spin" )) 		return  true;
	  if( sAction.equals("target"))  	return  true;
	  if( sAction.equals("timer"))  	return  true;
	  if( sAction.equals("turnto" ))	return  true;
	  
	  return false;
	  
  }
  
  public String toText(){

    return "Auton Number: " + String.valueOf(iAutonNumber) + System.lineSeparator() + 
        "Step: " + String.valueOf(iAutonStep)  + System.lineSeparator() +
        "Action Number:" + String.valueOf(iActionNumber)  + System.lineSeparator() +
        "Action:" + String.valueOf(sAction) + System.lineSeparator() +
        "Value:" + String.valueOf(dValue) + System.lineSeparator() +
        "Description: " + sDescription;

  }

  public String getCSVLine(){
    return sCSVLine;
  }


}
