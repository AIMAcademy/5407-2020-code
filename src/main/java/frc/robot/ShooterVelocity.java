package frc.robot;

//import java.io.File;
//import java.io.FileWriter;
import java.io.*;
import java.util.TreeMap;			// usig TreeMap vs HashMap becasue TreeMap sorts by key or velocity. 
import java.util.Map;



public class ShooterVelocity{ 

  private String sFileName = null;
  
  private TreeMap<Integer, Double> mapValues = new TreeMap<Integer, Double>();

  public ShooterVelocity(String sPassedFilePath, String sPassedFileName){

    sFileName = sPassedFilePath + File.separator + sPassedFileName;
    
  }

  public void saveVelocity(Integer iVelocity, Double dFeedForward) {

	FileWriter fh = null;

	try {
		fh = new FileWriter(sFileName, true);                        //True means append
		fh.write(String.valueOf(iVelocity) + "," + String.valueOf(dFeedForward) + System.lineSeparator());	
		fh.close();
	}
	catch(IOException e) {
	  e.printStackTrace();
	}

  }

  public void addVelocity(Integer iVelocity, Double dFeedForward) {
	mapValues.put(iVelocity, dFeedForward); 
	//System.out.println("KV Pair:" + sKVPair[0] + ", " + sKVPair[1]);
  }

  
  public void loadTable() {

	mapValues.clear();

    BufferedReader bufr = null;
	  
	try {
		FileReader fr = new FileReader(sFileName);
        bufr = new BufferedReader(fr);
	        
		String line = bufr.readLine();
		while(line != null){
			String[] sKVPair = line.split(",");											// break up line into Key and Value Strings
			mapValues.put(Integer.valueOf(sKVPair[0]), Double.valueOf(sKVPair[1]));		// save to map as Integet, Double
			System.out.println("KV Pair:" + sKVPair[0] + ", " + sKVPair[1]);
			line = bufr.readLine();														// get next line
		}

		bufr.close();
	
	}
	catch(IOException e) {
	  e.printStackTrace();
	}

  }
  
  public void dumpMap() {
	  
	          // using for-each loop for iteration over Map.entrySet() 
        for (Map.Entry<Integer,Double> entry : mapValues.entrySet())  
            System.out.println("Key = " + entry.getKey() + 
                             ", Value = " + entry.getValue()); 
	  
  }
  
  //public void getFeedForward(int iVelocity) {
	  
//	  getFeedForward( Integer.valueOf(iVelocity) );
	  
//  }

  public void getFeedForward(Integer iVelocity) {
	
	Integer floorKey = mapValues.floorKey(Integer.valueOf(iVelocity));
	Integer ceilingKey = mapValues.ceilingKey(Integer.valueOf(iVelocity));

    System.out.println("Key = " + String.valueOf(iVelocity) + " | " + 
                             "floor: " + String.valueOf(floorKey) + " , " + mapValues.get(floorKey) +
							 "  next: " + String.valueOf(ceilingKey)  + " , " + mapValues.get(ceilingKey) );

  }

  public Double getFeedForwardRange(Integer iVelocity) {
	
	Integer floorKey = mapValues.floorKey(Integer.valueOf(iVelocity));
	//Integer ceilingKey = mapValues.ceilingKey(Integer.valueOf(iVelocity));

	return 0.04805590000000001;
	//return mapValues.get(floorKey);
	
    //System.out.println("Key = " + String.valueOf(iVelocity) + " | " + 
    //                         "floor: " + String.valueOf(floorKey) + " , " + mapValues.get(floorKey) +
	//						 "  next: " + String.valueOf(ceilingKey)  + " , " + mapValues.get(ceilingKey) );
	  
  }


  
}
