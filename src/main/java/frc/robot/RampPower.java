
package frc.robot;



public class RampPower{
	
	public double dPctCloseToTarget = 0.0;			// percentage clost to target to call for stop. Signal arrived 
	public boolean bArrived = false;				// signal that we have arrived, Motor power will be set to 0.0;
	public boolean bStopOnArrival = false;			// signal that we want to stop on arrival, Motor power will be set to 0.0;
	public boolean bCloseToTarget = false;			// signal that we are within range of the target
	
	public double dStartPosition = 0.0;				// where did we start in this?
	public double dEndPosition = 0.0;				// end position of where you want to go
	public double dRampUpPct = 0.0;					// how much of the start distance do I want to ramp up the power  0.0 = none
	public double dRampDownPct = 0.0;				// how much of the end distance do I want to ramp down the power  0.0 = none
	public double dDownProportion = 0.0;			// proportional ramp down value, if 0.0 not used
	public double dMinPower = 0.0;					// minimum power to use. Just enough to move it
	public double dMaxPower = 0.0;					// maximum power to use
	public double dRampDownDistancePct =  0.0;		// what percentage of total distance is the end part
	public double dTotalDistance = 0.0;				// calculated total distance, Math.abs(end - start)
	public double dPowerInverter = 1.0;				// used to reverse sign on output power if motor needs it, 1.0 = no inversion
	public double dInvertDirection = 1.0;			// set the motor to reverse if we are moving negative. 1.0 = no inversion
	public boolean bDebug = false;
	
	// Overloading the constructor  /////////////////////////////////
	public RampPower(double dOrigMinPower, double dOrigMaxPower, double dOrigRampUpPct, 
										   double dOrigRampDownPct, double dOrigDownProportion){
		this.dMinPower 		 	  	= Math.abs(dOrigMinPower);
		this.dMaxPower 		 	  	= Math.abs(dOrigMaxPower);
		this.dRampUpPct 	 	  	= Math.abs(dOrigRampUpPct);
		this.dRampDownPct 	 		= Math.abs(dOrigRampDownPct);
		this.dDownProportion 		= Math.abs(dOrigDownProportion);
		//this.dPctCloseToTarget 		= Math.abs(dOrigPCTCloseToStop);
		
		//if(bInvertOutput == true){
		//	this.dPowerInverter = -1.0;
		//}
		

		this.dRampDownDistancePct 	= 1.0-dRampDownPct;	 // what percentage of total distance is the end part
		
	}

	//public RampPower(double dOrigMinPower, double dOrigMaxPower){
	//	this.dMinPower = Math.abs(dOrigMinPower);
	//	this.dMaxPower = Math.abs(dOrigMaxPower);
	//}

	////////////////////////////////////////////////////////////////
	
	public void setStartEndPosition(double dNewStartPosition, double dNewEndPosition){
		this.dStartPosition = dNewStartPosition;
		this.dEndPosition = dNewEndPosition;
		this.bArrived = false;
		this.dTotalDistance = Math.abs(this.dEndPosition - this.dStartPosition);
		
		this.dInvertDirection = 1.0;								// assumes no direction inversion
		if( this.dEndPosition < this.dStartPosition){				// direct is negative
			this.dInvertDirection = -1.0;							// set inverter to -1.0 to flip the sign. 
		}
	}
	
	public void setRampUpPct( double dNewRampUpPct ){
		this.dRampUpPct = dNewRampUpPct;
	}
	
	public void setRampDownPct( double dNewRampDownPct ){
		this.dRampDownPct = dNewRampDownPct;
		this.dRampDownDistancePct = 1.0-dRampDownPct;	 // what percentage of total distance is the end part
	}

	public void setRampDownProportion( double dNewDownProportion ){
		this.dDownProportion = dNewDownProportion;
	}
	
	public void printHeader(){
		if( this.bDebug == true ){
			System.out.println( String.format("\n\nDist: %5.2f, Start: %5.0f, End: %5.0f, DownProp: %5.2f", 
									this.dTotalDistance, this.dStartPosition, this.dEndPosition, this.dDownProportion));
			System.out.println( String.format("****: %4s | %5s | %5s | %5s | ", "DFT ", "PFSTR", "PPUP ", "Power"));    
		}
	}
	
	public void setPowerOutputInversion(boolean bState ){
		if( bState == false ){
			this.dPowerInverter	=  1.0;
		} else {
			this.dPowerInverter	= -1.0;
		}
	}	

	public void setPctCloseToTarget(double dPassedPctCloseToTarget ){
		this.dPctCloseToTarget 	= Math.abs(dPassedPctCloseToTarget);
	}	

	public void setStopOnArrival(boolean bInValue, double dPassedPctCloseToTarget ){
		this.bStopOnArrival = bInValue;
		if( bInValue == true ){
			this.dPctCloseToTarget 	= Math.abs(dPassedPctCloseToTarget);
		} else {
			this.dPctCloseToTarget 	= 0.0;
		}
	}	

	
	public double calcPower( double dPower, double dCurrPosition ){

		dPower = Math.abs(dPower);

		//double dInverter = 1.0;				// save this for the end. we will do the power calcs in positive range
		//if(dPower < 0){
		//		dInverter = -1.0;
		//		dPower = Math.abs(dPower);
		//}

		
		// 0.........1.........2.........3.........4.........5.........6.........7.........8.........9.........100
		// 0....5....0....5....0....5....0....5....0....5....0....5....0....5....0....5....0....5....0....5....0
		// Example:  Start = 20^                             ^ current = 50         			     ^End = 90
		// distance = curr - start  50-20 = 30          
		// PctFromStart = 50/90 = .56 = 56% 

		double dDistanceFromTarget = Math.abs(dCurrPosition - this.dStartPosition);				// how far have we gone?
		double dDistanceToTarget   = this.dTotalDistance  - dDistanceFromTarget;		// how much left?
		this.bCloseToTarget = false;
		this.bArrived = false;

		// | ----- Ramp Up ----|------------------------ Full Power -----------------------|----- Ramp Down ---|
		// 0.........1.........2.........3.........4.........5.........6.........7.........8.........9.........100
		// 0....5....0....5....0....5....0....5....0....5....0....5....0....5....0....5....0....5....0....5....0
		// Example:            ^ % from start                                              ^ %from end = .80
			
		double dPctFromStart 	 = dDistanceFromTarget/
												this.dTotalDistance;  	// what percentage from the start are we
		double dPctFromEnd 		 = 1.00 - dPctFromStart;				// what percentage to the end are we  1.00 = 100% 
		
		if(this.bStopOnArrival == true && 
			dPctCloseToTarget > 0.0 &&									// did we want to stop before the target?
			dPctFromEnd <= dPctCloseToTarget) {							// are we close enough to the target
			this.bArrived = true;										// set flag

			if( this.bDebug == true )
				System.out.println( String.format("ARIV: arrived = true"));    

			return 0.0;													// return 0.0 motor power to stop motors
		}

		if(dPctCloseToTarget > 0.0 &&									// we want to indicate we are close to the target?
			Math.abs(dPctFromEnd) <= dPctCloseToTarget) {				// are we close enough to the target
				this.bCloseToTarget = true;								// set flag
		}

		
		// *****************************************************
		// test ramp up	
		// *****************************************************
		if( dRampUpPct > 0.0 &&													// do we have a ramp up percentage, default = 0.0 = no ramp up
			dPctFromStart < dRampUpPct) {										// we are within the start pct from beginning 
				double dPctOfUpPower = dPctFromStart/dRampUpPct;				// what percentage of the start up are we now at 100% to RampDown%? 
				dPower = normalizePower(dPower * dPctOfUpPower, 
											dDistanceToTarget);   				// return a percentage of the power 
			
			if( this.bDebug == true )
				System.out.println( String.format("  Up: % 4.0f | %5.2f | %5.2f | %5.2f | ", dDistanceFromTarget, dPctFromStart, dPctOfUpPower, dPower));    
				
		// *****************************************************
		// test for ramp down
		// *****************************************************
		} else if( dRampDownPct > 0.0 &&							 	 		// do we have a ramp down percentage, , default = 0.0 = no ramp down
			dPctFromStart > dRampDownPct) {							 	 		// we are within the end ramp down pct RampDown% to 100%
			
				if(this.dDownProportion > 0.0){									// use proportional calcuation vs. linear
					dPower = normalizePower(Math.abs(dDistanceToTarget) * this.dDownProportion, 
											dDistanceToTarget);   				// return a percentage of the power 
					
					if( this.bDebug == true )
						System.out.println( String.format("DPro: % 4.0f | %5.2f | %5.2f | %5.2f | %5.5s | %5.5s ", 
								dDistanceFromTarget, dPctFromStart, 
								dDownProportion, dPower, 
								String.valueOf(this.bCloseToTarget), String.valueOf(this.bArrived) ));    
					
				} else {														// using linear calculation
					double dPctIntoRampDown = dPctFromStart - dRampDownPct;  	// what pct are we into ramp down.
					double dPctOfDownPower = (this.dRampDownDistancePct -dPctIntoRampDown)/this.dRampDownDistancePct; // what percentage of the end down are we now at? 
					dPower = normalizePower( dPower * dPctOfDownPower, 
													dDistanceToTarget);		 	// return a percenatge of the power 
				
				if( this.bDebug == true )
					System.out.println( String.format("DOWN: % 4.0f | %5.2f | %5.2f | %5.2f | %5.5s | %5.5s ", 
								dDistanceFromTarget, dPctFromStart, 
								dDownProportion, dPower, 
								String.valueOf(this.bCloseToTarget), String.valueOf(this.bArrived) ));    
				}
				
		// *****************************************************
		// we are in the middle of the run
		// *****************************************************
		} else { 
				dPower = normalizePower( dPower, dDistanceToTarget );

				if( this.bDebug == true )
					System.out.println( String.format("Full: % 4.0f | %5.2f | %5.2f | %5.2f | ", dDistanceFromTarget, dPctFromStart, 0.0, dPower));    
		}
		
		
		return dPower; 

	}
	
	private double normalizePower( double dPower, double dDistanceToTarget ){

		if( dPower <= this.dMinPower ){
			dPower = dMinPower;
		} else if( dPower > this.dMaxPower){
			dPower = dMaxPower;
		}			
		
		if( dDistanceToTarget < 0.0){						// did we pass the end point?
			dPower *= -1.0;									// invert as we passed the end point. 
		}
		
		
		dPower *= this.dPowerInverter;		 				// apply the requested inversion by the user
		dPower *= this.dInvertDirection;					// apply direction inversion as the direction is negative. 

		return dPower;

	}		
	
}