package frc.robot;

import edu.wpi.first.wpilibj.Solenoid;

public class Air {

    private Solenoid transmission;
    private final int transmission_ID = 0;

    public Air(){
        transmission = new Solenoid(transmission_ID);
    }

    public void airint(){
        transmission.set(false);
    }

    public void set_transmission(boolean high_low){transmission.set(high_low);}
}