package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;

public class Controller{

    public XboxController driver;
    private double driver_left_x;
    private double driver_left_y;
    private double driver_right_x;
    private double driver_right_y;
    private boolean driver_LB;

    public Joystick opperator;
    private double opperator_y;
    private double opperator_x;
    private boolean opperator_trigger;

    public Controller(){
        driver = new XboxController(0);
        opperator = new Joystick(3);
    }

    public void ReadVaues(){
        //driver
        //joysitcks
        driver_left_x = driver.getRawAxis(0);
        if(driver_left_x <= .02 && driver_left_x >= -.02){driver_left_x = 0;}
        driver_left_y = driver.getRawAxis(1);
        if(driver_left_y <= .02 && driver_left_y >= -.02){driver_left_y = 0;}
        driver_right_x = driver.getRawAxis(4);
        if(driver_right_x <= .02 && driver_right_x >= -.02){driver_right_x = 0;}
        driver_right_y = driver.getRawAxis(5);
        if(driver_right_y <= .02 && driver_right_y >= -.02){driver_right_y = 0;}
        //buttons
        driver_LB = driver.getRawButton(5);

        //opperator
        //joysticks
        opperator_x = opperator.getRawAxis(0);
        if(opperator_x <= .02 && opperator_x >= -.02){opperator_x = 0;}
        opperator_y = opperator.getRawAxis(1);
        if(opperator_y <= .02 && opperator_y >= -.02){opperator_y = 0;}
        //buttons
        opperator_trigger = opperator.getRawButton(1);
    }
    //driver
    public double getdriver_left_y(){return driver_left_y;}
    public double getldriver_left_x(){return driver_right_y;}
    public double getdriver_right_y(){return driver_left_y;}
    public double getdriver_right_x(){return driver_right_y;}
    public boolean getdriver_LB(){return driver_LB;}
    //opperator
    public double getopperator_joystick_x(){return opperator_x;}
    public double getopperator_joystick_y(){return opperator_y;}
    public boolean getopperator_trigger(){return opperator_trigger;}
}