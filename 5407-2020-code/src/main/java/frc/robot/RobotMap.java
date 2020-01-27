package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.SpeedControllerGroup;

public class RobotMap{
    //drive train
    public WPI_TalonFX talon0;
    final int drive0_ID = 0;

    public WPI_TalonFX talon1;
    final int drive1_ID = 1;

    public WPI_TalonFX talon2;
    final int drive2_ID = 2;

    public WPI_TalonFX talon3;
    final int drive3_ID = 3;

    public SpeedControllerGroup left_drive, right_drive;

    //shooter
    public WPI_TalonSRX shooting_direction;
    final int shooting_direction_ID = 13;

    public WPI_TalonFX shooting_wheel;
    final int shooting_wheel_ID = 4;

    public Servo hood;
    final int hood_ID = 5;

    //public DifferentialDrive drive_train;

    public RobotMap(){
        //drive train
        talon0 = new WPI_TalonFX(drive0_ID);
        talon1 = new WPI_TalonFX(drive1_ID);
        talon2 = new WPI_TalonFX(drive2_ID);
        talon3 = new WPI_TalonFX(drive3_ID);

        right_drive = new SpeedControllerGroup(talon0, talon1);
        left_drive = new SpeedControllerGroup(talon2, talon3);

        //drive_train = new DifferentialDrive(left_drive, right_drive);

        //shooter
        shooting_direction = new WPI_TalonSRX(shooting_direction_ID);
        shooting_wheel = new WPI_TalonFX(shooting_wheel_ID);
        hood = new Servo(hood_ID);
    }
}