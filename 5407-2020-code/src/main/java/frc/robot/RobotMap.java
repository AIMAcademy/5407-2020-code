package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.SpeedControllerGroup;

public class RobotMap{
    //drive train
    public WPI_TalonFX drive_right_up;
    final int drive_right_up_ID = 1;

    public WPI_TalonFX drive_right_down;
    final int drive_right_down_ID = 2;

    public WPI_TalonFX drive_left_up;
    final int drive_left_up_ID = 3;

    public WPI_TalonFX drive_left_down;
    final int drive_left_down_ID = 4;

    public SpeedControllerGroup left_drive, right_drive;

    //shooter
    public WPI_TalonSRX shooting_direction;
    final int shooting_direction_ID = 6;

    public WPI_TalonFX shooting_wheel;
    final int shooting_wheel_ID = 5;

    public Servo hood;
    final int hood_ID = 0;

    public RobotMap(){
        //drive train
        drive_right_up = new WPI_TalonFX(drive_right_up_ID);
        drive_right_down = new WPI_TalonFX(drive_right_down_ID);
        drive_left_up = new WPI_TalonFX(drive_left_up_ID);
        drive_left_down = new WPI_TalonFX(drive_left_down_ID);

        right_drive = new SpeedControllerGroup(drive_right_up, drive_right_down);
        left_drive = new SpeedControllerGroup(drive_left_up, drive_left_down);

        //shooter
        shooting_direction = new WPI_TalonSRX(shooting_direction_ID);
        shooting_wheel = new WPI_TalonFX(shooting_wheel_ID);
        hood = new Servo(hood_ID);
    }
}