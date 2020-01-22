package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

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

    public DifferentialDrive drive_train;

    public RobotMap(){
        talon0 = new WPI_TalonFX(drive0_ID);
        talon1 = new WPI_TalonFX(drive1_ID);
        talon2 = new WPI_TalonFX(drive2_ID);
        talon3 = new WPI_TalonFX(drive3_ID);

        left_drive = new SpeedControllerGroup(talon0, talon1);
        right_drive = new SpeedControllerGroup(talon2, talon3);

        drive_train = new DifferentialDrive(left_drive, right_drive);
    }
}