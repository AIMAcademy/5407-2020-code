/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  RobotMap robotmap = new RobotMap();
  Controller controller = new Controller();
  Air air = new Air();

  double left_drive;
  double right_drive;

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    air.airint();
  }

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {
    controller.ReadVaues();

    //drive train
    right_drive = controller.getdriver_left_y();
    left_drive = -controller.getdriver_left_y();

    right_drive += controller.getdriver_right_x();
    left_drive += controller.getdriver_right_x();

    robotmap.left_drive.set(left_drive);
    robotmap.right_drive.set(right_drive);

    //transmission
    if(controller.getdriver_LB() == true){air.set_transmission(true);}
    else{air.set_transmission(false);}

    //shooter
    robotmap.shooting_direction.set(controller.getopperator_joystick_y());
  }

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}
}
