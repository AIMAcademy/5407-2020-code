/**
 * Simple class containing constants used throughout project
 */
package frc.robot;

class RobotMap {

    // USB Ports
    public final static int kUSBPort_DriverControl = 0; 
    public final static int kUSBPort_OperatorControl = 1;
    public final static int kUSBPort_TestJoyStick = 2;

    // CAN Bus IDs
    public final static int kCANId_RightDriveMotorA = 1;
    public final static int kCANId_RightDriveMotorB = 2;
    public final static int kCANId_LeftDriveMotorA = 3;
    public final static int kCANId_LeftDriveMotorB = 4;

    public final static int kCANId_ShooterMotor = 5;
    public final static int kCANId_ShooterTurretMotor = 6;

    public final static int kCANId_PCM = 10;

    // Pneumatic Control Module Ids
    public final static int kPCMPort_DriveShifter = 0;
    public final static int kPCMPort_BallPusherUpper = 4;

   // Digital Inputs/Outputs
   //public final static int kDigitalIOPort_ = 0;

   // Analog Inputs

   //public final static int kAnalogPort_ = 0;
   public final static int kAnalogPort_ShooterHeight = 0;
   public final static int kAnalogPort_TurretPos = 1;

   // PWM Ports
   public final static int kPWMPort_ShooterHeight = 5;

   // Input Bottons
   public final static int kButton_ShooterHeightRaise = 4;
   public final static int kButton_ShooterHeightLower = 6;

   public final static int kButton_ShooterVelocity_Raise = 8;
   public final static int kButton_ShooterVelocity_Lower = 9;

}