// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix6.hardware.Pigeon2;
//import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

//import edu.wpi.first.math.kinematics.SwerveDriveOdometry;

// import java.io.IOException;
// import java.nio.file.Path;
// import java.util.List;

// import edu.wpi.first.math.controller.RamseteController;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.trajectory.Trajectory;
// import edu.wpi.first.math.trajectory.TrajectoryConfig;
// import edu.wpi.first.math.trajectory.TrajectoryUtil;
// import edu.wpi.first.math.trajectory.TrajectoryGenerator;
// import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
//import edu.wpi.first.wpilibj.Joystick;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
// import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.wpilibj.drive.DifferentialDrive;
// import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
// import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Subsystems.CommandSwerveDrivetrain;
// import frc.robot.generated.TunerConstants;
// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.math.controller.RamseteController;
// import edu.wpi.first.math.path.*;
// import edu.wpi.first.wpilibj.Timer;
import frc.robot.generated.TunerConstants;

public class Robot extends TimedRobot {
  Pigeon2 pigeon2;
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  //private SwerveRequest.FieldCentric drive;
  public final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain;
  public static DigitalInput MasterStagingSensor = new DigitalInput(0);
  public static DigitalOutput LightOutput = new DigitalOutput(3);
  //public static DigitalOutput motorLightOutput = new DigitalOutput(2);
  public static Spark spark = new Spark(0); // 0 is the RIO PWM port this is connected to

 // the % output of the motor, between -1 and 1
  // public boolean NoteLightsControl = false;
  public WPI_TalonSRX NoteLightsMotor;
  public WPI_TalonSRX RedLightsMotor;

  

  @Override
  public void robotInit() {
   m_robotContainer = new RobotContainer();
    try{
   NoteLightsMotor = new WPI_TalonSRX(20);
  }
  catch(Exception e){
    System.out.println("Note Lights Talon is missing");
  }
  try{
    RedLightsMotor = new WPI_TalonSRX(19);
  }
  catch(Exception e){
    System.out.println("Missing Amp Arm Talon");
  }
  }

  @Override
  public void robotPeriodic() {
     

    if (MasterStagingSensor.get()){
     // LightOutput.set(true);
     spark.set(0.61); 
     
      NoteLightsMotor.set(0);
      RedLightsMotor.set(.5);
    } else {
      // LightOutput.set(false);
      spark.set(0.77); 

      NoteLightsMotor.set(.3);
      RedLightsMotor.set(0);
    }
    CommandScheduler.getInstance().run(); 

  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");
  NetworkTableEntry tid = table.getEntry("tid");
  //NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid").getDoubleArray(new double[6]);


  
  //System.out.println((m_DriveJoystick.getRawAxis(3) + 1) / 2);
  //read values periodically
  double x = tx.getDouble(0.0);
  double y = ty.getDouble(0.0);
  double area = ta.getDouble(0.0);
  double aid = tid.getDouble(0);

  
  //post to smart dashboard periodically
  SmartDashboard.putNumber("LimelightX", x);
  SmartDashboard.putNumber("LimelightY", y);
  SmartDashboard.putNumber("LimelightArea", area);
  SmartDashboard.putNumber("tid", aid);
  m_robotContainer.presentRotation();

  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
      m_robotContainer.presentRotation();
    }
  }

  @Override
  public void autonomousPeriodic() {
        //m_robotContainer.drivetrain.moveLinear(.2, 0, m_robotContainer.drivetrain, m_robotContainer.drive);
        drivetrain.CheckMotorVoltage();
        
  }

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }
  
  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {
  }
}