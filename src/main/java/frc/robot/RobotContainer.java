// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Commands.IntakingCommand;
import frc.robot.Commands.ShooterReverseCommand;
import frc.robot.Commands.ShootingCommand;
import frc.robot.Commands.StagingCommand;
import frc.robot.Commands.AutoTest;
import frc.robot.Commands.AutoShoot;
import frc.robot.Commands.shootandstage;
import frc.robot.Subsystems.CommandSwerveDrivetrain;
import frc.robot.Subsystems.IntakeSubsystem;
import frc.robot.generated.TunerConstants;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;



public class RobotContainer {
  // SlewRateLimiter is used to make joystick inputs more gentle; 1/3 sec from 0 to 1;
  private final SlewRateLimiter m_SpeedLimiter = new SlewRateLimiter(0); 
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(0);

  private final Joystick m_IntakeJoystick = new Joystick(0); 
  private final Joystick m_DriveJoystick = new Joystick(1);
  private final Joystick m_ShootingJoystick = new Joystick(2);
  private final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
  public AutoTest m_AutoTest;
  public AutoShoot m_AutoShoot;
  private double start_angle;
  

  private final SendableChooser<Command> autoChooser;

  private double MaxSpeed = 2; // 6 meters per second desired top speed
  private double MaxAngularRate = .5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  private final CommandJoystick joystick = new CommandJoystick(1);
  public final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

  public final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.2).withRotationalDeadband(MaxAngularRate * 0.5) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final Telemetry logger;






  private void configureBindings() {
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getY() * MaxSpeed) // Drive forward with
                                                                                       // negative Y (forward)
            .withVelocityY(-joystick.getX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-joystick.getTwist() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));
    

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    
  }

 
  public RobotContainer() {
    autoChooser = AutoBuilder.buildAutoChooser();

   // SmartDashboard.putData("Auto Chooser", autoChooser);
   // NamedCommands.registerCommand("IntakeCommand", new IntakeCommand(m_IntakeSubsystem));
        
   // m_chooser.setDefaultOption("Tests");
   // m_chooser.addOption("Nothing");
    //, new balanceAutoInside(s_Swerve, s_Intake)
   // SmartDashboard.putNumber("SpeedLimit", 1);

    logger = new Telemetry(MaxSpeed);
    drivetrain.registerTelemetry(logger::telemeterize);
    start_angle = drivetrain.getRotation3d().getZ();
    final JoystickButton IntakeButton = new JoystickButton(m_IntakeJoystick, 1);
    final JoystickButton ShootingButton = new JoystickButton(m_DriveJoystick, 2);
    final JoystickButton ShooterReverseButon = new JoystickButton(m_DriveJoystick, 3);
    final JoystickButton IntakeButton2 = new JoystickButton(m_DriveJoystick,1);

   // final JoystickButton AutoButton = new JoystickButton(m_IntakeJoystick, 4);
   
    ShooterReverseButon.whileTrue(new ShooterReverseCommand(m_IntakeSubsystem));
    ShootingButton.whileTrue(new ShootingCommand(m_IntakeSubsystem));
    IntakeButton2.whileTrue(new IntakingCommand(m_IntakeSubsystem));
    (IntakeButton.and(ShootingButton)).whileTrue(new shootandstage(m_IntakeSubsystem)); //composed command for multiple commands
    //AutoButton.onTrue(new AutoTest(drivetrain));

    
    
    
    configureBindings();

     //m_AutoTest = new AutoTest(drivetrain);
     //m_AutoShoot = new AutoShoot(m_IntakeSubsystem);

  }
  
  public void presentRotation(){
    SmartDashboard.putNumber("offset", Math.abs(drivetrain.getRotation3d().getZ() - start_angle));
  }


  public Command getAutonomousCommand() {
  
    // PathPlannerPath path = PathPlannerPath.fromPathFile("Tests");
     
   //return autoChooser.getSelected();

    return new PathPlannerAuto("Tests"); 
  }
}
