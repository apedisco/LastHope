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
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Commands.ClimbJoystickCommand;
import frc.robot.Commands.IntakingWithSensorCommand;
import frc.robot.Commands.IntakingWithoutSensorCommand;
import frc.robot.Commands.NoteDumpCommand;
import frc.robot.Commands.ShooterReverseCommand;
import frc.robot.Commands.ToggleClimbJoysticksCommand;
import frc.robot.AutoCommands.AutoShootHighSeqential;
import frc.robot.AutoCommands.IntakeSeqential;
import frc.robot.AutoCommands.AutoShootHighSeqential;
import frc.robot.Commands.RevAndDeliverCommands.RevAndDeliverAmpCommand;
import frc.robot.Commands.RevAndDeliverCommands.RevAndDeliverSpeakerCommand;
import frc.robot.Commands.RevAndDeliverCommands.RevAndDeliverTrapCommand;
import frc.robot.Commands.RevCommands.RevAmpCommand;
import frc.robot.Commands.RevCommands.RevSpeakerCommand;
import frc.robot.Commands.RevCommands.RevTrapCommand;
import frc.robot.Commands.ShootingCommands.RevAndShootHighCommand;
import frc.robot.Commands.ShootingCommands.ShootAmpCommand;
import frc.robot.Commands.ShootingCommands.ShootHighCommand;
import frc.robot.Commands.ShootingCommands.ShootTrapCommand;
//import frc.robot.Commands.TestCommands.AntiNoteCommand;
import frc.robot.Commands.TestCommands.RevAndDeliverAprilTageTestCommand;
import frc.robot.Commands.TestCommands.RevAprilTagTestCommand;
import frc.robot.Commands.TestCommands.StagingCommand;
import frc.robot.Commands.TestCommands.shootandstage;
import frc.robot.Commands.TestCommands.ClimbCommands.DualClimbCommand;
import frc.robot.Commands.TestCommands.ClimbCommands.DualClimbUpCommand;
import frc.robot.Commands.TestCommands.ClimbCommands.LeftClimbCommand;
import frc.robot.Commands.TestCommands.ClimbCommands.LeftClimbUpCommand;
import frc.robot.Commands.TestCommands.ClimbCommands.RightCLimbUpCommand;
import frc.robot.Commands.TestCommands.ClimbCommands.RightClimbCommand;
import frc.robot.Subsystems.CommandSwerveDrivetrain;
import frc.robot.Subsystems.IntakeSubsystem;
import frc.robot.generated.TunerConstants;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.Sendable;



public class RobotContainer {
  // SlewRateLimiter is used to make joystick inputs more gentle; 1/3 sec from 0 to 1;
  // private final SlewRateLimiter m_XSpeedLimiter = new SlewRateLimiter(.95); 
  // private final SlewRateLimiter m_ySpeedLimiter = new SlewRateLimiter(.95); 
  // private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(.95);

  private final Joystick m_IntakeJoystick = new Joystick(0); 
  private final Joystick m_DriveJoystick = new Joystick(1);
  private final Joystick m_ShootingJoystick = new Joystick(2);
  private final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
  private ClimbJoystickCommand m_ClimbJoystickCommand;
  private double start_angle;

  
  // private final SendableChooser<Command> autoChooser;

  private double MaxSpeed = 4; // 6 meters per second desired top speed
  private double MaxAngularRate = 1 * Math.PI; // 3/4 of a rotation per second max angular velocity

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
        drivetrain.applyRequest(() -> drive.withVelocityX((-joystick.getY()) * MaxSpeed) // Drive forward with
                                                                                       // negative Y (forward)
            .withVelocityY((-joystick.getX()) * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate((-joystick.getTwist()) * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));
  // private void configureBindings() {
  //   drivetr
  // }
        // drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        // drivetrain.applyRequest(() -> drive.withVelocityX(m_SpeedLimiter.calculate(-joystick.getY()) * MaxSpeed) // Drive forward with
        //                                                                                // negative Y (forward)
        //     .withVelocityY(m_SpeedLimiter.calculate(-joystick.getX()) * MaxSpeed) // Drive left with negative X (left)
        //     .withRotationalRate(-joystick.getTwist() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        // ));  
    

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    
  }


  private double sendX(){
    double applied = -joystick.getY();
    if (applied < .1){
      return applied * .5 * MaxSpeed;
    } else 
      return applied * MaxSpeed;
  }
  

 
  public RobotContainer() {

  // boolean climbToggle = false;
    

  // NamedCommands.registerCommand("IntakingCommand", new IntakeCommand(m_IntakeSubsystem));
  NamedCommands.registerCommand("Intaking2Command", new IntakeSeqential(m_IntakeSubsystem));
  NamedCommands.registerCommand("IntakingCommand", new IntakingWithSensorCommand(m_IntakeSubsystem).withTimeout(2));
  NamedCommands.registerCommand("ShootHighCommand", new AutoShootHighSeqential(m_IntakeSubsystem));
  NamedCommands.registerCommand("RevSpeakerCommand", new RevSpeakerCommand(m_IntakeSubsystem).withTimeout(3));
  NamedCommands.registerCommand("RevAndDelieverSpeakerCommand", new RevAndDeliverSpeakerCommand(m_IntakeSubsystem).withTimeout(3));
  //.withIntteruptingBehavior might be something to try?
  // NamedCommands.registerCommand("ShootHighCommand2", new AutoShootHighSeqential2(m_IntakeSubsystem));

    logger = new Telemetry(MaxSpeed);
    drivetrain.registerTelemetry(logger::telemeterize);
    start_angle = drivetrain.getRotation3d().getZ();

    //Creates the Button, what joystick it is used for, and gives it a button ID;
    final JoystickButton IntakingWithSensorButton = new JoystickButton(m_IntakeJoystick,1);
    final JoystickButton RevAndDeliverButton = new JoystickButton(m_IntakeJoystick, 2);
    final JoystickButton ShooterReverseButon = new JoystickButton(m_IntakeJoystick, 3);
    final JoystickButton intakingWithoutSensorButton = new JoystickButton(m_IntakeJoystick, 5);
    // final JoystickButton AnitNoteButton = new JoystickButton(m_IntakeJoystick, 4);
     final JoystickButton DualClimbButton = new JoystickButton(m_IntakeJoystick, 4);
     final JoystickButton NoteDumpButton = new JoystickButton(m_IntakeJoystick, 5);
    // final JoystickButton RightClimbButton = new JoystickButton(m_IntakeJoystick, 6);
    // final JoystickButton LeftClimbUpButton = new JoystickButton(m_IntakeJoystick, 7);
    // final JoystickButton RightClimbUpButton = new JoystickButton(m_IntakeJoystick, 8);
    final JoystickButton DualClimbUpButton = new JoystickButton(m_IntakeJoystick, 10);
    final JoystickButton toggleClimbJoystickButton = new JoystickButton(m_IntakeJoystick, 12);
    final JoystickButton RevSpeakerButton = new JoystickButton(m_ShootingJoystick, 1);
    final JoystickButton RevTrapButton = new JoystickButton(m_ShootingJoystick, 2);
    final JoystickButton RevAmpButton = new JoystickButton(m_ShootingJoystick, 3);
    // final JoystickButton shootTrapButton = new JoystickButton(m_ShootingJoystick, 2);
    // final JoystickButton RevAprilTagButton = new JoystickButton(m_ShootingJoystick, 1);
    // final JoystickButton RevAndDeliverButton = new JoystickButton(m_ShootingJoystick, 2);
    final JoystickButton ResetOrientation = new JoystickButton(m_ShootingJoystick, 4);

    

   
    //Tells the button what to do 
    IntakingWithSensorButton.whileTrue(new IntakingWithSensorCommand(m_IntakeSubsystem));
    (RevAndDeliverButton.and(RevSpeakerButton)).whileTrue(new RevAndDeliverSpeakerCommand(m_IntakeSubsystem));
    (RevAndDeliverButton.and(RevAmpButton)).whileTrue(new RevAndDeliverAmpCommand(m_IntakeSubsystem));
    (RevAndDeliverButton.and(RevTrapButton)).whileTrue(new RevAndDeliverTrapCommand(m_IntakeSubsystem));
    ShooterReverseButon.whileTrue(new ShooterReverseCommand(m_IntakeSubsystem));
    intakingWithoutSensorButton.whileTrue(new IntakingWithoutSensorCommand(m_IntakeSubsystem));
    // AnitNoteButton.whileTrue(new AntiNoteCommand(m_IntakeSubsystem));
     DualClimbButton.whileTrue(new DualClimbCommand(m_IntakeSubsystem));
     NoteDumpButton.whileTrue(new NoteDumpCommand(m_IntakeSubsystem));
    // RightClimbButton.whileTrue(new RightClimbCommand(m_IntakeSubsystem));
    // (LeftClimbButton.and(RightClimbButton)).whileTrue(new DualClimbCommand(m_IntakeSubsystem)); //composed command used for running multiple button imputs 
    // LeftClimbUpButton.whileTrue(new LeftClimbUpCommand(m_IntakeSubsystem));
    // RightClimbUpButton.whileTrue(new RightCLimbUpCommand(m_IntakeSubsystem));
    // (LeftClimbUpButton.and(RightClimbUpButton)).whileTrue(new DualClimbUpCommand(m_IntakeSubsystem));
     DualClimbUpButton.whileTrue(new DualClimbUpCommand(m_IntakeSubsystem));
    toggleClimbJoystickButton.toggleOnTrue(new ToggleClimbJoysticksCommand(m_IntakeSubsystem, m_ShootingJoystick, m_IntakeJoystick));
    RevSpeakerButton.whileTrue(new RevSpeakerCommand(m_IntakeSubsystem));
    RevAmpButton.whileTrue(new RevAmpCommand(m_IntakeSubsystem));
    RevTrapButton.whileTrue(new RevTrapCommand(m_IntakeSubsystem));
    ResetOrientation.whileTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    //AutoButton.onTrue(new AutoTest(drivetrain));
    // final JoystickButton AutoButton = new JoystickButton(m_IntakeJoystick, 4);
    
    configureBindings();
    // SendableChooser<Command> m_chooser = new SendableChooser<>();
    //  autoChooser = AutoBuilder.buildAutoChooser();
    //  SmartDashboard.putData("Autonomous", m_chooser);
    //  m_chooser.setDefaultOption("Nothing", new PathPlannerAuto("Nothing"));
    //  m_chooser.addOption("Tests", new PathPlannerAuto("AutoAttack"));
  }
  
  public void presentRotation(){
    SmartDashboard.putNumber("offset", Math.abs(drivetrain.getRotation3d().getZ() - start_angle));
  }


  public Command getAutonomousCommand() {
  
 // PathPlannerPath path = PathPlannerPath.fromPathFile("Tests");
 // return autoChooser.getSelected();

     return new PathPlannerAuto("4Note"); 
  }
}