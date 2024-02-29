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
import frc.robot.Commands.IntakeCommand;
import frc.robot.Commands.IntakingCommand;
import frc.robot.Commands.LeftClimbCommand;
import frc.robot.Commands.LeftClimbUpCommand;
import frc.robot.Commands.RightCLimbUpCommand;
import frc.robot.Commands.RightClimbCommand;
import frc.robot.Commands.ShootHighCommand;
import frc.robot.Commands.ShootTrapCommand;
import frc.robot.Commands.ShooterReverseCommand;
import frc.robot.Commands.ShootingCommand;
import frc.robot.Commands.StagingCommand;
import frc.robot.Commands.AutoTest;
//import frc.robot.Commands.CommandGroupTest;
import frc.robot.AutoCommands.AutoShootHighSeqential;
import frc.robot.AutoCommands.AutoShootHighSeqential2;
import frc.robot.AutoCommands.IntakeSeqential;
import frc.robot.Commands.DualClimbCommand;
import frc.robot.Commands.DualClimbUpCommand;
import frc.robot.AutoCommands.AutoShootHighSeqential;
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
import edu.wpi.first.util.sendable.Sendable;



public class RobotContainer {
  // SlewRateLimiter is used to make joystick inputs more gentle; 1/3 sec from 0 to 1;
  private final SlewRateLimiter m_XSpeedLimiter = new SlewRateLimiter(.95); 
  private final SlewRateLimiter m_ySpeedLimiter = new SlewRateLimiter(.95); 
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(.95);

  private final Joystick m_IntakeJoystick = new Joystick(0); 
  private final Joystick m_DriveJoystick = new Joystick(1);
  private final Joystick m_ShootingJoystick = new Joystick(2);
  private final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
  public AutoTest m_AutoTest;
  public AutoShoot m_AutoShoot;
  private double start_angle;

  
  private final SendableChooser<Command> autoChooser;

  private double MaxSpeed = 1; // 6 meters per second desired top speed
  private double MaxAngularRate = .25 * Math.PI; // 3/4 of a rotation per second max angular velocity

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

    

  //  NamedCommands.registerCommand("IntakingCommand", new IntakeCommand(m_IntakeSubsystem));
    NamedCommands.registerCommand("Intaking2Command", new IntakeSeqential(m_IntakeSubsystem));
  NamedCommands.registerCommand("IntakingCommand", new IntakingCommand(m_IntakeSubsystem).withTimeout(2));
  NamedCommands.registerCommand("ShootHighCommand", new AutoShootHighSeqential(m_IntakeSubsystem));
  //NamedCommands.registerCommand("ShootHighCommand2", new AutoShootHighSeqential2(m_IntakeSubsystem));
        
   //, new balanceAutoInside(s_Swerve, s_Intake)
   // SmartDashboard.putNumber("SpeedLimit", 1);

    logger = new Telemetry(MaxSpeed);
    drivetrain.registerTelemetry(logger::telemeterize);
    start_angle = drivetrain.getRotation3d().getZ();
    //Creates the Button, what joystick it is used for, and gives it a button ID;

    final JoystickButton IntakeButton = new JoystickButton(m_IntakeJoystick,1);
    final JoystickButton ShootingButton = new JoystickButton(m_IntakeJoystick, 2);
    final JoystickButton ShooterReverseButon = new JoystickButton(m_IntakeJoystick, 3);
    final JoystickButton DualClimbButton = new JoystickButton(m_IntakeJoystick, 4);
    final JoystickButton LeftClimbButton = new JoystickButton(m_IntakeJoystick, 5);
    final JoystickButton RightClimbButton = new JoystickButton(m_IntakeJoystick, 6);
    final JoystickButton LeftClimbUpButton = new JoystickButton(m_IntakeJoystick, 7);
    final JoystickButton RightClimbUpButton = new JoystickButton(m_IntakeJoystick, 8);
    final JoystickButton DualClimbUpButton = new JoystickButton(m_IntakeJoystick, 10);
    final JoystickButton ShootHighButton = new JoystickButton(m_ShootingJoystick, 1);
    final JoystickButton shootTrapButton = new JoystickButton(m_ShootingJoystick, 2);
    final JoystickButton ResetOrientation = new JoystickButton(m_ShootingJoystick, 3);

    

   
    //Tells the button what to do 


    IntakeButton.whileTrue(new IntakingCommand(m_IntakeSubsystem));
    ShootingButton.whileTrue(new ShootingCommand(m_IntakeSubsystem));
    (IntakeButton.and(ShootingButton)).whileTrue(new shootandstage(m_IntakeSubsystem)); //composed command used for running multiple button imputs 
    ShooterReverseButon.whileTrue(new ShooterReverseCommand(m_IntakeSubsystem));
    DualClimbButton.whileTrue(new DualClimbCommand(m_IntakeSubsystem));
    LeftClimbButton.whileTrue(new LeftClimbCommand(m_IntakeSubsystem));
    RightClimbButton.whileTrue(new RightClimbCommand(m_IntakeSubsystem));
    (LeftClimbButton.and(RightClimbButton)).whileTrue(new DualClimbCommand(m_IntakeSubsystem));
    LeftClimbUpButton.whileTrue(new LeftClimbUpCommand(m_IntakeSubsystem));
    RightClimbUpButton.whileTrue(new RightCLimbUpCommand(m_IntakeSubsystem));
    (LeftClimbUpButton.and(RightClimbUpButton)).whileTrue(new DualClimbUpCommand(m_IntakeSubsystem));
    DualClimbUpButton.whileTrue(new DualClimbUpCommand(m_IntakeSubsystem));
    ShootHighButton.whileTrue(new ShootHighCommand(m_IntakeSubsystem));
    shootTrapButton.whileTrue(new ShootTrapCommand(m_IntakeSubsystem));
    ResetOrientation.whileTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    //AutoButton.onTrue(new AutoTest(drivetrain));
    // final JoystickButton AutoButton = new JoystickButton(m_IntakeJoystick, 4);
    
    configureBindings();
    // SendableChooser<Command> m_chooser = new SendableChooser<>();
      autoChooser = AutoBuilder.buildAutoChooser();
    //  SmartDashboard.putData("Autonomous", m_chooser);
    //  m_chooser.setDefaultOption("Tests");
    //  m_chooser.addOption("Nothing");
     //m_AutoTest = new AutoTest(drivetrain);
     //m_AutoShoot = new AutoShoot(m_IntakeSubsystem);
  }
  
  public void presentRotation(){
    SmartDashboard.putNumber("offset", Math.abs(drivetrain.getRotation3d().getZ() - start_angle));
  }


  public Command getAutonomousCommand() {
  
    // PathPlannerPath path = PathPlannerPath.fromPathFile("Tests");
     
  // return autoChooser.getSelected();

    return new PathPlannerAuto("AutoAttack"); 
  }
}