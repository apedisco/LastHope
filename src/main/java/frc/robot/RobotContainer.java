// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
// import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
// import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
//import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
//import frc.robot.Commands.CheckMotorVoltagesCommand;
//import frc.robot.Commands.ClimbJoystickCommand;
import frc.robot.Commands.IntakingWithSensorCommand;
import frc.robot.Commands.IntakingWithoutSensorCommand;
import frc.robot.Commands.ShooterReverseCommand;
import frc.robot.Commands.ToggleClimbJoysticksCommand;
import frc.robot.AutoCommands.AutoShootHighSeqential;
import frc.robot.Commands.RevAndDeliverCommands.RevAndDeliverAmpCommand;
import frc.robot.Commands.RevAndDeliverCommands.RevAndDeliverSpeakerCommand;
import frc.robot.Commands.RevAndDeliverCommands.RevAndDeliverTrapCommand;
import frc.robot.Commands.RevAndDeliverCommands.ClimbCommands.DualClimbCommand;
import frc.robot.Commands.RevAndDeliverCommands.ClimbCommands.DualClimbUpCommand;
import frc.robot.Commands.RevCommands.RevAmpCommand;
import frc.robot.Commands.RevCommands.RevSpeakerCommand;
import frc.robot.Commands.RevCommands.RevTrapCommand;
import frc.robot.Subsystems.ClimbingSubsystem;
import frc.robot.Subsystems.CommandSwerveDrivetrain;
import frc.robot.Subsystems.IntakeSubsystem;
import frc.robot.Subsystems.ShootingSubsystem;
import frc.robot.Subsystems.StagingSubsystem;
import frc.robot.generated.TunerConstants;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



public class RobotContainer {
  // SlewRateLimiter is used to make joystick inputs more gentle; 1/3 sec from 0 to 1;
  // private final SlewRateLimiter m_XSpeedLimiter = new SlewRateLimiter(.95); 
  // private final SlewRateLimiter m_ySpeedLimiter = new SlewRateLimiter(.95); 
  // private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(.95);
  private final Joystick m_IntakeJoystick = new Joystick(0); 
  private final Joystick m_DriveJoystick = new Joystick(1);
  private final Joystick m_ShootingJoystick = new Joystick(2);

  private final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
  private final ShootingSubsystem m_ShootingSubsystem = new ShootingSubsystem();
  private final StagingSubsystem m_StagingSubsystem = new StagingSubsystem();
  private final ClimbingSubsystem m_ClimbingSubsystem = new ClimbingSubsystem();

  private double start_angle;

  SendableChooser<Command> m_chooser = new SendableChooser<>();

  public double MaxSpeed = 4; // 6 meters per second desired top speed
  private double MaxAngularRate = 1 * Math.PI; // 3/4 of a rotation per second max angular velocity

  private final CommandJoystick joystick = new CommandJoystick(1);
  public final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

  public final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.2).withRotationalDeadband(MaxAngularRate * 0.5) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  // private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  // private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
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

  // private double sendX(){
  //   double applied = -joystick.getY();
  //   if (applied < .1){
  //     return applied * .5 * MaxSpeed;
  //   } else 
  //     return applied * MaxSpeed;
  // }
  

 
  public RobotContainer() {

  NamedCommands.registerCommand("IntakingCommand", new IntakingWithSensorCommand(m_IntakeSubsystem, m_StagingSubsystem).withTimeout(2));
  NamedCommands.registerCommand("ShootHighCommand", new AutoShootHighSeqential(m_ShootingSubsystem, m_StagingSubsystem));

  //.withIntteruptingBehavior might be something to try?

    logger = new Telemetry(MaxSpeed);
    drivetrain.registerTelemetry(logger::telemeterize);
    start_angle = drivetrain.getRotation3d().getZ();


  //Creates the Button, what joystick it is used for, and gives it a button ID;

    // IntakeJoystick (0)
    final JoystickButton RevAmpButton = new JoystickButton(m_IntakeJoystick, 1);
    final JoystickButton RevAndDeliverButton = new JoystickButton(m_IntakeJoystick, 2);
    final JoystickButton ShooterReverseButon = new JoystickButton(m_IntakeJoystick, 3);
                                                                                                    //final JoystickButton TestingMotorVoltagesButton = new JoystickButton(m_IntakeJoystick, 4);
    final JoystickButton DualClimbButton = new JoystickButton(m_IntakeJoystick, 5);
    final JoystickButton DualClimbUpButton = new JoystickButton(m_IntakeJoystick, 6);
    final JoystickButton toggleClimbJoystickButton = new JoystickButton(m_IntakeJoystick, 12);

    // DriveJoystick (1){ 
    final JoystickButton IntakingWithSensorButton = new JoystickButton(m_DriveJoystick,1);
    final JoystickButton ResetOrientation = new JoystickButton(m_DriveJoystick, 2);
    final JoystickButton intakingWithoutSensorButton = new JoystickButton(m_DriveJoystick, 3);
                                                                                                             //final JoystickButton testPID = new JoystickButton(m_DriveJoystick, 3);}

    // ShootingJoystick (2)
    final JoystickButton RevSpeakerButton = new JoystickButton(m_ShootingJoystick, 1);
    final JoystickButton RevTrapButton = new JoystickButton(m_ShootingJoystick, 2);
    


    

   
  //Tells the button what to do 

    // IntakeJoystick (0)
   //RevAmpButton.onTrue(new InstantCommand(() -> presentRotation()));
    RevAmpButton.whileTrue(new RevAmpCommand(m_ShootingSubsystem));
    (RevAndDeliverButton.and(RevSpeakerButton)).whileTrue(new RevAndDeliverSpeakerCommand(m_ShootingSubsystem, m_StagingSubsystem));
    (RevAndDeliverButton.and(RevAmpButton)).whileTrue(new RevAndDeliverAmpCommand(m_IntakeSubsystem, m_ShootingSubsystem, m_StagingSubsystem));
    (RevAndDeliverButton.and(RevTrapButton)).whileTrue(new RevAndDeliverTrapCommand(m_ShootingSubsystem,m_StagingSubsystem, m_DriveJoystick, m_IntakeJoystick, m_ShootingJoystick));
    ShooterReverseButon.whileTrue(new ShooterReverseCommand(m_IntakeSubsystem, m_ShootingSubsystem, m_StagingSubsystem));
                                                                                                                          //TestingMotorVoltagesButton.toggleOnTrue(new CheckMotorVoltagesCommand(drivetrain));
    DualClimbButton.whileTrue(new DualClimbCommand(m_ClimbingSubsystem));
    DualClimbUpButton.whileTrue(new DualClimbUpCommand(m_ClimbingSubsystem));
    toggleClimbJoystickButton.toggleOnTrue(new ToggleClimbJoysticksCommand(m_ClimbingSubsystem, m_ShootingJoystick, m_IntakeJoystick));

    // DriveJoystick (1)
    intakingWithoutSensorButton.whileTrue(new IntakingWithoutSensorCommand(m_IntakeSubsystem, m_StagingSubsystem));
    ResetOrientation.whileTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));
    IntakingWithSensorButton.whileTrue(new IntakingWithSensorCommand(m_IntakeSubsystem, m_StagingSubsystem));
                                                                                                            //testPID.onTrue(new InstantCommand(() -> m_IntakeSubsystem.pidShooting(500)));
   
    // ShootingJoystick (2) 
    RevSpeakerButton.whileTrue(new RevSpeakerCommand(m_ShootingSubsystem));
    RevTrapButton.whileTrue(new RevTrapCommand(m_ShootingSubsystem, m_DriveJoystick, m_IntakeJoystick, m_ShootingJoystick));
    


    
    // DO NOT DELETE, this is important for the drivetrain to work
    configureBindings();
    // DO NOT DELETE, this is important for the drivetrain to work
    
     SmartDashboard.putData("Autonomous", m_chooser);
     m_chooser.setDefaultOption("Nothing", new PathPlannerAuto("Nothing"));
     m_chooser.addOption("TestAuto", new PathPlannerAuto("TestAuto"));
     m_chooser.addOption("Row2Notes4&5", new PathPlannerAuto("Row2Notes4&5"));
     m_chooser.addOption("4Note", new PathPlannerAuto("4Note"));
     m_chooser.addOption("Row1Notes2&3", new PathPlannerAuto("Row1Notes2&3"));
     m_chooser.addOption("Row1Note3", new PathPlannerAuto("Row1Note3"));
     m_chooser.addOption("New Auto", new PathPlannerAuto("New Auto"));
  }
  
  public void presentRotation(){
    SmartDashboard.putNumber("offset", Math.abs(drivetrain.getRotation3d().getZ() - start_angle));
  }
  

  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }

}