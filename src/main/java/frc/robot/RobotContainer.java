// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Commands.IntakeCommand;
import frc.robot.Commands.ShooterReverseCommand;
import frc.robot.Commands.ShootingCommand;
import frc.robot.Commands.StagingCommand;
import frc.robot.Commands.AutoTest;
import frc.robot.Commands.shootandstage;
import frc.robot.Subsystems.IntakeSubsystem;
import frc.robot.generated.TunerConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;


public class RobotContainer {
 

  private final Joystick m_IntakeJoystick = new Joystick(0); 
  private final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
  private AutoTest m_AutoTest;

  private double MaxSpeed = 2; // 6 meters per second desired top speed
  private double MaxAngularRate = .5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  private final CommandJoystick joystick = new CommandJoystick(1);
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.2).withRotationalDeadband(MaxAngularRate * 0.5) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final Telemetry logger = new Telemetry(MaxSpeed);






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
    drivetrain.registerTelemetry(logger::telemeterize);
  }




  
  public RobotContainer() {
    final JoystickButton IntakeButton = new JoystickButton(m_IntakeJoystick, 1);
    final JoystickButton ShootingButton = new JoystickButton(m_IntakeJoystick, 2);
    final JoystickButton ShooterReverseButon = new JoystickButton(m_IntakeJoystick, 3);
    final JoystickButton AutoButton = new JoystickButton(m_IntakeJoystick, 4);


    ShooterReverseButon.whileTrue(new ShooterReverseCommand(m_IntakeSubsystem));
    IntakeButton.whileTrue(new IntakeCommand(m_IntakeSubsystem));
    ShootingButton.whileTrue(new ShootingCommand(m_IntakeSubsystem));
    (IntakeButton.and(ShootingButton)).whileTrue(new shootandstage(m_IntakeSubsystem)); //composed command for multiple commands
    AutoButton.onTrue(new AutoTest(drivetrain));
    

    configureBindings();
  }
  
  public Command getAutonomousCommand() {
    return m_AutoTest;
  }
}
