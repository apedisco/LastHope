// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.IntakeSubsystem;

public class ClimbJoystickCommand extends Command {
  IntakeSubsystem m_IntakeSubsystem;
  DoubleSupplier m_RightClimbSpeed;
  DoubleSupplier m_LeftClimbSpeed;

  /** Creates a new ClimbJoystickCommand. */
  public ClimbJoystickCommand(IntakeSubsystem intakeSubsystem, DoubleSupplier RightClimbSpeed, DoubleSupplier LeftClimbSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_RightClimbSpeed = RightClimbSpeed;
    m_LeftClimbSpeed = LeftClimbSpeed;
    m_IntakeSubsystem = intakeSubsystem;
    addRequirements(m_IntakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("joystick input");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double ActualRightClimbSpeed = m_RightClimbSpeed.getAsDouble();
    double ActualLeftClimbSpeed = m_LeftClimbSpeed.getAsDouble();
    System.out.println("joystick input");
    // if (Math.abs(ActualRightClimbSpeed) <= Math.abs(0.1)){
    //   m_IntakeSubsystem.RightCLimb(RightClimbMotorSpeed);
    // }
    if (ActualLeftClimbSpeed > 0){
      m_IntakeSubsystem.RightCLimbDown();
    } else if (ActualLeftClimbSpeed < 0){
      m_IntakeSubsystem.RightCLimbUp();
    }
    if (ActualRightClimbSpeed > 0){
      m_IntakeSubsystem.LeftClimbDown();
    } else if (ActualRightClimbSpeed < 0){
      m_IntakeSubsystem.LeftClimbUp();
    }



  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
