// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.IntakeSubsystem;

public class IntakingWithoutSensorCommand extends Command {
  IntakeSubsystem m_IntakeSubsystem;
  /** Creates a new IntakingWithoutSensorCommand. */
  public IntakingWithoutSensorCommand(IntakeSubsystem intakeSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_IntakeSubsystem = intakeSubsystem;
    addRequirements(m_IntakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_IntakeSubsystem.IntakeIn();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_IntakeSubsystem.IntakeOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
