// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.ClimbCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.ClimbingSubsystem;


public class RightClimbCommand extends Command {
  ClimbingSubsystem m_ClimbingSubsystem;
  /** Creates a new RightClimbCommand. */
  public RightClimbCommand(ClimbingSubsystem climbingSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_ClimbingSubsystem = climbingSubsystem;
    addRequirements(m_ClimbingSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_ClimbingSubsystem.RightCLimbDown();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_ClimbingSubsystem.RightCLimbStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
