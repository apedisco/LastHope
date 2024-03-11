// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.TestCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.IntakeSubsystem;

public class shootandstage extends Command {
  IntakeSubsystem m_IntakeSubsystem;
  /** Creates a new IntakeCommand. */
  public shootandstage(IntakeSubsystem intakeSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_IntakeSubsystem = intakeSubsystem;
    addRequirements(m_IntakeSubsystem);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //System.out.println("First Motor \n");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_IntakeSubsystem.IntakeIn();
    m_IntakeSubsystem.Rev(.12);//.10
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_IntakeSubsystem.IntakeOff();
    m_IntakeSubsystem.Rev(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
