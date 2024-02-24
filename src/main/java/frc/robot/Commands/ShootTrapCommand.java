// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.IntakeSubsystem;

public class ShootTrapCommand extends Command {
  IntakeSubsystem m_IntakeSubsystem;
  private double EngageTime;
  /** Creates a new ShootTrapCommand. */
  public ShootTrapCommand(IntakeSubsystem intakeSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_IntakeSubsystem = intakeSubsystem;
    addRequirements(m_IntakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    EngageTime = System.currentTimeMillis();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_IntakeSubsystem.Rev(.45);//Trap.45// Amp speed .1// Speaker Speed? .5
    

    if (System.currentTimeMillis() - EngageTime > 600){
      m_IntakeSubsystem.Deliver(-.5);
     // m_IntakeSubsystem.Intake();
      // Amp -.30 time 200//Speaker -.5 time 600//Trap -.5 time 800
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_IntakeSubsystem.Rev(0);
    m_IntakeSubsystem.Deliver(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
