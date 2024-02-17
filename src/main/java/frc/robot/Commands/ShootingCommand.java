// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj.Timer;

public class ShootingCommand extends Command {
  public Timer m_timer;

  IntakeSubsystem m_IntakeSubsystem;
  double EngageTime;
  /** Creates a new ShootingCommand. */
  public ShootingCommand(IntakeSubsystem intakeSubsystem) {
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
    

    m_IntakeSubsystem.Rev(.10);// Amp speed .1// Speaker Speed? .5
    

    if (System.currentTimeMillis() - EngageTime > 200){
      m_IntakeSubsystem.Deliver(-.30);// Amp -.25 time 200//Speaker -.5 time 600
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_IntakeSubsystem.ShootOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
