// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.IntakeSubsystem;
import frc.robot.Subsystems.ShootingSubsystem;
import frc.robot.Subsystems.StagingSubsystem;

public class ShooterReverseCommand extends Command {
  IntakeSubsystem m_IntakeSubsystem;
  ShootingSubsystem m_ShootingSubsystem;
  StagingSubsystem m_StagingSubsystem;
  /** Creates a new ShooterReverseCommand. */
  public ShooterReverseCommand(IntakeSubsystem intakeSubsystem, ShootingSubsystem shootingSubsystem, StagingSubsystem stagingSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_IntakeSubsystem = intakeSubsystem;
    addRequirements(m_IntakeSubsystem);
    m_ShootingSubsystem = shootingSubsystem;
    addRequirements(m_ShootingSubsystem);
    m_StagingSubsystem = stagingSubsystem;
    addRequirements(m_StagingSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_ShootingSubsystem.ShootingMotor1.set(-.5);
    m_ShootingSubsystem.ShootingMotor2.set(-.5);
    m_StagingSubsystem.StagingMotor1.set(-0.15);
    m_StagingSubsystem.StagingMotor2.set(-0.15);
    m_IntakeSubsystem.IntakeMotor1.set(-0.1);
    m_IntakeSubsystem.IntakeMotor2.set(-0.1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_IntakeSubsystem.IntakeMotor1.set(0);
    m_IntakeSubsystem.IntakeMotor2.set(0);
    m_StagingSubsystem.StagingMotor1.set(0);
    m_StagingSubsystem.StagingMotor2.set(0);
    m_ShootingSubsystem.Rev(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
