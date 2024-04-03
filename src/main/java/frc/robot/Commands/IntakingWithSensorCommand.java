// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.IntakeSubsystem;
import frc.robot.Subsystems.StagingSubsystem;
import frc.robot.Robot;

public class IntakingWithSensorCommand extends Command {
  StagingSubsystem m_StagingSubsystem;
  IntakeSubsystem m_IntakeSubsystem;
  boolean NoteLightsControl;
  /** Creates a new IntakingCommand. */
  public IntakingWithSensorCommand(IntakeSubsystem intakeSubsystem, StagingSubsystem stagingSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_StagingSubsystem = stagingSubsystem;
    addRequirements(m_StagingSubsystem);
    m_IntakeSubsystem = intakeSubsystem;
    addRequirements(m_IntakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  // Runs the intake
  //  m_IntakeSubsystem.IntakeMotor1.set(.2);
  //  m_IntakeSubsystem.IntakeMotor2.set(.2);
  //  m_StagingSubsystem.StagingMotor1.set(-.15);
  //  m_StagingSubsystem.StagingMotor2.set(.15);

    if(Robot.MasterStagingSensor.get()){
      m_IntakeSubsystem.IntakeMotor1.set(.2);
      m_IntakeSubsystem.IntakeMotor2.set(.2);
      m_StagingSubsystem.StagingMotor1.set(.15);
      m_StagingSubsystem.StagingMotor2.set(.15);
    }

    else{
      m_IntakeSubsystem.IntakeMotor1.set(0);
      m_IntakeSubsystem.IntakeMotor2.set(0);
      m_StagingSubsystem.StagingMotor1.set(0);
      m_StagingSubsystem.StagingMotor2.set(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_IntakeSubsystem.IntakeMotor1.set(0);
    m_IntakeSubsystem.IntakeMotor2.set(0);
    m_StagingSubsystem.StagingMotor1.set(0);
    m_StagingSubsystem.StagingMotor2.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
