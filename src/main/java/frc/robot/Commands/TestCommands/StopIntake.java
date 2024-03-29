// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.TestCommands;

import frc.robot.Robot;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.Subsystems.IntakeSubsystem;

public class StopIntake extends Command {
   IntakeSubsystem m_IntakeSubsystem;
   
  /** Creates a new StopIntake. */
  public StopIntake(IntakeSubsystem intakeSubsystem) {
    m_IntakeSubsystem = intakeSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_IntakeSubsystem.IntakeOff();
    //StagingSensor = Robot.MasterStagingSensor;
    if(Robot.MasterStagingSensor.get()){
       m_IntakeSubsystem.IntakeIn();
    }
    else{
      m_IntakeSubsystem.IntakeOff();
    }

    // if(m_IntakeSubsystem.StagingSensor.get()){
    //    m_IntakeSubsystem.IntakeIn();
    // }
    // else{
    //   m_IntakeSubsystem.IntakeOff();
    // }
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
