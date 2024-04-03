// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.RevAndDeliverCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.ShootingSubsystem;
import frc.robot.Subsystems.StagingSubsystem;
import edu.wpi.first.wpilibj.Timer;

public class RevAndDeliverSpeakerCommand extends Command {
  ShootingSubsystem m_ShootingSubsystem;
  StagingSubsystem m_StagingSubsystem;
  public Timer m_timer;
  private double EngageTime;
  // boolean NoteLightsControl;
  /** Creates a new RevAndDeliverSpeakerCommand. */
  public RevAndDeliverSpeakerCommand(ShootingSubsystem shootingSubsystem, StagingSubsystem stagingSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_ShootingSubsystem = shootingSubsystem;
    addRequirements(m_ShootingSubsystem);
    m_StagingSubsystem = stagingSubsystem;
    addRequirements(m_StagingSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    EngageTime = System.currentTimeMillis();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    while(System.currentTimeMillis() > 0){
    if(System.currentTimeMillis() - EngageTime > 0 && System.currentTimeMillis() - EngageTime < 500){  
    m_ShootingSubsystem.ShootingMotor1.set(0.45);//-.45
    m_ShootingSubsystem.ShootingMotor2.set(0.75);//.75
    m_StagingSubsystem.StagingMotor1.set(0.7);
    m_StagingSubsystem.StagingMotor2.set(0.7);
    }

    if(System.currentTimeMillis() - EngageTime > 500){
      break;
    }
   }
 }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_ShootingSubsystem.ShootingMotor1.set(0);
    m_ShootingSubsystem.ShootingMotor2.set(0);
    m_StagingSubsystem.StagingMotor1.set(0);
    m_StagingSubsystem.StagingMotor2.set(0);
    // NoteLightsControl = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
