// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.RevAndDeliverCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.IntakeSubsystem;
import frc.robot.Subsystems.ShootingSubsystem;
import frc.robot.Subsystems.StagingSubsystem;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.Timer;

public class RevAndDeliverAmpCommand extends Command {
  IntakeSubsystem m_IntakeSubsystem;
  ShootingSubsystem m_ShootingSubsystem;
  StagingSubsystem m_StagingSubsystem;
  public Timer m_timer;
  private double EngageTime;
  // boolean NoteLightsControl;
  /** Creates a new RevAndDeliverAmpCommand. */
  public RevAndDeliverAmpCommand(IntakeSubsystem intakeSubsystem, ShootingSubsystem shootingSubsystem, StagingSubsystem stagingSubsystem) {
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
  public void initialize() {
    EngageTime = System.currentTimeMillis();
    m_ShootingSubsystem.ShootingMotor2.setIdleMode(IdleMode.kBrake);
    //m_ShootingSubsystem.ShootingMotor1.setInverted(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    while(System.currentTimeMillis() > 0){
    // if(System.currentTimeMillis() - EngageTime < 200){
    // m_IntakeSubsystem.ShootingMotor1.set(-.25);
    // }

    if(System.currentTimeMillis() - EngageTime > 0 && System.currentTimeMillis() - EngageTime < 1100){
    //  m_ShootingSubsystem.pidShooting1(1650);
    //  m_ShootingSubsystem.pidShooting2(700);
      
      //Shooter -8%
      m_ShootingSubsystem.pidShooting1(2000); //1700
      m_ShootingSubsystem.pidShooting2(450); //400

     //m_StagingSubsystem.DeliverLow();

      m_IntakeSubsystem.IntakeMotor1.set(0.2);
      m_IntakeSubsystem.IntakeMotor2.set(0.2);
      m_StagingSubsystem.StagingMotor1.set(0.2);
      m_StagingSubsystem.StagingMotor2.set(0.2);
    }

    if(System.currentTimeMillis() - EngageTime > 1100){
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
    m_IntakeSubsystem.IntakeMotor1.set(0);
    m_IntakeSubsystem.IntakeMotor2.set(0);
    m_StagingSubsystem.StagingMotor1.set(0);
    m_StagingSubsystem.StagingMotor2.set(0);
    m_ShootingSubsystem.ShootingMotor2.setIdleMode(IdleMode.kCoast);
   // m_ShootingSubsystem.ShootingMotor1.setInverted(false);
    // NoteLightsControl = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

