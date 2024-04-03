// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.RevCommands;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.Subsystems.ShootingSubsystem;

public class RevAmpCommand extends Command {
  ShootingSubsystem m_ShootingSubsystem;
  /** Creates a new RevAmpCommand. */
  public RevAmpCommand(ShootingSubsystem shootingSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_ShootingSubsystem = shootingSubsystem;
    addRequirements(m_ShootingSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_ShootingSubsystem.ShootingMotor1.setIdleMode(IdleMode.kBrake);
   // m_ShootingSubsystem.ShootingMotor1.setInverted(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_ShootingSubsystem.pidShooting1(1650);
    m_ShootingSubsystem.pidShooting2(600);
   if(m_ShootingSubsystem.shooterEncoder1.getVelocity() > 1200){
    Robot.motorLightOutput.set(true);
   }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_ShootingSubsystem.Rev(0);
    Robot.motorLightOutput.set(false);
   m_ShootingSubsystem.ShootingMotor1.setIdleMode(IdleMode.kCoast);
 //  m_ShootingSubsystem.ShootingMotor1.setInverted(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
