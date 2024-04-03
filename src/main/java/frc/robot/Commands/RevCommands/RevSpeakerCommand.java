// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.RevCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.Subsystems.ShootingSubsystem;

public class RevSpeakerCommand extends Command {
  public static final String InterruptBehavior = null;
  ShootingSubsystem m_ShootingSubsystem;
  /** Creates a new RevCommand. */
  public RevSpeakerCommand(ShootingSubsystem shootingSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_ShootingSubsystem = shootingSubsystem;
    addRequirements(m_ShootingSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_ShootingSubsystem.ShootingMotor1.set(0.45);//-.45
    m_ShootingSubsystem.ShootingMotor2.set(0.75);//.75
    //m_IntakeSubsystem.ShootOn();
   // m_IntakeSubsystem.Rev(-.55);
   if(m_ShootingSubsystem.shooterEncoder2.getVelocity() > 4500){
     Robot.motorLightOutput.set(true);
   }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_ShootingSubsystem.Rev(0);
    Robot.motorLightOutput.set(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
