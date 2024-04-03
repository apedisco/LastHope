// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.AutoCommands;

import edu.wpi.first.wpilibj2.command.Command;
//import frc.robot.Subsystems.IntakeSubsystem;
import frc.robot.Subsystems.ShootingSubsystem;
import frc.robot.Subsystems.StagingSubsystem;

public class AutoShootHighCommand extends Command {
  ShootingSubsystem m_ShootingSubsystem;
  StagingSubsystem m_StagingSubsystem;
  private double EngageTime;
  /** Creates a new AutoShootHighCommand. */
  public AutoShootHighCommand(ShootingSubsystem shootingSubsystem, StagingSubsystem stagingSubsystem) {
    m_ShootingSubsystem = shootingSubsystem;
    addRequirements(m_ShootingSubsystem);
    m_StagingSubsystem = stagingSubsystem;
    addRequirements(m_StagingSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("ShootingItilizing");
    EngageTime = System.currentTimeMillis();
    m_ShootingSubsystem.Rev(0.75);//0.75
    
   while(true){
    m_ShootingSubsystem.Rev(0.75);//0.75
      if(System.currentTimeMillis() - EngageTime > 600 && System.currentTimeMillis() - EngageTime < 900){
        m_ShootingSubsystem.Rev(0.75);//0.75
        m_StagingSubsystem.DeliverHigh();
      }

      else if(System.currentTimeMillis() - EngageTime > 800){
        m_ShootingSubsystem.Rev(0);
        System.out.println("ShootingDoneIntilizing");
        break;
      }
      }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("ShootingDone");
    m_ShootingSubsystem.Rev(0);
    m_StagingSubsystem.DeliverStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
