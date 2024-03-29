// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.AutoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.IntakeSubsystem;

public class AutoShootHighCommand extends Command {
  IntakeSubsystem m_IntakeSubsystem;
  private double EngageTime;
  /** Creates a new AutoShootHighCommand. */
  public AutoShootHighCommand(IntakeSubsystem intakeSubsystem) {
    m_IntakeSubsystem = intakeSubsystem;
    addRequirements(m_IntakeSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("ShootingItilizing");
    EngageTime = System.currentTimeMillis();
    m_IntakeSubsystem.Rev(.55);
    
   while(true){
    m_IntakeSubsystem.Rev(.8);
      if(System.currentTimeMillis() - EngageTime > 600 && System.currentTimeMillis() - EngageTime < 900){
        m_IntakeSubsystem.Rev(.8);
        m_IntakeSubsystem.Deliver(-.5);
      }

      else if(System.currentTimeMillis() - EngageTime > 800){
        m_IntakeSubsystem.ShootOff();
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
    m_IntakeSubsystem.ShootOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
