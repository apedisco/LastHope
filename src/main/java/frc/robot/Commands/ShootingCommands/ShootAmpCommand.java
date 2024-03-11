// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.ShootingCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.Subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj.Timer;

public class ShootAmpCommand extends Command {
  public Timer m_timer;

  IntakeSubsystem m_IntakeSubsystem;
  private double EngageTime;
  /** Creates a new ShootingCommand. */
  public ShootAmpCommand(IntakeSubsystem intakeSubsystem) {
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
    

    m_IntakeSubsystem.Rev(.1);//Trap.45// Amp speed .1// Speaker Speed? .5
    

    if (System.currentTimeMillis() - EngageTime > 600){
      m_IntakeSubsystem.Deliver(-.3);
      Robot.motorLightOutput.set(true);
     // m_IntakeSubsystem.Intake();
      // Amp -.30 time 200//Speaker -.5 time 600//Trap -.5 time 800
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.motorLightOutput.set(false);
    m_IntakeSubsystem.ShootOff();
   // m_IntakeSubsystem.Outtake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
