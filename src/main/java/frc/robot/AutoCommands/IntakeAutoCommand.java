// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.AutoCommands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj.Timer;


public class IntakeAutoCommand extends Command {

  IntakeSubsystem m_IntakeSubsystem;
  private double EngageTime;
  private boolean cancelCommand;
  /** Creates a new IntakingCommand. */
  public IntakeAutoCommand(IntakeSubsystem intakeSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_IntakeSubsystem = intakeSubsystem;
    addRequirements(m_IntakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
     m_IntakeSubsystem.IntakeIn();
    

    while(m_IntakeSubsystem.StagingSensor.get()){
      m_IntakeSubsystem.IntakeIn();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // m_IntakeSubsystem.IntakeIn();
    // System.out.println(m_IntakeSubsystem.StagingSensor.get());
    // if(m_IntakeSubsystem.StagingSensor.get()){
    //    m_IntakeSubsystem.IntakeIn();
    // }
    // else{
    //   m_IntakeSubsystem.IntakeOff();
    //   cancelCommand = true;
    //   // this.end(true);
    // }
   
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
   
    m_IntakeSubsystem.IntakeOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
