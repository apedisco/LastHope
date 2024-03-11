// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.AutoCommands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;


public class IntakeAutoCommand extends Command {

  IntakeSubsystem m_IntakeSubsystem;
  private double EngageTime;
  private boolean cancelCommand;
  private boolean StagingSensor;
  /** Creates a new IntakingCommand. */
  public IntakeAutoCommand(IntakeSubsystem intakeSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_IntakeSubsystem = intakeSubsystem;
    
    addRequirements(m_IntakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    EngageTime = System.currentTimeMillis();
     m_IntakeSubsystem.IntakeIn();
     System.out.println("IntakeIntilize");
    
  //Once the command is called by the Sequential it runs the initalize this is to prevent it from getting stuck in the execute;
    while(true){
      // StagingSensor = m_IntakeSubsystem.StagingSensor.get();
      StagingSensor = Robot.MasterStagingSensor.get();
  //Once the command is initiaized it runs intake till the beam brake sensor reads false;
      m_IntakeSubsystem.IntakeIn();
      if(StagingSensor == false){
  //If the beam break reads a objeck it shuts the intake off and breaks the loop.
        m_IntakeSubsystem.IntakeOff();
        System.out.println("IntakeDoneRunning");
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
    System.out.println("IntakingDone");
    m_IntakeSubsystem.IntakeOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
