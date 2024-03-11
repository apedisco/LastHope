// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.TestCommands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.Subsystems.IntakeSubsystem;

public class RevAndDeliverAprilTageTestCommand extends Command {
  IntakeSubsystem m_IntakeSubsystem;
  private boolean Amp = false;
  private boolean Speaker = false;
  private boolean Trap = false;
  /** Creates a new RevAndDeliverAprilTageTestCommand. */
  public RevAndDeliverAprilTageTestCommand(IntakeSubsystem intakeSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_IntakeSubsystem = intakeSubsystem;
    addRequirements(m_IntakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
       NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tid = table.getEntry("tid");
    double aid = tid.getDouble(0);
    System.out.println(aid);
    //if the robot sees no april tag the motors will not spin
       m_IntakeSubsystem.Rev(0);
       m_IntakeSubsystem.Deliver(0);
      Robot.motorLightOutput.set(false);
    while(aid > 0){
      //If the limelight sees any of the speaker april tags it revs it to that speed
     if(aid >= 3 || aid <= 4 || aid >= 7 || aid <= 8){
        m_IntakeSubsystem.Rev(.55);
        m_IntakeSubsystem.Deliver(-.5);
        Robot.motorLightOutput.set(true);
        table.getEntry("tid");
        Speaker = true;
      }

       //If the limelight sees any of the amp april tgas it reves it to that speed
     if(aid >= 5 || aid <= 6){
        m_IntakeSubsystem.Rev(.1);
        m_IntakeSubsystem.Deliver(-.3);
        Robot.motorLightOutput.set(true);
        table.getEntry("tid");
        Amp = true;
      }

      if(aid >= 11 || aid <= 16){
        m_IntakeSubsystem.Rev(.45);
        m_IntakeSubsystem.Deliver(-.5);
        Robot.motorLightOutput.set(true);
        table.getEntry("tid");
        Trap = true;
      }
      
      if(Speaker = true){
         m_IntakeSubsystem.Rev(.55);
         m_IntakeSubsystem.Deliver(-.5);
        Robot.motorLightOutput.set(true);
        table.getEntry("tid");
        if(aid >= 5 || aid <= 6 || aid >= 11 || aid <= 16){
          Speaker = false;
        }
      }

      if(Amp = true){
        m_IntakeSubsystem.Rev(.1);
        m_IntakeSubsystem.Deliver(-.3);
        Robot.motorLightOutput.set(true);
        table.getEntry("tid");
         if(aid >= 3 || aid <= 4 || aid >= 7 || aid <= 8 || aid >= 11 || aid <= 16){
          Amp = false;
         }
      }

      if(Trap = true){
        m_IntakeSubsystem.Rev(.45);
         m_IntakeSubsystem.Deliver(-.5);
        Robot.motorLightOutput.set(true);
        table.getEntry("tid");
        if(aid >= 5 || aid <= 6 || aid >= 3 || aid <= 4 || aid >= 7 || aid <= 8){
          Trap = false;
        }
      }

      else{
        break;
      }
    }
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
