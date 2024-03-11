// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.IntakeSubsystem;

public class ToggleClimbJoysticksCommand extends Command {
  IntakeSubsystem m_IntakeSubsystem;
  Joystick m_leftJoystick;
  Joystick m_rightJoystick;
  /** Creates a new ToggleClimbJoysticksCommand. */
  public ToggleClimbJoysticksCommand(IntakeSubsystem intakeSubsystem, Joystick leftJoystick, Joystick rightJoystick) {
    m_leftJoystick = leftJoystick;
    m_rightJoystick = rightJoystick;
    m_IntakeSubsystem = intakeSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("joysticks on");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Math.abs(m_rightJoystick.getY()) < .1){
      m_IntakeSubsystem.LeftCLimbStop();
    } else if (m_rightJoystick.getY() > 0){
      m_IntakeSubsystem.LeftClimbDown();
    } else if (m_rightJoystick.getY() < 0){
      m_IntakeSubsystem.LeftClimbUp();
    }
     if (Math.abs(m_leftJoystick.getY()) < .1){
      m_IntakeSubsystem.RightCLimbStop();
    } else if (m_leftJoystick.getY() > 0){
      m_IntakeSubsystem.RightCLimbDown();
    } else if (m_leftJoystick.getY() < 0){
      m_IntakeSubsystem.RightCLimbUp();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_IntakeSubsystem.RightCLimbStop();
    m_IntakeSubsystem.LeftCLimbStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
