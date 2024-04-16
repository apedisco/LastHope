// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.RevAndDeliverCommands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.ShootingSubsystem;
import frc.robot.Subsystems.StagingSubsystem;

public class RevAndDeliverTrapCommand extends Command {
  ShootingSubsystem m_ShootingSubsystem;
  StagingSubsystem m_StagingSubsystem;
  Joystick m_DriveJoystick;
  Joystick m_IntakeJoystick;
  Joystick m_ShootingJoystick;
  // boolean NoteLightsControl;
  /** Creates a new RevAndDeliverTrapCommand. */
  public RevAndDeliverTrapCommand(ShootingSubsystem shootingSubsystem, StagingSubsystem stagingSubsystem, Joystick DriveJoystick, Joystick IntakeJoystick, Joystick shootingJoystick) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_ShootingSubsystem = shootingSubsystem;
    addRequirements(m_ShootingSubsystem);
    m_StagingSubsystem = stagingSubsystem;
    m_DriveJoystick = DriveJoystick;
    m_IntakeJoystick = IntakeJoystick;
    m_ShootingJoystick = shootingJoystick;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_ShootingSubsystem.ShootingMotor1.set((m_IntakeJoystick.getRawAxis(3) + 1) / 2);//-.2 for long speaker // -.29 for amp
    m_ShootingSubsystem.ShootingMotor2.set((m_ShootingJoystick.getRawAxis(3) + 1) / 2);//.8 for long speaker // .35 for amp
    m_StagingSubsystem.StagingMotor1.set(0.3);
    m_StagingSubsystem.StagingMotor2.set(0.3);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_ShootingSubsystem.ShootingMotor1.set(0);
    m_ShootingSubsystem.ShootingMotor2.set(0);
    m_StagingSubsystem.StagingMotor1.set(0);
    m_StagingSubsystem.StagingMotor2.set(0);
    // NoteLightsControl = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
