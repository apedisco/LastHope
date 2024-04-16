// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.RevCommands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.Subsystems.ShootingSubsystem;

public class RevTrapCommand extends Command {
  ShootingSubsystem m_ShootingSubsystem;
  Joystick m_DriveJoystick;
  Joystick m_IntakeJoystick;
  Joystick m_ShootingJoystick;
  /** Creates a new RevTrapCommand. */
  public RevTrapCommand(ShootingSubsystem shootingSubsystem, Joystick DriveJoystick, Joystick intakeJoystick, Joystick shootingJoystick) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_ShootingSubsystem = shootingSubsystem;
    addRequirements(m_ShootingSubsystem);
    m_DriveJoystick = DriveJoystick;
    m_IntakeJoystick = intakeJoystick;
    m_ShootingJoystick = shootingJoystick;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // m_ShootingSubsystem.ShootingMotor1.set(.4);//-.2 for long speaker // -.29 for amp
    // m_ShootingSubsystem.ShootingMotor2.set(.4);//.8 for long speaker // .35 for amp
    m_ShootingSubsystem.ShootingMotor1.set((m_IntakeJoystick.getRawAxis(3) + 1) / 2);//-.2 for long speaker // -.29 for amp
    m_ShootingSubsystem.ShootingMotor2.set((m_ShootingJoystick.getRawAxis(3) + 1) / 2);//.8 for long speaker // .35 for amp
    // System.out.println((m_DriveJoystick.getRawAxis(3) + 1) / 2);
    //System.out.println((m_IntakeJoystick.getRawAxis(3) + 1) / 2);
    //m_IntakeSubsystem.Rev(1);
    // Robot.motorLightOutput.set(true);
    SmartDashboard.putNumber("RightStick", (m_IntakeJoystick.getRawAxis(3) + 1) / 2);
    SmartDashboard.putNumber("LeftStick", (m_ShootingJoystick.getRawAxis(3) + 1) / 2);
    Robot.spark.set(-0.09);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_ShootingSubsystem.Rev(0);
    // Robot.motorLightOutput.set(false);
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
