// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.CommandSwerveDrivetrain;
import frc.robot.Subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj.Timer;
import com.ctre.phoenix6.Utils;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

public class AutoShoot extends Command {
  IntakeSubsystem m_IntakeSubsystem;

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric();
  private Timer m_time = new Timer();
  public double start_angle;
  
  /** Creates a new AutoTest. */
  public AutoShoot(IntakeSubsystem intakeSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_IntakeSubsystem = intakeSubsystem;

    

    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_time.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      // if( Math.abs(m_CommandSwerveDrivetrain.getRotation3d().getZ() - start_angle) <  Math.PI- .0001){
      //   m_CommandSwerveDrivetrain.rotation(.6, m_CommandSwerveDrivetrain, drive);
      
      // }
      
      // else {
      //   m_CommandSwerveDrivetrain.rotation(0, m_CommandSwerveDrivetrain, drive);
      // }

      if (m_time.get() < 2){
        //m_CommandSwerveDrivetrain.moveLinear(.2, 0, m_CommandSwerveDrivetrain, drive);
        m_IntakeSubsystem.Rev(.4);
      }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    m_IntakeSubsystem.Rev(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
